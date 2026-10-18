//! Controller block framework (EQ-CTRL-04 loop semantics):
//!
//! - every loop is a discrete block at its own rate with an output clamp and a ramp
//!   limiter on its reference;
//! - a disabled loop passes the setpoint of the quantity it commands straight to the
//!   inner loop (position off → `ctrl.omega_ref` drives the velocity loop);
//! - a controller family is a set of such blocks wired on the bus through command
//!   signals, built from `ControllerParams` by `build_family`.
//!
//! The loop law (P, PI, PID; EQ-CTRL-01/02) is a `LoopLaw` with no bus coupling.

use sim_model::params::ControllerParams;

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId};
use crate::engine::time::{SimTime, period_from_hz};

/// Rate limiter on a reference: moves towards the target at most `rate`·dt per step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RampLimiter {
    /// Maximum rate [unit/s]; infinite = no limit.
    pub rate: f64,
    y: Option<f64>,
}

impl RampLimiter {
    pub fn new(rate: f64) -> Self {
        Self { rate, y: None }
    }

    /// The limited value; the first call (or after `restart`) starts at the target.
    pub fn step(&mut self, target: f64, dt: f64) -> f64 {
        let y = match self.y {
            Some(y) if self.rate.is_finite() => {
                let d = self.rate * dt;
                y + (target - y).clamp(-d, d)
            }
            _ => target,
        };
        self.y = Some(y);
        y
    }

    /// Continue from `y` (bumpless start), or from the next target with `None`.
    pub fn restart(&mut self, y: Option<f64>) {
        self.y = y;
    }

    pub fn value(&self) -> Option<f64> {
        self.y
    }
}

/// One loop's settings: enable, sample period, output clamp, reference ramp.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopConfig {
    pub enabled: bool,
    pub period: SimTime,
    /// Output limits (lo, hi).
    pub limits: (f64, f64),
    /// Reference ramp [reference unit/s]; infinite = none.
    pub ramp: f64,
}

impl LoopConfig {
    /// From a rate in Hz (rounded to whole ns, EQ-NUM).
    pub fn from_hz(
        enabled: bool,
        rate_hz: f64,
        limits: (f64, f64),
        ramp: f64,
    ) -> Result<Self, String> {
        let period = period_from_hz(rate_hz).map_err(|e| e.to_string())?.period;
        // Also refuses NaN limits.
        if limits.0.partial_cmp(&limits.1).is_none_or(|o| o.is_gt()) {
            return Err(format!(
                "loop limits ({}, {}) are not ordered",
                limits.0, limits.1
            ));
        }
        if ramp.is_nan() || ramp <= 0.0 {
            return Err(format!("ramp rate must be > 0, got {ramp}"));
        }
        Ok(Self {
            enabled,
            period,
            limits,
            ramp,
        })
    }
}

/// A loop law: reference r and feedback y → command u, at period T [s].
pub trait LoopLaw: Send {
    fn update(&mut self, r: f64, y: f64, t: f64) -> f64;
    /// One update clamped to this tick's limits; laws with anti-windup override it so the
    /// integrator respects the limits that actually apply.
    fn update_limited(&mut self, r: f64, y: f64, t: f64, lo: f64, hi: f64) -> f64 {
        self.update(r, y, t).clamp(lo, hi)
    }
    fn reset(&mut self);
    /// Follow an externally imposed output u while the loop is disabled, so that
    /// re-enabling it is bumpless (an integrator starts from u).
    fn track(&mut self, _u: f64, _r: f64, _y: f64) {}
    fn save(&self) -> serde_json::Value {
        serde_json::Value::Null
    }
    fn restore(&mut self, _v: &serde_json::Value) -> Result<(), String> {
        Ok(())
    }
}

/// u = K_p (r − y): the position P loop of EQ-CTRL-04.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Proportional {
    pub kp: f64,
}

impl LoopLaw for Proportional {
    fn update(&mut self, r: f64, y: f64, _t: f64) -> f64 {
        self.kp * (r - y)
    }
    fn reset(&mut self) {}
}

/// A bus signal times a constant (unit or gear-ratio conversion).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scaled {
    pub id: SignalId,
    pub gain: f64,
}

impl Scaled {
    pub fn of(bus: &SignalBus, path: &str, gain: f64) -> Result<Self, SignalError> {
        Ok(Self {
            id: bus.id(path)?,
            gain,
        })
    }
    fn read(&self, bus: &SignalBus) -> f64 {
        self.gain * bus.get(self.id)
    }
}

/// Where a loop reads and writes. All values are in the loop's own units after the
/// gains: `reference` and `feedback` in the controlled quantity, `bypass` and `out` in
/// the commanded one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopWiring {
    pub reference: Scaled,
    pub feedback: Scaled,
    /// Setpoint of the commanded quantity, passed through while the loop is disabled.
    pub bypass: Scaled,
    pub out: SignalId,
}

/// One cascade loop as a discrete block. Its id (e.g. `controller.position`) is also
/// its parameter root (D-020): `<id>.enabled` (0/1) switches it live.
pub struct LoopBlock {
    id: String,
    cfg: LoopConfig,
    io: LoopWiring,
    law: Box<dyn LoopLaw>,
    ramp: RampLimiter,
    enabled: bool,
    /// |reference| limit (e.g. the speed limit on a velocity reference); ∞ = none.
    ref_limit: f64,
    /// Output limits published by another block each tick (lo, hi), intersected with the
    /// static ones (e.g. the six-step current cap).
    dyn_limits: Option<(SignalId, SignalId)>,
}

impl LoopBlock {
    /// Also clamp the output to the limits in signals `lo`, `hi` (read each tick).
    pub fn with_dynamic_limits(mut self, lo: SignalId, hi: SignalId) -> Self {
        self.dyn_limits = Some((lo, hi));
        self
    }

    /// Clamp the reference to ±`lim` before the ramp.
    pub fn with_reference_limit(mut self, lim: f64) -> Self {
        self.ref_limit = lim;
        self
    }

    pub fn new(id: &str, cfg: LoopConfig, io: LoopWiring, law: Box<dyn LoopLaw>) -> Self {
        Self {
            id: id.into(),
            cfg,
            io,
            law,
            ramp: RampLimiter::new(cfg.ramp),
            enabled: cfg.enabled,
            ref_limit: f64::INFINITY,
            dyn_limits: None,
        }
    }
}

impl DiscreteBlock for LoopBlock {
    fn id(&self) -> &str {
        &self.id
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.cfg.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let t = self.cfg.period.as_secs_f64();
        let (mut lo, mut hi) = self.cfg.limits;
        if let Some((l, h)) = self.dyn_limits {
            (lo, hi) = (lo.max(ctx.bus.get(l)), hi.min(ctx.bus.get(h)));
            if lo > hi {
                (lo, hi) = (0.0, 0.0);
            }
        }
        let (r_in, y) = (
            self.io
                .reference
                .read(ctx.bus)
                .clamp(-self.ref_limit, self.ref_limit),
            self.io.feedback.read(ctx.bus),
        );
        let u = if self.enabled {
            let r = self.ramp.step(r_in, t);
            self.law.update_limited(r, y, t, lo, hi)
        } else {
            // Bypass: the inner loop gets the setpoint of its own quantity. The law
            // follows it, and the ramp restarts from the measurement on re-enable.
            let u = self.io.bypass.read(ctx.bus).clamp(lo, hi);
            self.law.track(u, r_in, y);
            self.ramp.restart(Some(y));
            u
        };
        ctx.bus.set(self.io.out, u);
        Ok(())
    }
    fn reset(&mut self) {
        self.law.reset();
        self.ramp.restart(None);
        self.enabled = self.cfg.enabled;
    }
    fn params(&self) -> Vec<(String, String)> {
        vec![("enabled".into(), "-".into())]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        (name == "enabled").then_some(f64::from(u8::from(self.enabled)))
    }
    fn set_param(&mut self, name: &str, value: f64) -> Result<f64, String> {
        if name != "enabled" {
            return Err(format!("unknown parameter `{name}`"));
        }
        if !matches!(value, 0.0 | 1.0) {
            return Err(format!("enabled must be 0 or 1, got {value}"));
        }
        let old = f64::from(u8::from(self.enabled));
        self.enabled = value == 1.0;
        Ok(old)
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "enabled": self.enabled,
            "ramp": self.ramp.value(),
            "law": self.law.save(),
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        self.enabled = v
            .get("enabled")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `enabled`")?;
        let ramp: Option<f64> = serde_json::from_value(v.get("ramp").cloned().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        self.ramp.restart(ramp);
        self.law
            .restore(v.get("law").unwrap_or(&serde_json::Value::Null))
    }
}

/// What a family needs to know about the plant to design and wire its loops.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlantInfo {
    /// Gear ratio N (motor = N · load).
    pub ratio: f64,
    /// Torque constant K_t = 1.5·p·λ [N·m/A].
    pub kt: f64,
    /// Total inertia reflected to the motor shaft [kg·m²].
    pub j_motor: f64,
}

/// The blocks of one controller family, wired on the bus.
pub struct Family {
    pub blocks: Vec<Box<dyn DiscreteBlock>>,
}

/// The controller factory: a family from its parameters. Families plug in as their
/// tasks land; until then a family is refused with the task that brings it.
pub fn build_family(
    p: &ControllerParams,
    _plant: &PlantInfo,
    _bus: &mut SignalBus,
) -> Result<Family, String> {
    let (family, task) = match p {
        ControllerParams::Foc { .. } => ("foc", "P09.T03/T04"),
        ControllerParams::SixStep { .. } => ("six_step", "P09.T05"),
        ControllerParams::OpenLoop { .. } => ("open_loop", "P09.T06"),
        ControllerParams::Mit { .. } => ("mit", "P09.T07"),
        ControllerParams::Custom { .. } => ("custom", "P09.T10"),
    };
    Err(format!("controller family `{family}` arrives in {task}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::commands::{ChangeSource, EngineCommand};
    use crate::engine::engine::Engine;
    use crate::engine::plant::Plant;
    use crate::engine::signals::SignalKind;

    #[test]
    fn ramp_limits_the_rate_and_starts_at_the_first_target() {
        let mut r = RampLimiter::new(10.0);
        assert_eq!(r.step(5.0, 0.1), 5.0);
        assert_eq!(r.step(0.0, 0.1), 4.0);
        assert_eq!(r.step(4.5, 0.1), 4.5);
        r.restart(Some(-1.0));
        assert_eq!(r.step(10.0, 0.25), 1.5);
        let mut free = RampLimiter::new(f64::INFINITY);
        free.step(0.0, 1.0);
        assert_eq!(free.step(1e9, 1e-6), 1e9);
    }

    #[test]
    fn loop_config_checks_rate_limits_and_ramp() {
        let c = LoopConfig::from_hz(true, 1e3, (-1.0, 1.0), f64::INFINITY).unwrap();
        assert_eq!(c.period, SimTime::from_nanos(1_000_000));
        assert!(LoopConfig::from_hz(true, 0.0, (-1.0, 1.0), 1.0).is_err());
        assert!(LoopConfig::from_hz(true, 1e3, (1.0, -1.0), 1.0).is_err());
        assert!(LoopConfig::from_hz(true, 1e3, (-1.0, 1.0), 0.0).is_err());
    }

    /// A position P loop (1 kHz) over a velocity P loop (5 kHz) on a frozen plant
    /// (angle and speed set directly). N = 4: the position loop works on the load angle,
    /// commands motor speed; the bypass passes N·ω_ref.
    fn cascade(position_enabled: bool) -> Engine {
        let mut bus = SignalBus::new();
        for (p, u) in [("motor.theta", "rad"), ("motor.omega", "rad/s")] {
            bus.register(p, u, "", SignalKind::Input).unwrap();
        }
        let sp = crate::control::setpoint::Setpoints::get_or_register(&mut bus).unwrap();
        let w_cmd = bus
            .register("ctrl.omega_cmd", "rad/s", "", SignalKind::Output)
            .unwrap();
        let iq = bus
            .register("ctrl.foc.iq_ref", "A", "", SignalKind::Output)
            .unwrap();
        let n = 4.0;
        let pos = LoopBlock::new(
            "controller.position",
            LoopConfig::from_hz(position_enabled, 1e3, (-500.0, 500.0), f64::INFINITY).unwrap(),
            LoopWiring {
                reference: Scaled {
                    id: sp.theta,
                    gain: 1.0,
                },
                feedback: Scaled::of(&bus, "motor.theta", 1.0 / n).unwrap(),
                bypass: Scaled {
                    id: sp.omega,
                    gain: n,
                },
                out: w_cmd,
            },
            // K_p = ω_p, and ×N to the motor side.
            Box::new(Proportional { kp: 20.0 * n }),
        );
        let vel = LoopBlock::new(
            "controller.velocity",
            LoopConfig::from_hz(true, 5e3, (-10.0, 10.0), f64::INFINITY).unwrap(),
            LoopWiring {
                reference: Scaled {
                    id: w_cmd,
                    gain: 1.0,
                },
                feedback: Scaled::of(&bus, "motor.omega", 1.0).unwrap(),
                bypass: Scaled {
                    id: sp.torque,
                    gain: 1.0,
                },
                out: iq,
            },
            Box::new(Proportional { kp: 0.05 }),
        );
        let mut e = Engine::new(Plant::new(), bus, vec![Box::new(pos), Box::new(vel)], 1e-4);
        let mut set = |p: &str, v: f64| {
            let id = e.bus.id(p).unwrap();
            e.bus.set(id, v);
        };
        set("ctrl.theta_ref", 1.0);
        set("ctrl.omega_ref", 30.0);
        e
    }

    fn sig(e: &Engine, n: &str) -> f64 {
        e.bus.get(e.bus.id(n).unwrap())
    }

    /// EQ-CTRL-04 loop semantics (P09.T01 verify): with the position loop disabled the
    /// velocity setpoint drives the velocity loop directly; enabled, the position error
    /// does.
    #[test]
    fn disabling_the_position_loop_passes_the_velocity_setpoint_through() {
        let mut e = cascade(true);
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        // θ_L = 0, θ* = 1 rad: ω_cmd = 80 · 1 (motor side); i_q = 0.05 · 80 = 4 A.
        assert_eq!(sig(&e, "ctrl.omega_cmd"), 80.0);
        assert_eq!(sig(&e, "ctrl.foc.iq_ref"), 4.0);
        // Live: controller.position.enabled = 0 → ω_cmd = N·ω_ref = 120 (load 30 rad/s).
        e.queue(EngineCommand::SetParam {
            path: "controller.position.enabled".into(),
            value: 0.0,
            source: ChangeSource::Internal,
        });
        e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        assert_eq!(e.get_param("controller.position.enabled"), Some(0.0));
        assert_eq!(sig(&e, "ctrl.omega_cmd"), 120.0);
        // The velocity loop now follows the setpoint: 0.05 · 120 = 6 A (below the 10 A clamp).
        assert_eq!(sig(&e, "ctrl.foc.iq_ref"), 6.0);
        let mut f = cascade(false);
        f.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        assert_eq!(sig(&f, "ctrl.omega_cmd"), 120.0, "built disabled");
        // A disabled velocity loop passes the torque setpoint (here 0).
        f.queue(EngineCommand::SetParam {
            path: "controller.velocity.enabled".into(),
            value: 0.0,
            source: ChangeSource::Internal,
        });
        f.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        assert_eq!(sig(&f, "ctrl.foc.iq_ref"), 0.0);
    }

    /// Re-enabling restarts the reference ramp from the measurement (no step in the
    /// command); save/restore reproduces the outputs; bad values are refused.
    #[test]
    fn re_enabling_ramps_from_the_measurement_and_restores() {
        let mut bus = SignalBus::new();
        let mut reg = |p: &str| bus.register(p, "-", "", SignalKind::Input).unwrap();
        let (r, y, byp, out) = (reg("x.r"), reg("x.y"), reg("x.b"), reg("x.u"));
        let s = |id| Scaled { id, gain: 1.0 };
        let mk = || {
            LoopBlock::new(
                "controller.position",
                // 1 kHz, ramp 100 units/s = 0.1 per step.
                LoopConfig::from_hz(false, 1e3, (-50.0, 50.0), 100.0).unwrap(),
                LoopWiring {
                    reference: s(r),
                    feedback: s(y),
                    bypass: s(byp),
                    out,
                },
                Box::new(Proportional { kp: 1.0 }),
            )
        };
        let mut blk = mk();
        bus.set(r, 10.0);
        bus.set(y, 2.0);
        bus.set(byp, 7.0);
        let mut k = 0;
        let mut step = |blk: &mut LoopBlock, bus: &mut SignalBus| {
            k += 1;
            let mut ctx = StepCtx {
                t: SimTime::from_nanos(k * 1_000_000),
                bus,
            };
            blk.step(&mut ctx).unwrap();
            ctx.bus.get(out)
        };
        assert_eq!(step(&mut blk, &mut bus), 7.0, "disabled: bypass");
        blk.set_param("enabled", 1.0).unwrap();
        // The ramp starts at y = 2: r = 2.1, 2.2, … and u = r − y.
        assert!((step(&mut blk, &mut bus) - 0.1).abs() < 1e-12);
        assert!((step(&mut blk, &mut bus) - 0.2).abs() < 1e-12);
        let saved = blk.save();
        let mut other = mk();
        other.restore(&saved).unwrap();
        let (a, b) = (step(&mut blk, &mut bus), step(&mut other, &mut bus));
        assert!((a - 0.3).abs() < 1e-12 && a == b, "{a} vs {b}");
        assert!(blk.set_param("enabled", 0.5).is_err());
        assert!(blk.set_param("gain", 1.0).is_err());
    }

    #[test]
    fn families_without_an_implementation_are_refused_with_their_task() {
        let p: ControllerParams = sim_model::io::parse_yaml(
            "family: open_loop\nrate: 10 kHz\nv0: 1 V\nk_vf: 0.01\naccel: 100\nlimits: { current: 5 A }\n",
            "c",
        )
        .unwrap();
        let info = PlantInfo {
            ratio: 1.0,
            kt: 0.1,
            j_motor: 1e-4,
        };
        let err = build_family(&p, &info, &mut SignalBus::new())
            .err()
            .unwrap();
        assert!(err.contains("P09.T06"), "{err}");
    }
}
