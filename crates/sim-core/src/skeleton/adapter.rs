//! Skeleton motor + FOC on the production engine (P03.T12). The dq plant stays a
//! temporary model (replaced by the stationary-frame motor in P05; kept as a
//! test fixture), but it already runs on the engine with energy accounting.

use super::foc::{Foc, FocConfig, FocGains};
use super::model::PmsmParams;
use crate::energy::PowerKind;
use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::engine::Engine;
use crate::engine::plant::{Plant, PlantModule};
use crate::engine::signals::{SignalBus, SignalId, SignalKind};
use crate::engine::time::{SimTime, period_from_hz};

#[derive(Debug, Clone, Copy)]
struct MotorSignals {
    v_d: SignalId,
    v_q: SignalId,
    i_d: SignalId,
    i_q: SignalId,
    omega: SignalId,
    theta: SignalId,
    i_a: SignalId,
    i_b: SignalId,
    i_c: SignalId,
}

/// dq PMSM plant module: states (i_d, i_q, ω, θ); reads ctrl.v_d/v_q from the bus.
struct DqMotor {
    /// `p.j` is the total shaft inertia: rotor + `extra_j`.
    p: PmsmParams,
    /// Reflected gearbox/load inertia [kg·m²] (fixed for the run).
    extra_j: f64,
    locked: bool,
    s: MotorSignals,
}

impl PlantModule for DqMotor {
    fn save(&self) -> serde_json::Value {
        // Live parameters belong in snapshots (Snapshot doc).
        let p = &self.p;
        serde_json::json!({ "locked": self.locked, "r": p.r, "ld": p.ld, "lq": p.lq, "lambda": p.lambda, "j": p.j, "b": p.b, "extra_j": self.extra_j })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        let p = PmsmParams {
            p: self.p.p,
            r: f("r")?,
            ld: f("ld")?,
            lq: f("lq")?,
            lambda: f("lambda")?,
            j: f("j")?,
            b: f("b")?,
        };
        self.locked = v
            .get("locked")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `locked`")?;
        self.p = p;
        self.extra_j = f("extra_j")?;
        Ok(())
    }
    fn name(&self) -> &str {
        "motor"
    }
    fn n_states(&self) -> usize {
        4
    }
    fn state_names(&self) -> Vec<(String, String)> {
        [
            ("i_d", "A"),
            ("i_q", "A"),
            ("omega", "rad/s"),
            ("theta", "rad"),
        ]
        .iter()
        .map(|(a, b)| (a.to_string(), b.to_string()))
        .collect()
    }
    fn init(&self, x: &mut [f64]) {
        x.fill(0.0);
    }
    fn outputs(&mut self, _: f64, x: &[f64], o: usize, bus: &mut SignalBus) {
        let (id, iq, w, th) = (x[o], x[o + 1], x[o + 2], x[o + 3]);
        bus.set(self.s.i_d, id);
        bus.set(self.s.i_q, iq);
        bus.set(self.s.omega, w);
        bus.set(self.s.theta, th);
        // Inverse Park + inverse Clarke (EQ-CONV-02/04).
        let (sn, cs) = (self.p.p * th).sin_cos();
        let (ia, ib) = (id * cs - iq * sn, id * sn + iq * cs);
        let k = 3f64.sqrt() / 2.0;
        bus.set(self.s.i_a, ia);
        bus.set(self.s.i_b, -0.5 * ia + k * ib);
        bus.set(self.s.i_c, -0.5 * ia - k * ib);
    }
    fn derivatives(&self, _: f64, x: &[f64], o: usize, bus: &SignalBus, dx: &mut [f64]) {
        let p = &self.p;
        let (id, iq, w) = (x[o], x[o + 1], x[o + 2]);
        let (vd, vq) = (bus.get(self.s.v_d), bus.get(self.s.v_q));
        let we = p.p * w;
        dx[0] = (vd - p.r * id + we * p.lq * iq) / p.ld;
        dx[1] = (vq - p.r * iq - we * p.ld * id - we * p.lambda) / p.lq;
        let torque = 1.5 * p.p * (p.lambda * iq + (p.ld - p.lq) * id * iq);
        if self.locked {
            dx[2] = 0.0;
            dx[3] = 0.0;
        } else {
            dx[2] = (torque - p.b * w) / p.j;
            dx[3] = w;
        }
    }
    // Canonical ids (CONVENTIONS §3): `motor.<name>` matches the sim-model path.
    fn params(&self) -> Vec<(String, String)> {
        [
            ("locked", "-"),
            ("electrical.r_phase", "ohm"),
            ("electrical.l_d", "H"),
            ("electrical.l_q", "H"),
            ("electrical.lambda_m", "Wb"),
            ("mechanical.j_rotor", "kg*m^2"),
            ("mechanical.friction.viscous", "N*m*s/rad"),
        ]
        .iter()
        .map(|(n, u)| ((*n).into(), (*u).into()))
        .collect()
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        let p = &self.p;
        Some(match name {
            "locked" => f64::from(u8::from(self.locked)),
            "electrical.r_phase" => p.r,
            "electrical.l_d" => p.ld,
            "electrical.l_q" => p.lq,
            "electrical.lambda_m" => p.lambda,
            "mechanical.j_rotor" => p.j - self.extra_j,
            "mechanical.friction.viscous" => p.b,
            _ => return None,
        })
    }
    fn set_param(&mut self, name: &str, v: f64, _: &mut [f64]) -> Result<f64, String> {
        if name == "locked" {
            let old = f64::from(u8::from(self.locked));
            self.locked = v != 0.0;
            return Ok(old);
        }
        // Validated by sim-model's constraint rules before they get here (P04.T13);
        // keep a last guard so a bad command cannot poison the state.
        let viscous = name == "mechanical.friction.viscous";
        if !v.is_finite() || v < 0.0 || (v == 0.0 && !viscous) {
            return Err(format!("`{name}` must be finite and positive, got {v}"));
        }
        let extra = self.extra_j;
        let p = &mut self.p;
        let (slot, offset) = match name {
            "electrical.r_phase" => (&mut p.r, 0.0),
            "electrical.l_d" => (&mut p.ld, 0.0),
            "electrical.l_q" => (&mut p.lq, 0.0),
            "electrical.lambda_m" => (&mut p.lambda, 0.0),
            "mechanical.j_rotor" => (&mut p.j, extra),
            "mechanical.friction.viscous" => (&mut p.b, 0.0),
            _ => return Err(format!("unknown parameter `{name}`")),
        };
        Ok(std::mem::replace(slot, v + offset) - offset)
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![
            ("electrical_in".into(), PowerKind::Input),
            ("copper".into(), PowerKind::Loss),
            ("viscous".into(), PowerKind::Loss),
            // Work done *on* the motor by the lock constraint (locked rotor only).
            ("lock_reaction".into(), PowerKind::External),
        ]
    }
    fn powers(&self, _: f64, x: &[f64], o: usize, bus: &SignalBus, pw: &mut [f64]) {
        let p = &self.p;
        let (id, iq, w) = (x[o], x[o + 1], x[o + 2]);
        // Amplitude-invariant dq power has the 3/2 factor (EQ-CONV-07).
        pw[0] = 1.5 * (bus.get(self.s.v_d) * id + bus.get(self.s.v_q) * iq);
        pw[1] = 1.5 * p.r * (id * id + iq * iq);
        pw[2] = if self.locked { 0.0 } else { p.b * w * w };
        let torque = 1.5 * p.p * (p.lambda * iq + (p.ld - p.lq) * id * iq);
        pw[3] = if self.locked { -torque * w } else { 0.0 };
    }
    fn stored_energy(&self, x: &[f64], o: usize, _: &SignalBus) -> f64 {
        let p = &self.p;
        let (id, iq, w) = (x[o], x[o + 1], x[o + 2]);
        0.5 * p.j * w * w + 1.5 * 0.5 * (p.ld * id * id + p.lq * iq * iq)
    }
}

/// FOC at 20 kHz on the bus. With `open_loop_vq` it outputs v_d = 0, v_q = const.
struct FocBlock {
    foc: Foc,
    period: SimTime,
    open_loop_vq: Option<f64>,
    s: MotorSignals,
    omega_ref: SignalId,
    iq_ref: SignalId,
    /// Gear ratio N: `ctrl.omega_ref` is load-side, the speed loop runs on the motor side.
    ratio: f64,
}

impl DiscreteBlock for FocBlock {
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "foc": self.foc.save_state(), "open_loop_vq": self.open_loop_vq })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let st: [f64; 5] = serde_json::from_value(v["foc"].clone()).map_err(|e| e.to_string())?;
        self.foc.restore_state(st);
        self.open_loop_vq =
            serde_json::from_value(v["open_loop_vq"].clone()).map_err(|e| e.to_string())?;
        Ok(())
    }
    fn id(&self) -> &str {
        "ctrl"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let (vd, vq) = match self.open_loop_vq {
            Some(v) => (0.0, v),
            None => {
                self.foc.omega_ref = self.ratio * ctx.bus.get(self.omega_ref);
                self.foc.update(
                    ctx.bus.get(self.s.i_d),
                    ctx.bus.get(self.s.i_q),
                    ctx.bus.get(self.s.omega),
                )
            }
        };
        ctx.bus.set(self.s.v_d, vd);
        ctx.bus.set(self.s.v_q, vq);
        ctx.bus.set(self.iq_ref, self.foc.iq_ref);
        Ok(())
    }
    fn reset(&mut self) {
        self.foc.reset();
    }
}

/// Options for the skeleton engine.
#[derive(Debug, Clone, Copy)]
pub struct SkeletonOptions {
    pub locked: bool,
    pub open_loop_vq: Option<f64>,
    /// Integration step limit [s].
    pub dt_max: f64,
    /// Reflected gearbox/load inertia added to `PmsmParams::j` by the caller [kg·m²].
    pub extra_j: f64,
    /// Gear ratio N (load-side setpoints × N = motor side).
    pub ratio: f64,
}

impl Default for SkeletonOptions {
    fn default() -> Self {
        Self {
            locked: false,
            open_loop_vq: None,
            dt_max: 5e-6,
            extra_j: 0.0,
            ratio: 1.0,
        }
    }
}

/// Build the skeleton engine (6020-class motor + 20 kHz FOC).
pub fn build_engine(opts: SkeletonOptions) -> Engine {
    build_engine_with(PmsmParams::skeleton_6020(), FocConfig::skeleton(), opts)
}

/// Skeleton engine with given motor parameters and controller configuration.
pub fn build_engine_with(mp: PmsmParams, cfg: FocConfig, opts: SkeletonOptions) -> Engine {
    let mut bus = SignalBus::new();
    // INVARIANT: the skeleton registers a fixed, unique set of paths into a fresh bus,
    // so `register` cannot fail here.
    let mut reg = |p: &str, u: &str, d: &str, k: SignalKind| {
        bus.register(p, u, d, k).expect("unique skeleton signal")
    };
    let s = MotorSignals {
        v_d: reg(
            "ctrl.foc.v_d",
            "V",
            "d-axis voltage command",
            SignalKind::Output,
        ),
        v_q: reg(
            "ctrl.foc.v_q",
            "V",
            "q-axis voltage command",
            SignalKind::Output,
        ),
        i_d: reg("motor.i_d", "A", "d-axis current", SignalKind::State),
        i_q: reg("motor.i_q", "A", "q-axis current", SignalKind::State),
        omega: reg(
            "motor.omega",
            "rad/s",
            "mechanical speed",
            SignalKind::State,
        ),
        theta: reg("motor.theta", "rad", "mechanical angle", SignalKind::State),
        i_a: reg("motor.i_a", "A", "phase a current", SignalKind::Output),
        i_b: reg("motor.i_b", "A", "phase b current", SignalKind::Output),
        i_c: reg("motor.i_c", "A", "phase c current", SignalKind::Output),
    };
    let omega_ref = reg(
        "ctrl.omega_ref",
        "rad/s",
        "speed setpoint",
        SignalKind::Input,
    );
    let iq_ref = reg(
        "ctrl.foc.iq_ref",
        "A",
        "q-axis current reference",
        SignalKind::Output,
    );
    let mut plant = Plant::new();
    plant.add(Box::new(DqMotor {
        p: mp,
        extra_j: opts.extra_j,
        locked: opts.locked,
        s,
    }));
    let period = period_from_hz(1.0 / cfg.dt)
        .map(|p| p.period)
        .unwrap_or(SimTime(50_000));
    let blocks: Vec<Box<dyn DiscreteBlock>> = vec![Box::new(FocBlock {
        foc: Foc::new(cfg, FocGains::design(&mp, &cfg)),
        period,
        open_loop_vq: opts.open_loop_vq,
        s,
        omega_ref,
        iq_ref,
        ratio: opts.ratio,
    })];
    Engine::new(plant, bus, blocks, opts.dt_max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::commands::{ChangeSource, EngineCommand};

    #[test]
    fn engine_skeleton_reaches_speed_with_closed_energy_balance() {
        let mut e = build_engine(SkeletonOptions::default());
        e.queue(EngineCommand::SetSignal {
            path: "ctrl.omega_ref".into(),
            value: 20.0,
            source: ChangeSource::Internal,
        });
        e.step_until(SimTime::from_secs_f64(0.3)).unwrap();
        let w = e.bus.get(e.bus.id("motor.omega").unwrap());
        assert!((w - 20.0).abs() < 0.02 * 20.0, "omega {w}");
        let r = e.bus.get(e.bus.id("energy.residual").unwrap());
        assert!(r.abs() < 1e-3, "energy residual {r}");
    }

    #[test]
    fn locked_rotor_energy_balance_closes() {
        let mut e = build_engine(SkeletonOptions {
            locked: true,
            open_loop_vq: Some(1.0),
            dt_max: 2.5e-7,
            ..Default::default()
        });
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        let r = e.bus.get(e.bus.id("energy.residual").unwrap());
        assert!(r.abs() < 1e-6, "energy residual {r}");
    }
}
