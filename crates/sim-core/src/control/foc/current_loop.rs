//! FOC current loop (EQ-CTRL-03):
//!
//! 1. measured i_a, i_b, i_c → Clarke → Park at θ̂e (encoder: p·θ + θ_e0, or `est.theta`);
//! 2. PI on d and q (EQ-CTRL-01) with the decoupling feed-forward
//!    v_d^ff = −ω̂e L_q i_q, v_q^ff = ω̂e (L_d i_d + λ_m);
//! 3. voltage limit with d priority: V_max = m_max V_bus/√3, |v_d| ≤ V_max,
//!    |v_q| ≤ √(V_max² − v_d²); saturation and anti-windup act on the total command;
//! 4. inverse Park at θ̂e + 1.5 T ω̂e, then SVPWM duties (EQ-INV-05) for an inverter.
//!
//! Gains by pole-zero cancellation: K_p = ω_c L, K_i = ω_c R (L_d on d, L_q on q). The
//! command computed at t_k is applied from t_{k+1} (one period of computation delay).

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;
use crate::physics::inverter::modulation::svpwm_ab;
use crate::physics::motor::electrical::clarke;

use crate::control::estimators::velocity::DiffLpf;
use crate::control::framework::Scaled;
use crate::control::pid::{AntiWindup, Pid};

/// Default modulation limit m_max (EQ-CTRL-03).
pub const M_MAX: f64 = 0.95;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurrentLoopConfig {
    pub period: SimTime,
    /// First tick offset (0 = `pwm.center` when the period is T_pwm).
    pub phase: SimTime,
    /// The controller's motor model: R [Ω], L_d, L_q [H], λ_m [Wb].
    pub r: f64,
    pub ld: f64,
    pub lq: f64,
    pub lambda: f64,
    /// Design bandwidth ω_c [rad/s].
    pub omega_c: f64,
    pub m_max: f64,
    pub aw: AntiWindup,
}

impl CurrentLoopConfig {
    /// Default ω_c = 2π f_ctrl / 20 (EQ-CTRL-03).
    pub fn default_omega_c(period: SimTime) -> f64 {
        2.0 * std::f64::consts::PI / period.as_secs_f64() / 20.0
    }
    fn gains(&self) -> [(f64, f64); 2] {
        [
            (self.omega_c * self.ld, self.omega_c * self.r),
            (self.omega_c * self.lq, self.omega_c * self.r),
        ]
    }
}

/// Electrical speed ω̂e for the decoupling and the angle advance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpeedSource {
    /// A speed signal times a gain (e.g. `est.omega`, or `motor.omega` × p).
    Signal(Scaled),
    /// Difference + low-pass on θ̂e itself (EQ-CTRL-08a).
    FromAngle(DiffLpf),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusVoltage {
    Signal(SignalId),
    Fixed(f64),
}

/// What the loop reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurrentLoopInputs {
    /// Phase currents (ADC readings or the true currents).
    pub currents: [SignalId; 3],
    /// θ̂e = gain · signal + offset (encoder: gain p, offset θ_e0).
    pub angle: Scaled,
    pub angle_offset: f64,
    pub speed: SpeedSource,
    pub v_bus: BusVoltage,
}

/// The loop's signals (`ctrl.foc.*`); register them before the plant that reads
/// `v_alpha`/`v_beta`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FocSignals {
    pub id_ref: SignalId,
    pub iq_ref: SignalId,
    pub v_d: SignalId,
    pub v_q: SignalId,
    pub v_alpha: SignalId,
    pub v_beta: SignalId,
    pub i_d: SignalId,
    pub i_q: SignalId,
    pub saturated: SignalId,
    /// θ̂e0 added to the measured angle every tick (EQ-CTRL-11; calibration writes it).
    pub angle_offset: SignalId,
    /// While `angle_forced` = 1 the loop uses `angle_cmd` instead (calibration).
    pub angle_cmd: SignalId,
    pub angle_forced: SignalId,
}

impl FocSignals {
    pub fn register(bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut reg = |p: &str, u: &str, d: &str, k| bus.register(p, u, d, k);
        use SignalKind::{Input, Output};
        Ok(Self {
            id_ref: reg("ctrl.foc.id_ref", "A", "d-current reference", Input)?,
            iq_ref: reg("ctrl.foc.iq_ref", "A", "q-current reference", Input)?,
            v_d: reg("ctrl.foc.v_d", "V", "d voltage command (applied)", Output)?,
            v_q: reg("ctrl.foc.v_q", "V", "q voltage command (applied)", Output)?,
            v_alpha: reg(
                "ctrl.foc.v_alpha",
                "V",
                "α voltage command (applied)",
                Output,
            )?,
            v_beta: reg(
                "ctrl.foc.v_beta",
                "V",
                "β voltage command (applied)",
                Output,
            )?,
            i_d: reg(
                "ctrl.foc.i_d",
                "A",
                "measured d current (controller frame)",
                Output,
            )?,
            i_q: reg(
                "ctrl.foc.i_q",
                "A",
                "measured q current (controller frame)",
                Output,
            )?,
            saturated: reg(
                "ctrl.foc.saturated",
                "-",
                "1 while the voltage limit clamps the command",
                Output,
            )?,
            angle_offset: reg(
                "ctrl.foc.angle_offset",
                "rad",
                "electrical angle offset θ̂e0 added to the measured angle (calibration)",
                Input,
            )?,
            angle_cmd: reg(
                "ctrl.foc.angle_cmd",
                "rad",
                "commanded electrical angle used while ctrl.foc.angle_forced = 1",
                Input,
            )?,
            angle_forced: reg(
                "ctrl.foc.angle_forced",
                "-",
                "1: the current loop uses ctrl.foc.angle_cmd (calibration)",
                Input,
            )?,
        })
    }
}

/// The command waiting for the next tick: (v_d, v_q, v_α, v_β, duties, saturated).
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
struct Command {
    vd: f64,
    vq: f64,
    va: f64,
    vb: f64,
    duty: [f64; 3],
    saturated: bool,
}

pub struct FocCurrentLoop {
    cfg: CurrentLoopConfig,
    inp: CurrentLoopInputs,
    sig: FocSignals,
    duties: Option<[SignalId; 3]>,
    pi: [Pid; 2],
    /// The bandwidth as set [Hz] (ω_c = 2π·this), returned unchanged.
    bw_hz: f64,
    speed: SpeedSource,
    next: Command,
    /// Test mode: v_d = 0, v_q fixed (no current control).
    open_loop_vq: Option<f64>,
}

impl FocCurrentLoop {
    /// `duties`: the inverter duty inputs (`inverter.duty_a..c`), or `None` with an
    /// ideal source reading `ctrl.foc.v_alpha`/`v_beta`.
    pub fn new(
        cfg: CurrentLoopConfig,
        inp: CurrentLoopInputs,
        sig: FocSignals,
        duties: Option<[SignalId; 3]>,
    ) -> Result<Self, String> {
        for (n, x) in [
            ("R", cfg.r),
            ("L_d", cfg.ld),
            ("L_q", cfg.lq),
            ("ω_c", cfg.omega_c),
        ] {
            if !(x.is_finite() && x > 0.0) {
                return Err(format!("current loop: {n} must be > 0, got {x}"));
            }
        }
        if !(cfg.m_max > 0.0 && cfg.m_max <= 1.0) {
            return Err(format!(
                "modulation limit must be in (0, 1], got {}",
                cfg.m_max
            ));
        }
        let pi = cfg
            .gains()
            .map(|(kp, ki)| Pid::pi(kp, ki, (f64::NEG_INFINITY, f64::INFINITY), cfg.aw));
        Ok(Self {
            cfg,
            inp,
            sig,
            duties,
            pi,
            bw_hz: cfg.omega_c / (2.0 * std::f64::consts::PI),
            speed: inp.speed,
            next: Command::default(),
            open_loop_vq: None,
        })
    }

    /// Open-loop test mode: apply v_d = 0, v_q = `vq` (`None`: closed loop).
    pub fn with_open_loop_vq(mut self, vq: Option<f64>) -> Self {
        self.open_loop_vq = vq;
        self
    }
}

impl DiscreteBlock for FocCurrentLoop {
    fn id(&self) -> &str {
        "controller.current"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.cfg.period)
    }
    fn phase(&self) -> SimTime {
        self.cfg.phase
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let bus = &mut *ctx.bus;
        // Measure.
        let t = self.cfg.period.as_secs_f64();
        let th = if bus.get(self.sig.angle_forced) > 0.5 {
            bus.get(self.sig.angle_cmd)
        } else {
            self.inp.angle.gain * bus.get(self.inp.angle.id)
                + self.inp.angle_offset
                + bus.get(self.sig.angle_offset)
        };
        let w = match &mut self.speed {
            SpeedSource::Signal(s) => s.gain * bus.get(s.id),
            SpeedSource::FromAngle(f) => f.update(th).1,
        };
        let [ia, ib, ic] = self.inp.currents.map(|id| bus.get(id));
        let (al, be) = clarke(ia, ib, ic);
        let (sn, cs) = th.sin_cos();
        let (id, iq) = (cs * al + sn * be, -sn * al + cs * be);
        bus.set(self.sig.i_d, id);
        bus.set(self.sig.i_q, iq);
        // PI + decoupling, d-priority voltage limit (EQ-CTRL-03 steps 2–3).
        let v_bus = match self.inp.v_bus {
            BusVoltage::Signal(s) => bus.get(s),
            BusVoltage::Fixed(v) => v,
        }
        .max(0.0);
        let v_max = self.cfg.m_max * v_bus / 3f64.sqrt();
        let p = &self.cfg;
        let ff_d = -w * p.lq * iq;
        let ff_q = w * (p.ld * id + p.lambda);
        let (rd, rq) = (bus.get(self.sig.id_ref), bus.get(self.sig.iq_ref));
        let (vd, vq) = match self.open_loop_vq {
            Some(v) => (0.0, v),
            None => {
                let vd = self.pi[0].step(rd, id, ff_d, (-v_max, v_max), t);
                let vq_max = (v_max * v_max - vd * vd).max(0.0).sqrt();
                (vd, self.pi[1].step(rq, iq, ff_q, (-vq_max, vq_max), t))
            }
        };
        // Inverse Park at the advanced angle; SVPWM for an inverter. Closed loop: the
        // command applies one period later and is held, 1.5·T on average. The open-loop
        // test mode is a voltage source, not a computation: it applies now, held 0.5·T.
        let lead = if self.open_loop_vq.is_some() {
            0.5
        } else {
            1.5
        };
        let (sn, cs) = (th + lead * t * w).sin_cos();
        let (va, vb) = (cs * vd - sn * vq, sn * vd + cs * vq);
        let duty = if self.duties.is_some() {
            svpwm_ab(va, vb, v_bus.max(1e-9)).0
        } else {
            [0.0; 3]
        };
        let new = Command {
            vd,
            vq,
            va,
            vb,
            duty,
            saturated: self.pi[0].saturated || self.pi[1].saturated,
        };
        // Apply the command computed at the previous tick (EQ-CTRL-03 timing), or the
        // open-loop voltage at once.
        let c = if self.open_loop_vq.is_some() {
            new
        } else {
            std::mem::replace(&mut self.next, new)
        };
        for (id, v) in [
            (self.sig.v_d, c.vd),
            (self.sig.v_q, c.vq),
            (self.sig.v_alpha, c.va),
            (self.sig.v_beta, c.vb),
            (self.sig.saturated, f64::from(u8::from(c.saturated))),
        ] {
            bus.set(id, v);
        }
        if let Some(ids) = self.duties {
            for (id, d) in ids.iter().zip(c.duty) {
                bus.set(*id, d);
            }
        }
        Ok(())
    }
    fn reset(&mut self) {
        self.pi.iter_mut().for_each(Pid::reset);
        self.speed = self.inp.speed;
        self.next = Command::default();
    }
    fn params(&self) -> Vec<(String, String)> {
        vec![("bandwidth".into(), "Hz".into())]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        (name == "bandwidth").then_some(self.bw_hz)
    }
    /// `controller.current.bandwidth` [Hz]: re-designs K_p, K_i live (integrators kept).
    fn set_param(&mut self, name: &str, value: f64) -> Result<f64, String> {
        if name != "bandwidth" {
            return Err(format!("unknown parameter `{name}`"));
        }
        if !(value.is_finite() && value > 0.0) {
            return Err(format!("bandwidth must be > 0, got {value}"));
        }
        let old = self.get_param(name).unwrap_or_default();
        self.bw_hz = value;
        self.cfg.omega_c = 2.0 * std::f64::consts::PI * value;
        for (pi, (kp, ki)) in self.pi.iter_mut().zip(self.cfg.gains()) {
            (pi.kp, pi.ki) = (kp, ki);
        }
        Ok(old)
    }
    fn save(&self) -> serde_json::Value {
        use crate::control::framework::LoopLaw;
        serde_json::json!({
            "pi": [LoopLaw::save(&self.pi[0]), LoopLaw::save(&self.pi[1])],
            "next": self.next,
            "speed": match &self.speed {
                SpeedSource::FromAngle(f) => serde_json::to_value(f).unwrap_or_default(),
                SpeedSource::Signal(_) => serde_json::Value::Null,
            },
            "bandwidth": self.bw_hz,
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        use crate::control::framework::LoopLaw;
        let bw = v
            .get("bandwidth")
            .and_then(serde_json::Value::as_f64)
            .ok_or("missing `bandwidth`")?;
        self.set_param("bandwidth", bw)?;
        let pi = v
            .get("pi")
            .and_then(serde_json::Value::as_array)
            .ok_or("missing `pi`")?;
        if pi.len() != 2 {
            return Err("expected two PI states".into());
        }
        for (p, s) in self.pi.iter_mut().zip(pi) {
            LoopLaw::restore(p, s)?;
        }
        self.next = serde_json::from_value(v.get("next").cloned().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        if let SpeedSource::FromAngle(f) = &mut self.speed {
            *f = serde_json::from_value(v.get("speed").cloned().unwrap_or_default())
                .map_err(|e| e.to_string())?;
        }
        Ok(())
    }
}
