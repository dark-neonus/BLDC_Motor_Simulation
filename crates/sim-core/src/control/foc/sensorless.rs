//! Sensorless FOC startup and hand-off (EQ-CTRL-10): one supervisor block owns the
//! sequence align → I/f ramp → blend → closed loop (with fallback), the velocity PI, and
//! the electrical angle and speed the current loop uses (`ctrl.sensorless.theta/omega`).
//! The observer (EQ-CTRL-09) provides `est.theta`, `est.omega` (electrical), `est.valid`.

use std::f64::consts::{PI, TAU};

use crate::control::pid::{AntiWindup, Pid};
use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

/// Startup settings; `None` fields take the EQ-CTRL-10 defaults.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StartupOptions {
    pub align_current: Option<f64>,
    pub align_time: Option<f64>,
    pub current: Option<f64>,
    /// Load side [rad/s²].
    pub accel: Option<f64>,
    /// Electrical [rad/s].
    pub handoff_speed: Option<f64>,
    pub valid_time: Option<f64>,
    pub blend_time: Option<f64>,
}

/// Resolved startup values (SI; speeds electrical, accel electrical).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Startup {
    pub align_current: f64,
    pub align_time: f64,
    pub current: f64,
    pub accel_e: f64,
    pub handoff_e: f64,
    pub valid_time: f64,
    pub blend_time: f64,
}

impl StartupOptions {
    /// Defaults (EQ-CTRL-10): I_align = I_if = 0.5·I_lim, t_align 0.3 s,
    /// a = 0.25·K_t I_if/(N J) load side, ω_handoff = 3·ω_min, t_valid 20 ms, t_blend 50 ms.
    /// `n_p` = N·p (electrical per load rad), `j` the motor-side inertia.
    pub fn resolve(
        &self,
        i_lim: f64,
        kt: f64,
        j: f64,
        n_p: f64,
        ratio: f64,
        omega_min: f64,
    ) -> Startup {
        let i_if = self.current.unwrap_or(0.5 * i_lim);
        let accel_load = self.accel.unwrap_or(0.25 * kt * i_if / (ratio * j));
        Startup {
            align_current: self.align_current.unwrap_or(0.5 * i_lim),
            align_time: self.align_time.unwrap_or(0.3),
            current: i_if,
            accel_e: n_p * accel_load,
            handoff_e: self.handoff_speed.unwrap_or(3.0 * omega_min),
            valid_time: self.valid_time.unwrap_or(0.02),
            blend_time: self.blend_time.unwrap_or(0.05),
        }
    }
}

fn wrap_pi(x: f64) -> f64 {
    let y = x.rem_euclid(TAU);
    if y > PI { y - TAU } else { y }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum Mode {
    Align,
    IfRamp,
    Blend,
    Closed,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
struct State {
    mode: Mode,
    t_mode: Option<f64>,
    /// θ*, ω*_e of the open-loop ramp.
    theta: f64,
    omega: f64,
    /// Since when est.valid (I/f) or its loss (closed loop) has held.
    since: Option<f64>,
    /// Blend: θ̂ − θ* unwrapped, and its last wrapped value.
    delta: f64,
    last: f64,
}

impl Default for State {
    fn default() -> Self {
        Self {
            mode: Mode::Align,
            t_mode: None,
            theta: 0.0,
            omega: 0.0,
            since: None,
            delta: 0.0,
            last: 0.0,
        }
    }
}

/// The supervisor's inputs and outputs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorlessIo {
    pub omega_ref: SignalId,
    pub est_theta: SignalId,
    pub est_omega: SignalId,
    pub est_valid: SignalId,
    pub id_ref: SignalId,
    pub iq_ref: SignalId,
    /// `ctrl.sensorless.theta`, `.omega` (electrical), `.state` — registered by `register`.
    pub theta: SignalId,
    pub omega: SignalId,
    pub state: SignalId,
}

/// Registers `ctrl.sensorless.theta/omega/state`; returns their ids.
pub fn register(bus: &mut SignalBus) -> Result<[SignalId; 3], SignalError> {
    let mut reg = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Output);
    Ok([
        reg(
            "ctrl.sensorless.theta",
            "rad",
            "electrical angle used by the current loop",
        )?,
        reg(
            "ctrl.sensorless.omega",
            "rad/s",
            "electrical speed used by the current loop",
        )?,
        reg(
            "ctrl.sensorless.state",
            "-",
            "startup state: 0 align, 1 I/f, 2 blend, 3 closed loop",
        )?,
    ])
}

pub struct SensorlessBlock {
    period: SimTime,
    s: Startup,
    io: SensorlessIo,
    pi: Pid,
    i_max: f64,
    /// N·p: load rad/s → electrical rad/s; p: electrical → mechanical (motor).
    n_p: f64,
    p: f64,
    st: State,
}

impl SensorlessBlock {
    /// `pi`: the velocity PI (motor-side speed → A), clamped to ±`i_max`.
    pub fn new(
        period: SimTime,
        s: Startup,
        io: SensorlessIo,
        pi: Pid,
        i_max: f64,
        n_p: f64,
        p: f64,
    ) -> Self {
        Self {
            period,
            s,
            io,
            pi,
            i_max,
            n_p,
            p,
            st: State::default(),
        }
    }
}

impl DiscreteBlock for SensorlessBlock {
    fn id(&self) -> &str {
        "controller.sensorless"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let now = ctx.t.as_secs_f64();
        let t = self.period.as_secs_f64();
        let b = &*ctx.bus;
        let target_e = self.n_p * b.get(self.io.omega_ref);
        let (th_hat, w_hat) = (b.get(self.io.est_theta), b.get(self.io.est_omega));
        let valid = b.get(self.io.est_valid) > 0.5;
        let (s, st) = (self.s, &mut self.st);
        let t_mode = *st.t_mode.get_or_insert(now);
        let sign = |w: f64| if w < 0.0 { -1.0 } else { 1.0 };
        // Advance the open-loop ramp (I/f and blend).
        let ramp = |st: &mut State| {
            let a = s.accel_e * t;
            st.omega += (target_e - st.omega).clamp(-a, a);
            st.theta = (st.theta + st.omega * t).rem_euclid(TAU);
        };
        let (id, iq, th, w) = match st.mode {
            Mode::Align => {
                if now - t_mode >= s.align_time {
                    (st.mode, st.t_mode, st.since) = (Mode::IfRamp, Some(now), None);
                }
                (s.align_current, 0.0, 0.0, 0.0)
            }
            Mode::IfRamp => {
                ramp(st);
                if valid && st.omega.abs() > s.handoff_e {
                    let since = *st.since.get_or_insert(now);
                    if now - since >= s.valid_time {
                        (st.mode, st.t_mode, st.since) = (Mode::Blend, Some(now), None);
                        // Wrapped once at the start; unwrapped from here on.
                        st.last = wrap_pi(th_hat - st.theta);
                        st.delta = st.last;
                    }
                } else {
                    st.since = None;
                }
                (0.0, s.current * sign(st.omega), st.theta, st.omega)
            }
            Mode::Blend => {
                ramp(st);
                let wgt = ((now - t_mode) / s.blend_time).min(1.0);
                let d = wrap_pi(th_hat - st.theta);
                st.delta += wrap_pi(d - st.last);
                st.last = d;
                let th = st.theta + wgt * st.delta;
                let iq = s.current * sign(st.omega);
                if wgt >= 1.0 {
                    // Bumpless: the velocity PI starts from the I/f current.
                    let y = w_hat / self.p;
                    self.pi.track(iq, target_e / self.p, y);
                    (st.mode, st.t_mode, st.since) = (Mode::Closed, Some(now), None);
                }
                (0.0, iq, th, st.omega + wgt * (w_hat - st.omega))
            }
            Mode::Closed => {
                if !valid {
                    let since = *st.since.get_or_insert(now);
                    if now - since >= s.valid_time {
                        // Fallback to I/f from the last estimate.
                        (st.theta, st.omega) = (th_hat, w_hat);
                        (st.mode, st.t_mode, st.since) = (Mode::IfRamp, Some(now), None);
                    }
                } else {
                    st.since = None;
                }
                let iq = self
                    .pi
                    .update(target_e / self.p, w_hat / self.p, t)
                    .clamp(-self.i_max, self.i_max);
                (0.0, iq, th_hat, w_hat)
            }
        };
        let mode = self.st.mode as u8;
        let bus = &mut *ctx.bus;
        bus.set(self.io.id_ref, id);
        bus.set(self.io.iq_ref, iq);
        bus.set(self.io.theta, th.rem_euclid(TAU));
        bus.set(self.io.omega, w);
        bus.set(self.io.state, f64::from(mode));
        Ok(())
    }
    fn reset(&mut self) {
        self.st = State::default();
        self.pi.reset();
    }
    fn save(&self) -> serde_json::Value {
        use crate::control::framework::LoopLaw;
        serde_json::json!({ "st": self.st, "pi": LoopLaw::save(&self.pi) })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        use crate::control::framework::LoopLaw;
        self.st = serde_json::from_value(v.get("st").cloned().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        LoopLaw::restore(&mut self.pi, v.get("pi").ok_or("missing `pi`")?)
    }
}

/// The velocity PI for the closed-loop phase (EQ-CTRL-04 design, clamping).
pub fn velocity_pi(kp: f64, ki: f64, i_max: f64) -> Pid {
    Pid::pi(kp, ki, (-i_max, i_max), AntiWindup::Clamping)
}
