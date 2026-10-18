//! Open-loop V/f (EQ-CTRL-06):
//!
//! θ*_e = ∫ω*_e dt (ω*_e ramped at a),  |v| = V_0 + k_vf |ω*_e|,
//! v_αβ = |v|·[−sin θ*_e, cos θ*_e],
//!
//! with no feedback. Loss of synchronism is flagged (`ctrl.open_loop.sync_lost`, and a
//! warning) when |wrap(θ*_e − θ_e)| > π/2 for longer than one electrical period; the true
//! angle is used for this diagnostic only. The vector is applied at the centre of its
//! hold interval (θ* + ½ω*T).

use std::f64::consts::{PI, TAU};

use crate::control::foc::current_loop::BusVoltage;
use crate::control::framework::Scaled;
use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;
use crate::physics::inverter::modulation::svpwm_ab;

/// The open-loop family in SI units.
#[derive(Debug, Clone, PartialEq)]
pub struct OpenLoopSpec {
    pub rate_hz: f64,
    /// Boost V_0 [V] and slope k_vf [V per electrical rad/s]; `None`: the motor-relative
    /// defaults 2R·0.3·I_lim and 1.1·λ_m (resolved by `resolve`).
    pub v0: Option<f64>,
    pub k_vf: Option<f64>,
    /// Current limit [A] (for the default boost).
    pub i_max: f64,
    /// Speed-reference ramp, load side [rad/s²].
    pub accel: f64,
    /// Load-side speed limit [rad/s] (∞ = none).
    pub speed_limit: f64,
}

/// `ctrl.open_loop.*` signals; register before the ideal source that reads v_α, v_β.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpenLoopSignals {
    pub v_alpha: SignalId,
    pub v_beta: SignalId,
    pub theta_ref: SignalId,
    pub omega_ref: SignalId,
    pub sync_lost: SignalId,
}

impl OpenLoopSignals {
    pub fn register(bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut reg = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Output);
        Ok(Self {
            v_alpha: reg("ctrl.open_loop.v_alpha", "V", "α voltage command")?,
            v_beta: reg("ctrl.open_loop.v_beta", "V", "β voltage command")?,
            theta_ref: reg(
                "ctrl.open_loop.theta_ref",
                "rad",
                "commanded electrical angle θ*",
            )?,
            omega_ref: reg(
                "ctrl.open_loop.omega_ref",
                "rad/s",
                "commanded electrical speed ω* (ramped)",
            )?,
            sync_lost: reg(
                "ctrl.open_loop.sync_lost",
                "-",
                "1 once synchronism was lost (|θ* − θe| > 90° for an electrical period)",
            )?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
struct State {
    theta: f64,
    omega: f64,
    /// When |θ* − θe| first exceeded 90° (None while in sync).
    err_since: Option<f64>,
    lost: bool,
    /// Unwrapped error, its first value, and the last wrapped error (pole-slip rule).
    unwrapped: f64,
    first: Option<f64>,
    last: f64,
}

/// Resolved V/f settings.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Vf {
    v0: f64,
    k_vf: f64,
    accel: f64,
    speed_limit: f64,
}

pub struct OpenLoopBlock {
    period: SimTime,
    spec: Vf,
    /// Load setpoint → electrical speed: gain N·p on `ctrl.omega_ref`.
    reference: Scaled,
    elec_per_load: f64,
    theta_true: SignalId,
    sig: OpenLoopSignals,
    duties: Option<[SignalId; 3]>,
    v_bus: BusVoltage,
    st: State,
    warnings: Vec<String>,
}

impl OpenLoopBlock {
    /// `r`, `lambda`: the motor's phase resistance and flux (defaults of V_0 and k_vf).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        spec: OpenLoopSpec,
        r: f64,
        lambda: f64,
        omega_ref: SignalId,
        elec_per_load: f64,
        theta_true: SignalId,
        sig: OpenLoopSignals,
        duties: Option<[SignalId; 3]>,
        v_bus: BusVoltage,
    ) -> Result<Self, String> {
        let period = crate::engine::time::period_from_hz(spec.rate_hz)
            .map_err(|e| format!("controller.rate: {e}"))?
            .period;
        let spec = Vf {
            v0: spec.v0.unwrap_or(2.0 * r * 0.3 * spec.i_max),
            k_vf: spec.k_vf.unwrap_or(1.1 * lambda),
            accel: spec.accel,
            speed_limit: spec.speed_limit,
        };
        for (n, x) in [("v0", spec.v0), ("k_vf", spec.k_vf)] {
            if !(x.is_finite() && x >= 0.0) {
                return Err(format!("controller.{n} must be ≥ 0, got {x}"));
            }
        }
        if !spec.accel.is_finite() || spec.accel <= 0.0 {
            return Err(format!("controller.accel must be > 0, got {}", spec.accel));
        }
        Ok(Self {
            period,
            spec,
            reference: Scaled {
                id: omega_ref,
                gain: elec_per_load,
            },
            elec_per_load,
            theta_true,
            sig,
            duties,
            v_bus,
            st: State::default(),
            warnings: vec![],
        })
    }
}

fn wrap_pi(x: f64) -> f64 {
    let y = x.rem_euclid(TAU);
    if y > PI { y - TAU } else { y }
}

impl DiscreteBlock for OpenLoopBlock {
    fn id(&self) -> &str {
        "controller"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let t = self.period.as_secs_f64();
        let now = ctx.t.as_secs_f64();
        let k = self.elec_per_load;
        let lim = k.abs() * self.spec.speed_limit;
        let target = (self.reference.gain * ctx.bus.get(self.reference.id)).clamp(-lim, lim);
        let a = k.abs() * self.spec.accel * t;
        let st = &mut self.st;
        st.omega += (target - st.omega).clamp(-a, a);
        // Synchronism diagnostic (true angle; not used as feedback).
        let err = wrap_pi(st.theta - ctx.bus.get(self.theta_true));
        match st.first {
            None => (st.first, st.unwrapped) = (Some(err), err),
            Some(_) => st.unwrapped += wrap_pi(err - st.last),
        }
        st.last = err;
        let sustained = if st.omega != 0.0 && err.abs() > PI / 2.0 {
            let since = *st.err_since.get_or_insert(now);
            now - since > TAU / st.omega.abs()
        } else {
            st.err_since = None;
            false
        };
        let slipped = (st.unwrapped - st.first.unwrap_or(0.0)).abs() >= TAU;
        if !st.lost && (sustained || slipped) {
            st.lost = true;
            let why = if slipped {
                "the rotor slipped a pole"
            } else {
                "|θ* − θe| > 90° for more than an electrical period"
            };
            self.warnings.push(format!(
                "open-loop V/f lost synchronism at {now:.4} s: {why} (the load exceeds the pull-out torque)"
            ));
        }
        // Apply the vector at the centre of the hold interval, then advance θ*.
        let mag = self.spec.v0 + self.spec.k_vf * st.omega.abs();
        let th = st.theta + 0.5 * st.omega * t;
        let (va, vb) = (-mag * th.sin(), mag * th.cos());
        let bus = &mut *ctx.bus;
        bus.set(self.sig.v_alpha, va);
        bus.set(self.sig.v_beta, vb);
        if let Some(ids) = self.duties {
            let v = match self.v_bus {
                BusVoltage::Signal(s) => bus.get(s),
                BusVoltage::Fixed(v) => v,
            };
            let (d, _) = svpwm_ab(va, vb, v.max(1e-9));
            for (id, d) in ids.iter().zip(d) {
                bus.set(*id, d);
            }
        }
        bus.set(self.sig.theta_ref, st.theta);
        bus.set(self.sig.omega_ref, st.omega);
        bus.set(self.sig.sync_lost, f64::from(u8::from(st.lost)));
        st.theta = (st.theta + st.omega * t).rem_euclid(TAU);
        Ok(())
    }
    fn reset(&mut self) {
        self.st = State::default();
    }
    fn take_warnings(&mut self) -> Vec<String> {
        std::mem::take(&mut self.warnings)
    }
    fn params(&self) -> Vec<(String, String)> {
        vec![
            ("v0".into(), "V".into()),
            ("k_vf".into(), "V*s/rad".into()),
            ("accel".into(), "rad/s^2".into()),
        ]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match name {
            "v0" => Some(self.spec.v0),
            "k_vf" => Some(self.spec.k_vf),
            "accel" => Some(self.spec.accel),
            _ => None,
        }
    }
    fn set_param(&mut self, name: &str, value: f64) -> Result<f64, String> {
        let slot = match name {
            "v0" => &mut self.spec.v0,
            "k_vf" => &mut self.spec.k_vf,
            "accel" => &mut self.spec.accel,
            _ => return Err(format!("unknown parameter `{name}`")),
        };
        let ok = if name == "accel" {
            value > 0.0
        } else {
            value >= 0.0
        };
        if !(value.is_finite() && ok) {
            return Err(format!("{name}: invalid value {value}"));
        }
        Ok(std::mem::replace(slot, value))
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "st": self.st, "v0": self.spec.v0, "k_vf": self.spec.k_vf, "accel": self.spec.accel })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        (self.spec.v0, self.spec.k_vf, self.spec.accel) = (f("v0")?, f("k_vf")?, f("accel")?);
        self.st = serde_json::from_value(v.get("st").cloned().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        Ok(())
    }
}
