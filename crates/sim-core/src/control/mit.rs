//! Impedance / MIT mode (EQ-CTRL-07):
//!
//! τ* = K_p(θ* − θ_L) + K_d(ω* − ω_L) + τ_ff,   i_q* = clamp(τ*/(N K_t)),
//!
//! on the load side, at its own rate, on top of the FOC current loop. No efficiency
//! factor: gear loss is speed-dependent and zero at standstill (EQ-MECH-03). τ_ff is
//! `controller.tau_ff` plus the torque setpoint `ctrl.torque_ref`.

use crate::control::framework::Scaled;
use crate::control::setpoint::Setpoints;
use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId};
use crate::engine::time::SimTime;

/// The MIT law in SI units (load side).
#[derive(Debug, Clone, PartialEq)]
pub struct MitSpec {
    /// The current loop underneath (its cascade is not built).
    pub foc: crate::control::foc::FocSpec,
    pub rate_hz: f64,
    /// Stiffness K_p [N·m/rad], damping K_d [N·m·s/rad], feed-forward τ_ff [N·m]; without
    /// K_p / K_d: K_p = J_L ω², K_d = 2ζ√(K_p J_L) (EQ-CTRL-12).
    pub kp: Option<f64>,
    pub kd: Option<f64>,
    pub omega: Option<f64>,
    pub zeta: f64,
    pub tau_ff: f64,
}

impl MitSpec {
    /// (K_p, K_d) for the load-side total inertia J_L.
    pub fn gains(&self, j_load: f64) -> Result<(f64, f64), String> {
        let kp = match (self.kp, self.omega) {
            (Some(kp), _) => kp,
            (None, Some(w)) => j_load * w * w,
            (None, None) => return Err("controller.kp: give kp or a bandwidth".into()),
        };
        let kd = self.kd.unwrap_or(2.0 * self.zeta * (kp * j_load).sqrt());
        Ok((kp, kd))
    }
}

/// Damping ratio of the hold: ζ = K_d / (2√(K_p J_L,tot)) (EQ-CTRL-07), with J_L,tot the
/// total inertia seen at the load (J_load + N² J_motor).
pub fn damping_ratio(kp: f64, kd: f64, j_load_total: f64) -> f64 {
    kd / (2.0 * (kp * j_load_total).sqrt())
}

pub struct MitBlock {
    period: SimTime,
    kp: f64,
    kd: f64,
    tau_ff: f64,
    sp: Setpoints,
    theta_l: Scaled,
    omega_l: Scaled,
    /// 1/(N K_t) and the current limit.
    amps_per_nm: f64,
    i_max: f64,
    iq_ref: SignalId,
}

impl MitBlock {
    /// `theta_l`/`omega_l`: the load angle and speed feedback; writes `iq_ref`.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        spec: &MitSpec,
        theta_l: Scaled,
        omega_l: Scaled,
        ratio: f64,
        kt: f64,
        j_motor: f64,
        iq_ref: SignalId,
        bus: &mut SignalBus,
    ) -> Result<Self, String> {
        let (kp, kd) = spec.gains(j_motor * ratio * ratio)?;
        let period = crate::engine::time::period_from_hz(spec.rate_hz)
            .map_err(|e| format!("controller.rate: {e}"))?
            .period;
        for (n, x) in [("kp", kp), ("kd", kd)] {
            if !(x.is_finite() && x >= 0.0) {
                return Err(format!("controller.{n} must be ≥ 0, got {x}"));
            }
        }
        let sp = Setpoints::get_or_register(bus).map_err(|e: SignalError| e.to_string())?;
        Ok(Self {
            period,
            kp,
            kd,
            tau_ff: spec.tau_ff,
            sp,
            theta_l,
            omega_l,
            amps_per_nm: 1.0 / (ratio * kt),
            i_max: spec.foc.i_max,
            iq_ref,
        })
    }
}

impl DiscreteBlock for MitBlock {
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
        let g = |s: Scaled| s.gain * ctx.bus.get(s.id);
        let (th, w) = (g(self.theta_l), g(self.omega_l));
        let tau = self.kp * (ctx.bus.get(self.sp.theta) - th)
            + self.kd * (ctx.bus.get(self.sp.omega) - w)
            + self.tau_ff
            + ctx.bus.get(self.sp.torque);
        let iq = (tau * self.amps_per_nm).clamp(-self.i_max, self.i_max);
        ctx.bus.set(self.iq_ref, iq);
        Ok(())
    }
    fn reset(&mut self) {}
    fn params(&self) -> Vec<(String, String)> {
        vec![
            ("kp".into(), "N*m/rad".into()),
            ("kd".into(), "N*m*s/rad".into()),
            ("tau_ff".into(), "N*m".into()),
        ]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match name {
            "kp" => Some(self.kp),
            "kd" => Some(self.kd),
            "tau_ff" => Some(self.tau_ff),
            _ => None,
        }
    }
    /// `controller.kp`, `controller.kd` (≥ 0), `controller.tau_ff` live (D-020).
    fn set_param(&mut self, name: &str, value: f64) -> Result<f64, String> {
        let (slot, nonneg) = match name {
            "kp" => (&mut self.kp, true),
            "kd" => (&mut self.kd, true),
            "tau_ff" => (&mut self.tau_ff, false),
            _ => return Err(format!("unknown parameter `{name}`")),
        };
        if !value.is_finite() || (nonneg && value < 0.0) {
            return Err(format!("{name}: invalid value {value}"));
        }
        Ok(std::mem::replace(slot, value))
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "kp": self.kp, "kd": self.kd, "tau_ff": self.tau_ff })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        (self.kp, self.kd, self.tau_ff) = (f("kp")?, f("kd")?, f("tau_ff")?);
        Ok(())
    }
}
