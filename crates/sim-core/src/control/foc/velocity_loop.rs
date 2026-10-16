//! FOC velocity loop (EQ-CTRL-04): PI on the motor speed → i_q*, clamped to the current
//! limit. Design K_p = ω_v J/K_t, K_i = K_p ω_v/4 (double closed-loop pole at ω_v/2,
//! 13.5 % step overshoot from the PI zero); the optional setpoint prefilter
//! 1/(1 + 4s/ω_v) removes the overshoot.

use crate::control::framework::LoopLaw;
use crate::control::pid::Pid;

/// (K_p, K_i) for bandwidth ω_v [rad/s], motor-side inertia J [kg·m²], K_t [N·m/A].
pub fn design(omega_v: f64, j: f64, kt: f64) -> (f64, f64) {
    let kp = omega_v * j / kt;
    (kp, kp * omega_v / 4.0)
}

/// A PI law behind the first-order setpoint prefilter 1/(1 + τs), τ = 4/ω_v,
/// discretised with backward Euler: r_f += T/(T + τ)·(r − r_f).
pub struct Prefiltered {
    pub pi: Pid,
    pub tau: f64,
    rf: Option<f64>,
}

impl Prefiltered {
    pub fn new(pi: Pid, omega_v: f64) -> Self {
        Self {
            pi,
            tau: 4.0 / omega_v,
            rf: None,
        }
    }
}

impl LoopLaw for Prefiltered {
    fn update(&mut self, r: f64, y: f64, t: f64) -> f64 {
        // Starts at the measurement: a loop enabled at speed does not see a step.
        let rf = self.rf.unwrap_or(y);
        let rf = rf + t / (t + self.tau) * (r - rf);
        self.rf = Some(rf);
        self.pi.update(rf, y, t)
    }
    fn reset(&mut self) {
        self.pi.reset();
        self.rf = None;
    }
    fn track(&mut self, u: f64, _r: f64, y: f64) {
        self.rf = Some(y);
        self.pi.track(u, y, y);
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "pi": LoopLaw::save(&self.pi), "rf": self.rf })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        LoopLaw::restore(&mut self.pi, v.get("pi").ok_or("missing `pi`")?)?;
        self.rf = v.get("rf").and_then(serde_json::Value::as_f64);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::pid::AntiWindup;

    /// The design on a pure inertia J·ω̇ = K_t·i_q (ideal current loop), discretised
    /// at a rate far above ω_v: 13.5 % overshoot without the prefilter, ≈ 0 with it
    /// (EQ-CTRL-04, V-CTRL-002 shape; atol 2 %).
    #[test]
    fn velocity_design_overshoot_with_and_without_prefilter() {
        let (j, kt, wv, t) = (1e-4, 0.05, 100.0, 1e-4);
        let (kp, ki) = design(wv, j, kt);
        let run = |pref: bool| {
            let pi = Pid::pi(
                kp,
                ki,
                (f64::NEG_INFINITY, f64::INFINITY),
                AntiWindup::Clamping,
            );
            let mut law: Box<dyn LoopLaw> = if pref {
                Box::new(Prefiltered::new(pi, wv))
            } else {
                Box::new(pi)
            };
            let (mut w, mut peak) = (0.0f64, 0.0f64);
            // Prefilter starts at the measurement (0) like a loop at rest.
            for _ in 0..20_000 {
                let iq = law.update(1.0, w, t);
                w += t * kt * iq / j;
                peak = peak.max(w);
            }
            (peak - 1.0, w)
        };
        let (os, w_end) = run(false);
        assert!((os - 0.135).abs() < 0.02, "overshoot {os}");
        assert!((w_end - 1.0).abs() < 1e-6);
        let (os, _) = run(true);
        assert!(os.abs() < 0.02, "prefiltered overshoot {os}");
    }
}
