//! FOC position loop (EQ-CTRL-04): P (or PI) on the load angle → ω* on the motor side
//! (×N), clamped to the speed limit. Design K_{p,p} = ω_p (load rad/s per rad).

/// Motor-side (K_p, K_i) from the load-side K_p (= ω_p by default) and K_i (PI; none:
/// P), times the ratio N.
pub fn design(kp_load: f64, ratio: f64, ki_load: Option<f64>) -> (f64, f64) {
    (ratio * kp_load, ratio * ki_load.unwrap_or(0.0))
}

/// A P-only position loop over a PI velocity loop holds zero error under a constant
/// load; over a P-only velocity loop the error is τ_L/(N² K_t K_{p,v} K_{p,p})
/// (EQ-CTRL-04).
pub fn p_over_p_error(tau_l: f64, ratio: f64, kt: f64, kp_v: f64, kp_p_load: f64) -> f64 {
    tau_l / (ratio * ratio * kt * kp_v * kp_p_load)
}
