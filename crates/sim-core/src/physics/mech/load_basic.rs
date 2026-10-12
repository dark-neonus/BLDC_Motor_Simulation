//! Simple load torques on the output shaft (EQ-MECH-07) and the disturbance pulse
//! (EQ-MECH-08).

/// ω_ε of the brake's sign smoothing [rad/s] (EQ-MECH-07).
pub const BRAKE_OMEGA_EPS: f64 = 1e-3;

/// Constant torque T₀ (fixed direction, e.g. a weight on a drum): external work.
pub fn constant(t0: f64) -> f64 {
    t0
}

/// Brake: −T₀·tanh(ω_L/ω_ε), always opposing motion (dissipated).
pub fn brake(t0: f64, omega_l: f64) -> f64 {
    -t0 * (omega_l / BRAKE_OMEGA_EPS).tanh()
}

/// Viscous (fan/pump-like at low speed): −c·ω_L (dissipated).
pub fn viscous(c: f64, omega_l: f64) -> f64 {
    -c * omega_l
}

/// Pulse length for an impulse H (EQ-MECH-08): Δt = max(1 ms, 5·dt_max).
pub fn impulse_duration(dt_max: f64) -> f64 {
    (5.0 * dt_max).max(1e-3)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dissipative_loads_oppose_motion() {
        for w in [-3.0, -1e-4, 1e-4, 2.0] {
            assert!(brake(0.5, w) * w < 0.0);
            assert!(viscous(0.1, w) * w < 0.0);
        }
        assert_eq!(constant(0.3), 0.3);
        assert_eq!(impulse_duration(5e-6), 1e-3);
        assert_eq!(impulse_duration(1e-3), 5e-3);
    }
}
