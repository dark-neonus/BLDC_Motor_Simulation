//! Rigid gearbox (EQ-MECH-02/03): ratio N (motor turns per output turn), efficiency η
//! the same both ways, inertia on each side. Bypass = N 1, η 1, no inertia.

/// ω_ε of the loss's sign smoothing [rad/s] (EQ-MECH-03).
pub const OMEGA_EPS: f64 = 1e-3;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gearbox {
    /// Ratio N ≥ 1 [-].
    pub n: f64,
    /// Efficiency 0 < η ≤ 1 [-].
    pub eta: f64,
    /// Inertia on the motor side J_gi [kg·m²].
    pub j_in: f64,
    /// Inertia on the output side J_go [kg·m²].
    pub j_out: f64,
}

impl Default for Gearbox {
    fn default() -> Self {
        Self::BYPASS
    }
}

impl Gearbox {
    pub const BYPASS: Self = Self {
        n: 1.0,
        eta: 1.0,
        j_in: 0.0,
        j_out: 0.0,
    };

    /// Inertia J_eq seen at the motor shaft for a motor-side inertia `j_m` and a load
    /// inertia `j_load` (EQ-MECH-02).
    pub fn j_eq(&self, j_m: f64, j_load: f64) -> f64 {
        j_m + self.j_in + (self.j_out + j_load) / (self.n * self.n)
    }

    /// Meshing loss torque T_gl on the motor shaft (EQ-MECH-03) from the lossless
    /// acceleration `a0`, the load-side torque `t_l`, the load inertia and ω_m.
    /// T_gl·ω_m ≤ 0 always.
    pub fn loss_torque(&self, a0: f64, t_l: f64, j_load: f64, omega_m: f64) -> f64 {
        if self.eta >= 1.0 {
            return 0.0;
        }
        let tau_in = ((self.j_out + j_load) / self.n * a0 - t_l) / self.n;
        self.loss_for(tau_in, omega_m)
    }

    /// Loss torque for a known input-side transmitted torque τ_in (with backlash
    /// τ_in = τ_c/N).
    pub fn loss_for(&self, tau_in: f64, omega_m: f64) -> f64 {
        if self.eta >= 1.0 {
            return 0.0;
        }
        -(1.0 - self.eta) * tau_in.abs() * (omega_m / OMEGA_EPS).tanh()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reflected_inertia_and_loss_sign() {
        let g = Gearbox {
            n: 10.0,
            eta: 0.8,
            j_in: 1e-6,
            j_out: 1e-4,
        };
        assert!((g.j_eq(2e-5, 0.01) - (2e-5 + 1e-6 + 0.0101 / 100.0)).abs() < 1e-18);
        // The loss always opposes motion, whatever the sign of the transmitted torque.
        for (a0, t_l, w) in [(5.0, 0.3, 2.0), (-5.0, 0.3, 2.0), (1.0, -2.0, -3.0)] {
            assert!(g.loss_torque(a0, t_l, 0.01, w) * w <= 0.0);
        }
        assert_eq!(Gearbox::BYPASS.loss_torque(5.0, 1.0, 0.01, 2.0), 0.0);
    }
}
