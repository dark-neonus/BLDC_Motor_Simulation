//! The dq parameter bag the production build path designs the skeleton FOC from.

/// Motor parameters, SI units.
#[derive(Debug, Clone, Copy)]
pub struct PmsmParams {
    /// Pole pairs [-].
    pub p: f64,
    /// Phase resistance [Ω].
    pub r: f64,
    /// d-axis inductance [H].
    pub ld: f64,
    /// q-axis inductance [H].
    pub lq: f64,
    /// Peak phase flux linkage of the magnets [Wb].
    pub lambda: f64,
    /// Rotor (+ load) inertia [kg·m²].
    pub j: f64,
    /// Viscous friction [N·m·s/rad].
    pub b: f64,
}

impl PmsmParams {
    /// Illustrative 6020-class gimbal motor (P01.T01). Kt ≈ 0.63 N·m/A, Kv ≈ 13 rpm/V.
    /// Real presets arrive in P04.
    pub fn skeleton_6020() -> Self {
        Self {
            p: 14.0,
            r: 1.0,
            ld: 2.5e-3,
            lq: 2.5e-3,
            lambda: 0.03,
            j: 2.5e-4,
            b: 1e-4,
        }
    }

    /// Torque constant Kt = 1.5·p·λ [N·m per A of i_q].
    pub fn kt(&self) -> f64 {
        1.5 * self.p * self.lambda
    }
}
