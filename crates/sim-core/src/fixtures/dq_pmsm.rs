//! Ideal surface-mount PMSM in the rotor (dq) frame — temporary skeleton model.
//!
//! Conventions (CONVENTIONS §5): amplitude-invariant Park transform, `p` = pole
//! pairs, λ = peak phase flux linkage, ωe = p·ωm.
//!
//! ```text
//! did/dt = (vd − R·id + ωe·Lq·iq) / Ld
//! diq/dt = (vq − R·iq − ωe·Ld·id − ωe·λ) / Lq
//! T      = 1.5·p·(λ·iq + (Ld − Lq)·id·iq)
//! J·dωm/dt = T − B·ωm,   dθm/dt = ωm
//! ```

use crate::skeleton::rk4::rk4_step;

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

/// State of the skeleton model.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PmsmState {
    /// d-axis current [A].
    pub id: f64,
    /// q-axis current [A].
    pub iq: f64,
    /// Mechanical speed [rad/s].
    pub omega: f64,
    /// Mechanical angle [rad].
    pub theta: f64,
}

/// Ideal PMSM driven by dq voltages.
#[derive(Debug, Clone)]
pub struct Pmsm {
    pub params: PmsmParams,
    pub state: PmsmState,
    /// When true the rotor is held: ω and θ stay constant (locked-rotor test).
    pub locked: bool,
}

impl Pmsm {
    pub fn new(params: PmsmParams) -> Self {
        Self {
            params,
            state: PmsmState::default(),
            locked: false,
        }
    }

    /// Electromagnetic torque [N·m] for the current state.
    pub fn torque(&self) -> f64 {
        let p = &self.params;
        let s = &self.state;
        1.5 * p.p * (p.lambda * s.iq + (p.ld - p.lq) * s.id * s.iq)
    }

    /// d/dt of (i_d, i_q, ω, θ) for given dq voltages (EQ-MOT-06 + rigid rotor).
    pub fn derivatives(p: &PmsmParams, locked: bool, vd: f64, vq: f64, x: &[f64; 4]) -> [f64; 4] {
        let (id, iq, omega) = (x[0], x[1], x[2]);
        let we = p.p * omega;
        let torque = 1.5 * p.p * (p.lambda * iq + (p.ld - p.lq) * id * iq);
        let (dw, dth) = if locked {
            (0.0, 0.0)
        } else {
            ((torque - p.b * omega) / p.j, omega)
        };
        [
            (vd - p.r * id + we * p.lq * iq) / p.ld,
            (vq - p.r * iq - we * p.ld * id - we * p.lambda) / p.lq,
            dw,
            dth,
        ]
    }

    /// Advance by `dt` seconds with constant dq voltages (zero-order hold).
    pub fn step(&mut self, vd: f64, vq: f64, dt: f64) {
        let p = self.params;
        let locked = self.locked;
        let s = self.state;
        let mut x = [s.id, s.iq, s.omega, s.theta];
        rk4_step(&mut x, dt, |x, dx| {
            let d = Self::derivatives(&p, locked, vd, vq, &[x[0], x[1], x[2], x[3]]);
            dx.copy_from_slice(&d);
        });
        self.state = PmsmState {
            id: x[0],
            iq: x[1],
            omega: x[2],
            theta: x[3],
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Locked rotor (ω = 0), vq step: iq(t) = (V/R)·(1 − e^{−t·R/L}), id stays 0.
    #[test]
    fn locked_rotor_current_rises_with_rl_time_constant() {
        let params = PmsmParams::skeleton_6020();
        let mut m = Pmsm::new(params);
        m.locked = true;
        let v = 1.0;
        let tau = params.lq / params.r;
        let dt = tau / 100.0;
        let mut t = 0.0;
        for _ in 0..500 {
            m.step(0.0, v, dt);
            t += dt;
            let exact = v / params.r * (1.0 - (-t / tau).exp());
            // RK4 local error at dt = τ/100 is ~1e-10 relative; atol covers t ≈ 0.
            let tol = 1e-9 + 1e-6 * exact.abs();
            assert!(
                (m.state.iq - exact).abs() < tol,
                "t={t}: iq={} exact={exact}",
                m.state.iq
            );
            assert!(m.state.id.abs() < 1e-12);
        }
    }

    /// Steady state of the full model with derivatives = 0 (Ld = Lq = L):
    /// id = ωe·L·iq/R,  V = R·iq + ωe·L·id + ωe·λ,  Kt·iq = B·ω.
    /// Solved for ω by bisection; with B → 0 this tends to ω = V/(p·λ).
    fn steady_speed(p: &PmsmParams, v: f64) -> f64 {
        let residual = |w: f64| {
            let we = p.p * w;
            let iq = p.b * w / p.kt();
            let id = we * p.lq * iq / p.r;
            p.r * iq + we * p.ld * id + we * p.lambda - v
        };
        let (mut lo, mut hi) = (0.0, 2.0 * v / (p.p * p.lambda));
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if residual(mid) > 0.0 {
                hi = mid
            } else {
                lo = mid
            }
        }
        0.5 * (lo + hi)
    }

    #[test]
    fn free_spin_reaches_analytic_steady_state_speed() {
        let params = PmsmParams::skeleton_6020();
        let mut m = Pmsm::new(params);
        let v = 6.0;
        let dt = 5e-6;
        for _ in 0..(0.5 / dt) as usize {
            m.step(0.0, v, dt);
        }
        let expected = steady_speed(&params, v);
        // After 0.5 s ≫ the ~1 ms electromechanical time constant the transient has decayed
        // to round-off; 1e-4 rel (+1e-9 abs floor) leaves margin for the bisection error.
        let err = (m.state.omega - expected).abs();
        assert!(
            err <= 1e-9 + 1e-4 * expected,
            "omega={} expected={expected}",
            m.state.omega
        );
        // Sanity vs. the B → 0 limit V/(p·λ): B·ω is ~0.1 % of the torque scale, so within 1 %.
        let ideal = v / (params.p * params.lambda);
        assert!((m.state.omega - ideal).abs() <= 1e-9 + 0.01 * ideal);
    }
}
