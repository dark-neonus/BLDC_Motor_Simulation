//! Integrators for the continuous plant (EQ-NUM-03/04).

use super::plant::Plant;
use super::signals::SignalBus;

/// Classic RK4 with preallocated scratch buffers (no allocation per step).
#[derive(Debug, Clone, Default)]
pub struct Rk4 {
    k1: Vec<f64>,
    k2: Vec<f64>,
    k3: Vec<f64>,
    k4: Vec<f64>,
    tmp: Vec<f64>,
}

impl Rk4 {
    pub fn new(n: usize) -> Self {
        Self {
            k1: vec![0.0; n],
            k2: vec![0.0; n],
            k3: vec![0.0; n],
            k4: vec![0.0; n],
            tmp: vec![0.0; n],
        }
    }

    fn ensure(&mut self, n: usize) {
        if self.k1.len() != n {
            *self = Self::new(n);
        }
    }

    /// One RK4 step of size `h` from `t`. EQ-NUM-03.
    // Index loops mirror the textbook stage formulas; iterator zips would hide them.
    #[allow(clippy::needless_range_loop)]
    pub fn step(&mut self, plant: &mut Plant, bus: &mut SignalBus, t: f64, x: &mut [f64], h: f64) {
        let n = x.len();
        self.ensure(n);
        plant.eval(t, x, bus, &mut self.k1);
        for i in 0..n {
            self.tmp[i] = x[i] + 0.5 * h * self.k1[i];
        }
        plant.eval(t + 0.5 * h, &self.tmp, bus, &mut self.k2);
        for i in 0..n {
            self.tmp[i] = x[i] + 0.5 * h * self.k2[i];
        }
        plant.eval(t + 0.5 * h, &self.tmp, bus, &mut self.k3);
        for i in 0..n {
            self.tmp[i] = x[i] + h * self.k3[i];
        }
        plant.eval(t + h, &self.tmp, bus, &mut self.k4);
        for i in 0..n {
            x[i] += h / 6.0 * (self.k1[i] + 2.0 * self.k2[i] + 2.0 * self.k3[i] + self.k4[i]);
        }
    }

    /// Integrate from `t0` to `t1` in `ceil((t1−t0)/dt_max)` equal substeps, then
    /// refresh the bus outputs so they match the accepted state.
    pub fn integrate(
        &mut self,
        plant: &mut Plant,
        bus: &mut SignalBus,
        x: &mut [f64],
        t0: f64,
        t1: f64,
        dt_max: f64,
    ) {
        let span = t1 - t0;
        if span > 0.0 {
            let n = (span / dt_max).ceil().max(1.0) as usize;
            let h = span / n as f64;
            for k in 0..n {
                self.step(plant, bus, t0 + k as f64 * h, x, h);
            }
        }
        plant.outputs(t1, x, bus);
    }
}

/// Exact update of a linear first-order RL state with constant input over `h`:
/// `i' = (v − R i)/L` ⇒ `i(h) = i∞ + (i0 − i∞)·e^{−hR/L}`, `i∞ = v/R`. Stable for any h.
/// EQ-NUM-04 (optional stiff electrical step, Ideal/Standard tiers).
pub fn exp_rl_step(i0: f64, v: f64, r: f64, l: f64, h: f64) -> f64 {
    let i_inf = v / r;
    i_inf + (i0 - i_inf) * (-h * r / l).exp()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::plant::PlantModule;

    struct Linear {
        /// dx = a·x (decay) or harmonic oscillator when `osc`.
        a: f64,
        osc: bool,
    }
    impl PlantModule for Linear {
        fn name(&self) -> &str {
            "lin"
        }
        fn n_states(&self) -> usize {
            2
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![("x".into(), "-".into()), ("y".into(), "-".into())]
        }
        fn init(&self, x: &mut [f64]) {
            x[0] = 1.0;
            x[1] = 0.0;
        }
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
        fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
            if self.osc {
                dx[0] = x[o + 1];
                dx[1] = -x[o];
            } else {
                dx[0] = self.a * x[o];
                dx[1] = 0.0;
            }
        }
    }

    fn run(a: f64, osc: bool, t_end: f64, h: f64) -> Vec<f64> {
        let mut p = Plant::new();
        p.add(Box::new(Linear { a, osc }));
        let mut bus = SignalBus::new();
        let mut x = p.initial_state();
        Rk4::new(2).integrate(&mut p, &mut bus, &mut x, 0.0, t_end, h);
        x
    }

    #[test]
    fn exponential_decay_is_accurate() {
        let x = run(-1.0, false, 1.0, 1e-2);
        // RK4 global error ≈ 1e-11 here; 1e-10 abs + 1e-9 rel.
        assert!((x[0] - (-1.0f64).exp()).abs() <= 1e-10 + 1e-9 * (-1.0f64).exp());
    }

    #[test]
    fn rk4_is_fourth_order() {
        let exact = (-1.0f64).exp();
        let e1 = (run(-1.0, false, 1.0, 0.1)[0] - exact).abs();
        let e2 = (run(-1.0, false, 1.0, 0.05)[0] - exact).abs();
        let ratio = e1 / e2;
        assert!(
            (ratio - 16.0).abs() <= 1.6,
            "error ratio {ratio} (expected ≈ 16)"
        );
    }

    #[test]
    fn oscillator_energy_drift_is_bounded() {
        // 1e4 periods would be slow in debug; 1000 periods at h = T/200 (2π/200).
        let periods = 1000.0;
        let x = run(
            0.0,
            true,
            periods * 2.0 * std::f64::consts::PI,
            2.0 * std::f64::consts::PI / 200.0,
        );
        let energy = 0.5 * (x[0] * x[0] + x[1] * x[1]);
        // RK4 dissipates ~h^5 per step: at h = 0.0314 the drift over 2e5 steps is ~1e-6.
        assert!((energy - 0.5).abs() < 1e-5, "energy {energy}");
    }

    #[test]
    fn exponential_rl_step_is_exact_for_constant_input() {
        let (r, l, v, h) = (1.0, 2.5e-3, 3.0, 1e-3);
        let mut i = 0.0;
        for k in 1..=10 {
            i = exp_rl_step(i, v, r, l, h);
            let exact = v / r * (1.0 - (-(k as f64) * h * r / l).exp());
            assert!((i - exact).abs() <= 1e-12 + 1e-12 * exact.abs());
        }
        // Stable even with h ≫ τ.
        assert!((exp_rl_step(0.0, v, r, l, 10.0) - v / r).abs() < 1e-12);
    }
}
