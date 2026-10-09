//! Cross-check of the engine's RK4 against diffsol (independent adaptive solver),
//! EQ-NUM-07 / V-NUM-002. Each test ODE is solved both ways and compared.

use diffsol::{NalgebraLU, NalgebraMat, OdeBuilder, OdeSolverMethod};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::SignalBus;
use sim_core::engine::time::SimTime;

type M = NalgebraMat<f64>;

/// Wrap a plain rhs function as a single plant module.
struct FnModule {
    n: usize,
    x0: Vec<f64>,
    f: fn(&[f64], &mut [f64]),
}
impl PlantModule for FnModule {
    fn name(&self) -> &str {
        "fn"
    }
    fn n_states(&self) -> usize {
        self.n
    }
    fn state_names(&self) -> Vec<(String, String)> {
        (0..self.n).map(|i| (format!("x{i}"), "-".into())).collect()
    }
    fn init(&self, x: &mut [f64]) {
        x.copy_from_slice(&self.x0);
    }
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
    fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
        (self.f)(&x[o..o + self.n], dx);
    }
}

fn rk4_solution(f: fn(&[f64], &mut [f64]), x0: &[f64], t_end: f64, h: f64) -> Vec<f64> {
    let mut p = Plant::new();
    p.add(Box::new(FnModule {
        n: x0.len(),
        x0: x0.to_vec(),
        f,
    }));
    let mut bus = SignalBus::new();
    // Through the engine's advance path (not bare Rk4::integrate) — P03.T20.
    let _ = &mut bus;
    let mut e = Engine::new(p, SignalBus::new(), vec![], h);
    e.step_until(SimTime::from_secs_f64(t_end))
        .expect("engine run");
    e.x[..x0.len()].to_vec()
}

/// Finite-difference Jacobian-vector product (diffsol needs J·v; only used by its
/// implicit methods' Newton iterations, so FD accuracy is sufficient).
fn jv(f: fn(&[f64], &mut [f64]), x: &[f64], v: &[f64], y: &mut [f64]) {
    let eps = 1e-7;
    let n = x.len();
    let xp: Vec<f64> = x.iter().zip(v).map(|(a, b)| a + eps * b).collect();
    let (mut f0, mut f1) = (vec![0.0; n], vec![0.0; n]);
    f(x, &mut f0);
    f(&xp, &mut f1);
    for i in 0..n {
        y[i] = (f1[i] - f0[i]) / eps;
    }
}

macro_rules! diffsol_solution {
    ($f:expr, $x0:expr, $t_end:expr) => {{
        let x0: Vec<f64> = $x0.to_vec();
        let n = x0.len();
        let problem = OdeBuilder::<M>::new()
            .rtol(1e-10)
            .atol([1e-12])
            .rhs_implicit(
                move |x: &[f64], _p: &[f64], _t: f64, y: &mut [f64]| $f(x, y),
                move |x: &[f64], _p: &[f64], _t: f64, v: &[f64], y: &mut [f64]| jv($f, x, v, y),
            )
            .init(
                move |_p: &[f64], _t: f64, y: &mut [f64]| y.copy_from_slice(&x0),
                n,
            )
            .build()
            .expect("build problem");
        let mut solver = problem.bdf::<NalgebraLU<f64>>().expect("bdf");
        let (y, _stop) = solver.solve_dense(&[$t_end]).expect("solve");
        (0..n).map(|i| y[(i, 0)]).collect::<Vec<f64>>()
    }};
}

fn decay(x: &[f64], dx: &mut [f64]) {
    dx[0] = -2.0 * x[0];
}
fn oscillator(x: &[f64], dx: &mut [f64]) {
    dx[0] = x[1];
    dx[1] = -4.0 * x[0];
}
fn van_der_pol(x: &[f64], dx: &mut [f64]) {
    let mu = 1.0;
    dx[0] = x[1];
    dx[1] = mu * (1.0 - x[0] * x[0]) * x[1] - x[0];
}
/// Skeleton PMSM open loop (dq), vd = 0, vq = 6 V; states id, iq, ω, θ.
fn pmsm(x: &[f64], dx: &mut [f64]) {
    let (p, r, l, lam, j, b) = (14.0, 1.0, 2.5e-3, 0.03, 2.5e-4, 1e-4);
    let (id, iq, w) = (x[0], x[1], x[2]);
    let we = p * w;
    dx[0] = (0.0 - r * id + we * l * iq) / l;
    dx[1] = (6.0 - r * iq - we * l * id - we * lam) / l;
    dx[2] = (1.5 * p * lam * iq - b * w) / j;
    dx[3] = w;
}

fn assert_close(a: &[f64], b: &[f64], what: &str) {
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        // diffsol at rtol 1e-10 is the reference; RK4 at the fine step is ~1e-9 accurate.
        let tol = 1e-8 + 1e-6 * y.abs();
        assert!((x - y).abs() <= tol, "{what}[{i}]: rk4={x} diffsol={y}");
    }
}

#[test]
fn xcheck_decay() {
    let r = rk4_solution(decay, &[1.0], 2.0, 1e-3);
    let d = diffsol_solution!(decay, [1.0], 2.0);
    assert_close(&r, &d, "decay");
}

#[test]
fn xcheck_oscillator() {
    let r = rk4_solution(oscillator, &[1.0, 0.0], 5.0, 1e-3);
    let d = diffsol_solution!(oscillator, [1.0, 0.0], 5.0);
    assert_close(&r, &d, "oscillator");
}

#[test]
fn xcheck_van_der_pol() {
    let r = rk4_solution(van_der_pol, &[2.0, 0.0], 5.0, 1e-3);
    let d = diffsol_solution!(van_der_pol, [2.0, 0.0], 5.0);
    assert_close(&r, &d, "van der pol");
}

#[test]
fn xcheck_skeleton_pmsm_open_loop() {
    let r = rk4_solution(pmsm, &[0.0, 0.0, 0.0, 0.0], 0.05, 2e-6);
    let d = diffsol_solution!(pmsm, [0.0, 0.0, 0.0, 0.0], 0.05);
    assert_close(&r, &d, "pmsm");
}
