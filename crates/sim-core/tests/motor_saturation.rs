//! P05.T06: co-energy saturation (EQ-MOT-10) on the engine.

use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, MotorElectrical, SatParams, inv_clarke,
};

const P: f64 = 7.0;
const R: f64 = 1.0;
const L: f64 = 2.5e-3;
const LAMBDA: f64 = 0.03;
// Positive definite over the reachable currents (|i| ≲ 60 A here): λ·|i_d|·max h″ =
// 0.03·60·2c/i_k² = 4.5e-4 < L_∞ (EQ-MOT-10 validity, checked by sim-model).
const SAT: SatParams = SatParams {
    i_k: 20.0,
    l_inf: 0.75e-3,
    c: 0.05,
};

struct Volts {
    ids: [SignalId; 3],
    f: Box<dyn Fn(f64) -> [f64; 3] + Send>,
}

impl PlantModule for Volts {
    fn name(&self) -> &str {
        "inverter"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, t: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        for (id, v) in self.ids.iter().zip((self.f)(t)) {
            bus.set(*id, v);
        }
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

fn build(
    sat: Option<SatParams>,
    locked: bool,
    i_q0: f64,
    f: Box<dyn Fn(f64) -> [f64; 3] + Send>,
    dt: f64,
) -> Engine {
    let mut bus = SignalBus::new();
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let rp = RotorParams {
        j: 1e-4,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: P,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .unwrap()
        .with_locked(locked);
    let inp = ElectricalInputs {
        v,
        theta_e: bus.id("motor.theta_e").unwrap(),
        omega_e: bus.id("motor.omega_e").unwrap(),
    };
    let ep = ElectricalParams {
        r: R,
        ld: L,
        lq: L,
        lambda: LAMBDA,
        pole_pairs: P,
        shape: Shape::Sinusoidal,
        i_d0: 0.0,
        i_q0,
        theta_e0: 0.0,
        saturation: sat,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus).unwrap();
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let mut plant = Plant::new();
    plant.add(Box::new(Volts { ids: v, f }));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    Engine::new(plant, bus, vec![], dt)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

/// At θe = 0 the q axis is β: v_αβ = (0, v_q).
fn q_voltage(vq: f64) -> [f64; 3] {
    inv_clarke(0.0, vq)
}

#[test]
fn below_the_knee_it_behaves_like_the_linear_model() {
    // Step to 0.5 A (5 % of the knee).
    let step = |sat| build(sat, true, 0.0, Box::new(|_| q_voltage(0.5 * R)), 1e-5);
    let (mut s, mut l) = (step(Some(SAT)), step(None));
    for t in [1e-3, 2.5e-3, 5e-3, 2e-2] {
        s.step_until(SimTime::from_secs_f64(t)).unwrap();
        l.step_until(SimTime::from_secs_f64(t)).unwrap();
        let (a, b) = (sig(&s, "motor.i_q"), sig(&l, "motor.i_q"));
        assert!((a / b - 1.0).abs() < 0.01, "t = {t}: {a} vs linear {b}");
        let (ta, tb) = (sig(&s, "motor.torque_em"), sig(&l, "motor.torque_em"));
        assert!((ta / tb - 1.0).abs() < 0.01, "torque {ta} vs {tb}");
    }
}

#[test]
fn above_the_knee_torque_per_amp_drops_per_the_curve() {
    let iq = 2.5 * SAT.i_k;
    // Hold the current: steady state needs v = R·i.
    let mut e = build(
        Some(SAT),
        true,
        iq,
        Box::new(move |_| q_voltage(R * iq)),
        1e-5,
    );
    e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
    let u = iq / SAT.i_k;
    let h = SAT.c * u * u / (1.0 + u * u);
    let want = 1.5 * P * LAMBDA * (1.0 - h) * iq; // EQ-MOT-10 with i_d = 0
    assert!((sig(&e, "motor.i_q") / iq - 1.0).abs() < 1e-9);
    assert!((sig(&e, "motor.torque_em") / want - 1.0).abs() < 1e-9);
    // h(2.5) = 0.05·6.25/7.25 ≈ 4.3 %: torque per amp is visibly below the linear Kt.
    assert!(want < 0.97 * 1.5 * P * LAMBDA * iq, "torque per amp fell");
}

#[test]
fn energy_closes_for_a_pulse_into_saturation() {
    // 20 ms pulse that drives i_q to ~3× the knee on a free rotor, then a short circuit.
    let f = |t: f64| q_voltage(if t < 0.02 { 60.0 } else { 0.0 });
    let mut e = build(Some(SAT), false, 0.0, Box::new(f), 1e-6);
    let mut peak: f64 = 0.0;
    for k in 1..=40 {
        e.step_until(SimTime::from_secs_f64(k as f64 * 1e-3))
            .unwrap();
        peak = peak.max(sig(&e, "motor.i_q").abs());
    }
    // Back-EMF of the spinning rotor limits the current; 1.5·i_k is well past the knee.
    assert!(
        peak > 1.5 * SAT.i_k,
        "pulse reached saturation: peak {peak}"
    );
    // The voltage step is a discontinuity in t, so RK4 drops order once: rtol 1e-6.
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}
