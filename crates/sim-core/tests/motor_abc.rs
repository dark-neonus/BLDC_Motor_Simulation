//! P05.T01/T02: abc electrical model on the engine (EQ-MOT-01…07).

use std::f64::consts::PI;

use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, MotorElectrical, inv_clarke,
};

/// Ideal terminal-voltage source: v(t) per phase.
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

const P: f64 = 7.0;
const R: f64 = 1.0;
const L: f64 = 2.5e-3;
const LAMBDA: f64 = 0.03;

struct Setup {
    shape: Shape,
    rotor: RotorParams,
    locked: bool,
    i_q0: f64,
    v: Box<dyn Fn(f64) -> [f64; 3] + Send>,
    dt: f64,
}

fn build(s: Setup) -> Engine {
    let mut bus = SignalBus::new();
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"].map(|p| {
        bus.register(p, "V", "terminal voltage", SignalKind::Output)
            .unwrap()
    });
    let rotor = RotorRigid::new(s.rotor, &mut bus, vec![], vec![])
        .unwrap()
        .with_locked(s.locked);
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
        shape: s.shape,
        i_d0: 0.0,
        i_q0: s.i_q0,
        theta_e0: P * s.rotor.theta0,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus).unwrap();
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let mut plant = Plant::new();
    plant.add(Box::new(Volts { ids: v, f: s.v }));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    Engine::new(plant, bus, vec![], s.dt)
}

fn sig(e: &Engine, p: &str) -> f64 {
    e.bus.get(e.bus.id(p).unwrap())
}

fn rotor(theta0: f64, omega0: f64, j: f64) -> RotorParams {
    RotorParams {
        j,
        b: 0.0,
        pole_pairs: P,
        theta0,
        omega0,
    }
}

#[test]
fn locked_rotor_rl_step_has_tau_l_over_r() {
    // v_a = 1 V, v_b = v_c = 0 → v_α = 2/3 V: i_a = (2/3R)(1 − e^(−t/τ)), τ = L/R.
    let mut e = build(Setup {
        shape: Shape::Sinusoidal,
        rotor: rotor(0.0, 0.0, 1e-4),
        locked: true,
        i_q0: 0.0,
        v: Box::new(|_| [1.0, 0.0, 0.0]),
        dt: 1e-5,
    });
    let tau = L / R;
    for t in [1e-3, 2.5e-3, 5e-3, 1e-2] {
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        let want = 2.0 / 3.0 / R * (1.0 - (-t / tau).exp());
        // RK4 at dt = τ/250: local error ~ (dt/τ)⁵ → rtol 1e-8.
        assert!((sig(&e, "motor.i_a") / want - 1.0).abs() < 1e-8, "t={t}");
        assert!((sig(&e, "motor.i_b") + sig(&e, "motor.i_c") + sig(&e, "motor.i_a")).abs() < 1e-12);
    }
    assert!(
        sig(&e, "energy.residual").abs() < 1e-8,
        "{}",
        sig(&e, "energy.residual")
    );
}

#[test]
fn back_emf_line_to_line_peak_is_ke_times_omega() {
    // Huge inertia: ω stays at 100 rad/s while the shorted windings brake it.
    let mut e = build(Setup {
        shape: Shape::Sinusoidal,
        rotor: rotor(0.0, 100.0, 1e3),
        locked: false,
        i_q0: 0.0,
        v: Box::new(|_| [0.0; 3]),
        dt: 1e-5,
    });
    let mut peak: f64 = 0.0;
    let mut t = 0.0;
    while t < 2.0 * PI / (P * 100.0) * 1.5 {
        t += 1e-6;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        peak = peak.max(sig(&e, "motor.e_a") - sig(&e, "motor.e_b"));
    }
    // Ke = √3·p·λ (EQ-CONV-10); sampling at 1 µs: phase step 7e-4 rad → rtol 1e-6.
    let ke = 3f64.sqrt() * P * LAMBDA;
    assert!(
        (peak / (ke * 100.0) - 1.0).abs() < 1e-6,
        "{peak} vs {}",
        ke * 100.0
    );
}

#[test]
fn stall_torque_is_kt_times_iq() {
    let th = 0.3;
    let iq = 2.0;
    // Hold the current: v_αβ = R·i_αβ with i_αβ = R(θe)[0, iq].
    let (ia, ib) = (-(P * th).sin() * iq, (P * th).cos() * iq);
    let v = inv_clarke(R * ia, R * ib);
    let mut e = build(Setup {
        shape: Shape::Sinusoidal,
        rotor: rotor(th, 0.0, 1e-4),
        locked: true,
        i_q0: iq,
        v: Box::new(move |_| v),
        dt: 1e-5,
    });
    e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
    let kt = 1.5 * P * LAMBDA; // EQ-CONV-09
    assert!((sig(&e, "motor.i_q") / iq - 1.0).abs() < 1e-12);
    assert!((sig(&e, "motor.torque_em") / (kt * iq) - 1.0).abs() < 1e-12);
}

#[test]
fn trapezoidal_motor_conserves_energy_while_spinning() {
    // Free rotor, trapezoidal EMF (120° flat top), sinusoidal terminal voltages: the
    // harmonic torque term must match the harmonic back-EMF or energy leaks.
    let mut e = build(Setup {
        shape: Shape::Trapezoidal { w: 2.0 * PI / 3.0 },
        rotor: rotor(0.0, 0.0, 1e-4),
        locked: false,
        i_q0: 0.0,
        v: Box::new(|t| {
            let w = 2.0 * PI * 20.0;
            std::array::from_fn(|k| 6.0 * (w * t - k as f64 * 2.0 * PI / 3.0).cos())
        }),
        dt: 2e-6,
    });
    e.step_until(SimTime::from_secs_f64(0.2)).unwrap();
    assert!(sig(&e, "motor.omega").abs() > 1.0, "the motor moved");
    // Kinks of the trapezoid limit RK4 to low order there: rtol 1e-6 on the balance.
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}
