//! P05.T03 / EQ-MOT-06: the stationary-frame flux model (sinusoidal, linear) and the
//! classic dq model (P03.T12 fixture) integrate the same ODE in different coordinates,
//! so with identical dq voltages and initial conditions they agree to integrator accuracy.

use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::fixtures::dq_pmsm::{Pmsm, PmsmParams};
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, MotorElectrical, inv_clarke,
};

/// Rotor-frame voltage program (smooth, so both models see the same function of t).
fn vdq(t: f64) -> (f64, f64) {
    (
        0.5 * (30.0 * t).sin(),
        12.0 * (1.0 - (-t / 0.02).exp()) - 30.0 * (t - 0.1).max(0.0),
    )
}

fn params() -> PmsmParams {
    // Salient on purpose (L_d ≠ L_q) to exercise the reluctance term.
    PmsmParams {
        ld: 2.0e-3,
        lq: 3.0e-3,
        ..PmsmParams::skeleton_6020()
    }
}

/// abc terminal voltages = inverse Park/Clarke of vdq(t) at the model's own θe.
struct Source {
    v: [SignalId; 3],
    theta_e: SignalId,
}

impl PlantModule for Source {
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
        let th = bus.get(self.theta_e);
        let (vd, vq) = vdq(t);
        let (s, c) = th.sin_cos();
        let v = inv_clarke(c * vd - s * vq, s * vd + c * vq);
        for (id, vk) in self.v.iter().zip(v) {
            bus.set(*id, vk);
        }
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

/// The dq fixture as an engine module (same voltages, evaluated per RK stage).
struct DqRef {
    p: PmsmParams,
    out: [SignalId; 4],
}

impl PlantModule for DqRef {
    fn name(&self) -> &str {
        "dq"
    }
    fn n_states(&self) -> usize {
        4
    }
    fn state_names(&self) -> Vec<(String, String)> {
        ["id", "iq", "omega", "theta"]
            .iter()
            .map(|s| ((*s).into(), String::new()))
            .collect()
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        for k in 0..4 {
            bus.set(self.out[k], x[off + k]);
        }
    }
    fn derivatives(&self, t: f64, x: &[f64], off: usize, _: &SignalBus, dx: &mut [f64]) {
        let (vd, vq) = vdq(t);
        let s = [x[off], x[off + 1], x[off + 2], x[off + 3]];
        dx.copy_from_slice(&Pmsm::derivatives(&self.p, false, vd, vq, &s));
    }
}

const DT: f64 = 2e-6;

fn abc_engine(p: PmsmParams) -> Engine {
    let mut bus = SignalBus::new();
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let rp = RotorParams {
        j: p.j,
        j_load: 0.0,
        b: p.b,
        pole_pairs: p.p,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![]).unwrap();
    let theta_e = bus.id("motor.theta_e").unwrap();
    let inp = ElectricalInputs {
        v,
        theta_e,
        omega_e: bus.id("motor.omega_e").unwrap(),
    };
    let ep = ElectricalParams {
        r: p.r,
        ld: p.ld,
        lq: p.lq,
        lambda: p.lambda,
        pole_pairs: p.p,
        shape: Shape::Sinusoidal,
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus).unwrap();
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let mut plant = Plant::new();
    plant.add(Box::new(rotor));
    plant.add(Box::new(Source { v, theta_e }));
    plant.add(Box::new(motor));
    Engine::new(plant, bus, vec![], DT)
}

fn dq_engine(p: PmsmParams) -> Engine {
    let mut bus = SignalBus::new();
    let out = ["dq.id", "dq.iq", "dq.omega", "dq.theta"]
        .map(|n| bus.register(n, "", "", SignalKind::State).unwrap());
    let mut plant = Plant::new();
    plant.add(Box::new(DqRef { p, out }));
    Engine::new(plant, bus, vec![], DT)
}

fn get(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

#[test]
fn abc_matches_dq_for_sinusoidal_linear_machine() {
    let p = params();
    let (mut a, mut d) = (abc_engine(p), dq_engine(p));
    let mut worst: f64 = 0.0;
    for k in 1..=200 {
        let t = SimTime::from_secs_f64(k as f64 * 1e-3);
        a.step_until(t).unwrap();
        d.step_until(t).unwrap();
        let (id, iq, w) = (get(&d, "dq.id"), get(&d, "dq.iq"), get(&d, "dq.omega"));
        let torque = 1.5 * p.p * (p.lambda * iq + (p.ld - p.lq) * id * iq);
        for (abc, dq) in [
            (get(&a, "motor.i_d"), id),
            (get(&a, "motor.i_q"), iq),
            (get(&a, "motor.omega"), w),
            (get(&a, "motor.torque_em"), torque),
        ] {
            // CONVENTIONS §7: 1e-9 abs (i_d ≈ 0 at times) + 1e-6 rel, same dt.
            let tol = 1e-9 + 1e-6 * dq.abs();
            worst = worst.max((abc - dq).abs() / tol);
            assert!(
                (abc - dq).abs() <= tol,
                "t = {} ms: abc {abc} vs dq {dq}",
                k
            );
        }
    }
    assert!(
        get(&d, "dq.omega").abs() > 10.0,
        "the test exercises real motion"
    );
    eprintln!("worst error / tolerance = {worst:.3}");
}
