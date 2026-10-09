//! P05.T07: open-phase mode (EQ-MOT-11/12).

use std::f64::consts::PI;

use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{ElectricalInputs, ElectricalParams, MotorElectrical};

struct Volts {
    ids: [SignalId; 3],
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
        let w = 2.0 * PI * 30.0;
        for (k, id) in self.ids.iter().enumerate() {
            bus.set(*id, 12.0 + 8.0 * (w * t - k as f64 * 2.0 * PI / 3.0).cos());
        }
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

fn engine(shape: Shape) -> Engine {
    let mut bus = SignalBus::new();
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let rp = RotorParams {
        j: 1e-4,
        j_load: 0.0,
        b: 1e-4,
        pole_pairs: 7.0,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![]).unwrap();
    let inp = ElectricalInputs {
        v,
        theta_e: bus.id("motor.theta_e").unwrap(),
        omega_e: bus.id("motor.omega_e").unwrap(),
    };
    let ep = ElectricalParams {
        r: 1.0,
        ld: 2.5e-3,
        lq: 2.5e-3,
        lambda: 0.03,
        pole_pairs: 7.0,
        shape,
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus).unwrap();
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let mut plant = Plant::new();
    plant.add(Box::new(Volts { ids: v }));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    Engine::new(plant, bus, vec![], 2e-6)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn set(e: &mut Engine, v: f64) {
    e.queue(EngineCommand::SetParam {
        path: "motor.open_phase".into(),
        value: v,
        source: ChangeSource::Internal,
    });
}

#[test]
fn open_phase_c_keeps_ic_zero_and_energy_closes() {
    for shape in [Shape::Sinusoidal, Shape::Trapezoidal { w: 2.0 * PI / 3.0 }] {
        let mut e = engine(shape.clone());
        e.step_until(SimTime::from_secs_f64(0.05)).unwrap();
        assert!(
            sig(&e, "motor.i_c").abs() > 0.1,
            "current flows before the fault"
        );
        set(&mut e, 2.0);
        let mut opened_at = None;
        for k in 1..=500 {
            let t = 0.05 + k as f64 * 1e-4;
            e.step_until(SimTime::from_secs_f64(t)).unwrap();
            let (ia, ib, ic) = (
                sig(&e, "motor.i_a"),
                sig(&e, "motor.i_b"),
                sig(&e, "motor.i_c"),
            );
            assert!(ia.is_finite() && ib.is_finite(), "no NaN at t = {t}");
            if sig(&e, "motor.open_phase") == 2.0 {
                opened_at.get_or_insert(t);
                assert_eq!(ic, 0.0, "i_c stays exactly zero once open");
                assert!((ia + ib).abs() < 1e-12, "i_a = −i_b");
            }
        }
        let t0 = opened_at.expect("phase c opened at a current zero");
        assert!(
            t0 < 0.05 + 0.02,
            "{shape:?}: opened within a period of the 30 Hz drive"
        );
        assert!(
            sig(&e, "motor.i_a").abs() > 0.01,
            "the a–b path still carries current"
        );
        // Bisection puts the switch at i_c ≈ 0 (T_TOL 1e-10 s): rtol 1e-6 on the balance.
        assert!(
            sig(&e, "energy.residual").abs() < 1e-6,
            "{shape:?}: {}",
            sig(&e, "energy.residual")
        );
        // Reconnect: currents are continuous and the model returns to two states.
        let ia_before = sig(&e, "motor.i_a");
        set(&mut e, -1.0);
        e.step_until(SimTime::from_secs_f64(0.1 + 2e-6)).unwrap();
        assert_eq!(sig(&e, "motor.open_phase"), -1.0);
        assert!(
            (sig(&e, "motor.i_a") - ia_before).abs() < 0.05,
            "continuous on reconnect"
        );
        e.step_until(SimTime::from_secs_f64(0.15)).unwrap();
        assert!(sig(&e, "motor.i_c").abs() > 0.0 && sig(&e, "energy.residual").abs() < 1e-6);
    }
}

/// A snapshot taken while the open is pending, and one taken while it is open, both restore
/// into a fresh engine (through bytes) and continue bit-identically, reconnect included.
#[test]
fn snapshot_restore_mid_fault_continues_bit_identically() {
    use sim_core::engine::snapshot::Snapshot;
    let t = SimTime::from_secs_f64;
    let fresh = |s: &Snapshot| {
        let mut b = engine(Shape::Sinusoidal);
        b.restore(&Snapshot::from_bytes(&s.to_bytes().unwrap()).unwrap())
            .unwrap();
        b
    };
    let mut a = engine(Shape::Sinusoidal);
    a.step_until(t(0.05)).unwrap();
    set(&mut a, 2.0);
    a.step_until(t(0.05 + 1e-5)).unwrap();
    assert_eq!(sig(&a, "motor.open_phase"), -1.0, "still pending");
    let pending = a.snapshot();
    a.step_until(t(0.08)).unwrap();
    assert_eq!(sig(&a, "motor.open_phase"), 2.0);
    let mut b = fresh(&pending);
    b.step_until(t(0.08)).unwrap();
    assert_eq!(a.snapshot().x, b.snapshot().x, "pending → open");

    let open = a.snapshot();
    let mut b = fresh(&open);
    for e in [&mut a, &mut b] {
        set(e, -1.0);
        e.step_until(t(0.12)).unwrap();
    }
    let (sa, sb) = (a.snapshot(), b.snapshot());
    assert_eq!(sa.x, sb.x, "open → reconnect");
    assert_eq!(sa.bus, sb.bus);
    assert!(sig(&b, "energy.residual").abs() < 1e-6);
}

/// At rest every current is zero: no crossing will come, so the open takes effect at once.
#[test]
fn open_request_at_zero_current_takes_effect_immediately() {
    let mut e = engine(Shape::Sinusoidal);
    set(&mut e, 0.0);
    e.apply_pending();
    assert_eq!(sig(&e, "motor.open_phase"), 0.0);
    e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
    assert_eq!(sig(&e, "motor.i_a"), 0.0);
    assert!(sig(&e, "motor.i_b").abs() > 0.01, "b–c path conducts");
}
