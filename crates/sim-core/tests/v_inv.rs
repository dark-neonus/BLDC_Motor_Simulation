//! V-INV-004 and V-INV-006 (validation-catalog.md) on the switching inverter.

use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::inverter::leg::{Devices, SwitchingInverter, duty_ids};
use sim_core::physics::inverter::pwm::{Pwm, PwmConfig};
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, LegInputs, MotorElectrical,
};

const D: Devices = Devices {
    r_on: 0.02,
    v_f: 0.8,
};
const V_BUS: f64 = 24.0;
const T_NS: i64 = 50_000;
const L: f64 = 2.5e-3;

struct Bus(SignalId);
impl PlantModule for Bus {
    fn name(&self) -> &str {
        "bus"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        bus.set(self.0, V_BUS);
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

/// Locked rotor, switching inverter with PWM at the given duties and dead time.
fn rig(duty: [f64; 3], dead_ns: i64) -> Engine {
    let mut bus = SignalBus::new();
    let vb = bus.register("bus.v", "V", "", SignalKind::Output).unwrap();
    let inv = SwitchingInverter::new(D, vb, &mut bus).unwrap();
    let rp = RotorParams {
        j: 1e-4,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: 7.0,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .unwrap()
        .with_locked(true);
    let inp = ElectricalInputs {
        v: inv.outputs_ids(),
        theta_e: bus.id("motor.theta_e").unwrap(),
        omega_e: bus.id("motor.omega_e").unwrap(),
    };
    let ep = ElectricalParams {
        r: 1.0,
        ld: L,
        lq: L,
        lambda: 0.03,
        pole_pairs: 7.0,
        shape: Shape::Sinusoidal,
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let legs = ["inverter.leg_a", "inverter.leg_b", "inverter.leg_c"].map(|n| bus.id(n).unwrap());
    let motor = MotorElectrical::new(ep, inp, &mut bus)
        .unwrap()
        .with_legs(LegInputs {
            legs,
            v_bus: vb,
            v_f: D.v_f,
            dead_time: dead_ns as f64 * 1e-9,
        });
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let inv = inv.with_motor(&bus).unwrap();
    for (id, d) in duty_ids(&mut bus).unwrap().iter().zip(duty) {
        bus.set(*id, d);
    }
    let pwm = Pwm::new(
        PwmConfig {
            period_ns: T_NS,
            dead_ns,
        },
        &mut bus,
    )
    .unwrap();
    let mut plant = Plant::new();
    plant.add(Box::new(Bus(vb)));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    plant.add(Box::new(inv));
    Engine::new(plant, bus, vec![Box::new(pwm)], 1e-6)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

/// Average of `names` over one PWM period starting at `t0_ns`, sampled every `h_ns`
/// (midpoint of each interval) — the edges sit on multiples of 5 ns here.
fn period_mean(e: &mut Engine, t0_ns: i64, h_ns: i64, names: &[&str]) -> Vec<f64> {
    let n = T_NS / h_ns;
    let mut m = vec![0.0; names.len()];
    for k in 0..n {
        e.step_until(SimTime::from_nanos(t0_ns + k * h_ns + h_ns / 2))
            .unwrap();
        for (j, name) in names.iter().enumerate() {
            m[j] += sig(e, name) / n as f64;
        }
    }
    m
}

/// V-INV-004: with dead time, the period-average terminal voltage of a leg errs by
/// ∓(t_d/T)(V_bus + 2V_f − 2R_on|i|) for i ≷ 0 (rtol 2 % on the error).
#[test]
fn v_inv_004_dead_time_voltage_error() {
    let duty = [0.6, 0.4, 0.4];
    let mut e = rig(duty, 1000);
    e.step_until(SimTime::from_secs_f64(0.04)).unwrap();
    let t0 = 800 * T_NS;
    let m = period_mean(
        &mut e,
        t0,
        5,
        &["inverter.v_a", "inverter.v_b", "motor.i_a", "motor.i_b"],
    );
    let frac = 1000.0 / T_NS as f64;
    for (x, (v, i)) in [(m[0], m[2]), (m[1], m[3])].into_iter().enumerate() {
        let err = v - (duty[x] * V_BUS - D.r_on * i);
        let want = -i.signum() * frac * (V_BUS + 2.0 * D.v_f - 2.0 * D.r_on * i.abs());
        assert!(
            (err / want - 1.0).abs() < 0.02,
            "leg {x}: i = {i}, error {err} vs {want}"
        );
    }
    assert!(m[2] > 0.0 && m[3] < 0.0, "both current signs covered");
    // Legs b and c share their dead time (equal duties): a PWM transient, no warning.
    let two_off = e.drain_events().into_iter().any(|ev| {
        matches!(ev, sim_core::engine::commands::EngineEvent::Warning { ref msg, .. } if msg.contains("legs stay Off"))
    });
    assert!(!two_off, "two-legs-Off warning during dead time");
}

/// V-INV-006: locked rotor, leg a PWM at D, leg b Low (D = 0), phase c disconnected,
/// t_d = 0: the a–b path current ripples by Δi = V_bus·D(1−D)·T/(2L_s) (rtol 5 %).
#[test]
fn v_inv_006_current_ripple() {
    let dd = 0.3;
    let mut e = rig([dd, 0.0, 0.0], 0);
    e.queue(EngineCommand::SetParam {
        path: "motor.open_phase".into(),
        value: 2.0,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(0.04)).unwrap();
    assert_eq!(sig(&e, "motor.open_phase"), 2.0);
    let t0 = 800 * T_NS;
    let (mut lo, mut hi) = (f64::MAX, f64::MIN);
    for k in 0..=1000 {
        e.step_until(SimTime::from_nanos(t0 + k * 50)).unwrap();
        let i = sig(&e, "motor.i_a");
        (lo, hi) = (lo.min(i), hi.max(i));
    }
    let want = V_BUS * dd * (1.0 - dd) * (T_NS as f64 * 1e-9) / (2.0 * L);
    assert!(
        ((hi - lo) / want - 1.0).abs() < 0.05,
        "{} vs {want}",
        hi - lo
    );
}
