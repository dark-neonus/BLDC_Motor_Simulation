//! P09.T03: the FOC current loop (EQ-CTRL-03) on the abc motor with an ideal source:
//! V-CTRL-001 step time constant, i_d ≈ 0 while accelerating, no windup at the voltage
//! limit.

use sim_core::control::foc::current_loop::{
    BusVoltage, CurrentLoopConfig, CurrentLoopInputs, FocCurrentLoop, FocSignals, M_MAX,
    SpeedSource,
};
use sim_core::control::framework::Scaled;
use sim_core::control::pid::AntiWindup;
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::Plant;
use sim_core::engine::signals::SignalBus;
use sim_core::engine::time::SimTime;
use sim_core::physics::inverter::ideal::IdealVoltageSource;
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{ElectricalInputs, ElectricalParams, MotorElectrical};

const P: f64 = 7.0;
const R: f64 = 1.0;
const L: f64 = 2.5e-3;
const LAMBDA: f64 = 0.03;
const V_BUS: f64 = 24.0;
/// 20 kHz controller: ω_c = 2π·20 kHz/20 = 6283 rad/s, 1/ω_c = 159 µs.
const T_NS: i64 = 50_000;

fn rig(locked: bool, j: f64, aw: AntiWindup) -> Engine {
    let mut bus = SignalBus::new();
    let sig = FocSignals::register(&mut bus).unwrap();
    let src = IdealVoltageSource::new(&mut bus, sig.v_alpha, sig.v_beta, V_BUS).unwrap();
    let rp = RotorParams {
        j,
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
        v: src.outputs_ids(),
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
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus).unwrap();
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let period = SimTime::from_nanos(T_NS);
    let cfg = CurrentLoopConfig {
        period,
        phase: SimTime::from_nanos(0),
        r: R,
        ld: L,
        lq: L,
        lambda: LAMBDA,
        omega_c: CurrentLoopConfig::default_omega_c(period),
        m_max: M_MAX,
        aw,
        gains: None,
    };
    let currents = ["motor.i_a", "motor.i_b", "motor.i_c"].map(|n| bus.id(n).unwrap());
    let inputs = CurrentLoopInputs {
        currents,
        angle: Scaled::of(&bus, "motor.theta_e", 1.0).unwrap(),
        angle_offset: 0.0,
        speed: SpeedSource::Signal(Scaled::of(&bus, "motor.omega_e", 1.0).unwrap()),
        v_bus: BusVoltage::Fixed(V_BUS),
    };
    let foc = FocCurrentLoop::new(cfg, inputs, sig, None).unwrap();
    let mut plant = Plant::new();
    plant.add(Box::new(src));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    Engine::new(plant, bus, vec![Box::new(foc)], 2e-6)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn set(e: &mut Engine, n: &str, v: f64) {
    let id = e.bus.id(n).unwrap();
    e.bus.set(id, v);
}

fn omega_c() -> f64 {
    CurrentLoopConfig::default_omega_c(SimTime::from_nanos(T_NS))
}

/// V-CTRL-001: a q-current step (locked rotor, unsaturated) reaches 63.2 % after
/// τ63 ≈ 1/ω_c (rtol 15 %), measured from the reference change.
#[test]
fn v_ctrl_001_current_step_time_constant() {
    let mut e = rig(true, 1e-4, AntiWindup::Clamping);
    let step = 0.5;
    set(&mut e, "ctrl.foc.iq_ref", step);
    let dt = 1e-6;
    let mut t63 = None;
    for k in 1..=2000 {
        let t = k as f64 * dt;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        if t63.is_none() && sig(&e, "motor.i_q") >= 0.632 * step {
            t63 = Some(t);
        }
    }
    let t63 = t63.expect("reached 63 %");
    let want = 1.0 / omega_c();
    assert!(
        (t63 / want - 1.0).abs() < 0.15,
        "τ63 {:.1} µs vs 1/ω_c {:.1} µs",
        t63 * 1e6,
        want * 1e6
    );
    // Settles on the reference with no steady-state error, unsaturated.
    assert!((sig(&e, "motor.i_q") - step).abs() < 1e-3 * step);
    assert_eq!(sig(&e, "ctrl.foc.saturated"), 0.0);
}

/// With decoupling, i_d stays ≈ 0 while a q current accelerates the rotor through a
/// large speed change (0 → ~1200 rad/s electrical).
#[test]
fn id_stays_near_zero_while_accelerating() {
    let mut e = rig(false, 1.5e-4, AntiWindup::Clamping);
    let iq = 1.0;
    set(&mut e, "ctrl.foc.iq_ref", iq);
    let mut worst: f64 = 0.0;
    let mut t = 0.0;
    while t < 0.02 {
        t += 1e-4;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        if t > 1e-3 {
            worst = worst.max(sig(&e, "motor.i_d").abs());
        }
    }
    let w = sig(&e, "motor.omega_e");
    assert!(w > 250.0, "accelerated: ω_e {w}");
    assert!(
        sig(&e, "ctrl.foc.saturated") == 0.0,
        "stayed below the voltage limit"
    );
    assert!(worst < 0.02 * iq, "|i_d| ≤ {worst} A");
}

/// Voltage saturation without windup: 20 A asks for 20 V on the locked rotor, more than
/// V_max = 0.95·24/√3 = 13.2 V, so the loop sits at the limit (flag set, i_q ≈ 13.2 A).
/// Dropping the reference to 5 A, the current can fall no faster than the −V_max limit
/// allows: t_slew = (L/R)·ln((13.2 + 13.2)/(5.25 + 13.2)) ≈ 0.90 ms to the 5 % band.
/// Measured to stay within the band: back-calculation 0.95 ms (≈ t_slew), clamping
/// 1.75 ms (its integrator was frozen near 0 during the saturation and rebuilds the 5 V
/// with the PI's L/R zero), no anti-windup 26.9 ms (the integrator charged to hundreds
/// of volts holds the command at the limit).
#[test]
fn voltage_saturation_is_handled_without_windup() {
    let recover = |aw: AntiWindup| {
        let mut e = rig(true, 1e-4, aw);
        set(&mut e, "ctrl.foc.iq_ref", 20.0);
        e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        let v_max = M_MAX * V_BUS / 3f64.sqrt();
        assert_eq!(sig(&e, "ctrl.foc.saturated"), 1.0);
        assert!((sig(&e, "motor.i_q") - v_max / R).abs() < 0.01 * v_max / R);
        set(&mut e, "ctrl.foc.iq_ref", 5.0);
        let (t0, mut t) = (0.02, 0.02);
        let mut last_out = t0;
        while t < t0 + 0.2 {
            t += 1e-5;
            e.step_until(SimTime::from_secs_f64(t)).unwrap();
            if (sig(&e, "motor.i_q") - 5.0).abs() > 0.25 {
                last_out = t;
            }
        }
        last_out - t0
    };
    let with = recover(AntiWindup::Clamping);
    let without = recover(AntiWindup::None);
    let bc = recover(AntiWindup::BackCalculation);
    let v_max = M_MAX * V_BUS / 3f64.sqrt();
    let t_slew = L / R * ((v_max / R + v_max / R) / (5.25 + v_max / R)).ln();
    let ms = |x: f64| x * 1e3;
    assert!(
        bc < t_slew + 1.0 / omega_c(),
        "back-calculation {} ms",
        ms(bc)
    );
    assert!(with < t_slew + 6.0 / omega_c(), "clamping {} ms", ms(with));
    assert!(without > 10.0 * with, "no anti-windup: {} ms", ms(without));
}

#[test]
fn live_bandwidth_and_snapshot_restore() {
    let mut e = rig(true, 1e-4, AntiWindup::Clamping);
    set(&mut e, "ctrl.foc.iq_ref", 0.5);
    e.step_until(SimTime::from_secs_f64(2e-4)).unwrap();
    let snap = e.snapshot();
    let mut f = rig(true, 1e-4, AntiWindup::Clamping);
    f.restore(&snap).unwrap();
    for x in [&mut e, &mut f] {
        x.step_until(SimTime::from_secs_f64(5e-4)).unwrap();
    }
    assert_eq!(sig(&e, "motor.i_q"), sig(&f, "motor.i_q"));
    assert_eq!(sig(&e, "ctrl.foc.v_q"), sig(&f, "ctrl.foc.v_q"));
    let bw = omega_c() / (2.0 * std::f64::consts::PI);
    assert!((e.get_param("controller.current.bandwidth").unwrap() - bw).abs() < 1e-9);
    e.queue(sim_core::engine::commands::EngineCommand::SetParam {
        path: "controller.current.bandwidth".into(),
        value: 500.0,
        source: sim_core::engine::commands::ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(6e-4)).unwrap();
    assert_eq!(e.get_param("controller.current.bandwidth"), Some(500.0));
}
