//! P07.T01: switching inverter legs (EQ-INV-01) with the motor's automatic floating.

use sim_core::engine::commands::{ChangeSource, EngineCommand, EngineEvent};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::inverter::leg::{Devices, LegState, SwitchingInverter};
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, LegInputs, MotorElectrical,
};

const D: Devices = Devices {
    r_on: 0.02,
    v_f: 0.8,
};
const V_BUS: f64 = 12.0;

/// Holds `bus.v` at a constant.
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

fn rig(locked: bool, omega0: f64, legs: [LegState; 3]) -> Engine {
    rig_dt(locked, omega0, legs, 0.0)
}

/// As `rig`, with the PWM dead time the motor assumes for its two-Off warning.
fn rig_dt(locked: bool, omega0: f64, legs: [LegState; 3], dead_time: f64) -> Engine {
    let mut bus = SignalBus::new();
    let vb = bus.register("bus.v", "V", "", SignalKind::Output).unwrap();
    let inv = SwitchingInverter::new(D, vb, &mut bus).unwrap();
    let rp = RotorParams {
        j: 1e-3,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: 7.0,
        theta0: 0.0,
        omega0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .unwrap()
        .with_locked(locked);
    let inp = ElectricalInputs {
        v: inv.outputs_ids(),
        theta_e: bus.id("motor.theta_e").unwrap(),
        omega_e: bus.id("motor.omega_e").unwrap(),
    };
    let ep = ElectricalParams {
        r: 1.0,
        ld: 2.5e-3,
        lq: 2.5e-3,
        lambda: 0.03,
        pole_pairs: 7.0,
        shape: Shape::Sinusoidal,
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let leg_ids =
        ["inverter.leg_a", "inverter.leg_b", "inverter.leg_c"].map(|n| bus.id(n).unwrap());
    let motor = MotorElectrical::new(ep, inp, &mut bus)
        .unwrap()
        .with_legs(LegInputs {
            legs: leg_ids,
            v_bus: vb,
            v_f: D.v_f,
            r_on: D.r_on,
            ho_duty: None,
            dead_time,
        });
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let inv = inv.with_motor(&bus).unwrap();
    inv.set_legs(&mut bus, legs);
    let mut plant = Plant::new();
    plant.add(Box::new(Bus(vb)));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    plant.add(Box::new(inv));
    Engine::new(plant, bus, vec![], 1e-6)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn set_leg(e: &mut Engine, x: &str, s: LegState) {
    e.queue(EngineCommand::SetSignal {
        path: format!("inverter.leg_{x}"),
        value: s.to_signal(),
        source: ChangeSource::Internal,
    });
}

/// A–(B,C) driven, then A turned Off: the current freewheels through the low diode
/// (v_a = −V_f), reaches zero, and phase a floats with i_a ≡ 0; turning A High again
/// reconnects it.
#[test]
fn off_leg_freewheels_then_floats_then_reconnects() {
    use LegState::*;
    let mut e = rig(true, 0.0, [High, Low, Low]);
    e.step_until(SimTime::from_secs_f64(0.06)).unwrap();
    let ia = sig(&e, "motor.i_a");
    // Steady: V_bus over R + R/2 plus the on-resistances (a high, b ∥ c low).
    let expect = V_BUS / (1.5 * 1.0 + 1.5 * D.r_on);
    assert!((ia / expect - 1.0).abs() < 1e-6, "{ia} vs {expect}");
    assert!((sig(&e, "inverter.v_a") - (V_BUS - D.r_on * ia)).abs() < 1e-9);
    set_leg(&mut e, "a", Off);
    e.step_until(SimTime::from_secs_f64(0.06 + 1e-4)).unwrap();
    assert_eq!(sig(&e, "inverter.v_a"), -D.v_f, "low diode clamps");
    assert!(sig(&e, "motor.i_a") > 0.0);
    e.step_until(SimTime::from_secs_f64(0.08)).unwrap();
    assert_eq!(sig(&e, "motor.open_phase"), 0.0, "floating after the zero");
    assert_eq!(sig(&e, "motor.i_a"), 0.0);
    set_leg(&mut e, "a", High);
    e.step_until(SimTime::from_secs_f64(0.14)).unwrap();
    assert_eq!(sig(&e, "motor.open_phase"), -1.0);
    assert!((sig(&e, "motor.i_a") / expect - 1.0).abs() < 1e-6);
}

/// Spinning faster than the bus with leg A Off: its free terminal voltage passes
/// V_bus + V_f and −V_f each cycle. The high diode conducts (i_a < 0, regeneration), the
/// low one too (i_a > 0; with B and C Low it shorts the motor), and the terminal stays
/// within the clamps. The inductive current reaches zero only after the EMF has passed
/// the opposite bound, so the current hands over diode to diode (EQ-MOT-12: the phase
/// reconnects at once when it opens beyond a bound).
#[test]
fn floating_terminal_reconnects_through_its_diode() {
    use LegState::*;
    // e peak = λ·p·ω = 0.03·7·200 = 42 V ≫ 12 V.
    let mut e = rig(false, 200.0, [Off, Low, Low]);
    let (mut neg, mut pos, mut handovers) = (false, false, 0);
    let mut prev = sig(&e, "motor.diode_a");
    for k in 1..=8000 {
        e.step_until(SimTime::from_secs_f64(k as f64 * 5e-6))
            .unwrap();
        let (ia, va) = (sig(&e, "motor.i_a"), sig(&e, "inverter.v_a"));
        assert!(
            ia.is_finite() && (-D.v_f - 1e-9..=V_BUS + D.v_f + 1e-9).contains(&va),
            "{va}"
        );
        neg |= ia < -1e-3;
        pos |= ia > 1e-3;
        let d = sig(&e, "motor.diode_a");
        if d != 0.0 && prev != 0.0 && d != prev {
            handovers += 1;
        }
        if d != 0.0 {
            prev = d;
        }
        // Each diode carries its own current direction only.
        if d == -1.0 {
            assert!(ia <= 1e-6, "high diode with i_a = {ia}");
        }
        if d == 1.0 {
            assert!(ia >= -1e-6, "low diode with i_a = {ia}");
        }
    }
    assert!(neg && pos, "both diodes conducted");
    assert!(handovers >= 4, "diode handovers: {handovers}");
}

/// P07 review #9: the `motor.open_phase` fault takes over a leg-driven float (same
/// phase) instead of failing with "already open"; turning the leg back on then does not
/// reconnect the faulted phase.
#[test]
fn open_phase_fault_overrides_a_leg_float() {
    use LegState::*;
    let mut e = rig(true, 0.0, [High, Low, Low]);
    e.step_until(SimTime::from_secs_f64(0.06)).unwrap();
    set_leg(&mut e, "a", Off);
    e.step_until(SimTime::from_secs_f64(0.08)).unwrap();
    assert_eq!(sig(&e, "motor.open_phase"), 0.0, "leg float");
    e.queue(EngineCommand::SetParam {
        path: "motor.open_phase".into(),
        value: 0.0,
        source: ChangeSource::Internal,
    });
    set_leg(&mut e, "a", High);
    e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    assert_eq!(sig(&e, "motor.open_phase"), 0.0, "the fault holds it open");
    assert_eq!(sig(&e, "motor.i_a"), 0.0);
}

fn warnings(e: &mut Engine) -> Vec<String> {
    e.drain_events()
        .into_iter()
        .filter_map(|ev| match ev {
            EngineEvent::Warning { msg, .. } => Some(msg),
            _ => None,
        })
        .collect()
}

fn open_phase(e: &mut Engine, k: f64) {
    e.queue(EngineCommand::SetParam {
        path: "motor.open_phase".into(),
        value: k,
        source: ChangeSource::Internal,
    });
}

/// One floating phase at a time (P08 review m1): a fault on phase a while phase b's leg
/// is Off is refused; a leg going Off while a fault holds another phase open warns.
#[test]
fn a_second_floating_phase_is_refused_or_warned() {
    use LegState::*;
    let mut e = rig(true, 0.0, [High, Low, Low]);
    e.step_until(SimTime::from_secs_f64(0.06)).unwrap();
    set_leg(&mut e, "b", Off);
    e.step_until(SimTime::from_secs_f64(0.08)).unwrap();
    assert_eq!(sig(&e, "motor.open_phase"), 1.0, "b floats on its leg");
    e.drain_events();
    open_phase(&mut e, 0.0);
    e.step_until(SimTime::from_secs_f64(0.081)).unwrap();
    let rejected = e.drain_events().into_iter().any(|ev| {
        matches!(ev, EngineEvent::CommandRejected { reason, .. } if reason.contains("zero-current"))
    });
    assert!(rejected, "fault on a while b's leg is Off");
    assert_eq!(sig(&e, "motor.open_phase"), 1.0);

    let mut e = rig(true, 0.0, [High, Low, Low]);
    e.step_until(SimTime::from_secs_f64(0.06)).unwrap();
    open_phase(&mut e, 0.0);
    e.step_until(SimTime::from_secs_f64(0.061)).unwrap();
    assert!(warnings(&mut e).is_empty());
    set_leg(&mut e, "c", Off);
    e.step_until(SimTime::from_secs_f64(0.062)).unwrap();
    let w = warnings(&mut e);
    assert!(
        w.iter().any(|m| m.contains("while another phase is open")),
        "{w:?}"
    );
}

/// The all-legs-Off warning threshold follows the dead time, 2·t_d (P08 review m4):
/// 15 µs with every leg Off is a transient at t_d = 10 µs, a commanded state at t_d = 0.
/// (Two legs Off with one driven is the modelled zero-current mode, EQ-MOT-12.)
#[test]
fn two_off_warning_threshold_follows_the_dead_time() {
    use LegState::*;
    for (dt, warns) in [(10e-6, false), (0.0, true)] {
        let mut e = rig_dt(false, 0.0, [High, Low, Low], dt);
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        e.drain_events();
        set_leg(&mut e, "a", Off);
        set_leg(&mut e, "b", Off);
        set_leg(&mut e, "c", Off);
        e.step_until(SimTime::from_secs_f64(0.01 + 15e-6)).unwrap();
        // A discrete event while all are still Off evaluates the warning.
        set_leg(&mut e, "a", Off);
        e.step_until(SimTime::from_secs_f64(0.01 + 16e-6)).unwrap();
        let w = warnings(&mut e);
        assert_eq!(
            w.iter().any(|m| m.contains("legs stay Off")),
            warns,
            "t_d {dt}: {w:?}"
        );
    }
}
