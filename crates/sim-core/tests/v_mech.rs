//! V-MECH catalog cases (validation-catalog.md) on the drivetrain module alone.

use std::f64::consts::PI;

use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::mech::backlash::Backlash;
use sim_core::physics::mech::friction::Friction;
use sim_core::physics::mech::gearbox::Gearbox;
use sim_core::physics::mech::load::Load;
use sim_core::physics::mech::load_arm::{Arm, G};
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};

/// The EQ-MECH-06 check case: m_a 0.1 kg, L_a 0.2 m, m 0.5 kg at 0.15 m, J_m 2.5e-4.
const ARM: Arm = Arm {
    arm_length: 0.2,
    arm_mass: 0.1,
    mass: 0.5,
    distance: 0.15,
};
const J_M: f64 = 2.5e-4;

fn drivetrain(theta_l0: f64, gb: Gearbox, load: Load, conserve: bool, dt: f64) -> Engine {
    let mut bus = SignalBus::new();
    let p = RotorParams {
        j: J_M,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: 7.0,
        theta0: theta_l0 * gb.n,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(p, &mut bus, vec![], vec![])
        .unwrap()
        .with_gearbox(gb)
        .with_load(load, conserve);
    let mut plant = Plant::new();
    plant.add(Box::new(rotor));
    Engine::new(plant, bus, vec![], dt)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

/// Times at which `load.theta` crosses zero downwards, linearly interpolated.
fn down_crossings(e: &mut Engine, t_end: f64, h: f64) -> Vec<f64> {
    let mut out = vec![];
    let (mut t, mut th) = (0.0, sig(e, "load.theta"));
    while t < t_end {
        let t1 = t + h;
        e.step_until(SimTime::from_secs_f64(t1)).unwrap();
        let th1 = sig(e, "load.theta");
        if th > 0.0 && th1 <= 0.0 {
            out.push(t + h * th / (th - th1));
        }
        (t, th) = (t1, th1);
    }
    out
}

/// V-MECH-001: small-angle period of the unpowered arm, T₀ = 2π√(J_tot/(g·M_eff)),
/// rtol 5e-4. The 0.01 rad amplitude adds θ²/16 ≈ 6e-6 to the period.
#[test]
fn v_mech_001_pendulum_period() {
    let j_tot = ARM.inertia() + J_M;
    let t0 = 2.0 * PI * (j_tot / (G * ARM.m_eff())).sqrt();
    assert!((t0 - 0.77961).abs() < 1e-5, "spec value: {t0}");
    let mut e = drivetrain(0.01, Gearbox::BYPASS, Load::Arm(ARM), false, 1e-4);
    let c = down_crossings(&mut e, 8.0 * t0, 1e-4);
    let period = (c[c.len() - 1] - c[0]) / (c.len() - 1) as f64;
    assert!((period / t0 - 1.0).abs() < 5e-4, "{period} vs {t0}");
}

/// V-MECH-002: a large swing (released at 150°) conserves energy, residual < 1e-6 over
/// 10 periods.
#[test]
fn v_mech_002_large_swing_energy() {
    let mut e = drivetrain(
        150f64.to_radians(),
        Gearbox::BYPASS,
        Load::Arm(ARM),
        false,
        1e-4,
    );
    let e0 = sig(&e, "energy.stored");
    e.step_until(SimTime::from_secs_f64(15.0)).unwrap();
    let r = sig(&e, "energy.residual");
    assert!(r.abs() < 1e-6, "{r}");
    // No loss terms: kinetic + potential stays at its start value.
    assert!(
        (sig(&e, "energy.stored") - e0).abs() < 1e-6 * ARM.potential(PI),
        "stored drifted"
    );
}

/// V-MECH-009: a live mass change through a gearbox keeps ω (`keep_speed`) or J·ω
/// (`conserve_momentum`), rtol 1e-9, with ΔE booked as external energy.
#[test]
fn v_mech_009_live_mass_change() {
    let gb = Gearbox {
        n: 9.0,
        eta: 1.0,
        j_in: 1e-6,
        j_out: 1e-4,
    };
    for conserve in [false, true] {
        let mut e = drivetrain(0.5, gb, Load::Arm(ARM), conserve, 1e-4);
        e.step_until(SimTime::from_secs_f64(0.2)).unwrap();
        let w0 = sig(&e, "load.omega");
        assert!(w0.abs() > 0.1, "swinging");
        let j0 = gb.j_eq(J_M, ARM.inertia());
        e.queue(EngineCommand::SetParam {
            path: "load.mass".into(),
            value: 1.0,
            source: ChangeSource::Internal,
        });
        e.apply_pending();
        let j1 = gb.j_eq(J_M, Arm { mass: 1.0, ..ARM }.inertia());
        let expect = if conserve { w0 * j0 / j1 } else { w0 };
        let w1 = sig(&e, "load.omega");
        assert!(
            (w1 / expect - 1.0).abs() < 1e-9,
            "conserve = {conserve}: {w1} vs {expect}"
        );
        e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
        assert!(sig(&e, "energy.residual").abs() < 1e-6);
        assert!(sig(&e, "energy.external").abs() > 1e-6, "ΔE is booked");
    }
}

/// V-MECH-008: an impulse H on a free system changes the load speed by H/J_tot,L
/// (J_tot,L = N²·J_eq), rtol 1e-6; the pulse's work is booked as external energy.
#[test]
fn v_mech_008_impulse() {
    let gb = Gearbox {
        n: 9.0,
        eta: 1.0,
        j_in: 1e-6,
        j_out: 1e-4,
    };
    let load = Load::Flywheel { j: 0.02 };
    let mut e = drivetrain(0.0, gb, load, false, 5e-6);
    e.queue(EngineCommand::Impulse {
        h: 0.004,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(0.0005)).unwrap();
    assert!(sig(&e, "load.disturbance") > 0.0, "pulse running");
    e.step_until(SimTime::from_secs_f64(0.003)).unwrap();
    assert_eq!(sig(&e, "load.disturbance"), 0.0, "pulse over after 1 ms");
    let j_tot_l = 81.0 * gb.j_eq(J_M, 0.02);
    let w = sig(&e, "load.omega");
    assert!((w / (0.004 / j_tot_l) - 1.0).abs() < 1e-6, "{w}");
    assert!(sig(&e, "energy.residual").abs() < 1e-9);
}

/// EQ-MECH-07: brake and viscous loads dissipate, a constant torque does external
/// work; the balance closes for each.
#[test]
fn basic_loads_book_their_energy() {
    for (load, w0) in [
        (Load::Brake { t0: 0.05 }, 20.0),
        (Load::Viscous { c: 0.002 }, 20.0),
        (Load::Constant { t0: 0.01 }, 0.0),
    ] {
        let mut bus = SignalBus::new();
        let p = RotorParams {
            j: J_M,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: w0,
        };
        let rotor = RotorRigid::new(p, &mut bus, vec![], vec![])
            .unwrap()
            .with_load(load.clone(), false);
        let mut plant = Plant::new();
        plant.add(Box::new(rotor));
        let mut e = Engine::new(plant, bus, vec![], 1e-4);
        e.step_until(SimTime::from_secs_f64(0.05)).unwrap();
        let w = sig(&e, "motor.omega");
        match load {
            // Brake: a constant deceleration T₀/J while turning.
            Load::Brake { .. } => assert!((w - (20.0 - 0.05 / J_M * 0.05)).abs() < 1e-6, "{w}"),
            Load::Viscous { .. } => {
                assert!(
                    (w / (20.0 * (-0.002 / J_M * 0.05f64).exp()) - 1.0).abs() < 1e-8,
                    "{w}"
                )
            }
            _ => assert!((w - 0.01 / J_M * 0.05).abs() < 1e-9, "{w}"),
        }
        assert!(sig(&e, "energy.residual").abs() < 1e-6, "{load:?}");
    }
}

const FR: Friction = Friction {
    t_s: 0.05,
    t_c: 0.03,
    b: 0.0,
};
const B: f64 = 1e-4;

/// Motor-side friction FR + viscous B, a constant load torque `t0` (N = 1).
fn friction_rig(t0: f64) -> Engine {
    let mut bus = SignalBus::new();
    let p = RotorParams {
        j: J_M,
        j_load: 0.0,
        b: B,
        pole_pairs: 7.0,
        theta0: 0.3,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(p, &mut bus, vec![], vec![])
        .unwrap()
        .with_load(Load::Constant { t0 }, false)
        .with_friction(FR, Friction::default());
    let mut plant = Plant::new();
    plant.add(Box::new(rotor));
    Engine::new(plant, bus, vec![], 1e-4)
}

/// V-MECH-006: 0.98·T_s on a stuck shaft leaves θ bit-identical.
#[test]
fn v_mech_006_stays_stuck_below_breakaway() {
    let mut e = friction_rig(0.98 * FR.t_s);
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    assert_eq!(sig(&e, "motor.theta"), 0.3);
    assert_eq!(sig(&e, "motor.omega"), 0.0);
}

/// V-MECH-007: 1.2·T_s breaks the shaft loose; after the torque is removed it coasts down
/// per Coulomb + viscous, ω(t) = (ω₀ + T_c/b)·e^{−bt/J} − T_c/b, and sticks again at
/// ω = 0. Energy closes (rtol 1e-3).
#[test]
fn v_mech_007_breakaway_coast_and_restick() {
    let mut e = friction_rig(1.2 * FR.t_s);
    e.step_until(SimTime::from_secs_f64(0.05)).unwrap();
    // Slipping under 1.2·T_s − T_c − b·ω from rest.
    let a = (1.2 * FR.t_s - FR.t_c) / J_M;
    let w_drive = a * J_M / B * (1.0 - (-B / J_M * 0.05f64).exp());
    let w0 = sig(&e, "motor.omega");
    assert!((w0 / w_drive - 1.0).abs() < 1e-3, "{w0} vs {w_drive}");
    e.queue(EngineCommand::SetParam {
        path: "load.torque".into(),
        value: 0.0,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(0.07)).unwrap();
    let tc_b = FR.t_c / B;
    let expect = (w0 + tc_b) * (-B / J_M * 0.02f64).exp() - tc_b;
    let w = sig(&e, "motor.omega");
    assert!((w / expect - 1.0).abs() < 1e-3, "{w} vs {expect}");
    // Stop time from the same law, then stuck for good.
    let t_stop = J_M / B * ((w0 + tc_b) / tc_b).ln();
    e.step_until(SimTime::from_secs_f64(0.05 + t_stop + 0.05))
        .unwrap();
    let th = sig(&e, "motor.theta");
    assert_eq!(sig(&e, "motor.omega"), 0.0);
    e.step_until(SimTime::from_secs_f64(0.05 + t_stop + 0.5))
        .unwrap();
    assert_eq!(sig(&e, "motor.theta"), th, "re-stuck");
    assert!(
        sig(&e, "energy.residual").abs() < 1e-3,
        "{}",
        sig(&e, "energy.residual")
    );
}

/// Motor-side PD drive tracking θm_ref = N·A·sin(2π·f·t) (writes `test.torque`).
struct Pd {
    out: SignalId,
    th: SignalId,
    w: SignalId,
    n: f64,
}

impl PlantModule for Pd {
    fn name(&self) -> &str {
        "test"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, t: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        let w = 2.0 * PI * 0.5;
        let (r, dr) = (
            self.n * 0.05 * (w * t).sin(),
            self.n * 0.05 * w * (w * t).cos(),
        );
        let u = 50.0 * (r - bus.get(self.th)) + 1.0 * (dr - bus.get(self.w));
        bus.set(self.out, u);
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

/// V-MECH-005: a slowly reversing motor with backlash b: the load lags by b (the
/// deflection swings between ±b/2 plus the contact penetration T_c/k), atol 1e-4 rad.
/// Contact loss is never negative and the balance closes; no NaN at dt_max.
#[test]
fn v_mech_005_backlash_lost_motion() {
    let gb = Gearbox {
        n: 9.0,
        eta: 1.0,
        j_in: 0.0,
        j_out: 0.0,
    };
    let bl = Backlash {
        width: 0.01,
        k: 3438.0,
        d: 2.0,
    };
    let fr_l = Friction {
        t_s: 0.02,
        t_c: 0.01,
        b: 0.0,
    };
    let mut bus = SignalBus::new();
    let tq = bus
        .register("test.torque", "N*m", "drive", SignalKind::Output)
        .unwrap();
    let p = RotorParams {
        j: J_M,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: 7.0,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(p, &mut bus, vec![], vec![tq])
        .unwrap()
        .with_gearbox(gb)
        .with_load(Load::Flywheel { j: 0.01 }, false)
        .with_friction(Friction::default(), fr_l)
        .with_backlash(bl);
    let pd = Pd {
        out: tq,
        th: bus.id("motor.theta").unwrap(),
        w: bus.id("motor.omega").unwrap(),
        n: 9.0,
    };
    let mut plant = Plant::new();
    plant.add(Box::new(rotor));
    plant.add(Box::new(pd));
    let mut e = Engine::new(plant, bus, vec![], 2e-5);
    // δ at the stroke mid-points (max speed, zero acceleration, contact settled): forward
    // at t = 2 s, backward at t = 1 s and 3 s. The re-engagement impact makes the peak δ
    // larger for a few ms, so peaks are not used.
    let at = |t: f64, e: &mut Engine| {
        let mut s = e.time.as_secs_f64();
        while s < t {
            s = (s + 1e-3).min(t);
            e.step_until(SimTime::from_secs_f64(s)).unwrap();
            assert!(sig(e, "gearbox.delta").is_finite() && sig(e, "load.omega").is_finite());
        }
        sig(e, "gearbox.delta")
    };
    let back1 = at(1.0, &mut e);
    let fwd = at(2.0, &mut e);
    let back2 = at(3.0, &mut e);
    let lost = bl.width + 2.0 * fr_l.t_c / bl.k;
    for back in [back1, back2] {
        assert!(((fwd - back) - lost).abs() < 1e-4, "{fwd} {back}");
    }
    assert!(sig(&e, "energy.loss.backlash") >= 0.0);
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}
