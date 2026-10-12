//! V-MECH catalog cases (validation-catalog.md) on the drivetrain module alone.

use std::f64::consts::PI;

use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::Plant;
use sim_core::engine::signals::SignalBus;
use sim_core::engine::time::SimTime;
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
