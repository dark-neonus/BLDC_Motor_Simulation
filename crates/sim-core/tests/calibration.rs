//! P09.T09 / V-CTRL-007: encoder offset calibration (EQ-CTRL-11) recovers a random
//! mount offset within 1° electrical, finds p and the direction, and the motor then
//! runs on the calibrated angle.

use std::f64::consts::{PI, TAU};

use sim_core::build::build_engine;
use sim_core::control::calibration::{CalibrationOptions, calibrate};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

const P: f64 = 11.0;

/// 4108 (p = 11) without cogging, an ideal 16-bit magnetic encoder mounted at `offset`
/// degrees (mechanical), FOC with a velocity loop.
fn scene(offset_deg: f64) -> Engine {
    let dir = std::env::temp_dir().join(format!("bldc-cal-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\n\
         motor: {{ preset: \"builtin:motors/4108-outrunner\", overrides: {{ magnetic.cogging: [] }} }}\n\
         inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         sensors: {{ encoder: {{ kind: magnetic, bits: 16, mount_offset: {offset_deg} deg }} }}\n\
         controller: {{ family: foc, current: {{ rate: 20 kHz, bandwidth: 1 kHz }}, velocity: {{ rate: 5 kHz, bandwidth: 20 Hz }}, limits: {{ current: 4 A }} }}\n"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine
}

fn wrap(x: f64) -> f64 {
    let y = x.rem_euclid(TAU);
    if y > PI { y - TAU } else { y }
}

#[test]
fn v_ctrl_007_recovers_a_random_offset_within_1_deg() {
    // Fixed pseudo-random mount offsets (seeded, deterministic).
    let mut x: u64 = 0x9E37_79B9_7F4A_7C15;
    for _ in 0..2 {
        x = x
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let offset_deg = (x >> 11) as f64 / (1u64 << 53) as f64 * 360.0;
        let mut e = scene(offset_deg);
        let r = calibrate(&mut e, &CalibrationOptions::defaults(4.0, P)).unwrap();
        let want = (-P * offset_deg.to_radians()).rem_euclid(TAU);
        let err = wrap(r.offset - want).abs().to_degrees();
        assert!(
            err < 1.0,
            "mount {offset_deg:.2}°: offset error {err:.3}° electrical"
        );
        // p̂ identifies p after rounding (the d-axis swing leaves ≈ 0.5 %, EQ-CTRL-11).
        let p_hat = r.pole_pairs.unwrap();
        assert!((p_hat - P).abs() < 0.25 && p_hat.round() == P, "p̂ {p_hat}");
        assert_eq!(r.direction, Some(1.0));
        // The calibrated motor then runs on its speed loop.
        e.queue(EngineCommand::SetSignal {
            path: "ctrl.omega_ref".into(),
            value: 30.0,
            source: ChangeSource::Internal,
        });
        let t = e.time.as_secs_f64() + 1.0;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        let w = e.bus.get(e.bus.id("motor.omega").unwrap());
        assert!((w / 30.0 - 1.0).abs() < 0.02, "ω {w}");
    }
}

/// Without calibration the same offset leaves the controller's angle off by p·θ₀, and
/// the motor does not follow the speed command (the "wrong offset" P10 demonstrates).
#[test]
fn an_uncalibrated_offset_spoils_the_control() {
    let mut e = scene(9.0); // 9° mechanical × 11 = 99° electrical: past the q axis
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 30.0,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    let w = e.bus.get(e.bus.id("motor.omega").unwrap());
    assert!((w / 30.0 - 1.0).abs() > 0.2, "ω {w}");
}
