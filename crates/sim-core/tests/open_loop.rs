//! P09.T06: open-loop V/f (EQ-CTRL-06) — follows the commanded speed at a light load,
//! loses synchronism under a load step above pull-out (V-CTRL-009), with the flag and a
//! warning.

use sim_core::build::build_engine;
use sim_core::engine::commands::{ChangeSource, EngineCommand, EngineEvent};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

/// 5010 (λ = 1.27 mWb, p = 14) on the ideal 24 V source, a light brake; V/f just above
/// the back-EMF line: k_vf = 1.05·λ, V_0 = 0.3 V.
fn scene() -> Engine {
    let dir = std::env::temp_dir().join(format!("bldc-ol-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = "schema_version: 1\nname: s\n\
         motor: { preset: \"builtin:motors/5010-outrunner\" }\n\
         load: { kind: brake, torque: 0.02 N*m }\n\
         inverter: { preset: \"builtin:inverters/medium-48v-40a\" }\n\
         supply: { preset: \"builtin:supplies/psu-24v-5a\" }\n\
         controller: { family: open_loop, rate: 10 kHz, v0: 0.3 V, k_vf: 0.001334, accel: 100 rad/s^2, limits: { current: 5 A } }\n\
         fidelity: { tier: ideal, inverter_mode: ideal }\n";
    let sc: Scene = parse_yaml(yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

/// Mean motor speed over [t0, t1] (1 ms samples).
fn mean_speed(e: &mut Engine, t0: f64, t1: f64) -> f64 {
    let n = ((t1 - t0) / 1e-3) as usize;
    let mut sum = 0.0;
    for k in 1..=n {
        e.step_until(SimTime::from_secs_f64(t0 + k as f64 * 1e-3))
            .unwrap();
        sum += sig(e, "motor.omega");
    }
    sum / n as f64
}

fn start(e: &mut Engine, w: f64) {
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: w,
        source: ChangeSource::Internal,
    });
}

#[test]
fn follows_the_commanded_speed_at_light_load() {
    let mut e = scene();
    start(&mut e, 50.0);
    // Ramp 0.5 s at 100 rad/s², then settle.
    e.step_until(SimTime::from_secs_f64(1.5)).unwrap();
    let w = mean_speed(&mut e, 1.5, 2.0);
    assert_eq!(sig(&e, "ctrl.open_loop.sync_lost"), 0.0, "in sync");
    // Synchronous: the mean speed is the commanded one (any hunting averages out).
    assert!((w / 50.0 - 1.0).abs() < 0.01, "mean ω {w}");
    assert!((sig(&e, "ctrl.open_loop.omega_ref") - 50.0 * 14.0).abs() < 1e-9);
}

/// V-CTRL-009: a 3 N·m load step, above the ≈ 2 N·m pull-out at this V/f point, slips
/// poles: the flag latches, a warning is raised, and the rotor no longer follows.
#[test]
fn v_ctrl_009_load_step_above_pull_out_loses_sync() {
    let mut e = scene();
    start(&mut e, 50.0);
    e.step_until(SimTime::from_secs_f64(1.5)).unwrap();
    e.drain_events();
    e.queue(EngineCommand::Disturbance {
        torque: -3.0,
        duration: 0.3,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(1.8)).unwrap();
    assert_eq!(sig(&e, "ctrl.open_loop.sync_lost"), 1.0);
    let warned = e.drain_events().into_iter().any(
        |ev| matches!(ev, EngineEvent::Warning { msg, .. } if msg.contains("lost synchronism")),
    );
    assert!(warned, "warning raised");
    let w = mean_speed(&mut e, 1.8, 2.0);
    assert!((w / 50.0 - 1.0).abs() > 0.1, "slipping: mean ω {w}");
}
