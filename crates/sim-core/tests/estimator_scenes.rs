//! P09.T15: estimators built from a scene (EQ-CTRL-08/09) — the encoder's PLL speed
//! feeds the loops, a flux observer converges in a running scene, an unstable observer
//! gain is refused, and six-step's speed loop uses the Hall-interval speed.

use std::f64::consts::{PI, TAU};

use sim_core::build::{BuildError, BuiltScene, build_engine};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

fn build(body: &str) -> Result<BuiltScene, BuildError> {
    let dir = std::env::temp_dir().join(format!("bldc-est-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!("schema_version: 1\nname: s\nseed: 3\n{body}");
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap())
}

const FOC: &str = "motor: { preset: \"builtin:motors/4108-outrunner\" }\n\
    inverter: { preset: \"builtin:inverters/small-24v-10a\" }\n\
    supply: { preset: \"builtin:supplies/psu-24v-5a\" }\n\
    controller: { family: foc, current: { rate: 20 kHz, bandwidth: 1 kHz }, velocity: { rate: 5 kHz, bandwidth: 20 Hz }, limits: { current: 5 A } }\n";

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn speed(e: &mut Engine, w: f64) {
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: w,
        source: ChangeSource::Internal,
    });
}

/// An encoder scene: the PLL on the encoder angle publishes `est.omega` (mechanical),
/// which the velocity loop regulates to the setpoint.
#[test]
fn encoder_scene_regulates_on_the_pll_speed() {
    let mut e = build(&format!(
        "{FOC}sensors: {{ preset: \"builtin:sensors/as5047p\" }}\n"
    ))
    .unwrap()
    .engine;
    speed(&mut e, 40.0);
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    let (mut est, mut tru) = (0.0, 0.0);
    for k in 1..=200 {
        e.step_until(SimTime::from_secs_f64(1.0 + k as f64 * 1e-3))
            .unwrap();
        est += sig(&e, "est.omega") / 200.0;
        tru += sig(&e, "motor.omega") / 200.0;
    }
    assert!((est / 40.0 - 1.0).abs() < 0.01, "est.omega {est}");
    assert!((tru / 40.0 - 1.0).abs() < 0.01, "motor.omega {tru}");
}

/// `sensors.observer` in a FOC scene (ideal feedback for the control): the observer on
/// the currents and the applied voltage converges — angle error < 5° electrical and
/// `est.valid` set at speed (EQ-CTRL-09).
#[test]
fn observer_converges_in_a_running_scene() {
    let mut e = build(&format!("{FOC}sensors: {{ observer: {{}} }}\n"))
        .unwrap()
        .engine;
    speed(&mut e, 60.0);
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    let mut worst: f64 = 0.0;
    for k in 1..=100 {
        e.step_until(SimTime::from_secs_f64(1.0 + k as f64 * 1e-3))
            .unwrap();
        let err = (sig(&e, "est.theta") - sig(&e, "motor.theta_e")).rem_euclid(TAU);
        worst = worst.max(err.min(TAU - err));
        assert_eq!(sig(&e, "est.valid"), 1.0);
    }
    assert!(
        worst < 5f64.to_radians(),
        "angle error {}°",
        worst * 180.0 / PI
    );
    // Speed: est.omega is electrical (p = 11 for the 4108).
    assert!((sig(&e, "est.omega") / (11.0 * sig(&e, "motor.omega")) - 1.0).abs() < 0.05);
}

/// γλ²T ≥ 2 makes the observer's forward-Euler step unstable: refused at build.
#[test]
fn unstable_observer_gain_is_refused() {
    // 4108: λ = 18.56 mWb; T = 50 µs → unstable from γ = 2/(λ²T) ≈ 1.2e8.
    let err = build(&format!(
        "{FOC}sensors: {{ observer: {{ gamma: 1e10 }} }}\n"
    ))
    .err()
    .expect("refused");
    assert!(err.to_string().contains("γλ²T"), "{err}");
}

/// Six-step on Halls regulates on the Hall-interval speed `ctrl.six_step.omega`, which
/// tracks the true speed at steady state.
#[test]
fn six_step_speed_loop_uses_the_hall_speed() {
    let dir = std::env::temp_dir().join(format!("bldc-est6-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../presets/scenes/six-step-demo.yaml"
    ))
    .unwrap();
    let sc: Scene = parse_yaml(&text, "six-step-demo").unwrap();
    let mut e = build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine;
    speed(&mut e, 100.0);
    e.step_until(SimTime::from_secs_f64(3.0)).unwrap();
    let (mut hall, mut tru) = (0.0, 0.0);
    for k in 1..=200 {
        e.step_until(SimTime::from_secs_f64(3.0 + k as f64 * 1e-3))
            .unwrap();
        hall += sig(&e, "ctrl.six_step.omega") / 200.0;
        tru += sig(&e, "motor.omega") / 200.0;
    }
    assert!((tru / 100.0 - 1.0).abs() < 0.02, "motor.omega {tru}");
    assert!(
        (hall / tru - 1.0).abs() < 0.02,
        "Hall speed {hall} vs {tru}"
    );
}
