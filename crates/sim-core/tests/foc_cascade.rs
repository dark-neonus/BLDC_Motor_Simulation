//! P09.T04: the FOC cascade (EQ-CTRL-04) — speed step against the design, arm hold at
//! 90° with the steady error the loop types predict, and the arm-servo preset scene.

use sim_core::build::{BuiltScene, build_engine as build_scene};
use sim_core::control::foc::FocSpec;
use sim_core::control::foc::position_loop::p_over_p_error;
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_core::skeleton::adapter::{SkeletonOptions, build_engine};
use sim_core::skeleton::params::PmsmParams;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;
use std::f64::consts::FRAC_PI_2;

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn set_signal(e: &mut Engine, path: &str, value: f64) {
    e.queue(EngineCommand::SetSignal {
        path: path.into(),
        value,
        source: ChangeSource::Internal,
    });
}

/// Ideal current loop + PI velocity design: ω/ω* = (ω_v s + a²)/(s + a)², a = ω_v/2,
/// step response y = 1 − e^{−at} + a·t·e^{−at} (peak 1 + e^{−2} at t = 2/a).
fn design_step(a: f64, t: f64) -> f64 {
    1.0 - (-a * t).exp() + a * t * (-a * t).exp()
}

/// (overshoot, 2 % settling time) of a sampled response to a unit step.
fn metrics(samples: &[(f64, f64)]) -> (f64, f64) {
    let peak = samples.iter().map(|s| s.1).fold(f64::MIN, f64::max);
    let settle = samples
        .iter()
        .filter(|s| (s.1 - 1.0).abs() > 0.02)
        .map(|s| s.0)
        .fold(0.0, f64::max);
    (peak - 1.0, settle)
}

/// Speed step on the skeleton motor (ideal drive, ω_c = 2π·1 kHz).
/// V-CTRL-002 (ω_v = ω_c/20): overshoot 13.5 % (atol 2 %), 2 % settling time within
/// 15 % of the design's; ≈ 0 with the prefilter. At the default ω_v = ω_c/10 the
/// current loop's lag gives the 15–16 % control.md states.
#[test]
fn v_ctrl_002_speed_step_settles_per_the_design() {
    let mp = PmsmParams::skeleton_6020();
    let wc = 2.0 * std::f64::consts::PI * 1000.0;
    let run = |sep: f64, prefilter: bool| {
        let wv = wc / sep;
        // Small step: K_p,v·Δω = 2 A stays inside the 5 A limit.
        let dw = 2.0 / (wv * mp.j / mp.kt());
        let mut spec = FocSpec {
            prefilter,
            ..FocSpec::skeleton()
        };
        if let Some(v) = spec.velocity.as_mut() {
            v.omega = Some(wv);
        }
        let mut e = build_engine(SkeletonOptions {
            controller: sim_core::control::ControllerSpec::Foc(spec),
            ..Default::default()
        });
        set_signal(&mut e, "ctrl.omega_ref", dw);
        let mut out = vec![];
        for k in 1..=2400 {
            let t = k as f64 * 50e-6;
            e.step_until(SimTime::from_secs_f64(t)).unwrap();
            out.push((t, sig(&e, "motor.omega") / dw));
        }
        out
    };
    let a = wc / 20.0 / 2.0;
    let design: Vec<(f64, f64)> = (1..=2400)
        .map(|k| (k as f64 * 50e-6, design_step(a, k as f64 * 50e-6)))
        .collect();
    let (os_d, ts_d) = metrics(&design);
    assert!(
        (os_d - (-2f64).exp()).abs() < 1e-3,
        "design overshoot {os_d}"
    );
    let (os, ts) = metrics(&run(20.0, false));
    assert!((os - 0.135).abs() < 0.02, "overshoot {os}");
    assert!(
        (ts / ts_d - 1.0).abs() < 0.15,
        "settling {ts} s vs design {ts_d} s"
    );
    let (os, _) = metrics(&run(20.0, true));
    assert!(os.abs() < 0.02, "prefiltered overshoot {os}");
    let (os, _) = metrics(&run(10.0, false));
    assert!((0.15..0.165).contains(&os), "ω_c/10: overshoot {os}");
}

fn scene(controller: &str, extra: &str) -> BuiltScene {
    let dir = std::env::temp_dir().join(format!("bldc-casc-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\nseed: 1\n\
         motor: {{ preset: \"builtin:motors/8108-outrunner\" }}\n\
         inverter: {{ preset: \"builtin:inverters/medium-48v-40a\" }}\n\
         supply: {{ preset: \"builtin:supplies/lipo-6s\" }}\n\
         load: {{ preset: \"builtin:loads/arm-15cm-2kg\" }}\n\
         controller: {controller}\n{extra}"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_scene(&sc.resolve(&lib).unwrap()).unwrap()
}

/// Arm at 90° under gravity on a rigid 1:9 gear (η = 1, Ideal tier: ideal feedback,
/// no cogging), starting at the target. Position P over velocity PI: zero steady error.
/// Position P over velocity P: θ* − θ_L = τ_L/(N² K_t K_p,v K_p,p) (EQ-CTRL-04, 2 %).
#[test]
fn arm_holds_90_deg_with_the_error_the_loop_types_predict() {
    let rigid = "gearbox: { name: rigid, ratio: 9, efficiency: 1 }\n\
                 fidelity: { tier: ideal, inverter_mode: ideal }\n\
                 initial: { theta: 90 deg }\n";
    let hold = |velocity: &str| {
        let ctrl = format!(
            "{{ family: foc, current: {{ rate: 40 kHz, bandwidth: 1.5 kHz }}, velocity: {velocity}, position: {{ rate: 1 kHz, bandwidth: 8 Hz }}, limits: {{ current: 20 A }} }}"
        );
        let mut e = scene(&ctrl, rigid).engine;
        e.step_until(SimTime::from_secs_f64(3.0)).unwrap();
        (
            FRAC_PI_2 - sig(&e, "load.theta"),
            sig(&e, "load.torque_gravity").abs(),
        )
    };
    let (err, _) = hold("{ rate: 8 kHz, bandwidth: 60 Hz }");
    assert!(err.abs() < 1e-6, "PI velocity: error {err} rad");
    let kp_v = 0.5;
    let (err, tau) = hold(&format!("{{ rate: 8 kHz, kp: {kp_v}, ki: 0 }}"));
    let kt = 1.5 * 21.0 * 2.6254e-3;
    let want = p_over_p_error(tau, 9.0, kt, kp_v, 2.0 * std::f64::consts::PI * 8.0);
    assert!(want > 1e-3, "a measurable error: {want} rad");
    assert!(
        (err / want - 1.0).abs() < 0.02,
        "P/P error {err} vs {want} rad"
    );
}

/// The arm-servo preset (encoder with INL, 0.3° backlash, compliant gear, η = 0.92,
/// averaged inverter): from hanging down, a 90° position target is reached and held.
/// The loop zeroes its own error (encoder angle / N); the true load angle is off by at
/// most the encoder's INL/N plus half the backlash plus the gear twist.
#[test]
fn arm_servo_preset_reaches_and_holds_90_deg() {
    let dir = std::env::temp_dir().join(format!("bldc-arm-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../presets/scenes/arm-servo.yaml"
    ))
    .unwrap();
    let sc: Scene = parse_yaml(&text, "arm-servo").unwrap();
    let mut e = build_scene(&sc.resolve(&lib).unwrap()).unwrap().engine;
    set_signal(&mut e, "ctrl.theta_ref", FRAC_PI_2);
    e.step_until(SimTime::from_secs_f64(3.0)).unwrap();
    let meas = sig(&e, "sensors.encoder.angle") / 9.0;
    assert!(
        (meas - FRAC_PI_2).abs() < 2e-4,
        "loop error {} rad",
        FRAC_PI_2 - meas
    );
    // INL 0.8° (motor) / 9 + backlash 0.3°/2, plus the twist τ/k with τ ≈ 3.1 N·m and
    // k = 1 N·m/arcmin.
    let twist = 3.1 / (1.0 * 60.0 * 180.0 / std::f64::consts::PI);
    let bound = (0.8f64 / 9.0 + 0.15).to_radians() + twist;
    let err = FRAC_PI_2 - sig(&e, "load.theta");
    assert!(err.abs() < bound, "load error {err} rad, bound {bound}");
    assert!(sig(&e, "load.omega").abs() < 1e-2, "at rest");
}

/// The live UI/MCP speed command (`EngineCommand::VelocityTarget`) switches a scene
/// with a position loop (arm-servo) to velocity mode, as a scenario velocity target does.
#[test]
fn velocity_target_switches_off_the_position_loop() {
    let dir = std::env::temp_dir().join(format!("bldc-vt-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../presets/scenes/arm-servo.yaml"
    ))
    .unwrap();
    let sc: Scene = parse_yaml(&text, "arm-servo").unwrap();
    let mut e = build_scene(&sc.resolve(&lib).unwrap()).unwrap().engine;
    assert_eq!(e.get_param("controller.position.enabled"), Some(1.0));
    e.queue(EngineCommand::VelocityTarget {
        value: 2.0,
        source: ChangeSource::Ui,
    });
    e.step_until(SimTime::from_secs_f64(1.5)).unwrap();
    assert_eq!(e.get_param("controller.position.enabled"), Some(0.0));
    let w = sig(&e, "load.omega");
    assert!((w / 2.0 - 1.0).abs() < 0.1, "load ω {w}");
}
