//! P09.T05: six-step with Halls (EQ-CTRL-05, EQ-INV-07) on a trapezoidal machine:
//! steady speed through the speed loop, flat torque between commutations, and the
//! floating phase's current decaying through a diode after each commutation.

use std::f64::consts::PI;

use sim_core::build::build_engine;
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

const TRAP: &str = "overrides: { magnetic.emf_shape: { kind: trapezoidal, flat_top: 120 deg } }";
/// The same with a flywheel-sized rotor (speed ripple negligible).
const TRAP_HEAVY: &str = "overrides: { magnetic.emf_shape: { kind: trapezoidal, flat_top: 120 deg }, mechanical.j_rotor: 7700 g*cm^2 }";

/// The scene with an optional load (YAML value of `load:`).
fn scene(extra: &str, load: Option<&str>) -> Engine {
    scene_with(TRAP, extra, load)
}

fn scene_with(motor_overrides: &str, extra: &str, load: Option<&str>) -> Engine {
    let dir = std::env::temp_dir().join(format!("bldc-six-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let load = load.map_or(String::new(), |l| format!("load: {l}\n"));
    let yaml = format!(
        "schema_version: 1\nname: s\nseed: 1\n\
         motor: {{ preset: \"builtin:motors/5010-outrunner\", {motor_overrides} }}\n{load}\
         inverter: {{ preset: \"builtin:inverters/medium-48v-40a\"}}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         sensors: {{ preset: \"builtin:sensors/hall-set\" }}\n\
         controller: {{ family: six_step, rate: 20 kHz, speed_loop: {{ rate: 1 kHz, bandwidth: 10 Hz }}, limits: {{ current: 5 A, speed: 3000 rpm }} }}\n{extra}"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn param(e: &mut Engine, path: &str, value: f64) {
    e.queue(EngineCommand::SetParam {
        path: path.into(),
        value,
        source: ChangeSource::Internal,
    });
}

fn signal(e: &mut Engine, path: &str, value: f64) {
    e.queue(EngineCommand::SetSignal {
        path: path.into(),
        value,
        source: ChangeSource::Internal,
    });
}

/// The six-step demo (Halls with hysteresis and latency, flywheel, averaged inverter)
/// with a trapezoidal motor reaches and holds a speed target through its speed PI, the
/// energy balance closed throughout.
#[test]
fn trapezoidal_motor_reaches_steady_speed() {
    let mut e = scene("", Some("{ preset: \"builtin:loads/flywheel\" }"));
    signal(&mut e, "ctrl.omega_ref", 150.0);
    for k in 1..=300 {
        e.step_until(SimTime::from_secs_f64(k as f64 * 0.01))
            .unwrap();
        // Energy closes through the discontinuous conduction near the target (EQ-MOT-12).
        assert_eq!(
            sig(&e, "energy.ok"),
            1.0,
            "t {}: residual {:.2e}",
            k as f64 * 0.01,
            sig(&e, "energy.residual")
        );
    }
    let w = sig(&e, "motor.omega");
    assert!((w / 150.0 - 1.0).abs() < 0.01, "ω {w} rad/s");
    assert!(sig(&e, "ctrl.six_step.sector") >= 0.0, "valid Hall sectors");
}

/// Fixed duty 0.045 against a 0.05 N·m brake, Ideal tier (ideal Halls, no cogging), a
/// flywheel-sized rotor: ω_e ≈ 100 rad/s, so a sector lasts ≈ 10 ms = 10·τ_e. The torque
/// between commutations is flat (EQ-CTRL-05: within 5 % of the mean outside ±20° of each
/// commutation at θe = 30° + k·60°), and after each commutation the phase turned Off
/// conducts through a diode (terminal at −V_f or V_bus + V_f) until its current reaches
/// zero, then floats.
#[test]
fn flat_torque_and_floating_phase_decay() {
    let mut e = scene_with(
        TRAP_HEAVY,
        "fidelity: { tier: ideal }\n",
        Some("{ kind: brake, torque: 0.05 N*m }"),
    );
    param(&mut e, "controller.speed_loop.enabled", 0.0);
    signal(&mut e, "ctrl.duty_ref", 0.045);
    let t0 = 4.0;
    e.step_until(SimTime::from_secs_f64(t0)).unwrap();
    let w0 = sig(&e, "motor.omega");
    assert!(w0 > 5.0, "spinning: {w0}");
    // One electrical period, sampled every 20 µs.
    let we = sig(&e, "motor.omega_e");
    let dt = 20e-6;
    let n = ((2.0 * PI / we) / dt) as usize + 1;
    let (v_f, v_bus) = (0.8, sig(&e, "bus.v"));
    let (mut tq, mut clamped, mut floated) = (vec![], 0usize, 0usize);
    let mut prev_open = sig(&e, "motor.open_phase");
    for k in 1..=n {
        e.step_until(SimTime::from_secs_f64(t0 + k as f64 * dt))
            .unwrap();
        let th = sig(&e, "motor.theta_e").rem_euclid(2.0 * PI).to_degrees();
        // Commutations at 30° + k·60°: keep the samples more than 20° away from them.
        let off_comm = ((th - 30.0).rem_euclid(60.0) - 30.0).abs() < 10.0;
        if off_comm {
            tq.push(sig(&e, "motor.torque_em"));
        }
        // The Off leg: diode-clamped while its current flows, then floating.
        for (x, leg) in ["a", "b", "c"].iter().enumerate() {
            if sig(&e, &format!("inverter.leg_{leg}")) != 0.0 {
                continue;
            }
            let i = sig(&e, &format!("motor.i_{leg}"));
            let v = sig(&e, &format!("inverter.v_{leg}"));
            if sig(&e, "motor.open_phase") == x as f64 {
                assert_eq!(i, 0.0);
            } else if i.abs() > 1e-3 {
                let at_clamp = (v + v_f).abs() < 0.05 || (v - v_bus - v_f).abs() < 0.05;
                assert!(at_clamp, "phase {leg}: i {i} A, v {v} V");
                clamped += 1;
            }
        }
        let open = sig(&e, "motor.open_phase");
        if open >= 0.0 && open != prev_open {
            floated += 1;
        }
        prev_open = open;
    }
    let mean = tq.iter().sum::<f64>() / tq.len() as f64;
    let span =
        tq.iter().cloned().fold(f64::MIN, f64::max) - tq.iter().cloned().fold(f64::MAX, f64::min);
    assert!(mean > 0.0);
    assert!(
        span / mean < 0.05,
        "ripple {:.2} % of {mean} N·m",
        100.0 * span / mean
    );
    assert!(clamped > 0, "diode conduction seen");
    assert!(
        floated >= 5,
        "a floating phase after each commutation: {floated}"
    );
}

/// Reverse: a negative command swaps High and Low and spins the motor backwards; an
/// ideal drive cannot float a phase, so six-step refuses it.
#[test]
fn negative_command_reverses_and_ideal_drive_is_refused() {
    let mut e = scene("fidelity: { tier: ideal }\n", None);
    param(&mut e, "controller.speed_loop.enabled", 0.0);
    signal(&mut e, "ctrl.duty_ref", -0.3);
    e.step_until(SimTime::from_secs_f64(0.5)).unwrap();
    assert!(
        sig(&e, "motor.omega") < -20.0,
        "ω {}",
        sig(&e, "motor.omega")
    );
    let dir = std::env::temp_dir().join(format!("bldc-six2-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\n\
         motor: {{ preset: \"builtin:motors/5010-outrunner\", {TRAP} }}\n\
         inverter: {{ preset: \"builtin:inverters/medium-48v-40a\"}}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         controller: {{ family: six_step, rate: 20 kHz, limits: {{ current: 5 A }} }}\n\
         fidelity: {{ inverter_mode: ideal }}\n"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    let err = build_engine(&sc.resolve(&lib).unwrap())
        .err()
        .expect("refused");
    assert!(
        err.to_string().contains("six-step needs an inverter"),
        "{err}"
    );
}

/// Switching mode: the PWM block turns the pattern into H/O pulses on the High leg (no
/// Low state on it), the Low leg stays L, and the motor spins up.
#[test]
fn switching_six_step_pulses_the_high_leg_and_spins() {
    let mut e = scene(
        "fidelity: { tier: ideal, inverter_mode: switching }\n",
        None,
    );
    param(&mut e, "controller.speed_loop.enabled", 0.0);
    signal(&mut e, "ctrl.duty_ref", 0.1);
    e.step_until(SimTime::from_secs_f64(0.05)).unwrap();
    let (mut seen_h, mut seen_o) = (false, false);
    for k in 0..200 {
        e.step_until(SimTime::from_secs_f64(0.05 + k as f64 * 1e-6))
            .unwrap();
        for x in ["a", "b", "c"] {
            let pat = sig(&e, &format!("inverter.pattern_{x}"));
            let leg = sig(&e, &format!("inverter.leg_{x}"));
            match pat as i64 {
                1 => {
                    assert_ne!(leg, -1.0, "the PWM'd High leg never goes Low");
                    seen_h |= leg == 1.0;
                    seen_o |= leg == 0.0;
                }
                -1 => assert_eq!(leg, -1.0, "the Low leg stays L"),
                _ => {}
            }
        }
    }
    assert!(seen_h && seen_o, "H/O pulses");
    assert!(sig(&e, "motor.omega") > 1.0, "ω {}", sig(&e, "motor.omega"));
}

/// The shipped six-step demo scene: spin up, then command a lower speed. The
/// unidirectional drive coasts (u = 0: High leg Off, Low leg on, the third Off — the
/// zero-current mode, EQ-MOT-12) and the frictionless flywheel keeps its speed; the
/// energy balance closes at every sample (EQ-ENER) and the PSU bus stays at its set
/// point (P09 review M2: the bus climbed to 67 V and the rotor sped up while coasting).
#[test]
fn six_step_demo_closes_the_energy_balance() {
    let dir = std::env::temp_dir().join(format!("bldc-sixd-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let text = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../presets/scenes/six-step-demo.yaml"
    ))
    .unwrap();
    let sc: Scene = parse_yaml(&text, "six-step-demo").unwrap();
    let mut e = build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine;
    let mut w_cmd_change = 0.0;
    for (t0, t1, w) in [(0.0, 2.0, 200.0), (2.0, 4.0, 60.0)] {
        signal(&mut e, "ctrl.omega_ref", w);
        for k in 1..=((t1 - t0) * 1e3) as usize {
            let t = t0 + k as f64 * 1e-3;
            e.step_until(SimTime::from_secs_f64(t)).unwrap();
            assert_eq!(
                sig(&e, "energy.ok"),
                1.0,
                "t {t:.3}: residual {:.2e}",
                sig(&e, "energy.residual")
            );
            assert!(
                sig(&e, "bus.v") < 24.5,
                "t {t:.3}: bus {} V",
                sig(&e, "bus.v")
            );
        }
        if t1 == 2.0 {
            w_cmd_change = sig(&e, "motor.omega");
        }
    }
    let w = sig(&e, "motor.omega");
    assert!(w > 50.0, "spun up: {w_cmd_change} rad/s at 2 s");
    assert!(
        w <= w_cmd_change * 1.001,
        "coasting cannot speed up: {w_cmd_change} → {w} rad/s"
    );
}
