//! P09 review M3: the shipped MIT, sensorless and open-loop scenes do what their
//! descriptions say, with the energy balance closed at every sample.

use std::f64::consts::FRAC_PI_2;

use sim_core::build::build_engine;
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

fn scene(name: &str) -> Engine {
    let dir = std::env::temp_dir().join(format!("bldc-ps-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let text = std::fs::read_to_string(format!(
        "{}/../../presets/scenes/{name}.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let sc: Scene = parse_yaml(&text, name).unwrap();
    build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn set(e: &mut Engine, path: &str, value: f64) {
    e.queue(EngineCommand::SetSignal {
        path: path.into(),
        value,
        source: ChangeSource::Internal,
    });
}

/// Runs to `t_end` in 1 ms samples, asserting energy.ok; returns the mean of `judged`
/// over the last `window` seconds.
fn run(e: &mut Engine, t_end: f64, judged: &str, window: f64) -> f64 {
    let (mut sum, mut n) = (0.0, 0.0);
    let steps = (t_end * 1e3).round() as usize;
    for k in 1..=steps {
        let t = k as f64 * 1e-3;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        assert_eq!(
            sig(e, "energy.ok"),
            1.0,
            "t {t:.3}: residual {:.2e}",
            sig(e, "energy.residual")
        );
        if t > t_end - window {
            sum += sig(e, judged);
            n += 1.0;
        }
    }
    sum / n
}

#[test]
fn mit_arm_holds_horizontal() {
    let mut e = scene("mit-arm-hold");
    set(&mut e, "ctrl.theta_ref", FRAC_PI_2);
    let th = run(&mut e, 2.0, "load.theta", 0.2);
    // Static deflection τ_L/K_p (EQ-CTRL-07): m·g·r = 2.94 N·m against K_p.
    let kp = e.get_param("controller.kp").unwrap();
    let defl = 2.0 * 9.81 * 0.15 / kp;
    assert!(
        (th - (FRAC_PI_2 - defl)).abs() < 1f64.to_radians(),
        "θ {:.2}° vs {:.2}° (K_p {kp:.0} N·m/rad)",
        th.to_degrees(),
        (FRAC_PI_2 - defl).to_degrees()
    );
}

#[test]
fn sensorless_spins_up_from_standstill() {
    let mut e = scene("sensorless-spin");
    set(&mut e, "ctrl.omega_ref", 30.0);
    let w = run(&mut e, 3.0, "motor.omega", 0.2);
    assert!((w / 30.0 - 1.0).abs() < 0.02, "ω {w} rad/s");
    assert_eq!(sig(&e, "ctrl.sensorless.state"), 3.0, "closed loop");
}

#[test]
fn open_loop_follows_the_rotating_field() {
    let mut e = scene("open-loop-spin");
    set(&mut e, "ctrl.omega_ref", 20.0);
    let w = run(&mut e, 2.0, "motor.omega", 0.2);
    assert!((w / 20.0 - 1.0).abs() < 0.02, "ω {w} rad/s");
    assert_eq!(sig(&e, "ctrl.open_loop.sync_lost"), 0.0, "in sync");
}
