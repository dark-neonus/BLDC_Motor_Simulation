//! P09.T12: every controller preset on three motor presets for 1 s — no instability (the
//! tracking error envelope does not grow), tracking (the mean of the last 0.2 s near the
//! target), the energy balance closed at every sample, no over-current (≤ 1.5·limit, the
//! trip level protection uses), no bus over-voltage, open loop stays in sync.

use std::f64::consts::PI;

use sim_core::build::{build_engine, pmsm_from};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

const MOTORS: [&str; 3] = ["2804-outrunner", "5010-outrunner", "8108-outrunner"];

fn rated(motor: &str) -> f64 {
    let text = std::fs::read_to_string(format!(
        "{}/../../presets/motors/{motor}.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let line = text
        .lines()
        .find(|l| l.contains("current_continuous"))
        .unwrap();
    line.split('"')
        .nth(1)
        .unwrap()
        .trim_end_matches(" A")
        .parse()
        .unwrap()
}

/// The peak cogging torque [N·m]: the sum of the harmonic amplitudes.
fn cogging(motor: &str) -> f64 {
    let text = std::fs::read_to_string(format!(
        "{}/../../presets/motors/{motor}.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    text.lines()
        .filter(|l| l.trim_start().starts_with("cogging"))
        .flat_map(|l| {
            l.split('"')
                .skip(1)
                .step_by(2)
                .map(str::to_owned)
                .collect::<Vec<_>>()
        })
        .filter_map(|v| v.strip_suffix(" N*m").and_then(|x| x.parse::<f64>().ok()))
        .sum()
}

fn scene_for(preset: &str, motor: &str) -> (sim_model::scene::ResolvedScene, f64) {
    let dir = std::env::temp_dir().join(format!("bldc-cp-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let sensors = if preset.starts_with("six-step") {
        "sensors: { preset: \"builtin:sensors/hall-set\" }\n"
    } else if preset.starts_with("foc-") && preset != "foc-sensorless" || preset.starts_with("mit")
    {
        "sensors: { preset: \"builtin:sensors/as5047p\" }\n"
    } else {
        ""
    };
    let i_lim = rated(motor);
    // A light load (10 % of the rated torque): a drive always has one, and a
    // unidirectional six-step cannot slow an unloaded, frictionless rotor.
    let text = std::fs::read_to_string(format!(
        "{}/../../presets/motors/{motor}.yaml",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap();
    let dir0 = std::env::temp_dir().join(format!("bldc-cpm-{}", std::process::id()));
    let lib0 = Library::new(None, dir0.join("user"), vec![]).unwrap();
    let mp: sim_model::params::MotorParams = parse_yaml(&text, motor).unwrap();
    let _ = lib0;
    let mm = pmsm_from(&mp, 0.0).unwrap();
    // An MIT hold has no load: its stiffness K_p = ω²J comes from the bare rotor
    // (EQ-CTRL-12), so a Coulomb brake holds the rotor wherever K_p·e < τ_brake.
    // A unidirectional six-step slows down only through its load: with 10 % the 5010
    // overshoots, coasts and hunts ±1.5 % (P09 review: the diode braking that used to
    // damp this was non-physical, EQ-MOT-12), so it gets 30 %.
    let share = if preset.starts_with("mit") {
        0.0
    } else if preset.starts_with("six-step") {
        0.3
    } else {
        0.1
    };
    let tau = share * 1.5 * mm.p * mm.lambda * i_lim;
    let yaml = format!(
        "schema_version: 1\nname: s\nseed: 1\n\
         motor: {{ preset: \"builtin:motors/{motor}\" }}\n\
         inverter: {{ preset: \"builtin:inverters/medium-48v-40a\" }}\n\
         supply: {{ preset: \"builtin:supplies/lipo-6s\" }}\n\
         load: {{ kind: brake, torque: {tau} }}\n{sensors}\
         controller: {{ preset: \"builtin:controllers/{preset}\", overrides: {{ limits.current: {i_lim} A }} }}\n"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    let scene = sc.resolve(&lib).unwrap();
    (scene, i_lim)
}

fn engine(preset: &str, motor: &str) -> (sim_core::engine::engine::Engine, f64) {
    let (scene, i_lim) = scene_for(preset, motor);
    (build_engine(&scene).unwrap().engine, i_lim)
}

fn run(preset: &str, motor: &str) {
    let (scene, i_lim) = scene_for(preset, motor);
    let m = pmsm_from(&scene.motor, 0.0).unwrap();
    let v_bus = sim_core::build::supply_voltage(&scene.supply).unwrap();
    let w_nl = v_bus / (3f64.sqrt() * m.lambda * m.p);
    let mut e = build_engine(&scene)
        .unwrap_or_else(|err| panic!("{preset} × {motor}: {err}"))
        .engine;
    // The target and the signal it is judged on.
    let (path, target, judged) = if preset.starts_with("mit") {
        ("ctrl.theta_ref", 0.5, "motor.theta")
    } else if preset.starts_with("open-loop") {
        ("ctrl.omega_ref", 20.0, "motor.omega")
    } else if preset == "foc-sensorless" {
        ("ctrl.omega_ref", 0.3 * w_nl, "motor.omega")
    } else if preset.starts_with("six-step") {
        ("ctrl.omega_ref", 0.2 * w_nl, "motor.omega")
    } else {
        ("ctrl.omega_ref", 0.1 * w_nl, "motor.omega")
    };
    e.queue(EngineCommand::SetSignal {
        path: path.into(),
        value: target,
        source: ChangeSource::Internal,
    });
    let (mut early, mut late, mut peak_i) = (0.0f64, 0.0f64, 0.0f64);
    let (mut sum, mut count, mut worst_r) = (0.0, 0.0, 0.0f64);
    for k in 1..=1000 {
        let t = k as f64 * 1e-3;
        e.step_until(SimTime::from_secs_f64(t))
            .unwrap_or_else(|err| panic!("{preset} × {motor}: {err}"));
        let g = |n: &str| e.bus.get(e.bus.id(n).unwrap());
        let y = g(judged);
        assert!(y.is_finite(), "{preset} × {motor}: {judged} = {y}");
        let i = ["motor.i_a", "motor.i_b", "motor.i_c"]
            .map(g)
            .iter()
            .fold(0.0f64, |m, x| m.max(x.abs()));
        peak_i = peak_i.max(i);
        assert!(
            g("bus.v") < 1.2 * v_bus,
            "{preset} × {motor}: bus {} V",
            g("bus.v")
        );
        let r = g("energy.residual");
        worst_r = worst_r.max(r.abs());
        assert_eq!(
            g("energy.ok"),
            1.0,
            "{preset} × {motor}: energy residual {r:.2e} at {t:.3} s"
        );
        let err = (y - target).abs();
        if (0.4..0.6).contains(&t) {
            early = early.max(err);
        }
        if t >= 0.8 {
            late = late.max(err);
            sum += y;
            count += 1.0;
        }
    }
    assert!(
        peak_i <= 1.5 * i_lim,
        "{preset} × {motor}: peak |i| {peak_i:.2} A, limit {i_lim} A"
    );
    // No growth of the error envelope (an unstable loop grows; a settling one shrinks);
    // within 2 % of the target counts as settled (sensor ripple, cogging).
    assert!(
        late <= (1.1 * early).max(0.02 * target.abs().max(PI / 180.0)),
        "{preset} × {motor}: error grows {early} → {late}"
    );
    // Tracking: the mean over the last 0.2 s within 2 % of the target (1° for a hold).
    let mean = sum / count;
    let band = if judged == "motor.theta" {
        // A compliant hold stops where K_p·e meets the cogging torque (EQ-MOT-08).
        let kp = e.get_param("controller.kp").unwrap();
        let band = cogging(motor) / kp + PI / 180.0;
        assert!(
            band <= 0.5 * target,
            "{preset} × {motor}: K_p {kp:.3} N·m/rad cannot hold against cogging"
        );
        band
    } else {
        0.02 * target.abs()
    };
    assert!(
        (mean - target).abs() <= band,
        "{preset} × {motor}: mean {mean:.4} vs target {target:.4} (worst residual {worst_r:.1e})"
    );
    if preset.starts_with("open-loop") {
        let lost = e.bus.get(e.bus.id("ctrl.open_loop.sync_lost").unwrap());
        assert_eq!(lost, 0.0, "{preset} × {motor}: lost synchronism");
    }
}

/// Plain V/f has no damping beyond R: on the low-R 5010 it hunts and slips poles
/// (EQ-CTRL-06 limitation) — detected, with the current still bounded.
fn open_loop_on_low_r(preset: &str) {
    let motor = "5010-outrunner";
    let (mut e, i_lim) = engine(preset, motor);
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 20.0,
        source: ChangeSource::Internal,
    });
    let mut peak_i = 0.0f64;
    for k in 1..=1000 {
        e.step_until(SimTime::from_secs_f64(k as f64 * 1e-3))
            .unwrap();
        let g = |n: &str| e.bus.get(e.bus.id(n).unwrap());
        peak_i = peak_i.max(
            ["motor.i_a", "motor.i_b", "motor.i_c"]
                .map(g)
                .iter()
                .fold(0.0f64, |m, x| m.max(x.abs())),
        );
    }
    let lost = e.bus.get(e.bus.id("ctrl.open_loop.sync_lost").unwrap());
    assert_eq!(
        lost, 1.0,
        "{preset} × {motor}: the documented loss of synchronism"
    );
    assert!(
        peak_i <= 1.5 * i_lim,
        "{preset} × {motor}: peak |i| {peak_i:.2} A"
    );
}

macro_rules! presets {
    ($($name:ident: $preset:expr,)*) => {$(
        #[test]
        fn $name() {
            for motor in MOTORS {
                run($preset, motor);
            }
        }
    )*};
}

presets! {
    foc_soft: "foc-soft",
    foc_medium: "foc-medium",
    foc_stiff: "foc-stiff",
    foc_sensorless: "foc-sensorless",
    six_step_soft: "six-step-soft",
    six_step_medium: "six-step-medium",
    six_step_stiff: "six-step-stiff",

    mit_soft: "mit-soft",
    mit_medium: "mit-medium",
    mit_stiff: "mit-stiff",
    foc_default: "foc-default",
}

#[test]
fn open_loop_slow() {
    for motor in ["2804-outrunner", "8108-outrunner"] {
        run("open-loop-slow", motor);
    }
    open_loop_on_low_r("open-loop-slow");
}

#[test]
fn open_loop_fast() {
    for motor in ["2804-outrunner", "8108-outrunner"] {
        run("open-loop-fast", motor);
    }
    open_loop_on_low_r("open-loop-fast");
}
