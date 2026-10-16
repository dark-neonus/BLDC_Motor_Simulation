//! P09.T11 / V-CTRL-008: auto-tune (EQ-CTRL-12) on every motor preset — R, L, J, λ
//! within 5 % of the truth, the tuned current loop within ±20 % of its bandwidth, and the
//! tuned loops stable over 1 s.

use sim_core::build::{build_engine, pmsm_from};
use sim_core::control::autotune::{Autotune, apply, autotune};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::{ResolvedScene, Scene};

/// The medium inverter's switch on-resistance [Ω] (in series with every phase).
const R_ON: f64 = 3e-3;
/// Copper α [1/K] and the reference temperature of R [K] (EQ-THERM-03 defaults).
const ALPHA_CU: f64 = 0.00393;
const T_REF: f64 = 293.15;

/// The preset's continuous current rating [A].
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

/// The motor on the 48 V medium inverter, current limit = its continuous rating.
fn scene(motor: &str) -> ResolvedScene {
    let dir = std::env::temp_dir().join(format!("bldc-at-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\n\
         motor: {{ preset: \"builtin:motors/{motor}\" }}\n\
         inverter: {{ preset: \"builtin:inverters/medium-48v-40a\" }}\n\
         supply: {{ preset: \"builtin:supplies/psu-48v-10a\" }}\n\
         controller: {{ family: foc, current: {{ rate: 20 kHz }}, velocity: {{ rate: 5 kHz }}, limits: {{ current: {} A }} }}\n",
        rated(motor)
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    sc.resolve(&lib).unwrap()
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn command(e: &mut Engine, path: &str, value: f64, param: bool) {
    e.queue(if param {
        EngineCommand::SetParam {
            path: path.into(),
            value,
            source: ChangeSource::Internal,
        }
    } else {
        EngineCommand::SetSignal {
            path: path.into(),
            value,
            source: ChangeSource::Internal,
        }
    });
}

fn check(motor: &str) {
    let s = scene(motor);
    let truth = pmsm_from(&s.motor, 0.0).unwrap();
    let a: Autotune = autotune(&s).unwrap_or_else(|e| panic!("{motor}: {e}"));
    let m = a.measured;
    // R as the current loop sees it: the winding at its temperature plus R_on.
    let t = m.t_winding.expect("thermal model in the Standard tier");
    let r_true = truth.r * (1.0 + ALPHA_CU * (t - T_REF)) + R_ON;
    for (name, got, want) in [
        ("R", m.r, r_true),
        ("L", m.l, truth.ld),
        ("J", m.j, truth.j),
        ("λ", m.lambda, truth.lambda),
    ] {
        assert!(
            (got / want - 1.0).abs() < 0.05,
            "{motor}: {name} {got:.4e} vs {want:.4e}"
        );
    }
    // Tuned current loop, locked rotor: a small-signal q step from 25 % to 50 % of the
    // rating (same sign throughout, so the dead-time offset stays constant), timed from
    // the moment ctrl.foc.iq_ref changes: τ63 ≈ 1/ω_c (±20 %).
    let tuned = apply(&s, &a);
    let mut e = build_engine(&tuned).unwrap().engine;
    command(&mut e, "motor.mechanical.locked", 1.0, true);
    command(&mut e, "controller.velocity.enabled", 0.0, true);
    let (i1, i2) = (0.25 * rated(motor), 0.5 * rated(motor));
    let kt = 1.5 * truth.p * truth.lambda;
    command(&mut e, "ctrl.torque_ref", i1 * kt, false);
    e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
    command(&mut e, "ctrl.torque_ref", i2 * kt, false);
    let (mut t_ref, mut t63) = (None, None);
    for k in 1..=200_000 {
        let t = 0.02 + k as f64 * 1e-7;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        if t_ref.is_none() && sig(&e, "ctrl.foc.iq_ref") > 0.5 * (i1 + i2) {
            t_ref = Some(t);
        }
        if let Some(t_ref) = t_ref
            && sig(&e, "motor.i_q") >= i1 + 0.632 * (i2 - i1)
        {
            t63 = Some(t - t_ref);
            break;
        }
    }
    let t63 = t63.unwrap_or_else(|| panic!("{motor}: no 63 % point"));
    let want = 1.0 / a.gains.omega_c;
    assert!(
        (t63 / want - 1.0).abs() < 0.2,
        "{motor}: τ63 {:.1} µs vs {:.1} µs",
        t63 * 1e6,
        want * 1e6
    );
    // Tuned velocity loop: a step to 10 % of the no-load speed. Stable: the error
    // envelope (cogging ripple included) does not grow from [0.2, 0.4] s to [0.8, 1] s
    // (≤ 10 %), and the mean speed has settled on the target (2 %).
    let mut e = build_engine(&tuned).unwrap().engine;
    let w_nl = 48.0 / (3f64.sqrt() * truth.lambda * truth.p);
    let target = 0.1 * w_nl;
    command(&mut e, "ctrl.omega_ref", target, false);
    let (mut early, mut late, mut mean, mut n) = (0.0f64, 0.0f64, 0.0, 0);
    for k in 1..=1000 {
        let t = k as f64 * 1e-3;
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        let w = sig(&e, "motor.omega");
        let err = (w - target).abs();
        if (0.2..0.4).contains(&t) {
            early = early.max(err);
        }
        if t >= 0.8 {
            late = late.max(err);
            mean += w;
            n += 1;
        }
    }
    let mean = mean / f64::from(n);
    assert!(
        late <= 1.1 * early + 1e-9,
        "{motor}: error grows {early} → {late}"
    );
    assert!(
        (mean / target - 1.0).abs() < 0.02,
        "{motor}: mean {mean} vs {target}"
    );
}

macro_rules! motors {
    ($($name:ident: $motor:expr,)*) => {$(
        #[test]
        fn $name() {
            check($motor);
        }
    )*};
}

motors! {
    v_ctrl_008_2804: "2804-outrunner",
    v_ctrl_008_4108: "4108-outrunner",
    v_ctrl_008_5010: "5010-outrunner",
    v_ctrl_008_6010: "6010-outrunner",
    v_ctrl_008_6020: "6020-outrunner",
    v_ctrl_008_8010: "8010-outrunner",
    v_ctrl_008_8108: "8108-outrunner",
    v_ctrl_008_10015: "10015-outrunner",
    v_ctrl_008_12020: "12020-outrunner",
}
