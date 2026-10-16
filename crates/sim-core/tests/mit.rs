//! P09.T07: MIT / impedance mode (EQ-CTRL-07) on the FOC current loop — static
//! deflection τ_L/K_p under a constant load (V-CTRL-003) and the damping ratio
//! ζ = K_d/(2√(K_p J)) from the step overshoot.

use std::f64::consts::PI;

use sim_core::build::build_engine;
use sim_core::control::mit::damping_ratio;
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

/// Flywheel 7.7 kg·cm² + 5010 rotor 476.847 g·cm², no gearbox (N = 1).
const J: f64 = 7.7e-4 + 476.847e-7;
const WN: f64 = 2.0 * PI * 5.0;
const ZETA: f64 = 0.3;

fn gains() -> (f64, f64) {
    let kp = WN * WN * J;
    (kp, 2.0 * ZETA * (kp * J).sqrt())
}

fn scene() -> Engine {
    let (kp, kd) = gains();
    let dir = std::env::temp_dir().join(format!("bldc-mit-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\n\
         motor: {{ preset: \"builtin:motors/5010-outrunner\" }}\n\
         load: {{ preset: \"builtin:loads/flywheel\" }}\n\
         inverter: {{ preset: \"builtin:inverters/medium-48v-40a\" }}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         controller: {{ family: mit, current: {{ rate: 20 kHz, bandwidth: 1 kHz }}, rate: 5 kHz, kp: {kp}, kd: {kd}, limits: {{ current: 20 A }} }}\n\
         fidelity: {{ tier: ideal, inverter_mode: ideal }}\n"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap()).unwrap().engine
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

/// V-CTRL-003: a constant 0.1 N·m on the load deflects the hold by τ_L/K_p (rtol 2 %).
#[test]
fn v_ctrl_003_static_deflection_is_load_over_kp() {
    let mut e = scene();
    let tau = 0.1;
    e.queue(EngineCommand::Disturbance {
        torque: tau,
        duration: 10.0,
        source: ChangeSource::Internal,
    });
    // ζω_n ≈ 9.4 /s: 2 s is ≈ 19 time constants.
    e.step_until(SimTime::from_secs_f64(2.0)).unwrap();
    let (kp, _) = gains();
    let defl = sig(&e, "motor.theta");
    assert!(
        (defl.abs() / (tau / kp) - 1.0).abs() < 0.02,
        "deflection {defl} rad vs τ/K_p = {}",
        tau / kp
    );
    assert!(sig(&e, "motor.omega").abs() < 1e-3);
}

/// A 0.1 rad step in θ*: the overshoot gives ζ = −ln(OS)/√(π² + ln² OS), which matches
/// the design K_d/(2√(K_p J)) within 5 % (current loop and 5 kHz sampling ≪ ω_n).
#[test]
fn damping_ratio_matches_the_second_order_formula() {
    let mut e = scene();
    let (kp, kd) = gains();
    assert!((damping_ratio(kp, kd, J) - ZETA).abs() < 1e-12);
    let step = 0.1;
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.theta_ref".into(),
        value: step,
        source: ChangeSource::Internal,
    });
    let mut peak: f64 = 0.0;
    for k in 1..=1500 {
        e.step_until(SimTime::from_secs_f64(k as f64 * 2e-4))
            .unwrap();
        peak = peak.max(sig(&e, "motor.theta"));
    }
    let os = peak / step - 1.0;
    let zeta = -os.ln() / (PI * PI + os.ln().powi(2)).sqrt();
    assert!(
        (zeta / ZETA - 1.0).abs() < 0.05,
        "ζ {zeta} (overshoot {os})"
    );
    // The stiffness is live (`controller.kp`, D-020).
    e.queue(EngineCommand::SetParam {
        path: "controller.kp".into(),
        value: 2.0 * kp,
        source: ChangeSource::Internal,
    });
    e.step_until(SimTime::from_secs_f64(0.31)).unwrap();
    assert_eq!(e.get_param("controller.kp"), Some(2.0 * kp));
}
