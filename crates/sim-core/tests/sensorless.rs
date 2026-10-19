//! P09.T08: sensorless FOC (EQ-CTRL-10) — from standstill through align, I/f, blend and
//! closed loop on the flux observer to the speed target; the hand-off keeps i_q*
//! continuous and the measured current below the limit.

use sim_core::build::build_engine;
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

const I_LIM: f64 = 4.0;

fn scene() -> Engine {
    try_scene("").unwrap()
}

fn try_scene(extra: &str) -> Result<Engine, String> {
    let dir = std::env::temp_dir().join(format!("bldc-sl-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\n\
         motor: {{ preset: \"builtin:motors/4108-outrunner\" }}\n\
         inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         controller: {{ family: foc, sensorless: true, current: {{ rate: 20 kHz, bandwidth: 1 kHz }}, velocity: {{ rate: 20 kHz, bandwidth: 10 Hz }}, limits: {{ current: {I_LIM} A{extra} }} }}\n"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap())
        .map(|b| b.engine)
        .map_err(|e| e.to_string())
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

#[test]
fn starts_from_standstill_and_hands_off_smoothly() {
    let mut e = scene();
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 30.0,
        source: ChangeSource::Internal,
    });
    let dt = 50e-6;
    let (mut states, mut peak_i, mut prev_iq, mut max_jump) = (vec![], 0.0f64, 0.0, 0.0f64);
    let mut last_state = -1.0;
    for k in 1..=(3.0 / dt) as usize {
        e.step_until(SimTime::from_secs_f64(k as f64 * dt)).unwrap();
        let st = sig(&e, "ctrl.sensorless.state");
        if st != last_state {
            states.push(st);
            last_state = st;
        }
        let i = sig(&e, "motor.i_d").hypot(sig(&e, "motor.i_q"));
        peak_i = peak_i.max(i);
        let iq = sig(&e, "ctrl.foc.iq_ref");
        // The jump across the blend → closed-loop switch (state 2 → 3).
        if states.ends_with(&[2.0, 3.0]) && max_jump == 0.0 {
            max_jump = (iq - prev_iq).abs().max(1e-12);
        }
        prev_iq = iq;
    }
    assert_eq!(
        states,
        [0.0, 1.0, 2.0, 3.0],
        "align → I/f → blend → closed loop"
    );
    let w = sig(&e, "motor.omega");
    assert!((w / 30.0 - 1.0).abs() < 0.02, "ω {w} rad/s");
    assert!(peak_i < I_LIM, "peak current {peak_i} A");
    // Bumpless: i_q* moves by at most one PI step at the switch (< 5 % of I_if = 2 A).
    assert!(max_jump < 0.1, "i_q* jump {max_jump} A");
    // Fallback: commanded to stop, the speed falls below ω_min, `est.valid` drops, and the
    // supervisor returns to the I/f ramp (EQ-CTRL-10 step 4).
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 0.0,
        source: ChangeSource::Internal,
    });
    let mut fell_back = false;
    for k in 1..=(1.5 / dt) as usize {
        e.step_until(SimTime::from_secs_f64(3.0 + k as f64 * dt))
            .unwrap();
        fell_back |= sig(&e, "ctrl.sensorless.state") == 1.0;
    }
    assert!(fell_back, "fallback to I/f");
}

#[test]
fn speed_limit_and_ramp_apply() {
    // limits.speed 20 rad/s, ramp 20 rad/s²: a 30 rad/s command settles at 20 and gets
    // there no sooner than 1 s.
    let mut e = try_scene(", speed: 20 rad/s, ramp: 20 rad/s^2").unwrap();
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 30.0,
        source: ChangeSource::Internal,
    });
    let mut at_half = 0.0;
    for k in 1..=3000 {
        e.step_until(SimTime::from_secs_f64(k as f64 * 1e-3))
            .unwrap();
        if k == 500 {
            at_half = sig(&e, "motor.omega");
        }
    }
    assert!(at_half < 0.5 * 20.0 * 1.1, "ramped: ω(0.5 s) = {at_half}");
    let w = sig(&e, "motor.omega");
    assert!((w / 20.0 - 1.0).abs() < 0.02, "limited: ω {w} rad/s");
}

#[test]
fn position_with_sensorless_is_refused() {
    let dir = std::env::temp_dir().join(format!("bldc-slp-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = "schema_version: 1\nname: s\n\
         motor: { preset: \"builtin:motors/4108-outrunner\" }\n\
         inverter: { preset: \"builtin:inverters/small-24v-10a\" }\n\
         supply: { preset: \"builtin:supplies/psu-24v-5a\" }\n\
         controller: { family: foc, sensorless: true, current: { rate: 20 kHz }, velocity: { rate: 20 kHz }, position: { rate: 2 kHz }, limits: { current: 4 A } }\n";
    let sc: Scene = parse_yaml(yaml, "s").unwrap();
    let err = build_engine(&sc.resolve(&lib).unwrap())
        .err()
        .expect("refused");
    assert!(err.to_string().contains("sensorless"), "{err}");
}
