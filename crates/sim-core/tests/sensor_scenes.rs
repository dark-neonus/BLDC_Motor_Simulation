//! P08.T11: sensors are built from a scene (EQ-SENS-*): presets publish their signals,
//! bypass reads the true value at every controller step, sampling follows the rules of
//! `build::sensors` (ADC at `pwm.center` when switching).

use sim_core::build::{BuildError, BuiltScene, build_engine};
use sim_core::engine::engine::Engine;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::scene::Scene;

/// 20 kHz controller (foc-default) → 50 µs.
const CTRL_NS: i64 = 50_000;

fn build(sensors: &str, mode: &str) -> Result<BuiltScene, BuildError> {
    let dir = std::env::temp_dir().join(format!("bldc-sens-{}", std::process::id()));
    let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
    let yaml = format!(
        "schema_version: 1\nname: s\nseed: 7\n\
         motor: {{ preset: \"builtin:motors/4108-outrunner\" }}\n\
         inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n\
         supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n\
         controller: {{ preset: \"builtin:controllers/foc-default\" }}\n\
         fidelity: {{ inverter_mode: {mode} }}\n\
         sensors: {sensors}\n"
    );
    let sc: Scene = parse_yaml(&yaml, "s").unwrap();
    build_engine(&sc.resolve(&lib).unwrap())
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus
        .get(e.bus.id(n).unwrap_or_else(|_| panic!("no signal {n}")))
}

/// (period, phase) of the block `id`.
fn sampling(e: &Engine, id: &str) -> (SimTime, SimTime) {
    let b = e.blocks().iter().find(|b| b.id() == id).unwrap();
    (b.period().unwrap(), b.phase())
}

#[test]
fn every_sensor_preset_builds_and_publishes() {
    for (preset, signals) in [
        ("as5047p", &["sensors.encoder.angle"][..]),
        ("as5600", &["sensors.encoder.angle"]),
        ("mt6701", &["sensors.encoder.angle"]),
        (
            "optical-1000cpr",
            &["sensors.encoder.angle", "sensors.encoder.count"],
        ),
        ("hall-set", &["sensors.hall.code", "sensors.hall.sector"]),
        ("shunt-adc-12bit", &["sensors.adc.i_a", "sensors.adc.i_c"]),
    ] {
        let mut e = build(
            &format!("{{ preset: \"builtin:sensors/{preset}\" }}"),
            "averaged",
        )
        .unwrap_or_else(|err| panic!("{preset}: {err}"))
        .engine;
        e.step_until(SimTime::from_secs_f64(2e-3)).unwrap();
        for s in signals {
            assert!(sig(&e, s).is_finite(), "{preset}: {s}");
        }
    }
    // The Hall set at standstill (θe = 0) reads a valid sector.
    let mut e = build("{ preset: \"builtin:sensors/hall-set\" }", "averaged")
        .unwrap()
        .engine;
    e.step_until(SimTime::from_secs_f64(1e-3)).unwrap();
    assert!((1.0..=6.0).contains(&sig(&e, "sensors.hall.sector")));
}

/// EQ-SENS-01 bypass: whatever `update_rate` says, a bypassed sensor samples at the
/// controller period, so at each controller instant the reading is the true value.
#[test]
fn bypass_reads_the_true_value_at_every_controller_step() {
    let sensors = "\n  encoder: { kind: magnetic, bits: 12, inl: 1 deg, noise: 0.1 deg, update_rate: 1 kHz, bypass: true }\
                   \n  adc: { bits: 12, full_scale: 20 A, shunts: 2, offset: 0.3 A, noise: 50 mA, latency: 10 us, bypass: true }\
                   \n  hall: { offsets: [5 deg, 0 deg, 0 deg], update_rate: 1 kHz, bypass: true }";
    let mut e = build(sensors, "ideal").unwrap().engine;
    for id in ["sensors.encoder", "sensors.adc", "sensors.hall"] {
        assert_eq!(
            sampling(&e, id),
            (SimTime::from_nanos(CTRL_NS), SimTime::from_nanos(0)),
            "{id}"
        );
    }
    // Spin the rotor up through the FOC so the angle and the currents move.
    e.queue(sim_core::engine::commands::EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 50.0,
        source: sim_core::engine::commands::ChangeSource::Internal,
    });
    for k in [1i64, 7, 13, 401, 1999] {
        // The true values at the controller instant, then the events at it (they fire
        // when time moves past it).
        e.step_until(SimTime::from_nanos(k * CTRL_NS)).unwrap();
        let (th, ia, ib) = (
            sig(&e, "motor.theta"),
            sig(&e, "motor.i_a"),
            sig(&e, "motor.i_b"),
        );
        e.step_until(SimTime::from_nanos(k * CTRL_NS + 1)).unwrap();
        assert_eq!(sig(&e, "sensors.encoder.angle"), th, "k {k}");
        assert_eq!(sig(&e, "sensors.adc.i_a"), ia, "k {k}");
        // Two shunts: c is reconstructed from the exact a and b.
        assert_eq!(sig(&e, "sensors.adc.i_c"), -ia - ib, "k {k}");
    }
    assert!(sig(&e, "motor.theta").abs() > 1e-3, "the rotor moved");
}

#[test]
fn sampling_follows_the_mode_and_update_rate() {
    let adc = "{ adc: { bits: 12, full_scale: 20 A, shunts: 2 } }";
    // Switching: T_pwm with phase 0 = pwm.center (small-24v-10a: 20 kHz).
    let e = build(adc, "switching").unwrap().engine;
    let f_pwm = 20e3;
    assert_eq!(
        sampling(&e, "sensors.adc"),
        (
            SimTime::from_nanos((1e9 / f_pwm) as i64),
            SimTime::from_nanos(0)
        )
    );
    // Averaged: the controller period; an explicit rate wins in either mode.
    let e = build(adc, "averaged").unwrap().engine;
    assert_eq!(sampling(&e, "sensors.adc").0, SimTime::from_nanos(CTRL_NS));
    let e = build(
        "{ adc: { bits: 12, full_scale: 20 A, shunts: 2, update_rate: 8 kHz } }",
        "switching",
    )
    .unwrap()
    .engine;
    assert_eq!(sampling(&e, "sensors.adc").0, SimTime::from_nanos(125_000));
    // Magnetic encoder: 10 kHz without a rate (the chip presets give their polling rate,
    // AS5600 1 kHz); incremental and Halls: the controller rate.
    let e = build("{ encoder: { kind: magnetic, bits: 14 } }", "averaged")
        .unwrap()
        .engine;
    assert_eq!(
        sampling(&e, "sensors.encoder").0,
        SimTime::from_nanos(100_000)
    );
    let e = build("{ preset: \"builtin:sensors/as5600\" }", "averaged")
        .unwrap()
        .engine;
    assert_eq!(
        sampling(&e, "sensors.encoder").0,
        SimTime::from_nanos(1_000_000)
    );
    for p in ["optical-1000cpr", "hall-set"] {
        let e = build(
            &format!("{{ preset: \"builtin:sensors/{p}\" }}"),
            "averaged",
        )
        .unwrap()
        .engine;
        let id = if p == "hall-set" {
            "sensors.hall"
        } else {
            "sensors.encoder"
        };
        assert_eq!(sampling(&e, id).0, SimTime::from_nanos(CTRL_NS), "{p}");
    }
}

/// Per-phase ADC errors and the bus-voltage channel reach the block; the bus channel
/// needs a DC bus.
#[test]
fn adc_phase_errors_and_bus_channel() {
    let sensors = "{ adc: { bits: 24, full_scale: 20 A, shunts: 3, offsets: [0.5 A, 0 A, 0 A], gain_errors: [0, 0.1, 0], bus: { full_scale: 60 V } } }";
    let mut e = build(sensors, "averaged").unwrap().engine;
    e.step_until(SimTime::from_secs_f64(1e-3)).unwrap();
    let lsb = 40.0 / 2f64.powi(24);
    // At rest the currents are ~0: a reads its offset, b and c ~0.
    let (ia, ib) = (sig(&e, "motor.i_a"), sig(&e, "motor.i_b"));
    assert!((sig(&e, "sensors.adc.i_a") - (ia + 0.5)).abs() <= lsb + 1e-9);
    assert!((sig(&e, "sensors.adc.i_b") - 1.1 * ib).abs() <= lsb + 1e-9);
    let vb = sig(&e, "bus.v");
    assert!((sig(&e, "sensors.adc.v_bus") - vb).abs() <= 120.0 / 2f64.powi(24) + 1e-9);
    assert!(vb > 20.0);
    let err = build(sensors, "ideal")
        .err()
        .expect("refused without a bus");
    assert!(err.to_string().contains("sensors.adc.bus"), "{err}");
}

/// The scene seed drives the noise: same seed, same readings.
#[test]
fn sensor_noise_is_seeded() {
    let s = "{ adc: { bits: 24, full_scale: 20 A, shunts: 2, noise: 50 mA } }";
    let run = || {
        let mut e = build(s, "averaged").unwrap().engine;
        e.step_until(SimTime::from_secs_f64(1e-3)).unwrap();
        sig(&e, "sensors.adc.i_a")
    };
    let (a, b) = (run(), run());
    assert_eq!(a, b);
    assert!(a.abs() > 1e-4, "noise present: {a}");
}
