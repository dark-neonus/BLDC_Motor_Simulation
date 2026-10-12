//! P08.T03: the encoder chip presets load and build per the sensors.md chip table.

use sim_core::build::magnetic_encoder;
use sim_core::engine::time::SimTime;
use sim_model::io::parse_yaml;
use sim_model::params::SensorsParams;

fn load(name: &str) -> SensorsParams {
    let path = format!(
        "{}/../../presets/sensors/{name}.yaml",
        env!("CARGO_MANIFEST_DIR")
    );
    let text = std::fs::read_to_string(&path).unwrap();
    parse_yaml(&text, name).unwrap()
}

#[test]
fn chip_presets_build_with_the_datasheet_values() {
    let deg = f64::to_radians;
    for (name, bits, inl, noise, rate, latency_ns) in [
        ("as5047p", 14, 0.8, 0.068, 10e3, 10_000),
        ("as5600", 12, 1.0, 0.015, 1e3, 1_000_000),
        ("mt6701", 14, 1.0, 0.05, 10e3, 5_000),
    ] {
        let p = load(name);
        let (c, period) = magnetic_encoder(p.encoder.as_ref().unwrap())
            .unwrap()
            .unwrap();
        assert_eq!(c.bits, bits, "{name}");
        assert!((c.inl - deg(inl)).abs() < 1e-12, "{name} inl {}", c.inl);
        assert!(
            (c.noise - deg(noise)).abs() < 1e-12,
            "{name} noise {}",
            c.noise
        );
        assert_eq!(period, SimTime::from_secs_f64(1.0 / rate), "{name}");
        assert_eq!(c.latency_ns, latency_ns, "{name}");
        assert!(!c.bypass && c.air_gap.is_none());
    }
}
