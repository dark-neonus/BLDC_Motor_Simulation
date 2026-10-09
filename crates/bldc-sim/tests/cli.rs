//! Smoke test of the `bldc-sim` binary (also proves nextest discovers tests).

use std::process::Command;

#[test]
fn version_flag_prints_package_version() {
    let out = Command::new(env!("CARGO_BIN_EXE_bldc-sim"))
        .arg("--version")
        .output()
        .expect("binary runs");
    assert!(out.status.success());
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains(env!("CARGO_PKG_VERSION")), "got: {stdout}");
}

#[test]
fn validate_file_exit_codes() {
    let dir = std::env::temp_dir().join(format!("bldc-vf-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let good = "schema_version: 1\nidentity: { name: t, topology: inrunner }\nelectrical:\n  r_phase: 1 ohm\n  l_d: 1 mH\n  l_q: 1 mH\n  lambda_m: 0.01 Wb\n  entered_as: lambda_m\n  connection: star\nmagnetic: { emf_shape: { kind: sinusoidal } }\nwinding: { slots: 12, pole_pairs: 7 }\nmechanical: { j_rotor: 1e-5 }\n";
    let cases = [
        ("good.yaml", good.to_string(), true),
        ("slots.yaml", good.replace("slots: 12", "slots: 14"), false),
        ("syntax.yaml", "electrical: [\n".to_string(), false),
    ];
    for (name, text, expect_ok) in cases {
        let f = dir.join(name);
        std::fs::write(&f, text).unwrap();
        let out = Command::new(env!("CARGO_BIN_EXE_bldc-sim"))
            .arg("validate-file")
            .arg(&f)
            .output()
            .expect("binary runs");
        assert_eq!(
            out.status.success(),
            expect_ok,
            "{name}: {}",
            String::from_utf8_lossy(&out.stdout)
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}
