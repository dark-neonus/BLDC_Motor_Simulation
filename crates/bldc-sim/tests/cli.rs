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
