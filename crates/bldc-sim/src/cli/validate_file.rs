//! `bldc-sim validate-file <files...>` (P04.T06): load + derive + validate each file and
//! print every issue; returns false on any parse error or Reject.

use std::path::Path;

use sim_model::constraints::{self, Severity};
use sim_model::io::load_yaml;
use sim_model::params::MotorParams;

pub fn run(files: &[std::path::PathBuf]) -> bool {
    let mut ok = true;
    for f in files {
        ok &= check(f);
    }
    ok
}

fn check(f: &Path) -> bool {
    let name = f.display();
    // File type by top-level keys; other types join as their validators land (P04.T10/T11).
    let text = match std::fs::read_to_string(f) {
        Ok(t) => t,
        Err(e) => {
            println!("{name}: error: {e}");
            return false;
        }
    };
    if !text.lines().any(|l| l.starts_with("electrical:")) {
        println!("{name}: error: unknown file type (only motor files are validated so far)");
        return false;
    }
    let mut m: MotorParams = match load_yaml(f) {
        Ok(m) => m,
        Err(e) => {
            println!("error: {e}");
            return false;
        }
    };
    let mut issues = Vec::new();
    constraints::derive(&mut m, &mut issues);
    issues.extend(constraints::validate(&m));
    let mut ok = true;
    for i in &issues {
        let sev = match i.severity {
            Severity::Info => "info",
            Severity::Warn => "warn",
            Severity::Reject => {
                ok = false;
                "reject"
            }
        };
        println!("{name}: {sev}: {}: {}", i.path, i.message);
    }
    if ok {
        println!("{name}: ok");
    }
    ok
}
