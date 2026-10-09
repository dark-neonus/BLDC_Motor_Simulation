//! `bldc-sim validate-file <files...>` (P04.T06): load + derive + validate each file and
//! print every issue; returns false on any parse error or Reject.

use std::path::Path;

use sim_model::constraints::{self, Severity};
use sim_model::io::{load_yaml, parse_yaml};
use sim_model::params::{
    ControllerParams, GearboxParams, InverterParams, LoadParams, MotorParams, SensorsParams,
    SupplyParams,
};

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
    // File type from the library folder (`presets/<kind>/x.yaml`); motor files are also
    // recognised by their `electrical:` key. Non-motor types are checked by a typed parse
    // (unknown fields, tags, value forms); their constraint rules come with their modules.
    let dir = f
        .parent()
        .and_then(|d| d.file_name())
        .and_then(|d| d.to_str())
        .unwrap_or("");
    let typed = |r: Result<(), sim_model::io::IoError>| match r {
        Ok(()) => {
            println!("{name}: ok");
            true
        }
        Err(e) => {
            println!("error: {e}");
            false
        }
    };
    let n = name.to_string();
    match dir {
        "gearboxes" => return typed(parse_yaml::<GearboxParams>(&text, &n).map(drop)),
        "loads" => return typed(parse_yaml::<LoadParams>(&text, &n).map(drop)),
        "inverters" => return typed(parse_yaml::<InverterParams>(&text, &n).map(drop)),
        "supplies" => return typed(parse_yaml::<SupplyParams>(&text, &n).map(drop)),
        "sensors" => return typed(parse_yaml::<SensorsParams>(&text, &n).map(drop)),
        "controllers" => return typed(parse_yaml::<ControllerParams>(&text, &n).map(drop)),
        _ => {}
    }
    if !text.lines().any(|l| l.starts_with("electrical:")) {
        println!(
            "{name}: error: unknown file type (put it in a library folder such as motors/ or loads/)"
        );
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
