//! `bldc-sim validate-file <files...>` (P04.T06): load + derive + validate each file and
//! print every issue; returns false on any parse error or Reject.

use std::path::Path;

use sim_model::constraints::{self, Severity};
use sim_model::io::{IoError, load_yaml, parse_yaml};
use sim_model::library::Library;
use sim_model::params::{
    ControllerParams, GearboxParams, InverterParams, LoadParams, MotorParams, SensorsParams,
    SupplyParams,
};
use sim_model::scenario::Scenario;
use sim_model::scene::Scene;

pub fn run(files: &[std::path::PathBuf]) -> bool {
    let mut ok = true;
    for f in files {
        ok &= check(f);
    }
    ok
}

fn msg(n: &str, e: impl std::fmt::Display) -> IoError {
    IoError::Parse {
        path: n.into(),
        msg: e.to_string(),
    }
}

fn library(n: &str) -> Result<Library, IoError> {
    Library::open_default().map_err(|e| msg(n, e))
}

/// Resolve every library reference and run the motor constraint rules.
fn resolve(s: &Scene, n: &str) -> Result<(), IoError> {
    let r = s.resolve(&library(n)?).map_err(|e| msg(n, e))?;
    let mut m = r.motor;
    let mut issues = Vec::new();
    constraints::derive(&mut m, &mut issues);
    issues.extend(constraints::validate(&m));
    match issues.iter().find(|i| i.severity == Severity::Reject) {
        Some(i) => Err(msg(n, format!("motor: {}: {}", i.path, i.message))),
        None => Ok(()),
    }
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
        "scenes" => return typed(parse_yaml::<Scene>(&text, &n).and_then(|s| resolve(&s, &n))),
        "scenarios" => {
            return typed(parse_yaml::<Scenario>(&text, &n).and_then(|s| {
                let lib = library(&n)?;
                let scene = s.scene.resolve(&lib, "scene").map_err(|e| msg(&n, e))?;
                resolve(&scene, &n)
            }));
        }
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
