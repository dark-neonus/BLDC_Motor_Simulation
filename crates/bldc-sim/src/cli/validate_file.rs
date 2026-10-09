//! `bldc-sim validate-file <files...>` (P04.T06): load + derive + validate each file and
//! print every issue; returns false on any parse error or Reject.

use std::path::Path;

use sim_model::components::*;
use sim_model::constraints::{self, Issue, Severity};
use sim_model::io::parse_yaml;
use sim_model::library::Library;
use sim_model::params::*;
use sim_model::scenario::Scenario;
use sim_model::scene::{Component, Scene};

pub fn run(files: &[std::path::PathBuf]) -> bool {
    let mut ok = true;
    for f in files {
        ok &= check(f);
    }
    ok
}

/// Print every issue; true if none is a Reject.
fn report(name: &str, issues: &[Issue]) -> bool {
    let mut ok = true;
    for i in issues {
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

/// Resolve every library reference, derive, and run all rules.
fn scene_issues(c: &Component<Scene>) -> Result<Vec<Issue>, String> {
    let lib = Library::open_default().map_err(|e| e.to_string())?;
    let scene = c.resolve(&lib, "scene").map_err(|e| e.to_string())?;
    let mut r = scene.resolve(&lib).map_err(|e| e.to_string())?;
    Ok(r.check())
}

fn issues_for(dir: &str, text: &str, n: &str) -> Result<Vec<Issue>, String> {
    fn p<T: serde::de::DeserializeOwned>(text: &str, n: &str) -> Result<T, String> {
        parse_yaml(text, n).map_err(|e| e.to_string())
    }
    Ok(match dir {
        "gearboxes" => validate_gearbox(&p(text, n)?),
        "loads" => validate_load(&p(text, n)?),
        "inverters" => validate_inverter(&p(text, n)?),
        "supplies" => validate_supply(&p(text, n)?),
        "sensors" => validate_sensors(&p(text, n)?),
        "controllers" => validate_controller(&p(text, n)?),
        "scenes" => scene_issues(&Component::Inline(p(text, n)?))?,
        "scenarios" => scene_issues(&p::<Scenario>(text, n)?.scene)?,
        _ if text.lines().any(|l| l.starts_with("electrical:")) => {
            let mut m: MotorParams = p(text, n)?;
            let mut issues = Vec::new();
            constraints::derive(&mut m, &mut issues);
            issues.extend(constraints::validate(&m));
            issues
        }
        _ => {
            return Err(format!(
                "{n}: unknown file type (put it in a library folder such as motors/ or loads/)"
            ));
        }
    })
}

fn check(f: &Path) -> bool {
    let name = f.display().to_string();
    // File type from the library folder (`presets/<kind>/x.yaml`); motor files are also
    // recognised by their `electrical:` key.
    let dir = f
        .parent()
        .and_then(|d| d.file_name())
        .and_then(|d| d.to_str())
        .unwrap_or("");
    let result = std::fs::read_to_string(f)
        .map_err(|e| format!("{name}: {e}"))
        .and_then(|text| issues_for(dir, &text, &name));
    match result {
        Ok(issues) => report(&name, &issues),
        Err(e) => {
            println!("error: {e}");
            false
        }
    }
}
