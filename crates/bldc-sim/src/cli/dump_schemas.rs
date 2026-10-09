//! `bldc-sim dump-schemas --out schemas/` (P04.T07): one JSON Schema per file type, used
//! by editors (yaml-language-server modeline) and the CI schema check.
//! `scene`/`scenario` join when their types land (P04.T11), `lesson` in P18.T09.

use std::path::Path;

use anyhow::Context;
use schemars::{JsonSchema, schema_for};
use sim_model::params::{
    ControllerParams, GearboxParams, InverterParams, LoadParams, MotorParams, SensorsParams,
    SupplyParams,
};

fn write<T: JsonSchema>(dir: &Path, name: &str) -> anyhow::Result<()> {
    let path = dir.join(format!("{name}.schema.json"));
    let text = serde_json::to_string_pretty(&schema_for!(T))? + "\n";
    std::fs::write(&path, text).with_context(|| format!("writing {}", path.display()))
}

pub fn run(out: &Path) -> anyhow::Result<()> {
    std::fs::create_dir_all(out)?;
    write::<MotorParams>(out, "motor")?;
    write::<GearboxParams>(out, "gearbox")?;
    write::<LoadParams>(out, "load")?;
    write::<InverterParams>(out, "inverter")?;
    write::<SupplyParams>(out, "supply")?;
    write::<SensorsParams>(out, "sensors")?;
    write::<ControllerParams>(out, "controller")?;
    Ok(())
}
