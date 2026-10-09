//! Scenes (P04.T11): a complete simulation setup. Each component is a library reference
//! (with optional dotted-path overrides) or inline parameters.

use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};

use crate::library::Library;
use crate::param::Param;
use crate::params::{
    BusParams, ControllerParams, FidelityParams, GearboxParams, InverterParams, LoadParams,
    MotorParams, ProtectionParams, SensorsParams, SupplyParams,
};

/// A library reference with overrides, or inline parameters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum Component<T> {
    Ref(PresetRef),
    Inline(T),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PresetRef {
    /// Library id, e.g. `builtin:motors/8010-outrunner`.
    pub preset: String,
    /// Dotted path inside the component → new value, e.g. `electrical.r_phase: 0.2 ohm`.
    #[serde(default, skip_serializing_if = "serde_json::Map::is_empty")]
    pub overrides: serde_json::Map<String, serde_json::Value>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Initial {
    /// Load-side angle [rad or deg].
    pub theta: Option<Param>,
    /// Load-side speed [rad/s or rpm].
    pub omega: Option<Param>,
    /// All thermal nodes start here (default: ambient).
    pub temperature: Option<Param>,
    pub ambient: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Scene {
    pub schema_version: u32,
    pub name: String,
    pub description: Option<String>,
    pub motor: Component<MotorParams>,
    pub gearbox: Option<Component<GearboxParams>>,
    pub load: Option<Component<LoadParams>>,
    pub inverter: Component<InverterParams>,
    pub supply: Component<SupplyParams>,
    pub bus: Option<BusParams>,
    pub sensors: Option<Component<SensorsParams>>,
    pub controller: Component<ControllerParams>,
    pub protection: Option<ProtectionParams>,
    pub fidelity: Option<FidelityParams>,
    #[serde(default)]
    pub seed: u64,
    #[serde(default)]
    pub initial: Initial,
}

/// A scene with every reference loaded and overrides applied.
#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedScene {
    pub name: String,
    pub motor: MotorParams,
    pub gearbox: Option<GearboxParams>,
    pub load: Option<LoadParams>,
    pub inverter: InverterParams,
    pub supply: SupplyParams,
    pub bus: Option<BusParams>,
    pub sensors: Option<SensorsParams>,
    pub controller: ControllerParams,
    pub protection: Option<ProtectionParams>,
    pub fidelity: Option<FidelityParams>,
    pub seed: u64,
    pub initial: Initial,
}

#[derive(Debug, thiserror::Error)]
pub enum ResolveError {
    #[error("{0}: {1}")]
    Library(String, crate::library::LibError),
    #[error("{0}: {1}")]
    Invalid(String, String),
}

/// Set `value` at a dotted path inside `doc` (intermediate maps are created).
pub fn set_path(
    doc: &mut serde_json::Value,
    path: &str,
    value: serde_json::Value,
) -> Result<(), String> {
    let mut cur = doc;
    let parts: Vec<&str> = path.split('.').collect();
    for (i, key) in parts.iter().enumerate() {
        let obj = cur
            .as_object_mut()
            .ok_or_else(|| format!("`{path}`: `{key}` is not inside a map"))?;
        if i + 1 == parts.len() {
            obj.insert((*key).to_owned(), value);
            return Ok(());
        }
        cur = obj
            .entry((*key).to_owned())
            .or_insert_with(|| serde_json::json!({}));
    }
    Err(format!("empty path `{path}`"))
}

impl<T: DeserializeOwned + Serialize + Clone> Component<T> {
    pub fn resolve(&self, lib: &Library, what: &str) -> Result<T, ResolveError> {
        match self {
            Component::Inline(t) => Ok(t.clone()),
            Component::Ref(r) => {
                let ctx = format!("{what} ({})", r.preset);
                let text = lib
                    .load(&r.preset)
                    .map_err(|e| ResolveError::Library(what.into(), e))?;
                let mut doc: serde_json::Value = serde_saphyr::from_str(&text)
                    .map_err(|e| ResolveError::Invalid(ctx.clone(), e.to_string()))?;
                for (path, v) in &r.overrides {
                    set_path(&mut doc, path, v.clone())
                        .map_err(|e| ResolveError::Invalid(ctx.clone(), e))?;
                }
                serde_json::from_value(doc).map_err(|e| ResolveError::Invalid(ctx, e.to_string()))
            }
        }
    }
}

fn opt<T: DeserializeOwned + Serialize + Clone>(
    c: &Option<Component<T>>,
    lib: &Library,
    what: &str,
) -> Result<Option<T>, ResolveError> {
    c.as_ref().map(|c| c.resolve(lib, what)).transpose()
}

impl Scene {
    pub fn resolve(&self, lib: &Library) -> Result<ResolvedScene, ResolveError> {
        Ok(ResolvedScene {
            name: self.name.clone(),
            motor: self.motor.resolve(lib, "motor")?,
            gearbox: opt(&self.gearbox, lib, "gearbox")?,
            load: opt(&self.load, lib, "load")?,
            inverter: self.inverter.resolve(lib, "inverter")?,
            supply: self.supply.resolve(lib, "supply")?,
            bus: self.bus.clone(),
            sensors: opt(&self.sensors, lib, "sensors")?,
            controller: self.controller.resolve(lib, "controller")?,
            protection: self.protection.clone(),
            fidelity: self.fidelity.clone(),
            seed: self.seed,
            initial: self.initial.clone(),
        })
    }
}
