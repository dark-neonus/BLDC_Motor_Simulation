//! Sandboxed Luau custom controller block (P09.T10, docs/docs/control/custom-block.md).
//!
//! The script defines `init(params) -> state` and `step(inputs, state, dt) -> outputs
//! [, state]`. `inputs` is one table reused every call, keyed by the subscribed signal
//! paths; only the declared output paths are read back and written to the bus. The VM is
//! sandboxed (`Lua::sandbox`), has a memory limit and an instruction budget per call
//! (Luau interrupts on loop back-edges and calls; exceeding the budget is a block error,
//! which pauses the run). The chunk is compiled once; errors carry `custom:LINE:`.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use mlua::{Function, Lua, Table, Value, VmState};

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalId};
use crate::engine::time::SimTime;

/// Default memory limit [bytes] and per-call interrupt budget.
pub const DEFAULT_MEMORY: usize = 16 << 20;
pub const DEFAULT_BUDGET: u64 = 1_000_000;

#[derive(Debug, Clone, PartialEq)]
pub struct LuauConfig {
    pub script: String,
    pub period: SimTime,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    /// Passed to `init` as a table of numbers.
    pub params: Vec<(String, f64)>,
    pub memory: usize,
    pub budget: u64,
}

pub struct LuauBlock {
    cfg: LuauConfig,
    lua: Lua,
    step_fn: Function,
    init_fn: Function,
    inputs: Table,
    state: Value,
    in_ids: Vec<SignalId>,
    out_ids: Vec<SignalId>,
    counter: Arc<AtomicU64>,
    warnings: Vec<String>,
}

fn lua_err(e: mlua::Error) -> String {
    // Keep the script location (`custom:LINE: message`), drop the traceback.
    let s = e.to_string();
    s.lines().next().unwrap_or(&s).to_string()
}

impl LuauBlock {
    pub fn new(cfg: LuauConfig, bus: &SignalBus) -> Result<Self, String> {
        let in_ids = cfg
            .inputs
            .iter()
            .map(|p| bus.id(p).map_err(|e| format!("input {p}: {e}")))
            .collect::<Result<Vec<_>, _>>()?;
        let out_ids = cfg
            .outputs
            .iter()
            .map(|p| bus.id(p).map_err(|e| format!("output {p}: {e}")))
            .collect::<Result<Vec<_>, _>>()?;
        let lua = Lua::new();
        lua.set_memory_limit(cfg.memory).map_err(lua_err)?;
        let counter = Arc::new(AtomicU64::new(0));
        let (c, budget) = (counter.clone(), cfg.budget);
        lua.set_interrupt(move |_| {
            if c.fetch_add(1, Ordering::Relaxed) >= budget {
                return Err(mlua::Error::runtime(format!(
                    "instruction budget exceeded ({budget} interrupts in one call): an endless loop?"
                )));
            }
            Ok(VmState::Continue)
        });
        lua.load(cfg.script.as_str())
            .set_name("=custom")
            .exec()
            .map_err(lua_err)?;
        // Sandbox after the definitions: the script's globals become read-only.
        lua.sandbox(true).map_err(lua_err)?;
        let g = lua.globals();
        let get = |n: &str| -> Result<Function, String> {
            g.get::<Function>(n)
                .map_err(|_| format!("the script must define `function {n}(...)`"))
        };
        let (init_fn, step_fn) = (get("init")?, get("step")?);
        let inputs = lua
            .create_table_with_capacity(0, in_ids.len())
            .map_err(lua_err)?;
        let mut b = Self {
            cfg,
            lua,
            step_fn,
            init_fn,
            inputs,
            state: Value::Nil,
            in_ids,
            out_ids,
            counter,
            warnings: vec![],
        };
        b.state = b.run_init()?;
        Ok(b)
    }

    fn run_init(&self) -> Result<Value, String> {
        let p = self.lua.create_table().map_err(lua_err)?;
        for (k, v) in &self.cfg.params {
            p.set(k.as_str(), *v).map_err(lua_err)?;
        }
        self.counter.store(0, Ordering::Relaxed);
        self.init_fn.call::<Value>(p).map_err(lua_err)
    }
}

impl DiscreteBlock for LuauBlock {
    fn id(&self) -> &str {
        "controller.custom"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.cfg.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let err = |msg: String| SimError::Block {
            block: "controller.custom".into(),
            msg,
        };
        for (path, id) in self.cfg.inputs.iter().zip(&self.in_ids) {
            self.inputs
                .raw_set(path.as_str(), ctx.bus.get(*id))
                .map_err(|e| err(lua_err(e)))?;
        }
        self.counter.store(0, Ordering::Relaxed);
        let dt = self.cfg.period.as_secs_f64();
        let (out, new_state): (Value, Value) = self
            .step_fn
            .call((self.inputs.clone(), self.state.clone(), dt))
            .map_err(|e| err(lua_err(e)))?;
        if !matches!(new_state, Value::Nil) {
            self.state = new_state;
        }
        let Value::Table(t) = out else {
            return Err(err("`step` must return a table of outputs".into()));
        };
        for (path, id) in self.cfg.outputs.iter().zip(&self.out_ids) {
            match t
                .raw_get::<Value>(path.as_str())
                .map_err(|e| err(lua_err(e)))?
            {
                Value::Number(x) => ctx.bus.set(*id, x),
                Value::Integer(x) => ctx.bus.set(*id, x as f64),
                Value::Nil => {}
                v => {
                    return Err(err(format!(
                        "output `{path}` must be a number, got {}",
                        v.type_name()
                    )));
                }
            }
        }
        Ok(())
    }
    fn reset(&mut self) {
        match self.run_init() {
            Ok(s) => self.state = s,
            Err(e) => self.warnings.push(format!("init on reset failed: {e}")),
        }
    }
    fn take_warnings(&mut self) -> Vec<String> {
        std::mem::take(&mut self.warnings)
    }
    /// The script state, when it holds only numbers, strings, booleans and tables.
    fn save(&self) -> serde_json::Value {
        match to_json(&self.state, 0) {
            Some(v) => serde_json::json!({ "state": v }),
            None => serde_json::json!({ "state": null, "unsaved": true }),
        }
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        if v.get("unsaved").and_then(serde_json::Value::as_bool) == Some(true) {
            self.warnings.push(
                "the custom block's state held functions or userdata and was not in the snapshot: it restarts from init"
                    .into(),
            );
            self.state = self.run_init()?;
            return Ok(());
        }
        self.state = from_json(
            &self.lua,
            v.get("state").unwrap_or(&serde_json::Value::Null),
        )
        .map_err(lua_err)?;
        Ok(())
    }
}

/// Lua state → JSON (numbers, strings, booleans, tables with string or integer keys);
/// `None` for anything else.
fn to_json(v: &Value, depth: usize) -> Option<serde_json::Value> {
    if depth > 32 {
        return None;
    }
    Some(match v {
        Value::Nil => serde_json::Value::Null,
        Value::Boolean(b) => serde_json::Value::Bool(*b),
        Value::Integer(i) => serde_json::json!({ "i": i }),
        Value::Number(x) => serde_json::json!({ "n": x }),
        Value::String(s) => serde_json::Value::String(s.to_str().ok()?.to_string()),
        Value::Table(t) => {
            let mut entries = vec![];
            for pair in t.pairs::<Value, Value>() {
                let (k, val) = pair.ok()?;
                entries.push(serde_json::json!([
                    to_json(&k, depth + 1)?,
                    to_json(&val, depth + 1)?
                ]));
            }
            serde_json::json!({ "t": entries })
        }
        _ => return None,
    })
}

fn from_json(lua: &Lua, v: &serde_json::Value) -> mlua::Result<Value> {
    Ok(match v {
        serde_json::Value::Null => Value::Nil,
        serde_json::Value::Bool(b) => Value::Boolean(*b),
        serde_json::Value::String(s) => Value::String(lua.create_string(s)?),
        serde_json::Value::Object(o) => {
            if let Some(i) = o.get("i").and_then(serde_json::Value::as_i64) {
                Value::Integer(i as _)
            } else if let Some(x) = o.get("n").and_then(serde_json::Value::as_f64) {
                Value::Number(x)
            } else {
                let t = lua.create_table()?;
                for e in o
                    .get("t")
                    .and_then(serde_json::Value::as_array)
                    .into_iter()
                    .flatten()
                {
                    let pair = e.as_array().map(Vec::as_slice).unwrap_or(&[]);
                    if let [k, val] = pair {
                        t.raw_set(from_json(lua, k)?, from_json(lua, val)?)?;
                    }
                }
                Value::Table(t)
            }
        }
        _ => Value::Nil,
    })
}
