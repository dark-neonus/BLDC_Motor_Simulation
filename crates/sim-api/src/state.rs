//! State wire format (D-012): a JSON/msgpack map of dotted signal paths → values,
//! plus the `sim.*` runner status. Shared by REST, WebSocket and MCP.

use serde_json::{Map, Value};
use sim_core::engine::runner::{RunnerHandle, RunnerStatus};

pub fn state_map(h: &RunnerHandle, st: &RunnerStatus) -> Map<String, Value> {
    let mut m = Map::new();
    m.insert("sim.t".into(), st.t.into());
    m.insert("sim.running".into(), st.running.into());
    m.insert(
        "sim.time_scale".into(),
        if st.time_scale.is_finite() {
            st.time_scale.into()
        } else {
            Value::Null
        },
    );
    m.insert("sim.real_ratio".into(), st.real_ratio.into());
    m.insert("sim.lagging".into(), st.lagging.into());
    if let Some(e) = &st.error {
        m.insert("sim.error".into(), e.clone().into());
    }
    for (meta, v) in h.signals.iter().zip(&st.values) {
        m.insert(meta.path.clone(), (*v).into());
    }
    m
}
