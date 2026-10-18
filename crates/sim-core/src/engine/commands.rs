//! Engine commands and events. Commands are queued and applied only between
//! events, never mid-integration (P03.T07). Parameter paths are
//! `<module or block id>.<name>` until P04 routes edits through `apply_edit`.

use serde::{Deserialize, Serialize};

use super::time::SimTime;

/// Who caused a change (shown in the UI so agent edits are visible).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ChangeSource {
    Ui,
    Mcp,
    Scenario,
    Internal,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum EngineCommand {
    /// Set a live parameter.
    SetParam {
        path: String,
        value: f64,
        source: ChangeSource,
    },
    /// Write an input signal on the bus (setpoints), e.g. `ctrl.omega_ref`.
    SetSignal {
        path: String,
        value: f64,
        source: ChangeSource,
    },
    /// A torque pulse on the load shaft for `duration` seconds (EQ-MECH-08).
    Disturbance {
        torque: f64,
        duration: f64,
        source: ChangeSource,
    },
    /// An impulse H [N·m·s] (mouse flick): the pulse H/Δt with Δt = max(1 ms, 5·dt_max).
    Impulse { h: f64, source: ChangeSource },
    /// A velocity target [rad/s, load side] in velocity mode (EQ-CTRL-04): the position
    /// loop off and the velocity loop on (where they exist), then `ctrl.omega_ref`.
    VelocityTarget { value: f64, source: ChangeSource },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum EngineEvent {
    ParamChanged {
        t: SimTime,
        path: String,
        old: f64,
        new: f64,
        source: ChangeSource,
    },
    SignalSet {
        t: SimTime,
        path: String,
        old: f64,
        new: f64,
        source: ChangeSource,
    },
    /// Non-fatal diagnostic from the engine or a block.
    Warning {
        t: SimTime,
        source: String,
        msg: String,
    },
    CommandRejected {
        t: SimTime,
        reason: String,
    },
}
