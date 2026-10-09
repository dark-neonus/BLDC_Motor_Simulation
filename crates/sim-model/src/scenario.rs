//! Scenarios (P04.T11): a scene plus a timeline of actions. The timeline runs in order;
//! `wait` advances time, every other action happens at the current time. Asserts make a
//! scenario a regression test (executed by P04.T13).

use serde::{Deserialize, Serialize};

use crate::param::Param;
use crate::scene::{Component, Scene};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Scenario {
    pub schema_version: u32,
    pub name: String,
    pub description: Option<String>,
    pub scene: Component<Scene>,
    /// Signal ids to record (`motor.omega_m`, …). Empty: the default set.
    #[serde(default)]
    pub record: Vec<String>,
    /// Recording period (default: every controller tick).
    pub sample: Option<Param>,
    pub timeline: Vec<Action>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum TargetKind {
    Position,
    Velocity,
    Torque,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Op {
    Lt,
    Le,
    Gt,
    Ge,
    /// |signal − value| ≤ tol.
    Eq,
    Ne,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Assert {
    pub signal: String,
    pub op: Op,
    /// SI value.
    pub value: f64,
    #[serde(default)]
    pub tol: f64,
    /// Absolute check time (default: the current timeline time).
    pub at: Option<Param>,
    /// The condition must hold over [t, t + window].
    pub window: Option<Param>,
    pub message: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum Action {
    /// Change a parameter or signal by its dotted id.
    Set {
        path: String,
        value: serde_json::Value,
    },
    /// New setpoint for the active controller (load side).
    Target {
        kind: TargetKind,
        value: Param,
        ramp: Option<Param>,
    },
    Fault {
        id: String,
        on: bool,
    },
    /// Extra load torque for a duration.
    Disturbance {
        torque: Param,
        duration: Param,
    },
    /// Advance time.
    Wait(Param),
    Assert(Assert),
    /// Save a named snapshot.
    Snapshot(String),
    Stop,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::parse_yaml;
    use crate::library::Library;

    const SC: &str = r#"
schema_version: 1
name: arm step
scene: { preset: "builtin:scenes/arm-servo" }
record: [motor.omega_m, motor.i_q]
timeline:
  - target: { kind: position, value: 90 deg, ramp: 0.2 s }
  - wait: 1 s
  - assert: { signal: load.theta, op: eq, value: 1.5708, tol: 0.02, window: 0.5 s }
  - set: { path: motor.electrical.r_phase, value: 0.2 ohm }
  - fault: { id: hall_a_stuck, on: true }
  - disturbance: { torque: 0.5 N*m, duration: 50 ms }
  - snapshot: after-step
  - stop
"#;

    #[test]
    fn scenario_round_trip_and_scene_resolves() {
        let s: Scenario = parse_yaml(SC, "t").unwrap();
        assert_eq!(s.timeline.len(), 8);
        assert_eq!(s.timeline[7], Action::Stop);
        let back: Scenario = parse_yaml(&serde_saphyr::to_string(&s).unwrap(), "rt").unwrap();
        assert_eq!(back, s);
        assert!(parse_yaml::<Scenario>(&SC.replace("op: eq", "op: about"), "t").is_err());

        let dir = std::env::temp_dir().join(format!("bldc-scn-{}", std::process::id()));
        let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
        let Component::Ref(r) = &s.scene else {
            panic!()
        };
        let scene: Scene = parse_yaml(&lib.load(&r.preset).unwrap(), "scene").unwrap();
        let res = scene.resolve(&lib).unwrap();
        assert!(res.gearbox.is_some() && res.load.is_some());
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn overrides_apply_and_bad_refs_fail() {
        let dir = std::env::temp_dir().join(format!("bldc-ovr-{}", std::process::id()));
        let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
        let c: Component<crate::params::MotorParams> = parse_yaml(
            "preset: builtin:motors/8108-outrunner\noverrides: { electrical.r_phase: 0.2 ohm, identity.name: tuned }\n",
            "c",
        )
        .unwrap();
        let m = c.resolve(&lib, "motor").unwrap();
        assert_eq!(m.identity.name, "tuned");
        assert_eq!(
            m.electrical.r_phase.value,
            crate::param::Raw::Text("0.2 ohm".into())
        );
        let bad: Component<crate::params::MotorParams> =
            parse_yaml("preset: builtin:motors/nope\n", "c").unwrap();
        assert!(bad.resolve(&lib, "motor").is_err());
        let bad: Component<crate::params::MotorParams> = parse_yaml(
            "preset: builtin:motors/8108-outrunner\noverrides: { electrical.colour: red }\n",
            "c",
        )
        .unwrap();
        assert!(
            bad.resolve(&lib, "motor").is_err(),
            "unknown field via override is rejected"
        );
        let _ = std::fs::remove_dir_all(dir);
    }
}
