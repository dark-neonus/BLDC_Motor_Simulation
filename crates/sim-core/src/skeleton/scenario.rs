//! Skeleton scenario runner (P01.T04): a minimal batch simulation used by the
//! `run-scenario` CLI and the Python validation tests. Replaced by the full
//! scenario format in P04.T11/T13 — keep the CLI output contract stable.

use serde::Deserialize;

/// One timeline action.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Action {
    /// Time at which the action applies [s].
    pub t: f64,
    /// New speed setpoint [rad/s].
    pub set_target_speed: Option<f64>,
}

/// Minimal scenario file.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SkeletonScenario {
    /// Simulated duration [s].
    pub duration: f64,
    /// Plant integration step [s] (default 5 µs).
    #[serde(default = "default_dt")]
    pub dt: f64,
    /// Recording interval [s]; must be a multiple of `dt` (default 100 µs).
    #[serde(default = "default_record_every")]
    pub record_every: f64,
    /// Hold the rotor (ω = 0).
    #[serde(default)]
    pub lock_rotor: bool,
    /// If set, bypass FOC and apply v_d = 0, v_q = this value [V].
    pub vq_open_loop: Option<f64>,
    /// Timeline of setpoint changes.
    #[serde(default)]
    pub actions: Vec<Action>,
}

fn default_dt() -> f64 {
    5e-6
}
fn default_record_every() -> f64 {
    1e-4
}

/// Recorded signal columns, named per CONVENTIONS §3.
pub const COLUMNS: [&str; 6] = [
    "t",
    "motor.omega",
    "motor.theta",
    "motor.i_d",
    "motor.i_q",
    "ctrl.omega_ref",
];

/// Run the scenario on the production engine; one row per record instant, matching [`COLUMNS`].
pub fn simulate(sc: &SkeletonScenario) -> Result<Vec<[f64; 6]>, String> {
    use super::adapter::{SkeletonOptions, build_engine};
    use crate::engine::commands::{ChangeSource, EngineCommand};
    use crate::engine::time::SimTime;

    let steps_per_record = (sc.record_every / sc.dt).round();
    if sc.dt <= 0.0 || sc.duration <= 0.0 || steps_per_record < 1.0 {
        return Err("dt, duration must be > 0 and record_every >= dt".into());
    }
    if ((sc.record_every / sc.dt) - steps_per_record).abs() > 1e-9 {
        return Err(format!(
            "record_every ({}) must be a multiple of dt ({})",
            sc.record_every, sc.dt
        ));
    }
    let ctrl_dt = super::foc::FocConfig::skeleton().dt;
    let ratio = ctrl_dt / sc.dt;
    if ratio < 1.0 - 1e-9 || (ratio - ratio.round()).abs() > 1e-9 {
        return Err(format!(
            "dt ({}) must divide the controller period ({ctrl_dt} s)",
            sc.dt
        ));
    }
    let mut e = build_engine(SkeletonOptions {
        locked: sc.lock_rotor,
        open_loop_vq: sc.vq_open_loop,
        dt_max: sc.dt,
    });
    let ids: Vec<_> = COLUMNS[1..]
        .iter()
        .map(|c| e.bus.id(c))
        .collect::<Result<_, _>>()
        .map_err(|err| err.to_string())?;
    let mut actions = sc.actions.clone();
    actions.sort_by(|a, b| a.t.total_cmp(&b.t));
    let n = (sc.duration / sc.record_every).round() as usize;
    let mut rows = Vec::with_capacity(n + 1);
    let mut next_action = 0;
    for k in 0..=n {
        let t = k as f64 * sc.record_every;
        // Apply actions due up to this record instant, at their own times.
        while next_action < actions.len() && actions[next_action].t <= t + 0.5 * sc.dt {
            let a = &actions[next_action];
            e.step_until(SimTime::from_secs_f64(a.t))
                .map_err(|err| err.to_string())?;
            if let Some(w) = a.set_target_speed {
                e.queue(EngineCommand::SetSignal {
                    path: "ctrl.omega_ref".into(),
                    value: w,
                    source: ChangeSource::Scenario,
                });
                e.apply_pending();
            }
            next_action += 1;
        }
        e.step_until(SimTime::from_secs_f64(t))
            .map_err(|err| err.to_string())?;
        let mut row = [t, 0.0, 0.0, 0.0, 0.0, 0.0];
        for (j, &id) in ids.iter().enumerate() {
            row[j + 1] = e.bus.get(id);
        }
        rows.push(row);
    }
    Ok(rows)
}
