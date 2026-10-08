//! Skeleton scenario runner (P01.T04): a minimal batch simulation used by the
//! `run-scenario` CLI and the Python validation tests. Replaced by the full
//! scenario format in P04.T11/T13 — keep the CLI output contract stable.

use serde::Deserialize;

use super::foc::{Foc, FocConfig, FocGains};
use super::model::{Pmsm, PmsmParams};

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

/// Run the scenario; returns one row per record instant, matching [`COLUMNS`].
pub fn simulate(sc: &SkeletonScenario) -> Result<Vec<[f64; 6]>, String> {
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
    let mp = PmsmParams::skeleton_6020();
    let cfg = FocConfig::skeleton();
    let steps_per_ctrl = (cfg.dt / sc.dt).round().max(1.0) as u64;
    let mut motor = Pmsm::new(mp);
    motor.locked = sc.lock_rotor;
    let mut foc = Foc::new(cfg, FocGains::design(&mp, &cfg));
    let mut actions = sc.actions.clone();
    actions.sort_by(|a, b| a.t.total_cmp(&b.t));
    let mut next_action = 0;

    let total_steps = (sc.duration / sc.dt).round() as u64;
    let steps_per_record = steps_per_record as u64;
    let mut rows = Vec::with_capacity((total_steps / steps_per_record + 1) as usize);
    let (mut vd, mut vq) = (0.0, 0.0);
    let record = |rows: &mut Vec<[f64; 6]>, t: f64, m: &Pmsm, w_ref: f64| {
        let s = m.state;
        rows.push([t, s.omega, s.theta, s.id, s.iq, w_ref]);
    };
    record(&mut rows, 0.0, &motor, foc.omega_ref);
    for k in 0..total_steps {
        let t = k as f64 * sc.dt;
        while next_action < actions.len() && actions[next_action].t <= t + 0.5 * sc.dt {
            if let Some(w) = actions[next_action].set_target_speed {
                foc.omega_ref = w;
            }
            next_action += 1;
        }
        if let Some(v) = sc.vq_open_loop {
            (vd, vq) = (0.0, v);
        } else if k % steps_per_ctrl == 0 {
            let s = motor.state;
            (vd, vq) = foc.update(s.id, s.iq, s.omega);
        }
        motor.step(vd, vq, sc.dt);
        if (k + 1) % steps_per_record == 0 {
            record(&mut rows, (k + 1) as f64 * sc.dt, &motor, foc.omega_ref);
        }
    }
    Ok(rows)
}
