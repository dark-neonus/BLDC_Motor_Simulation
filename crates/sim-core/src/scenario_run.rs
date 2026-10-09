//! Scenario runner (P04.T13): drives an engine built from a scene through a scenario
//! timeline, records signals and evaluates `assert` actions at their time or over their
//! window. Commands at an instant are applied after stepping to it (as the skeleton
//! runner does; EQ-NUM-02 ordering). Setpoints are load-side (signals.md); the
//! controller applies the gear ratio.

use sim_model::param::Param;
use sim_model::scenario::{Action, Assert, Op, Scenario, TargetKind};
use sim_model::units::{Kind, kind_of_unit};

use crate::build::BuiltScene;
use crate::engine::commands::{ChangeSource, EngineCommand, EngineEvent};
use crate::engine::time::SimTime;
use crate::skeleton::scenario::COLUMNS;

const OMEGA_REF: &str = "ctrl.omega_ref";
/// Ramp resolution: one setpoint step per millisecond.
const RAMP_STEP: f64 = 1e-3;

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct AssertResult {
    pub signal: String,
    pub op: Op,
    pub value: f64,
    pub tol: f64,
    pub t_start: f64,
    pub t_end: f64,
    pub passed: bool,
    /// The first failing sample, or the last checked one if all passed (NaN if never checked).
    pub observed: f64,
    pub t_observed: f64,
    pub message: Option<String>,
    /// Why the assert failed without a failing sample (not evaluated, window cut short).
    pub note: Option<String>,
}

#[derive(Debug, Default)]
pub struct RunOutput {
    pub columns: Vec<String>,
    pub rows: Vec<Vec<f64>>,
    pub asserts: Vec<AssertResult>,
    pub warnings: Vec<String>,
    pub snapshots: Vec<(String, Vec<u8>)>,
}

#[derive(Debug, Clone)]
enum Ev {
    Set(String, serde_json::Value),
    Signal(String, f64),
    Snapshot(String),
    AssertStart(usize),
    AssertEnd(usize),
}

fn secs(p: &Param, what: &str) -> Result<f64, String> {
    p.value.si(Kind::Time).map_err(|e| format!("{what}: {e}"))
}

fn holds(op: Op, v: f64, x: f64, tol: f64) -> bool {
    match op {
        Op::Lt => v < x + tol,
        Op::Le => v <= x + tol,
        Op::Gt => v > x - tol,
        Op::Ge => v >= x - tol,
        Op::Eq => (v - x).abs() <= tol,
        Op::Ne => (v - x).abs() > tol,
    }
}

/// The scheduled speed reference: a linear segment (t0, v0) → (t1, v1).
#[derive(Clone, Copy)]
struct Ramp {
    t0: f64,
    v0: f64,
    t1: f64,
    v1: f64,
}

impl Ramp {
    fn at(&self, t: f64) -> f64 {
        if t >= self.t1 || self.t1 <= self.t0 {
            self.v1
        } else if t <= self.t0 {
            self.v0
        } else {
            self.v0 + (self.v1 - self.v0) * (t - self.t0) / (self.t1 - self.t0)
        }
    }
}

struct Schedule {
    events: Vec<(f64, Ev)>,
    end: f64,
    warnings: Vec<String>,
    checks: Vec<Assert>,
    /// Planned [start, end] per assert.
    windows: Vec<(f64, f64)>,
}

/// Turn the sequential timeline into timed events.
fn schedule(sc: &Scenario, asserts: bool) -> Result<Schedule, String> {
    let mut s = Schedule {
        events: vec![],
        end: 0.0,
        warnings: vec![],
        checks: vec![],
        windows: vec![],
    };
    let mut t = 0.0_f64;
    let mut ramp = Ramp {
        t0: 0.0,
        v0: 0.0,
        t1: 0.0,
        v1: 0.0,
    };
    // A new reference at `t` replaces whatever the previous ramp still had scheduled.
    let restart = |s: &mut Schedule, t: f64| {
        s.events
            .retain(|(te, e)| !(matches!(e, Ev::Signal(p, _) if p == OMEGA_REF) && *te > t));
    };
    for (i, a) in sc.timeline.iter().enumerate() {
        let at = format!("timeline[{i}]");
        match a {
            Action::Wait(d) => {
                let d = secs(d, &at)?;
                if d < 0.0 {
                    return Err(format!("{at}: wait must not be negative"));
                }
                t += d;
            }
            Action::Stop => {
                s.end = t;
                return Ok(s);
            }
            Action::Set { path, value } => {
                if path == OMEGA_REF
                    && let Some(v) = value.as_f64()
                {
                    restart(&mut s, t);
                    ramp = Ramp {
                        t0: t,
                        v0: v,
                        t1: t,
                        v1: v,
                    };
                }
                s.events.push((t, Ev::Set(path.clone(), value.clone())));
            }
            Action::Target {
                kind,
                value,
                ramp: r,
            } => {
                if *kind != TargetKind::Velocity {
                    return Err(format!(
                        "{at}: {kind:?} targets need the position/torque controllers (P09); the current engine supports velocity"
                    ));
                }
                let w = value
                    .value
                    .si(Kind::AngularVelocity)
                    .map_err(|e| format!("{at}: {e}"))?;
                let r = r.as_ref().map(|r| secs(r, &at)).transpose()?.unwrap_or(0.0);
                if r < 0.0 {
                    return Err(format!("{at}: ramp must not be negative"));
                }
                restart(&mut s, t);
                let start = ramp.at(t);
                ramp = Ramp {
                    t0: t,
                    v0: start,
                    t1: t + r,
                    v1: w,
                };
                let n = (r / RAMP_STEP).ceil().max(1.0) as usize;
                for k in 1..=n {
                    let tk = t + r * k as f64 / n as f64;
                    s.events
                        .push((tk, Ev::Signal(OMEGA_REF.into(), ramp.at(tk))));
                }
                s.end = s.end.max(t + r);
            }
            Action::Fault { id, .. } => s.warnings.push(format!(
                "{at}: fault `{id}` ignored (fault injection arrives in P10)"
            )),
            Action::Disturbance { .. } => s.warnings.push(format!(
                "{at}: disturbance ignored (load torque input arrives in P05)"
            )),
            Action::Snapshot(name) => s.events.push((t, Ev::Snapshot(name.clone()))),
            Action::Assert(a) if asserts => {
                let t0 =
                    a.at.as_ref()
                        .map(|p| secs(p, &at))
                        .transpose()?
                        .unwrap_or(t);
                let w = a
                    .window
                    .as_ref()
                    .map(|p| secs(p, &at))
                    .transpose()?
                    .unwrap_or(0.0);
                if w < 0.0 || t0 < 0.0 {
                    return Err(format!("{at}: assert time and window must not be negative"));
                }
                let i = s.checks.len();
                s.events.push((t0, Ev::AssertStart(i)));
                if w > 0.0 {
                    s.events.push((t0 + w, Ev::AssertEnd(i)));
                }
                s.end = s.end.max(t0 + w);
                s.checks.push(a.clone());
                s.windows.push((t0, t0 + w));
            }
            Action::Assert(_) => {}
        }
        s.end = s.end.max(t);
    }
    Ok(s)
}

pub fn run(sc: &Scenario, mut b: BuiltScene, asserts: bool) -> Result<RunOutput, String> {
    let Schedule {
        mut events,
        end,
        warnings,
        checks,
        windows,
    } = schedule(sc, asserts)?;
    events.sort_by(|x, y| x.0.total_cmp(&y.0)); // stable: same-time events keep timeline order
    // A `stop` ends the run; nothing scheduled after it happens.
    events.retain(|(t, _)| *t <= end + 1e-12);
    let sample = match &sc.sample {
        Some(p) => secs(p, "sample")?,
        None => b.ctrl_dt,
    };
    if sample.is_nan() || sample <= 0.0 {
        return Err("sample must be > 0".into());
    }
    let names: Vec<String> = if sc.record.is_empty() {
        COLUMNS[1..].iter().map(|s| (*s).to_owned()).collect()
    } else {
        sc.record.clone()
    };
    let e = &mut b.engine;
    let ids = names
        .iter()
        .map(|n| e.bus.id(n).map_err(|err| format!("record: {err}")))
        .collect::<Result<Vec<_>, _>>()?;
    let check_ids = checks
        .iter()
        .map(|a| e.bus.id(&a.signal).map_err(|err| format!("assert: {err}")))
        .collect::<Result<Vec<_>, _>>()?;
    // Assert values and tolerances in the signal's own unit kind (CONVENTIONS §4).
    let resolved: Vec<(f64, f64)> = checks
        .iter()
        .zip(&check_ids)
        .map(|(a, &id)| {
            let kind = kind_of_unit(&e.bus.meta(id).unit).unwrap_or(Kind::Dimensionless);
            let v = a
                .value
                .value
                .si(kind)
                .map_err(|err| format!("assert `{}` value: {err}", a.signal))?;
            let tol = match &a.tol {
                Some(t) => t
                    .value
                    .si(kind)
                    .map_err(|err| format!("assert `{}` tol: {err}", a.signal))?,
                None => 0.0,
            };
            Ok((v, tol))
        })
        .collect::<Result<_, String>>()?;

    let mut out = RunOutput {
        columns: std::iter::once("t".to_owned()).chain(names).collect(),
        warnings,
        ..Default::default()
    };
    let mut results: Vec<Option<AssertResult>> = vec![None; checks.len()];
    let mut active: Vec<usize> = Vec::new();
    let eval = |i: usize, t: f64, v: f64, results: &mut Vec<Option<AssertResult>>| {
        let a = &checks[i];
        let (value, tol) = resolved[i];
        let ok = holds(a.op, v, value, tol);
        let r = results[i].get_or_insert_with(|| AssertResult {
            signal: a.signal.clone(),
            op: a.op,
            value,
            tol,
            t_start: t,
            t_end: t,
            passed: true,
            observed: v,
            t_observed: t,
            message: a.message.clone(),
            note: None,
        });
        r.t_end = t;
        if r.passed {
            (r.observed, r.t_observed, r.passed) = (v, t, ok);
        }
    };

    // ceil: the last sample is at or after `end`, so every event is processed.
    let n = (end / sample - 1e-9).ceil().max(0.0) as usize;
    let mut next = 0;
    for k in 0..=n {
        let ts = k as f64 * sample;
        while next < events.len() && events[next].0 <= ts + 1e-12 {
            let (t, ev) = events[next].clone();
            next += 1;
            e.step_until(SimTime::from_secs_f64(t))
                .map_err(|err| err.to_string())?;
            match ev {
                Ev::Set(path, value) => {
                    let cmds = if e.bus.id(&path).is_ok() {
                        let v = value
                            .as_f64()
                            .ok_or(format!("set `{path}`: signal values are SI numbers"))?;
                        vec![EngineCommand::SetSignal {
                            path,
                            value: v,
                            source: ChangeSource::Scenario,
                        }]
                    } else {
                        b.model.edit(&path, value, ChangeSource::Scenario)?
                    };
                    cmds.into_iter().for_each(|c| e.queue(c));
                }
                Ev::Signal(path, v) => e.queue(EngineCommand::SetSignal {
                    path,
                    value: v,
                    source: ChangeSource::Scenario,
                }),
                Ev::Snapshot(name) => out.snapshots.push((
                    name,
                    e.snapshot().to_bytes().map_err(|err| err.to_string())?,
                )),
                Ev::AssertStart(i) => {
                    eval(i, t, e.bus.get(check_ids[i]), &mut results);
                    if windows[i].1 > windows[i].0 {
                        active.push(i);
                    }
                }
                Ev::AssertEnd(i) => {
                    eval(i, t, e.bus.get(check_ids[i]), &mut results);
                    active.retain(|&j| j != i);
                }
            }
            e.apply_pending();
            for x in e.drain_events() {
                if let EngineEvent::CommandRejected { reason, .. } = x {
                    return Err(format!("t = {t} s: {reason}"));
                }
            }
        }
        e.step_until(SimTime::from_secs_f64(ts))
            .map_err(|err| err.to_string())?;
        for &i in &active {
            eval(i, ts, e.bus.get(check_ids[i]), &mut results);
        }
        out.rows.push(
            std::iter::once(ts)
                .chain(ids.iter().map(|&id| e.bus.get(id)))
                .collect(),
        );
    }
    // Asserts that never ran, or whose window a `stop` cut short, fail explicitly.
    out.asserts = results
        .into_iter()
        .enumerate()
        .map(|(i, r)| {
            let a = &checks[i];
            let (t0, t1) = windows[i];
            let mut r = r.unwrap_or_else(|| AssertResult {
                signal: a.signal.clone(),
                op: a.op,
                value: resolved[i].0,
                tol: resolved[i].1,
                t_start: t0,
                t_end: t1,
                passed: false,
                observed: f64::NAN,
                t_observed: f64::NAN,
                message: a.message.clone(),
                note: Some("not evaluated (after the end of the run)".into()),
            });
            if t1 > end + 1e-12 && r.note.is_none() {
                r.passed = false;
                r.note = Some(format!("window to {t1} s cut short by `stop` at {end} s"));
            }
            r
        })
        .collect();
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build::{build_engine, pmsm_from};
    use sim_model::io::parse_yaml;
    use sim_model::library::Library;

    fn go(yaml: &str) -> RunOutput {
        let dir =
            std::env::temp_dir().join(format!("bldc-srun-{}-{}", std::process::id(), yaml.len()));
        let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
        let sc: Scenario = parse_yaml(yaml, "t").unwrap();
        let scene = sc
            .scene
            .resolve(&lib, "scene")
            .unwrap()
            .resolve(&lib)
            .unwrap();
        let out = run(&sc, build_engine(&scene).unwrap(), true).unwrap();
        let _ = std::fs::remove_dir_all(dir);
        out
    }

    const HEAD: &str =
        "schema_version: 1\nname: t\nscene: { preset: \"builtin:scenes/gimbal-hold\" }\n";

    #[test]
    fn assert_after_last_sample_is_evaluated() {
        let o = go(&format!(
            "{HEAD}sample: 0.3 s\ntimeline:\n  - wait: 0.7 s\n  - assert: {{ signal: motor.omega, op: gt, value: 1000 }}\n"
        ));
        assert_eq!(o.asserts.len(), 1);
        assert!(!o.asserts[0].passed);
    }

    #[test]
    fn stop_cuts_a_window_short_and_fails_it() {
        let o = go(&format!(
            "{HEAD}sample: 1 ms\ntimeline:\n  - wait: 0.1 s\n  - assert: {{ signal: motor.omega, op: ge, value: -1, window: 2 s }}\n  - wait: 0.1 s\n  - stop\n"
        ));
        assert!(!o.asserts[0].passed);
        assert!(o.asserts[0].note.as_deref().unwrap_or("").contains("stop"));
        assert!(
            (o.rows.last().unwrap()[0] - 0.2).abs() < 1e-9,
            "run ends at the stop"
        );
    }

    #[test]
    fn overlapping_ramps_restart_from_the_current_reference() {
        let o = go(&format!(
            "{HEAD}sample: 1 ms\nrecord: [ctrl.omega_ref]\ntimeline:\n  - target: {{ kind: velocity, value: 10 rad/s, ramp: 1 s }}\n  - wait: 0.5 s\n  - target: {{ kind: velocity, value: 0 rad/s, ramp: 0.5 s }}\n  - wait: 0.6 s\n"
        ));
        let r: Vec<(f64, f64)> = o.rows.iter().map(|x| (x[0], x[1])).collect();
        let peak = r.iter().map(|x| x.1).fold(f64::MIN, f64::max);
        // 1 ms ramp steps: the reference peaks at the ramp value at t = 0.5 s (5 rad/s).
        assert!((peak - 5.0).abs() < 0.02, "peak {peak}");
        let after: Vec<f64> = r.iter().filter(|x| x.0 >= 0.5).map(|x| x.1).collect();
        assert!(
            after.windows(2).all(|w| w[1] <= w[0] + 1e-12),
            "non-monotonic after the second target"
        );
        assert_eq!(*after.last().unwrap(), 0.0);
    }

    #[test]
    fn setpoints_are_load_side_with_a_gearbox() {
        let o = go(
            "schema_version: 1\nname: t\nscene: { preset: \"builtin:scenes/arm-servo\" }\nsample: 1 ms\nrecord: [ctrl.omega_ref, motor.omega]\ntimeline:\n  - target: { kind: velocity, value: 2 rad/s }\n  - wait: 1 s\n",
        );
        let last = o.rows.last().unwrap();
        assert_eq!(last[1], 2.0, "ctrl.omega_ref holds the load-side value");
        // Motor side = N · load side = 9 · 2; 2 % covers the PI settling after 1 s.
        assert!(
            (last[2] / 18.0 - 1.0).abs() < 0.02,
            "motor.omega {}",
            last[2]
        );
    }

    #[test]
    fn motor_constant_overrides_reach_the_engine() {
        let dir = std::env::temp_dir().join(format!("bldc-ovr2-{}", std::process::id()));
        let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
        let mk = |ovr: &str| {
            let sc: Scenario = parse_yaml(
                &format!("schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: {{ preset: \"builtin:motors/4108-outrunner\"{ovr} }}\n  inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n  supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n  controller: {{ preset: \"builtin:controllers/foc-default\" }}\ntimeline: []\n"),
                "t",
            )
            .unwrap();
            let s = sc
                .scene
                .resolve(&lib, "scene")
                .unwrap()
                .resolve(&lib)
                .unwrap();
            pmsm_from(&build_engine(&s).unwrap().model.motor, 0.0)
                .unwrap()
                .lambda
        };
        let (base, fast) = (mk(""), mk(", overrides: { electrical.kv: 270 rpm/V }"));
        // λ ∝ 1/Kv (EQ-CONV-11): 27 → 270 rpm/V divides λ by 10.
        assert!((base / fast - 10.0).abs() < 1e-9, "{base} {fast}");
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn cogging_follows_the_fidelity_tier() {
        let dir = std::env::temp_dir().join(format!("bldc-cog-{}", std::process::id()));
        let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
        let scene: sim_model::scene::Scene =
            parse_yaml(&lib.load("builtin:scenes/gimbal-hold").unwrap(), "s").unwrap();
        let mut r = scene.resolve(&lib).unwrap();
        // Default tier (Standard) enables cogging (EQ-NUM-07 table).
        let b = build_engine(&r).unwrap();
        assert!(b.engine.bus.id("motor.torque_cog").is_ok());
        r.fidelity = Some(sim_model::params::FidelityParams {
            tier: sim_model::params::TierParam::Ideal,
            inverter_mode: None,
            dt_max: None,
        });
        assert!(
            build_engine(&r)
                .unwrap()
                .engine
                .bus
                .id("motor.torque_cog")
                .is_err()
        );
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn assert_values_take_quantities_in_the_signal_unit() {
        // motor.omega is rad/s: 5 rad/s = 47.746 rpm; the tolerance is in rpm too.
        let o = go(&format!(
            "{HEAD}sample: 1 ms\ntimeline:\n  - target: {{ kind: velocity, value: 5 rad/s, ramp: 0.1 s }}\n  - wait: 0.6 s\n  - assert: {{ signal: motor.omega, op: eq, value: 47.746 rpm, tol: 2 rpm }}\n  - assert: {{ signal: motor.omega, op: gt, value: 100 rpm }}\n"
        ));
        assert!(o.asserts[0].passed, "{:?}", o.asserts[0]);
        assert!((o.asserts[0].value - 5.0).abs() < 1e-4, "converted to SI");
        assert!((o.asserts[0].tol - 2.0 * std::f64::consts::PI / 30.0).abs() < 1e-12);
        assert!(!o.asserts[1].passed, "100 rpm is not reached");
    }
}
