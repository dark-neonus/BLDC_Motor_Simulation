//! Scenario runner (P04.T13): drives an engine built from a scene through a scenario
//! timeline, records signals and evaluates `assert` actions at their time or over their
//! window. Commands at an instant are applied after stepping to it (as the skeleton
//! runner does; EQ-NUM-02 ordering).

use sim_model::param::Param;
use sim_model::scenario::{Action, Assert, Op, Scenario, TargetKind};
use sim_model::units::Kind;

use crate::build::BuiltScene;
use crate::engine::commands::{ChangeSource, EngineCommand, EngineEvent};
use crate::engine::time::SimTime;
use crate::skeleton::scenario::COLUMNS;

#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct AssertResult {
    pub signal: String,
    pub op: Op,
    pub value: f64,
    pub tol: f64,
    pub t_start: f64,
    pub t_end: f64,
    pub passed: bool,
    /// The first failing sample, or the last checked one if all passed.
    pub observed: f64,
    pub t_observed: f64,
    pub message: Option<String>,
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

/// Timed events, end time, warnings and the asserts to check.
type Schedule = (Vec<(f64, Ev)>, f64, Vec<String>, Vec<Assert>);

/// Turn the sequential timeline into timed events.
fn schedule(sc: &Scenario, b: &BuiltScene, asserts: bool) -> Result<Schedule, String> {
    let mut ev = Vec::new();
    let mut warn = Vec::new();
    let mut checks = Vec::new();
    let (mut t, mut end, mut omega_ref) = (0.0_f64, 0.0_f64, 0.0_f64);
    for (i, a) in sc.timeline.iter().enumerate() {
        let at = format!("timeline[{i}]");
        match a {
            Action::Wait(d) => t += secs(d, &at)?,
            Action::Stop => {
                end = t;
                return Ok((ev, end, warn, checks));
            }
            Action::Set { path, value } => ev.push((t, Ev::Set(path.clone(), value.clone()))),
            Action::Target { kind, value, ramp } => {
                if *kind != TargetKind::Velocity {
                    return Err(format!(
                        "{at}: {kind:?} targets need the position/torque controllers (P09); the current engine supports velocity"
                    ));
                }
                let w = b.ratio
                    * value
                        .value
                        .si(Kind::AngularVelocity)
                        .map_err(|e| format!("{at}: {e}"))?;
                let r = ramp
                    .as_ref()
                    .map(|r| secs(r, &at))
                    .transpose()?
                    .unwrap_or(0.0);
                let n = (r / 1e-3).ceil().max(1.0) as usize;
                for k in 1..=n {
                    let f = k as f64 / n as f64;
                    ev.push((
                        t + r * f,
                        Ev::Signal("ctrl.omega_ref".into(), omega_ref + (w - omega_ref) * f),
                    ));
                }
                omega_ref = w;
                end = end.max(t + r);
            }
            Action::Fault { id, .. } => warn.push(format!(
                "{at}: fault `{id}` ignored (fault injection arrives in P10)"
            )),
            Action::Disturbance { .. } => warn.push(format!(
                "{at}: disturbance ignored (load torque input arrives in P05)"
            )),
            Action::Snapshot(name) => ev.push((t, Ev::Snapshot(name.clone()))),
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
                ev.push((t0, Ev::AssertStart(checks.len())));
                if w > 0.0 {
                    ev.push((t0 + w, Ev::AssertEnd(checks.len())));
                }
                end = end.max(t0 + w);
                checks.push(a.clone());
            }
            Action::Assert(_) => {}
        }
        end = end.max(t);
    }
    Ok((ev, end, warn, checks))
}

pub fn run(sc: &Scenario, mut b: BuiltScene, asserts: bool) -> Result<RunOutput, String> {
    let (mut events, end, warnings, checks) = schedule(sc, &b, asserts)?;
    events.sort_by(|x, y| x.0.total_cmp(&y.0)); // stable: same-time events keep timeline order
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

    let mut out = RunOutput {
        columns: std::iter::once("t".to_owned()).chain(names).collect(),
        warnings,
        ..Default::default()
    };
    let mut results: Vec<Option<AssertResult>> = vec![None; checks.len()];
    let mut active: Vec<usize> = Vec::new();
    let eval = |i: usize, t: f64, v: f64, results: &mut Vec<Option<AssertResult>>| {
        let a = &checks[i];
        let ok = holds(a.op, v, a.value, a.tol);
        let r = results[i].get_or_insert_with(|| AssertResult {
            signal: a.signal.clone(),
            op: a.op,
            value: a.value,
            tol: a.tol,
            t_start: t,
            t_end: t,
            passed: true,
            observed: v,
            t_observed: t,
            message: a.message.clone(),
        });
        r.t_end = t;
        if r.passed {
            (r.observed, r.t_observed, r.passed) = (v, t, ok);
        }
    };

    let n = (end / sample).round() as usize;
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
                        b.model.edit(&path, value)?
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
                    if events
                        .iter()
                        .any(|(_, x)| matches!(x, Ev::AssertEnd(j) if *j == i))
                    {
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
    out.asserts = results.into_iter().flatten().collect();
    Ok(out)
}
