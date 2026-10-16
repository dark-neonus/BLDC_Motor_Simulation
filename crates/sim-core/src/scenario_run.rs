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
const TORQUE_REF: &str = "ctrl.torque_ref";
const THETA_REF: &str = "ctrl.theta_ref";
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
    /// Load-shaft torque pulse (torque, duration) (EQ-MECH-08).
    Disturbance(f64, f64),
    /// Controller mode from the target kind: which cascade loops run (EQ-CTRL-04).
    Mode(TargetKind),
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
    let mut mode: Option<TargetKind> = None;
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
                if mode != Some(*kind) {
                    s.events.push((t, Ev::Mode(*kind)));
                    mode = Some(*kind);
                }
                if *kind == TargetKind::Position {
                    // Load-side angle; the position loop's speed limit shapes the move.
                    if r.is_some() {
                        return Err(format!("{at}: position targets take no ramp"));
                    }
                    let th = value
                        .value
                        .si(Kind::Angle)
                        .map_err(|e| format!("{at}: {e}"))?;
                    restart(&mut s, t);
                    s.events.push((t, Ev::Signal(THETA_REF.into(), th)));
                    continue;
                }
                if *kind == TargetKind::Torque {
                    // Load-side torque → i_q in the interim FOC; no ramp.
                    if r.is_some() {
                        return Err(format!("{at}: torque targets take no ramp"));
                    }
                    let tq = value
                        .value
                        .si(Kind::Torque)
                        .map_err(|e| format!("{at}: {e}"))?;
                    s.events.push((t, Ev::Signal(TORQUE_REF.into(), tq)));
                    continue;
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
            Action::Disturbance { torque, duration } => {
                let tq = torque
                    .value
                    .si(Kind::Torque)
                    .map_err(|e| format!("{at}: disturbance torque: {e}"))?;
                let d = secs(duration, &format!("{at}: disturbance duration"))?;
                s.events.push((t, Ev::Disturbance(tq, d)));
                s.end = s.end.max(t + d);
            }
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
                Ev::Mode(m) => {
                    let has_position = e.get_param("controller.position.enabled").is_some();
                    let (position, velocity) = match m {
                        TargetKind::Velocity => (0.0, 1.0),
                        TargetKind::Torque => (0.0, 0.0),
                        TargetKind::Position if has_position => (1.0, 1.0),
                        TargetKind::Position => {
                            return Err(
                                "position targets need a scene with a position loop (controller.position)"
                                    .into(),
                            );
                        }
                    };
                    let speed = [
                        "controller.velocity.enabled",
                        "controller.speed_loop.enabled",
                    ]
                    .into_iter()
                    .find(|p| e.get_param(p).is_some());
                    let mut set = |path: &str, value: f64| {
                        e.queue(EngineCommand::SetParam {
                            path: path.into(),
                            value,
                            source: ChangeSource::Scenario,
                        })
                    };
                    if has_position {
                        set("controller.position.enabled", position);
                    }
                    // FOC: the velocity loop; six-step: its speed loop (no torque mode).
                    match (speed, m) {
                        (Some(p), _) if p.ends_with("velocity.enabled") => set(p, velocity),
                        (Some(p), TargetKind::Velocity) => set(p, 1.0),
                        // Open loop: ctrl.omega_ref drives it directly.
                        (None, TargetKind::Velocity) => {}
                        _ => {
                            return Err(format!(
                                "{m:?} targets are not supported by this scene's controller"
                            ));
                        }
                    }
                }
                Ev::Disturbance(torque, duration) => e.queue(EngineCommand::Disturbance {
                    torque,
                    duration,
                    source: ChangeSource::Scenario,
                }),
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
    fn the_scene_emf_shape_reaches_the_motor() {
        // Spin at a steady 20 rad/s and compare the e_a peak with the fundamental λ·p·ω:
        // sinusoidal → 1, trapezoid (120° flat top, fundamental normalised to 1) → 1/b₁
        // (EQ-MOT-03). 1/b₁(120°) ≈ 0.82, well apart from 1.
        let peak_ratio = |ovr: &str| {
            let o = go(&format!(
                "schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: {{ preset: \"builtin:motors/4108-outrunner\"{ovr} }}\n  inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n  supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n  controller: {{ preset: \"builtin:controllers/foc-default\" }}\nsample: 20 us\nrecord: [motor.e_a, motor.omega_e]\ntimeline:\n  - target: {{ kind: velocity, value: 20 rad/s, ramp: 0.1 s }}\n  - wait: 0.6 s\n"
            ));
            let tail: Vec<&Vec<f64>> = o.rows.iter().filter(|r| r[0] >= 0.4).collect();
            let peak = tail.iter().map(|r| r[1].abs()).fold(0.0, f64::max);
            let we = tail.iter().map(|r| r[2]).sum::<f64>() / tail.len() as f64;
            (peak, we)
        };
        let (sin_peak, we1) = peak_ratio("");
        let (trap_peak, we2) = peak_ratio(
            ", overrides: { magnetic.emf_shape: { kind: trapezoidal, flat_top: 120 deg } }",
        );
        assert!((we1 / we2 - 1.0).abs() < 0.02, "{we1} {we2}");
        let expect =
            1.0 / crate::physics::motor::backemf::trap_b1(std::f64::consts::PI * 2.0 / 3.0);
        // 2 %: speed ripple and the 20 µs sampling of the peak.
        assert!(
            (trap_peak / sin_peak - expect).abs() < 0.02,
            "ratio {} expected {expect}",
            trap_peak / sin_peak
        );
    }

    /// P06.T10: the production build wires iron loss (EQ-MOT-09) from the motor's k_hy /
    /// k_ed at the Standard tier: the loss is booked and the balance closes.
    #[test]
    fn production_build_with_iron_loss() {
        let o = go(
            "schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: { preset: \"builtin:motors/4108-outrunner\", overrides: { magnetic.k_hy: 2e-4, magnetic.k_ed: 1e-6 } }\n  inverter: { preset: \"builtin:inverters/small-24v-10a\" }\n  supply: { preset: \"builtin:supplies/psu-24v-5a\" }\n  controller: { preset: \"builtin:controllers/foc-default\" }\nsample: 10 ms\nrecord: [energy.loss.iron, motor.p_fe, energy.residual]\ntimeline:\n  - target: { kind: velocity, value: 30 rad/s, ramp: 0.1 s }\n  - wait: 0.4 s\n",
        );
        let last = o.rows.last().unwrap();
        assert!(last[1] > 0.0 && last[2] > 0.0, "iron loss booked: {last:?}");
        assert!(last[3].abs() < 1e-3, "residual {}", last[3]);
    }

    /// Review B1: a backlash gearbox with no inertia behind it (no `j_out`, no load)
    /// builds rigid instead of dividing by a zero load-side inertia.
    #[test]
    fn backlash_without_load_inertia_runs_rigid() {
        let o = go(
            "schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: { preset: \"builtin:motors/6020-outrunner\" }\n  gearbox: { preset: \"builtin:gearboxes/planetary-9\" }\n  inverter: { preset: \"builtin:inverters/small-24v-10a\" }\n  supply: { preset: \"builtin:supplies/psu-24v-5a\" }\n  controller: { preset: \"builtin:controllers/foc-default\" }\nsample: 10 ms\nrecord: [load.omega]\ntimeline:\n  - target: { kind: velocity, value: 2 rad/s, ramp: 0.05 s }\n  - wait: 0.2 s\n",
        );
        assert!(o.rows.last().unwrap()[1].is_finite());
    }

    /// Review M1: `initial.theta` sets the electrical angle too, so the motor starts with
    /// zero current (it started 180° off for θ_L = 90°, p = 10: i_d = −2λ/L_d).
    #[test]
    fn initial_angle_starts_with_aligned_flux() {
        let o = go(
            "schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: { preset: \"builtin:motors/6020-outrunner\" }\n  load: { preset: \"builtin:loads/arm-20cm-0.5kg\" }\n  inverter: { preset: \"builtin:inverters/small-24v-10a\" }\n  supply: { preset: \"builtin:supplies/psu-24v-5a\" }\n  controller: { preset: \"builtin:controllers/foc-default\" }\n  initial: { theta: 90 deg }\nsample: 0.1 ms\nrecord: [motor.i_d]\ntimeline:\n  - wait: 0.5 ms\n",
        );
        for r in &o.rows {
            assert!(r[1].abs() < 0.05, "i_d = {} at t = {}", r[1], r[0]);
        }
    }

    /// P07.T12: scenes drive the motor through SVPWM → inverter → DC bus with the scene's
    /// PSU, averaged by default and switching on request; energy closes through the chain.
    /// V-INV-001: in loaded FOC steady state (0.45 N·m brake) the two modes agree on the
    /// (PWM-center-sampled, i.e. ripple-averaged) currents within 1 % of rated (5 A).
    #[test]
    fn v_inv_001_scenes_drive_through_inverter_and_bus() {
        let mut iq = vec![];
        for mode in [
            "",
            "  fidelity: { inverter_mode: switching, dt_max: 0.5 us }\n",
        ] {
            let o = go(&format!(
                "schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: {{ preset: \"builtin:motors/4108-outrunner\" }}\n  load: {{ kind: brake, torque: 0.45 N*m }}\n  inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n  supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n  controller: {{ preset: \"builtin:controllers/foc-default\" }}\n{mode}sample: 1 ms\nrecord: [motor.omega, bus.v, inverter.p_loss, energy.residual, motor.i_d, motor.i_q]\ntimeline:\n  - target: {{ kind: velocity, value: 20 rad/s, ramp: 0.05 s }}\n  - wait: 0.1 s\n"
            ));
            let last = o.rows.last().unwrap();
            assert!((last[1] - 20.0).abs() < 1.0, "{mode}: ω {}", last[1]);
            // A brief regen at the end of the ramp can lift a blocking PSU bus above 24 V.
            assert!(last[2] > 22.0 && last[2] < 25.0, "{mode}: bus {}", last[2]);
            assert!(last[3] > 0.0, "{mode}: inverter loss");
            assert!(last[4].abs() < 1e-3, "{mode}: residual {}", last[4]);
            let tail = &o.rows[o.rows.len() - 20..];
            let mean = |k: usize| tail.iter().map(|r| r[k]).sum::<f64>() / tail.len() as f64;
            iq.push((mean(5), mean(6)));
        }
        let ((d0, q0), (d1, q1)) = (iq[0], iq[1]);
        // A loaded operating point: the 0.45 N·m brake needs i_q ≈ T/K_t ≈ 1.5 A (K_t =
        // 1.5·p·λ ≈ 0.31 N·m/A; 2.5 A would exceed the V_bus/√3 limit with R = 5.6 Ω).
        assert!(q0 > 1.3 && q1 > 1.3, "i_q {q0} / {q1}");
        assert!(
            (d0 - d1).abs() < 0.05 && (q0 - q1).abs() < 0.05,
            "avg ({d0}, {q0}) vs sw ({d1}, {q1})"
        );
    }

    /// Resolve + build a scenario's scene, run it; also returns the shaft inertia J_eq.
    fn go_with_j(yaml: &str) -> (RunOutput, f64) {
        let dir =
            std::env::temp_dir().join(format!("bldc-gj-{}-{}", std::process::id(), yaml.len()));
        let lib = Library::new(None, dir.join("user"), vec![]).unwrap();
        let sc: Scenario = parse_yaml(yaml, "t").unwrap();
        let scene = sc
            .scene
            .resolve(&lib, "scene")
            .unwrap()
            .resolve(&lib)
            .unwrap();
        let b = build_engine(&scene).unwrap();
        let j = b.engine.get_param("motor.mechanical.j_rotor").unwrap()
            + b.engine.get_param("load.inertia").unwrap_or(0.0);
        let out = run(&sc, b, true).unwrap();
        let _ = std::fs::remove_dir_all(dir);
        (out, j)
    }

    fn brake_scene(chopper: &str) -> String {
        format!(
            "schema_version: 1\nname: t\nscene:\n  schema_version: 1\n  name: s\n  motor: {{ preset: \"builtin:motors/8010-outrunner\" }}\n  load: {{ kind: flywheel, inertia: 2.5e-4 kg*m^2 }}\n  inverter: {{ preset: \"builtin:inverters/small-24v-10a\" }}\n  supply: {{ preset: \"builtin:supplies/psu-24v-5a\" }}\n  bus: {{ capacitance: 470 uF{chopper} }}\n  controller: {{ preset: \"builtin:controllers/foc-default\" }}\n  initial: {{ omega: 60 rad/s }}\nsample: 0.01 ms\nrecord: [bus.v, motor.omega, energy.residual, energy.loss.brake, bus.p_brake, bus.chopper_on]\ntimeline:\n  - target: {{ kind: velocity, value: 0 rad/s }}\n  - wait: 0.3 s\n"
        )
    }

    /// V-SUP-002: braking a flywheel (low-R 8010 motor at 60 rad/s, so the machine
    /// regenerates rather than burning it all in copper) into a blocking PSU (no chopper) lifts the bus, but
    /// never above the lossless bound √(V₁² + Jω₀²/C); energy closes (rtol 1e-3).
    #[test]
    fn v_sup_002_regen_into_a_blocking_psu() {
        let (o, j) = go_with_j(&brake_scene(""));
        let v_max = o.rows.iter().map(|r| r[1]).fold(f64::MIN, f64::max);
        let bound = (24.0f64.powi(2) + j * 60.0 * 60.0 / 470e-6).sqrt();
        assert!(v_max > 24.5, "the bus rose: {v_max}");
        assert!(v_max <= bound, "{v_max} > {bound}");
        let last = o.rows.last().unwrap();
        assert!(last[2].abs() < 1.0, "stopped: ω {}", last[2]);
        assert!(o.rows.iter().all(|r| r[3].abs() < 1e-3), "residual");
    }

    /// V-SUP-003: the same with a chopper: the bus stays at V_on (events located: within
    /// 0.01 V, sampled every 10 µs), the resistor takes most of the kinetic energy, its
    /// booked energy matches ∫ p_brake dt, and energy closes.
    #[test]
    fn v_sup_003_regen_with_chopper() {
        let (o, j) = go_with_j(&brake_scene(
            ", chopper: { r_brake: 10 ohm, v_on: 25 V, v_off: 24.5 V }",
        ));
        let v_max = o.rows.iter().map(|r| r[1]).fold(f64::MIN, f64::max);
        assert!(v_max <= 25.0 + 0.01, "{v_max}");
        let last = o.rows.last().unwrap();
        let ke = 0.5 * j * 60.0 * 60.0;
        // The resistor takes most of the kinetic energy (the rest: copper, inverter).
        assert!(
            last[4] > 0.5 * ke && last[4] < ke,
            "brake {} of {ke} J",
            last[4]
        );
        // Independent of the energy book: ∫ p_brake dt (trapezoid over the 10 µs samples)
        // agrees within the sampling bound, half a sample at V_on²/R_b per chopper toggle.
        let trap: f64 = o
            .rows
            .windows(2)
            .map(|w| 0.5 * (w[0][5] + w[1][5]) * (w[1][0] - w[0][0]))
            .sum();
        let toggles = o.rows.windows(2).filter(|w| w[0][6] != w[1][6]).count();
        assert!(toggles > 2, "the chopper cycles: {toggles}");
        let bound = toggles as f64 * (25.0 * 25.0 / 10.0) * 0.5e-5;
        assert!(
            (trap - last[4]).abs() <= bound,
            "{trap} vs {} (±{bound})",
            last[4]
        );
        assert!(o.rows.iter().all(|r| r[3].abs() < 1e-3), "residual");
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
