//! The engine: continuous plant + discrete blocks on a multi-rate event schedule.
//!
//! Algorithm (EQ-NUM-02): at the current time, fire every block due now in
//! (priority, registration) order; then integrate the plant up to the next event
//! (or the target) with ≤ `dt_max` substeps, holding discrete outputs constant (ZOH).

use super::block::{DiscreteBlock, SimError, StepCtx};
use super::commands::{EngineCommand, EngineEvent};
use super::integrate::Rk4;
use super::plant::Plant;
use super::recorder::Recorder;
use super::signals::SignalBus;
use super::time::SimTime;
use crate::energy::{EnergyBook, EnergySignals, R_TOL_STANDARD};

/// "Never" for blocks with no upcoming event.
const NEVER: SimTime = SimTime(i64::MAX);

pub struct Engine {
    pub time: SimTime,
    pub plant: Plant,
    /// Global continuous state vector.
    pub x: Vec<f64>,
    pub bus: SignalBus,
    blocks: Vec<Box<dyn DiscreteBlock>>,
    next_fire: Vec<SimTime>,
    integrator: Rk4,
    /// Maximum integration substep [s] (EQ-NUM-04).
    pub dt_max: f64,
    /// Scratch: indices of blocks due now (no allocation per event).
    due: Vec<usize>,
    /// Commands waiting for the next event boundary.
    pending: Vec<EngineCommand>,
    /// Events produced since the last `drain_events`.
    events: Vec<EngineEvent>,
    /// Optional recorder, sampled after every accepted substep.
    pub recorder: Option<Recorder>,
    /// Energy accounting (present when the plant has modules).
    energy: Option<(EnergySignals, EnergyBook)>,
    // Scratch for state-event handling.
    x_save: Vec<f64>,
    g_prev: Vec<f64>,
    g_new: Vec<f64>,
}

impl Engine {
    /// Build an engine. The bus is frozen here; blocks first fire at their phase.
    pub fn new(
        mut plant: Plant,
        mut bus: SignalBus,
        blocks: Vec<Box<dyn DiscreteBlock>>,
        dt_max: f64,
    ) -> Self {
        plant.finalize_energy();
        let energy_sig = if plant.modules().is_empty() {
            None
        } else {
            EnergySignals::register(&mut bus).ok()
        };
        bus.freeze();
        let x = plant.initial_state();
        let n = x.len();
        let next_fire = blocks.iter().map(|b| first_fire(b.as_ref())).collect();
        let mut e = Self {
            time: SimTime::ZERO,
            plant,
            x,
            bus,
            due: Vec::with_capacity(blocks.len()),
            blocks,
            next_fire,
            integrator: Rk4::new(n),
            dt_max,
            pending: Vec::new(),
            events: Vec::new(),
            recorder: None,
            energy: energy_sig.map(|s| {
                (
                    s,
                    EnergyBook {
                        tol: R_TOL_STANDARD,
                        ..Default::default()
                    },
                )
            }),
            x_save: Vec::with_capacity(n),
            g_prev: Vec::new(),
            g_new: Vec::new(),
        };
        // Make the bus consistent with the initial state.
        e.plant.outputs(0.0, &e.x, &mut e.bus);
        if let Some((_, book)) = &mut e.energy {
            book.e_st0 = e.plant.stored_energy(&e.x, &e.bus);
            let _ = e.plant.take_external_energy();
        }
        e.update_energy();
        e
    }

    /// Recompute the energy signals (EQ-ENER-03).
    pub fn update_energy(&mut self) {
        if let Some((sig, book)) = &mut self.energy {
            book.update(&mut self.plant, &self.x, &mut self.bus, sig);
        }
    }

    /// Set the energy-residual tolerance (per tier).
    pub fn set_energy_tolerance(&mut self, tol: f64) {
        if let Some((_, book)) = &mut self.energy {
            book.tol = tol;
        }
    }

    /// Time of the next scheduled block event (or `None`).
    pub fn next_event_time(&self) -> Option<SimTime> {
        self.next_fire.iter().copied().filter(|&t| t != NEVER).min()
    }

    /// Fire all blocks due at the current time, in priority order.
    fn fire_due(&mut self) -> Result<(), SimError> {
        self.due.clear();
        for (i, &t) in self.next_fire.iter().enumerate() {
            if t == self.time {
                self.due.push(i);
            }
        }
        if self.due.is_empty() {
            return Ok(());
        }
        let blocks = &self.blocks;
        self.due.sort_by_key(|&i| (blocks[i].priority(), i));
        for k in 0..self.due.len() {
            let i = self.due[k];
            let mut ctx = StepCtx {
                t: self.time,
                bus: &mut self.bus,
            };
            self.blocks[i].step(&mut ctx)?;
            let b = &self.blocks[i];
            self.next_fire[i] = match b.period() {
                Some(p) => self.time + p,
                None => b
                    .next_event(self.time)
                    .filter(|&t| t > self.time)
                    .unwrap_or(NEVER),
            };
        }
        Ok(())
    }

    /// Advance to `target`, firing events on the way. Events due exactly at
    /// `target` fire at the start of the next call.
    pub fn step_until(&mut self, target: SimTime) -> Result<(), SimError> {
        while self.time < target {
            self.apply_pending();
            self.fire_due()?;
            let t_next = self.next_event_time().map_or(target, |t| t.min(target));
            let (t0, t1) = (self.time.as_secs_f64(), t_next.as_secs_f64());
            self.advance(t0, t1)?;
            self.update_energy();
            self.time = t_next;
        }
        Ok(())
    }

    /// Integrate the plant from `t0` to `t1` (no discrete events in between) in ≤ dt_max
    /// substeps, locating state events by bisection to 1 ns (EQ-NUM-05).
    fn advance(&mut self, t0: f64, t1: f64) -> Result<(), SimError> {
        // 0.1 ns: impact timing errors propagate through resets, so bisect 10× finer than the 1 ns spec.
        const T_TOL: f64 = 1e-10;
        const ZENO_EVENTS: usize = 100;
        let ne = self.plant.n_events();
        let mut t = t0;
        if ne > 0 {
            self.g_prev.resize(ne, 0.0);
            self.g_new.resize(ne, 0.0);
            self.plant
                .event_functions(t, &self.x, &self.bus, &mut self.g_prev);
        }
        let mut burst = (t0, 0usize); // Zeno guard: events within 1 µs of sim time
        while t < t1 {
            let remaining = t1 - t;
            let n = (remaining / self.dt_max).ceil().max(1.0);
            let h = remaining / n;
            self.x_save.clone_from(&self.x);
            self.integrator
                .step(&mut self.plant, &mut self.bus, t, &mut self.x, h);
            self.plant.outputs(t + h, &self.x, &mut self.bus);
            if ne > 0 {
                self.plant
                    .event_functions(t + h, &self.x, &self.bus, &mut self.g_new);
                let crossed = (0..ne).find(|&i| (self.g_prev[i] < 0.0) != (self.g_new[i] < 0.0));
                if let Some(_first) = crossed {
                    // Bisect the step size for the earliest crossing of any function.
                    let (mut lo, mut hi) = (0.0, h);
                    while hi - lo > T_TOL {
                        let mid = 0.5 * (lo + hi);
                        self.x.clone_from(&self.x_save);
                        self.integrator
                            .step(&mut self.plant, &mut self.bus, t, &mut self.x, mid);
                        self.plant.outputs(t + mid, &self.x, &mut self.bus);
                        self.plant
                            .event_functions(t + mid, &self.x, &self.bus, &mut self.g_new);
                        if (0..ne).any(|i| (self.g_prev[i] < 0.0) != (self.g_new[i] < 0.0)) {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    // Land just after the crossing.
                    self.x.clone_from(&self.x_save);
                    self.integrator
                        .step(&mut self.plant, &mut self.bus, t, &mut self.x, hi);
                    self.plant.outputs(t + hi, &self.x, &mut self.bus);
                    self.plant
                        .event_functions(t + hi, &self.x, &self.bus, &mut self.g_new);
                    t += hi;
                    for i in 0..ne {
                        if (self.g_prev[i] < 0.0) != (self.g_new[i] < 0.0) {
                            let rising = self.g_new[i] >= 0.0;
                            self.plant
                                .handle_event(i, rising, t, &mut self.x, &mut self.bus);
                        }
                    }
                    // Zeno guard.
                    if t - burst.0 > 1e-6 {
                        burst = (t, 0);
                    }
                    burst.1 += 1;
                    if burst.1 > ZENO_EVENTS {
                        return Err(SimError::Numerical {
                            t: SimTime::from_secs_f64(t),
                            msg: format!(
                                "more than {ZENO_EVENTS} state events within 1 µs (Zeno behaviour)"
                            ),
                        });
                    }
                    self.plant.outputs(t, &self.x, &mut self.bus);
                    self.plant
                        .event_functions(t, &self.x, &self.bus, &mut self.g_prev);
                    if let Some(rec) = &mut self.recorder {
                        rec.sample(t, &self.bus);
                    }
                    continue;
                }
                std::mem::swap(&mut self.g_prev, &mut self.g_new);
            }
            t += h;
            if let Some(rec) = &mut self.recorder {
                rec.sample(t, &self.bus);
            }
        }
        Ok(())
    }

    /// Advance by `dt`.
    pub fn step_time(&mut self, dt: SimTime) -> Result<(), SimError> {
        self.step_until(self.time + dt)
    }

    /// Advance to the next event, `n` times (single-stepping in the UI).
    pub fn step_events(&mut self, n: usize) -> Result<(), SimError> {
        for _ in 0..n {
            match self.next_event_time() {
                Some(t) if t > self.time => self.step_until(t)?,
                Some(_) => {
                    // An event is due now: fire it and move to the following one.
                    self.fire_due()?;
                    if let Some(t) = self.next_event_time() {
                        self.step_until(t)?;
                    }
                }
                None => return Ok(()),
            }
        }
        Ok(())
    }

    pub fn blocks(&self) -> &[Box<dyn DiscreteBlock>] {
        &self.blocks
    }

    /// Queue a command; it is applied at the next event boundary.
    pub fn queue(&mut self, cmd: EngineCommand) {
        self.pending.push(cmd);
    }

    /// Apply queued commands now (also called automatically by `step_until`).
    pub fn apply_pending(&mut self) {
        if self.pending.is_empty() {
            return;
        }
        let cmds = std::mem::take(&mut self.pending);
        for cmd in cmds {
            let ev = self.apply(cmd);
            self.events.push(ev);
        }
        // Keep the bus consistent with possibly changed plant parameters/states.
        self.plant
            .outputs(self.time.as_secs_f64(), &self.x, &mut self.bus);
        self.update_energy();
    }

    fn apply(&mut self, cmd: EngineCommand) -> EngineEvent {
        let t = self.time;
        let reject = |reason: String| EngineEvent::CommandRejected { t, reason };
        match cmd {
            EngineCommand::SetSignal {
                path,
                value,
                source,
            } => match self.bus.id(&path) {
                Ok(id) => {
                    let old = self.bus.get(id);
                    self.bus.set(id, value);
                    EngineEvent::SignalSet {
                        t,
                        path,
                        old,
                        new: value,
                        source,
                    }
                }
                Err(e) => reject(e.to_string()),
            },
            EngineCommand::SetParam {
                path,
                value,
                source,
            } => {
                let Some((owner, name)) = path.rsplit_once('.') else {
                    return reject(format!("invalid parameter path `{path}`"));
                };
                if !value.is_finite() {
                    return reject(format!("`{path}`: value must be finite"));
                }
                if let Some(i) = self.plant.modules().iter().position(|m| m.name() == owner) {
                    let (m, xs) = self.plant.module_mut(i, &mut self.x);
                    return match m.set_param(name, value, xs) {
                        Ok(old) => EngineEvent::ParamChanged {
                            t,
                            path,
                            old,
                            new: value,
                            source,
                        },
                        Err(e) => reject(format!("`{path}`: {e}")),
                    };
                }
                if let Some(b) = self.blocks.iter_mut().find(|b| b.id() == owner) {
                    return match b.set_param(name, value) {
                        Ok(old) => EngineEvent::ParamChanged {
                            t,
                            path,
                            old,
                            new: value,
                            source,
                        },
                        Err(e) => reject(format!("`{path}`: {e}")),
                    };
                }
                reject(format!("no module or block `{owner}` for `{path}`"))
            }
        }
    }

    /// Take the events produced so far.
    pub fn drain_events(&mut self) -> Vec<EngineEvent> {
        std::mem::take(&mut self.events)
    }

    pub(crate) fn next_fire_times(&self) -> &[SimTime] {
        &self.next_fire
    }

    pub(crate) fn set_next_fire_times(&mut self, t: &[SimTime]) {
        self.next_fire.copy_from_slice(t);
    }

    pub(crate) fn block_mut(&mut self, i: usize) -> &mut dyn DiscreteBlock {
        self.blocks[i].as_mut()
    }

    /// Read a live parameter by path.
    pub fn get_param(&self, path: &str) -> Option<f64> {
        let (owner, name) = path.rsplit_once('.')?;
        if let Some(m) = self.plant.modules().iter().find(|m| m.name() == owner) {
            return m.get_param(name);
        }
        self.blocks
            .iter()
            .find(|b| b.id() == owner)?
            .get_param(name)
    }
}

fn first_fire(b: &dyn DiscreteBlock) -> SimTime {
    match b.period() {
        Some(_) => b.phase(),
        None => b.next_event(SimTime(-1)).unwrap_or(NEVER),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::signals::{SignalId, SignalKind};
    use crate::engine::time::period_from_hz;

    struct Counter {
        id: String,
        period: SimTime,
        prio: u8,
        out: SignalId,
        code: f64,
        seq: Option<SignalId>,
    }
    impl DiscreteBlock for Counter {
        fn id(&self) -> &str {
            &self.id
        }
        fn period(&self) -> Option<SimTime> {
            Some(self.period)
        }
        fn priority(&self) -> u8 {
            self.prio
        }
        fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
            ctx.bus.set(self.out, ctx.bus.get(self.out) + 1.0);
            if let Some(s) = self.seq {
                // Records firing order at t = 0 as decimal digits.
                if ctx.t == SimTime::ZERO {
                    ctx.bus.set(s, ctx.bus.get(s) * 10.0 + self.code);
                }
            }
            Ok(())
        }
        fn reset(&mut self) {}
    }

    struct Irregular {
        times: Vec<i64>,
        out: SignalId,
    }
    impl DiscreteBlock for Irregular {
        fn id(&self) -> &str {
            "irregular"
        }
        fn period(&self) -> Option<SimTime> {
            None
        }
        fn priority(&self) -> u8 {
            40
        }
        fn next_event(&self, now: SimTime) -> Option<SimTime> {
            self.times.iter().map(|&t| SimTime(t)).find(|&t| t > now)
        }
        fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
            // Store the firing time in ns (exactly representable in f64 here).
            ctx.bus.set(self.out, ctx.t.nanos() as f64);
            Ok(())
        }
        fn reset(&mut self) {}
    }

    fn counter(
        bus: &mut SignalBus,
        name: &str,
        hz: f64,
        prio: u8,
        code: f64,
        seq: Option<SignalId>,
    ) -> Box<dyn DiscreteBlock> {
        let out = bus
            .register(&format!("test.{name}"), "-", "", SignalKind::Diagnostic)
            .unwrap();
        Box::new(Counter {
            id: name.into(),
            period: period_from_hz(hz).unwrap().period,
            prio,
            out,
            code,
            seq,
        })
    }

    #[test]
    fn multi_rate_counts_are_exact() {
        let mut bus = SignalBus::new();
        let blocks = vec![
            counter(&mut bus, "fast", 20_000.0, 30, 1.0, None),
            counter(&mut bus, "mid", 5_000.0, 30, 2.0, None),
            counter(&mut bus, "slow", 1_000.0, 30, 3.0, None),
        ];
        let mut e = Engine::new(Plant::new(), bus, blocks, 1e-5);
        e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
        // Firings in [0, 1 s): exactly rate × 1 s.
        let get = |p: &str| e.bus.get(e.bus.id(p).unwrap());
        assert_eq!(
            (get("test.fast"), get("test.mid"), get("test.slow")),
            (20_000.0, 5_000.0, 1_000.0)
        );
    }

    #[test]
    fn coincident_events_fire_in_priority_order() {
        let mut bus = SignalBus::new();
        let seq = bus
            .register("test.seq", "-", "", SignalKind::Diagnostic)
            .unwrap();
        // Registered controller (30) first, sensor (10) second: the sensor must fire first.
        let blocks = vec![
            counter(&mut bus, "ctrl", 1_000.0, 30, 1.0, Some(seq)),
            counter(&mut bus, "sensor", 1_000.0, 10, 2.0, Some(seq)),
        ];
        let mut e = Engine::new(Plant::new(), bus, blocks, 1e-4);
        e.step_until(SimTime::from_secs_f64(1e-3)).unwrap();
        assert_eq!(e.bus.get(seq), 21.0);
    }

    /// First-order decay x' = −x/τ with a live parameter τ.
    struct Decay {
        tau: f64,
    }
    impl crate::engine::plant::PlantModule for Decay {
        fn name(&self) -> &str {
            "decay"
        }
        fn n_states(&self) -> usize {
            1
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![("x".into(), "-".into())]
        }
        fn init(&self, x: &mut [f64]) {
            x[0] = 1.0;
        }
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
        fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
            dx[0] = -x[o] / self.tau;
        }
        fn get_param(&self, name: &str) -> Option<f64> {
            (name == "tau").then_some(self.tau)
        }
        fn set_param(&mut self, name: &str, v: f64, _: &mut [f64]) -> Result<f64, String> {
            if name != "tau" {
                return Err(format!("unknown parameter `{name}`"));
            }
            if v <= 0.0 {
                return Err("tau must be > 0".into());
            }
            Ok(std::mem::replace(&mut self.tau, v))
        }
    }

    #[test]
    fn param_change_applies_at_boundary_and_emits_one_event() {
        use crate::engine::commands::{ChangeSource, EngineCommand, EngineEvent};
        let mut p = Plant::new();
        p.add(Box::new(Decay { tau: 1.0 }));
        let mut e = Engine::new(p, SignalBus::new(), vec![], 1e-3);
        e.step_until(SimTime::from_secs_f64(0.5)).unwrap();
        let x_half = e.x[0];
        e.queue(EngineCommand::SetParam {
            path: "decay.tau".into(),
            value: 0.25,
            source: ChangeSource::Mcp,
        });
        assert_eq!(
            e.get_param("decay.tau"),
            Some(1.0),
            "not applied before the boundary"
        );
        e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
        let ev = e.drain_events();
        assert_eq!(ev.len(), 1);
        assert!(
            matches!(&ev[0], EngineEvent::ParamChanged { old, new, source: ChangeSource::Mcp, .. } if *old == 1.0 && *new == 0.25)
        );
        // Second half decays with τ = 0.25: x(1) = x(0.5)·e^{−0.5/0.25}.
        let expected = x_half * (-2.0f64).exp();
        assert!((e.x[0] - expected).abs() <= 1e-9 + 1e-8 * expected);
        // Invalid values are rejected without changing anything.
        e.queue(EngineCommand::SetParam {
            path: "decay.tau".into(),
            value: -1.0,
            source: ChangeSource::Ui,
        });
        e.apply_pending();
        assert!(matches!(
            e.drain_events()[0],
            EngineEvent::CommandRejected { .. }
        ));
        assert_eq!(e.get_param("decay.tau"), Some(0.25));
    }

    /// Noisy sampler: writes a Gaussian sample at 1 kHz (exercises RNG snapshotting).
    struct Noisy {
        rng: crate::engine::rng::BlockRng,
        out: SignalId,
    }
    impl DiscreteBlock for Noisy {
        fn id(&self) -> &str {
            "noisy"
        }
        fn period(&self) -> Option<SimTime> {
            Some(SimTime(1_000_000))
        }
        fn priority(&self) -> u8 {
            10
        }
        fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
            let v = ctx.bus.get(self.out) + self.rng.normal();
            ctx.bus.set(self.out, v);
            Ok(())
        }
        fn reset(&mut self) {}
        fn save(&self) -> serde_json::Value {
            serde_json::to_value(self.rng.save()).unwrap_or_default()
        }
        fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
            let st = serde_json::from_value(v.clone()).map_err(|e| e.to_string())?;
            self.rng = crate::engine::rng::BlockRng::restore(&st);
            Ok(())
        }
    }

    #[test]
    fn snapshot_restore_continues_bit_identically() {
        use crate::engine::rng::RngService;
        use crate::engine::snapshot::Snapshot;
        let build = || {
            let mut bus = SignalBus::new();
            let out = bus
                .register("test.noise", "-", "", SignalKind::Diagnostic)
                .unwrap();
            let mut p = Plant::new();
            p.add(Box::new(Decay { tau: 0.3 }));
            let blocks: Vec<Box<dyn DiscreteBlock>> = vec![Box::new(Noisy {
                rng: RngService::new(9).stream("noisy"),
                out,
            })];
            Engine::new(p, bus, blocks, 1e-4)
        };
        let mut e = build();
        e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
        let snap = Snapshot::from_bytes(&e.snapshot().to_bytes().unwrap()).unwrap();
        e.step_until(SimTime::from_secs_f64(0.2)).unwrap();
        let (xa, ba) = (e.x.clone(), e.bus.values().to_vec());

        let mut f = build();
        f.restore(&snap).unwrap();
        f.step_until(SimTime::from_secs_f64(0.2)).unwrap();
        assert_eq!(xa, f.x);
        assert_eq!(ba, f.bus.values());
    }

    #[test]
    fn variable_events_are_honoured_exactly() {
        let mut bus = SignalBus::new();
        let out = bus
            .register("test.irregular", "ns", "", SignalKind::Diagnostic)
            .unwrap();
        let times = vec![0, 1_234, 50_001, 999_999];
        let blocks: Vec<Box<dyn DiscreteBlock>> = vec![Box::new(Irregular {
            times: times.clone(),
            out,
        })];
        let mut e = Engine::new(Plant::new(), bus, blocks, 1e-5);
        for &t in &times[1..] {
            e.step_events(1).unwrap();
            assert_eq!(e.time, SimTime(t));
        }
        e.step_until(SimTime(2_000_000)).unwrap();
        assert_eq!(e.bus.get(out), 999_999.0);
    }
}
