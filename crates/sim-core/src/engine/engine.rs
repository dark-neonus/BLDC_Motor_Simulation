//! The engine: continuous plant + discrete blocks on a multi-rate event schedule.
//!
//! Algorithm (EQ-NUM-02): at the current time, fire every block due now in
//! (priority, registration) order; then integrate the plant up to the next event
//! (or the target) with ≤ `dt_max` substeps, holding discrete outputs constant (ZOH).

use super::block::{DiscreteBlock, SimError, StepCtx};
use super::integrate::Rk4;
use super::plant::Plant;
use super::signals::SignalBus;
use super::time::SimTime;

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
}

impl Engine {
    /// Build an engine. The bus is frozen here; blocks first fire at their phase.
    pub fn new(
        plant: Plant,
        mut bus: SignalBus,
        blocks: Vec<Box<dyn DiscreteBlock>>,
        dt_max: f64,
    ) -> Self {
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
        };
        // Make the bus consistent with the initial state.
        e.plant.outputs(0.0, &e.x, &mut e.bus);
        e
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
            self.fire_due()?;
            let t_next = self.next_event_time().map_or(target, |t| t.min(target));
            let (t0, t1) = (self.time.as_secs_f64(), t_next.as_secs_f64());
            self.integrator.integrate(
                &mut self.plant,
                &mut self.bus,
                &mut self.x,
                t0,
                t1,
                self.dt_max,
            );
            self.time = t_next;
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
