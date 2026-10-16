//! Center-aligned PWM with dead time and shadow registers (EQ-INV-02), as a variable-
//! event block. Duties `inverter.duty_x` are latched at each period start t_k (the
//! middle of the all-Low zero vector, `pwm.center`); every switch edge is an exact event
//! (ns), and every turn-on is delayed by the dead time (the leg is Off in between).

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

use super::leg::LegState;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PwmConfig {
    /// PWM period T_pwm [ns].
    pub period_ns: i64,
    /// Dead time t_d [ns].
    pub dead_ns: i64,
}

pub struct Pwm {
    cfg: PwmConfig,
    duty: [SignalId; 3],
    leg: [SignalId; 3],
    /// Period counter, incremented at each `pwm.center` (ADC trigger, P08).
    count: SignalId,
    /// Start of the next period [ns].
    next_period: i64,
    /// Edges of the current period: (time ns, leg, new state), sorted by time.
    edges: Vec<(i64, usize, LegState)>,
    state: [LegState; 3],
    latched: [f64; 3],
    periods: f64,
    /// Six-step (EQ-INV-07): `inverter.pattern_x` (+1 High = H/O PWM, −1 Low, 0 Off).
    pattern: Option<[SignalId; 3]>,
}

impl Pwm {
    /// Registers `inverter.duty_a..c` (inputs) and `inverter.pwm_count`; drives the legs
    /// `inverter.leg_a..c` (registered by the inverter).
    pub fn new(cfg: PwmConfig, bus: &mut SignalBus) -> Result<Self, SignalError> {
        let duty = super::leg::duty_ids(bus)?;
        let count = bus.register(
            "inverter.pwm_count",
            "-",
            "PWM periods so far (+1 at each pwm.center)",
            SignalKind::Output,
        )?;
        let leg = [
            bus.id("inverter.leg_a")?,
            bus.id("inverter.leg_b")?,
            bus.id("inverter.leg_c")?,
        ];
        Ok(Self {
            cfg,
            duty,
            leg,
            count,
            next_period: 0,
            edges: vec![],
            state: [LegState::Low; 3],
            latched: [0.0; 3],
            periods: 0.0,
            pattern: None,
        })
    }

    /// Six-step mode (EQ-INV-07): registers `inverter.pattern_a..c`. Per period a High
    /// leg is H for d·T centred on the period, O otherwise (no dead time between H and
    /// O); a Low leg is L and an Off leg O all period. A direct H↔L change goes through
    /// O for the dead time.
    pub fn with_six_step(mut self, bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut reg = |x: &str| {
            bus.register(
                &format!("inverter.pattern_{x}"),
                "-",
                "six-step leg pattern: +1 High (H/O PWM), −1 Low, 0 Off",
                SignalKind::Input,
            )
        };
        self.pattern = Some([reg("a")?, reg("b")?, reg("c")?]);
        Ok(self)
    }

    fn schedule_six_step(&mut self, t0: i64, pattern: [SignalId; 3], bus: &SignalBus) {
        let (tp, td) = (self.cfg.period_ns, self.cfg.dead_ns);
        self.edges.clear();
        #[allow(clippy::needless_range_loop)] // x indexes four parallel arrays
        for x in 0..3 {
            let d = bus.get(self.duty[x]);
            let d = if d.is_finite() {
                d.clamp(0.0, 1.0)
            } else {
                0.0
            };
            self.latched[x] = d;
            let cur = self.state[x];
            match LegState::from_signal(bus.get(pattern[x])) {
                LegState::High => {
                    let mut high_ns = (d * tp as f64).round() as i64;
                    let mut t_on = t0 + (tp - high_ns) / 2;
                    if cur == LegState::Low && t_on < t0 + td && high_ns > 0 {
                        // From L the high switch waits for the dead time.
                        high_ns -= t0 + td - t_on;
                        t_on = t0 + td;
                    }
                    // From L, the low switch opens at t0 (O until the pulse).
                    if cur != LegState::Off {
                        self.edges.push((t0, x, LegState::Off));
                    }
                    if high_ns > 0 {
                        self.edges.push((t_on, x, LegState::High));
                        if high_ns < tp {
                            self.edges.push((t_on + high_ns, x, LegState::Off));
                        }
                    }
                }
                LegState::Low => match cur {
                    LegState::High => {
                        self.edges.push((t0, x, LegState::Off));
                        self.edges.push((t0 + td, x, LegState::Low));
                    }
                    LegState::Off => self.edges.push((t0, x, LegState::Low)),
                    LegState::Low => {}
                },
                LegState::Off => {
                    if cur != LegState::Off {
                        self.edges.push((t0, x, LegState::Off));
                    }
                }
            }
        }
        self.edges.sort_by_key(|e| e.0);
    }

    /// Latch the duties and lay out this period's edges (EQ-INV-02). Edges of the last
    /// period at or after t0 (the turn-on into Low at d near 1 ends after the period:
    /// t_on + high + t_d ≥ t0 + T when T − high < 2·t_d) carry over.
    fn schedule(&mut self, t0: i64, bus: &SignalBus) {
        let (tp, td) = (self.cfg.period_ns, self.cfg.dead_ns);
        let carried: Vec<_> = self.edges.drain(..).filter(|e| e.0 >= t0).collect();
        for x in 0..3 {
            let d = bus.get(self.duty[x]);
            let d = if d.is_finite() {
                d.clamp(0.0, 1.0)
            } else {
                0.0
            };
            self.latched[x] = d;
            let high_ns = (d * tp as f64).round() as i64;
            // Pulse dropping: within t_d of 0 or 1 the leg stays Low / High all period.
            let full = if high_ns <= td {
                Some(LegState::Low)
            } else if tp - high_ns <= td {
                Some(LegState::High)
            } else {
                None
            };
            // Each period starts in the zero vector: Low, unless the leg stays High.
            let start = full.unwrap_or(LegState::Low);
            let turn = |t: i64, to: LegState, edges: &mut Vec<(i64, usize, LegState)>| {
                edges.push((t, x, LegState::Off));
                edges.push((t + td, x, to));
            };
            // The leg's state once its carried edges are done, and when that is.
            let pending: Vec<_> = carried.iter().filter(|e| e.1 == x).copied().collect();
            let (cur, t_free) = match pending.last() {
                Some(&(t, _, s)) => (s, t),
                None => (self.state[x], t0),
            };
            if start != cur {
                // A carried turn-on is replaced: the leg stays Off until t_d after it.
                turn(t_free, start, &mut self.edges);
            } else {
                self.edges.extend(pending);
            }
            if full.is_none() {
                let t_on = t0 + (tp - high_ns) / 2;
                turn(t_on, LegState::High, &mut self.edges);
                turn(t_on + high_ns, LegState::Low, &mut self.edges);
            }
        }
        self.edges.sort_by_key(|e| e.0);
    }
}

impl DiscreteBlock for Pwm {
    fn id(&self) -> &str {
        "inverter.pwm"
    }
    fn period(&self) -> Option<SimTime> {
        None
    }
    fn priority(&self) -> u8 {
        priority::INVERTER
    }
    fn next_event(&self, now: SimTime) -> Option<SimTime> {
        let n = now.nanos();
        let edge = self.edges.iter().map(|e| e.0).find(|&t| t > n);
        let next = match edge {
            Some(t) => t.min(self.next_period),
            None => self.next_period,
        };
        // The first firing is at t = 0 (period start).
        Some(SimTime::from_nanos(if next > n { next } else { n + 1 }))
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let t = ctx.t.nanos();
        if t >= self.next_period {
            let t0 = self.next_period;
            match self.pattern {
                Some(p) => self.schedule_six_step(t0, p, ctx.bus),
                None => self.schedule(t0, ctx.bus),
            }
            self.next_period = t0 + self.cfg.period_ns;
            self.periods += 1.0;
            ctx.bus.set(self.count, self.periods);
        }
        for &(te, x, s) in &self.edges {
            if te == t {
                self.state[x] = s;
            }
        }
        for x in 0..3 {
            ctx.bus.set(self.leg[x], self.state[x].to_signal());
        }
        Ok(())
    }
    fn reset(&mut self) {
        self.next_period = 0;
        self.edges.clear();
        self.state = [LegState::Low; 3];
        self.periods = 0.0;
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "next_period": self.next_period,
            "edges": self.edges.iter().map(|&(t, x, s)| (t, x, s.to_signal())).collect::<Vec<_>>(),
            "state": self.state.map(LegState::to_signal),
            "latched": self.latched,
            "periods": self.periods,
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let g = |k: &str| v.get(k).cloned().ok_or(format!("missing `{k}`"));
        self.next_period = serde_json::from_value(g("next_period")?).map_err(|e| e.to_string())?;
        let edges: Vec<(i64, usize, f64)> =
            serde_json::from_value(g("edges")?).map_err(|e| e.to_string())?;
        self.edges = edges
            .into_iter()
            .map(|(t, x, s)| (t, x, LegState::from_signal(s)))
            .collect();
        let st: [f64; 3] = serde_json::from_value(g("state")?).map_err(|e| e.to_string())?;
        self.state = st.map(LegState::from_signal);
        self.latched = serde_json::from_value(g("latched")?).map_err(|e| e.to_string())?;
        self.periods = serde_json::from_value(g("periods")?).map_err(|e| e.to_string())?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::commands::{ChangeSource, EngineCommand};
    use crate::engine::engine::Engine;
    use crate::engine::plant::Plant;

    const CFG: PwmConfig = PwmConfig {
        period_ns: 50_000, // 20 kHz
        dead_ns: 500,
    };

    fn engine(duties: [f64; 3]) -> Engine {
        let mut bus = SignalBus::new();
        for x in ["a", "b", "c"] {
            bus.register(&format!("inverter.leg_{x}"), "-", "", SignalKind::Input)
                .unwrap();
        }
        let pwm = Pwm::new(CFG, &mut bus).unwrap();
        for (x, d) in ["a", "b", "c"].iter().zip(duties) {
            let id = bus.id(&format!("inverter.duty_{x}")).unwrap();
            bus.set(id, d);
        }
        Engine::new(Plant::new(), bus, vec![Box::new(pwm)], 1e-6)
    }

    /// Leg states just after the events at `t` ns (time must not go backwards).
    fn at(e: &mut Engine, t: i64) -> [f64; 3] {
        e.step_until(SimTime::from_nanos(t + 1)).unwrap();
        ["inverter.leg_a", "inverter.leg_b", "inverter.leg_c"]
            .map(|n| e.bus.get(e.bus.id(n).unwrap()))
    }

    #[test]
    fn edges_are_exact_with_dead_time() {
        let mut e = engine([0.6, 0.2, 0.0]);
        // Leg a, d = 0.6: High interval [10, 40] µs; dead time 0.5 µs on each turn-on.
        assert_eq!(at(&mut e, 9_998)[0], -1.0, "Low before the edge");
        assert_eq!(at(&mut e, 10_000)[0], 0.0, "Off at the commanded edge");
        assert_eq!(at(&mut e, 10_498)[0], 0.0, "still in dead time");
        assert_eq!(at(&mut e, 10_500)[0], 1.0, "High after t_d");
        // Leg b, d = 0.2: [20, 30] µs.
        assert_eq!(at(&mut e, 20_000)[1], 0.0);
        assert_eq!(at(&mut e, 20_500)[1], 1.0);
        assert_eq!(at(&mut e, 30_000)[1], 0.0);
        assert_eq!(at(&mut e, 30_500)[1], -1.0);
        assert_eq!(at(&mut e, 40_000)[0], 0.0);
        assert_eq!(at(&mut e, 40_500)[0], -1.0);
        // Leg c, d = 0: no pulse at all (dropped), over the whole period.
        let mut e = engine([0.6, 0.2, 0.0]);
        let mut prev = [-1.0; 3];
        for t in (0..50_000).step_by(250) {
            let s = at(&mut e, t);
            assert_eq!(s[2], -1.0);
            // Every change passes through Off (sampled finer than the dead time).
            for x in 0..3 {
                if s[x] != prev[x] {
                    assert!(
                        s[x] == 0.0 || prev[x] == 0.0,
                        "direct H/L switch on leg {x} at {t}"
                    );
                }
            }
            prev = s;
        }
    }

    /// Pulse dropping (EQ-INV-02/03): a duty inside t_d of 0 or 1 (here half the dead
    /// fraction) keeps the leg Low / High all period, with no Off interval.
    #[test]
    fn duties_within_the_dead_time_drop_the_pulse() {
        let h = 0.5 * CFG.dead_ns as f64 / CFG.period_ns as f64;
        let mut e = engine([h, 1.0 - h, 0.5]);
        at(&mut e, 0);
        // Leg b enters High from the initial Low through one dead time (t_d = 500 ns).
        for t in (750..2 * CFG.period_ns).step_by(250) {
            let s = at(&mut e, t);
            assert_eq!((s[0], s[1]), (-1.0, 1.0), "t {t}");
        }
    }

    /// EQ-INV-07 six-step: High leg H for d·T centred and O otherwise (no dead time),
    /// Low leg L, Off leg O; a High→Low pattern change passes through O for t_d.
    #[test]
    fn six_step_pattern_gives_h_o_pulses() {
        let mut bus = SignalBus::new();
        for x in ["a", "b", "c"] {
            bus.register(&format!("inverter.leg_{x}"), "-", "", SignalKind::Input)
                .unwrap();
        }
        let pwm = Pwm::new(CFG, &mut bus)
            .unwrap()
            .with_six_step(&mut bus)
            .unwrap();
        let set = |bus: &mut SignalBus, p: &str, v: f64| {
            let id = bus.id(p).unwrap();
            bus.set(id, v);
        };
        set(&mut bus, "inverter.pattern_a", 1.0);
        set(&mut bus, "inverter.pattern_b", -1.0);
        set(&mut bus, "inverter.pattern_c", 0.0);
        set(&mut bus, "inverter.duty_a", 0.4);
        let mut e = Engine::new(Plant::new(), bus, vec![Box::new(pwm)], 1e-6);
        // a: H for 20 µs centred: [15, 35] µs, O otherwise; b: L; c: O.
        assert_eq!(at(&mut e, 0), [0.0, -1.0, 0.0]);
        assert_eq!(at(&mut e, 14_999)[0], 0.0);
        assert_eq!(at(&mut e, 15_000)[0], 1.0);
        assert_eq!(at(&mut e, 34_999)[0], 1.0);
        assert_eq!(at(&mut e, 35_000)[0], 0.0);
        // Pattern Low from O: L at once (the high switch is already open).
        let (pa, da) = (
            e.bus.id("inverter.pattern_a").unwrap(),
            e.bus.id("inverter.duty_a").unwrap(),
        );
        e.bus.set(pa, -1.0);
        assert_eq!(at(&mut e, 50_000)[0], -1.0);
        // Full duty: H all period, entered from L through the dead time.
        e.bus.set(pa, 1.0);
        e.bus.set(da, 1.0);
        assert_eq!(at(&mut e, 100_000)[0], 0.0);
        assert_eq!(at(&mut e, 100_500)[0], 1.0);
        assert_eq!(at(&mut e, 149_999)[0], 1.0);
        // High → Low at the period start: O for t_d, then L.
        e.bus.set(pa, -1.0);
        assert_eq!(at(&mut e, 150_000)[0], 0.0, "O for the dead time");
        assert_eq!(at(&mut e, 150_500)[0], -1.0);
    }

    #[test]
    fn duty_changes_mid_period_apply_next_period() {
        let mut e = engine([0.6, 0.5, 0.5]);
        let _ = at(&mut e, 5_000);
        e.queue(EngineCommand::SetSignal {
            path: "inverter.duty_a".into(),
            value: 0.2,
            source: ChangeSource::Internal,
        });
        // This period keeps 0.6 ([10, 40] µs), the next uses 0.2 ([70, 80] µs).
        assert_eq!(at(&mut e, 10_500)[0], 1.0);
        assert_eq!(at(&mut e, 60_000)[0], -1.0);
        assert_eq!(at(&mut e, 70_500)[0], 1.0);
        assert_eq!(at(&mut e, 80_500)[0], -1.0);
        assert_eq!(e.bus.get(e.bus.id("inverter.pwm_count").unwrap()), 2.0);
    }

    /// Review P07 #1: at d just below 1 − 2·t_d/T the Low turn-on ends after the period;
    /// it must carry over (no lost edge, no direct Low ↔ High switch).
    #[test]
    fn near_full_duty_edges_carry_into_the_next_period() {
        for k in 0..=30 {
            let d = 0.97 + 0.001 * f64::from(k);
            let mut e = engine([d, 0.5, 0.5]);
            let mut prev = -1.0;
            let mut low_in_period = [false; 3];
            // Sampled every 25 ns: the shortest Low interval kept is T − high − t_d = 50 ns.
            for t in (0..150_000).step_by(25) {
                let s = at(&mut e, t)[0];
                if s != prev {
                    assert!(s == 0.0 || prev == 0.0, "d {d}: direct switch at {t}");
                }
                if s == -1.0 {
                    low_in_period[(t / 50_000) as usize] = true;
                }
                prev = s;
            }
            // Pulses are kept while T − high > t_d (d < 0.99); each period then has Low.
            let high_ns = (d * 50_000.0).round() as i64;
            if 50_000 - high_ns > 500 {
                assert!(
                    low_in_period[1] && low_in_period[2],
                    "d {d}: {low_in_period:?}"
                );
            }
        }
    }

    #[test]
    fn near_full_duty_stays_high_with_dead_time_at_entry() {
        let mut e = engine([0.995, 0.5, 0.5]);
        // (1 − d)·T = 250 ns ≤ t_d: High all period; entering it from Low at t = 0 still
        // takes the dead time.
        assert_eq!(at(&mut e, 0)[0], 0.0);
        assert_eq!(at(&mut e, 500)[0], 1.0);
        assert_eq!(at(&mut e, 49_999)[0], 1.0);
        assert_eq!(at(&mut e, 75_000)[0], 1.0);
    }
}
