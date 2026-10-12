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
}

impl Pwm {
    /// Registers `inverter.duty_a..c` (inputs) and `inverter.pwm_count`; drives the legs
    /// `inverter.leg_a..c` (registered by the inverter).
    pub fn new(cfg: PwmConfig, bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut duty = |x: &str| {
            bus.register(
                &format!("inverter.duty_{x}"),
                "-",
                "duty (fraction of the period High)",
                SignalKind::Input,
            )
        };
        let duty = [duty("a")?, duty("b")?, duty("c")?];
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
        })
    }

    /// Latch the duties and lay out this period's edges (EQ-INV-02).
    fn schedule(&mut self, t0: i64, bus: &SignalBus) {
        let (tp, td) = (self.cfg.period_ns, self.cfg.dead_ns);
        self.edges.clear();
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
            let mut turn = |t: i64, to: LegState, edges: &mut Vec<(i64, usize, LegState)>| {
                edges.push((t, x, LegState::Off));
                edges.push((t + td, x, to));
            };
            if start != self.state[x] {
                turn(t0, start, &mut self.edges);
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
            self.schedule(t0, ctx.bus);
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
