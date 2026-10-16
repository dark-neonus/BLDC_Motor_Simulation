//! Incremental (quadrature) optical encoder (EQ-SENS-05):
//!
//! c_k = ⌊(4·N_CPR/2π)·(θm(t_k) − θ_z)⌋,  θ_meas = (2π/(4·N_CPR))·c_k,
//!
//! θ_z is the angle at power-up, so the reading is relative until the index pulse
//! (once per turn at θm ≡ θ_index); from then on it is anchored there:
//! θ = θ_index + (c − c_index)·Δ. θm passes through the common pipeline first (latency,
//! noise; EQ-SENS-01). `drop_counts` is the hook for the missed-count fault (P10.T02).

use std::f64::consts::{PI, TAU};

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

use super::pipeline::SensorPipeline;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IncrementalConfig {
    /// Lines per revolution N_CPR (4·N_CPR counts after ×4 decoding).
    pub cpr: u32,
    /// Index position θ_index [rad, mechanical].
    pub index_angle: f64,
}

impl IncrementalConfig {
    /// Count step Δ = 2π/(4·N_CPR) [rad].
    pub fn step(&self) -> f64 {
        TAU / (4.0 * f64::from(self.cpr))
    }
}

/// Wrap to (−π, π].
fn wrap_pi(x: f64) -> f64 {
    let y = x.rem_euclid(TAU);
    if y > PI { y - TAU } else { y }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
struct State {
    /// θ_z: the (delayed) angle at the first sample.
    theta_z: f64,
    /// Last (delayed) angle, for index crossings.
    last: f64,
    /// Counts lost to the missed-count fault.
    lost: i64,
    /// (c_index, θ_index unwrapped near the crossing) once the index has been seen.
    index: Option<(i64, f64)>,
}

pub struct IncrementalEncoder {
    cfg: IncrementalConfig,
    period: SimTime,
    theta: SignalId,
    count: SignalId,
    index: SignalId,
    indexed: SignalId,
    raw: SignalId,
    angle: SignalId,
    error: SignalId,
    pipe: SensorPipeline,
    st: Option<State>,
    /// First-sample offset within the period.
    phase: SimTime,
}

impl IncrementalEncoder {
    /// Reads `motor.theta`; registers `sensors.encoder.{count, index, indexed, angle_raw,
    /// angle, error}`. `pipe` should not quantise or wrap (the counter does).
    pub fn new(
        cfg: IncrementalConfig,
        period: SimTime,
        pipe: SensorPipeline,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let theta = bus.id("motor.theta")?;
        let mut reg = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Output);
        Ok(Self {
            cfg,
            period,
            theta,
            count: reg("sensors.encoder.count", "-", "quadrature count (×4)")?,
            index: reg(
                "sensors.encoder.index",
                "-",
                "1 at the sample that saw the index",
            )?,
            indexed: reg(
                "sensors.encoder.indexed",
                "-",
                "1 once the index was seen (absolute angle)",
            )?,
            raw: reg(
                "sensors.encoder.angle_raw",
                "rad",
                "measured angle wrapped to [0, 2π)",
            )?,
            angle: reg("sensors.encoder.angle", "rad", "measured angle, unwrapped")?,
            error: reg(
                "sensors.encoder.error",
                "rad",
                "true − measured angle, wrapped (diagnostic)",
            )?,
            pipe,
            st: None,
            phase: SimTime::from_nanos(0),
        })
    }

    /// Missed-count fault hook (P10.T02): the next readings lose `m` counts.
    pub fn drop_counts(&mut self, m: i64) {
        if let Some(st) = &mut self.st {
            st.lost += m;
        }
    }
}

impl IncrementalEncoder {
    /// First sample offset within the period (default 0).
    pub fn with_phase(mut self, phase: SimTime) -> Self {
        self.phase = phase;
        self
    }
}

impl DiscreteBlock for IncrementalEncoder {
    fn id(&self) -> &str {
        "sensors.encoder"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn phase(&self) -> SimTime {
        self.phase
    }
    fn priority(&self) -> u8 {
        priority::SENSOR
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let th_true = ctx.bus.get(self.theta);
        let th = self.pipe.sample(ctx.t.nanos(), th_true);
        let d = self.cfg.step();
        let (angle, count, pulse, indexed) = if self.pipe.config().bypass {
            (th, (th / d).floor(), false, true)
        } else {
            let st = self.st.get_or_insert(State {
                theta_z: th,
                last: th,
                lost: 0,
                index: None,
            });
            // A turn boundary of θ − θ_index crossed since the last sample: index pulse.
            let turns = |x: f64| ((x - self.cfg.index_angle) / TAU).floor();
            let pulse = turns(th) != turns(st.last);
            let c = ((th - st.theta_z) / d).floor() as i64 - st.lost;
            if pulse && st.index.is_none() {
                // The index edge between the samples: θ_index in the turn just entered
                // (forward) or left (backward), and its count.
                let k = turns(th).max(turns(st.last));
                let th_idx = self.cfg.index_angle + TAU * k;
                let c_idx = ((th_idx - st.theta_z) / d).floor() as i64 - st.lost;
                st.index = Some((c_idx, th_idx));
            }
            st.last = th;
            let angle = match st.index {
                Some((c_idx, th_idx)) => th_idx + (c - c_idx) as f64 * d,
                None => c as f64 * d,
            };
            (angle, c as f64, pulse, st.index.is_some())
        };
        let b = |x: bool| f64::from(u8::from(x));
        ctx.bus.set(self.count, count);
        ctx.bus.set(self.index, b(pulse));
        ctx.bus.set(self.indexed, b(indexed));
        ctx.bus.set(self.raw, angle.rem_euclid(TAU));
        ctx.bus.set(self.angle, angle);
        ctx.bus.set(self.error, wrap_pi(th_true - angle));
        Ok(())
    }
    fn reset(&mut self) {
        self.pipe.reset();
        self.st = None;
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "pipe": self.pipe.save(), "st": self.st })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let st = v.get("st").cloned().ok_or("missing `st`")?;
        self.st = serde_json::from_value(st).map_err(|e| e.to_string())?;
        self.pipe.restore(v.get("pipe").ok_or("missing `pipe`")?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::engine::Engine;
    use crate::engine::plant::Plant;
    use crate::engine::rng::RngService;
    use crate::physics::sensors::pipeline::PipelineConfig;

    const CFG: IncrementalConfig = IncrementalConfig {
        cpr: 1000,
        index_angle: 1.0,
    };

    struct Rig {
        e: Engine,
        th: SignalId,
        k: i64,
    }

    impl Rig {
        fn new(theta0: f64, bypass: bool) -> Self {
            let mut bus = SignalBus::new();
            let th = bus
                .register("motor.theta", "rad", "", SignalKind::Input)
                .unwrap();
            bus.set(th, theta0);
            let pipe = SensorPipeline::new(
                PipelineConfig {
                    bypass,
                    ..Default::default()
                },
                RngService::new(1).stream("sensors.encoder"),
            );
            let enc =
                IncrementalEncoder::new(CFG, SimTime::from_nanos(1000), pipe, &mut bus).unwrap();
            let mut e = Engine::new(Plant::new(), bus, vec![Box::new(enc)], 1e-6);
            e.step_until(SimTime::from_nanos(1)).unwrap();
            Self { e, th, k: 0 }
        }
        /// Set θ and take the next sample.
        fn at(&mut self, theta: f64) -> (f64, f64, f64, f64) {
            self.k += 1;
            self.e.bus.set(self.th, theta);
            self.e
                .step_until(SimTime::from_nanos(1000 * self.k + 1))
                .unwrap();
            let g = |n: &str| self.e.bus.get(self.e.bus.id(n).unwrap());
            (
                g("sensors.encoder.count"),
                g("sensors.encoder.index"),
                g("sensors.encoder.angle"),
                g("sensors.encoder.indexed"),
            )
        }
    }

    /// V-SENS-007: 1000 CPR → 4000 counts per turn, signed by direction; index once per
    /// turn.
    #[test]
    fn v_sens_007_counts_4000_per_rev_with_direction_sign_and_index_once_per_rev() {
        let theta0 = 0.3;
        let mut r = Rig::new(theta0, false);
        let n = 7200; // 0.05° per sample (Δ = 0.09°)
        // Samples half a count off the count edges, so floor() is not at a tie.
        let h = 0.5 * CFG.step();
        let mut pulses = vec![];
        let mut last = (0.0, 0.0, 0.0, 0.0);
        for k in 1..=3 * n {
            let th = theta0 + h + TAU * k as f64 / n as f64;
            last = r.at(th);
            if last.1 == 1.0 {
                pulses.push(th);
            }
        }
        assert_eq!(last.0, 3.0 * 4000.0, "×4 counts over three turns");
        // Index once per turn, at the first sample past θ_index (+ 2π k).
        assert_eq!(pulses.len(), 3);
        for (k, th) in pulses.iter().enumerate() {
            let at = 1.0 + TAU * k as f64;
            assert!(*th >= at && th - at < TAU / n as f64, "{th}");
        }
        // Absolute after the index: within one count of the true angle.
        let d = CFG.step();
        assert_eq!(last.3, 1.0);
        assert!((last.2 - (theta0 + h + 3.0 * TAU)).abs() <= d, "{}", last.2);
        // Backward: the count falls, one more index pulse per turn.
        let mut back = 0;
        for k in 1..=n {
            let r2 = r.at(theta0 + h + 3.0 * TAU - TAU * k as f64 / n as f64);
            back += r2.1 as i32;
            last = r2;
        }
        assert_eq!(last.0, 2.0 * 4000.0);
        assert_eq!(back, 1);
        assert!((last.2 - (theta0 + h + 2.0 * TAU)).abs() <= d);
    }

    #[test]
    fn relative_until_the_index_then_anchored() {
        let mut r = Rig::new(0.3, false);
        // Before the index (θ_index = 1.0): angle relative to the power-up angle 0.3.
        let (_, _, a, idx) = r.at(0.8);
        assert_eq!(idx, 0.0);
        assert!((a - 0.5).abs() <= CFG.step());
        // Past it: absolute (θ ≈ 1.2 within a count).
        let (_, p, a, idx) = r.at(1.2);
        assert_eq!((p, idx), (1.0, 1.0));
        assert!((a - 1.2).abs() <= CFG.step(), "{a}");
    }

    #[test]
    fn missed_counts_and_bypass() {
        let mut bus = SignalBus::new();
        let th = bus
            .register("motor.theta", "rad", "", SignalKind::Input)
            .unwrap();
        let pipe = SensorPipeline::new(
            PipelineConfig::default(),
            RngService::new(1).stream("sensors.encoder"),
        );
        let mut enc =
            IncrementalEncoder::new(CFG, SimTime::from_nanos(1000), pipe, &mut bus).unwrap();
        let mut ctx_bus = bus;
        let mut step = |enc: &mut IncrementalEncoder, t: i64, x: f64| {
            ctx_bus.set(th, x);
            let mut ctx = StepCtx {
                t: SimTime::from_nanos(t),
                bus: &mut ctx_bus,
            };
            enc.step(&mut ctx).unwrap();
            ctx_bus.get(ctx_bus.id("sensors.encoder.count").unwrap())
        };
        step(&mut enc, 0, 0.0);
        assert_eq!(step(&mut enc, 1, 100.0 * CFG.step() + 1e-9), 100.0);
        enc.drop_counts(3);
        assert_eq!(step(&mut enc, 2, 100.0 * CFG.step() + 1e-9), 97.0);
        // Bypass: the true angle.
        let mut r = Rig::new(0.3, true);
        let (_, _, a, idx) = r.at(2.345);
        assert_eq!((a, idx), (2.345, 1.0));
    }
}
