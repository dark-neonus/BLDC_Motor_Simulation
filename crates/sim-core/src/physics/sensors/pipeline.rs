//! Common sensor non-ideality pipeline (EQ-SENS-01):
//!
//! y_k = sat(Q(x(t_k − τ) + e(x(t_k − τ)) + n_k)),  n_k ~ N(0, σ²),
//!
//! held (ZOH) until the next sample. The true-signal history is kept with integer-ns
//! timestamps and read with linear interpolation, so a latency of N sample periods
//! returns exactly the sample N periods back. `bypass` returns x unchanged (no delay,
//! no noise, no quantisation; the RNG is not drawn).

use std::collections::VecDeque;

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::rng::{BlockRng, BlockRngState};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

/// Deterministic error e(x) (nonlinearity, eccentricity, gain/offset).
pub type ErrorFn = Box<dyn Fn(f64) -> f64 + Send>;

/// Pipeline settings (all stages optional).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PipelineConfig {
    /// Ideal feedback: y = x.
    pub bypass: bool,
    /// Latency τ [ns].
    pub latency_ns: i64,
    /// Noise standard deviation σ (signal units).
    pub noise_sigma: f64,
    /// Quantisation step Δ (0: none).
    pub step: f64,
    /// Wrap the result to [0, period) (angles: 2π); applied after quantisation.
    pub wrap: Option<f64>,
    /// Range limits (ADC full scale).
    pub range: Option<(f64, f64)>,
}

pub struct SensorPipeline {
    cfg: PipelineConfig,
    error: Option<ErrorFn>,
    rng: BlockRng,
    /// True-signal history (t ns, x), oldest first.
    hist: VecDeque<(i64, f64)>,
    held: f64,
}

impl SensorPipeline {
    pub fn new(cfg: PipelineConfig, rng: BlockRng) -> Self {
        Self {
            cfg,
            error: None,
            rng,
            hist: VecDeque::new(),
            held: 0.0,
        }
    }

    pub fn with_error(mut self, e: ErrorFn) -> Self {
        self.error = Some(e);
        self
    }

    pub fn config(&self) -> &PipelineConfig {
        &self.cfg
    }

    /// The last reading (zero-order hold).
    pub fn held(&self) -> f64 {
        self.held
    }

    /// Take a sample of the true value `x` at `t_ns`; returns (and holds) the reading.
    /// Samples must come in non-decreasing time.
    pub fn sample(&mut self, t_ns: i64, x: f64) -> f64 {
        if self.cfg.bypass {
            self.held = x;
            return x;
        }
        self.hist.push_back((t_ns, x));
        let xd = self.delayed(t_ns - self.cfg.latency_ns);
        let mut y = xd;
        if let Some(e) = &self.error {
            y += e(xd);
        }
        if self.cfg.noise_sigma > 0.0 {
            y += self.rng.gaussian(self.cfg.noise_sigma);
        }
        if self.cfg.step > 0.0 {
            y = self.cfg.step * (y / self.cfg.step).round();
        }
        if let Some(p) = self.cfg.wrap {
            y = y.rem_euclid(p);
        }
        if let Some((lo, hi)) = self.cfg.range {
            y = y.clamp(lo, hi);
        }
        self.held = y;
        y
    }

    /// x at `t` from the history (linear interpolation; before the first sample, the
    /// first sample). Drops samples no longer needed.
    fn delayed(&mut self, t: i64) -> f64 {
        while self.hist.len() > 2 && self.hist[1].0 <= t {
            self.hist.pop_front();
        }
        let (t0, x0) = self.hist[0];
        if t <= t0 || self.hist.len() == 1 {
            return x0;
        }
        let (t1, x1) = self.hist[1];
        if t >= t1 {
            return x1;
        }
        x0 + (x1 - x0) * (t - t0) as f64 / (t1 - t0) as f64
    }

    pub fn reset(&mut self) {
        self.hist.clear();
        self.held = 0.0;
    }

    pub fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "hist": self.hist,
            "held": self.held,
            "rng": self.rng.save(),
        })
    }

    pub fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let g = |k: &str| v.get(k).cloned().ok_or(format!("missing `{k}`"));
        let hist: VecDeque<(i64, f64)> =
            serde_json::from_value(g("hist")?).map_err(|e| e.to_string())?;
        let held: f64 = serde_json::from_value(g("held")?).map_err(|e| e.to_string())?;
        let rng: BlockRngState = serde_json::from_value(g("rng")?).map_err(|e| e.to_string())?;
        (self.hist, self.held, self.rng) = (hist, held, BlockRng::restore(&rng));
        Ok(())
    }
}

/// A sensor as a fixed-rate discrete block: reads a true signal, writes the reading.
/// PWM-triggered sampling (`pwm.center`) is a period of T_pwm with phase 0: the PWM
/// periods start at the centre of the all-Low zero vector (EQ-INV-02). With `bypass`
/// the reading is exact at each firing; the continuous bypass of EQ-SENS-01 (consumers
/// reading x(t) itself) is wired by the sensor's builder.
pub struct SensorBlock {
    id: String,
    period: SimTime,
    phase: SimTime,
    input: SignalId,
    output: SignalId,
    pipe: SensorPipeline,
}

impl SensorBlock {
    /// Registers `output` and reads `input` (registered by its owner).
    pub fn new(
        id: &str,
        period: SimTime,
        input: &str,
        output: (&str, &str, &str),
        pipe: SensorPipeline,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let input = bus.id(input)?;
        let output = bus.register(output.0, output.1, output.2, SignalKind::Output)?;
        Ok(Self {
            id: id.into(),
            period,
            phase: SimTime::from_nanos(0),
            input,
            output,
            pipe,
        })
    }

    pub fn with_phase(mut self, phase: SimTime) -> Self {
        self.phase = phase;
        self
    }
}

impl DiscreteBlock for SensorBlock {
    fn id(&self) -> &str {
        &self.id
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
        let y = self.pipe.sample(ctx.t.nanos(), ctx.bus.get(self.input));
        ctx.bus.set(self.output, y);
        Ok(())
    }
    fn reset(&mut self) {
        self.pipe.reset();
    }
    fn save(&self) -> serde_json::Value {
        self.pipe.save()
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        self.pipe.restore(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::rng::RngService;

    fn pipe(cfg: PipelineConfig) -> SensorPipeline {
        SensorPipeline::new(cfg, RngService::new(7).stream("test.sensor"))
    }

    #[test]
    fn quantisation_step_wrap_and_range() {
        let mut p = pipe(PipelineConfig {
            step: 0.25,
            ..Default::default()
        });
        assert_eq!(p.sample(0, 1.13), 1.25);
        assert_eq!(p.sample(1, -0.37), -0.25);
        assert_eq!(p.held(), -0.25);
        // 12-bit angle: Δ = 2π/4096, wrapped to [0, 2π).
        let d = 2.0 * std::f64::consts::PI / 4096.0;
        let mut p = pipe(PipelineConfig {
            step: d,
            wrap: Some(2.0 * std::f64::consts::PI),
            ..Default::default()
        });
        let y = p.sample(0, 7.0);
        assert!((y - d * ((7.0 - 2.0 * std::f64::consts::PI) / d).round()).abs() < 1e-12);
        let mut p = pipe(PipelineConfig {
            range: Some((-10.0, 10.0)),
            ..Default::default()
        });
        assert_eq!(p.sample(0, 12.0), 10.0);
        assert_eq!(p.sample(1, -12.0), -10.0);
    }

    #[test]
    fn delay_is_exactly_n_samples_and_interpolates_between() {
        let t_s = 50_000;
        let mut p = pipe(PipelineConfig {
            latency_ns: 3 * t_s,
            ..Default::default()
        });
        let x = |k: i64| (k * k) as f64;
        for k in 0..20 {
            let y = p.sample(k * t_s, x(k));
            // Before the history fills, the first sample is held.
            assert_eq!(y, x((k - 3).max(0)), "k = {k}");
        }
        // Half a period extra: the midpoint of the two samples around it.
        let mut p = pipe(PipelineConfig {
            latency_ns: 3 * t_s + t_s / 2,
            ..Default::default()
        });
        for k in 0..20 {
            let y = p.sample(k * t_s, x(k));
            if k >= 4 {
                assert_eq!(y, 0.5 * (x(k - 4) + x(k - 3)), "k = {k}");
            }
        }
    }

    #[test]
    fn noise_std_within_bounds_and_seeded() {
        let sigma = 0.3;
        let cfg = PipelineConfig {
            noise_sigma: sigma,
            ..Default::default()
        };
        let mut p = pipe(cfg);
        let n = 10_000;
        let ys: Vec<f64> = (0..n).map(|k| p.sample(k, 1.0) - 1.0).collect();
        let mean = ys.iter().sum::<f64>() / n as f64;
        let sd = (ys.iter().map(|y| (y - mean).powi(2)).sum::<f64>() / (n - 1) as f64).sqrt();
        assert!((sd / sigma - 1.0).abs() < 0.05, "sd {sd}");
        assert!(mean.abs() < 4.0 * sigma / (n as f64).sqrt(), "mean {mean}");
        // Same seed and block id → the same sequence.
        let mut q = pipe(cfg);
        assert_eq!(q.sample(0, 1.0) - 1.0, ys[0]);
    }

    #[test]
    fn bypass_is_exact_and_draws_no_noise() {
        let cfg = PipelineConfig {
            bypass: true,
            latency_ns: 1_000_000,
            noise_sigma: 1.0,
            step: 0.5,
            range: Some((0.0, 1.0)),
            ..Default::default()
        };
        let mut p = pipe(cfg).with_error(Box::new(|x| 0.1 * x));
        for (k, x) in [0.123_456_789, -3.7, 42.0].into_iter().enumerate() {
            assert_eq!(p.sample(k as i64, x), x);
        }
    }

    #[test]
    fn error_applies_at_the_delayed_value() {
        let mut p = pipe(PipelineConfig {
            latency_ns: 10,
            ..Default::default()
        })
        .with_error(Box::new(|x| 0.01 * x + 0.5));
        p.sample(0, 100.0);
        assert_eq!(p.sample(10, 200.0), 100.0 + 1.0 + 0.5);
    }

    #[test]
    fn block_samples_at_its_rate_and_restores() {
        use crate::engine::engine::Engine;
        use crate::engine::plant::Plant;
        use crate::engine::snapshot::Snapshot;
        let mk = || {
            let mut bus = SignalBus::new();
            bus.register("test.x", "-", "", SignalKind::Input).unwrap();
            let pipe = pipe(PipelineConfig {
                noise_sigma: 0.1,
                latency_ns: 100_000,
                ..Default::default()
            });
            let b = SensorBlock::new(
                "sensors.test",
                SimTime::from_nanos(100_000),
                "test.x",
                ("sensors.test.y", "-", "reading"),
                pipe,
                &mut bus,
            )
            .unwrap();
            Engine::new(Plant::new(), bus, vec![Box::new(b)], 1e-5)
        };
        let mut a = mk();
        a.step_until(SimTime::from_nanos(550_000)).unwrap();
        let snap = Snapshot::from_bytes(&a.snapshot().to_bytes().unwrap()).unwrap();
        let mut b = mk();
        b.restore(&snap).unwrap();
        a.step_until(SimTime::from_nanos(1_050_000)).unwrap();
        b.step_until(SimTime::from_nanos(1_050_000)).unwrap();
        let y = |e: &Engine| e.bus.get(e.bus.id("sensors.test.y").unwrap());
        assert_eq!(y(&a), y(&b));
        assert!(y(&a).abs() < 1.0 && y(&a) != 0.0);
    }
}
