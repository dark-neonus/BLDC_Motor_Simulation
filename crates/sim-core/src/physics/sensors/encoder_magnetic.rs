//! On-axis magnetic encoder (EQ-SENS-03/04):
//!
//! θ_meas = Q_b(θm(t_k − τ) + θ0 + e_INL(θm) + e_ecc(θm) + n_k),  Δ = 2π/2^b,
//!
//! with the fixed INL shape and eccentricity slope of sensors.md, and the air-gap
//! approximation σ(z) = σ_nom·max(1, (z/z_max)³), INL(z) = INL·(1 + max(0, z_min − z)/z_min).
//! Outputs `sensors.encoder.angle_raw` [0, 2π), `.angle` (unwrapped) and `.error`.

use std::f64::consts::{PI, TAU};

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::rng::BlockRng;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

use super::pipeline::{PipelineConfig, SensorPipeline};

/// Eccentricity slope k_ecc = 0.8°/mm [rad/m] (sensors.md, AS5047P figure).
pub const K_ECC: f64 = 0.8 * PI / 180.0 / 1e-3;

/// Relative INL shape Σ a_n sin(nθ + φ_n), (a) ∝ (0.25, 1, 0.5), (φ) = (0, π/3, π/2).
fn inl_shape(th: f64) -> f64 {
    0.25 * th.sin() + (2.0 * th + PI / 3.0).sin() + 0.5 * (3.0 * th + PI / 2.0).sin()
}

/// max_θ |inl_shape(θ)| (dense grid, then a golden-section refinement of the best cell).
fn inl_shape_peak() -> f64 {
    let n = 20_000;
    let h = TAU / n as f64;
    let best = (0..n)
        .map(|k| k as f64 * h)
        .max_by(|a, b| inl_shape(*a).abs().total_cmp(&inl_shape(*b).abs()))
        .unwrap_or(0.0);
    let (mut a, mut b) = (best - h, best + h);
    let g = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..60 {
        let (c, d) = (b - g * (b - a), a + g * (b - a));
        if inl_shape(c).abs() > inl_shape(d).abs() {
            b = d;
        } else {
            a = c;
        }
    }
    inl_shape(0.5 * (a + b)).abs()
}

/// Air gap z and the recommended window [z_min, z_max] [m].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirGap {
    pub z: f64,
    pub z_min: f64,
    pub z_max: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagneticEncoderConfig {
    pub bits: u32,
    /// Datasheet INL peak [rad].
    pub inl: f64,
    /// Magnet–chip lateral displacement r_d [m].
    pub displacement: f64,
    /// Mounting offset θ0 [rad].
    pub mount_offset: f64,
    /// Nominal noise σ_nom [rad].
    pub noise: f64,
    pub air_gap: Option<AirGap>,
    /// Latency τ [ns].
    pub latency_ns: i64,
    pub bypass: bool,
}

impl MagneticEncoderConfig {
    /// Effective (σ, INL peak) after the air-gap approximation (EQ-SENS-04, k_sat = 1).
    pub fn effective(&self) -> (f64, f64) {
        match self.air_gap {
            Some(g) => (
                self.noise * (g.z / g.z_max).powi(3).max(1.0),
                self.inl * (1.0 + (g.z_min - g.z).max(0.0) / g.z_min),
            ),
            None => (self.noise, self.inl),
        }
    }

    /// The deterministic error e(θm) = θ0 + e_INL(θm) + e_ecc(θm).
    pub fn error_fn(&self) -> impl Fn(f64) -> f64 + Send + 'static {
        let (_, inl) = self.effective();
        let k = inl / inl_shape_peak();
        let (a_ecc, th0) = (K_ECC * self.displacement, self.mount_offset);
        move |th| th0 + k * inl_shape(th) + a_ecc * th.sin()
    }

    pub fn pipeline(&self, rng: BlockRng) -> SensorPipeline {
        let (sigma, _) = self.effective();
        let cfg = PipelineConfig {
            bypass: self.bypass,
            latency_ns: self.latency_ns,
            noise_sigma: sigma,
            step: TAU / 2f64.powi(self.bits as i32),
            wrap: Some(TAU),
            range: None,
        };
        SensorPipeline::new(cfg, rng).with_error(Box::new(self.error_fn()))
    }
}

/// Wrap to (−π, π].
fn wrap_pi(x: f64) -> f64 {
    let y = x.rem_euclid(TAU);
    if y > PI { y - TAU } else { y }
}

pub struct MagneticEncoder {
    period: SimTime,
    theta: SignalId,
    raw: SignalId,
    angle: SignalId,
    error: SignalId,
    pipe: SensorPipeline,
    /// Last register value and unwrapped angle (None before the first sample).
    last: Option<(f64, f64)>,
    /// First-sample offset within the period.
    phase: SimTime,
}

impl MagneticEncoder {
    /// Reads `motor.theta`; registers the encoder outputs.
    pub fn new(
        cfg: &MagneticEncoderConfig,
        period: SimTime,
        rng: BlockRng,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let theta = bus.id("motor.theta")?;
        let mut reg = |p: &str, d: &str| bus.register(p, "rad", d, SignalKind::Output);
        Ok(Self {
            period,
            theta,
            raw: reg(
                "sensors.encoder.angle_raw",
                "encoder register angle [0, 2π)",
            )?,
            angle: reg("sensors.encoder.angle", "measured angle, unwrapped")?,
            error: reg(
                "sensors.encoder.error",
                "true − measured angle, wrapped (diagnostic)",
            )?,
            pipe: cfg.pipeline(rng),
            last: None,
            phase: SimTime::from_nanos(0),
        })
    }
}

impl MagneticEncoder {
    /// First sample offset within the period (default 0).
    pub fn with_phase(mut self, phase: SimTime) -> Self {
        self.phase = phase;
        self
    }
}

impl DiscreteBlock for MagneticEncoder {
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
        let th = ctx.bus.get(self.theta);
        let raw = self.pipe.sample(ctx.t.nanos(), th);
        let raw = if self.pipe.config().bypass {
            raw.rem_euclid(TAU)
        } else {
            raw
        };
        let angle = match self.last {
            Some((r0, a0)) => a0 + wrap_pi(raw - r0),
            // Start within half a turn of the true angle.
            None => raw + TAU * ((th - raw) / TAU).round(),
        };
        self.last = Some((raw, angle));
        ctx.bus.set(self.raw, raw);
        ctx.bus.set(self.angle, angle);
        ctx.bus.set(self.error, wrap_pi(th - raw));
        Ok(())
    }
    fn reset(&mut self) {
        self.pipe.reset();
        self.last = None;
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "pipe": self.pipe.save(), "last": self.last })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let last = v.get("last").cloned().ok_or("missing `last`")?;
        self.last = serde_json::from_value(last).map_err(|e| e.to_string())?;
        self.pipe.restore(v.get("pipe").ok_or("missing `pipe`")?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::rng::RngService;

    fn cfg() -> MagneticEncoderConfig {
        MagneticEncoderConfig {
            bits: 14,
            inl: 0.0,
            displacement: 0.0,
            mount_offset: 0.0,
            noise: 0.0,
            air_gap: None,
            latency_ns: 0,
            bypass: false,
        }
    }

    fn pipe(c: &MagneticEncoderConfig) -> SensorPipeline {
        c.pipeline(RngService::new(3).stream("sensors.encoder"))
    }

    /// V-SENS-001: a slow ramp moves the 14-bit reading in steps of exactly 2π/16384.
    #[test]
    fn v_sens_001_resolution_step() {
        let d = TAU / 16384.0;
        let mut p = pipe(&cfg());
        let mut prev = p.sample(0, 0.1);
        let mut steps = 0;
        for k in 1..20_000 {
            let y = p.sample(k, 0.1 + k as f64 * d / 50.0);
            if y != prev {
                assert!(((y - prev) / d - 1.0).abs() < 1e-9, "step {}", y - prev);
                steps += 1;
            }
            assert!((y / d - (y / d).round()).abs() < 1e-9, "on the grid");
            prev = y;
        }
        // Every code boundary in the ramp is seen once.
        let codes = |x: f64| (x / d).round() as i64;
        assert_eq!(steps, codes(0.1 + 19_999.0 * d / 50.0) - codes(0.1));
    }

    #[test]
    fn eccentricity_is_a_first_harmonic_of_the_expected_amplitude() {
        // r_d = 0.5 mm → a_ecc = 0.4°; 20-bit so quantisation is negligible.
        let c = MagneticEncoderConfig {
            bits: 20,
            displacement: 0.5e-3,
            ..cfg()
        };
        let a = 0.4f64.to_radians();
        assert!((K_ECC * 0.5e-3 - a).abs() < 1e-15);
        let mut p = pipe(&c);
        let d = TAU / 2f64.powi(20);
        let mut peak: f64 = 0.0;
        for k in 0..3600 {
            let th = TAU * k as f64 / 3600.0;
            let err = wrap_pi(th - p.sample(k, th));
            // true − measured = −a·sin θ (one period per turn).
            assert!((err + a * th.sin()).abs() <= d, "θ {th}: {err}");
            peak = peak.max(err.abs());
        }
        assert!((peak / a - 1.0).abs() < 1e-3, "{peak}");
    }

    #[test]
    fn inl_peak_matches_the_datasheet_value() {
        let c = MagneticEncoderConfig {
            bits: 24,
            inl: 0.8f64.to_radians(),
            ..cfg()
        };
        let e = c.error_fn();
        let peak = (0..100_000)
            .map(|k| e(TAU * k as f64 / 100_000.0).abs())
            .fold(0.0, f64::max);
        assert!((peak / 0.8f64.to_radians() - 1.0).abs() < 1e-6, "{peak}");
    }

    #[test]
    fn air_gap_outside_the_window_raises_noise_and_inl() {
        let sd = |z: f64| {
            let c = MagneticEncoderConfig {
                bits: 24,
                noise: 0.068f64.to_radians(),
                inl: 0.8f64.to_radians(),
                air_gap: Some(AirGap {
                    z,
                    z_min: 0.5e-3,
                    z_max: 3e-3,
                }),
                ..cfg()
            };
            let e = c.error_fn();
            let mut p = pipe(&c);
            let n = 10_000;
            let r: Vec<f64> = (0..n)
                .map(|k| wrap_pi(p.sample(k, 1.0) - 1.0 - e(1.0)))
                .collect();
            let m = r.iter().sum::<f64>() / n as f64;
            let s = (r.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (n - 1) as f64).sqrt();
            (s, c.effective())
        };
        let nom = 0.068f64.to_radians();
        let (s_in, (sig_in, inl_in)) = sd(2e-3);
        assert_eq!((sig_in, inl_in), (nom, 0.8f64.to_radians()));
        assert!((s_in / nom - 1.0).abs() < 0.05, "{s_in}");
        // Twice the maximum gap: σ × 8 (B ∝ z⁻³).
        let (s_far, (sig_far, _)) = sd(6e-3);
        assert!((sig_far / nom - 8.0).abs() < 1e-12);
        assert!((s_far / (8.0 * nom) - 1.0).abs() < 0.05, "{s_far}");
        // Half the minimum gap: INL × 1.5, noise nominal.
        let (_, (sig_close, inl_close)) = sd(0.25e-3);
        assert_eq!(sig_close, nom);
        assert!((inl_close / 0.8f64.to_radians() - 1.5).abs() < 1e-12);
    }

    #[test]
    fn block_unwraps_and_reports_the_error() {
        use crate::engine::engine::Engine;
        use crate::engine::plant::Plant;
        let mut bus = SignalBus::new();
        let th = bus
            .register("motor.theta", "rad", "", SignalKind::Input)
            .unwrap();
        let c = MagneticEncoderConfig {
            mount_offset: 0.01,
            ..cfg()
        };
        let enc = MagneticEncoder::new(
            &c,
            SimTime::from_nanos(100_000),
            RngService::new(1).stream("sensors.encoder"),
            &mut bus,
        )
        .unwrap();
        let mut e = Engine::new(Plant::new(), bus, vec![Box::new(enc)], 1e-5);
        let g = |e: &Engine, n: &str| e.bus.get(e.bus.id(n).unwrap());
        // Three and a bit turns, 0.5 rad per sample.
        for k in 0..20 {
            let t = 0.5 * k as f64 + 0.2;
            e.bus.set(th, t);
            e.step_until(SimTime::from_nanos(100_000 * k + 1)).unwrap();
            let d = TAU / 16384.0;
            assert!(
                (g(&e, "sensors.encoder.angle") - (t + 0.01)).abs() <= d,
                "k {k}"
            );
            assert!(g(&e, "sensors.encoder.angle_raw") < TAU);
            assert!((g(&e, "sensors.encoder.error") + 0.01).abs() <= d);
        }
    }
}
