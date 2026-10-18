//! Velocity estimation from a measured angle (EQ-CTRL-08):
//!
//! (a) difference + low-pass: ω̂_k = (1 − α)·ω̂_{k−1} + α·wrap(θ_k − θ_{k−1})/T,
//!     α = T/(T + τ_f);
//! (b) PLL: e = wrap(θ_meas − θ̂), ω̂ += T·K_i·e, θ̂ += T·(ω̂ + K_p·e),
//!     K_p = 2ζω_n, K_i = ω_n².
//!
//! `VelocityEstimator` runs either at a fixed rate on an angle signal and writes
//! `est.theta` and `est.omega`.

use std::f64::consts::{PI, TAU};

use serde::{Deserialize, Serialize};

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

/// Wrap to (−π, π].
pub fn wrap_pi(x: f64) -> f64 {
    let y = x.rem_euclid(TAU);
    if y > PI { y - TAU } else { y }
}

/// Difference + first-order low-pass (EQ-CTRL-08a).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DiffLpf {
    pub alpha: f64,
    pub t: f64,
    last: Option<f64>,
    pub omega: f64,
}

impl DiffLpf {
    /// Sample period T and filter time constant τ_f [s].
    pub fn new(t: f64, tau_f: f64) -> Self {
        Self {
            alpha: t / (t + tau_f),
            t,
            last: None,
            omega: 0.0,
        }
    }

    /// One update; returns (θ̂, ω̂). θ̂ is the measurement itself.
    pub fn update(&mut self, theta: f64) -> (f64, f64) {
        if let Some(prev) = self.last {
            let raw = wrap_pi(theta - prev) / self.t;
            self.omega = (1.0 - self.alpha) * self.omega + self.alpha * raw;
        }
        self.last = Some(theta);
        (theta, self.omega)
    }
}

/// Phase-locked loop (EQ-CTRL-08b). θ̂ is unwrapped.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pll {
    pub kp: f64,
    pub ki: f64,
    pub t: f64,
    pub theta: f64,
    pub omega: f64,
    started: bool,
}

impl Pll {
    /// From the natural frequency ω_n [rad/s] and damping ζ.
    pub fn design(t: f64, omega_n: f64, zeta: f64) -> Self {
        Self {
            kp: 2.0 * zeta * omega_n,
            ki: omega_n * omega_n,
            t,
            theta: 0.0,
            omega: 0.0,
            started: false,
        }
    }

    /// One update; returns (θ̂, ω̂). The first sample initialises θ̂ = θ_meas. With the
    /// EQ-CTRL-08 update order θ̂ after the update predicts the next sample, θ(t_k + T).
    pub fn update(&mut self, theta: f64) -> (f64, f64) {
        if !self.started {
            self.started = true;
            self.theta = theta;
        }
        let e = wrap_pi(theta - self.theta);
        self.omega += self.t * self.ki * e;
        self.theta += self.t * (self.omega + self.kp * e);
        (self.theta, self.omega)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Method {
    DiffLpf(DiffLpf),
    Pll(Pll),
}

impl Method {
    fn update(&mut self, theta: f64) -> (f64, f64) {
        match self {
            Method::DiffLpf(f) => f.update(theta),
            Method::Pll(p) => p.update(theta),
        }
    }
}

pub struct VelocityEstimator {
    period: SimTime,
    input: SignalId,
    theta: SignalId,
    omega: SignalId,
    init: Method,
    m: Method,
}

impl VelocityEstimator {
    /// Reads the angle signal `input` (e.g. `sensors.encoder.angle`); registers
    /// `est.theta` and `est.omega`. The method's T must equal `period`.
    pub fn new(
        m: Method,
        period: SimTime,
        input: &str,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let input = bus.id(input)?;
        let theta = bus.register("est.theta", "rad", "estimated angle", SignalKind::Output)?;
        let omega = bus.register("est.omega", "rad/s", "estimated speed", SignalKind::Output)?;
        Ok(Self {
            period,
            input,
            theta,
            omega,
            init: m,
            m,
        })
    }
}

impl DiscreteBlock for VelocityEstimator {
    fn id(&self) -> &str {
        "est.velocity"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::ESTIMATOR
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let (th, w) = self.m.update(ctx.bus.get(self.input));
        ctx.bus.set(self.theta, th);
        ctx.bus.set(self.omega, w);
        Ok(())
    }
    fn reset(&mut self) {
        self.m = self.init;
    }
    fn save(&self) -> serde_json::Value {
        serde_json::to_value(self.m).unwrap_or_default()
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        self.m = serde_json::from_value(v.clone()).map_err(|e| e.to_string())?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: f64 = 50e-6;

    #[test]
    fn diff_lpf_tracks_a_ramp_with_the_filter_lag() {
        let (tau, w0) = (2e-3, 300.0);
        let mut f = DiffLpf::new(T, tau);
        let a = T / (T + tau);
        // A wrapped ramp (the encoder register): the wrap is undone by the difference.
        for k in 0..400 {
            let (_, w) = f.update((w0 * T * k as f64).rem_euclid(TAU));
            // First-order step response from the second sample: ω0·(1 − (1 − α)^k).
            let want = w0 * (1.0 - (1.0 - a).powi(k));
            assert!((w - want).abs() < 1e-9 * w0, "k {k}: {w} vs {want}");
        }
    }

    #[test]
    fn pll_tracks_a_ramp_without_steady_state_lag() {
        let (wn, w0) = (2.0 * PI * 50.0, 300.0);
        let mut p = Pll::design(T, wn, 1.0);
        let mut err = 0.0;
        let mut w = 0.0;
        for k in 0..40_000 {
            let th = w0 * T * k as f64;
            let (est, om) = p.update(th.rem_euclid(TAU));
            // θ̂ after the update is the prediction for the next sample.
            err = wrap_pi(th + w0 * T - est);
            w = om;
        }
        // Type-2 loop: zero steady-state error on a ramp; 2 s ≫ 1/ω_n.
        assert!(err.abs() < 1e-9, "θ error {err}");
        assert!((w / w0 - 1.0).abs() < 1e-9, "ω̂ {w}");
    }

    /// For ζ = 1 the −3 dB bandwidth of θ̂/θ is 2.48·ω_n (EQ-CTRL-08): a small sinusoid at
    /// that frequency comes through at 1/√2 (rtol 2 %), and at ω_n/10 at ≈ 1.
    #[test]
    fn v_ctrl_004_pll_bandwidth_matches_the_design_formula() {
        let wn = 2.0 * PI * 20.0;
        let gain = |w: f64| {
            let mut p = Pll::design(T, wn, 1.0);
            let amp = 0.1;
            let n = (20.0 / wn / T) as usize + (10.0 * TAU / w / T) as usize;
            let tail = (2.0 * TAU / w / T) as usize;
            let mut peak: f64 = 0.0;
            for k in 0..n {
                let (est, _) = p.update(amp * (w * T * k as f64).sin());
                if k >= n - tail {
                    peak = peak.max(est.abs());
                }
            }
            peak / amp
        };
        let g3 = gain(2.48 * wn);
        assert!(
            (g3 * 2f64.sqrt() - 1.0).abs() < 0.02,
            "|H(2.48 ω_n)| = {g3}"
        );
        let g_low = gain(0.1 * wn);
        assert!((g_low - 1.0).abs() < 0.02, "|H(ω_n/10)| = {g_low}");
    }

    #[test]
    fn block_writes_est_signals_and_restores() {
        use crate::engine::engine::Engine;
        use crate::engine::plant::Plant;
        use crate::engine::snapshot::Snapshot;
        let mk = || {
            let mut bus = SignalBus::new();
            bus.register("sensors.encoder.angle", "rad", "", SignalKind::Input)
                .unwrap();
            let b = VelocityEstimator::new(
                Method::Pll(Pll::design(T, 300.0, 1.0)),
                SimTime::from_nanos(50_000),
                "sensors.encoder.angle",
                &mut bus,
            )
            .unwrap();
            Engine::new(Plant::new(), bus, vec![Box::new(b)], 1e-5)
        };
        let set = |e: &mut Engine, x: f64| {
            let id = e.bus.id("sensors.encoder.angle").unwrap();
            e.bus.set(id, x);
        };
        let mut a = mk();
        set(&mut a, 0.5);
        a.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        let snap = Snapshot::from_bytes(&a.snapshot().to_bytes().unwrap()).unwrap();
        let mut b = mk();
        b.restore(&snap).unwrap();
        set(&mut a, 0.7);
        set(&mut b, 0.7);
        a.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        b.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        let g = |e: &Engine, n: &str| e.bus.get(e.bus.id(n).unwrap());
        assert_eq!(g(&a, "est.theta"), g(&b, "est.theta"));
        assert_eq!(g(&a, "est.omega"), g(&b, "est.omega"));
        assert!((g(&a, "est.theta") - 0.7).abs() < 0.05);
    }
}
