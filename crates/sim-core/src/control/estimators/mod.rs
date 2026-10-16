//! Estimators (EQ-CTRL-08/09), and building them from a scene: one estimator owns
//! `est.*` — the flux observer when requested (or sensorless), else a velocity estimator
//! on the encoder angle.

pub mod flux_observer;
pub mod velocity;

use std::f64::consts::PI;

use crate::engine::block::DiscreteBlock;
use crate::engine::signals::SignalBus;
use crate::engine::time::SimTime;

use flux_observer::{FluxObserver, FluxObserverBlock, ObserverParams};
use velocity::{DiffLpf, Method, Pll, VelocityEstimator};

/// Velocity-estimator settings; `None` fields take the EQ-CTRL-08 defaults.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum VelocitySpec {
    /// PLL: ω_n [rad/s] (default 2π f_s/50) and ζ (default 1).
    Pll {
        omega_n: Option<f64>,
        zeta: Option<f64>,
    },
    /// Difference + LPF: τ_f [s] (default 10 samples).
    DiffLpf { tau_f: Option<f64> },
    /// The default (a PLL with default gains).
    #[default]
    Default,
}

/// Flux-observer settings; `None` fields take the EQ-CTRL-09 defaults.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ObserverSpec {
    pub gamma: Option<f64>,
    /// ω_min [rad/s electrical].
    pub omega_min: Option<f64>,
    /// PLL ω_n [rad/s] and ζ.
    pub pll_omega_n: Option<f64>,
    pub pll_zeta: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct EstimatorSpec {
    pub velocity: VelocitySpec,
    /// A flux observer (replaces the velocity estimator).
    pub observer: Option<ObserverSpec>,
}

/// The velocity estimator on `sensors.encoder.angle`, sampled every `period`.
pub fn velocity_block(
    spec: VelocitySpec,
    period: SimTime,
    bus: &mut SignalBus,
) -> Result<Box<dyn DiscreteBlock>, String> {
    let t = period.as_secs_f64();
    let m = match spec {
        VelocitySpec::DiffLpf { tau_f } => {
            Method::DiffLpf(DiffLpf::new(t, tau_f.unwrap_or(10.0 * t)))
        }
        VelocitySpec::Pll { omega_n, zeta } => Method::Pll(Pll::design(
            t,
            omega_n.unwrap_or(2.0 * PI / t / 50.0),
            zeta.unwrap_or(1.0),
        )),
        VelocitySpec::Default => Method::Pll(Pll::design(t, 2.0 * PI / t / 50.0, 1.0)),
    };
    let b = VelocityEstimator::new(m, period, "sensors.encoder.angle", bus)
        .map_err(|e| e.to_string())?;
    Ok(Box::new(b))
}

/// The motor values the observer assumes and the nominal bus voltage (for ω_min).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObserverModel {
    pub r: f64,
    pub ls: f64,
    pub lambda: f64,
    pub v_bus: f64,
}

impl ObserverModel {
    /// ω_min, or its default 0.05·V_bus/(√3·λ_m) (EQ-CTRL-09).
    pub fn default_omega_min(&self, given: Option<f64>) -> f64 {
        given.unwrap_or(0.05 * self.v_bus / (3f64.sqrt() * self.lambda))
    }
}

/// The flux observer at the current-loop `period`, on the given phase-current signals
/// and the applied voltage command `ctrl.foc.v_alpha/v_beta` (EQ-CTRL-09). Refuses
/// γλ²T ≥ 2 (forward Euler on the radial dynamics).
pub fn observer_block(
    spec: ObserverSpec,
    m: ObserverModel,
    period: SimTime,
    currents: [&str; 3],
    bus: &mut SignalBus,
) -> Result<Box<dyn DiscreteBlock>, String> {
    let t = period.as_secs_f64();
    let gamma = spec
        .gamma
        .unwrap_or(ObserverParams::default_gamma(m.lambda));
    if gamma * m.lambda * m.lambda * t >= 2.0 {
        return Err(format!(
            "sensors.observer.gamma: γλ²T = {:.3} ≥ 2 makes the observer's forward-Euler step unstable (EQ-CTRL-09); lower γ or raise the control rate",
            gamma * m.lambda * m.lambda * t
        ));
    }
    let p = ObserverParams {
        r: m.r,
        ls: m.ls,
        lambda: m.lambda,
        gamma,
        omega_min: m.default_omega_min(spec.omega_min),
    };
    let pll = Pll::design(
        t,
        spec.pll_omega_n.unwrap_or(2.0 * PI / t / 100.0),
        spec.pll_zeta.unwrap_or(1.0),
    );
    let b = FluxObserverBlock::new(
        FluxObserver::new(p, t, pll),
        period,
        currents,
        ["ctrl.foc.v_alpha", "ctrl.foc.v_beta"],
        bus,
    )
    .map_err(|e| e.to_string())?;
    Ok(Box::new(b))
}
