//! Fidelity tiers and step-size rules (POLISHED_IDEA §3.2, EQ-NUM-04).

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Tier {
    Ideal,
    Standard,
    Detailed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InverterMode {
    /// Phase voltages applied directly (tests, skeleton).
    Ideal,
    Averaged,
    Switching,
}

/// Feature switches; tier presets fill them, users may override each flag.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FidelityConfig {
    pub tier: Tier,
    pub inverter_mode: InverterMode,
    pub enable_cogging: bool,
    pub enable_iron_loss: bool,
    pub enable_saturation: bool,
    pub enable_thermal: bool,
    pub enable_sensor_nonideal: bool,
    /// User override of the step limit [s]; the computed rule still applies as an upper bound.
    pub dt_max_override: Option<f64>,
}

impl FidelityConfig {
    pub fn preset(tier: Tier) -> Self {
        let (avg, cog, fe, sat, th, sens) = match tier {
            Tier::Ideal => (InverterMode::Averaged, false, false, false, false, false),
            Tier::Standard => (InverterMode::Averaged, true, true, false, true, true),
            Tier::Detailed => (InverterMode::Switching, true, true, true, true, true),
        };
        Self {
            tier,
            inverter_mode: avg,
            enable_cogging: cog,
            enable_iron_loss: fe,
            enable_saturation: sat,
            enable_thermal: th,
            enable_sensor_nonideal: sens,
            dt_max_override: None,
        }
    }
}

impl Default for FidelityConfig {
    fn default() -> Self {
        Self::preset(Tier::Standard)
    }
}

/// Time constants that bound the integration step (EQ-NUM-04). All in seconds / rad/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StepLimits {
    /// Electrical time constant L/R.
    pub tau_e: f64,
    /// Fastest controller period.
    pub t_ctrl: f64,
    /// Highest natural frequency present (mechanical/contact) [rad/s]; 0 if none.
    pub omega_max: f64,
    /// Source resistance × bus capacitance (0 for an ideal source).
    pub tau_bus: f64,
    /// PWM period (switching mode only).
    pub t_pwm: f64,
}

/// `dt_max` per EQ-NUM-04 (with the user override as an additional cap).
pub fn dt_max(cfg: &FidelityConfig, lim: &StepLimits) -> f64 {
    let mut dt = (lim.tau_e / 20.0).min(lim.t_ctrl / 2.0);
    if lim.omega_max > 0.0 {
        dt = dt.min(1.0 / (20.0 * lim.omega_max));
    }
    if lim.tau_bus > 0.0 {
        dt = dt.min(lim.tau_bus / 20.0);
    }
    if cfg.inverter_mode == InverterMode::Switching {
        dt = dt.min(lim.t_pwm / 200.0);
    }
    if let Some(o) = cfg.dt_max_override {
        dt = dt.min(o);
    }
    dt
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lim() -> StepLimits {
        StepLimits {
            tau_e: 2.5e-3,
            t_ctrl: 50e-6,
            omega_max: 0.0,
            tau_bus: 9.4e-6,
            t_pwm: 50e-6,
        }
    }

    #[test]
    fn averaged_rule_uses_smallest_constraint() {
        let cfg = FidelityConfig::preset(Tier::Standard);
        // tau_e/20 = 125 µs, t_ctrl/2 = 25 µs, tau_bus/20 = 0.47 µs → bus RC dominates.
        assert!((dt_max(&cfg, &lim()) - 9.4e-6 / 20.0).abs() < 1e-15);
        let ideal_src = StepLimits {
            tau_bus: 0.0,
            ..lim()
        };
        assert!((dt_max(&cfg, &ideal_src) - 25e-6).abs() < 1e-15);
    }

    #[test]
    fn switching_and_override_caps() {
        let cfg = FidelityConfig::preset(Tier::Detailed);
        let l = StepLimits {
            tau_bus: 0.0,
            ..lim()
        };
        assert!((dt_max(&cfg, &l) - 50e-6 / 200.0).abs() < 1e-15);
        let mut c2 = FidelityConfig::preset(Tier::Ideal);
        c2.dt_max_override = Some(1e-6);
        assert_eq!(dt_max(&c2, &l), 1e-6);
    }

    #[test]
    fn tier_presets_match_spec_table() {
        let i = FidelityConfig::preset(Tier::Ideal);
        assert!(!i.enable_cogging && !i.enable_thermal);
        let s = FidelityConfig::preset(Tier::Standard);
        assert!(s.enable_cogging && s.enable_iron_loss && s.enable_thermal && !s.enable_saturation);
        let d = FidelityConfig::preset(Tier::Detailed);
        assert!(d.enable_saturation && d.inverter_mode == InverterMode::Switching);
    }
}
