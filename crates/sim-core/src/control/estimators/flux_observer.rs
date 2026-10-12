//! Sensorless nonlinear flux observer `[Lee2010]` (EQ-CTRL-09):
//!
//! η = x − L_s·i_αβ,  ẋ = v_αβ − R·i_αβ + (γ/2)·η·(λ_m² − |η|²),  θ̂e = atan2(η_β, η_α),
//!
//! integrated with forward Euler at the control rate from the measured currents and the
//! voltage applied over the last period; speed from a PLL on θ̂e (EQ-CTRL-08b).
//! `est.valid`: |ω̂e| > ω_min and ||η| − λ_m| < 0.2·λ_m. Outputs are electrical.

use serde::{Deserialize, Serialize};

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;
use crate::physics::motor::electrical::clarke;

use super::velocity::Pll;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ObserverParams {
    /// Phase resistance R and synchronous inductance L_s the observer assumes.
    pub r: f64,
    pub ls: f64,
    /// Magnet flux λ_m [Wb].
    pub lambda: f64,
    /// Gain γ (default 1000/λ_m²).
    pub gamma: f64,
    /// Validity speed threshold ω_min [rad/s electrical].
    pub omega_min: f64,
}

impl ObserverParams {
    pub fn default_gamma(lambda: f64) -> f64 {
        1000.0 / (lambda * lambda)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct FluxObserver {
    pub p: ObserverParams,
    pub t: f64,
    /// Observer state x [Wb], αβ.
    pub x: [f64; 2],
    pub pll: Pll,
}

impl FluxObserver {
    /// x starts at L_s·i (η = 0) on the first update.
    pub fn new(p: ObserverParams, t: f64, pll: Pll) -> Self {
        Self {
            p,
            t,
            x: [f64::NAN; 2],
            pll,
        }
    }

    /// One step from the measured i_αβ and the voltage applied over the last period;
    /// returns (θ̂e, ω̂e, valid, |η|).
    pub fn update(&mut self, i: [f64; 2], v: [f64; 2]) -> (f64, f64, bool, f64) {
        let p = &self.p;
        if self.x[0].is_nan() {
            self.x = [p.ls * i[0], p.ls * i[1]];
        }
        let eta = [self.x[0] - p.ls * i[0], self.x[1] - p.ls * i[1]];
        let n2 = eta[0] * eta[0] + eta[1] * eta[1];
        let k = 0.5 * p.gamma * (p.lambda * p.lambda - n2);
        for a in 0..2 {
            self.x[a] += self.t * (v[a] - p.r * i[a] + k * eta[a]);
        }
        let eta = [self.x[0] - p.ls * i[0], self.x[1] - p.ls * i[1]];
        let th = eta[1].atan2(eta[0]);
        let (_, w) = self.pll.update(th);
        let mag = eta[0].hypot(eta[1]);
        let valid = w.abs() > p.omega_min && (mag - p.lambda).abs() < 0.2 * p.lambda;
        (th, w, valid, mag)
    }
}

/// The observer as a block: reads three phase currents and v_α, v_β (signal paths given
/// by the builder, e.g. `sensors.adc.i_a..c` and the controller's commanded voltages);
/// writes `est.theta`, `est.omega` (electrical) and `est.valid`.
pub struct FluxObserverBlock {
    period: SimTime,
    i: [SignalId; 3],
    v: [SignalId; 2],
    theta: SignalId,
    omega: SignalId,
    valid: SignalId,
    init: FluxObserver,
    obs: FluxObserver,
}

impl FluxObserverBlock {
    pub fn new(
        obs: FluxObserver,
        period: SimTime,
        currents: [&str; 3],
        voltages: [&str; 2],
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let i = [
            bus.id(currents[0])?,
            bus.id(currents[1])?,
            bus.id(currents[2])?,
        ];
        let v = [bus.id(voltages[0])?, bus.id(voltages[1])?];
        let mut reg = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Output);
        Ok(Self {
            period,
            i,
            v,
            theta: reg(
                "est.theta",
                "rad",
                "estimated electrical angle (flux observer)",
            )?,
            omega: reg(
                "est.omega",
                "rad/s",
                "estimated electrical speed (flux observer)",
            )?,
            valid: reg(
                "est.valid",
                "-",
                "1 while the observer estimate is trustworthy",
            )?,
            init: obs,
            obs,
        })
    }
}

impl DiscreteBlock for FluxObserverBlock {
    fn id(&self) -> &str {
        "est.observer"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::ESTIMATOR
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let g = |id: SignalId| ctx.bus.get(id);
        let (ia, ib) = clarke(g(self.i[0]), g(self.i[1]), g(self.i[2]));
        let v = [g(self.v[0]), g(self.v[1])];
        let (th, w, ok, _) = self.obs.update([ia, ib], v);
        ctx.bus.set(self.theta, th);
        ctx.bus.set(self.omega, w);
        ctx.bus.set(self.valid, f64::from(u8::from(ok)));
        Ok(())
    }
    fn reset(&mut self) {
        self.obs = self.init;
    }
    fn save(&self) -> serde_json::Value {
        serde_json::to_value(self.obs).unwrap_or_default()
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        self.obs = serde_json::from_value(v.clone()).map_err(|e| e.to_string())?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::estimators::velocity::wrap_pi;
    use std::f64::consts::FRAC_PI_2;

    const T: f64 = 50e-6;
    const R: f64 = 0.5;
    const L: f64 = 1e-3;
    const LAMBDA: f64 = 0.01;

    fn observer(r_err: f64) -> FluxObserver {
        let p = ObserverParams {
            r: R * (1.0 + r_err),
            ls: L,
            lambda: LAMBDA,
            gamma: ObserverParams::default_gamma(LAMBDA),
            omega_min: 50.0,
        };
        FluxObserver::new(
            p,
            T,
            Pll::design(T, 2.0 * std::f64::consts::PI * 100.0, 1.0),
        )
    }

    /// A PMSM at constant electrical speed ω with a pure i_q of 2 A, sampled every T:
    /// i_αβ = 2·e^{j(θ+π/2)}, ψ = L·i + λ·e^{jθ}, and the voltage over the last period
    /// v = (ψ_k − ψ_{k−1})/T + R·i_k (consistent with the Euler step). Returns the angle
    /// errors and the last validity flag.
    fn run(obs: &mut FluxObserver, w: f64, secs: f64) -> (Vec<f64>, bool) {
        let n = (secs / T) as usize;
        let at = |k: usize| {
            let th = w * T * k as f64;
            let i = [2.0 * (th + FRAC_PI_2).cos(), 2.0 * (th + FRAC_PI_2).sin()];
            let psi = [L * i[0] + LAMBDA * th.cos(), L * i[1] + LAMBDA * th.sin()];
            (th, i, psi)
        };
        let (mut errs, mut valid) = (vec![], false);
        let (_, _, mut psi_prev) = at(0);
        for k in 1..=n {
            let (th, i, psi) = at(k);
            let v = [
                (psi[0] - psi_prev[0]) / T + R * i[0],
                (psi[1] - psi_prev[1]) / T + R * i[1],
            ];
            psi_prev = psi;
            let (est, _, ok, _) = obs.update(i, v);
            errs.push(wrap_pi(th - est).abs().to_degrees());
            valid = ok;
        }
        (errs, valid)
    }

    #[test]
    fn converges_above_ten_percent_speed() {
        // Rated ω_e = 2000 rad/s here; 10 % = 200 rad/s. From η = 0 (x = L·i) with the
        // default γ the angle error is below 5° within 100 ms (EQ-CTRL-09; ~55 ms at
        // 200 rad/s) and stays there.
        for w in [200.0, 2000.0] {
            let mut obs = observer(0.0);
            let (errs, valid) = run(&mut obs, w, 0.3);
            let k100 = (0.1 / T) as usize;
            let worst = errs[k100..].iter().cloned().fold(0.0, f64::max);
            assert!(worst < 5.0, "ω_e {w}: {worst}° after 100 ms");
            assert!(valid, "ω_e {w}: valid");
            assert!(
                (obs.pll.omega / w - 1.0).abs() < 0.01,
                "ω̂ {}",
                obs.pll.omega
            );
        }
        // With a 3 % resistance error the high-speed estimate stays good (sensors-free
        // margin, EQ-CTRL-09 table: ~0.5° at 700 rad/s).
        let mut obs = observer(0.03);
        let (errs, _) = run(&mut obs, 2000.0, 0.05);
        let mean = errs[errs.len() / 2..].iter().sum::<f64>() / (errs.len() / 2) as f64;
        assert!(mean < 5.0, "{mean}°");
    }

    #[test]
    fn fails_at_very_low_speed_as_documented() {
        // ω_e = 1.4 rad/s with a 3 % R error: the back-EMF (14 mV) is smaller than the
        // resistance error (30 mV), so the angle is lost (EQ-CTRL-09 table: tens of
        // degrees) and the estimate is flagged invalid.
        let mut obs = observer(0.03);
        let (errs, valid) = run(&mut obs, 1.4, 2.0);
        let mean = errs[errs.len() / 2..].iter().sum::<f64>() / (errs.len() / 2) as f64;
        assert!(mean > 20.0, "mean error {mean}°");
        assert!(!valid);
    }

    #[test]
    fn block_reads_phase_currents_and_writes_estimates() {
        use crate::engine::engine::Engine;
        use crate::engine::plant::Plant;
        let mut bus = SignalBus::new();
        let mut reg = |n: &str| bus.register(n, "-", "", SignalKind::Input).unwrap();
        let i = [
            reg("sensors.adc.i_a"),
            reg("sensors.adc.i_b"),
            reg("sensors.adc.i_c"),
        ];
        let _v = [reg("ctrl.v_alpha"), reg("ctrl.v_beta")];
        let blk = FluxObserverBlock::new(
            observer(0.0),
            SimTime::from_nanos(50_000),
            ["sensors.adc.i_a", "sensors.adc.i_b", "sensors.adc.i_c"],
            ["ctrl.v_alpha", "ctrl.v_beta"],
            &mut bus,
        )
        .unwrap();
        // i_a = 1 A, i_b = i_c = −0.5 A: i_α = 1, x starts at L·i, so η = 0 → θ̂ = 0.
        for (id, x) in i.iter().zip([1.0, -0.5, -0.5]) {
            bus.set(*id, x);
        }
        let mut e = Engine::new(Plant::new(), bus, vec![Box::new(blk)], 1e-5);
        e.step_until(SimTime::from_nanos(1)).unwrap();
        let g = |n: &str| e.bus.get(e.bus.id(n).unwrap());
        assert!(g("est.theta").is_finite() && g("est.omega").is_finite());
        assert_eq!(g("est.valid"), 0.0);
    }
}
