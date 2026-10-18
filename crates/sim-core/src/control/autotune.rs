//! Auto-tune (EQ-CTRL-12) on a separate engine built from a copy of the scene (D-006):
//!
//! - R: the current loop holds two d currents at a forced θe = 0; R = Δv_d/Δi_d (the
//!   difference cancels the dead-time and diode offsets);
//! - L: from the high level, a forced step of v_d to the low level; the current decays
//!   with τ = L/R (63 % point);
//! - J: two q currents on the free rotor; J = K_t Δi/Δω̇ (the difference cancels
//!   friction and constant loads);
//! - λ_m: coasting with i ≈ 0, λ_m = v_q/ω_e; K_t = 1.5 p λ̂ refines J.
//!
//! Gains: ω_c = 2π f/20 (K_p = ω_c L̂, K_i = ω_c R̂), ω_v = ω_c/10 (K_p = ω_v Ĵ/K̂_t,
//! K_i = K_p ω_v/4), ω_p = ω_v/5, MIT K_p = J_L ω_mit², K_d = 1.4 √(K_p J_L) with
//! ω_mit = ω_v/5. `apply` writes them into the scene only on request.

use std::f64::consts::PI;

use sim_model::param::{Param, Raw, Source};
use sim_model::params::{ControllerParams, LoopParams};
use sim_model::scene::ResolvedScene;
use sim_model::units::Kind;

use crate::build::{build_engine, pmsm_from};
use crate::control::foc::current_loop::M_MAX;
use crate::engine::commands::{ChangeSource, EngineCommand};
use crate::engine::engine::Engine;
use crate::engine::time::SimTime;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Measured {
    /// R [Ω] as the current loop sees it (winding R(T) + switch R_on), L [H],
    /// J [kg·m², motor side, total], λ_m [Wb].
    pub r: f64,
    pub l: f64,
    pub j: f64,
    pub lambda: f64,
    /// Winding temperature during the R test [K] (None without a thermal model).
    pub t_winding: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gains {
    pub omega_c: f64,
    pub current: (f64, f64),
    pub omega_v: f64,
    pub velocity: (f64, f64),
    /// Load-side position gain [1/s] (= ω_p).
    pub omega_p: f64,
    pub mit_kp: f64,
    pub mit_kd: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Autotune {
    pub measured: Measured,
    pub gains: Gains,
}

/// The current-loop rate and current limit of the scene's controller (20 kHz otherwise).
fn rate_and_limit(scene: &ResolvedScene) -> Result<(f64, f64), String> {
    let si = |p: &Param, k: Kind| p.value.si(k).map_err(|e| e.to_string());
    let (rate, limits) = match &scene.controller {
        ControllerParams::Foc {
            current, limits, ..
        }
        | ControllerParams::Mit {
            current, limits, ..
        } => (si(&current.rate, Kind::Frequency)?, limits),
        ControllerParams::SixStep { limits, .. }
        | ControllerParams::OpenLoop { limits, .. }
        | ControllerParams::Custom { limits, .. } => (20e3, limits),
    };
    Ok((rate, si(&limits.current, Kind::Current)?))
}

/// An SI number as a parameter; auto-tune results are measured.
fn num(v: f64, source: Source) -> Param {
    Param {
        value: Raw::Number(v),
        source,
        note: Some("auto-tune (EQ-CTRL-12)".into()),
    }
}

fn loop_params(rate: f64, enabled: bool) -> LoopParams {
    LoopParams {
        enabled,
        rate: num(rate, Source::Default),
        bandwidth: None,
        kp: None,
        ki: None,
    }
}

struct Rig {
    e: Engine,
}

impl Rig {
    fn set(&mut self, path: &str, value: f64) {
        self.e.queue(EngineCommand::SetSignal {
            path: path.into(),
            value,
            source: ChangeSource::Internal,
        });
    }
    fn sig(&self, path: &str) -> Result<f64, String> {
        Ok(self
            .e
            .bus
            .get(self.e.bus.id(path).map_err(|e| e.to_string())?))
    }
    fn run(&mut self, dt: f64) -> Result<(), String> {
        let t = self.e.time.as_secs_f64() + dt;
        self.e
            .step_until(SimTime::from_secs_f64(t))
            .map_err(|e| e.to_string())
    }
    /// Mean of a signal over `dt`, sampled `n` times.
    fn mean(&mut self, path: &str, dt: f64, n: usize) -> Result<f64, String> {
        let mut s = 0.0;
        for _ in 0..n {
            self.run(dt / n as f64)?;
            s += self.sig(path)?;
        }
        Ok(s / n as f64)
    }
    /// Runs up to `dt` (n samples) while `keep(ω)` holds and |v_q| < `v_stop`; returns the
    /// least-squares slopes dω/dt and dv_q/dω_e, and the last speed.
    fn window(
        &mut self,
        dt: f64,
        n: usize,
        v_stop: f64,
        keep: impl Fn(f64) -> bool,
    ) -> Result<(f64, f64, f64), String> {
        let (mut t_w, mut w_q) = (Fit::default(), Fit::default());
        let mut w = self.sig("motor.omega")?;
        for k in 0..n {
            self.run(dt / n as f64)?;
            w = self.sig("motor.omega")?;
            let vq = self.sig("ctrl.foc.v_q")?;
            t_w.add(k as f64 * dt / n as f64, w);
            w_q.add(self.sig("motor.omega_e")?, vq);
            if vq.abs() > v_stop || !keep(w) {
                break;
            }
        }
        if t_w.n < 10 {
            return Err("J test: too few samples before the voltage limit".into());
        }
        Ok((t_w.slope(), w_q.slope(), w))
    }
}

/// Least-squares line fit.
#[derive(Default)]
struct Fit {
    n: usize,
    sx: f64,
    sy: f64,
    sxx: f64,
    sxy: f64,
}

impl Fit {
    fn add(&mut self, x: f64, y: f64) {
        self.n += 1;
        (self.sx, self.sy, self.sxx, self.sxy) =
            (self.sx + x, self.sy + y, self.sxx + x * x, self.sxy + x * y);
    }
    fn slope(&self) -> f64 {
        let n = self.n as f64;
        (n * self.sxy - self.sx * self.sy) / (n * self.sxx - self.sx * self.sx)
    }
}

/// Measures R, L, J and λ_m on a copy of the scene and computes the EQ-CTRL-12 gains.
pub fn autotune(scene: &ResolvedScene) -> Result<Autotune, String> {
    let (rate, i_lim) = rate_and_limit(scene)?;
    // The measurement copy: FOC current loop, velocity loop off (its bypass turns
    // ctrl.torque_ref into i_q* = τ/(N K_t,model), so a torque setpoint commands i_q).
    let mut copy = scene.clone();
    copy.controller = ControllerParams::Foc {
        current: loop_params(rate, true),
        velocity: Some(loop_params(rate, false)),
        position: None,
        sensorless: false,
        startup: None,
        encoder_offset: None,
        prefilter: false,
        limits: sim_model::params::Limits {
            current: num(i_lim, Source::Default),
            speed: None,
            voltage_fraction: None,
            ramp: None,
        },
    };
    let model = pmsm_from(&copy.motor, 0.0).map_err(|e| e.to_string())?;
    let ratio = copy.gearbox.as_ref().map_or(1.0, |g| g.ratio);
    let kt_model = 1.5 * model.p * model.lambda;
    let v_nominal = crate::build::supply_voltage(&copy.supply).map_err(|e| e.to_string())?;
    let built = build_engine(&copy).map_err(|e| e.to_string())?;
    let mut rig = Rig { e: built.engine };
    let t = 1.0 / rate;
    // R: two d currents at a forced θe = 0 (the rotor aligns, no torque).
    let (i_lo, i_hi) = (0.25 * i_lim, 0.5 * i_lim);
    rig.set("ctrl.foc.angle_forced", 1.0);
    rig.set("ctrl.foc.angle_cmd", 0.0);
    rig.set("ctrl.foc.id_ref", i_lo);
    rig.run(0.3)?;
    let (v1, i1) = (
        rig.mean("ctrl.foc.v_d", 0.01, 50)?,
        rig.mean("ctrl.foc.i_d", 0.01, 50)?,
    );
    rig.set("ctrl.foc.id_ref", i_hi);
    rig.run(0.1)?;
    let (v2, i2) = (
        rig.mean("ctrl.foc.v_d", 0.01, 50)?,
        rig.mean("ctrl.foc.i_d", 0.01, 50)?,
    );
    let r = (v2 - v1) / (i2 - i1);
    let t_winding = rig.sig("thermal.t_winding").ok();
    // L: forced step of v_d from the high level to the low one; i decays with τ = L/R.
    let vq = rig.sig("ctrl.foc.v_q")?;
    rig.set("ctrl.foc.vd_cmd", v1);
    rig.set("ctrl.foc.vq_cmd", vq);
    rig.set("ctrl.foc.v_forced", 1.0);
    let target = i1 + (i2 - i1) * (-1f64).exp();
    let (t0, mut tau) = (rig.e.time.as_secs_f64(), None);
    let step = t / 50.0;
    for _ in 0..200_000 {
        rig.run(step)?;
        if rig.sig("motor.i_d")? <= target {
            tau = Some(rig.e.time.as_secs_f64() - t0);
            break;
        }
    }
    let tau = tau.ok_or("L test: the current did not decay")?;
    let l = r * tau;
    // J and λ: accelerate with +I, then decelerate with −I (same direction of motion, so
    // friction cancels in the difference and the speed comes back down). λ is the slope
    // of v_q against ω_e at constant current (the dead-time offset drops out). Each
    // window stops at half the voltage limit.
    rig.set("ctrl.foc.v_forced", 0.0);
    rig.set("ctrl.foc.angle_forced", 0.0);
    rig.set("ctrl.foc.id_ref", 0.0);
    let i_j = 0.3 * i_lim;
    let v_stop = 0.5 * M_MAX * rig.sig("bus.v").unwrap_or(v_nominal) / 3f64.sqrt();
    rig.set("ctrl.torque_ref", i_j * ratio * kt_model);
    rig.run(0.002)?;
    let (a1, lambda, w_peak) = rig.window(0.05, 500, v_stop, |_| true)?;
    rig.set("ctrl.torque_ref", -i_j * ratio * kt_model);
    rig.run(0.002)?;
    let (a2, _, _) = rig.window(0.05, 500, f64::INFINITY, |w| w > 0.2 * w_peak)?;
    rig.set("ctrl.torque_ref", 0.0);
    let kt = 1.5 * model.p * lambda;
    let j = kt * 2.0 * i_j / (a1 - a2);
    let measured = Measured {
        r,
        l,
        j,
        lambda,
        t_winding,
    };
    // Gains (EQ-CTRL-12).
    let omega_c = 2.0 * PI * rate / 20.0;
    let omega_v = omega_c / 10.0;
    let kp_v = omega_v * j / kt;
    let omega_p = omega_v / 5.0;
    let j_load = j * ratio * ratio;
    let mit_kp = j_load * (omega_v / 5.0).powi(2);
    let gains = Gains {
        omega_c,
        current: (omega_c * l, omega_c * r),
        omega_v,
        velocity: (kp_v, kp_v * omega_v / 4.0),
        omega_p,
        mit_kp,
        mit_kd: 2.0 * 0.7 * (mit_kp * j_load).sqrt(),
    };
    Ok(Autotune { measured, gains })
}

/// The scene with the tuned gains written into its controller (the user confirmed).
pub fn apply(scene: &ResolvedScene, a: &Autotune) -> ResolvedScene {
    let mut s = scene.clone();
    let g = &a.gains;
    let set_current = |c: &mut LoopParams| {
        (c.kp, c.ki) = (Some(g.current.0), Some(g.current.1));
    };
    match &mut s.controller {
        ControllerParams::Foc {
            current,
            velocity,
            position,
            ..
        } => {
            set_current(current);
            if let Some(v) = velocity {
                (v.kp, v.ki) = (Some(g.velocity.0), Some(g.velocity.1));
            }
            if let Some(p) = position {
                p.kp = Some(g.omega_p);
            }
        }
        ControllerParams::Mit {
            current, kp, kd, ..
        } => {
            set_current(current);
            (*kp, *kd) = (
                Some(num(g.mit_kp, Source::Measured)),
                Some(num(g.mit_kd, Source::Measured)),
            );
        }
        _ => {}
    }
    s
}
