//! Encoder offset calibration (EQ-CTRL-11), as a routine run on an engine with a FOC
//! current loop and an encoder:
//!
//! 1. velocity loop off (its bypass gives i_q* = 0), current loop forced to θe = 0,
//!    i_d* = I_cal for t_align: the rotor's d axis aligns with phase a;
//! 2. θ̂e0 = −p·θ_raw mod 2π, written to `ctrl.foc.angle_offset`;
//! 3. sweep the forced angle through n electrical turns: p̂ = 2πn/|Δθm|, direction from the
//!    sign of Δθm;
//! 4. restore: forced angle off, i_d* = 0, the velocity loop back on (bumpless: its law
//!    tracked the bypass).

use std::f64::consts::TAU;

use crate::engine::commands::{ChangeSource, EngineCommand};
use crate::engine::engine::Engine;
use crate::engine::time::SimTime;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationOptions {
    /// Alignment current I_cal [A] and time [s].
    pub current: f64,
    pub align_time: f64,
    /// Sweep: electrical turns n and speed [electrical rad/s] (0 turns: no check).
    pub turns: u32,
    pub sweep_speed: f64,
    /// The pole pairs the controller assumes.
    pub pole_pairs: f64,
}

impl CalibrationOptions {
    /// EQ-CTRL-11 defaults: I_cal = 0.5·I_lim, 0.5 s, 2 turns at one turn per second.
    pub fn defaults(i_lim: f64, pole_pairs: f64) -> Self {
        Self {
            current: 0.5 * i_lim,
            align_time: 0.5,
            turns: 2,
            sweep_speed: TAU,
            pole_pairs,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationResult {
    /// θ̂e0 [rad electrical], in [0, 2π).
    pub offset: f64,
    /// p̂ and the direction (+1: the encoder counts up for a forward electrical sweep).
    pub pole_pairs: Option<f64>,
    pub direction: Option<f64>,
}

fn set(e: &mut Engine, path: &str, value: f64) -> Result<(), String> {
    e.bus.id(path).map_err(|err| err.to_string())?;
    e.queue(EngineCommand::SetSignal {
        path: path.into(),
        value,
        source: ChangeSource::Internal,
    });
    Ok(())
}

fn sig(e: &Engine, path: &str) -> Result<f64, String> {
    Ok(e.bus.get(e.bus.id(path).map_err(|err| err.to_string())?))
}

fn run_for(e: &mut Engine, dt: f64) -> Result<(), String> {
    let t = e.time.as_secs_f64() + dt;
    e.step_until(SimTime::from_secs_f64(t))
        .map_err(|err| err.to_string())
}

/// Runs the calibration from the engine's current time and applies the offset.
pub fn calibrate(e: &mut Engine, o: &CalibrationOptions) -> Result<CalibrationResult, String> {
    let velocity = e.get_param("controller.velocity.enabled");
    if velocity.is_some() {
        e.queue(EngineCommand::SetParam {
            path: "controller.velocity.enabled".into(),
            value: 0.0,
            source: ChangeSource::Internal,
        });
    }
    set(e, "ctrl.torque_ref", 0.0)?;
    set(e, "ctrl.foc.angle_cmd", 0.0)?;
    set(e, "ctrl.foc.angle_forced", 1.0)?;
    set(e, "ctrl.foc.id_ref", o.current)?;
    run_for(e, o.align_time)?;
    let raw = sig(e, "sensors.encoder.angle")?;
    let offset = (-o.pole_pairs * raw).rem_euclid(TAU);
    // Pole-pair and direction check: a slow forced sweep, read the encoder's travel.
    let (mut pole_pairs, mut direction) = (None, None);
    if o.turns > 0 {
        // Sweep n + 1 turns at constant speed and read the encoder at the commanded
        // angles π and π + 2πn: the rotor lags the command by the same angle at both, so
        // the lag (and the undamped swing after a stop) does not enter the travel.
        let span = TAU * f64::from(o.turns);
        let steps_per_turn = 1000;
        let total = steps_per_turn * (o.turns as usize + 1);
        let dt = TAU / o.sweep_speed / steps_per_turn as f64;
        let (mut a, mut b) = (None, None);
        for k in 1..=total {
            let cmd = TAU * k as f64 / steps_per_turn as f64;
            set(e, "ctrl.foc.angle_cmd", cmd)?;
            run_for(e, dt)?;
            if k == steps_per_turn / 2 {
                a = Some(sig(e, "sensors.encoder.angle")?);
            }
            if k == steps_per_turn / 2 + steps_per_turn * o.turns as usize {
                b = Some(sig(e, "sensors.encoder.angle")?);
            }
        }
        if let (Some(a), Some(b)) = (a, b) {
            let travel = b - a;
            if travel != 0.0 {
                pole_pairs = Some(span / travel.abs());
                direction = Some(travel.signum());
            }
        }
        // Back to rest at the final angle (≡ 0) before handing over.
        run_for(e, 0.5 * o.align_time)?;
    }
    set(e, "ctrl.foc.angle_offset", offset)?;
    set(e, "ctrl.foc.id_ref", 0.0)?;
    set(e, "ctrl.foc.angle_forced", 0.0)?;
    if velocity.is_some() {
        e.queue(EngineCommand::SetParam {
            path: "controller.velocity.enabled".into(),
            value: velocity.unwrap_or(1.0),
            source: ChangeSource::Internal,
        });
    }
    Ok(CalibrationResult {
        offset,
        pole_pairs,
        direction,
    })
}
