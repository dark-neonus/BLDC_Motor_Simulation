//! Six-step commutation with Hall sensors (EQ-CTRL-05):
//!
//! 1. the Hall sector (or the true angle) selects the leg pattern (EQ-INV-07);
//! 2. the command u ∈ [−1, 1] (`ctrl.six_step.duty`, from `ctrl.duty_ref` or a speed PI)
//!    gives the duty |u| of the High leg and, by its sign, the direction (reverse swaps
//!    High and Low);
//! 3. an optional advance shifts the table earlier; with Halls the angle inside a sector
//!    is interpolated from the last sector's duration.
//!
//! The speed PI is a framework `LoopBlock` (`controller.speed_loop`) designed on the
//! DC-equivalent model: K_p = ω_s τ_m k_e / V_bus, K_i = ω_s k_e / V_bus.

use std::f64::consts::PI;

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;
use crate::physics::inverter::leg::LegState;
use crate::physics::inverter::modulation::six_step;
use crate::physics::motor::backemf::Shape;

const SECTOR: f64 = PI / 3.0;

/// Sector-average line-to-line EMF shape (k_max − k_min) of a machine: k_e = p·λ·this
/// [V per mech rad/s]; 3√3/π for a sinusoidal machine (EQ-CTRL-05).
pub fn emf_shape_ll(shape: &Shape) -> f64 {
    let n = 3600;
    (0..n)
        .map(|i| {
            let th = 2.0 * PI * (i as f64 + 0.5) / n as f64;
            let k: [f64; 3] = std::array::from_fn(|x| shape.k(th - x as f64 * 2.0 * PI / 3.0));
            k.iter().cloned().fold(f64::MIN, f64::max) - k.iter().cloned().fold(f64::MAX, f64::min)
        })
        .sum::<f64>()
        / n as f64
}

/// Speed-PI gains (K_p, K_i) for bandwidth ω_s [rad/s] (EQ-CTRL-05).
pub fn speed_design(omega_s: f64, k_e: f64, r: f64, j: f64, v_bus: f64) -> (f64, f64) {
    let tau_m = 2.0 * r * j / (k_e * k_e);
    (omega_s * tau_m * k_e / v_bus, omega_s * k_e / v_bus)
}

/// Where the commutation angle comes from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CommutationAngle {
    /// `sensors.hall.sector` (1–6 in rotation order, 0 invalid; EQ-SENS-02).
    Hall(SignalId),
    /// The true electrical angle (ideal feedback, no Halls fitted).
    True(SignalId),
}

/// Hall interpolation state: the last sector, when it was entered, and ω̂e.
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
struct Interp {
    sector: u8,
    t_edge: f64,
    omega: f64,
}

/// Duty cap from the DC model with the inverter (EQ-CTRL-05):
/// |u| ≤ (k_e|ω| + V_f + 2(R + R_on)·I_lim)/(V_bus + V_f).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DutyLimit {
    pub k_e: f64,
    /// Phase resistance R and switch on-resistance R_on [Ω], diode drop V_f [V].
    pub r: f64,
    pub r_on: f64,
    pub v_f: f64,
    pub i_max: f64,
    /// Motor speed [rad/s] and bus voltage [V] signals.
    pub omega: crate::control::framework::Scaled,
    pub v_bus: SignalId,
}

impl DutyLimit {
    pub fn cap(&self, bus: &SignalBus) -> f64 {
        let v = bus.get(self.v_bus);
        if v <= 0.0 {
            return 0.0;
        }
        let w = (self.omega.gain * bus.get(self.omega.id)).abs();
        let num = self.k_e * w + self.v_f + 2.0 * (self.r + self.r_on) * self.i_max;
        (num / (v + self.v_f)).min(1.0)
    }
}

pub struct SixStepBlock {
    limit: Option<DutyLimit>,
    period: SimTime,
    /// Advance [rad electrical].
    advance: f64,
    angle: CommutationAngle,
    cmd: SignalId,
    legs: [SignalId; 3],
    duties: [SignalId; 3],
    sector: SignalId,
    interp: Interp,
}

impl SixStepBlock {
    /// Reads `ctrl.six_step.duty` (registered here); writes the leg pattern to `legs`
    /// (`inverter.leg_x` in averaged mode, `inverter.pattern_x` with a PWM block) and the
    /// High leg's duty to `duties`.
    pub fn new(
        period: SimTime,
        advance: f64,
        angle: CommutationAngle,
        legs: [SignalId; 3],
        duties: [SignalId; 3],
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let cmd = bus.register(
            "ctrl.six_step.duty",
            "-",
            "six-step command u: duty |u|, direction sign(u)",
            SignalKind::Input,
        )?;
        let sector = bus.register(
            "ctrl.six_step.sector",
            "-",
            "commutation sector 0…5 (EQ-INV-07 order), −1 invalid",
            SignalKind::Output,
        )?;
        Ok(Self {
            limit: None,
            period,
            advance,
            angle,
            cmd,
            legs,
            duties,
            sector,
            interp: Interp::default(),
        })
    }

    pub fn with_duty_limit(mut self, l: DutyLimit) -> Self {
        self.limit = Some(l);
        self
    }

    /// θ̂e from the Hall sector, interpolated with the last sector's speed; `None` for
    /// an invalid code.
    fn hall_angle(&mut self, s: u8, t: f64) -> Option<f64> {
        if !(1..=6).contains(&s) {
            return None;
        }
        // Sector s spans [(s − 1)·60° − 30°, (s − 1)·60° + 30°] (EQ-SENS-02).
        let start = f64::from(s - 1) * SECTOR - SECTOR / 2.0;
        let i = &mut self.interp;
        if s != i.sector {
            if i.sector != 0 {
                let step = (i32::from(s) - i32::from(i.sector)).rem_euclid(6);
                let dt = t - i.t_edge;
                i.omega = match step {
                    1 if dt > 0.0 => SECTOR / dt,
                    5 if dt > 0.0 => -SECTOR / dt,
                    _ => 0.0,
                };
            }
            (i.sector, i.t_edge) = (s, t);
        }
        // Entered at its start going forward, at its end going backward.
        let edge = if i.omega >= 0.0 {
            start
        } else {
            start + SECTOR
        };
        let th = edge + i.omega * (t - i.t_edge);
        Some(th.clamp(start, start + SECTOR))
    }
}

impl DiscreteBlock for SixStepBlock {
    fn id(&self) -> &str {
        "controller"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let t = ctx.t.as_secs_f64();
        let cap = self.limit.map_or(1.0, |l| l.cap(ctx.bus));
        let u = ctx.bus.get(self.cmd).clamp(-cap, cap);
        let th = match self.angle {
            CommutationAngle::True(id) => Some(ctx.bus.get(id)),
            CommutationAngle::Hall(id) => {
                let s = ctx.bus.get(id);
                self.hall_angle(if s.is_finite() { s as u8 } else { 0 }, t)
            }
        };
        let Some(th) = th else {
            // Invalid Hall code: all legs Off.
            for (l, d) in self.legs.iter().zip(self.duties) {
                ctx.bus.set(*l, LegState::Off.to_signal());
                ctx.bus.set(d, 0.0);
            }
            ctx.bus.set(self.sector, -1.0);
            return Ok(());
        };
        let dir = if u < 0.0 { -1.0 } else { 1.0 };
        let th = th + dir * self.advance;
        let deg = th.to_degrees().rem_euclid(360.0);
        ctx.bus
            .set(self.sector, ((deg + 30.0) / 60.0).floor() % 6.0);
        let mut legs = six_step(th, 0.0);
        if dir < 0.0 {
            for l in &mut legs {
                *l = match *l {
                    LegState::High => LegState::Low,
                    LegState::Low => LegState::High,
                    LegState::Off => LegState::Off,
                };
            }
        }
        for ((out, duty), leg) in self.legs.iter().zip(self.duties).zip(legs) {
            ctx.bus.set(*out, leg.to_signal());
            let d = if leg == LegState::High { u.abs() } else { 0.0 };
            ctx.bus.set(duty, d);
        }
        Ok(())
    }
    fn reset(&mut self) {
        self.interp = Interp::default();
    }
    fn params(&self) -> Vec<(String, String)> {
        vec![("advance".into(), "rad".into())]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        (name == "advance").then_some(self.advance)
    }
    /// `controller.advance` [rad electrical], 0 … 30°.
    fn set_param(&mut self, name: &str, value: f64) -> Result<f64, String> {
        if name != "advance" {
            return Err(format!("unknown parameter `{name}`"));
        }
        if !(0.0..=SECTOR / 2.0 + 1e-12).contains(&value) {
            return Err(format!("advance must be within 0…30° (rad), got {value}"));
        }
        Ok(std::mem::replace(&mut self.advance, value))
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "advance": self.advance, "interp": self.interp })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        self.advance = v
            .get("advance")
            .and_then(serde_json::Value::as_f64)
            .ok_or("missing `advance`")?;
        self.interp = serde_json::from_value(v.get("interp").cloned().unwrap_or_default())
            .map_err(|e| e.to_string())?;
        Ok(())
    }
}

/// The six-step family in SI units.
#[derive(Debug, Clone, PartialEq)]
pub struct SixStepSpec {
    pub rate_hz: f64,
    /// Advance [rad electrical], 0 … 30°.
    pub advance: f64,
    pub speed_loop: Option<crate::control::foc::LoopSpec>,
    /// Load-side speed limit [rad/s] on the speed reference (∞ = none).
    pub speed_limit: f64,
    /// Bus voltage the speed PI is designed for [V].
    pub v_nominal: f64,
    /// Current limit I_lim [A] (duty cap, EQ-CTRL-05) and the inverter's R_on [Ω], V_f [V].
    pub i_max: f64,
    pub r_on: f64,
    pub v_f: f64,
}

/// Builds the speed loop (`controller.speed_loop`, disabled without one: `ctrl.duty_ref`
/// passes) and the commutation block. `legs` are the leg-pattern outputs, `duties` the
/// duty inputs of the inverter or PWM block.
#[allow(clippy::too_many_arguments)]
pub fn build(
    spec: &SixStepSpec,
    plant: &crate::control::framework::PlantInfo,
    motor: &crate::control::foc::MotorModel,
    shape: &Shape,
    angle: CommutationAngle,
    omega_m: crate::control::framework::Scaled,
    legs: [SignalId; 3],
    duties: [SignalId; 3],
    v_bus: SignalId,
    bus: &mut SignalBus,
) -> Result<Vec<Box<dyn DiscreteBlock>>, String> {
    use crate::control::framework::{LoopBlock, LoopConfig, LoopWiring, Scaled};
    use crate::control::pid::{AntiWindup, Pid};
    use crate::control::setpoint::Setpoints;
    let e = |err: SignalError| err.to_string();
    let period = crate::engine::time::period_from_hz(spec.rate_hz)
        .map_err(|err| format!("controller.rate: {err}"))?
        .period;
    if !(0.0..=SECTOR / 2.0 + 1e-12).contains(&spec.advance) {
        return Err(format!(
            "controller.advance must be within 0…30°, got {} rad",
            spec.advance
        ));
    }
    let k_e = motor.pole_pairs * motor.lambda * emf_shape_ll(shape);
    let block = SixStepBlock::new(period, spec.advance, angle, legs, duties, bus)
        .map_err(e)?
        .with_duty_limit(DutyLimit {
            k_e,
            r: motor.r,
            r_on: spec.r_on,
            v_f: spec.v_f,
            i_max: spec.i_max,
            omega: omega_m,
            v_bus,
        });
    let sp = Setpoints::get_or_register(bus).map_err(e)?;
    let duty_ref = match bus.id("ctrl.duty_ref") {
        Ok(id) => id,
        Err(_) => bus
            .register(
                "ctrl.duty_ref",
                "-",
                "six-step command setpoint u ∈ [−1, 1] (sign: direction)",
                SignalKind::Input,
            )
            .map_err(e)?,
    };
    let l = spec.speed_loop.unwrap_or(crate::control::foc::LoopSpec {
        enabled: false,
        rate_hz: spec.rate_hz,
        omega: None,
        kp: None,
        ki: None,
    });
    let omega_s = l.omega.unwrap_or(2.0 * PI * l.rate_hz / 50.0);
    let (kp_d, ki_d) = speed_design(omega_s, k_e, motor.r, plant.j_motor, spec.v_nominal);
    let (kp, ki) = (l.kp.unwrap_or(kp_d), l.ki.unwrap_or(ki_d));
    let n = plant.ratio;
    let speed = LoopBlock::new(
        "controller.speed_loop",
        LoopConfig::from_hz(l.enabled, l.rate_hz, (-1.0, 1.0), f64::INFINITY)?,
        LoopWiring {
            reference: Scaled {
                id: sp.omega,
                gain: n,
            },
            feedback: omega_m,
            bypass: Scaled {
                id: duty_ref,
                gain: 1.0,
            },
            out: block.cmd,
        },
        Box::new(Pid::pi(kp, ki, (-1.0, 1.0), AntiWindup::Clamping)),
    )
    .with_reference_limit(n * spec.speed_limit);
    // The speed loop first: at shared instants the commutation uses its fresh command.
    Ok(vec![Box::new(speed), Box::new(block)])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sinusoidal_ll_shape_is_3_sqrt3_over_pi() {
        let k = emf_shape_ll(&Shape::Sinusoidal);
        assert!((k - 3.0 * 3f64.sqrt() / PI).abs() < 1e-6, "{k}");
        // Trapezoidal with a 120° flat top: the line-to-line EMF is 2·k_flat throughout.
        let w = 2.0 * PI / 3.0;
        let tr = Shape::Trapezoidal { w };
        let flat = tr.k(PI / 2.0).abs();
        assert!((emf_shape_ll(&tr) - 2.0 * flat).abs() < 1e-3 * flat);
    }

    /// The Hall interpolation: entering sector 2 one sector-time after sector 1 gives
    /// ω̂ = 60°/Δt and the angle advances from the sector start, clamped at its end.
    #[test]
    fn hall_interpolation_tracks_and_clamps() {
        let mut bus = SignalBus::new();
        let mut reg = |p: &str| bus.register(p, "-", "", SignalKind::Input).unwrap();
        let hall = reg("sensors.hall.sector");
        let legs = [reg("x.l_a"), reg("x.l_b"), reg("x.l_c")];
        let duties = [reg("x.d_a"), reg("x.d_b"), reg("x.d_c")];
        let mut b = SixStepBlock::new(
            SimTime::from_nanos(50_000),
            0.0,
            CommutationAngle::Hall(hall),
            legs,
            duties,
            &mut bus,
        )
        .unwrap();
        assert_eq!(b.hall_angle(1, 0.0), Some(-SECTOR / 2.0));
        b.hall_angle(2, 0.01);
        let th = b.hall_angle(2, 0.015).unwrap();
        assert!((th - (SECTOR / 2.0 + SECTOR / 2.0)).abs() < 1e-12, "{th}");
        assert_eq!(b.hall_angle(2, 0.05), Some(1.5 * SECTOR), "clamped");
        // Backward (2 → 1): enters at the sector's end and moves down.
        b.hall_angle(1, 0.06);
        assert!(b.interp.omega < 0.0);
        assert_eq!(b.hall_angle(7, 0.07), None);
    }
}
