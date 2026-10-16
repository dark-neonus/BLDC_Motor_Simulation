//! Field-oriented control (EQ-CTRL-03/04): the current loop and the velocity/position
//! cascade, built as one family from a plain-data `FocSpec`.

pub mod current_loop;
pub mod position_loop;
pub mod velocity_loop;

use crate::engine::block::DiscreteBlock;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::{SimTime, period_from_hz};

use super::framework::{LoopBlock, LoopConfig, LoopLaw, PlantInfo, Scaled};
use super::pid::{AntiWindup, Pid};
use super::setpoint::Setpoints;
use current_loop::{
    BusVoltage, CurrentLoopConfig, CurrentLoopInputs, FocCurrentLoop, FocSignals, M_MAX,
    SpeedSource,
};
use velocity_loop::Prefiltered;

/// One outer loop: rate, enable, design bandwidth or explicit gains.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopSpec {
    pub enabled: bool,
    pub rate_hz: f64,
    /// Design bandwidth [rad/s]; `None` → the EQ-CTRL-04 default.
    pub omega: Option<f64>,
    pub kp: Option<f64>,
    pub ki: Option<f64>,
}

/// The FOC family in SI units (EQ-CTRL-03/04).
#[derive(Debug, Clone, PartialEq)]
pub struct FocSpec {
    pub current_rate_hz: f64,
    /// Current-loop bandwidth ω_c [rad/s]; `None` → 2π f/20.
    pub omega_c: Option<f64>,
    pub m_max: f64,
    pub aw: AntiWindup,
    /// |i_q*| limit [A].
    pub i_max: f64,
    /// Load-side speed limit [rad/s] and speed-reference ramp [rad/s²] (∞ = none).
    pub speed_limit: f64,
    pub ramp: f64,
    /// No velocity loop: the velocity block is built disabled (torque setpoint only).
    pub velocity: Option<LoopSpec>,
    pub position: Option<LoopSpec>,
    pub prefilter: bool,
    /// Test mode: v_d = 0, v_q fixed, no current control.
    pub open_loop_vq: Option<f64>,
}

impl FocSpec {
    /// The walking-skeleton controller: 20 kHz current loop at 1 kHz bandwidth, 5 A,
    /// velocity loop at the current rate with the default design.
    pub fn skeleton() -> Self {
        Self {
            current_rate_hz: 20e3,
            omega_c: Some(2.0 * std::f64::consts::PI * 1000.0),
            m_max: M_MAX,
            aw: AntiWindup::Clamping,
            i_max: 5.0,
            speed_limit: f64::INFINITY,
            ramp: f64::INFINITY,
            velocity: Some(LoopSpec {
                enabled: true,
                rate_hz: 20e3,
                omega: None,
                kp: None,
                ki: None,
            }),
            position: None,
            prefilter: false,
            open_loop_vq: None,
        }
    }
}

/// The motor model the controller designs with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorModel {
    pub r: f64,
    pub ld: f64,
    pub lq: f64,
    pub lambda: f64,
    pub pole_pairs: f64,
}

/// The signals the family reads, chosen by the builder (sensor readings, or the true
/// values for ideal feedback).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FocIo {
    pub currents: [SignalId; 3],
    /// θ̂e = gain·signal (encoder: p; `motor.theta_e`: 1).
    pub angle_e: Scaled,
    /// ω̂e for the decoupling.
    pub omega_e: Scaled,
    /// Motor-side speed for the velocity loop [rad/s].
    pub omega_m: Scaled,
    /// Load angle for the position loop [rad].
    pub theta_l: Scaled,
    pub v_bus: BusVoltage,
    pub duties: Option<[SignalId; 3]>,
}

fn period(hz: f64, what: &str) -> Result<SimTime, String> {
    period_from_hz(hz)
        .map(|p| p.period)
        .map_err(|e| format!("{what}: {e}"))
}

/// Builds the family: position (optional) → velocity → current blocks, in that order
/// (equal priority, so outer loops update first at shared instants). `sig` must be
/// registered before the plant that reads `ctrl.foc.v_alpha`/`v_beta`.
pub fn build(
    spec: &FocSpec,
    motor: &MotorModel,
    plant: &PlantInfo,
    io: &FocIo,
    sig: FocSignals,
    bus: &mut SignalBus,
) -> Result<Vec<Box<dyn DiscreteBlock>>, String> {
    let e = |err: SignalError| err.to_string();
    let sp = Setpoints::get_or_register(bus).map_err(e)?;
    let n = plant.ratio;
    let t_c = period(spec.current_rate_hz, "controller.current.rate")?;
    let omega_c = spec
        .omega_c
        .unwrap_or_else(|| CurrentLoopConfig::default_omega_c(t_c));
    let omega_v = spec
        .velocity
        .and_then(|v| v.omega)
        .unwrap_or(omega_c / 10.0);
    let mut blocks: Vec<Box<dyn DiscreteBlock>> = vec![];
    // Velocity reference: the position loop's output, or N·ω_ref without one.
    let vel_ref = match &spec.position {
        Some(p) => {
            let w_cmd = bus
                .register(
                    "ctrl.omega_cmd",
                    "rad/s",
                    "velocity-loop reference (motor side)",
                    SignalKind::Output,
                )
                .map_err(e)?;
            let omega_p = p.omega.unwrap_or(omega_v / 10.0);
            // Explicit gains are load-side; ×N to the motor-side speed command.
            let (kp, ki) = position_loop::design(p.kp.unwrap_or(omega_p), n, p.ki);
            let lim = n * spec.speed_limit;
            let cfg = LoopConfig::from_hz(p.enabled, p.rate_hz, (-lim, lim), f64::INFINITY)?;
            let law = Pid::pi(kp, ki, (-lim, lim), spec.aw);
            blocks.push(Box::new(LoopBlock::new(
                "controller.position",
                cfg,
                super::framework::LoopWiring {
                    reference: Scaled {
                        id: sp.theta,
                        gain: 1.0,
                    },
                    feedback: io.theta_l,
                    bypass: Scaled {
                        id: sp.omega,
                        gain: n,
                    },
                    out: w_cmd,
                },
                Box::new(law),
            )));
            Scaled {
                id: w_cmd,
                gain: 1.0,
            }
        }
        None => Scaled {
            id: sp.omega,
            gain: n,
        },
    };
    // Velocity loop (built disabled without one: the torque setpoint drives i_q).
    let v = spec.velocity.unwrap_or(LoopSpec {
        enabled: false,
        rate_hz: spec.current_rate_hz,
        omega: None,
        kp: None,
        ki: None,
    });
    // Each explicit gain overrides its designed value (K_i = 0: a P-only loop).
    let (kp_d, ki_d) = velocity_loop::design(omega_v, plant.j_motor, plant.kt);
    let (kp, ki) = (v.kp.unwrap_or(kp_d), v.ki.unwrap_or(ki_d));
    let i_lim = (-spec.i_max, spec.i_max);
    let cfg = LoopConfig::from_hz(v.enabled, v.rate_hz, i_lim, n * spec.ramp)?;
    let pi = Pid::pi(kp, ki, i_lim, spec.aw);
    let law: Box<dyn LoopLaw> = if spec.prefilter {
        Box::new(Prefiltered::new(pi, omega_v))
    } else {
        Box::new(pi)
    };
    blocks.push(Box::new(
        LoopBlock::new(
            "controller.velocity",
            cfg,
            super::framework::LoopWiring {
                reference: vel_ref,
                feedback: io.omega_m,
                bypass: Scaled {
                    id: sp.torque,
                    gain: 1.0 / (n * plant.kt),
                },
                out: sig.iq_ref,
            },
            law,
        )
        // The speed limit also bounds a velocity setpoint (motor side).
        .with_reference_limit(n * spec.speed_limit),
    ));
    // Current loop.
    let cfg = CurrentLoopConfig {
        period: t_c,
        phase: SimTime::from_nanos(0),
        r: motor.r,
        ld: motor.ld,
        lq: motor.lq,
        lambda: motor.lambda,
        omega_c,
        m_max: spec.m_max,
        aw: spec.aw,
    };
    let inputs = CurrentLoopInputs {
        currents: io.currents,
        angle: io.angle_e,
        angle_offset: 0.0,
        speed: SpeedSource::Signal(io.omega_e),
        v_bus: io.v_bus,
    };
    let current =
        FocCurrentLoop::new(cfg, inputs, sig, io.duties)?.with_open_loop_vq(spec.open_loop_vq);
    blocks.push(Box::new(current));
    Ok(blocks)
}
