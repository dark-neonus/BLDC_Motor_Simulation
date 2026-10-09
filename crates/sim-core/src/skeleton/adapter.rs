//! Skeleton FOC on the production plant (P03.T12, rebuilt in P05.T11): abc motor
//! (`physics::motor`) + rigid rotor + ideal voltage source, driven by the skeleton FOC.
//! INTERIM: the FOC block is replaced in P09.T03, the ideal source in P07. The dq plant
//! survives only as a test fixture (`fixtures::dq_pmsm`).

use super::foc::{Foc, FocConfig, FocGains};
use super::model::PmsmParams;
use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::engine::Engine;
use crate::engine::plant::Plant;
use crate::engine::signals::{SignalBus, SignalId, SignalKind};
use crate::engine::time::{SimTime, period_from_hz};
use crate::physics::inverter::ideal::IdealVoltageSource;
use crate::physics::mech::rotor::{RotorParams, RotorRigid};
use crate::physics::motor::backemf::Shape;
use crate::physics::motor::cogging::{Cogging, CoggingParams};
use crate::physics::motor::electrical::{ElectricalInputs, ElectricalParams, MotorElectrical};
use crate::physics::motor::iron_loss::{IronLoss, IronLossParams};

/// Signals the FOC block reads and writes.
#[derive(Debug, Clone, Copy)]
struct FocSignals {
    v_d: SignalId,
    v_q: SignalId,
    v_alpha: SignalId,
    v_beta: SignalId,
    i_d: SignalId,
    i_q: SignalId,
    omega: SignalId,
    theta_e: SignalId,
}

// INTERIM: replaced in P09.T03.
struct FocBlock {
    foc: Foc,
    period: SimTime,
    open_loop_vq: Option<f64>,
    s: FocSignals,
    omega_ref: SignalId,
    iq_ref: SignalId,
    /// Gear ratio N: `ctrl.omega_ref` is load-side, the speed loop runs on the motor side.
    ratio: f64,
}

impl DiscreteBlock for FocBlock {
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "foc": self.foc.save_state(), "open_loop_vq": self.open_loop_vq })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let st: [f64; 5] = serde_json::from_value(v["foc"].clone()).map_err(|e| e.to_string())?;
        self.foc.restore_state(st);
        self.open_loop_vq =
            serde_json::from_value(v["open_loop_vq"].clone()).map_err(|e| e.to_string())?;
        Ok(())
    }
    fn id(&self) -> &str {
        "ctrl"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn priority(&self) -> u8 {
        priority::CONTROLLER
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let (vd, vq) = match self.open_loop_vq {
            Some(v) => (0.0, v),
            None => {
                self.foc.omega_ref = self.ratio * ctx.bus.get(self.omega_ref);
                self.foc.update(
                    ctx.bus.get(self.s.i_d),
                    ctx.bus.get(self.s.i_q),
                    ctx.bus.get(self.s.omega),
                )
            }
        };
        ctx.bus.set(self.s.v_d, vd);
        ctx.bus.set(self.s.v_q, vq);
        // Inverse Park at the measured angle; held until the next tick (ZOH).
        let (sn, cs) = ctx.bus.get(self.s.theta_e).sin_cos();
        ctx.bus.set(self.s.v_alpha, cs * vd - sn * vq);
        ctx.bus.set(self.s.v_beta, sn * vd + cs * vq);
        ctx.bus.set(self.iq_ref, self.foc.iq_ref);
        Ok(())
    }
    fn reset(&mut self) {
        self.foc.reset();
    }
}

/// Options for the skeleton engine.
#[derive(Debug, Clone)]
pub struct SkeletonOptions {
    pub locked: bool,
    pub open_loop_vq: Option<f64>,
    /// Integration step limit [s].
    pub dt_max: f64,
    /// Reflected gearbox/load inertia added to `PmsmParams::j` by the caller [kg·m²].
    pub extra_j: f64,
    /// Gear ratio N (load-side setpoints × N = motor side).
    pub ratio: f64,
    /// Cogging (EQ-MOT-08), when the fidelity tier enables it.
    pub cogging: Option<CoggingParams>,
    /// Iron loss (EQ-MOT-09), when the fidelity tier enables it.
    pub iron: Option<IronLossParams>,
}

impl Default for SkeletonOptions {
    fn default() -> Self {
        Self {
            locked: false,
            open_loop_vq: None,
            dt_max: 5e-6,
            extra_j: 0.0,
            ratio: 1.0,
            cogging: None,
            iron: None,
        }
    }
}

/// Build the skeleton engine (6020-class motor + 20 kHz FOC).
pub fn build_engine(opts: SkeletonOptions) -> Engine {
    build_engine_with(PmsmParams::skeleton_6020(), FocConfig::skeleton(), opts)
}

/// Skeleton engine with given motor parameters (`mp.j` = rotor only) and controller.
pub fn build_engine_with(mp: PmsmParams, cfg: FocConfig, opts: SkeletonOptions) -> Engine {
    let mut bus = SignalBus::new();
    // INVARIANT: the skeleton registers a fixed, unique set of paths into a fresh bus,
    // so `register` cannot fail here.
    let mut reg = |p: &str, u: &str, d: &str, k: SignalKind| {
        bus.register(p, u, d, k).expect("unique skeleton signal")
    };
    let v_d = reg(
        "ctrl.foc.v_d",
        "V",
        "d-axis voltage command",
        SignalKind::Output,
    );
    let v_q = reg(
        "ctrl.foc.v_q",
        "V",
        "q-axis voltage command",
        SignalKind::Output,
    );
    let v_alpha = reg(
        "ctrl.foc.v_alpha",
        "V",
        "α voltage command",
        SignalKind::Output,
    );
    let v_beta = reg(
        "ctrl.foc.v_beta",
        "V",
        "β voltage command",
        SignalKind::Output,
    );
    let omega_ref = reg(
        "ctrl.omega_ref",
        "rad/s",
        "speed setpoint (load side)",
        SignalKind::Input,
    );
    let iq_ref = reg(
        "ctrl.foc.iq_ref",
        "A",
        "q-axis current reference",
        SignalKind::Output,
    );
    let source = IdealVoltageSource::new(&mut bus, v_alpha, v_beta, cfg.v_bus)
        .expect("unique skeleton signal");
    let rp = RotorParams {
        j: mp.j,
        j_load: opts.extra_j,
        b: mp.b,
        pole_pairs: mp.p,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .expect("unique skeleton signal")
        .with_locked(opts.locked);
    let id = |bus: &SignalBus, p: &str| bus.id(p).expect("registered above");
    let inp = ElectricalInputs {
        v: source.outputs_ids(),
        theta_e: id(&bus, "motor.theta_e"),
        omega_e: id(&bus, "motor.omega_e"),
    };
    let ep = ElectricalParams {
        r: mp.r,
        ld: mp.ld,
        lq: mp.lq,
        lambda: mp.lambda,
        pole_pairs: mp.p,
        shape: Shape::Sinusoidal,
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus).expect("unique skeleton signal");
    let mut rotor = rotor.with_internal(id(&bus, "motor.torque_em"));
    let cogging = match opts.cogging {
        Some(c) => {
            let m =
                Cogging::new(c, id(&bus, "motor.theta"), &mut bus).expect("unique skeleton signal");
            rotor = rotor.with_internal(m.torque_id());
            Some(m)
        }
        None => None,
    };
    let iron = match opts.iron {
        Some(p) => {
            let m = IronLoss::new(p, id(&bus, "motor.omega"), &mut bus)
                .expect("unique skeleton signal");
            rotor = rotor.with_internal(m.torque_id());
            Some(m)
        }
        None => None,
    };
    let s = FocSignals {
        v_d,
        v_q,
        v_alpha,
        v_beta,
        i_d: id(&bus, "motor.i_d"),
        i_q: id(&bus, "motor.i_q"),
        omega: id(&bus, "motor.omega"),
        theta_e: id(&bus, "motor.theta_e"),
    };
    let mut plant = Plant::new();
    plant.add(Box::new(rotor));
    plant.add(Box::new(source));
    plant.add(Box::new(motor));
    if let Some(c) = cogging {
        plant.add(Box::new(c));
    }
    if let Some(m) = iron {
        plant.add(Box::new(m));
    }
    let period = period_from_hz(1.0 / cfg.dt)
        .map(|p| p.period)
        .unwrap_or(SimTime(50_000));
    // Gains are designed for the whole shaft inertia.
    let design = PmsmParams {
        j: mp.j + opts.extra_j,
        ..mp
    };
    let blocks: Vec<Box<dyn DiscreteBlock>> = vec![Box::new(FocBlock {
        foc: Foc::new(cfg, FocGains::design(&design, &cfg)),
        period,
        open_loop_vq: opts.open_loop_vq,
        s,
        omega_ref,
        iq_ref,
        ratio: opts.ratio,
    })];
    Engine::new(plant, bus, blocks, opts.dt_max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::commands::{ChangeSource, EngineCommand};

    #[test]
    fn engine_skeleton_reaches_speed_with_closed_energy_balance() {
        let mut e = build_engine(SkeletonOptions::default());
        e.queue(EngineCommand::SetSignal {
            path: "ctrl.omega_ref".into(),
            value: 20.0,
            source: ChangeSource::Internal,
        });
        e.step_until(SimTime::from_secs_f64(0.3)).unwrap();
        let w = e.bus.get(e.bus.id("motor.omega").unwrap());
        assert!((w - 20.0).abs() < 0.02 * 20.0, "omega {w}");
        let r = e.bus.get(e.bus.id("energy.residual").unwrap());
        assert!(r.abs() < 1e-3, "energy residual {r}");
    }

    #[test]
    fn locked_rotor_energy_balance_closes() {
        let mut e = build_engine(SkeletonOptions {
            locked: true,
            open_loop_vq: Some(1.0),
            dt_max: 2.5e-7,
            ..Default::default()
        });
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        let r = e.bus.get(e.bus.id("energy.residual").unwrap());
        assert!(r.abs() < 1e-6, "energy residual {r}");
    }
}
