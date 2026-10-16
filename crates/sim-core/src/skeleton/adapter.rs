//! Engine assembler on the production plant (P03.T12, rebuilt in P05.T11, controller
//! from P09.T04): abc motor (`physics::motor`) + drivetrain + ideal source or inverter
//! and DC bus + sensors, driven by the FOC family (`control::foc`). The dq plant
//! survives only as a test fixture (`fixtures::dq_pmsm`).

use super::params::PmsmParams;
use crate::control::ControllerSpec;
use crate::control::foc::current_loop::{BusVoltage, FocSignals};
use crate::control::foc::{self, FocIo, FocSpec, MotorModel};
use crate::control::framework::{PlantInfo, Scaled};
use crate::control::mit::MitBlock;
use crate::control::open_loop::{OpenLoopBlock, OpenLoopSignals};
use crate::control::six_step::{self, CommutationAngle};
use crate::engine::block::DiscreteBlock;
use crate::engine::engine::Engine;
use crate::engine::plant::Plant;
use crate::engine::signals::SignalBus;
use crate::physics::inverter::ideal::IdealVoltageSource;
use crate::physics::inverter::leg::{Devices, LegState, SwitchingInverter};
use crate::physics::inverter::pwm::{Pwm, PwmConfig};
use crate::physics::mech::backlash::Backlash;
use crate::physics::mech::friction::Friction;
use crate::physics::mech::gearbox::Gearbox;
use crate::physics::mech::load::Load;
use crate::physics::mech::rotor::{RotorParams, RotorRigid};
use crate::physics::motor::backemf::Shape;
use crate::physics::motor::cogging::{Cogging, CoggingParams};
use crate::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, LambdaTemperature, LegInputs, MotorElectrical,
    RTemperature, SatParams,
};
use crate::physics::motor::iron_loss::{IronLoss, IronLossParams};
use crate::physics::sensors::SensorSpec;
use crate::physics::supply::bus::{Chopper, DcBus, Source};
use crate::physics::thermal::{Thermal, ThermalInputs, ThermalParams};

/// What drives the motor terminals.
#[derive(Debug, Clone, Default)]
pub enum Drive {
    /// Phase voltages applied directly (tests, skeleton; `inverter_mode = ideal`).
    #[default]
    Ideal,
    /// SVPWM → inverter (averaged or switching) → DC bus with its source.
    Inverter(InverterOptions),
}

#[derive(Debug, Clone)]
pub struct InverterOptions {
    /// Switching (PWM edges + dead time) instead of averaged.
    pub switching: bool,
    pub devices: Devices,
    pub f_pwm: f64,
    pub dead_time: f64,
    /// t_r + t_f [s] (switching loss).
    pub t_rise_fall: f64,
    /// Dead-time sign smoothing scale [A].
    pub i_eps: f64,
    pub source: Source,
    /// Bus capacitance [F].
    pub c_bus: f64,
    pub chopper: Option<Chopper>,
}

/// Options for the skeleton engine.
#[derive(Debug, Clone)]
pub struct SkeletonOptions {
    pub locked: bool,
    /// The FOC family (EQ-CTRL-03/04).
    pub controller: ControllerSpec,
    /// Ideal source voltage [V] (`Drive::Ideal`).
    pub v_supply: f64,
    /// Integration step limit [s].
    pub dt_max: f64,
    /// Gearbox on the output shaft (EQ-MECH-02/03); bypass by default.
    pub gearbox: Gearbox,
    /// Load on the output shaft (EQ-MECH-06/07).
    pub load: Load,
    /// EQ-MECH-09: live load changes keep momentum instead of speed.
    pub conserve_momentum: bool,
    /// Initial load angle θ_L [rad] (arm mounting offset) and speed ω_L [rad/s].
    pub theta_l0: f64,
    pub omega_l0: f64,
    /// Stick–slip friction: motor static/Coulomb (viscous is `PmsmParams::b`) and load.
    pub friction: (Friction, Friction),
    /// Two-inertia backlash (EQ-MECH-04); `None` = rigid.
    pub backlash: Option<Backlash>,
    /// Thermal network with R(T) and λ(T) coupling (EQ-THERM-01…03).
    pub thermal: Option<ThermalOptions>,
    /// Terminal drive (ideal source, or inverter + bus).
    pub drive: Drive,
    /// Gear ratio N (load-side setpoints × N = motor side).
    pub ratio: f64,
    /// Cogging (EQ-MOT-08), when the fidelity tier enables it.
    pub cogging: Option<CoggingParams>,
    /// Iron loss (EQ-MOT-09), when the fidelity tier enables it.
    pub iron: Option<IronLossParams>,
    /// Co-energy saturation (EQ-MOT-10), Detailed tier.
    pub saturation: Option<SatParams>,
    /// Back-EMF shape (EQ-MOT-03).
    pub shape: Shape,
    /// Sensor blocks (EQ-SENS-*), built after the plant's signals.
    pub sensors: SensorSpec,
}

impl Default for SkeletonOptions {
    fn default() -> Self {
        Self {
            locked: false,
            controller: ControllerSpec::Foc(FocSpec::skeleton()),
            v_supply: 24.0,
            dt_max: 5e-6,
            gearbox: Gearbox::BYPASS,
            load: Load::None,
            conserve_momentum: false,
            theta_l0: 0.0,
            omega_l0: 0.0,
            friction: (Friction::default(), Friction::default()),
            backlash: None,
            thermal: None,
            drive: Drive::Ideal,
            ratio: 1.0,
            cogging: None,
            iron: None,
            saturation: None,
            shape: Shape::Sinusoidal,
            sensors: SensorSpec::default(),
        }
    }
}

/// Thermal network and its couplings into the motor.
#[derive(Debug, Clone, Copy)]
pub struct ThermalOptions {
    pub params: ThermalParams,
    /// Reference temperature of R and λ [K].
    pub t_ref: f64,
    /// Magnet remanence coefficient α_Br [1/K].
    pub alpha_br: f64,
}

/// Build the skeleton engine (6020-class motor + 20 kHz FOC).
pub fn build_engine(opts: SkeletonOptions) -> Engine {
    build_engine_with(PmsmParams::skeleton_6020(), opts)
}

/// Skeleton engine with given motor parameters (`mp.j` = rotor only) and controller.
pub fn build_engine_with(mp: PmsmParams, opts: SkeletonOptions) -> Engine {
    // INVARIANT: without sensors every signal is registered by the skeleton itself, and
    // the default controller spec is valid.
    try_build_engine_with(mp, opts).expect("skeleton engine")
}

/// As `build_engine_with`; fails when a sensor's input signal does not exist (e.g. an
/// ADC bus-voltage channel without a DC bus) or the controller spec is invalid.
pub fn try_build_engine_with(mp: PmsmParams, opts: SkeletonOptions) -> Result<Engine, String> {
    let six = matches!(opts.controller, ControllerSpec::SixStep(_));
    if six && matches!(opts.drive, Drive::Ideal) {
        return Err(
            "six-step needs an inverter (fidelity.inverter_mode averaged or switching): the ideal source cannot float a phase"
                .into(),
        );
    }
    let mut bus = SignalBus::new();
    // The controller's signals first: the ideal source reads its v_α, v_β.
    let err = |e: crate::engine::signals::SignalError| e.to_string();
    let foc_sig = match &opts.controller {
        ControllerSpec::Foc(_) | ControllerSpec::Mit(_) => {
            Some(FocSignals::register(&mut bus).map_err(err)?)
        }
        _ => None,
    };
    let ol_sig = match &opts.controller {
        ControllerSpec::OpenLoop(_) => Some(OpenLoopSignals::register(&mut bus).map_err(err)?),
        _ => None,
    };
    let v_ab = foc_sig
        .map(|s| (s.v_alpha, s.v_beta))
        .or(ol_sig.map(|s| (s.v_alpha, s.v_beta)));
    // Front end: the ideal source, or bus + inverter (+ PWM block).
    // Built once per engine and consumed below; its size does not matter.
    #[allow(clippy::large_enum_variant)]
    enum Front {
        Ideal(IdealVoltageSource),
        Inv(DcBus, SwitchingInverter, InverterOptions),
    }
    // INVARIANT: as above, the front-end modules register fixed, unique paths into this
    // fresh bus, so their constructors cannot fail.
    let front = match &opts.drive {
        Drive::Ideal => {
            // INVARIANT: six-step (no v_α, v_β) on the ideal drive was refused above.
            let (va, vb) = v_ab.expect("a voltage-command family");
            Front::Ideal(
                IdealVoltageSource::new(&mut bus, va, vb, opts.v_supply)
                    .expect("unique skeleton signal"),
            )
        }
        Drive::Inverter(o) => {
            let dc = DcBus::new(o.source.clone(), o.c_bus, o.chopper, &mut bus)
                .expect("unique skeleton signal");
            let mut inv = SwitchingInverter::new(o.devices, dc.v_id(), &mut bus)
                .expect("unique skeleton signal")
                .with_switching_loss(o.t_rise_fall, o.f_pwm)
                .without_dc_input();
            if six {
                inv = inv.with_six_step();
            }
            if !o.switching {
                inv = inv
                    .averaged(&mut bus, o.dead_time * o.f_pwm, o.i_eps)
                    .expect("unique skeleton signal");
            }
            Front::Inv(dc, inv, o.clone())
        }
    };
    let terminals = match &front {
        Front::Ideal(s) => s.outputs_ids(),
        Front::Inv(_, inv, _) => inv.outputs_ids(),
    };
    let rp = RotorParams {
        j: mp.j,
        j_load: 0.0,
        b: mp.b,
        pole_pairs: mp.p,
        theta0: opts.theta_l0 * opts.gearbox.n,
        omega0: opts.omega_l0 * opts.gearbox.n,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .expect("unique skeleton signal")
        .with_locked(opts.locked)
        .with_gearbox(opts.gearbox)
        .with_load(opts.load.clone(), opts.conserve_momentum)
        .with_friction(opts.friction.0, opts.friction.1);
    let rotor = match opts.backlash {
        Some(b) => rotor.with_backlash(b),
        None => rotor,
    };
    let id = |bus: &SignalBus, p: &str| bus.id(p).expect("registered above");
    let inp = ElectricalInputs {
        v: terminals,
        theta_e: id(&bus, "motor.theta_e"),
        omega_e: id(&bus, "motor.omega_e"),
    };
    let ep = ElectricalParams {
        r: mp.r,
        ld: mp.ld,
        lq: mp.lq,
        lambda: mp.lambda,
        pole_pairs: mp.p,
        shape: opts.shape.clone(),
        i_d0: 0.0,
        i_q0: 0.0,
        // θe = p·θm with θm = N·θ_L (the flux starts aligned with the rotor).
        theta_e0: mp.p * opts.theta_l0 * opts.gearbox.n,
        saturation: opts.saturation,
    };
    let thermal = opts.thermal.map(|o| {
        let th = Thermal::new(o.params, &mut bus).expect("unique skeleton signal");
        (th, o)
    });
    let mut motor = MotorElectrical::new(ep, inp, &mut bus).expect("unique skeleton signal");
    if let Front::Inv(dc, _, o) = &front {
        let legs = ["inverter.leg_a", "inverter.leg_b", "inverter.leg_c"].map(|n| id(&bus, n));
        motor = motor
            .with_legs(LegInputs {
                legs,
                v_bus: dc.v_id(),
                v_f: o.devices.v_f,
                dead_time: o.dead_time,
            })
            .with_internal_terminals();
    }
    if let Some((_, o)) = &thermal {
        motor = motor
            .with_r_temperature(RTemperature {
                t_winding: id(&bus, "thermal.t_winding"),
                t_ref: o.t_ref,
            })
            .with_lambda_temperature(LambdaTemperature {
                t_magnet: id(&bus, "thermal.t_magnet"),
                alpha_br: o.alpha_br,
                t_ref: o.t_ref,
                t_start: o.params.t_init,
            });
    }
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
    let mut plant = Plant::new();
    let mut extra_blocks: Vec<Box<dyn DiscreteBlock>> = vec![];
    let mut duties = None;
    let mut inverter = None;
    match front {
        Front::Ideal(src) => {
            plant.add(Box::new(rotor));
            plant.add(Box::new(src));
        }
        Front::Inv(dc, inv, o) => {
            let inv = inv.with_motor(&bus).expect("motor registered above");
            let load = vec![id(&bus, "inverter.i_dc"), id(&bus, "inverter.i_sw")];
            let v_bus = dc.v_id();
            // The bus first (its voltage depends on its state only), the inverter after
            // the motor (it needs this stage's currents).
            plant.add(Box::new(dc.with_load(load)));
            plant.add(Box::new(rotor));
            let duty = crate::physics::inverter::leg::duty_ids(&mut bus).expect("duty inputs");
            if o.switching {
                let cfg = PwmConfig {
                    period_ns: (1e9 / o.f_pwm).round() as i64,
                    dead_ns: (o.dead_time * 1e9).round() as i64,
                };
                let mut pwm = Pwm::new(cfg, &mut bus).expect("pwm signals");
                if six {
                    pwm = pwm.with_six_step(&mut bus).expect("pattern signals");
                }
                extra_blocks.push(Box::new(pwm));
            } else if !six {
                // Averaged: every leg is driven by its duty (six-step writes the legs).
                inv.set_legs(&mut bus, [LegState::High; 3]);
            }
            duties = Some((duty, v_bus));
            inverter = Some(inv);
        }
    }
    if let Some((th, _)) = thermal {
        // Before the motor: its outputs (temperatures) feed R(T) and λ(T).
        let inputs = ThermalInputs {
            p_cu: bus.id("motor.p_cu").ok(),
            p_fe: bus.id("motor.p_fe").ok(),
            p_fric: bus.id("motor.p_fric").ok(),
            omega: bus.id("motor.omega").ok(),
        };
        plant.add(Box::new(th.with_inputs(inputs)));
    }
    plant.add(Box::new(motor));
    if let Some(c) = cogging {
        plant.add(Box::new(c));
    }
    if let Some(m) = iron {
        plant.add(Box::new(m));
    }
    if let Some(inv) = inverter {
        plant.add(Box::new(inv));
    }
    // PWM (modulator priority), then the sensors, then the controller family.
    let mut blocks: Vec<Box<dyn DiscreteBlock>> = extra_blocks;
    blocks.extend(opts.sensors.blocks(&mut bus).map_err(|e| e.to_string())?);
    let sc = |b: &SignalBus, p: &str, g: f64| Scaled::of(b, p, g).map_err(|e| e.to_string());
    let n = opts.ratio;
    // Feedback: sensor readings where a sensor is fitted, else the true values (ideal
    // feedback). The speeds are the true ones until the estimators are built (P09.T15).
    let currents = if opts.sensors.adc.is_some() {
        ["sensors.adc.i_a", "sensors.adc.i_b", "sensors.adc.i_c"]
    } else {
        ["motor.i_a", "motor.i_b", "motor.i_c"]
    }
    .map(|p| id(&bus, p));
    let (angle_e, theta_l) = if opts.sensors.encoder.is_some() {
        (
            sc(&bus, "sensors.encoder.angle", mp.p)?,
            sc(&bus, "sensors.encoder.angle", 1.0 / n)?,
        )
    } else {
        let load = match bus.id("load.theta") {
            Ok(_) => sc(&bus, "load.theta", 1.0)?,
            Err(_) => sc(&bus, "motor.theta", 1.0 / n)?,
        };
        (sc(&bus, "motor.theta_e", 1.0)?, load)
    };
    let v_bus = match duties {
        None => BusVoltage::Fixed(opts.v_supply),
        Some((_, v)) => match bus.id("sensors.adc.v_bus") {
            Ok(m) => BusVoltage::Signal(m),
            Err(_) => BusVoltage::Signal(v),
        },
    };
    let io = FocIo {
        currents,
        angle_e,
        omega_e: sc(&bus, "motor.omega_e", 1.0)?,
        omega_m: sc(&bus, "motor.omega", 1.0)?,
        theta_l,
        v_bus,
        duties: duties.map(|(d, _)| d),
    };
    let info = PlantInfo {
        ratio: n,
        kt: 1.5 * mp.p * mp.lambda,
        // Gains are designed for the whole shaft inertia.
        j_motor: opts.gearbox.j_eq(mp.j, opts.load.inertia()),
    };
    let model = MotorModel {
        r: mp.r,
        ld: mp.ld,
        lq: mp.lq,
        lambda: mp.lambda,
        pole_pairs: mp.p,
    };
    match &opts.controller {
        ControllerSpec::Foc(spec) => {
            let sig = foc_sig.expect("registered for FOC");
            blocks.extend(foc::build(spec, &model, &info, &io, sig, &mut bus)?);
        }
        ControllerSpec::Mit(spec) => {
            let sig = foc_sig.expect("registered for MIT");
            blocks.extend(foc::build(&spec.foc, &model, &info, &io, sig, &mut bus)?);
            // Load-side speed: the drivetrain's, else the motor's / N (true until P09.T15).
            let omega_l = match bus.id("load.omega") {
                Ok(_) => sc(&bus, "load.omega", 1.0)?,
                Err(_) => sc(&bus, "motor.omega", 1.0 / n)?,
            };
            blocks.push(Box::new(MitBlock::new(
                spec, io.theta_l, omega_l, n, info.kt, sig.iq_ref, &mut bus,
            )?));
        }
        ControllerSpec::OpenLoop(spec) => {
            let sp = crate::control::setpoint::Setpoints::get_or_register(&mut bus).map_err(err)?;
            blocks.push(Box::new(OpenLoopBlock::new(
                spec.clone(),
                sp.omega,
                n * mp.p,
                id(&bus, "motor.theta_e"),
                ol_sig.expect("registered for open loop"),
                io.duties,
                io.v_bus,
            )?));
        }
        ControllerSpec::SixStep(spec) => {
            let Some((duty, v_dc)) = duties else {
                return Err("six-step needs an inverter".into());
            };
            // Averaged: the pattern drives the legs directly; switching: the PWM block.
            let switching = matches!(&opts.drive, Drive::Inverter(o) if o.switching);
            let legs = if switching {
                [
                    "inverter.pattern_a",
                    "inverter.pattern_b",
                    "inverter.pattern_c",
                ]
            } else {
                ["inverter.leg_a", "inverter.leg_b", "inverter.leg_c"]
            }
            .map(|p| id(&bus, p));
            let angle = match bus.id("sensors.hall.sector") {
                Ok(h) => CommutationAngle::Hall(h),
                Err(_) => CommutationAngle::True(id(&bus, "motor.theta_e")),
            };
            blocks.extend(six_step::build(
                spec,
                &info,
                &model,
                &opts.shape,
                angle,
                io.omega_m,
                legs,
                duty,
                bus.id("sensors.adc.v_bus").unwrap_or(v_dc),
                &mut bus,
            )?);
        }
    }
    // A scene with a position loop holds its initial pose (EQ-CTRL-04).
    let th = id(&bus, "ctrl.theta_ref");
    bus.set(th, opts.theta_l0);
    Ok(Engine::new(plant, bus, blocks, opts.dt_max))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::commands::{ChangeSource, EngineCommand};
    use crate::engine::time::SimTime;

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
            controller: crate::control::ControllerSpec::Foc(crate::control::foc::FocSpec {
                open_loop_vq: Some(1.0),
                ..crate::control::foc::FocSpec::skeleton()
            }),
            dt_max: 2.5e-7,
            ..Default::default()
        });
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        let r = e.bus.get(e.bus.id("energy.residual").unwrap());
        assert!(r.abs() < 1e-6, "energy residual {r}");
    }
}
