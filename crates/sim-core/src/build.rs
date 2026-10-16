//! Engine from a scene (P04.T13). Until the full physics lands (P05+), a scene is mapped
//! onto the skeleton dq motor + FOC: motor R, L, λ, p and the rotor inertia plus the
//! reflected gearbox/load inertia; bus voltage from the supply; FOC current limit, current
//! bandwidth and rate from the controller; the inverter, bus and sensors from their sections.
//! Parameters arrive as unit strings and are converted to SI f64 here (D-003).

use sim_model::constraints::{self, Severity};
use sim_model::param::Param;
use sim_model::params::{
    ControllerParams, EmfShape, EncoderParams, GearboxParams, LoadParams, LoopParams, MotorParams,
    OnChange, SensorCommon, SupplyParams, TierParam, Topology,
};
use sim_model::scene::ResolvedScene;
use sim_model::units::Kind;

use crate::control::ControllerSpec;
use crate::control::foc::current_loop::M_MAX;
use crate::control::foc::{FocSpec, LoopSpec};
use crate::control::pid::AntiWindup;
use crate::control::six_step::SixStepSpec;
use crate::engine::commands::{ChangeSource, EngineCommand};
use crate::engine::engine::Engine;
use crate::engine::fidelity::InverterMode;
use crate::engine::fidelity::{FidelityConfig, Tier};
use crate::engine::time::SimTime;
use crate::engine::time::period_from_hz;
use crate::physics::inverter::leg::Devices;
use crate::physics::mech::backlash::{Backlash, DEFAULT_STIFFNESS, DEFAULT_ZETA};
use crate::physics::mech::friction::Friction;
use crate::physics::mech::gearbox::Gearbox;
use crate::physics::mech::load::Load;
use crate::physics::mech::load_arm::Arm;
use crate::physics::motor::backemf::Shape;
use crate::physics::motor::cogging::CoggingParams;
use crate::physics::motor::electrical::SatParams;
use crate::physics::motor::iron_loss::IronLossParams;
use crate::physics::sensors::adc::{AdcConfig, Channel};
use crate::physics::sensors::encoder_incremental::IncrementalConfig;
use crate::physics::sensors::encoder_magnetic::{AirGap, MagneticEncoderConfig};
use crate::physics::sensors::hall::HallConfig;
use crate::physics::sensors::pipeline::PipelineConfig;
use crate::physics::sensors::{EncoderSpec, Sampled, SensorSpec};
use crate::physics::supply::battery::Battery;
use crate::physics::supply::bus::{Chopper, Source};
use crate::physics::thermal::ThermalParams;
use crate::skeleton::adapter::{
    Drive, InverterOptions, SkeletonOptions, ThermalOptions, try_build_engine_with,
};
use crate::skeleton::params::PmsmParams;

#[derive(Debug, thiserror::Error)]
pub enum BuildError {
    #[error("{0}: {1}")]
    Param(String, String),
    #[error("{0}")]
    Unsupported(String),
    #[error("invalid scene:\n{0}")]
    Invalid(String),
}

fn si(p: &Param, kind: Kind, path: &str) -> Result<f64, BuildError> {
    p.value
        .si(kind)
        .map_err(|e| BuildError::Param(path.into(), e.to_string()))
}

fn opt(p: &Option<Param>, kind: Kind, path: &str) -> Result<f64, BuildError> {
    p.as_ref().map_or(Ok(0.0), |p| si(p, kind, path))
}

/// Load + gearbox inertia seen by the motor shaft [kg·m²].
pub fn reflected_inertia(
    gearbox: Option<&GearboxParams>,
    load: Option<&LoadParams>,
) -> Result<f64, BuildError> {
    let n = gearbox.map_or(1.0, |g| g.ratio);
    let j_load = match load {
        Some(LoadParams::Arm {
            arm_length,
            arm_mass,
            mass,
            distance,
            ..
        }) => {
            let l = si(arm_length, Kind::Length, "load.arm_length")?;
            let d = si(distance, Kind::Length, "load.distance")?;
            si(mass, Kind::Mass, "load.mass")? * d * d
                + si(arm_mass, Kind::Mass, "load.arm_mass")? * l * l / 3.0
        }
        Some(LoadParams::Flywheel { inertia }) => si(inertia, Kind::Inertia, "load.inertia")?,
        _ => 0.0,
    };
    let (j_in, j_out) = match gearbox {
        Some(g) => (
            opt(&g.j_in, Kind::Inertia, "gearbox.j_in")?,
            opt(&g.j_out, Kind::Inertia, "gearbox.j_out")?,
        ),
        None => (0.0, 0.0),
    };
    Ok(j_in + (j_out + j_load) / (n * n))
}

/// Skeleton motor parameters from the model (EQ-CONV-09: λ is canonical).
pub fn pmsm_from(m: &MotorParams, extra_j: f64) -> Result<PmsmParams, BuildError> {
    let e = &m.electrical;
    let b = match &m.mechanical.friction {
        Some(f) => si(
            &f.viscous,
            Kind::RotationalDamping,
            "motor.mechanical.friction.viscous",
        )?,
        None => 0.0,
    };
    Ok(PmsmParams {
        p: f64::from(m.winding.pole_pairs),
        r: si(&e.r_phase, Kind::Resistance, "motor.electrical.r_phase")?,
        ld: si(&e.l_d, Kind::Inductance, "motor.electrical.l_d")?,
        lq: si(&e.l_q, Kind::Inductance, "motor.electrical.l_q")?,
        lambda: si(&e.lambda_m, Kind::FluxLinkage, "motor.electrical.lambda_m")?,
        j: si(
            &m.mechanical.j_rotor,
            Kind::Inertia,
            "motor.mechanical.j_rotor",
        )? + extra_j,
        b,
    })
}

/// Bus voltage of the supply at the start [V].
pub fn supply_voltage(s: &SupplyParams) -> Result<f64, BuildError> {
    match s {
        SupplyParams::Ideal { voltage } | SupplyParams::Psu { voltage, .. } => {
            si(voltage, Kind::Voltage, "supply.voltage")
        }
        SupplyParams::Battery {
            series,
            soc_init,
            ocv_cell,
            ..
        } => {
            let ocv = interp(ocv_cell, *soc_init).ok_or_else(|| {
                BuildError::Param("supply.ocv_cell".into(), "empty OCV table".into())
            })?;
            Ok(f64::from(*series) * ocv)
        }
    }
}

fn interp(t: &[(f64, f64)], x: f64) -> Option<f64> {
    let (first, last) = (t.first()?, t.last()?);
    if x <= first.0 {
        return Some(first.1);
    }
    for w in t.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if x <= x1 {
            return Some(y0 + (y1 - y0) * (x - x0) / (x1 - x0));
        }
    }
    Some(last.1)
}

/// The editable model behind a running engine: parameter edits go through the
/// constraint rules first and only then become engine commands.
#[derive(Debug, Clone)]
pub struct SceneModel {
    pub motor: MotorParams,
    /// Reflected gearbox/load inertia added to the rotor [kg·m²].
    pub extra_j: f64,
}

impl SceneModel {
    /// Apply an edit to `motor.*` via `apply_edit`; returns the engine commands to queue.
    pub fn edit(
        &mut self,
        path: &str,
        value: serde_json::Value,
        source: ChangeSource,
    ) -> Result<Vec<EngineCommand>, String> {
        let before = pmsm_from(&self.motor, self.extra_j).map_err(|e| e.to_string())?;
        let r = constraints::apply_edit(&self.motor, path, value);
        if r.rejected {
            let why: Vec<_> = r
                .issues
                .iter()
                .filter(|i| i.severity == Severity::Reject)
                .map(|i| i.message.clone())
                .collect();
            return Err(format!("`{path}` rejected: {}", why.join("; ")));
        }
        let after = pmsm_from(&r.model, self.extra_j).map_err(|e| e.to_string())?;
        if after.p != before.p {
            return Err(format!("`{path}`: changing the pole count needs a reset"));
        }
        // Parts of the motor that are fixed when the engine is built: refuse a live change
        // instead of updating the scene but not the engine.
        let (m0, m1) = (&self.motor, &r.model);
        if m0.magnetic.emf_shape != m1.magnetic.emf_shape
            || m0.magnetic.cogging != m1.magnetic.cogging
            || m0.magnetic.saturation != m1.magnetic.saturation
            || m0.winding != m1.winding
        {
            return Err(format!(
                "`{path}` cannot change while running; edit the scene and reset"
            ));
        }
        let j = |m: &MotorParams| {
            si(
                &m.mechanical.j_rotor,
                Kind::Inertia,
                "motor.mechanical.j_rotor",
            )
        };
        let k = |p: &Option<Param>, path: &str| {
            p.as_ref().map_or(Ok(0.0), |p| {
                p.value
                    .si(Kind::Dimensionless)
                    .map_err(|e| BuildError::Param(path.into(), e.to_string()))
            })
        };
        let num = |r: Result<f64, BuildError>| r.map_err(|e| e.to_string());
        let pairs = [
            ("electrical.r_phase", before.r, after.r),
            ("electrical.l_d", before.ld, after.ld),
            ("electrical.l_q", before.lq, after.lq),
            ("electrical.lambda_m", before.lambda, after.lambda),
            ("mechanical.j_rotor", num(j(m0))?, num(j(m1))?),
            ("mechanical.friction.viscous", before.b, after.b),
            (
                "magnetic.k_hy",
                num(k(&m0.magnetic.k_hy, "motor.magnetic.k_hy"))?,
                num(k(&m1.magnetic.k_hy, "motor.magnetic.k_hy"))?,
            ),
            (
                "magnetic.k_ed",
                num(k(&m0.magnetic.k_ed, "motor.magnetic.k_ed"))?,
                num(k(&m1.magnetic.k_ed, "motor.magnetic.k_ed"))?,
            ),
        ];
        let cmds = pairs
            .iter()
            .filter(|(_, a, b)| a != b)
            .map(|(n, _, v)| EngineCommand::SetParam {
                path: format!("motor.{n}"),
                value: *v,
                source,
            })
            .collect();
        self.motor = r.model;
        Ok(cmds)
    }
}

/// A built engine plus what the scenario runner needs to drive it.
pub struct BuiltScene {
    pub engine: Engine,
    pub model: SceneModel,
    /// Gearbox ratio N (motor speed = N · load speed).
    pub ratio: f64,
    /// Controller period [s].
    pub ctrl_dt: f64,
}

/// The scene's fidelity tier (default Standard, EQ-NUM-07).
pub fn tier(scene: &ResolvedScene) -> Tier {
    match scene.fidelity.as_ref().map(|f| f.tier) {
        Some(TierParam::Ideal) => Tier::Ideal,
        Some(TierParam::Detailed) => Tier::Detailed,
        _ => Tier::Standard,
    }
}

/// Cogging terms (EQ-MOT-08) if the tier enables them and the motor has any.
fn cogging(scene: &ResolvedScene) -> Result<Option<CoggingParams>, BuildError> {
    let m = &scene.motor;
    if !FidelityConfig::preset(tier(scene)).enable_cogging || m.magnetic.cogging.is_empty() {
        return Ok(None);
    }
    let terms = m
        .magnetic
        .cogging
        .iter()
        .enumerate()
        .map(|(k, c)| {
            Ok((
                si(
                    &c.amplitude,
                    Kind::Torque,
                    &format!("motor.magnetic.cogging.{k}.amplitude"),
                )?,
                c.phase,
            ))
        })
        .collect::<Result<Vec<_>, BuildError>>()?;
    let n_c = f64::from(sim_model::winding::cogging_periods(
        m.winding.slots,
        m.winding.pole_pairs,
    ));
    Ok(Some(CoggingParams { n_c, terms }))
}

/// Iron-loss coefficients (EQ-MOT-09) if the tier enables them and the motor has any.
/// No unit kind exists for W·s/rad, so they are bare SI numbers.
fn iron(scene: &ResolvedScene) -> Result<Option<IronLossParams>, BuildError> {
    let m = &scene.motor.magnetic;
    if !FidelityConfig::preset(tier(scene)).enable_iron_loss
        || (m.k_hy.is_none() && m.k_ed.is_none())
    {
        return Ok(None);
    }
    let k = |p: &Option<Param>, path: &str| {
        p.as_ref().map_or(Ok(0.0), |p| {
            p.value
                .si(Kind::Dimensionless)
                .map_err(|e| BuildError::Param(path.into(), e.to_string()))
        })
    };
    Ok(Some(IronLossParams {
        k_hy: k(&m.k_hy, "motor.magnetic.k_hy")?,
        k_ed: k(&m.k_ed, "motor.magnetic.k_ed")?,
        pole_pairs: f64::from(scene.motor.winding.pole_pairs),
    }))
}

/// The scene's gearbox (EQ-MECH-02/03); bypass without one.
fn gearbox(scene: &ResolvedScene) -> Result<Gearbox, BuildError> {
    let Some(g) = &scene.gearbox else {
        return Ok(Gearbox::BYPASS);
    };
    Ok(Gearbox {
        n: g.ratio,
        eta: g.efficiency,
        j_in: opt(&g.j_in, Kind::Inertia, "gearbox.j_in")?,
        j_out: opt(&g.j_out, Kind::Inertia, "gearbox.j_out")?,
    })
}

/// Bus capacitance when the scene gives none [F] (supply.md's example drive, 470 µF).
pub const DEFAULT_C_BUS: f64 = 470e-6;

/// The terminal drive (P07.T12): `fidelity.inverter_mode` (else the tier's) picks the
/// ideal source, or SVPWM → averaged / switching inverter → DC bus with the scene's source
/// and chopper. Also returns its step cap: τ_bus/2 for a PSU or battery (D-018) and, in
/// switching mode, T_pwm/200 (EQ-NUM step table).
fn drive(scene: &ResolvedScene) -> Result<(Drive, f64), BuildError> {
    let mode = match scene
        .fidelity
        .as_ref()
        .and_then(|f| f.inverter_mode.as_deref())
    {
        Some("ideal") => InverterMode::Ideal,
        Some("averaged") => InverterMode::Averaged,
        Some("switching") => InverterMode::Switching,
        Some(other) => {
            return Err(BuildError::Param(
                "fidelity.inverter_mode".into(),
                format!("`{other}`: expected ideal, averaged or switching"),
            ));
        }
        None => FidelityConfig::preset(tier(scene)).inverter_mode,
    };
    if mode == InverterMode::Ideal {
        return Ok((Drive::Ideal, f64::INFINITY));
    }
    let i = &scene.inverter;
    let t = |p: &Param, n: &str| si(p, Kind::Time, &format!("inverter.{n}"));
    let f_pwm = si(&i.f_pwm, Kind::Frequency, "inverter.f_pwm")?;
    let c_bus = match &scene.bus {
        Some(b) => si(&b.capacitance, Kind::Capacitance, "bus.capacitance")?,
        None => DEFAULT_C_BUS,
    };
    let source = match &scene.supply {
        SupplyParams::Ideal { voltage } => Source::Ideal {
            v: si(voltage, Kind::Voltage, "supply.voltage")?,
        },
        SupplyParams::Psu {
            voltage,
            current_limit,
            r_out,
        } => Source::Psu {
            v_set: si(voltage, Kind::Voltage, "supply.voltage")?,
            i_lim: si(current_limit, Kind::Current, "supply.current_limit")?,
            r_out: si(r_out, Kind::Resistance, "supply.r_out")?,
        },
        SupplyParams::Battery {
            series,
            parallel,
            capacity_cell,
            r0_cell,
            r1_cell,
            c1_cell,
            soc_init,
            ocv_cell,
            ..
        } => Source::Battery {
            pack: Battery::from_cells(
                ocv_cell,
                *series,
                *parallel,
                si(r0_cell, Kind::Resistance, "supply.r0_cell")?,
                si(r1_cell, Kind::Resistance, "supply.r1_cell")?,
                si(c1_cell, Kind::Capacitance, "supply.c1_cell")?,
                si(capacity_cell, Kind::Charge, "supply.capacity_cell")? / 3600.0,
            ),
            soc0: *soc_init,
        },
    };
    let tau_bus = match &source {
        Source::Ideal { .. } => f64::INFINITY,
        Source::Psu { r_out, .. } => r_out * c_bus,
        Source::Battery { pack, .. } => pack.r0 * c_bus,
    };
    let chopper = match scene.bus.as_ref().and_then(|b| b.chopper.as_ref()) {
        Some(c) => Some(Chopper {
            r_brake: si(&c.r_brake, Kind::Resistance, "bus.chopper.r_brake")?,
            v_on: si(&c.v_on, Kind::Voltage, "bus.chopper.v_on")?,
            v_off: si(&c.v_off, Kind::Voltage, "bus.chopper.v_off")?,
        }),
        None => None,
    };
    let switching = mode == InverterMode::Switching;
    let mut cap = 0.5 * tau_bus;
    if switching {
        cap = cap.min(1.0 / f_pwm / 200.0);
    }
    // Validation rejects R = 0 and C = 0; a zero cap would stall the engine.
    if cap.is_nan() || cap <= 0.0 {
        return Err(BuildError::Param(
            "bus.capacitance".into(),
            format!("the bus step limit is {cap} s: source resistance and C_bus must be > 0"),
        ));
    }
    let o = InverterOptions {
        switching,
        devices: Devices {
            r_on: si(&i.r_on, Kind::Resistance, "inverter.r_on")?,
            v_f: si(&i.v_f, Kind::Voltage, "inverter.v_f")?,
        },
        f_pwm,
        dead_time: t(&i.dead_time, "dead_time")?,
        t_rise_fall: t(&i.t_rise, "t_rise")? + t(&i.t_fall, "t_fall")?,
        i_eps: match &i.i_eps {
            Some(p) => si(p, Kind::Current, "inverter.i_eps")?,
            None => 0.01,
        },
        source,
        c_bus,
        chopper,
    };
    Ok((Drive::Inverter(o), cap))
}

/// Default ambient temperature when the scene gives none [K] (25 °C).
pub const DEFAULT_AMBIENT: f64 = 298.15;
/// Default reference temperature of R and λ [K] (20 °C).
pub const DEFAULT_T_REF: f64 = 293.15;
/// Default magnet remanence coefficient α_Br [1/K] (EQ-THERM-03).
pub const DEFAULT_ALPHA_BR: f64 = -0.0012;

/// Thermal network (EQ-THERM-01) with R(T)/λ(T) coupling, if the tier enables it and the
/// motor has thermal data.
fn thermal(scene: &ResolvedScene) -> Result<Option<ThermalOptions>, BuildError> {
    let m = &scene.motor;
    let Some(t) = &m.thermal else {
        return Ok(None);
    };
    if !FidelityConfig::preset(tier(scene)).enable_thermal {
        return Ok(None);
    }
    let tr = |p: &Param, n: &str| si(p, Kind::ThermalResistance, &format!("motor.thermal.{n}"));
    let hc = |p: &Param, n: &str| si(p, Kind::HeatCapacity, &format!("motor.thermal.{n}"));
    let t_amb = match &scene.initial.ambient {
        Some(p) => si(p, Kind::Temperature, "initial.ambient")?,
        None => DEFAULT_AMBIENT,
    };
    let t_init = match &scene.initial.temperature {
        Some(p) => si(p, Kind::Temperature, "initial.temperature")?,
        None => t_amb,
    };
    // κ [s/rad] has no unit kind: a bare SI number.
    let kappa = match &t.kappa {
        Some(p) => p
            .value
            .si(Kind::Dimensionless)
            .map_err(|e| BuildError::Param("motor.thermal.kappa".into(), e.to_string()))?,
        None => 0.0,
    };
    let params = ThermalParams {
        c_w: hc(&t.c_w, "c_w")?,
        c_s: hc(&t.c_s, "c_s")?,
        c_h: hc(&t.c_h, "c_h")?,
        r_ws: tr(&t.r_ws, "r_ws")?,
        r_sh: tr(&t.r_sh, "r_sh")?,
        r_ha: tr(&t.r_ha, "r_ha")?,
        t_amb,
        t_init,
        t_max: si(&t.t_max, Kind::Temperature, "motor.thermal.t_max")?,
        kappa,
        outrunner: m.identity.topology == Topology::Outrunner,
    };
    Ok(Some(ThermalOptions {
        params,
        t_ref: match &t.t_ref {
            Some(p) => si(p, Kind::Temperature, "motor.thermal.t_ref")?,
            None => DEFAULT_T_REF,
        },
        alpha_br: match &m.magnetic.alpha_br {
            Some(p) => si(p, Kind::TemperatureCoefficient, "motor.magnetic.alpha_br")?,
            None => DEFAULT_ALPHA_BR,
        },
    }))
}

/// Two-inertia backlash (EQ-MECH-04) when the gearbox has backlash > 0, and the step
/// limit its contact needs (EQ-NUM: dt ≤ 1/(20·√(k_c/J_red))). Missing stiffness
/// defaults to 1 N·m/arcmin and missing damping to ζ = 0.1 (D-016).
fn backlash(scene: &ResolvedScene, j_m: f64) -> Result<(Option<Backlash>, f64), BuildError> {
    let Some(g) = &scene.gearbox else {
        return Ok((None, f64::INFINITY));
    };
    let width = opt(&g.backlash, Kind::Angle, "gearbox.backlash")?;
    if width <= 0.0 {
        return Ok((None, f64::INFINITY));
    }
    let gb = gearbox(scene)?;
    let (load, _) = load(scene)?;
    // Reduced inertia of the contact, both sides seen from the output.
    let j_ms = gb.n * gb.n * (j_m + gb.j_in);
    let j_ls = gb.j_out + load.inertia();
    if j_ls <= 0.0 {
        // No inertia behind the teeth (no `j_out`, a massless load): the load side has no
        // dynamics of its own, so backlash is only lost motion with nothing to move. Rigid.
        return Ok((None, f64::INFINITY));
    }
    let j_red = j_ms * j_ls / (j_ms + j_ls);
    let k = match &g.stiffness {
        Some(p) => si(p, Kind::TorsionalStiffness, "gearbox.stiffness")?,
        None => DEFAULT_STIFFNESS,
    };
    let d = match &g.damping {
        Some(p) => si(p, Kind::RotationalDamping, "gearbox.damping")?,
        None => Backlash::damping_for(k, DEFAULT_ZETA, j_red),
    };
    let b = Backlash { width, k, d };
    Ok((Some(b), b.dt_limit(j_red)))
}

/// Stick–slip friction (EQ-MECH-05): the motor's static/Coulomb part (its viscous part
/// is `PmsmParams::b`) and the arm's load-side friction.
fn friction(scene: &ResolvedScene) -> Result<(Friction, Friction), BuildError> {
    let conv = |f: &sim_model::params::Friction, at: &str| -> Result<Friction, BuildError> {
        Ok(Friction {
            t_s: si(
                &f.static_torque,
                Kind::Torque,
                &format!("{at}.static_torque"),
            )?,
            t_c: si(&f.coulomb, Kind::Torque, &format!("{at}.coulomb"))?,
            b: si(
                &f.viscous,
                Kind::RotationalDamping,
                &format!("{at}.viscous"),
            )?,
        })
    };
    let motor = match &scene.motor.mechanical.friction {
        Some(f) => Friction {
            b: 0.0,
            ..conv(f, "motor.mechanical.friction")?
        },
        None => Friction::default(),
    };
    let load = match &scene.load {
        Some(LoadParams::Arm {
            friction: Some(f), ..
        }) => conv(f, "load.friction")?,
        _ => Friction::default(),
    };
    Ok((motor, load))
}

/// Initial load-side angle and speed from `initial.theta` / `initial.omega` (the arm's
/// mounting offset, EQ-MECH-06: θ_L = 0 is straight down).
fn initial_load(scene: &ResolvedScene) -> Result<(f64, f64), BuildError> {
    let i = &scene.initial;
    Ok((
        opt(&i.theta, Kind::Angle, "initial.theta")?,
        opt(&i.omega, Kind::AngularVelocity, "initial.omega")?,
    ))
}

/// The scene's load and its EQ-MECH-09 rule (true: conserve momentum).
fn load(scene: &ResolvedScene) -> Result<(Load, bool), BuildError> {
    Ok(match &scene.load {
        None => (Load::None, false),
        Some(LoadParams::Flywheel { inertia }) => (
            Load::Flywheel {
                j: si(inertia, Kind::Inertia, "load.inertia")?,
            },
            false,
        ),
        Some(LoadParams::Arm {
            arm_length,
            arm_mass,
            mass,
            distance,
            on_change,
            ..
        }) => (
            Load::Arm(Arm {
                arm_length: si(arm_length, Kind::Length, "load.arm_length")?,
                arm_mass: si(arm_mass, Kind::Mass, "load.arm_mass")?,
                mass: si(mass, Kind::Mass, "load.mass")?,
                distance: si(distance, Kind::Length, "load.distance")?,
            }),
            *on_change == OnChange::ConserveMomentum,
        ),
        Some(LoadParams::Constant { torque }) => (
            Load::Constant {
                t0: si(torque, Kind::Torque, "load.torque")?,
            },
            false,
        ),
        Some(LoadParams::Brake { torque }) => (
            Load::Brake {
                t0: si(torque, Kind::Torque, "load.torque")?,
            },
            false,
        ),
        Some(LoadParams::Viscous { coefficient }) => (
            Load::Viscous {
                c: si(coefficient, Kind::RotationalDamping, "load.coefficient")?,
            },
            false,
        ),
    })
}

/// Back-EMF shape (EQ-MOT-03). Not tier-gated: the shape is part of the motor.
fn shape(scene: &ResolvedScene) -> Result<Shape, BuildError> {
    Ok(match &scene.motor.magnetic.emf_shape {
        EmfShape::Sinusoidal => Shape::Sinusoidal,
        EmfShape::Trapezoidal { flat_top } => Shape::Trapezoidal {
            w: si(flat_top, Kind::Angle, "motor.magnetic.emf_shape.flat_top")?,
        },
        EmfShape::Harmonics { terms } => {
            Shape::Harmonics(terms.iter().map(|h| (h.n, h.b, h.phase)).collect())
        }
    })
}

/// Saturation curve (EQ-MOT-10) if the tier enables it and the motor has one.
fn saturation(scene: &ResolvedScene) -> Result<Option<SatParams>, BuildError> {
    let Some(s) = &scene.motor.magnetic.saturation else {
        return Ok(None);
    };
    if !FidelityConfig::preset(tier(scene)).enable_saturation {
        return Ok(None);
    }
    Ok(Some(SatParams {
        i_k: si(&s.i_knee, Kind::Current, "motor.magnetic.saturation.i_knee")?,
        l_inf: si(
            &s.l_inf,
            Kind::Inductance,
            "motor.magnetic.saturation.l_inf",
        )?,
        c: s.cross,
    }))
}

pub fn build_engine(scene: &ResolvedScene) -> Result<BuiltScene, BuildError> {
    // Derive (so motor-constant overrides reach λ) and refuse anything with a Reject.
    let mut scene = scene.clone();
    let rejects: Vec<String> = scene
        .check()
        .into_iter()
        .filter(|i| i.severity == Severity::Reject)
        .map(|i| format!("  {}: {}", i.path, i.message))
        .collect();
    if !rejects.is_empty() {
        return Err(BuildError::Invalid(rejects.join("\n")));
    }
    let scene = &scene;
    let (mut controller, rate) =
        controller_spec(&scene.controller, supply_voltage(&scene.supply)?)?;
    let extra_j = reflected_inertia(scene.gearbox.as_ref(), scene.load.as_ref())?;
    let mp = pmsm_from(&scene.motor, 0.0)?;
    let (load, conserve_momentum) = load(scene)?;
    let (backlash, dt_contact) = backlash(scene, mp.j)?;
    let (theta_l0, omega_l0) = initial_load(scene)?;
    let (drive, dt_drive) = drive(scene)?;
    let dt_max = match scene.fidelity.as_ref().and_then(|f| f.dt_max.as_ref()) {
        Some(p) => si(p, Kind::Time, "fidelity.dt_max")?,
        None => 0.1 / rate,
    }
    .min(dt_contact)
    .min(dt_drive);
    if let (ControllerSpec::SixStep(s), Drive::Inverter(o)) = (&mut controller, &drive) {
        (s.r_on, s.v_f) = (o.devices.r_on, o.devices.v_f);
    }
    let drive_kind = drive.clone();
    let engine = try_build_engine_with(
        mp,
        SkeletonOptions {
            locked: false,
            controller,
            v_supply: supply_voltage(&scene.supply)?,
            dt_max,
            gearbox: gearbox(scene)?,
            load,
            conserve_momentum,
            theta_l0,
            omega_l0,
            friction: friction(scene)?,
            backlash,
            thermal: thermal(scene)?,
            drive,
            ratio: scene.gearbox.as_ref().map_or(1.0, |g| g.ratio),
            cogging: cogging(scene)?,
            iron: iron(scene)?,
            saturation: saturation(scene)?,
            shape: shape(scene)?,
            sensors: sensors(scene, rate, &drive_kind)?,
        },
    )
    .map_err(BuildError::Invalid)?;
    Ok(BuiltScene {
        engine,
        model: SceneModel {
            motor: scene.motor.clone(),
            extra_j,
        },
        ratio: scene.gearbox.as_ref().map_or(1.0, |g| g.ratio),
        ctrl_dt: 1.0 / rate,
    })
}

fn loop_spec(l: &LoopParams, at: &str) -> Result<LoopSpec, BuildError> {
    Ok(LoopSpec {
        enabled: l.enabled,
        rate_hz: si(&l.rate, Kind::Frequency, &format!("{at}.rate"))?,
        omega: match &l.bandwidth {
            Some(b) => Some(
                2.0 * std::f64::consts::PI * si(b, Kind::Frequency, &format!("{at}.bandwidth"))?,
            ),
            None => None,
        },
        kp: l.kp,
        ki: l.ki,
    })
}

/// The controller family from the scene (EQ-CTRL-03…05) and its fastest rate [Hz]
/// (the sensors' default sampling). Other families arrive with their tasks.
fn controller_spec(c: &ControllerParams, v_bus: f64) -> Result<(ControllerSpec, f64), BuildError> {
    if let ControllerParams::SixStep {
        rate,
        speed_loop,
        advance,
        limits,
    } = c
    {
        let rate = si(rate, Kind::Frequency, "controller.rate")?;
        let spec = SixStepSpec {
            rate_hz: rate,
            advance: opt(advance, Kind::Angle, "controller.advance")?,
            speed_loop: speed_loop
                .as_ref()
                .map(|l| loop_spec(l, "controller.speed_loop"))
                .transpose()?,
            speed_limit: match &limits.speed {
                Some(p) => si(p, Kind::AngularVelocity, "controller.limits.speed")?,
                None => f64::INFINITY,
            },
            v_nominal: v_bus,
            i_max: si(&limits.current, Kind::Current, "controller.limits.current")?,
            // Filled from the inverter by `build_engine` (it knows the devices).
            r_on: 0.0,
            v_f: 0.0,
        };
        return Ok((ControllerSpec::SixStep(spec), rate));
    }
    let ControllerParams::Foc {
        current,
        velocity,
        position,
        sensorless,
        prefilter,
        limits,
    } = c
    else {
        let err = crate::control::framework::build_family(
            c,
            &crate::control::framework::PlantInfo {
                ratio: 1.0,
                kt: 1.0,
                j_motor: 1.0,
            },
            &mut crate::engine::signals::SignalBus::new(),
        )
        .err()
        .unwrap_or_default();
        return Err(BuildError::Unsupported(err));
    };
    if *sensorless {
        return Err(BuildError::Unsupported(
            "sensorless FOC arrives in P09.T08".into(),
        ));
    }
    let cur = loop_spec(current, "controller.current")?;
    let opt_speed = |p: &Option<Param>, k: Kind, at: &str| match p {
        Some(p) => si(p, k, at),
        None => Ok(f64::INFINITY),
    };
    let spec = FocSpec {
        current_rate_hz: cur.rate_hz,
        omega_c: cur.omega,
        m_max: limits.voltage_fraction.unwrap_or(M_MAX),
        aw: AntiWindup::Clamping,
        i_max: si(&limits.current, Kind::Current, "controller.limits.current")?,
        speed_limit: opt_speed(
            &limits.speed,
            Kind::AngularVelocity,
            "controller.limits.speed",
        )?,
        ramp: opt_speed(
            &limits.ramp,
            Kind::AngularAcceleration,
            "controller.limits.ramp",
        )?,
        velocity: velocity
            .as_ref()
            .map(|l| loop_spec(l, "controller.velocity"))
            .transpose()?,
        position: position
            .as_ref()
            .map(|l| loop_spec(l, "controller.position"))
            .transpose()?,
        prefilter: *prefilter,
        open_loop_vq: None,
    };
    Ok((ControllerSpec::Foc(spec), cur.rate_hz))
}

/// A rate [Hz] as an exact-ns period.
fn period_of(hz: f64, path: &str) -> Result<SimTime, BuildError> {
    period_from_hz(hz)
        .map(|p| p.period)
        .map_err(|e| BuildError::Param(path.into(), e.to_string()))
}

/// Latency, noise and bypass of the common pipeline (no quantisation, wrap or range).
fn plain_pipe(c: &SensorCommon, noise: Kind, at: &str) -> Result<PipelineConfig, BuildError> {
    Ok(PipelineConfig {
        bypass: c.bypass,
        latency_ns: (opt(&c.latency, Kind::Time, &format!("{at}.latency"))? * 1e9).round() as i64,
        noise_sigma: opt(&c.noise, noise, &format!("{at}.noise"))?,
        ..Default::default()
    })
}

/// The scene's sensors (EQ-SENS-*, P08.T11). Sampling: `update_rate` when given;
/// otherwise the ADC at `pwm.center` (T_pwm, phase 0) in switching mode, the magnetic
/// encoder at `DEFAULT_ENCODER_RATE` and the rest at the controller rate `ctrl_hz`.
/// A bypassed sensor samples at exactly the controller period, so every controller
/// step reads the true value (EQ-SENS-01).
fn sensors(scene: &ResolvedScene, ctrl_hz: f64, drive: &Drive) -> Result<SensorSpec, BuildError> {
    let mut spec = SensorSpec {
        seed: scene.seed,
        ..Default::default()
    };
    let Some(s) = &scene.sensors else {
        return Ok(spec);
    };
    // The Ideal tier makes every sensor ideal (EQ-NUM-07).
    let ideal = !FidelityConfig::preset(tier(scene)).enable_sensor_nonideal;
    let bypassed = |c: &SensorCommon| SensorCommon {
        bypass: c.bypass || ideal,
        ..c.clone()
    };
    let ctrl = period_of(ctrl_hz, "controller.current.rate")?;
    let zero = SimTime::from_nanos(0);
    // (period, phase) for a sensor with `default` when no rate is given.
    let sampling = |c: &SensorCommon, default: (SimTime, SimTime), at: &str| {
        if c.bypass {
            return Ok((ctrl, zero));
        }
        match &c.update_rate {
            Some(r) => {
                let hz = si(r, Kind::Frequency, &format!("{at}.update_rate"))?;
                Ok((period_of(hz, &format!("{at}.update_rate"))?, zero))
            }
            None => Ok(default),
        }
    };
    if let Some(h) = &s.hall {
        let mut offsets = [0.0; 3];
        for (k, o) in h.offsets.iter().enumerate().take(3) {
            offsets[k] = si(o, Kind::Angle, &format!("sensors.hall.offsets.{k}"))?;
        }
        let cfg = HallConfig {
            offsets,
            hysteresis: h.hysteresis.unwrap_or(0.0),
        };
        let c = bypassed(&h.common);
        let pipe = plain_pipe(&c, Kind::Angle, "sensors.hall")?;
        let (period, phase) = sampling(&c, (ctrl, zero), "sensors.hall")?;
        spec.hall = Some(Sampled {
            cfg: (cfg, pipe),
            period,
            phase,
        });
    }
    match &s.encoder {
        Some(p @ EncoderParams::Magnetic { common, .. }) => {
            let (mut cfg, default) = magnetic_encoder(p)?
                .ok_or_else(|| BuildError::Unsupported("magnetic encoder".into()))?;
            let c = bypassed(common);
            cfg.bypass = c.bypass;
            let (period, phase) = sampling(&c, (default, zero), "sensors.encoder")?;
            spec.encoder = Some(Sampled {
                cfg: EncoderSpec::Magnetic(cfg),
                period,
                phase,
            });
        }
        Some(EncoderParams::Incremental {
            cpr,
            index_angle,
            common,
        }) => {
            let cfg = IncrementalConfig {
                cpr: *cpr,
                index_angle: opt(index_angle, Kind::Angle, "sensors.encoder.index_angle")?,
            };
            let c = bypassed(common);
            let pipe = plain_pipe(&c, Kind::Angle, "sensors.encoder")?;
            let (period, phase) = sampling(&c, (ctrl, zero), "sensors.encoder")?;
            spec.encoder = Some(Sampled {
                cfg: EncoderSpec::Incremental(cfg, pipe),
                period,
                phase,
            });
        }
        None => {}
    }
    if let Some(a) = &s.adc {
        let n = |x: &str| format!("sensors.adc.{x}");
        let fs = si(&a.full_scale, Kind::Current, &n("full_scale"))?;
        let off = opt(&a.offset, Kind::Current, &n("offset"))?;
        let mut phase = [Channel {
            full_scale: fs,
            gain_error: a.gain_error,
            offset: off,
        }; 3];
        for (k, g) in a.gain_errors.iter().enumerate().take(3) {
            phase[k].gain_error = *g;
        }
        for (k, o) in a.offsets.iter().enumerate().take(3) {
            phase[k].offset = si(o, Kind::Current, &n(&format!("offsets.{k}")))?;
        }
        let (bus, bus_noise) = match &a.bus {
            Some(b) => {
                if matches!(drive, Drive::Ideal) {
                    return Err(BuildError::Param(
                        n("bus"),
                        "a bus-voltage channel needs an inverter and DC bus (fidelity.inverter_mode averaged or switching)".into(),
                    ));
                }
                let ch = Channel {
                    full_scale: si(&b.full_scale, Kind::Voltage, &n("bus.full_scale"))?,
                    gain_error: b.gain_error,
                    offset: opt(&b.offset, Kind::Voltage, &n("bus.offset"))?,
                };
                (Some(ch), opt(&b.noise, Kind::Voltage, &n("bus.noise"))?)
            }
            None => (None, 0.0),
        };
        let c = bypassed(&a.common);
        let pipe = plain_pipe(&c, Kind::Current, "sensors.adc")?;
        let cfg = AdcConfig {
            bits: a.bits,
            phase,
            shunts: a.shunts,
            bus,
            noise: pipe.noise_sigma,
            bus_noise,
            latency_ns: pipe.latency_ns,
            bypass: pipe.bypass,
        };
        // Switching: sample at pwm.center, the start of each PWM period (EQ-INV-02).
        let default = match drive {
            Drive::Inverter(o) if o.switching => {
                (SimTime::from_nanos((1e9 / o.f_pwm).round() as i64), zero)
            }
            _ => (ctrl, zero),
        };
        let (period, phase) = sampling(&c, default, "sensors.adc")?;
        spec.adc = Some(Sampled { cfg, period, phase });
    }
    Ok(spec)
}

/// Encoder sample rate when the preset gives none [Hz] (a typical SPI/SSI polling rate).
pub const DEFAULT_ENCODER_RATE: f64 = 10e3;

/// A magnetic encoder's model and sample period from its parameters (EQ-SENS-03/04);
/// `None` for other encoder kinds. The air-gap model needs all of `air_gap`, `gap_min`
/// and `gap_max`.
pub fn magnetic_encoder(
    p: &EncoderParams,
) -> Result<Option<(MagneticEncoderConfig, SimTime)>, BuildError> {
    let EncoderParams::Magnetic {
        bits,
        inl,
        eccentricity,
        air_gap,
        gap_min,
        gap_max,
        mount_offset,
        common,
        ..
    } = p
    else {
        return Ok(None);
    };
    let n = |s: &str| format!("sensors.encoder.{s}");
    let gap = match (air_gap, gap_min, gap_max) {
        (Some(z), Some(lo), Some(hi)) => Some(AirGap {
            z: si(z, Kind::Length, &n("air_gap"))?,
            z_min: si(lo, Kind::Length, &n("gap_min"))?,
            z_max: si(hi, Kind::Length, &n("gap_max"))?,
        }),
        _ => None,
    };
    let rate = match &common.update_rate {
        Some(r) => si(r, Kind::Frequency, &n("update_rate"))?,
        None => DEFAULT_ENCODER_RATE,
    };
    let cfg = MagneticEncoderConfig {
        bits: *bits,
        inl: opt(inl, Kind::Angle, &n("inl"))?,
        displacement: opt(eccentricity, Kind::Length, &n("eccentricity"))?,
        mount_offset: opt(mount_offset, Kind::Angle, &n("mount_offset"))?,
        noise: opt(&common.noise, Kind::Angle, &n("noise"))?,
        air_gap: gap,
        latency_ns: (opt(&common.latency, Kind::Time, &n("latency"))? * 1e9).round() as i64,
        bypass: common.bypass,
    };
    Ok(Some((cfg, SimTime::from_secs_f64(1.0 / rate))))
}
