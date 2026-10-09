//! Engine from a scene (P04.T13). Until the full physics lands (P05+), a scene is mapped
//! onto the skeleton dq motor + FOC: motor R, L, λ, p and the rotor inertia plus the
//! reflected gearbox/load inertia; bus voltage from the supply; FOC current limit, current
//! bandwidth and rate from the controller. Inverter, sensors and gravity are not modelled yet.
//! Parameters arrive as unit strings and are converted to SI f64 here (D-003).

use sim_model::constraints::{self, Severity};
use sim_model::param::Param;
use sim_model::params::{
    ControllerParams, GearboxParams, LoadParams, MotorParams, SupplyParams, TierParam,
};
use sim_model::scene::ResolvedScene;
use sim_model::units::Kind;

use crate::engine::commands::{ChangeSource, EngineCommand};
use crate::engine::engine::Engine;
use crate::engine::fidelity::{FidelityConfig, Tier};
use crate::fixtures::dq_pmsm::PmsmParams;
use crate::physics::motor::cogging::CoggingParams;
use crate::physics::motor::electrical::SatParams;
use crate::physics::motor::iron_loss::IronLossParams;
use crate::skeleton::adapter::{SkeletonOptions, build_engine_with};
use crate::skeleton::foc::FocConfig;

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
            return Err(format!("`{path}`: changing the pole count needs a rebuild"));
        }
        self.motor = r.model;
        let pairs = [
            ("electrical.r_phase", before.r, after.r),
            ("electrical.l_d", before.ld, after.ld),
            ("electrical.l_q", before.lq, after.lq),
            ("electrical.lambda_m", before.lambda, after.lambda),
            (
                "mechanical.j_rotor",
                before.j - self.extra_j,
                after.j - self.extra_j,
            ),
            ("mechanical.friction.viscous", before.b, after.b),
        ];
        Ok(pairs
            .iter()
            .filter(|(_, a, b)| a != b)
            .map(|(n, _, v)| EngineCommand::SetParam {
                path: format!("motor.{n}"),
                value: *v,
                source: ChangeSource::Scenario,
            })
            .collect())
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
    let ControllerParams::Foc {
        current, limits, ..
    } = &scene.controller
    else {
        return Err(BuildError::Unsupported(
            "only the `foc` controller family can be simulated before P09 (six-step, open-loop, MIT, custom come then)".into(),
        ));
    };
    let rate = si(&current.rate, Kind::Frequency, "controller.current.rate")?;
    let bw = match &current.bandwidth {
        Some(b) => si(b, Kind::Frequency, "controller.current.bandwidth")?,
        None => 1000.0,
    };
    let cfg = FocConfig {
        v_bus: supply_voltage(&scene.supply)?,
        i_max: si(&limits.current, Kind::Current, "controller.limits.current")?,
        omega_c: 2.0 * std::f64::consts::PI * bw,
        dt: 1.0 / rate,
    };
    let extra_j = reflected_inertia(scene.gearbox.as_ref(), scene.load.as_ref())?;
    let mp = pmsm_from(&scene.motor, 0.0)?;
    let dt_max = match scene.fidelity.as_ref().and_then(|f| f.dt_max.as_ref()) {
        Some(p) => si(p, Kind::Time, "fidelity.dt_max")?,
        None => cfg.dt / 10.0,
    };
    let engine = build_engine_with(
        mp,
        cfg,
        SkeletonOptions {
            locked: false,
            open_loop_vq: None,
            dt_max,
            extra_j,
            ratio: scene.gearbox.as_ref().map_or(1.0, |g| g.ratio),
            cogging: cogging(scene)?,
            iron: iron(scene)?,
            saturation: saturation(scene)?,
        },
    );
    Ok(BuiltScene {
        engine,
        model: SceneModel {
            motor: scene.motor.clone(),
            extra_j,
        },
        ratio: scene.gearbox.as_ref().map_or(1.0, |g| g.ratio),
        ctrl_dt: cfg.dt,
    })
}
