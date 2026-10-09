//! Parameter types for every component (POLISHED_IDEA §3–5). Field names follow the
//! shared ID namespace (CONVENTIONS §3; `docs/docs/physics/signals.md`): the YAML path
//! `motor.electrical.r_phase` is the field `MotorParams.electrical.r_phase`.
//! Quantities are [`Param`]s (value + provenance); their SI kind is fixed per field and
//! checked by `validate` (P04.T05). JSON Schemas are added in P04.T07.

// Config types are loaded once, not in a hot path: enum size differences don't matter.
#![allow(clippy::large_enum_variant)]

use serde::{Deserialize, Serialize};

use crate::param::Param;

// ------------------------------------------------------------------ motor

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Topology {
    Inrunner,
    Outrunner,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Connection {
    Star,
    Delta,
}

/// Which motor constant the user entered (the others are derived, EQ-CONV-13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConstantForm {
    Kv,
    Kt,
    Ke,
    LambdaM,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorIdentity {
    pub name: String,
    /// Size class `DDHH` (stator diameter × stack height, mm), e.g. "8010".
    pub size_class: Option<String>,
    pub topology: Topology,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorElectrical {
    /// Phase resistance, star-equivalent [Ω] at `thermal.t_ref`.
    pub r_phase: Param,
    /// d-axis inductance [H] (= l_q for non-salient motors).
    pub l_d: Param,
    /// q-axis inductance [H].
    pub l_q: Param,
    /// Magnet flux linkage, peak per phase [Wb] — canonical constant (EQ-CONV-09…11).
    pub lambda_m: Param,
    /// Which constant was entered; the others are derived and locked.
    pub entered_as: ConstantForm,
    /// Velocity constant (LL peak) [rpm/V] — derived unless entered.
    pub kv: Option<Param>,
    /// Torque constant (per peak phase A) [N·m/A] — derived unless entered.
    pub kt: Option<Param>,
    /// Back-EMF constant (LL peak) [V·s/rad] — derived unless entered.
    pub ke: Option<Param>,
    pub connection: Connection,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EmfShape {
    Sinusoidal,
    /// Flat-top width per half period [rad or "120 deg"] (EQ-MOT-03).
    Trapezoidal {
        flat_top: Param,
    },
    /// Harmonics n (odd) with amplitude b_n relative to the fundamental and phase φ_n.
    Harmonics {
        terms: Vec<Harmonic>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Harmonic {
    pub n: u32,
    pub b: f64,
    #[serde(default)]
    pub phase: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Saturation {
    /// Knee current [A] (EQ-MOT-10).
    pub i_knee: Param,
    /// Incremental q inductance deep in saturation [H].
    pub l_inf: Param,
    /// Cross-saturation fraction c (0 ≤ c < 1).
    pub cross: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorMagnetic {
    pub emf_shape: EmfShape,
    /// Cogging harmonics: amplitude [N·m] and phase per harmonic of N_c (EQ-MOT-08).
    #[serde(default)]
    pub cogging: Vec<CoggingTerm>,
    /// Iron loss coefficients k_hy [W·s/rad], k_ed [W·s²/rad²] (EQ-MOT-09).
    pub k_hy: Option<Param>,
    pub k_ed: Option<Param>,
    pub saturation: Option<Saturation>,
    /// Magnet remanence temperature coefficient [1/K] (default −0.0012).
    pub alpha_br: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CoggingTerm {
    pub amplitude: Param,
    #[serde(default)]
    pub phase: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorWinding {
    pub slots: u32,
    pub pole_pairs: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorGeometry {
    pub stator_od: Option<Param>,
    pub stator_id: Option<Param>,
    pub stack_length: Option<Param>,
    pub air_gap: Option<Param>,
    pub magnet_thickness: Option<Param>,
    /// Magnet arc as a fraction of the pole pitch (0…1).
    pub magnet_arc: Option<f64>,
    pub rotor_od: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Friction {
    pub static_torque: Param,
    pub coulomb: Param,
    pub viscous: Param,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorMechanical {
    pub j_rotor: Param,
    pub mass: Option<Param>,
    pub friction: Option<Friction>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorThermal {
    pub r_ws: Param,
    pub r_sh: Param,
    pub r_ha: Param,
    pub c_w: Param,
    pub c_s: Param,
    pub c_h: Param,
    pub t_max: Param,
    pub t_ref: Option<Param>,
    /// Optional fan-cooling factor κ [s/rad] (EQ-THERM-04).
    pub kappa: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorRatings {
    pub voltage: Option<Param>,
    pub current_continuous: Option<Param>,
    pub current_peak: Option<Param>,
    pub speed: Option<Param>,
    pub torque_peak: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorParams {
    pub schema_version: u32,
    pub identity: MotorIdentity,
    pub electrical: MotorElectrical,
    pub magnetic: MotorMagnetic,
    pub winding: MotorWinding,
    #[serde(default = "empty_geometry")]
    pub geometry: MotorGeometry,
    pub mechanical: MotorMechanical,
    pub thermal: Option<MotorThermal>,
    #[serde(default = "empty_ratings")]
    pub ratings: MotorRatings,
}

fn empty_geometry() -> MotorGeometry {
    MotorGeometry {
        stator_od: None,
        stator_id: None,
        stack_length: None,
        air_gap: None,
        magnet_thickness: None,
        magnet_arc: None,
        rotor_od: None,
    }
}
fn empty_ratings() -> MotorRatings {
    MotorRatings {
        voltage: None,
        current_continuous: None,
        current_peak: None,
        speed: None,
        torque_peak: None,
    }
}

// ------------------------------------------------------------------ drivetrain & loads

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GearboxParams {
    pub name: String,
    pub ratio: f64,
    pub efficiency: f64,
    pub backlash: Option<Param>,
    pub stiffness: Option<Param>,
    pub damping: Option<Param>,
    pub j_in: Option<Param>,
    pub j_out: Option<Param>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OnChange {
    #[default]
    KeepSpeed,
    ConserveMomentum,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LoadParams {
    /// Arm + point mass under gravity (EQ-MECH-06).
    Arm {
        arm_length: Param,
        arm_mass: Param,
        mass: Param,
        distance: Param,
        #[serde(default)]
        on_change: OnChange,
        friction: Option<Friction>,
    },
    Constant {
        torque: Param,
    },
    Brake {
        torque: Param,
    },
    Viscous {
        coefficient: Param,
    },
    Flywheel {
        inertia: Param,
    },
}

// ------------------------------------------------------------------ power

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InverterParams {
    pub name: String,
    pub f_pwm: Param,
    pub dead_time: Param,
    pub r_on: Param,
    pub v_f: Param,
    pub t_rise: Param,
    pub t_fall: Param,
    pub i_eps: Option<Param>,
    pub v_max: Option<Param>,
    pub i_max: Option<Param>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Chemistry {
    LiIon,
    Lipo,
    Lifepo4,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum SupplyParams {
    Ideal {
        voltage: Param,
    },
    Psu {
        voltage: Param,
        current_limit: Param,
        r_out: Param,
    },
    Battery {
        chemistry: Chemistry,
        series: u32,
        parallel: u32,
        capacity_cell: Param,
        r0_cell: Param,
        r1_cell: Param,
        c1_cell: Param,
        soc_init: f64,
        /// Per-cell OCV table: (SoC, volts) pairs, ascending SoC.
        ocv_cell: Vec<(f64, f64)>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusParams {
    pub capacitance: Param,
    pub chopper: Option<ChopperParams>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChopperParams {
    pub r_brake: Param,
    pub v_on: Param,
    pub v_off: Param,
    pub power_rating: Option<Param>,
}

// ------------------------------------------------------------------ sensors

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorCommon {
    #[serde(default)]
    pub bypass: bool,
    pub update_rate: Option<Param>,
    pub latency: Option<Param>,
    pub noise: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EncoderParams {
    Magnetic {
        chip: Option<String>,
        bits: u32,
        inl: Option<Param>,
        eccentricity: Option<Param>,
        air_gap: Option<Param>,
        gap_min: Option<Param>,
        gap_max: Option<Param>,
        mount_offset: Option<Param>,
        #[serde(flatten)]
        common: SensorCommon,
    },
    Incremental {
        cpr: u32,
        index_angle: Option<Param>,
        #[serde(flatten)]
        common: SensorCommon,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HallParams {
    /// Per-sensor placement errors [rad electrical].
    #[serde(default)]
    pub offsets: Vec<Param>,
    pub hysteresis: Option<f64>,
    #[serde(flatten)]
    pub common: SensorCommon,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AdcParams {
    pub bits: u32,
    pub full_scale: Param,
    #[serde(default)]
    pub gain_error: f64,
    pub offset: Option<Param>,
    /// 2 or 3 shunts.
    pub shunts: u32,
    #[serde(flatten)]
    pub common: SensorCommon,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorsParams {
    pub hall: Option<HallParams>,
    pub encoder: Option<EncoderParams>,
    pub adc: Option<AdcParams>,
    pub observer_gamma: Option<Param>,
}

// ------------------------------------------------------------------ control

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoopParams {
    #[serde(default = "yes")]
    pub enabled: bool,
    pub rate: Param,
    /// Target bandwidth [rad/s] (gains derived, EQ-CTRL-03/04) or explicit gains.
    pub bandwidth: Option<Param>,
    pub kp: Option<f64>,
    pub ki: Option<f64>,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Limits {
    pub current: Param,
    pub speed: Option<Param>,
    pub voltage_fraction: Option<f64>,
    pub ramp: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "family", rename_all = "snake_case", deny_unknown_fields)]
pub enum ControllerParams {
    SixStep {
        rate: Param,
        speed_loop: Option<LoopParams>,
        advance: Option<Param>,
        limits: Limits,
    },
    OpenLoop {
        rate: Param,
        v0: Param,
        k_vf: Param,
        accel: Param,
        limits: Limits,
    },
    Foc {
        current: LoopParams,
        velocity: Option<LoopParams>,
        position: Option<LoopParams>,
        #[serde(default)]
        sensorless: bool,
        #[serde(default)]
        prefilter: bool,
        limits: Limits,
    },
    Mit {
        current: LoopParams,
        rate: Param,
        kp: Param,
        kd: Param,
        tau_ff: Option<Param>,
        limits: Limits,
    },
    /// User Luau script (P09.T10).
    Custom {
        script: String,
        rate: Param,
        inputs: Vec<String>,
        outputs: Vec<String>,
        limits: Limits,
    },
}

// ------------------------------------------------------------------ faults, protection, fidelity

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProtectionParams {
    pub over_current: Option<Threshold>,
    pub over_voltage: Option<Threshold>,
    pub under_voltage: Option<Threshold>,
    pub over_temperature: Option<Threshold>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProtectAction {
    #[default]
    DisablePwm,
    Brake,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Threshold {
    #[serde(default = "yes")]
    pub enabled: bool,
    pub level: Param,
    pub filter: Option<Param>,
    #[serde(default)]
    pub action: ProtectAction,
    #[serde(default)]
    pub latch: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TierParam {
    Ideal,
    #[default]
    Standard,
    Detailed,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FidelityParams {
    #[serde(default)]
    pub tier: TierParam,
    pub inverter_mode: Option<String>,
    pub dt_max: Option<Param>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Kind;

    const MOTOR: &str = r#"
schema_version: 1
identity: { name: Generic 8010 outrunner, size_class: "8010", topology: outrunner }
electrical:
  r_phase: { value: 0.18 ohm, source: datasheet }
  l_d: 120 uH
  l_q: 120 uH
  lambda_m: { value: 0.0115 Wb, source: derived }
  entered_as: kv
  kv: { value: 30 rpm/V, source: datasheet }
  connection: star
magnetic:
  emf_shape: { kind: trapezoidal, flat_top: 120 deg }
  cogging: [ { amplitude: 0.02 N*m } ]
  alpha_br: -0.12 %/K
winding: { slots: 36, pole_pairs: 21 }
mechanical:
  j_rotor: 1.2 kg*cm^2
  friction: { static_torque: 0.01 N*m, coulomb: 0.008 N*m, viscous: 1e-5 }
"#;

    #[test]
    fn motor_yaml_round_trips_and_parses_quantities() {
        let m: MotorParams = serde_saphyr::from_str(MOTOR).expect("motor yaml");
        assert_eq!(m.winding.pole_pairs, 21);
        assert!((m.mechanical.j_rotor.si(Kind::Inertia).unwrap() - 1.2e-4).abs() < 1e-15);
        assert!(matches!(m.magnetic.emf_shape, EmfShape::Trapezoidal { .. }));
        let back: MotorParams =
            serde_saphyr::from_str(&serde_saphyr::to_string(&m).unwrap()).unwrap();
        assert_eq!(back, m);
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let bad = MOTOR.replace("connection: star", "connection: star\n  colour: red");
        assert!(serde_saphyr::from_str::<MotorParams>(&bad).is_err());
    }

    #[test]
    fn tagged_components_parse() {
        let s: SupplyParams = serde_saphyr::from_str(
            "kind: psu\nvoltage: 24 V\ncurrent_limit: 5 A\nr_out: 50 mOhm\n",
        )
        .unwrap();
        assert!(matches!(s, SupplyParams::Psu { .. }));
        let l: LoadParams = serde_saphyr::from_str(
            "kind: arm\narm_length: 20 cm\narm_mass: 50 g\nmass: 0.5 kg\ndistance: 15 cm\n",
        )
        .unwrap();
        assert!(matches!(
            l,
            LoadParams::Arm {
                on_change: OnChange::KeepSpeed,
                ..
            }
        ));
        let c: ControllerParams = serde_saphyr::from_str(
            "family: foc\ncurrent: { rate: 20 kHz, bandwidth: 6283 rad/s }\nlimits: { current: 5 A }\n",
        )
        .unwrap();
        assert!(matches!(c, ControllerParams::Foc { .. }));
    }
}
