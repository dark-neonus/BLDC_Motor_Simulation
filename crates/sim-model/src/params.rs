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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Topology {
    Inrunner,
    Outrunner,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Connection {
    Star,
    Delta,
}

/// Which motor constant the user entered (the others are derived, EQ-CONV-13).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ConstantForm {
    Kv,
    Kt,
    Ke,
    LambdaM,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorIdentity {
    /// Display name of the motor (e.g. "Generic 8108 outrunner").
    pub name: String,
    /// Size class `DDHH` (stator diameter × stack height, mm), e.g. "8010".
    pub size_class: Option<String>,
    /// Rotor position: `inrunner` (rotor inside) or `outrunner` (rotating can outside).
    pub topology: Topology,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
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
    /// Winding connection (`star` or `delta`); the model always uses star-equivalent values (EQ-CONV-14).
    pub connection: Connection,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EmfShape {
    Sinusoidal,
    /// Flat-top width per half period [rad or "120 deg"] (EQ-MOT-03).
    Trapezoidal {
        /// Flat-top width per half period [rad or "120 deg"], 0 … π (EQ-MOT-03).
        flat_top: Param,
    },
    /// Harmonics n (odd) with amplitude b_n relative to the fundamental and phase φ_n.
    Harmonics {
        /// Harmonics in addition to the fundamental.
        terms: Vec<Harmonic>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Harmonic {
    /// Harmonic order (odd).
    pub n: u32,
    /// Amplitude relative to the fundamental.
    pub b: f64,
    /// Phase φ_n [rad].
    #[serde(default)]
    pub phase: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Saturation {
    /// Knee current [A] (EQ-MOT-10).
    pub i_knee: Param,
    /// Incremental q inductance deep in saturation [H].
    pub l_inf: Param,
    /// Cross-saturation fraction c (0 ≤ c < 1).
    pub cross: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorMagnetic {
    /// Back-EMF shape (EQ-MOT-03).
    pub emf_shape: EmfShape,
    /// Cogging harmonics: amplitude [N·m] and phase per harmonic of N_c (EQ-MOT-08).
    #[serde(default)]
    pub cogging: Vec<CoggingTerm>,
    /// Iron loss coefficients k_hy [W·s/rad], k_ed [W·s²/rad²] (EQ-MOT-09).
    pub k_hy: Option<Param>,
    /// Eddy-current iron-loss coefficient k_ed [W·s²/rad²] (EQ-MOT-09).
    pub k_ed: Option<Param>,
    /// Co-energy saturation curve (EQ-MOT-10, Detailed tier).
    pub saturation: Option<Saturation>,
    /// Magnet remanence temperature coefficient [1/K] (default −0.0012).
    pub alpha_br: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CoggingTerm {
    /// Cogging amplitude A_k [N·m] of harmonic k (EQ-MOT-08).
    pub amplitude: Param,
    /// Phase φ_k [rad].
    #[serde(default)]
    pub phase: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorWinding {
    /// Number of stator slots Q (e.g. 12 in "12N14P").
    pub slots: u32,
    /// Pole pairs p = magnets / 2 (e.g. 7 in "12N14P").
    pub pole_pairs: u32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorGeometry {
    /// Stator outer diameter.
    pub stator_od: Option<Param>,
    /// Stator inner diameter.
    pub stator_id: Option<Param>,
    /// Axial length of the stator stack.
    pub stack_length: Option<Param>,
    /// Mechanical air gap between stator and magnets.
    pub air_gap: Option<Param>,
    /// Radial magnet thickness.
    pub magnet_thickness: Option<Param>,
    /// Magnet arc as a fraction of the pole pitch (0…1).
    pub magnet_arc: Option<f64>,
    /// Rotor outer diameter (the can for an outrunner).
    pub rotor_od: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Friction {
    /// Breakaway (static) friction torque [N·m] (EQ-MECH-05).
    pub static_torque: Param,
    /// Coulomb (kinetic) friction torque [N·m], ≤ static.
    pub coulomb: Param,
    /// Viscous friction coefficient [N·m·s/rad].
    pub viscous: Param,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorMechanical {
    /// Rotor inertia [kg·m²].
    pub j_rotor: Param,
    /// Motor mass.
    pub mass: Option<Param>,
    /// Bearing friction (EQ-MECH-05).
    pub friction: Option<Friction>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorThermal {
    /// Thermal resistance winding → stator [K/W] (EQ-THERM-01).
    pub r_ws: Param,
    /// Thermal resistance stator → housing [K/W].
    pub r_sh: Param,
    /// Thermal resistance housing → ambient [K/W].
    pub r_ha: Param,
    /// Heat capacity of the winding [J/K].
    pub c_w: Param,
    /// Heat capacity of the stator iron [J/K].
    pub c_s: Param,
    /// Heat capacity of the housing / can [J/K].
    pub c_h: Param,
    /// Maximum winding temperature.
    pub t_max: Param,
    /// Reference temperature of R and λ (default 20 °C).
    pub t_ref: Option<Param>,
    /// Optional fan-cooling factor κ [s/rad] (EQ-THERM-04).
    pub kappa: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorRatings {
    /// Rated supply voltage.
    pub voltage: Option<Param>,
    /// Continuous phase current (peak per phase).
    pub current_continuous: Option<Param>,
    /// Peak phase current (short-term).
    pub current_peak: Option<Param>,
    /// Rated or maximum speed.
    pub speed: Option<Param>,
    /// Datasheet peak torque [N·m].
    pub torque_peak: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MotorParams {
    /// File format version (currently 1).
    pub schema_version: u32,
    /// Name and topology.
    pub identity: MotorIdentity,
    /// Resistance, inductances and the motor constant.
    pub electrical: MotorElectrical,
    /// Back-EMF shape, cogging, iron loss, saturation.
    pub magnetic: MotorMagnetic,
    /// Slot / pole combination.
    pub winding: MotorWinding,
    /// Dimensions (optional; used by estimates and the 3D view).
    #[serde(default = "empty_geometry")]
    pub geometry: MotorGeometry,
    /// Inertia, mass and bearing friction.
    pub mechanical: MotorMechanical,
    /// Three-node thermal network (EQ-THERM-01).
    pub thermal: Option<MotorThermal>,
    /// Datasheet ratings (for checks and limits).
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GearboxParams {
    /// Display name.
    pub name: String,
    /// Ratio N = motor turns per output turn (N > 1 reduces speed).
    pub ratio: f64,
    /// Efficiency η (0 < η ≤ 1), the same both ways (EQ-MECH-03).
    pub efficiency: f64,
    /// Total backlash at the output (EQ-MECH-04).
    pub backlash: Option<Param>,
    /// Torsional stiffness at the output [N·m/rad].
    pub stiffness: Option<Param>,
    /// Torsional damping at the output [N·m·s/rad].
    pub damping: Option<Param>,
    /// Inertia on the motor side [kg·m²].
    pub j_in: Option<Param>,
    /// Inertia on the output side [kg·m²].
    pub j_out: Option<Param>,
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize, schemars::JsonSchema,
)]
#[serde(rename_all = "snake_case")]
pub enum OnChange {
    #[default]
    KeepSpeed,
    ConserveMomentum,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum LoadParams {
    /// Arm + point mass under gravity (EQ-MECH-06).
    Arm {
        /// Length of the arm (rod).
        arm_length: Param,
        /// Mass of the arm itself (uniform rod).
        arm_mass: Param,
        /// Point mass on the arm.
        mass: Param,
        /// Distance of the point mass from the axis.
        distance: Param,
        /// What a live mass change keeps: speed (default) or momentum (D-007).
        #[serde(default)]
        on_change: OnChange,
        /// Load-side friction.
        friction: Option<Friction>,
    },
    Constant {
        /// Torque [N·m].
        torque: Param,
    },
    Brake {
        /// Torque [N·m].
        torque: Param,
    },
    Viscous {
        /// Viscous coefficient [N·m·s/rad].
        coefficient: Param,
    },
    Flywheel {
        /// Flywheel inertia [kg·m²].
        inertia: Param,
    },
}

// ------------------------------------------------------------------ power

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InverterParams {
    /// Display name.
    pub name: String,
    /// PWM frequency.
    pub f_pwm: Param,
    /// Dead time per switching edge.
    pub dead_time: Param,
    /// Switch on-resistance (incl. board traces).
    pub r_on: Param,
    /// Body-diode forward voltage.
    pub v_f: Param,
    /// Switching rise time (switching loss, EQ-INV-09).
    pub t_rise: Param,
    /// Switching fall time.
    pub t_fall: Param,
    /// Current smoothing for the dead-time sign (EQ-INV-03, default 10 mA).
    pub i_eps: Option<Param>,
    /// Maximum bus voltage of the stage.
    pub v_max: Option<Param>,
    /// Maximum phase current of the stage.
    pub i_max: Option<Param>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Chemistry {
    LiIon,
    Lipo,
    Lifepo4,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum SupplyParams {
    Ideal {
        /// Output voltage.
        voltage: Param,
    },
    Psu {
        /// Output voltage.
        voltage: Param,
        /// Current limit (constant-current mode above it).
        current_limit: Param,
        /// Output resistance.
        r_out: Param,
    },
    Battery {
        /// Cell chemistry (sets the OCV curve meaning).
        chemistry: Chemistry,
        /// Cells in series.
        series: u32,
        /// Cells in parallel.
        parallel: u32,
        /// Capacity per cell.
        capacity_cell: Param,
        /// Series resistance per cell (Thevenin R0, EQ-SUP-03).
        r0_cell: Param,
        /// RC-pair resistance per cell.
        r1_cell: Param,
        /// RC-pair capacitance per cell.
        c1_cell: Param,
        /// Initial state of charge (0 … 1).
        soc_init: f64,
        /// Per-cell OCV table: (SoC, volts) pairs, ascending SoC.
        ocv_cell: Vec<(f64, f64)>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BusParams {
    /// DC-link capacitance.
    pub capacitance: Param,
    /// Brake chopper (optional).
    pub chopper: Option<ChopperParams>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ChopperParams {
    /// Brake resistor.
    pub r_brake: Param,
    /// Bus voltage that switches the chopper on.
    pub v_on: Param,
    /// Bus voltage that switches it off (hysteresis).
    pub v_off: Param,
    /// Resistor power rating (for warnings).
    pub power_rating: Option<Param>,
}

// ------------------------------------------------------------------ sensors

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SensorCommon {
    /// Use the true value instead of the sensor model.
    #[serde(default)]
    pub bypass: bool,
    /// Sample rate.
    pub update_rate: Option<Param>,
    /// Delay from measurement to output.
    pub latency: Option<Param>,
    /// Noise standard deviation (in the sensor unit).
    pub noise: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum EncoderParams {
    Magnetic {
        /// Chip name (informational).
        chip: Option<String>,
        /// Resolution in bits.
        bits: u32,
        /// Integral nonlinearity (peak angle error).
        inl: Option<Param>,
        /// Magnet eccentricity.
        eccentricity: Option<Param>,
        /// Magnet-to-chip distance.
        air_gap: Option<Param>,
        /// Minimum allowed air gap.
        gap_min: Option<Param>,
        /// Maximum allowed air gap.
        gap_max: Option<Param>,
        /// Angular mounting offset.
        mount_offset: Option<Param>,
        /// Rate, latency, noise.
        #[serde(flatten)]
        common: SensorCommon,
    },
    Incremental {
        /// Counts per revolution (before ×4 decoding).
        cpr: u32,
        /// Angle of the index pulse.
        index_angle: Option<Param>,
        /// Rate, latency, noise.
        #[serde(flatten)]
        common: SensorCommon,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct HallParams {
    /// Per-sensor placement errors [rad electrical].
    #[serde(default)]
    pub offsets: Vec<Param>,
    /// Switching hysteresis (fraction of the field amplitude).
    pub hysteresis: Option<f64>,
    /// Rate, latency, noise.
    #[serde(flatten)]
    pub common: SensorCommon,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AdcParams {
    /// ADC resolution in bits.
    pub bits: u32,
    /// Current at full scale (±).
    pub full_scale: Param,
    /// Relative gain error.
    #[serde(default)]
    pub gain_error: f64,
    /// Offset current.
    pub offset: Option<Param>,
    /// 2 or 3 shunts.
    pub shunts: u32,
    /// Rate, latency, noise.
    #[serde(flatten)]
    pub common: SensorCommon,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SensorsParams {
    /// Hall sensors (optional).
    pub hall: Option<HallParams>,
    /// Angle encoder (optional).
    pub encoder: Option<EncoderParams>,
    /// Current-sense ADC (optional).
    pub adc: Option<AdcParams>,
    /// Gain γ of the sensorless flux observer (EQ-CTRL-09).
    pub observer_gamma: Option<Param>,
}

// ------------------------------------------------------------------ control

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct LoopParams {
    /// Run this loop (default true).
    #[serde(default = "yes")]
    pub enabled: bool,
    /// Loop rate.
    pub rate: Param,
    /// Target bandwidth [rad/s] (gains derived, EQ-CTRL-03/04) or explicit gains.
    pub bandwidth: Option<Param>,
    /// Proportional gain (overrides the bandwidth design).
    pub kp: Option<f64>,
    /// Integral gain (overrides the bandwidth design).
    pub ki: Option<f64>,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Limits {
    /// Phase current limit.
    pub current: Param,
    /// Speed limit.
    pub speed: Option<Param>,
    /// Usable fraction of the available voltage (0 … 1].
    pub voltage_fraction: Option<f64>,
    /// Setpoint ramp rate.
    pub ramp: Option<Param>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(tag = "family", rename_all = "snake_case", deny_unknown_fields)]
pub enum ControllerParams {
    SixStep {
        /// Controller rate.
        rate: Param,
        /// Optional speed loop around the six-step drive.
        speed_loop: Option<LoopParams>,
        /// Commutation advance angle.
        advance: Option<Param>,
        /// Current, speed and voltage limits.
        limits: Limits,
    },
    OpenLoop {
        /// Controller rate.
        rate: Param,
        /// Voltage at standstill (boost).
        v0: Param,
        /// Voltage per electrical speed (V/f slope) [V·s/rad].
        k_vf: Param,
        /// Open-loop acceleration [rad/s²].
        accel: Param,
        /// Current, speed and voltage limits.
        limits: Limits,
    },
    Foc {
        /// Current loop (EQ-CTRL-03).
        current: LoopParams,
        /// Velocity loop (EQ-CTRL-04).
        velocity: Option<LoopParams>,
        /// Position loop (EQ-CTRL-04).
        position: Option<LoopParams>,
        /// Use the flux observer instead of a sensor.
        #[serde(default)]
        sensorless: bool,
        /// Setpoint prefilter.
        #[serde(default)]
        prefilter: bool,
        /// Current, speed and voltage limits.
        limits: Limits,
    },
    Mit {
        /// Current loop (EQ-CTRL-03).
        current: LoopParams,
        /// Controller rate.
        rate: Param,
        /// MIT stiffness K_p [N·m/rad] (EQ-CTRL-07).
        kp: Param,
        /// MIT damping K_d [N·m·s/rad].
        kd: Param,
        /// MIT feed-forward torque [N·m].
        tau_ff: Option<Param>,
        /// Current, speed and voltage limits.
        limits: Limits,
    },
    /// User Luau script (P09.T10).
    Custom {
        /// Luau script source (P09.T10).
        script: String,
        /// Controller rate.
        rate: Param,
        /// Signal paths the script reads.
        inputs: Vec<String>,
        /// Signal paths the script writes.
        outputs: Vec<String>,
        /// Current, speed and voltage limits.
        limits: Limits,
    },
}

// ------------------------------------------------------------------ faults, protection, fidelity

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ProtectionParams {
    /// Over-current trip.
    pub over_current: Option<Threshold>,
    /// Bus over-voltage trip.
    pub over_voltage: Option<Threshold>,
    /// Bus under-voltage trip.
    pub under_voltage: Option<Threshold>,
    /// Winding over-temperature trip.
    pub over_temperature: Option<Threshold>,
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize, schemars::JsonSchema,
)]
#[serde(rename_all = "snake_case")]
pub enum ProtectAction {
    #[default]
    DisablePwm,
    Brake,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Threshold {
    /// Enable this protection (default true).
    #[serde(default = "yes")]
    pub enabled: bool,
    /// Trip level (in the watched quantity's unit).
    pub level: Param,
    /// Filter time before tripping.
    pub filter: Option<Param>,
    /// What a trip does.
    #[serde(default)]
    pub action: ProtectAction,
    /// Stay tripped until reset.
    #[serde(default)]
    pub latch: bool,
}

#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize, schemars::JsonSchema,
)]
#[serde(rename_all = "snake_case")]
pub enum TierParam {
    Ideal,
    #[default]
    Standard,
    Detailed,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FidelityParams {
    /// Model detail: ideal, standard (default) or detailed (EQ-NUM-07).
    #[serde(default)]
    pub tier: TierParam,
    /// Override the tier's inverter mode (averaged / switching).
    pub inverter_mode: Option<String>,
    /// Override the maximum integration step.
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
