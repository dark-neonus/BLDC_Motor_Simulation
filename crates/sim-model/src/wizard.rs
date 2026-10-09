//! Datasheet wizard (P04.T12): datasheet numbers + answers to convention questions →
//! a complete `MotorParams` with provenance. Missing values are filled by the
//! EQ-EST rules (docs/docs/physics/estimation.md), each with a confidence.

use std::f64::consts::{PI, SQRT_2};

use serde::{Deserialize, Serialize};

use crate::constraints::{self, Issue};
use crate::param::{Param, Raw, Source};
use crate::params::*;
use crate::units::Kind;
use crate::winding;

/// What the user copied from the datasheet. Quantities are numbers (SI) or strings with units.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct WizardInputs {
    pub name: String,
    pub topology: Option<Topology>,
    /// `DDHH` stator size, e.g. "8108".
    pub size_class: Option<String>,
    pub slots: u32,
    /// As printed; may be the pole count or the pole pairs (asked if ambiguous).
    pub poles: u32,
    pub kv: Option<Raw>,
    pub kt: Option<Raw>,
    pub resistance: Option<Raw>,
    pub inductance: Option<Raw>,
    pub mass: Option<Raw>,
    pub outer_diameter: Option<Raw>,
    pub height: Option<Raw>,
    pub j_rotor: Option<Raw>,
    pub current_continuous: Option<Raw>,
    pub current_peak: Option<Raw>,
    pub torque_peak: Option<Raw>,
    pub voltage: Option<Raw>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum QuestionId {
    PolesMeaning,
    KvDefinition,
    KtBasis,
    ResistanceMeasured,
    InductanceMeasured,
    Connection,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
pub struct Question {
    pub id: QuestionId,
    pub text: String,
    /// (answer key, label).
    pub options: Vec<(String, String)>,
    pub suggested: String,
    pub answer: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Confidence {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
pub struct Estimate {
    pub path: String,
    /// Rule id, e.g. `EQ-EST-01`.
    pub rule: String,
    pub confidence: Confidence,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WizardResult {
    pub motor: MotorParams,
    pub estimates: Vec<Estimate>,
    pub issues: Vec<Issue>,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum WizardError {
    #[error("{0}")]
    Input(String),
    #[error("unknown question `{0:?}`")]
    NoSuchQuestion(QuestionId),
    #[error("`{1}` is not an option for {0:?}")]
    BadAnswer(QuestionId, String),
    #[error("unanswered questions: {0:?}")]
    Unanswered(Vec<QuestionId>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct WizardState {
    inputs: WizardInputs,
    questions: Vec<Question>,
}

fn q(id: QuestionId, text: &str, options: &[(&str, &str)], suggested: &str) -> Question {
    Question {
        id,
        text: text.into(),
        options: options
            .iter()
            .map(|(k, l)| ((*k).into(), (*l).into()))
            .collect(),
        suggested: suggested.into(),
        answer: None,
    }
}

/// A valid winding with slots per pole per phase q ≥ 1/4 (real machines; q = 1/7 is
/// valid on paper but no one builds it).
fn plausible(slots: u32, pole_pairs: u32) -> bool {
    winding::check(slots, pole_pairs).is_ok() && slots as f64 / (6.0 * pole_pairs as f64) >= 0.25
}

fn si(r: &Option<Raw>, kind: Kind, what: &str) -> Result<Option<f64>, WizardError> {
    r.as_ref()
        .map(|r| {
            r.si(kind)
                .map_err(|e| WizardError::Input(format!("{what}: {e}")))
        })
        .transpose()
}

/// Start the wizard: checks the inputs and lists the convention questions they raise.
pub fn start(inputs: WizardInputs) -> Result<WizardState, WizardError> {
    use QuestionId::*;
    if inputs.kv.is_none() && inputs.kt.is_none() {
        return Err(WizardError::Input(
            "enter Kv or Kt (one motor constant is required)".into(),
        ));
    }
    if inputs.resistance.is_none() {
        return Err(WizardError::Input("enter the winding resistance".into()));
    }
    let mut qs = Vec::new();
    // "Poles" printed as 14 may be 14 magnets (7 pairs) or 14 pairs; let the slot count decide.
    let as_count = inputs.poles.is_multiple_of(2) && plausible(inputs.slots, inputs.poles / 2);
    let as_pairs = plausible(inputs.slots, inputs.poles);
    match (as_count, as_pairs) {
        (false, false) => {
            return Err(WizardError::Input(format!(
                "{} slots with {} poles is not a valid winding either as pole count or as pole pairs",
                inputs.slots, inputs.poles
            )));
        }
        (true, true) => qs.push(q(
            PolesMeaning,
            "Is the pole number the number of magnets (poles) or of pole pairs?",
            &[
                ("pole_count", "number of magnets (e.g. 12N14P → 14)"),
                ("pole_pairs", "pole pairs"),
            ],
            "pole_count",
        )),
        _ => {}
    }
    if inputs.kv.is_some() {
        qs.push(q(
            KvDefinition,
            "How is Kv defined on this datasheet?",
            &[
                (
                    "no_load_dc",
                    "rpm per volt of supply at no load (hobby / drone datasheets)",
                ),
                (
                    "ll_rms",
                    "rpm per volt RMS line-to-line (industrial datasheets)",
                ),
            ],
            "no_load_dc",
        ));
    }
    if inputs.kt.is_some() {
        qs.push(q(
            KtBasis,
            "Is Kt given per amp of peak or of RMS phase current?",
            &[("peak", "per peak phase amp"), ("rms", "per RMS phase amp")],
            "rms",
        ));
    }
    let measured = &[
        ("line_to_line", "between two motor wires"),
        ("phase", "one phase winding"),
    ];
    qs.push(q(
        ResistanceMeasured,
        "Is the resistance measured between two wires or per phase?",
        measured,
        "line_to_line",
    ));
    if inputs.inductance.is_some() {
        qs.push(q(
            InductanceMeasured,
            "Is the inductance measured between two wires or per phase?",
            measured,
            "line_to_line",
        ));
    }
    qs.push(q(
        Connection,
        "Is the winding star (wye) or delta connected?",
        &[("star", "star / wye"), ("delta", "delta")],
        "star",
    ));
    Ok(WizardState {
        inputs,
        questions: qs,
    })
}

fn param(v: f64, unit: &str, source: Source, note: Option<&str>) -> Param {
    Param {
        value: Raw::Text(format!("{v:.6} {unit}")),
        source,
        note: note.map(str::to_owned),
    }
}

impl WizardState {
    pub fn questions(&self) -> &[Question] {
        &self.questions
    }

    pub fn answer(&mut self, id: QuestionId, key: &str) -> Result<(), WizardError> {
        let q = self
            .questions
            .iter_mut()
            .find(|q| q.id == id)
            .ok_or(WizardError::NoSuchQuestion(id))?;
        if !q.options.iter().any(|(k, _)| k == key) {
            return Err(WizardError::BadAnswer(id, key.into()));
        }
        q.answer = Some(key.into());
        Ok(())
    }

    /// Accept every suggested answer that is still open.
    pub fn accept_suggestions(&mut self) {
        for q in &mut self.questions {
            q.answer.get_or_insert_with(|| q.suggested.clone());
        }
    }

    fn ans(&self, id: QuestionId) -> Option<&str> {
        self.questions
            .iter()
            .find(|q| q.id == id)
            .and_then(|q| q.answer.as_deref())
    }

    pub fn finish(&self) -> Result<WizardResult, WizardError> {
        use QuestionId::*;
        let open: Vec<_> = self
            .questions
            .iter()
            .filter(|q| q.answer.is_none())
            .map(|q| q.id)
            .collect();
        if !open.is_empty() {
            return Err(WizardError::Unanswered(open));
        }
        let i = &self.inputs;
        let ds = Source::Datasheet;
        let mut est = Vec::new();
        let mut mark = |path: &str, rule: &str, c: Confidence| {
            est.push(Estimate {
                path: path.into(),
                rule: rule.into(),
                confidence: c,
            });
        };

        let pole_pairs = match self.ans(PolesMeaning) {
            Some("pole_pairs") => i.poles,
            Some(_) => i.poles / 2,
            None if i.poles.is_multiple_of(2) && plausible(i.slots, i.poles / 2) => i.poles / 2,
            None => i.poles,
        };
        let delta = self.ans(Connection) == Some("delta");
        // Star-equivalent phase value: line-to-line → /2 for both star and delta;
        // a delta phase winding → /3.
        let star_eq = |v: f64, q: QuestionId| match (self.ans(q), delta) {
            (Some("line_to_line"), _) => v / 2.0,
            (_, true) => v / 3.0,
            _ => v,
        };
        let r = star_eq(
            si(&i.resistance, Kind::Resistance, "resistance")?.unwrap_or_default(),
            ResistanceMeasured,
        );
        let topology = i.topology.unwrap_or(Topology::Outrunner);

        // Size envelope: datasheet, or EQ-EST-05 from the size class.
        let class = i.size_class.as_deref().and_then(|s| {
            let (d, h) = s.split_at(s.len().checked_sub(2)?);
            Some((d.parse::<f64>().ok()? * 1e-3, h.parse::<f64>().ok()? * 1e-3))
        });
        let od = match si(&i.outer_diameter, Kind::Length, "outer diameter")? {
            Some(v) => Some((v, ds)),
            None => class.map(|(d, _)| {
                mark("motor.geometry.rotor_od", "EQ-EST-05", Confidence::Low);
                (1.07 * d + 5e-3, Source::Estimated)
            }),
        };
        let height =
            si(&i.height, Kind::Length, "height")?.or_else(|| class.map(|(_, h)| h + 15e-3));
        let mass = match si(&i.mass, Kind::Mass, "mass")? {
            Some(m) => Some((m, ds)),
            None => match (od, height) {
                (Some((d, _)), Some(h)) => {
                    mark("motor.mechanical.mass", "EQ-EST-04", Confidence::Low);
                    Some((2200.0 * PI / 4.0 * d * d * h, Source::Estimated))
                }
                _ => None,
            },
        };

        // Motor constant → λ (EQ-CONV-09…11). Kv is stored as LL-peak rpm/V.
        let (entered_as, kv_p, kt_p, lambda) =
            if let Some(kv) = si(&i.kv, Kind::VelocityConstant, "Kv")? {
                let (kv, note) = match self.ans(KvDefinition) {
                    Some("ll_rms") => (kv / SQRT_2, Some("converted from rpm per V RMS (÷√2)")),
                    _ => (kv, None),
                };
                let lambda = 1.0 / kv / (3f64.sqrt() * pole_pairs as f64);
                (
                    ConstantForm::Kv,
                    Some(param(kv * 60.0 / (2.0 * PI), "rpm/V", ds, note)),
                    None,
                    lambda,
                )
            } else {
                let kt = si(&i.kt, Kind::TorqueConstant, "Kt")?.unwrap_or_default();
                let (kt, note) = match self.ans(KtBasis) {
                    Some("rms") => (kt / SQRT_2, Some("converted from per-RMS-amp (÷√2)")),
                    _ => (kt, None),
                };
                let lambda = kt / (1.5 * pole_pairs as f64);
                (
                    ConstantForm::Kt,
                    None,
                    Some(param(kt, "N*m/A", ds, note)),
                    lambda,
                )
            };
        let kt_si = 1.5 * pole_pairs as f64 * lambda;

        let (l, l_src) = match si(&i.inductance, Kind::Inductance, "inductance")? {
            Some(v) => (star_eq(v, InductanceMeasured), ds),
            None => {
                mark("motor.electrical.l_d", "EQ-EST-02", Confidence::Low);
                (if r > 0.5 { 0.3e-3 } else { 1.0e-3 } * r, Source::Estimated)
            }
        };

        let j = match (si(&i.j_rotor, Kind::Inertia, "rotor inertia")?, mass, od) {
            (Some(j), _, _) => Some(param(j * 1e7, "g*cm^2", ds, None)),
            (None, Some((m, ms)), Some((d, ds2))) => {
                let c = if ms == ds && ds2 == ds {
                    Confidence::Medium
                } else {
                    Confidence::Low
                };
                mark("motor.mechanical.j_rotor", "EQ-EST-01", c);
                let k = if topology == Topology::Outrunner {
                    0.35
                } else {
                    0.045
                };
                Some(param(
                    k * m * (d / 2.0).powi(2) * 1e7,
                    "g*cm^2",
                    Source::Estimated,
                    Some("EQ-EST-01"),
                ))
            }
            _ => None,
        };
        let j_rotor = j.ok_or_else(|| {
            WizardError::Input(
                "rotor inertia: enter it, or the mass and size class so it can be estimated".into(),
            )
        })?;

        let thermal = match (mass, od, height) {
            (Some((m, _)), Some((d, _)), Some(h)) => {
                for p in ["r_ws", "r_sh", "r_ha", "c_w", "c_s", "c_h", "t_max"] {
                    mark(&format!("motor.thermal.{p}"), "EQ-EST-03", Confidence::Low);
                }
                let e = Source::Estimated;
                let a = PI * d * h + PI / 2.0 * d * d;
                Some(MotorThermal {
                    r_ws: param(0.5, "K/W", e, None),
                    r_sh: param(0.3, "K/W", e, None),
                    r_ha: param(1.0 / (12.0 * a), "K/W", e, None),
                    c_w: param(0.15 * m * 385.0, "J/K", e, None),
                    c_s: param(0.35 * m * 460.0, "J/K", e, None),
                    c_h: param(0.40 * m * 900.0, "J/K", e, None),
                    t_max: param(100.0, "degC", e, None),
                    t_ref: None,
                    kappa: None,
                })
            }
            _ => None,
        };

        let i_peak = si(&i.current_peak, Kind::Current, "peak current")?;
        let t_peak = si(&i.torque_peak, Kind::Torque, "peak torque")?;
        let current_peak = match (i_peak, t_peak) {
            (Some(v), _) => Some(param(v, "A", ds, None)),
            (None, Some(t)) => Some(param(
                t / kt_si,
                "A",
                Source::Derived,
                Some("peak torque / Kt"),
            )),
            _ => None,
        };
        let opt = |v: Option<f64>, unit: &str| v.map(|v| param(v, unit, ds, None));

        let mut motor = MotorParams {
            schema_version: crate::io::SCHEMA_VERSION,
            identity: MotorIdentity {
                name: i.name.clone(),
                size_class: i.size_class.clone(),
                topology,
            },
            electrical: MotorElectrical {
                r_phase: param(r, "ohm", ds, None),
                l_d: param(l, "H", l_src, None),
                l_q: param(l, "H", l_src, None),
                lambda_m: param(lambda, "Wb", Source::Derived, None),
                entered_as,
                kv: kv_p,
                kt: kt_p,
                ke: None,
                connection: if delta {
                    crate::params::Connection::Delta
                } else {
                    crate::params::Connection::Star
                },
            },
            magnetic: MotorMagnetic {
                emf_shape: EmfShape::Sinusoidal,
                cogging: vec![],
                k_hy: None,
                k_ed: None,
                saturation: None,
                alpha_br: None,
            },
            winding: MotorWinding {
                slots: i.slots,
                pole_pairs,
            },
            geometry: MotorGeometry {
                stator_od: None,
                stator_id: None,
                stack_length: None,
                air_gap: None,
                magnet_thickness: None,
                magnet_arc: None,
                rotor_od: od.map(|(d, s)| param(d * 1e3, "mm", s, None)),
            },
            mechanical: MotorMechanical {
                j_rotor,
                mass: mass.map(|(m, s)| param(m * 1e3, "g", s, None)),
                friction: None,
            },
            thermal,
            ratings: MotorRatings {
                voltage: opt(si(&i.voltage, Kind::Voltage, "voltage")?, "V"),
                current_continuous: opt(
                    si(&i.current_continuous, Kind::Current, "continuous current")?,
                    "A",
                ),
                current_peak,
                speed: None,
                torque_peak: opt(t_peak, "N*m"),
            },
        };
        let mut issues = Vec::new();
        constraints::derive(&mut motor, &mut issues);
        issues.extend(constraints::validate(&motor));
        Ok(WizardResult {
            motor,
            estimates: est,
            issues,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(s: &str) -> Option<Raw> {
        Some(Raw::Text(s.into()))
    }

    /// CubeMars AK10-9 V2.0 KV100 (motor stage of the 9:1 actuator), public spec table:
    /// <https://www.cubemars.com/goods.php?id=982> — 21 pole pairs (42 magnets), 36 slots,
    /// Kv 100 rpm/V, 90 mΩ / 331 µH phase-to-phase, Kt 0.095 N·m/A, 50 A peak,
    /// 38 N·m peak at the 9:1 output (4.22 N·m at the motor), 960 g with gearbox.
    fn ak10() -> WizardInputs {
        WizardInputs {
            name: "AK10-9 motor stage".into(),
            size_class: Some("10015".into()),
            slots: 36,
            poles: 42,
            kv: t("100 rpm/V"),
            resistance: t("90 mohm"),
            inductance: t("331 uH"),
            mass: t("630 g"),
            current_peak: t("50 A"),
            ..Default::default()
        }
    }

    #[test]
    fn real_datasheet_reproduces_kt_and_peak_torque() {
        let mut w = start(ak10()).unwrap();
        // 36 slots: 42 can only be the magnet count (21 pairs), so no poles question.
        assert!(
            w.questions()
                .iter()
                .all(|q| q.id != QuestionId::PolesMeaning)
        );
        assert!(matches!(w.finish(), Err(WizardError::Unanswered(_))));
        w.accept_suggestions();
        let r = w.finish().unwrap();
        assert!(
            r.issues
                .iter()
                .all(|i| i.severity != constraints::Severity::Reject),
            "{:?}",
            r.issues
        );
        let e = &r.motor.electrical;
        let kt = 1.5 * 21.0 * e.lambda_m.value.si(Kind::FluxLinkage).unwrap();
        assert!(
            (kt / 0.095 - 1.0).abs() < 0.2,
            "Kt {kt} vs datasheet 0.095 (hobby Kv vs Kt differ ~13 %)"
        );
        assert!(
            (kt * 50.0 / (38.0 / 9.0) - 1.0).abs() < 0.1,
            "peak torque {}",
            kt * 50.0
        );
        assert!((e.r_phase.value.si(Kind::Resistance).unwrap() - 0.045).abs() < 1e-9);
        assert_eq!(e.r_phase.source, Source::Datasheet);
        assert_eq!(e.lambda_m.source, Source::Derived);
        let j = r.estimates.iter().find(|x| x.rule == "EQ-EST-01").unwrap();
        assert_eq!(
            j.confidence,
            Confidence::Low,
            "OD was estimated from the size class"
        );
        assert!(r.estimates.iter().any(|x| x.path == "motor.thermal.r_ha"));
    }

    #[test]
    fn conventions_change_the_model() {
        let mut w = start(WizardInputs {
            kv: t("70.7 rpm/V"),
            ..ak10()
        })
        .unwrap();
        w.accept_suggestions();
        w.answer(QuestionId::KvDefinition, "ll_rms").unwrap();
        w.answer(QuestionId::Connection, "delta").unwrap();
        w.answer(QuestionId::ResistanceMeasured, "phase").unwrap();
        assert!(w.answer(QuestionId::Connection, "triangle").is_err());
        let m = w.finish().unwrap().motor;
        let kv = m
            .electrical
            .kv
            .unwrap()
            .value
            .si(Kind::VelocityConstant)
            .unwrap()
            * 60.0
            / (2.0 * PI);
        assert!((kv - 50.0).abs() < 0.01, "{kv}");
        assert!((m.electrical.r_phase.value.si(Kind::Resistance).unwrap() - 0.03).abs() < 1e-9);
        // 12 slots with "14 poles": 14 pairs is invalid → pole count, no question.
        let w = start(WizardInputs {
            slots: 12,
            poles: 14,
            ..ak10()
        })
        .unwrap();
        assert!(
            w.questions()
                .iter()
                .all(|q| q.id != QuestionId::PolesMeaning)
        );
        assert!(
            start(WizardInputs {
                slots: 12,
                poles: 12,
                ..ak10()
            })
            .is_err()
        );
        assert!(start(WizardInputs { kv: None, ..ak10() }).is_err());
    }
}
