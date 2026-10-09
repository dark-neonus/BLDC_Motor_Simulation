//! Constraint graph for motor parameters (POLISHED_IDEA §4.3, P04.T05).
//!
//! * **Derive** rules fill locked values from the entered ones (Kv ↔ Ke ↔ Kt ↔ λ,
//!   EQ-CONV-09…13).
//! * **Reject** rules block contradictory input (the edit is not applied).
//! * **Warn** rules flag plausible-but-unusual values.
//!
//! [`apply_edit`] edits one field by its dotted path (CONVENTIONS §3), re-derives,
//! validates and either returns the new model or rejects it unchanged.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::param::{Param, Raw, Source};
use crate::params::{ConstantForm, MotorParams};
use crate::units::Kind;
use crate::winding;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Severity {
    Info,
    Warn,
    Reject,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Issue {
    pub severity: Severity,
    /// Dotted parameter path, e.g. `motor.electrical.kv`.
    pub path: String,
    /// Beginner-friendly explanation.
    pub message: String,
    /// Help registry id (`param:<path>`).
    pub help_id: String,
}

fn issue(sev: Severity, path: &str, msg: impl Into<String>) -> Issue {
    Issue {
        severity: sev,
        path: path.into(),
        message: msg.into(),
        help_id: format!("param:{path}"),
    }
}

/// Result of one edit.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EditResult {
    pub model: MotorParams,
    /// (path, old, new) for every leaf that changed, including derived values.
    pub changed: Vec<(String, Value, Value)>,
    pub issues: Vec<Issue>,
    /// True when a Reject issue blocked the edit (`model` is the unchanged input).
    pub rejected: bool,
}

fn si(p: &Param, kind: Kind, path: &str, out: &mut Vec<Issue>) -> Option<f64> {
    match p.si(kind) {
        Ok(v) if v.is_finite() => Some(v),
        Ok(_) => {
            out.push(issue(
                Severity::Reject,
                path,
                "value must be a finite number",
            ));
            None
        }
        Err(e) => {
            out.push(issue(Severity::Reject, path, e.to_string()));
            None
        }
    }
}

fn positive(p: &Param, kind: Kind, path: &str, out: &mut Vec<Issue>) -> Option<f64> {
    let v = si(p, kind, path, out)?;
    if v <= 0.0 {
        out.push(issue(Severity::Reject, path, "must be greater than zero"));
        return None;
    }
    Some(v)
}

/// λ_m from the entered constant (EQ-CONV-13; Kv/Ke LL-peak, Kt per peak phase amp).
fn lambda_from_entered(m: &MotorParams, out: &mut Vec<Issue>) -> Option<f64> {
    let p = m.winding.pole_pairs as f64;
    let e = &m.electrical;
    let s3 = 3f64.sqrt();
    let need = |o: &Option<Param>, name: &str, out: &mut Vec<Issue>| {
        if o.is_none() {
            out.push(issue(
                Severity::Reject,
                &format!("motor.electrical.{name}"),
                format!("`entered_as: {name}` but no {name} value given"),
            ));
        }
        o.clone()
    };
    match e.entered_as {
        ConstantForm::LambdaM => positive(
            &e.lambda_m,
            Kind::FluxLinkage,
            "motor.electrical.lambda_m",
            out,
        ),
        ConstantForm::Kv => {
            let kv = positive(
                &need(&e.kv, "kv", out)?,
                Kind::VelocityConstant,
                "motor.electrical.kv",
                out,
            )?;
            Some(1.0 / (kv * s3 * p)) // Ke = 1/Kv_SI, λ = Ke/(√3 p)
        }
        ConstantForm::Ke => {
            let ke = positive(
                &need(&e.ke, "ke", out)?,
                Kind::BackEmfConstant,
                "motor.electrical.ke",
                out,
            )?;
            Some(ke / (s3 * p))
        }
        ConstantForm::Kt => {
            let kt = positive(
                &need(&e.kt, "kt", out)?,
                Kind::TorqueConstant,
                "motor.electrical.kt",
                out,
            )?;
            Some(kt / (1.5 * p))
        }
    }
}

fn derived(v: f64, unit: &str) -> Param {
    let text = crate::units::format_quantity(v, unit).unwrap_or_else(|| format!("{v}"));
    Param {
        value: Raw::Text(text),
        source: Source::Derived,
        note: Some("computed from the entered motor constant".into()),
    }
}

/// Apply all Derive rules in place (locked values recomputed).
pub fn derive(m: &mut MotorParams, out: &mut Vec<Issue>) {
    if winding::check(m.winding.slots, m.winding.pole_pairs).is_err() {
        return; // reported by validate
    }
    let Some(lam) = lambda_from_entered(m, out) else {
        return;
    };
    let p = m.winding.pole_pairs as f64;
    let ke = 3f64.sqrt() * p * lam;
    let e = &mut m.electrical;
    let keep = |slot: &mut Option<Param>,
                form: ConstantForm,
                entered: ConstantForm,
                v: f64,
                unit: &str| {
        if form != entered {
            *slot = Some(derived(v, unit));
        }
    };
    if e.entered_as != ConstantForm::LambdaM {
        e.lambda_m = derived(lam, "Wb");
    }
    keep(&mut e.kv, ConstantForm::Kv, e.entered_as, 1.0 / ke, "rpm/V");
    keep(&mut e.ke, ConstantForm::Ke, e.entered_as, ke, "V*s/rad");
    keep(
        &mut e.kt,
        ConstantForm::Kt,
        e.entered_as,
        1.5 * p * lam,
        "N*m/A",
    );
}

/// Run all Reject/Warn rules.
pub fn validate(m: &MotorParams) -> Vec<Issue> {
    let mut out = Vec::new();
    let e = &m.electrical;
    let r = positive(
        &e.r_phase,
        Kind::Resistance,
        "motor.electrical.r_phase",
        &mut out,
    );
    let ld = positive(&e.l_d, Kind::Inductance, "motor.electrical.l_d", &mut out);
    let lq = positive(&e.l_q, Kind::Inductance, "motor.electrical.l_q", &mut out);
    if let (Some(ld), Some(lq)) = (ld, lq)
        && (ld / lq > 3.0 || lq / ld > 3.0)
    {
        out.push(issue(
            Severity::Warn,
            "motor.electrical.l_q",
            "L_d and L_q differ by more than 3×; unusual for surface-magnet motors",
        ));
    }
    if let Err(err) = winding::check(m.winding.slots, m.winding.pole_pairs) {
        out.push(issue(
            Severity::Reject,
            "motor.winding.slots",
            format!("{err}. Common combinations: 9N12P, 12N14P, 24N28P, 36N42P."),
        ));
    }
    if let Some(lam) = lambda_from_entered(m, &mut out) {
        let kv_rpm =
            60.0 / (2.0 * std::f64::consts::PI * 3f64.sqrt() * m.winding.pole_pairs as f64 * lam);
        if !(1.0..=5000.0).contains(&kv_rpm) {
            out.push(issue(Severity::Warn, "motor.electrical.kv", format!("Kv = {kv_rpm:.1} rpm/V is outside the usual 1…5000 range — check units (rpm/V) and pole pairs")));
        }
        if let (Some(r), Some(lq)) = (r, lq) {
            let tau = lq / r;
            if !(1e-5..=0.1).contains(&tau) {
                out.push(issue(
                    Severity::Warn,
                    "motor.electrical.l_q",
                    format!(
                        "electrical time constant L/R = {tau:.2e} s is unusual (typical 0.1–2 ms)"
                    ),
                ));
            }
        }
    }
    if let Some(arc) = m.geometry.magnet_arc
        && !(0.0..=1.0).contains(&arc)
    {
        out.push(issue(
            Severity::Reject,
            "motor.geometry.magnet_arc",
            "magnet arc is a fraction of the pole pitch (0…1)",
        ));
    }
    if let Some(s) = &m.magnetic.saturation
        && !(0.0..1.0).contains(&s.cross)
    {
        out.push(issue(
            Severity::Reject,
            "motor.magnetic.saturation.cross",
            "cross-saturation fraction must be in [0, 1)",
        ));
    }
    let rt = &m.ratings;
    if let (Some(c), Some(pk)) = (&rt.current_continuous, &rt.current_peak) {
        let (c, pk) = (
            si(
                c,
                Kind::Current,
                "motor.ratings.current_continuous",
                &mut out,
            ),
            si(pk, Kind::Current, "motor.ratings.current_peak", &mut out),
        );
        if let (Some(c), Some(pk)) = (c, pk)
            && pk < c
        {
            out.push(issue(
                Severity::Warn,
                "motor.ratings.current_peak",
                "peak current is below the continuous current",
            ));
        }
    }
    if let Some(th) = &m.thermal {
        let tmax = si(
            &th.t_max,
            Kind::Temperature,
            "motor.thermal.t_max",
            &mut out,
        );
        let tref = th.t_ref.as_ref().map_or(Some(293.15), |t| {
            si(t, Kind::Temperature, "motor.thermal.t_ref", &mut out)
        });
        if let (Some(a), Some(b)) = (tmax, tref)
            && a <= b
        {
            out.push(issue(
                Severity::Reject,
                "motor.thermal.t_max",
                "maximum winding temperature must be above the reference temperature",
            ));
        }
        for (name, p) in [("r_ws", &th.r_ws), ("r_sh", &th.r_sh), ("r_ha", &th.r_ha)] {
            positive(
                p,
                Kind::ThermalResistance,
                &format!("motor.thermal.{name}"),
                &mut out,
            );
        }
        for (name, p) in [("c_w", &th.c_w), ("c_s", &th.c_s), ("c_h", &th.c_h)] {
            positive(
                p,
                Kind::HeatCapacity,
                &format!("motor.thermal.{name}"),
                &mut out,
            );
        }
    }
    positive(
        &m.mechanical.j_rotor,
        Kind::Inertia,
        "motor.mechanical.j_rotor",
        &mut out,
    );
    if let Some(f) = &m.mechanical.friction {
        let ts = si(
            &f.static_torque,
            Kind::Torque,
            "motor.mechanical.friction.static_torque",
            &mut out,
        );
        let tc = si(
            &f.coulomb,
            Kind::Torque,
            "motor.mechanical.friction.coulomb",
            &mut out,
        );
        if let (Some(ts), Some(tc)) = (ts, tc)
            && tc > ts
        {
            out.push(issue(
                Severity::Reject,
                "motor.mechanical.friction.coulomb",
                "Coulomb friction cannot exceed static friction (EQ-MECH-05)",
            ));
        }
    }
    out
}

fn leaves(prefix: &str, v: &Value, out: &mut Vec<(String, Value)>) {
    match v {
        Value::Object(map) => {
            for (k, x) in map {
                leaves(&format!("{prefix}.{k}"), x, out);
            }
        }
        _ => out.push((prefix.to_string(), v.clone())),
    }
}

/// Edit one motor field by path (`motor.<group>.<name>`, sub-fields of a long-form
/// Param allowed: `….kv.source`). Entering a motor constant switches `entered_as`.
pub fn apply_edit(model: &MotorParams, path: &str, value: Value) -> EditResult {
    let reject = |msg: String| EditResult {
        model: model.clone(),
        changed: vec![],
        issues: vec![issue(Severity::Reject, path, msg)],
        rejected: true,
    };
    let Some(rest) = path.strip_prefix("motor.") else {
        return reject("only `motor.*` paths are handled here".into());
    };
    let Ok(mut json) = serde_json::to_value(model) else {
        return reject("internal: model not serialisable".into());
    };
    // Long-form params: an edit of `….kv` keeps provenance; mark user-entered as datasheet.
    let mut cursor = &mut json;
    let segs: Vec<&str> = rest.split('.').collect();
    for (i, s) in segs.iter().enumerate() {
        let last = i == segs.len() - 1;
        let Some(obj) = cursor.as_object_mut() else {
            return reject(format!("`{s}` is not a field group"));
        };
        if last {
            obj.insert((*s).to_string(), value.clone());
            break;
        }
        cursor = obj
            .entry((*s).to_string())
            .or_insert_with(|| Value::Object(Default::default()));
    }
    let mut new: MotorParams = match serde_json::from_value(json) {
        Ok(m) => m,
        Err(e) => return reject(format!("invalid value: {e}")),
    };
    let entered = match rest {
        "electrical.kv" => Some(ConstantForm::Kv),
        "electrical.kt" => Some(ConstantForm::Kt),
        "electrical.ke" => Some(ConstantForm::Ke),
        "electrical.lambda_m" => Some(ConstantForm::LambdaM),
        _ => None,
    };
    if let Some(form) = entered {
        new.electrical.entered_as = form;
    }
    let mut issues = Vec::new();
    derive(&mut new, &mut issues);
    issues.extend(validate(&new));
    issues.sort_by_key(|i| std::cmp::Reverse(i.severity));
    issues.dedup();
    if issues.iter().any(|i| i.severity == Severity::Reject) {
        return EditResult {
            model: model.clone(),
            changed: vec![],
            issues,
            rejected: true,
        };
    }
    let (mut a, mut b) = (Vec::new(), Vec::new());
    if let (Ok(va), Ok(vb)) = (serde_json::to_value(model), serde_json::to_value(&new)) {
        leaves("motor", &va, &mut a);
        leaves("motor", &vb, &mut b);
    }
    let changed = b
        .iter()
        .filter_map(|(p, nv)| {
            let old = a
                .iter()
                .find(|(q, _)| q == p)
                .map(|(_, v)| v.clone())
                .unwrap_or(Value::Null);
            (old != *nv).then(|| (p.clone(), old, nv.clone()))
        })
        .collect();
    EditResult {
        model: new,
        changed,
        issues,
        rejected: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn motor() -> MotorParams {
        serde_saphyr::from_str(
            r#"
schema_version: 1
identity: { name: test, topology: outrunner }
electrical:
  r_phase: 1.0 ohm
  l_d: 2.5 mH
  l_q: 2.5 mH
  lambda_m: 0.03 Wb
  entered_as: lambda_m
  connection: star
magnetic: { emf_shape: { kind: sinusoidal } }
winding: { slots: 24, pole_pairs: 14 }
mechanical: { j_rotor: 2.5e-4 }
"#,
        )
        .unwrap()
    }

    fn si_of(p: &Option<Param>, k: Kind) -> f64 {
        p.as_ref().unwrap().si(k).unwrap()
    }

    #[test]
    fn kv_edit_derives_lambda_kt_ke() {
        let r = apply_edit(
            &motor(),
            "motor.electrical.kv",
            Value::String("100 rpm/V".into()),
        );
        assert!(!r.rejected, "{:?}", r.issues);
        let e = &r.model.electrical;
        assert_eq!(e.entered_as, ConstantForm::Kv);
        // EQ-CONV worked example: Kv 100 rpm/V, p = 14 → λ = 3.9381 mWb, Kt = 82.699 mN·m/A.
        assert!((e.lambda_m.si(Kind::FluxLinkage).unwrap() - 3.9381e-3).abs() < 1e-7);
        assert!((si_of(&e.kt, Kind::TorqueConstant) - 0.082699).abs() < 1e-6);
        assert_eq!(e.lambda_m.source, Source::Derived);
        assert!(
            r.changed
                .iter()
                .any(|(p, _, _)| p.starts_with("motor.electrical.lambda_m"))
        );
    }

    #[test]
    fn invalid_slot_pole_is_rejected_and_model_unchanged() {
        let m = motor();
        let r = apply_edit(&m, "motor.winding.slots", Value::from(14));
        assert!(r.rejected);
        assert_eq!(r.model, m);
        assert!(r.issues[0].message.contains("12N14P"));
    }

    #[test]
    fn units_and_signs_checked() {
        let m = motor();
        assert!(apply_edit(&m, "motor.electrical.r_phase", Value::String("2 mH".into())).rejected);
        assert!(apply_edit(&m, "motor.electrical.r_phase", Value::from(-1.0)).rejected);
        let w = apply_edit(&m, "motor.electrical.r_phase", Value::from(1e-6));
        assert!(
            !w.rejected && w.issues.iter().any(|i| i.severity == Severity::Warn),
            "{:?}",
            w.issues
        );
    }

    /// One case per `validate` rule not covered above: (edit path, value, expected severity, issue path).
    #[test]
    fn every_rule_fires() {
        use serde_json::json;
        let th = |t_max: &str, r_ws: &str| json!({"r_ws": r_ws, "r_sh": "0.3 K/W", "r_ha": "4 K/W", "c_w": "10 J/K", "c_s": "20 J/K", "c_h": "50 J/K", "t_max": t_max});
        let cases = [
            (
                "motor.electrical.l_q",
                json!("20 mH"),
                Severity::Warn,
                "motor.electrical.l_q",
            ),
            (
                "motor.electrical.lambda_m",
                json!("0.000001 Wb"),
                Severity::Warn,
                "motor.electrical.kv",
            ),
            (
                "motor.electrical.r_phase",
                json!(1e-6),
                Severity::Warn,
                "motor.electrical.l_q",
            ),
            (
                "motor.geometry.magnet_arc",
                json!(1.5),
                Severity::Reject,
                "motor.geometry.magnet_arc",
            ),
            (
                "motor.magnetic.saturation",
                json!({"i_knee": "10 A", "l_inf": "1 mH", "cross": 1.2}),
                Severity::Reject,
                "motor.magnetic.saturation.cross",
            ),
            (
                "motor.ratings",
                json!({"current_continuous": "10 A", "current_peak": "5 A"}),
                Severity::Warn,
                "motor.ratings.current_peak",
            ),
            (
                "motor.thermal",
                th("10 degC", "0.5 K/W"),
                Severity::Reject,
                "motor.thermal.t_max",
            ),
            (
                "motor.thermal",
                th("100 degC", "-0.5 K/W"),
                Severity::Reject,
                "motor.thermal.r_ws",
            ),
            (
                "motor.mechanical.j_rotor",
                json!(0.0),
                Severity::Reject,
                "motor.mechanical.j_rotor",
            ),
            (
                "motor.mechanical.friction",
                json!({"static_torque": "0.01 N*m", "coulomb": "0.02 N*m", "viscous": 0.0}),
                Severity::Reject,
                "motor.mechanical.friction.coulomb",
            ),
            (
                "motor.winding.pole_pairs",
                json!(12),
                Severity::Reject,
                "motor.winding.slots",
            ),
        ];
        for (path, value, sev, at) in cases {
            let r = apply_edit(&motor(), path, value);
            assert!(
                r.issues.iter().any(|i| i.severity == sev && i.path == at),
                "{path}: expected {sev:?} at {at}, got {:?}",
                r.issues
            );
            assert_eq!(r.rejected, sev == Severity::Reject, "{path}");
        }
        // The fixture itself is clean.
        assert!(validate(&motor()).is_empty(), "{:?}", validate(&motor()));
    }

    proptest! {
        /// Any sequence of accepted edits leaves a model with no Reject issue.
        #[test]
        fn accepted_edits_keep_model_valid(
            edits in proptest::collection::vec((0usize..4, -1e3f64..1e3), 1..20)
        ) {
            let mut m = motor();
            for (field, v) in edits {
                let (path, val) = match field {
                    0 => ("motor.electrical.kv", Value::String(format!("{} rpm/V", v.abs() + 0.1))),
                    1 => ("motor.electrical.r_phase", Value::from(v)),
                    2 => ("motor.winding.slots", Value::from((v.abs() as u64) % 60)),
                    _ => ("motor.electrical.kt", Value::String(format!("{} N*m/A", v / 100.0))),
                };
                let r = apply_edit(&m, path, val);
                if !r.rejected {
                    m = r.model;
                }
                prop_assert!(validate(&m).iter().all(|i| i.severity != Severity::Reject));
            }
        }
    }
}
