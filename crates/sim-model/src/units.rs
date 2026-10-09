//! Units (CONVENTIONS §4): SI internally; YAML/UI quantity strings like `"100 rpm/V"`
//! are parsed with a whitelisted unit table. Typed SI quantities for the model/API
//! boundary come from `uom` (D-003).

use serde::{Deserialize, Serialize};

/// `uom` SI quantity types (f64) for the model/API boundary (D-003).
pub use uom::si::f64 as si;

/// Physical kind of a quantity (what a parameter expects).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Dimensionless,
    Angle,
    AngularVelocity,
    Length,
    Mass,
    Time,
    Frequency,
    Current,
    Voltage,
    Resistance,
    Inductance,
    FluxLinkage,
    Torque,
    Inertia,
    Power,
    Energy,
    Capacitance,
    Charge,
    Temperature,
    ThermalResistance,
    HeatCapacity,
    TemperatureCoefficient,
    RotationalDamping,
    /// Kv: rad/s per volt (SI) — shown as rpm/V.
    VelocityConstant,
    /// Ke: V per rad/s (SI) — shown as V/krpm.
    BackEmfConstant,
    /// Kt: N·m per A.
    TorqueConstant,
}

const RPM: f64 = std::f64::consts::PI / 30.0; // rad/s per rpm
const KGF: f64 = 9.80665;

/// (canonical unit, kind, scale, offset): si = value·scale + offset.
const TABLE: &[(&str, Kind, f64, f64)] = &[
    ("rpm", Kind::AngularVelocity, RPM, 0.0),
    ("rad/s", Kind::AngularVelocity, 1.0, 0.0),
    (
        "deg/s",
        Kind::AngularVelocity,
        std::f64::consts::PI / 180.0,
        0.0,
    ),
    ("deg", Kind::Angle, std::f64::consts::PI / 180.0, 0.0),
    ("rad", Kind::Angle, 1.0, 0.0),
    ("mm", Kind::Length, 1e-3, 0.0),
    ("cm", Kind::Length, 1e-2, 0.0),
    ("m", Kind::Length, 1.0, 0.0),
    ("g", Kind::Mass, 1e-3, 0.0),
    ("kg", Kind::Mass, 1.0, 0.0),
    ("ns", Kind::Time, 1e-9, 0.0),
    ("us", Kind::Time, 1e-6, 0.0),
    ("ms", Kind::Time, 1e-3, 0.0),
    ("s", Kind::Time, 1.0, 0.0),
    ("hz", Kind::Frequency, 1.0, 0.0),
    ("khz", Kind::Frequency, 1e3, 0.0),
    ("ma", Kind::Current, 1e-3, 0.0),
    ("a", Kind::Current, 1.0, 0.0),
    ("mv", Kind::Voltage, 1e-3, 0.0),
    ("v", Kind::Voltage, 1.0, 0.0),
    ("mohm", Kind::Resistance, 1e-3, 0.0),
    ("ohm", Kind::Resistance, 1.0, 0.0),
    ("uh", Kind::Inductance, 1e-6, 0.0),
    ("mh", Kind::Inductance, 1e-3, 0.0),
    ("h", Kind::Inductance, 1.0, 0.0),
    ("mwb", Kind::FluxLinkage, 1e-3, 0.0),
    ("wb", Kind::FluxLinkage, 1.0, 0.0),
    ("n*m", Kind::Torque, 1.0, 0.0),
    ("nm", Kind::Torque, 1.0, 0.0),
    ("mn*m", Kind::Torque, 1e-3, 0.0),
    ("kgf*cm", Kind::Torque, KGF * 1e-2, 0.0),
    ("kg*m^2", Kind::Inertia, 1.0, 0.0),
    ("kg*cm^2", Kind::Inertia, 1e-4, 0.0),
    ("g*cm^2", Kind::Inertia, 1e-7, 0.0),
    ("w", Kind::Power, 1.0, 0.0),
    ("j", Kind::Energy, 1.0, 0.0),
    ("uf", Kind::Capacitance, 1e-6, 0.0),
    ("mf", Kind::Capacitance, 1e-3, 0.0),
    ("f", Kind::Capacitance, 1.0, 0.0),
    ("mah", Kind::Charge, 3.6, 0.0),
    ("ah", Kind::Charge, 3600.0, 0.0),
    ("c", Kind::Charge, 1.0, 0.0),
    ("degc", Kind::Temperature, 1.0, 273.15),
    ("k", Kind::Temperature, 1.0, 0.0),
    ("k/w", Kind::ThermalResistance, 1.0, 0.0),
    ("j/k", Kind::HeatCapacity, 1.0, 0.0),
    ("%/k", Kind::TemperatureCoefficient, 0.01, 0.0),
    ("1/k", Kind::TemperatureCoefficient, 1.0, 0.0),
    ("n*m*s/rad", Kind::RotationalDamping, 1.0, 0.0),
    ("rpm/v", Kind::VelocityConstant, RPM, 0.0),
    ("rad/s/v", Kind::VelocityConstant, 1.0, 0.0),
    ("v/krpm", Kind::BackEmfConstant, 1.0 / (1000.0 * RPM), 0.0),
    ("v*s/rad", Kind::BackEmfConstant, 1.0, 0.0),
    ("n*m/a", Kind::TorqueConstant, 1.0, 0.0),
    ("nm/a", Kind::TorqueConstant, 1.0, 0.0),
];

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum UnitError {
    #[error("`{0}` is not a number with an optional unit (e.g. \"2.5 mH\")")]
    Syntax(String),
    #[error("unknown unit `{unit}` in `{input}`")]
    UnknownUnit { input: String, unit: String },
    #[error("expected {expected:?}, but `{input}` is {got:?}")]
    WrongKind {
        input: String,
        expected: Kind,
        got: Kind,
    },
}

/// Normalise Unicode spellings to the table's ASCII keys (lower-case). Returns `None`
/// for a leading upper-case `M` (mega): lower-casing would turn `MΩ` into `mΩ`, a factor
/// of 10⁹, and no supported unit uses mega.
fn normalize(unit: &str) -> Option<String> {
    if unit.trim().starts_with('M') {
        return None;
    }
    let u = unit
        .trim()
        .replace(['µ', 'μ'], "u")
        .replace(['·', '⋅'], "*")
        .replace('Ω', "ohm")
        .replace('²', "^2")
        .replace("°c", "degc")
        .replace("°C", "degc")
        .replace('°', "deg")
        .replace(' ', "");
    Some(u.to_lowercase())
}

/// Parse `"<number> [unit]"`. A bare number is SI of the expected kind.
pub fn parse_quantity(input: &str, expected: Kind) -> Result<f64, UnitError> {
    let s = input.trim();
    let split = s
        .char_indices()
        .find(|&(i, c)| {
            !(c.is_ascii_digit()
                || c == '.'
                || c == '+'
                || c == '-'
                || ((c == 'e' || c == 'E')
                    && i > 0
                    && s[i + 1..]
                        .chars()
                        .next()
                        .is_some_and(|n| n.is_ascii_digit() || n == '-' || n == '+')))
        })
        .map_or(s.len(), |(i, _)| i);
    let (num, unit) = s.split_at(split);
    let value: f64 = num
        .trim()
        .parse()
        .map_err(|_| UnitError::Syntax(input.into()))?;
    let raw_unit = unit;
    let unit = normalize(unit).ok_or_else(|| UnitError::UnknownUnit {
        input: input.into(),
        unit: format!("{} (mega prefixes are not supported)", raw_unit.trim()),
    })?;
    if unit.is_empty() {
        return Ok(value);
    }
    let &(_, kind, scale, offset) =
        TABLE
            .iter()
            .find(|(u, _, _, _)| *u == unit)
            .ok_or_else(|| UnitError::UnknownUnit {
                input: input.into(),
                unit: unit.clone(),
            })?;
    if kind != expected {
        return Err(UnitError::WrongKind {
            input: input.into(),
            expected,
            got: kind,
        });
    }
    Ok(value * scale + offset)
}

/// The kind of a registered signal unit (`"rad/s"`, `"N*m"`, `"-"` …); `None` if the
/// unit is not in the table.
pub fn kind_of_unit(unit: &str) -> Option<Kind> {
    if matches!(unit.trim(), "-" | "") {
        return Some(Kind::Dimensionless);
    }
    let key = normalize(unit)?;
    TABLE
        .iter()
        .find(|(u, _, _, _)| *u == key)
        .map(|&(_, k, _, _)| k)
}

/// Format an SI value in `unit` (one of the table's canonical keys), e.g. `"2.5 mH"`.
pub fn format_quantity(si_value: f64, unit: &str) -> Option<String> {
    let key = normalize(unit)?;
    let &(_, _, scale, offset) = TABLE.iter().find(|(u, _, _, _)| *u == key)?;
    Some(format!("{} {}", (si_value - offset) / scale, unit))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Deserialize)]
    struct Case {
        input: String,
        kind: Kind,
        si: f64,
    }
    #[derive(Deserialize)]
    struct Fixture {
        cases: Vec<Case>,
    }

    #[test]
    fn shared_fixture_parses_exactly() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../schemas/units-fixture.yaml"
        );
        let text = std::fs::read_to_string(path).expect("fixture file");
        let fx: Fixture = serde_saphyr::from_str(&text).expect("fixture yaml");
        assert!(fx.cases.len() >= 40);
        for c in &fx.cases {
            let v = parse_quantity(&c.input, c.kind).unwrap_or_else(|e| panic!("{}: {e}", c.input));
            assert!(
                (v - c.si).abs() <= 1e-12 * c.si.abs() + 1e-15,
                "{}: {v} vs {}",
                c.input,
                c.si
            );
        }
    }

    #[test]
    fn helpful_errors() {
        assert!(matches!(
            parse_quantity("2 ohm", Kind::Inductance),
            Err(UnitError::WrongKind { .. })
        ));
        assert!(matches!(
            parse_quantity("2 furlongs", Kind::Length),
            Err(UnitError::UnknownUnit { .. })
        ));
        assert!(matches!(
            parse_quantity("abc", Kind::Length),
            Err(UnitError::Syntax(_))
        ));
        assert_eq!(format_quantity(0.0025, "mH").as_deref(), Some("2.5 mH"));
        // Offset units round-trip: 318.15 K is 45 °C (rtol 1e-12 for the offset subtraction).
        let c = format_quantity(318.15, "degC").unwrap();
        let back = parse_quantity(&c, Kind::Temperature).unwrap();
        assert!((back / 318.15 - 1.0).abs() < 1e-12, "{c}");
        assert!(c.starts_with("45") && c.ends_with(" degC"), "{c}");
        // Mega is not silently read as milli.
        assert!(parse_quantity("1 MΩ", Kind::Resistance).is_err());
        assert!(parse_quantity("1 Mohm", Kind::Resistance).is_err());
        assert!(parse_quantity("1 mΩ", Kind::Resistance).is_ok());
    }
}
