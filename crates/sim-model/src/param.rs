//! Provenance-tracked parameter values (POLISHED_IDEA §4.2).
//!
//! YAML accepts a short form `kv: "100 rpm/V"` (or a bare SI number) and a long form
//! `kv: { value: "100 rpm/V", source: datasheet, note: "seller page" }`.

use serde::{Deserialize, Serialize};

use crate::units::{Kind, UnitError, parse_quantity};

/// Where a value came from.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize, schemars::JsonSchema,
)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    Measured,
    Datasheet,
    Estimated,
    /// Computed from other parameters by a constraint rule (locked in the UI).
    Derived,
    #[default]
    Default,
}

/// A raw value as written: a number (SI) or a quantity string.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum Raw {
    Number(f64),
    Text(String),
}

impl Raw {
    /// SI value for the expected kind.
    pub fn si(&self, kind: Kind) -> Result<f64, UnitError> {
        match self {
            Raw::Number(v) => Ok(*v),
            Raw::Text(s) => parse_quantity(s, kind),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
enum ParamRepr {
    Long {
        value: Raw,
        #[serde(default)]
        source: Source,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        note: Option<String>,
    },
    Short(Raw),
}

/// A parameter value with provenance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(from = "ParamRepr", into = "ParamRepr")]
#[schemars(with = "ParamRepr")]
pub struct Param {
    pub value: Raw,
    pub source: Source,
    pub note: Option<String>,
}

impl From<ParamRepr> for Param {
    fn from(r: ParamRepr) -> Self {
        match r {
            ParamRepr::Short(value) => Param {
                value,
                source: Source::Default,
                note: None,
            },
            ParamRepr::Long {
                value,
                source,
                note,
            } => Param {
                value,
                source,
                note,
            },
        }
    }
}

impl From<Param> for ParamRepr {
    fn from(p: Param) -> Self {
        if p.source == Source::Default && p.note.is_none() {
            ParamRepr::Short(p.value)
        } else {
            ParamRepr::Long {
                value: p.value,
                source: p.source,
                note: p.note,
            }
        }
    }
}

impl Param {
    pub fn new(value: Raw, source: Source) -> Self {
        Self {
            value,
            source,
            note: None,
        }
    }

    /// SI value for `kind`.
    pub fn si(&self, kind: Kind) -> Result<f64, UnitError> {
        self.value.si(kind)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
    struct M {
        kv: Param,
        r: Param,
    }

    #[test]
    fn short_and_long_forms_round_trip() {
        let y = "kv: 100 rpm/V\nr:\n  value: 1.5 ohm\n  source: datasheet\n  note: seller page\n";
        let m: M = serde_saphyr::from_str(y).unwrap();
        assert_eq!(m.kv.source, Source::Default);
        assert!((m.kv.si(Kind::VelocityConstant).unwrap() - 10.471975511965976).abs() < 1e-12);
        assert_eq!(m.r.source, Source::Datasheet);
        assert_eq!(m.r.note.as_deref(), Some("seller page"));
        assert_eq!(m.r.si(Kind::Resistance).unwrap(), 1.5);
        let back: M = serde_saphyr::from_str(&serde_saphyr::to_string(&m).unwrap()).unwrap();
        assert_eq!(back, m);
    }

    #[test]
    fn bare_numbers_are_si() {
        let m: M = serde_saphyr::from_str("kv: 10.0\nr: 2\n").unwrap();
        assert_eq!(m.kv.si(Kind::VelocityConstant).unwrap(), 10.0);
        assert_eq!(m.r.si(Kind::Resistance).unwrap(), 2.0);
    }
}
