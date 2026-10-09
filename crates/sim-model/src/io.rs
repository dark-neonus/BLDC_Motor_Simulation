//! YAML file IO (P04.T06): typed load/save with a schema modeline, schema version and
//! migration hook; errors carry the file and serde-saphyr's location.

use std::path::Path;

use serde::Serialize;
use serde::de::DeserializeOwned;

pub const SCHEMA_VERSION: u32 = 1;

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("{path}: {source}")]
    Read {
        path: String,
        source: std::io::Error,
    },
    #[error("{path}: {msg}")]
    Parse { path: String, msg: String },
    #[error(
        "{path}: schema_version {found} is newer than this program supports ({SCHEMA_VERSION})"
    )]
    TooNew { path: String, found: u32 },
    #[error("{path}: {source}")]
    Write {
        path: String,
        source: std::io::Error,
    },
}

/// Upgrade an older document in place (YAML value level). v1 is the first version.
fn migrate(_from: u32, doc: serde_json::Value) -> serde_json::Value {
    doc
}

/// Load a typed YAML document. Unknown fields are rejected by the types themselves
/// (`deny_unknown_fields`); serde-saphyr reports line/column.
pub fn load_yaml<T: DeserializeOwned>(path: &Path) -> Result<T, IoError> {
    let p = path.display().to_string();
    let text = std::fs::read_to_string(path).map_err(|source| IoError::Read {
        path: p.clone(),
        source,
    })?;
    parse_yaml(&text, &p)
}

/// Parse YAML text (`origin` names it in errors).
pub fn parse_yaml<T: DeserializeOwned>(text: &str, origin: &str) -> Result<T, IoError> {
    let generic: serde_json::Value = serde_saphyr::from_str(text).map_err(|e| IoError::Parse {
        path: origin.into(),
        msg: e.to_string(),
    })?;
    let version = generic
        .get("schema_version")
        .and_then(serde_json::Value::as_u64)
        .unwrap_or(SCHEMA_VERSION as u64) as u32;
    if version > SCHEMA_VERSION {
        return Err(IoError::TooNew {
            path: origin.into(),
            found: version,
        });
    }
    if version < SCHEMA_VERSION {
        let upgraded = migrate(version, generic);
        return serde_json::from_value(upgraded).map_err(|e| IoError::Parse {
            path: origin.into(),
            msg: e.to_string(),
        });
    }
    // Same version: parse the text directly so errors keep their line/column.
    serde_saphyr::from_str(text).map_err(|e| IoError::Parse {
        path: origin.into(),
        msg: e.to_string(),
    })
}

/// Save with a YAML-language-server modeline pointing at the schema (editor completion).
pub fn save_yaml<T: Serialize>(path: &Path, value: &T, schema_rel: &str) -> Result<(), IoError> {
    let p = path.display().to_string();
    let body = serde_saphyr::to_string(value).map_err(|e| IoError::Parse {
        path: p.clone(),
        msg: e.to_string(),
    })?;
    let text = format!("# yaml-language-server: $schema={schema_rel}\n{body}");
    std::fs::write(path, text).map_err(|source| IoError::Write { path: p, source })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::MotorParams;

    const OK: &str = "schema_version: 1\nidentity: { name: t, topology: inrunner }\nelectrical:\n  r_phase: 1 ohm\n  l_d: 1 mH\n  l_q: 1 mH\n  lambda_m: 0.01 Wb\n  entered_as: lambda_m\n  connection: star\nmagnetic: { emf_shape: { kind: sinusoidal } }\nwinding: { slots: 12, pole_pairs: 7 }\nmechanical: { j_rotor: 1e-5 }\n";

    #[test]
    fn errors_have_location_and_unknown_fields_rejected() {
        let bad = OK.replace("connection: star", "connection: star\n  colour: red");
        let e = parse_yaml::<MotorParams>(&bad, "m.yaml")
            .unwrap_err()
            .to_string();
        assert!(e.contains("m.yaml") && e.contains("colour"), "{e}");
        // `colour` is on line 10 of the document, indented by two spaces.
        assert!(
            e.contains("line 10 column 3"),
            "error should locate the field: {e}"
        );
        let e = parse_yaml::<MotorParams>("a: [", "x.yaml")
            .unwrap_err()
            .to_string();
        assert!(e.starts_with("x.yaml"), "{e}");
    }

    #[test]
    fn newer_schema_rejected_and_save_load_round_trip() {
        assert!(matches!(
            parse_yaml::<MotorParams>(&OK.replace("schema_version: 1", "schema_version: 9"), "f"),
            Err(IoError::TooNew { .. })
        ));
        let m: MotorParams = parse_yaml(OK, "ok").unwrap();
        let dir = std::env::temp_dir().join(format!("bldc-io-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let f = dir.join("m.yaml");
        save_yaml(&f, &m, "../../schemas/motor.schema.json").unwrap();
        let text = std::fs::read_to_string(&f).unwrap();
        assert!(text.starts_with("# yaml-language-server: $schema="));
        assert_eq!(load_yaml::<MotorParams>(&f).unwrap(), m);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
