//! Signal bus: a frozen, preallocated vector of f64 values addressed by
//! [`SignalId`], plus a registry with metadata (CONVENTIONS §3).

use std::collections::HashMap;

/// Index of a signal on the bus.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub struct SignalId(pub u32);

/// What a signal represents.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SignalKind {
    State,
    Input,
    Output,
    Diagnostic,
}

/// Signal metadata.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SignalMeta {
    /// Dotted path, e.g. `motor.i_a`.
    pub path: String,
    /// SI unit, e.g. `A`.
    pub unit: String,
    pub description: String,
    /// Help registry id, e.g. `signal:motor.i_a`.
    pub help_id: String,
    pub kind: SignalKind,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SignalError {
    #[error("signal `{0}` is already registered")]
    Duplicate(String),
    #[error("signal path `{0}` is invalid (expected dotted snake_case, e.g. `motor.i_a`)")]
    InvalidPath(String),
    #[error("signal registry is frozen; register signals before building the engine")]
    Frozen,
    #[error("unknown signal `{0}`")]
    Unknown(String),
}

/// Registry of signals and the bus values.
#[derive(Debug, Clone, Default)]
pub struct SignalBus {
    meta: Vec<SignalMeta>,
    by_path: HashMap<String, SignalId>,
    values: Vec<f64>,
    frozen: bool,
}

fn valid_path(p: &str) -> bool {
    !p.is_empty()
        && p.split('.').all(|seg| {
            !seg.is_empty()
                && seg
                    .chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '_')
        })
        && p.contains('.')
}

impl SignalBus {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a signal (build time only). Its initial value is 0.
    pub fn register(
        &mut self,
        path: &str,
        unit: &str,
        description: &str,
        kind: SignalKind,
    ) -> Result<SignalId, SignalError> {
        if self.frozen {
            return Err(SignalError::Frozen);
        }
        if !valid_path(path) {
            return Err(SignalError::InvalidPath(path.into()));
        }
        if self.by_path.contains_key(path) {
            return Err(SignalError::Duplicate(path.into()));
        }
        let id = SignalId(self.meta.len() as u32);
        self.meta.push(SignalMeta {
            path: path.into(),
            unit: unit.into(),
            description: description.into(),
            help_id: format!("signal:{path}"),
            kind,
        });
        self.by_path.insert(path.into(), id);
        self.values.push(0.0);
        Ok(id)
    }

    /// Freeze the registry: no more registrations, values stay writable.
    pub fn freeze(&mut self) {
        self.frozen = true;
    }

    pub fn is_frozen(&self) -> bool {
        self.frozen
    }

    pub fn id(&self, path: &str) -> Result<SignalId, SignalError> {
        self.by_path
            .get(path)
            .copied()
            .ok_or_else(|| SignalError::Unknown(path.into()))
    }

    pub fn meta(&self, id: SignalId) -> &SignalMeta {
        &self.meta[id.0 as usize]
    }

    /// All signals in registration order.
    pub fn all(&self) -> &[SignalMeta] {
        &self.meta
    }

    #[inline]
    pub fn get(&self, id: SignalId) -> f64 {
        self.values[id.0 as usize]
    }

    #[inline]
    pub fn set(&mut self, id: SignalId, v: f64) {
        self.values[id.0 as usize] = v;
    }

    pub fn values(&self) -> &[f64] {
        &self.values
    }

    pub fn values_mut(&mut self) -> &mut [f64] {
        &mut self.values
    }

    pub fn len(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_lookup_set_get() {
        let mut bus = SignalBus::new();
        let ia = bus
            .register("motor.i_a", "A", "phase a current", SignalKind::State)
            .unwrap();
        let w = bus
            .register("motor.omega", "rad/s", "speed", SignalKind::State)
            .unwrap();
        assert_eq!(bus.id("motor.omega").unwrap(), w);
        assert_eq!(bus.meta(ia).help_id, "signal:motor.i_a");
        bus.set(ia, 1.25);
        assert_eq!(bus.get(ia), 1.25);
        assert_eq!(bus.get(w), 0.0);
    }

    #[test]
    fn duplicates_bad_paths_and_frozen_rejected() {
        let mut bus = SignalBus::new();
        bus.register("motor.i_a", "A", "", SignalKind::State)
            .unwrap();
        assert!(matches!(
            bus.register("motor.i_a", "A", "", SignalKind::State),
            Err(SignalError::Duplicate(_))
        ));
        assert!(matches!(
            bus.register("Motor.Ia", "A", "", SignalKind::State),
            Err(SignalError::InvalidPath(_))
        ));
        assert!(matches!(
            bus.register("nodot", "A", "", SignalKind::State),
            Err(SignalError::InvalidPath(_))
        ));
        bus.freeze();
        assert!(matches!(
            bus.register("motor.i_b", "A", "", SignalKind::State),
            Err(SignalError::Frozen)
        ));
        assert!(matches!(bus.id("x.y"), Err(SignalError::Unknown(_))));
    }
}
