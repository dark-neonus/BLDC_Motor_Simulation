//! Snapshots (P03.T09): the full engine state — time, continuous state, bus,
//! block schedule and every block's / module's internal data — serialised with
//! MessagePack. Restoring and continuing is bit-identical (EQ-NUM-08).

use serde::{Deserialize, Serialize};

use super::engine::Engine;
use super::time::SimTime;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Snapshot {
    pub version: u32,
    pub time: SimTime,
    pub x: Vec<f64>,
    pub bus: Vec<f64>,
    pub next_fire: Vec<SimTime>,
    /// (block id, saved data), in engine order.
    pub blocks: Vec<(String, serde_json::Value)>,
    /// (module name, saved data), in plant order. Modules/blocks must include their
    /// live parameters in `save()` so a restore into a fresh engine is complete.
    pub modules: Vec<(String, serde_json::Value)>,
    pub energy_book: Option<crate::energy::EnergyBook>,
    pub dt_max: f64,
    pub fidelity: Option<super::fidelity::FidelityConfig>,
}

#[derive(Debug, thiserror::Error)]
pub enum SnapshotError {
    #[error("snapshot does not match this engine: {0}")]
    Mismatch(String),
    #[error("encode/decode failed: {0}")]
    Codec(String),
    #[error("restore failed in `{0}`: {1}")]
    Restore(String, String),
}

impl Snapshot {
    pub fn to_bytes(&self) -> Result<Vec<u8>, SnapshotError> {
        rmp_serde::to_vec_named(self).map_err(|e| SnapshotError::Codec(e.to_string()))
    }

    pub fn from_bytes(b: &[u8]) -> Result<Self, SnapshotError> {
        rmp_serde::from_slice(b).map_err(|e| SnapshotError::Codec(e.to_string()))
    }
}

impl Engine {
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            version: 1,
            time: self.time,
            x: self.x.clone(),
            bus: self.bus.values().to_vec(),
            next_fire: self.next_fire_times().to_vec(),
            blocks: self
                .blocks()
                .iter()
                .map(|b| (b.id().to_string(), b.save()))
                .collect(),
            modules: self
                .plant
                .modules()
                .iter()
                .map(|m| (m.name().to_string(), m.save()))
                .collect(),
            energy_book: self.energy_book().cloned(),
            dt_max: self.dt_max,
            fidelity: self.fidelity,
        }
    }

    /// Restore a snapshot taken from an engine with the same structure.
    pub fn restore(&mut self, s: &Snapshot) -> Result<(), SnapshotError> {
        if !(s.dt_max.is_finite() && s.dt_max > 0.0) {
            return Err(SnapshotError::Mismatch(format!(
                "invalid dt_max {}",
                s.dt_max
            )));
        }
        // Block/module data is only validated by their own restore(); keep a backup and
        // roll back so a failure never leaves a half-restored engine.
        let backup = self.snapshot();
        match self.restore_unchecked(s) {
            Ok(()) => Ok(()),
            Err(e) => {
                let _ = self.restore_unchecked(&backup);
                Err(e)
            }
        }
    }

    fn restore_unchecked(&mut self, s: &Snapshot) -> Result<(), SnapshotError> {
        // Validate everything before mutating anything (imported files may not match).
        let ids: Vec<&str> = self.blocks().iter().map(|b| b.id()).collect();
        let names: Vec<&str> = self.plant.modules().iter().map(|m| m.name()).collect();
        if s.x.len() != self.x.len()
            || s.bus.len() != self.bus.len()
            || s.next_fire.len() != ids.len()
            || s.blocks.len() != ids.len()
            || s.blocks.iter().zip(&ids).any(|((a, _), b)| a != b)
            || s.modules.len() != names.len()
            || s.modules.iter().zip(&names).any(|((a, _), b)| a != b)
        {
            return Err(SnapshotError::Mismatch(
                "state size or block list differs".into(),
            ));
        }
        for (i, (id, data)) in s.blocks.iter().enumerate() {
            self.block_mut(i)
                .restore(data)
                .map_err(|e| SnapshotError::Restore(id.clone(), e))?;
        }
        for (i, (name, data)) in s.modules.iter().enumerate() {
            self.plant
                .module_restore(i, data)
                .map_err(|e| SnapshotError::Restore(name.clone(), e))?;
        }
        self.time = s.time;
        self.x.copy_from_slice(&s.x);
        self.bus.values_mut().copy_from_slice(&s.bus);
        self.set_next_fire_times(&s.next_fire);
        self.clear_queues();
        if let Some(b) = &s.energy_book {
            self.set_energy_book(b);
        }
        self.dt_max = s.dt_max;
        self.fidelity = s.fidelity;
        Ok(())
    }
}
