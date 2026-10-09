//! Energy-balance accounting (P03.T16, EQ-ENER-01..03). Power terms are integrated
//! as extra plant states (same integrator → consistent accuracy); stored energies
//! are summed; discrete jumps are booked as external energy.

use serde::{Deserialize, Serialize};

use crate::engine::plant::Plant;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

/// How a power term enters the balance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PowerKind {
    /// Energy entering the system (e.g. PSU output) [W].
    Input,
    /// Dissipated (≥ 0) [W].
    Loss,
    /// External work done on the system (constant loads, disturbances) [W].
    External,
}

/// Tolerances per tier (EQ-ENER-03).
pub const R_TOL_STANDARD: f64 = 1e-3;
pub const R_TOL_DETAILED: f64 = 5e-3;
const E_FLOOR: f64 = 1e-6;

/// Energy signals on the bus.
#[derive(Debug, Clone)]
pub struct EnergySignals {
    pub e_in: SignalId,
    pub e_loss: SignalId,
    pub e_ext: SignalId,
    pub e_stored: SignalId,
    pub residual: SignalId,
    pub ok: SignalId,
    /// `energy.loss.<module>` per module with loss terms: (module index, signal).
    pub per_module_loss: Vec<(usize, SignalId)>,
}

impl EnergySignals {
    pub fn register(bus: &mut SignalBus, plant: &Plant) -> Result<Self, SignalError> {
        let mut per_module_loss = Vec::new();
        for (mi, m) in plant.modules().iter().enumerate() {
            if m.power_terms().iter().any(|(_, k)| *k == PowerKind::Loss) {
                let path = format!("energy.loss.{}", m.name());
                per_module_loss.push((
                    mi,
                    bus.register(
                        &path,
                        "J",
                        "energy dissipated in this module",
                        SignalKind::Diagnostic,
                    )?,
                ));
            }
        }
        let mut r = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Diagnostic);
        Ok(Self {
            e_in: r("energy.in", "J", "energy delivered by sources")?,
            e_loss: r("energy.loss", "J", "energy dissipated")?,
            e_ext: r("energy.external", "J", "external work and discrete jumps")?,
            e_stored: r(
                "energy.stored",
                "J",
                "stored energy minus its initial value",
            )?,
            residual: r("energy.residual", "-", "normalised energy-balance residual")?,
            ok: r("energy.ok", "-", "1 when |residual| < tolerance")?,
            per_module_loss,
        })
    }
}

/// Running totals not held in the ODE state.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct EnergyBook {
    pub e_st0: f64,
    /// Discrete external energy (param changes, resets).
    pub e_jump: f64,
    pub tol: f64,
}

impl EnergyBook {
    /// Update the energy signals from the current state. EQ-ENER-03.
    pub fn update(
        &mut self,
        plant: &mut Plant,
        x: &[f64],
        bus: &mut SignalBus,
        sig: &EnergySignals,
    ) {
        self.e_jump += plant.take_external_energy();
        let (mut e_in, mut e_loss, mut e_ext) = (0.0, 0.0, self.e_jump);
        let eo = plant.energy_offset();
        let terms = plant.energy_terms();
        for &(_, id) in &sig.per_module_loss {
            bus.set(id, 0.0);
        }
        for (j, (mi, kind)) in terms.iter().enumerate() {
            let e = x[eo + j];
            match kind {
                PowerKind::Input => e_in += e,
                PowerKind::Loss => {
                    e_loss += e;
                    if let Some(&(_, id)) = sig.per_module_loss.iter().find(|(m, _)| m == mi) {
                        bus.set(id, bus.get(id) + e);
                    }
                }
                PowerKind::External => e_ext += e,
            }
        }
        let thr = if terms.is_empty() {
            0.0
        } else {
            x[eo + terms.len()]
        };
        let e_st = plant.stored_energy(x, bus);
        let d_st = e_st - self.e_st0;
        let denom = thr.max(self.e_st0.abs()).max(e_st.abs()).max(E_FLOOR);
        let r = (e_in + e_ext - e_loss - d_st) / denom;
        bus.set(sig.e_in, e_in);
        bus.set(sig.e_loss, e_loss);
        bus.set(sig.e_ext, e_ext);
        bus.set(sig.e_stored, d_st);
        bus.set(sig.residual, r);
        bus.set(sig.ok, if r.abs() < self.tol { 1.0 } else { 0.0 });
    }
}
