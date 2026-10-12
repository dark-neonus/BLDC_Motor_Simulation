//! Loads on the output shaft (EQ-MECH-06/07). The drivetrain (`rotor.rs`, D-015) owns
//! the states; a load contributes inertia, torques and potential energy at the load
//! angle θ_L and speed ω_L.

use serde::{Deserialize, Serialize};

/// Load-side torques [N·m], split by where their energy goes.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct LoadTorques {
    /// Conservative (gravity): its work is −dU/dt, booked through `potential`.
    pub gravity: f64,
    /// Dissipative (brake, viscous): its power −T·ω_L is a loss.
    pub loss: f64,
    /// Work done by the outside (constant torque, disturbance): external energy.
    pub ext: f64,
}

impl LoadTorques {
    pub fn total(&self) -> f64 {
        self.gravity + self.loss + self.ext
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub enum Load {
    #[default]
    None,
    /// A pure inertia [kg·m²].
    Flywheel { j: f64 },
}

impl Load {
    /// J_load [kg·m²].
    pub fn inertia(&self) -> f64 {
        match self {
            Load::None => 0.0,
            Load::Flywheel { j } => *j,
        }
    }

    pub fn torques(&self, _theta_l: f64, _omega_l: f64) -> LoadTorques {
        LoadTorques::default()
    }

    /// Potential energy U(θ_L) [J].
    pub fn potential(&self, _theta_l: f64) -> f64 {
        0.0
    }

    /// Live parameters under `load.` with their units.
    pub fn params(&self) -> Vec<(String, String)> {
        match self {
            Load::None => vec![],
            Load::Flywheel { .. } => vec![("load.inertia".into(), "kg*m^2".into())],
        }
    }

    pub fn get(&self, path: &str) -> Option<f64> {
        match (self, path) {
            (Load::Flywheel { j }, "load.inertia") => Some(*j),
            _ => None,
        }
    }

    /// Set a load parameter on a copy; returns the old value. The drivetrain applies
    /// the inertia/energy rule (EQ-MECH-09) around it.
    pub fn set(&mut self, path: &str, v: f64) -> Result<f64, String> {
        match (self, path) {
            (Load::Flywheel { j }, "load.inertia") => {
                if !(v.is_finite() && v >= 0.0) {
                    return Err(format!("inertia must be ≥ 0, got {v}"));
                }
                Ok(std::mem::replace(j, v))
            }
            _ => Err(format!("unknown parameter `{path}`")),
        }
    }
}
