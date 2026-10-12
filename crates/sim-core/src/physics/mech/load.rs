//! Loads on the output shaft (EQ-MECH-06/07). The drivetrain (`rotor.rs`, D-015) owns
//! the states; a load contributes inertia, torques and potential energy at the load
//! angle θ_L and speed ω_L.

use serde::{Deserialize, Serialize};

use super::load_arm::Arm;

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
    /// Arm + point mass under gravity (EQ-MECH-06).
    Arm(Arm),
}

impl Load {
    /// J_load [kg·m²].
    pub fn inertia(&self) -> f64 {
        match self {
            Load::None => 0.0,
            Load::Flywheel { j } => *j,
            Load::Arm(a) => a.inertia(),
        }
    }

    pub fn torques(&self, theta_l: f64, _omega_l: f64) -> LoadTorques {
        match self {
            Load::Arm(a) => LoadTorques {
                gravity: a.torque(theta_l),
                ..Default::default()
            },
            _ => LoadTorques::default(),
        }
    }

    /// Potential energy U(θ_L) [J].
    pub fn potential(&self, theta_l: f64) -> f64 {
        match self {
            Load::Arm(a) => a.potential(theta_l),
            _ => 0.0,
        }
    }

    /// Live parameters under `load.` with their units.
    pub fn params(&self) -> Vec<(String, String)> {
        match self {
            Load::None => vec![],
            Load::Flywheel { .. } => vec![("load.inertia".into(), "kg*m^2".into())],
            Load::Arm(_) => Arm::PARAMS
                .iter()
                .map(|(p, u)| ((*p).into(), (*u).into()))
                .collect(),
        }
    }

    pub fn get(&self, path: &str) -> Option<f64> {
        match (self, path) {
            (Load::Flywheel { j }, "load.inertia") => Some(*j),
            (Load::Arm(a), p) => a.get(p),
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
            (Load::Arm(a), p) => a.set(p, v),
            _ => Err(format!("unknown parameter `{path}`")),
        }
    }
}
