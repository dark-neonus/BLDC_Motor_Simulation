//! Arm with a point mass under gravity (EQ-MECH-06). θ_L is the arm angle from straight
//! down; a mounting offset is an initial θ_L chosen at build.

use serde::{Deserialize, Serialize};

/// Standard gravity [m/s²] (mechanical.md assumption 3).
pub const G: f64 = 9.80665;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Arm {
    /// Rod length L_a [m].
    pub arm_length: f64,
    /// Rod mass m_a [kg] (uniform, pivoted at one end).
    pub arm_mass: f64,
    /// Point mass m [kg].
    pub mass: f64,
    /// Distance d of the point mass from the axis [m].
    pub distance: f64,
}

impl Arm {
    /// M_eff = m_a·L_a/2 + m·d [kg·m].
    pub fn m_eff(&self) -> f64 {
        self.arm_mass * self.arm_length / 2.0 + self.mass * self.distance
    }

    /// J_load = m_a·L_a²/3 + m·d² [kg·m²].
    pub fn inertia(&self) -> f64 {
        self.arm_mass * self.arm_length * self.arm_length / 3.0
            + self.mass * self.distance * self.distance
    }

    /// T_g = −g·M_eff·sin θ_L [N·m].
    pub fn torque(&self, theta_l: f64) -> f64 {
        -G * self.m_eff() * theta_l.sin()
    }

    /// U_g = g·M_eff·(1 − cos θ_L) [J].
    pub fn potential(&self, theta_l: f64) -> f64 {
        G * self.m_eff() * (1.0 - theta_l.cos())
    }

    pub const PARAMS: [(&'static str, &'static str); 4] = [
        ("load.arm_length", "m"),
        ("load.arm_mass", "kg"),
        ("load.mass", "kg"),
        ("load.distance", "m"),
    ];

    fn field(&mut self, path: &str) -> Option<&mut f64> {
        match path {
            "load.arm_length" => Some(&mut self.arm_length),
            "load.arm_mass" => Some(&mut self.arm_mass),
            "load.mass" => Some(&mut self.mass),
            "load.distance" => Some(&mut self.distance),
            _ => None,
        }
    }

    pub fn get(&self, path: &str) -> Option<f64> {
        let mut a = *self;
        a.field(path).copied()
    }

    /// Set a geometry/mass value (≥ 0); returns the old value.
    pub fn set(&mut self, path: &str, v: f64) -> Result<f64, String> {
        let f = self
            .field(path)
            .ok_or_else(|| format!("unknown parameter `{path}`"))?;
        if !(v.is_finite() && v >= 0.0) {
            return Err(format!("`{path}` must be ≥ 0, got {v}"));
        }
        Ok(std::mem::replace(f, v))
    }
}
