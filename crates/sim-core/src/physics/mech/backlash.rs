//! Gear backlash: dead-zone, push-only contact between the motor and load sides
//! (EQ-MECH-04). δ = θm/N − θ_L is the load-side deflection.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Backlash {
    /// Total backlash b_bl at the output [rad].
    pub width: f64,
    /// Contact stiffness k_c [N·m/rad].
    pub k: f64,
    /// Contact damping d_c [N·m·s/rad].
    pub d: f64,
}

/// Default contact stiffness when the gearbox gives none: 1 N·m/arcmin (D-016).
pub const DEFAULT_STIFFNESS: f64 = 10800.0 / std::f64::consts::PI;
/// Default damping ratio ζ for d_c = 2ζ√(k_c·J_red) (EQ-MECH-04).
pub const DEFAULT_ZETA: f64 = 0.1;

impl Backlash {
    /// Penetration past the dead zone, max(0, |δ| − b/2).
    fn pen(&self, delta: f64) -> f64 {
        (delta.abs() - 0.5 * self.width).max(0.0)
    }

    /// Contact torque τ_c for deflection δ and its rate δ̇ (push-only).
    pub fn torque(&self, delta: f64, ddelta: f64) -> f64 {
        let e = self.pen(delta);
        if e <= 0.0 {
            return 0.0;
        }
        let s = delta.signum();
        s * (self.k * e + self.d * s * ddelta).max(0.0)
    }

    /// Spring energy ½·k_c·(|δ| − b/2)² in contact [J].
    pub fn energy(&self, delta: f64) -> f64 {
        let e = self.pen(delta);
        0.5 * self.k * e * e
    }

    /// d/dt of the spring energy.
    pub fn energy_rate(&self, delta: f64, ddelta: f64) -> f64 {
        self.k * self.pen(delta) * delta.signum() * ddelta
    }

    /// Damping from a ratio ζ and the reduced inertia (EQ-MECH-04).
    pub fn damping_for(k: f64, zeta: f64, j_red: f64) -> f64 {
        2.0 * zeta * (k * j_red).sqrt()
    }

    /// Step limit 1/(20·√(k_c/J_red)) (EQ-NUM step table).
    pub fn dt_limit(&self, j_red: f64) -> f64 {
        1.0 / (20.0 * (self.k / j_red).sqrt())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dead_zone_push_only_and_dissipative() {
        let b = Backlash {
            width: 0.01,
            k: 1000.0,
            d: 2.0,
        };
        assert_eq!(b.torque(0.004, 5.0), 0.0);
        assert!((b.torque(0.006, 0.0) - 1.0).abs() < 1e-12);
        assert!((b.torque(-0.006, 0.0) + 1.0).abs() < 1e-12);
        // Separating fast: damping would pull, the contact releases instead.
        assert_eq!(b.torque(0.006, -1.0), 0.0);
        // Absorbed power τ·δ̇ minus spring storage is never negative.
        for (dl, dd) in [(0.006, 0.3), (0.006, -0.3), (-0.007, 0.2), (-0.007, -2.0)] {
            assert!(b.torque(dl, dd) * dd - b.energy_rate(dl, dd) >= -1e-15);
        }
    }
}
