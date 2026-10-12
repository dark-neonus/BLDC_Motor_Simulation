//! Stick–slip friction, hybrid Karnopp-type (EQ-MECH-05). In the rigid drivetrain the
//! motor and load elements are lumped on ω_m (mechanical.md assumption 4).
//!
//! Modes: **slip** in direction `dir` (T_f = −dir·T_c − b·ω; ω crossing zero is a state
//! event that sticks or reverses), **stick** (ω ≡ 0 while |T_a| ≤ T_s; |T_a| crossing T_s
//! is a state event that starts slipping in the direction of T_a).

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Friction {
    /// Static (breakaway) torque T_s [N·m].
    pub t_s: f64,
    /// Coulomb torque T_c ≤ T_s [N·m].
    pub t_c: f64,
    /// Viscous coefficient b [N·m·s/rad].
    pub b: f64,
}

impl Friction {
    /// The rigid-drivetrain lump on the motor shaft: T_s,m + T_s,L/N, T_c,m + T_c,L/N,
    /// b_m + b_L/N².
    pub fn lumped(motor: &Friction, load: &Friction, n: f64) -> Friction {
        Friction {
            t_s: motor.t_s + load.t_s / n,
            t_c: motor.t_c + load.t_c / n,
            b: motor.b + load.b / (n * n),
        }
    }

    /// True when stick–slip logic is needed (any static or Coulomb part).
    pub fn is_hybrid(&self) -> bool {
        self.t_s > 0.0 || self.t_c > 0.0
    }

    /// Validate a value for `static_torque`, `coulomb` or `viscous` (≥ 0, finite).
    pub fn check(name: &str, v: f64) -> Result<(), String> {
        if v.is_finite() && v >= 0.0 {
            Ok(())
        } else {
            Err(format!("`{name}` must be ≥ 0, got {v}"))
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Mode {
    /// Slipping in direction `dir` (±1).
    Slip {
        dir: f64,
    },
    Stick,
}

/// Friction torque and acceleration-relevant result for one evaluation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Eval {
    /// Friction torque T_f on the shaft [N·m].
    pub torque: f64,
    /// True when the shaft is held (ω ≡ 0, no acceleration).
    pub held: bool,
}

/// Friction for applied torque `t_a` (everything but this friction) at speed `w`.
/// In stick mode with |T_a| > T_s (before the breakaway event is handled, or when a
/// step starts past it) the shaft slips in the direction of T_a.
pub fn eval(f: &Friction, mode: Mode, t_a: f64, w: f64) -> Eval {
    let slip = |dir: f64| Eval {
        torque: -dir * f.t_c - f.b * w,
        held: false,
    };
    match mode {
        Mode::Slip { dir } => slip(dir),
        Mode::Stick if t_a.abs() <= f.t_s => Eval {
            torque: -t_a,
            held: true,
        },
        Mode::Stick => slip(t_a.signum()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lumping_and_modes() {
        let m = Friction {
            t_s: 0.02,
            t_c: 0.01,
            b: 1e-4,
        };
        let l = Friction {
            t_s: 0.9,
            t_c: 0.45,
            b: 0.09,
        };
        let f = Friction::lumped(&m, &l, 9.0);
        assert!((f.t_s - 0.12).abs() < 1e-15 && (f.t_c - 0.06).abs() < 1e-15);
        assert!((f.b - (1e-4 + 0.09 / 81.0)).abs() < 1e-15);
        // Held below breakaway, friction balances exactly.
        let e = eval(&f, Mode::Stick, 0.1, 0.0);
        assert!(e.held && e.torque == -0.1);
        // Past breakaway it slips with the applied torque.
        let e = eval(&f, Mode::Stick, -0.2, 0.0);
        assert!(!e.held && e.torque == f.t_c);
        assert_eq!(
            eval(&f, Mode::Slip { dir: 1.0 }, 0.0, 2.0).torque,
            -f.t_c - f.b * 2.0
        );
    }
}
