//! Iron loss as a drag torque (P05.T05, EQ-MOT-09):
//! T_fe = −(k_hy·p·tanh(ω_m/ω_ε) + k_ed·p²·ω_m), loss power −T_fe·ω_m ≥ 0.
//! The torque is an internal rotor input; the loss is booked here as `iron` (its heat
//! goes to the stator node in P06).

use crate::energy::PowerKind;
use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

/// Regularisation speed for sign(ω) (EQ-MOT-09).
pub const OMEGA_EPS: f64 = 0.01;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IronLossParams {
    /// Hysteresis coefficient [W·s/rad] (per electrical rad/s).
    pub k_hy: f64,
    /// Eddy-current coefficient [W·s²/rad²].
    pub k_ed: f64,
    pub pole_pairs: f64,
}

pub struct IronLoss {
    p: IronLossParams,
    omega: SignalId,
    torque: SignalId,
    p_fe: SignalId,
}

impl IronLoss {
    /// Registers `motor.torque_fe` and `motor.p_fe`; reads the mechanical speed.
    pub fn new(
        p: IronLossParams,
        omega: SignalId,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        Ok(Self {
            p,
            omega,
            torque: bus.register(
                "motor.torque_fe",
                "N*m",
                "iron-loss drag torque",
                SignalKind::Output,
            )?,
            p_fe: bus.register("motor.p_fe", "W", "iron loss", SignalKind::Output)?,
        })
    }

    pub fn torque_id(&self) -> SignalId {
        self.torque
    }

    pub fn torque_at(&self, w: f64) -> f64 {
        let p = self.p.pole_pairs;
        -(self.p.k_hy * p * (w / OMEGA_EPS).tanh() + self.p.k_ed * p * p * w)
    }
}

impl PlantModule for IronLoss {
    fn name(&self) -> &str {
        "motor.magnetic"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _t: f64, _x: &[f64], _off: usize, bus: &mut SignalBus) {
        let w = bus.get(self.omega);
        let t = self.torque_at(w);
        bus.set(self.torque, t);
        bus.set(self.p_fe, -t * w);
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
    fn params(&self) -> Vec<(String, String)> {
        vec![
            ("k_hy".into(), "W*s/rad".into()),
            ("k_ed".into(), "W*s^2/rad^2".into()),
        ]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match name {
            "k_hy" => Some(self.p.k_hy),
            "k_ed" => Some(self.p.k_ed),
            _ => None,
        }
    }
    fn set_param(&mut self, name: &str, v: f64, _: &mut [f64]) -> Result<f64, String> {
        if !(v.is_finite() && v >= 0.0) {
            return Err(format!("`{name}` must be ≥ 0, got {v}"));
        }
        match name {
            "k_hy" => Ok(std::mem::replace(&mut self.p.k_hy, v)),
            "k_ed" => Ok(std::mem::replace(&mut self.p.k_ed, v)),
            _ => Err(format!("unknown parameter `{name}`")),
        }
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![("iron".into(), PowerKind::Loss)]
    }
    fn powers(&self, _t: f64, _x: &[f64], _off: usize, bus: &SignalBus, p: &mut [f64]) {
        let w = bus.get(self.omega);
        p[0] = -self.torque_at(w) * w;
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "k_hy": self.p.k_hy, "k_ed": self.p.k_ed })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        (self.p.k_hy, self.p.k_ed) = (f("k_hy")?, f("k_ed")?);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::engine::Engine;
    use crate::engine::plant::Plant;
    use crate::engine::time::SimTime;
    use crate::physics::mech::rotor::{RotorParams, RotorRigid};

    #[test]
    fn no_load_spin_down_matches_the_drag_law() {
        let (j, p, k_hy, k_ed, w0) = (2e-4, 7.0, 2e-4, 1e-6, 200.0);
        let mut bus = SignalBus::new();
        let rp = RotorParams {
            j,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: p,
            theta0: 0.0,
            omega0: w0,
        };
        let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![]).unwrap();
        let fe = IronLoss::new(
            IronLossParams {
                k_hy,
                k_ed,
                pole_pairs: p,
            },
            bus.id("motor.omega").unwrap(),
            &mut bus,
        )
        .unwrap();
        let rotor = rotor.with_internal(fe.torque_id());
        let mut plant = Plant::new();
        plant.add(Box::new(rotor));
        plant.add(Box::new(fe));
        let mut e = Engine::new(plant, bus, vec![], 1e-4);
        // For ω ≫ ω_ε, tanh = 1: J·ω' = −(a + b·ω), a = k_hy·p, b = k_ed·p²
        // → ω(t) = (ω₀ + a/b)·e^(−b·t/J) − a/b.
        let (a, b) = (k_hy * p, k_ed * p * p);
        let sig = |e: &Engine, n: &str| e.bus.get(e.bus.id(n).unwrap());
        for t in [0.1, 0.5, 1.0, 1.5] {
            e.step_until(SimTime::from_secs_f64(t)).unwrap();
            let want = (w0 + a / b) * (-b * t / j).exp() - a / b;
            assert!(want > 1.0, "test stays in the ω ≫ ω_ε range");
            // RK4 at dt = 1e-4 ≪ J/b = 0.08 s: rtol 1e-9.
            let w = sig(&e, "motor.omega");
            assert!((w / want - 1.0).abs() < 1e-9, "t = {t}: {w} vs {want}");
        }
        assert!(sig(&e, "energy.residual").abs() < 1e-9);
        assert!(sig(&e, "energy.loss.iron") > 0.0);
        assert!(sig(&e, "motor.p_fe") > 0.0);
    }

    #[test]
    fn drag_is_smooth_and_dissipative_through_zero() {
        let mut bus = SignalBus::new();
        let w = bus
            .register("motor.omega", "rad/s", "", SignalKind::State)
            .unwrap();
        let fe = IronLoss::new(
            IronLossParams {
                k_hy: 1e-3,
                k_ed: 1e-6,
                pole_pairs: 7.0,
            },
            w,
            &mut bus,
        )
        .unwrap();
        for x in [-1.0, -1e-3, 0.0, 1e-3, 1.0] {
            assert!(fe.torque_at(x) * x <= 0.0, "T_fe·ω ≤ 0 at {x}");
        }
        assert_eq!(fe.torque_at(0.0), 0.0);
    }
}
