//! Minimal rigid rotor (P05.T10, EQ-MECH-02 rigid case without gearbox or load model):
//! J·dω/dt = Σ T_internal + Σ T_external − B·ω, dθ/dt = ω. P06.T01 extends it.
//!
//! Torques arrive as bus signals. *Internal* torques (electromagnetic, cogging, iron
//! drag) move energy between modules and need no power term here: the module that
//! produces them accounts for its side. *External* torques (loads, disturbances) do
//! work on the system and are reported as `External` power.

use crate::energy::PowerKind;
use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

#[derive(Debug, Clone, Copy)]
pub struct RotorParams {
    /// Rotor inertia [kg·m²] (`motor.mechanical.j_rotor`).
    pub j: f64,
    /// Reflected gearbox/load inertia added to the shaft [kg·m²] (fixed here; P06 models
    /// the load itself).
    pub j_load: f64,
    /// Viscous friction [N·m·s/rad].
    pub b: f64,
    /// Pole pairs (θe = p·θm).
    pub pole_pairs: f64,
    pub theta0: f64,
    pub omega0: f64,
}

pub struct RotorRigid {
    p: RotorParams,
    locked: bool,
    internal: Vec<SignalId>,
    external: Vec<SignalId>,
    theta: SignalId,
    omega: SignalId,
    theta_e: SignalId,
    omega_e: SignalId,
    /// Energy injected by parameter changes since the last `take_external_energy`.
    jump: f64,
    omega_now: f64,
}

impl RotorRigid {
    /// Registers `motor.theta`, `motor.omega`, `motor.theta_e`, `motor.omega_e`.
    /// `internal` / `external` are torque signals [N·m] that must already be registered.
    pub fn new(
        p: RotorParams,
        bus: &mut SignalBus,
        internal: Vec<SignalId>,
        external: Vec<SignalId>,
    ) -> Result<Self, SignalError> {
        let mut r =
            |path: &str, unit: &str, d: &str| bus.register(path, unit, d, SignalKind::State);
        Ok(Self {
            theta: r("motor.theta", "rad", "mechanical angle")?,
            omega: r("motor.omega", "rad/s", "mechanical speed")?,
            theta_e: r("motor.theta_e", "rad", "electrical angle")?,
            omega_e: r("motor.omega_e", "rad/s", "electrical speed")?,
            p,
            locked: false,
            internal,
            external,
            jump: 0.0,
            omega_now: p.omega0,
        })
    }

    /// Add an internal torque input registered after the rotor (the motor's torque
    /// needs the rotor's angle signals first).
    pub fn with_internal(mut self, id: SignalId) -> Self {
        self.internal.push(id);
        self
    }

    /// Start with the shaft held (ω = 0); `motor.mechanical.locked` releases it.
    pub fn with_locked(mut self, locked: bool) -> Self {
        self.locked = locked;
        self
    }

    fn torque(&self, bus: &SignalBus, w: f64) -> f64 {
        let sum: f64 = self
            .internal
            .iter()
            .chain(&self.external)
            .map(|&id| bus.get(id))
            .sum();
        sum - self.p.b * w
    }
}

impl PlantModule for RotorRigid {
    fn name(&self) -> &str {
        "motor.mechanical"
    }
    fn n_states(&self) -> usize {
        2
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![
            ("theta".into(), "rad".into()),
            ("omega".into(), "rad/s".into()),
        ]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = self.p.theta0;
        x[1] = if self.locked { 0.0 } else { self.p.omega0 };
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let (th, w) = (x[off], x[off + 1]);
        self.omega_now = w;
        bus.set(self.theta, th);
        bus.set(self.omega, w);
        bus.set(self.theta_e, self.p.pole_pairs * th);
        bus.set(self.omega_e, self.p.pole_pairs * w);
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        if self.locked {
            dx[0] = 0.0;
            dx[1] = 0.0;
            return;
        }
        let w = x[off + 1];
        dx[0] = w;
        dx[1] = self.torque(bus, w) / (self.p.j + self.p.j_load);
    }
    fn params(&self) -> Vec<(String, String)> {
        vec![
            ("j_rotor".into(), "kg*m^2".into()),
            ("friction.viscous".into(), "N*m*s/rad".into()),
            ("locked".into(), "-".into()),
        ]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match name {
            "j_rotor" => Some(self.p.j),
            "friction.viscous" => Some(self.p.b),
            "locked" => Some(f64::from(u8::from(self.locked))),
            _ => None,
        }
    }
    fn set_param(&mut self, name: &str, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        match name {
            "j_rotor" => {
                if !(v.is_finite() && v > 0.0) {
                    return Err(format!("inertia must be positive, got {v}"));
                }
                // D-007: keep ω; the kinetic-energy change is an external jump.
                let w = x_own[1];
                self.jump += 0.5 * (v - self.p.j) * w * w;
                Ok(std::mem::replace(&mut self.p.j, v))
            }
            "friction.viscous" => {
                if !(v.is_finite() && v >= 0.0) {
                    return Err(format!("viscous friction must be ≥ 0, got {v}"));
                }
                Ok(std::mem::replace(&mut self.p.b, v))
            }
            "locked" => {
                let old = f64::from(u8::from(self.locked));
                self.locked = v != 0.0;
                if self.locked {
                    // Stopping the shaft removes its kinetic energy.
                    self.jump -= 0.5 * (self.p.j + self.p.j_load) * x_own[1] * x_own[1];
                    x_own[1] = 0.0;
                }
                Ok(old)
            }
            _ => Err(format!("unknown parameter `{name}`")),
        }
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![
            ("friction".into(), PowerKind::Loss),
            ("load".into(), PowerKind::External),
        ]
    }
    fn powers(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, p: &mut [f64]) {
        let w = if self.locked { 0.0 } else { x[off + 1] };
        p[0] = self.p.b * w * w;
        p[1] = self.external.iter().map(|&id| bus.get(id)).sum::<f64>() * w;
    }
    fn stored_energy(&self, x: &[f64], off: usize, _bus: &SignalBus) -> f64 {
        0.5 * (self.p.j + self.p.j_load) * x[off + 1] * x[off + 1]
    }
    fn take_external_energy(&mut self) -> f64 {
        std::mem::take(&mut self.jump)
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "j": self.p.j, "b": self.p.b, "locked": self.locked })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        let (j, b) = (f("j")?, f("b")?);
        self.locked = v
            .get("locked")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `locked`")?;
        (self.p.j, self.p.b) = (j, b);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::engine::Engine;
    use crate::engine::time::SimTime;

    /// Writes a constant torque on `load.torque_ext`.
    struct Push(SignalId, f64);
    impl PlantModule for Push {
        fn name(&self) -> &str {
            "load"
        }
        fn n_states(&self) -> usize {
            0
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![]
        }
        fn init(&self, _: &mut [f64]) {}
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
            bus.set(self.0, self.1);
        }
        fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
    }

    fn engine(t_ext: f64, p: RotorParams) -> Engine {
        let mut bus = SignalBus::new();
        let tq = bus
            .register("load.torque_ext", "N*m", "test torque", SignalKind::Output)
            .unwrap();
        let rotor = RotorRigid::new(p, &mut bus, vec![], vec![tq]).unwrap();
        let mut plant = crate::engine::plant::Plant::new();
        plant.add(Box::new(Push(tq, t_ext)));
        plant.add(Box::new(rotor));
        Engine::new(plant, bus, vec![], 1e-4)
    }

    fn sig(e: &Engine, p: &str) -> f64 {
        e.bus.get(e.bus.id(p).unwrap())
    }

    #[test]
    fn constant_torque_accelerates_linearly() {
        let p = RotorParams {
            j: 2e-4,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 0.0,
        };
        let mut e = engine(0.01, p);
        e.step_until(SimTime::from_secs_f64(0.5)).unwrap();
        // ω = T·t/J = 0.01·0.5/2e-4 = 25 rad/s; θ = ½·T·t²/J. RK4 is exact for polynomials
        // of this degree, so only rounding remains (rtol 1e-12).
        assert!((sig(&e, "motor.omega") / 25.0 - 1.0).abs() < 1e-12);
        assert!((sig(&e, "motor.theta") / 6.25 - 1.0).abs() < 1e-12);
        assert!((sig(&e, "motor.theta_e") / (7.0 * 6.25) - 1.0).abs() < 1e-12);
        assert!(
            sig(&e, "energy.residual").abs() < 1e-9,
            "external work = kinetic energy"
        );
    }

    #[test]
    fn viscous_spin_down_is_exponential() {
        let p = RotorParams {
            j: 2e-4,
            j_load: 0.0,
            b: 1e-3,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 100.0,
        };
        let mut e = engine(0.0, p);
        e.step_until(SimTime::from_secs_f64(0.4)).unwrap();
        // τ = J/B = 0.2 s → ω(0.4) = 100·e^-2. RK4 at dt = 1e-4 ≪ τ: rtol 1e-9.
        let w = sig(&e, "motor.omega");
        assert!((w / (100.0 * (-2.0f64).exp()) - 1.0).abs() < 1e-9, "{w}");
        assert!(sig(&e, "energy.residual").abs() < 1e-9);
        assert!(sig(&e, "energy.loss.friction") > 0.0);
    }

    #[test]
    fn inertia_change_keeps_speed_and_books_the_energy() {
        let p = RotorParams {
            j: 2e-4,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 50.0,
        };
        let mut e = engine(0.0, p);
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        e.queue(crate::engine::commands::EngineCommand::SetParam {
            path: "motor.mechanical.j_rotor".into(),
            value: 4e-4,
            source: crate::engine::commands::ChangeSource::Internal,
        });
        e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        assert_eq!(sig(&e, "motor.omega"), 50.0);
        assert!(sig(&e, "energy.residual").abs() < 1e-9);
    }
}
