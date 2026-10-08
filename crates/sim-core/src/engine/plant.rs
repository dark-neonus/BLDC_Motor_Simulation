//! Continuous plant: modules each own a slice of one global state vector.
//!
//! Every derivative evaluation (each RK stage) runs two passes in a fixed module
//! order (P03 design note, EQ-NUM-02):
//! 1. `outputs`: each module writes algebraic outputs computed from the FULL state
//!    to the bus (e.g. the rotor writes θe, ω; the motor writes currents, torque);
//! 2. `derivatives`: each module computes its slice of dx from the full state + bus.
//!
//! This couples modules consistently inside an integration step.

use super::signals::SignalBus;

/// A continuous subsystem.
pub trait PlantModule: Send {
    fn name(&self) -> &str;
    /// Number of states owned by this module.
    fn n_states(&self) -> usize;
    /// State names and units, for debugging and snapshots (len == n_states).
    fn state_names(&self) -> Vec<(String, String)>;
    /// Write initial values into this module's slice.
    fn init(&self, x: &mut [f64]);
    /// Pass 1: write algebraic outputs to the bus from the full state `x`.
    /// `off` is this module's offset in `x`.
    fn outputs(&mut self, t: f64, x: &[f64], off: usize, bus: &mut SignalBus);
    /// Pass 2: write d(own states)/dt into `dx` (this module's slice only).
    fn derivatives(&self, t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]);
    /// Live-settable parameters: `(name, unit)`; full path is `<module name>.<name>`.
    fn params(&self) -> Vec<(String, String)> {
        Vec::new()
    }
    fn get_param(&self, _name: &str) -> Option<f64> {
        None
    }
    /// Set a parameter; returns the old value. May adjust own states in `x_own`
    /// (e.g. the D-007 rule when inertia changes).
    fn set_param(&mut self, name: &str, _value: f64, _x_own: &mut [f64]) -> Result<f64, String> {
        Err(format!("unknown parameter `{name}`"))
    }
    /// Internal (non-state) data for snapshots, e.g. RNG streams, integrators, modes.
    fn save(&self) -> serde_json::Value {
        serde_json::Value::Null
    }
    fn restore(&mut self, _state: &serde_json::Value) -> Result<(), String> {
        Ok(())
    }
}

/// Composition of plant modules with a global state layout.
#[derive(Default)]
pub struct Plant {
    modules: Vec<Box<dyn PlantModule>>,
    offsets: Vec<usize>,
    n: usize,
}

impl Plant {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a module; returns its state offset. Order = evaluation order.
    pub fn add(&mut self, m: Box<dyn PlantModule>) -> usize {
        let off = self.n;
        self.n += m.n_states();
        self.offsets.push(off);
        self.modules.push(m);
        off
    }

    pub fn n_states(&self) -> usize {
        self.n
    }

    pub fn modules(&self) -> &[Box<dyn PlantModule>] {
        &self.modules
    }

    pub(crate) fn module_restore(
        &mut self,
        i: usize,
        data: &serde_json::Value,
    ) -> Result<(), String> {
        self.modules
            .get_mut(i)
            .ok_or_else(|| format!("no module {i}"))?
            .restore(data)
    }

    /// Mutable module `i` and its state slice.
    pub fn module_mut<'a>(
        &'a mut self,
        i: usize,
        x: &'a mut [f64],
    ) -> (&'a mut dyn PlantModule, &'a mut [f64]) {
        let off = self.offsets[i];
        let n = self.modules[i].n_states();
        (self.modules[i].as_mut(), &mut x[off..off + n])
    }

    /// Initial global state vector.
    pub fn initial_state(&self) -> Vec<f64> {
        let mut x = vec![0.0; self.n];
        for (m, &off) in self.modules.iter().zip(&self.offsets) {
            m.init(&mut x[off..off + m.n_states()]);
        }
        x
    }

    /// `(module, state, unit)` for every state, in vector order.
    pub fn layout(&self) -> Vec<(String, String, String)> {
        self.modules
            .iter()
            .flat_map(|m| {
                let name = m.name().to_string();
                m.state_names()
                    .into_iter()
                    .map(move |(s, u)| (name.clone(), s, u))
            })
            .collect()
    }

    /// Pass 1 only (used after an accepted step so the bus matches the state).
    pub fn outputs(&mut self, t: f64, x: &[f64], bus: &mut SignalBus) {
        for (m, &off) in self.modules.iter_mut().zip(&self.offsets) {
            m.outputs(t, x, off, bus);
        }
    }

    /// Both passes: dx = f(t, x) with bus-coupled modules.
    pub fn eval(&mut self, t: f64, x: &[f64], bus: &mut SignalBus, dx: &mut [f64]) {
        self.outputs(t, x, bus);
        for (m, &off) in self.modules.iter().zip(&self.offsets) {
            let n = m.n_states();
            m.derivatives(t, x, off, bus, &mut dx[off..off + n]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::signals::{SignalId, SignalKind};

    /// Mass on a spring, all in one module: states (x, v).
    struct OneModule {
        k: f64,
        m: f64,
    }
    impl PlantModule for OneModule {
        fn name(&self) -> &str {
            "one"
        }
        fn n_states(&self) -> usize {
            2
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![("x".into(), "m".into()), ("v".into(), "m/s".into())]
        }
        fn init(&self, x: &mut [f64]) {
            x[0] = 1.0;
        }
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
        fn derivatives(&self, _: f64, x: &[f64], off: usize, _: &SignalBus, dx: &mut [f64]) {
            dx[0] = x[off + 1];
            dx[1] = -self.k / self.m * x[off];
        }
    }

    /// The same system split in two: the mass (x, v) reads the spring force from the
    /// bus; the spring (1 dummy state) computes force from the published position.
    struct Mass {
        m: f64,
        pos: SignalId,
        force: SignalId,
    }
    struct Spring {
        k: f64,
        pos: SignalId,
        force: SignalId,
    }
    impl PlantModule for Mass {
        fn name(&self) -> &str {
            "mass"
        }
        fn n_states(&self) -> usize {
            2
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![("x".into(), "m".into()), ("v".into(), "m/s".into())]
        }
        fn init(&self, x: &mut [f64]) {
            x[0] = 1.0;
        }
        fn outputs(&mut self, _: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
            bus.set(self.pos, x[off]);
        }
        fn derivatives(&self, _: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
            dx[0] = x[off + 1];
            dx[1] = bus.get(self.force) / self.m;
        }
    }
    impl PlantModule for Spring {
        fn name(&self) -> &str {
            "spring"
        }
        fn n_states(&self) -> usize {
            1
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![("unused".into(), "-".into())]
        }
        fn init(&self, _: &mut [f64]) {}
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
            bus.set(self.force, -self.k * bus.get(self.pos));
        }
        fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, dx: &mut [f64]) {
            dx[0] = 0.0;
        }
    }

    /// RK4 driven only through `Plant::eval`, so each stage re-runs the outputs pass.
    fn rk4(plant: &mut Plant, bus: &mut SignalBus, x: &mut [f64], h: f64, steps: usize) {
        let n = x.len();
        let (mut k1, mut k2, mut k3, mut k4, mut tmp) = (
            vec![0.0; n],
            vec![0.0; n],
            vec![0.0; n],
            vec![0.0; n],
            vec![0.0; n],
        );
        for _ in 0..steps {
            plant.eval(0.0, x, bus, &mut k1);
            for i in 0..n {
                tmp[i] = x[i] + 0.5 * h * k1[i];
            }
            plant.eval(0.0, &tmp, bus, &mut k2);
            for i in 0..n {
                tmp[i] = x[i] + 0.5 * h * k2[i];
            }
            plant.eval(0.0, &tmp, bus, &mut k3);
            for i in 0..n {
                tmp[i] = x[i] + h * k3[i];
            }
            plant.eval(0.0, &tmp, bus, &mut k4);
            for i in 0..n {
                x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
        }
    }

    #[test]
    fn coupled_modules_match_single_module_per_stage() {
        let (k, m) = (40.0, 0.5);
        let mut bus1 = SignalBus::new();
        let mut p1 = Plant::new();
        p1.add(Box::new(OneModule { k, m }));
        let mut x1 = p1.initial_state();
        rk4(&mut p1, &mut bus1, &mut x1, 1e-3, 2000);

        let mut bus2 = SignalBus::new();
        let pos = bus2
            .register("mass.x", "m", "", SignalKind::Output)
            .unwrap();
        let force = bus2
            .register("spring.force", "N", "", SignalKind::Output)
            .unwrap();
        bus2.freeze();
        let mut p2 = Plant::new();
        p2.add(Box::new(Mass { m, pos, force }));
        p2.add(Box::new(Spring { k, pos, force }));
        let mut x2 = p2.initial_state();
        rk4(&mut p2, &mut bus2, &mut x2, 1e-3, 2000);

        assert!(
            (x1[0] - x2[0]).abs() <= 1e-12 && (x1[1] - x2[1]).abs() <= 1e-12,
            "{x1:?} vs {x2:?}"
        );
        assert_eq!(p2.layout().len(), 3);
    }
}
