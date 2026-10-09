//! Ideal voltage source (`inverter.mode = ideal`, P05.T11): the terminal voltages are
//! the commanded stationary-frame voltages (inverse Clarke, EQ-CONV-02) centred on
//! V_bus/2. No bus dynamics and no losses; the motor's `electrical_in` term is the
//! energy delivered. INTERIM until the real inverter (P07).

use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::physics::motor::electrical::inv_clarke;

pub struct IdealVoltageSource {
    v_alpha: SignalId,
    v_beta: SignalId,
    out: [SignalId; 3],
    v_bus: f64,
}

impl IdealVoltageSource {
    /// Registers `inverter.v_a` … `inverter.v_c`; reads the given command signals.
    pub fn new(
        bus: &mut SignalBus,
        v_alpha: SignalId,
        v_beta: SignalId,
        v_bus: f64,
    ) -> Result<Self, SignalError> {
        let out = [
            bus.register(
                "inverter.v_a",
                "V",
                "terminal voltage a (w.r.t. DC−)",
                SignalKind::Output,
            )?,
            bus.register(
                "inverter.v_b",
                "V",
                "terminal voltage b (w.r.t. DC−)",
                SignalKind::Output,
            )?,
            bus.register(
                "inverter.v_c",
                "V",
                "terminal voltage c (w.r.t. DC−)",
                SignalKind::Output,
            )?,
        ];
        Ok(Self {
            v_alpha,
            v_beta,
            out,
            v_bus,
        })
    }

    pub fn outputs_ids(&self) -> [SignalId; 3] {
        self.out
    }
}

impl PlantModule for IdealVoltageSource {
    fn name(&self) -> &str {
        "inverter"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _t: f64, _x: &[f64], _off: usize, bus: &mut SignalBus) {
        let v = inv_clarke(bus.get(self.v_alpha), bus.get(self.v_beta));
        for (id, vk) in self.out.iter().zip(v) {
            bus.set(*id, vk + 0.5 * self.v_bus);
        }
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}
