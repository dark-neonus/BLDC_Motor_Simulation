//! Energy accounting edge cases (P03.T20): External terms, discrete jumps, E_FLOOR.

use sim_core::energy::PowerKind;
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::SignalBus;
use sim_core::engine::time::SimTime;

/// Free mass pushed by a constant external force F (External term), optionally with a
/// one-off velocity jump booked via take_external_energy.
struct Pushed {
    m: f64,
    f: f64,
    jump_pending: Option<f64>,
}
impl PlantModule for Pushed {
    fn name(&self) -> &str {
        "pushed"
    }
    fn n_states(&self) -> usize {
        1
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![("v".into(), "m/s".into())]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = 0.0;
    }
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, dx: &mut [f64]) {
        dx[0] = self.f / self.m;
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![("push".into(), PowerKind::External)]
    }
    fn powers(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, p: &mut [f64]) {
        p[0] = self.f * x[o];
    }
    fn stored_energy(&self, x: &[f64], o: usize, _: &SignalBus) -> f64 {
        0.5 * self.m * x[o] * x[o]
    }
    fn set_param(&mut self, name: &str, v: f64, x: &mut [f64]) -> Result<f64, String> {
        if name != "kick" {
            return Err("unknown".into());
        }
        // Instant velocity change: book the kinetic-energy jump as external energy.
        let old = x[0];
        x[0] += v;
        self.jump_pending = Some(0.5 * self.m * (x[0] * x[0] - old * old));
        Ok(old)
    }
    fn take_external_energy(&mut self) -> f64 {
        self.jump_pending.take().unwrap_or(0.0)
    }
}

fn residual(e: &Engine) -> f64 {
    e.bus.get(e.bus.id("energy.residual").unwrap())
}

#[test]
fn external_work_and_jumps_close_the_balance() {
    use sim_core::engine::commands::{ChangeSource, EngineCommand};
    let mut p = Plant::new();
    p.add(Box::new(Pushed {
        m: 2.0,
        f: 3.0,
        jump_pending: None,
    }));
    let mut e = Engine::new(p, SignalBus::new(), vec![], 1e-3);
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    assert!(residual(&e).abs() < 1e-9, "after push: {}", residual(&e));
    e.queue(EngineCommand::SetParam {
        path: "pushed.kick".into(),
        value: 5.0,
        source: ChangeSource::Ui,
    });
    e.step_until(SimTime::from_secs_f64(2.0)).unwrap();
    assert!(residual(&e).abs() < 1e-9, "after kick: {}", residual(&e));
}

#[test]
fn idle_system_uses_energy_floor_not_nan() {
    let mut p = Plant::new();
    p.add(Box::new(Pushed {
        m: 1.0,
        f: 0.0,
        jump_pending: None,
    }));
    let mut e = Engine::new(p, SignalBus::new(), vec![], 1e-3);
    e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    assert_eq!(residual(&e), 0.0);
}
