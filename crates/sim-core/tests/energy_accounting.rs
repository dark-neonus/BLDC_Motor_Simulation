//! Energy accounting (P03.T16, EQ-ENER-03).

use sim_core::energy::PowerKind;
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;

/// Mass (x, v) reads the spring force from the bus; optional damper b.
struct Mass {
    m: f64,
    b: f64,
    x0: f64,
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
        x[0] = self.x0;
        x[1] = 0.0;
    }
    fn outputs(&mut self, _: f64, x: &[f64], o: usize, bus: &mut SignalBus) {
        bus.set(self.pos, x[o]);
    }
    fn derivatives(&self, _: f64, x: &[f64], o: usize, bus: &SignalBus, dx: &mut [f64]) {
        dx[0] = x[o + 1];
        dx[1] = (bus.get(self.force) - self.b * x[o + 1]) / self.m;
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![("damper".into(), PowerKind::Loss)]
    }
    fn powers(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, p: &mut [f64]) {
        p[0] = self.b * x[o + 1] * x[o + 1];
    }
    fn stored_energy(&self, x: &[f64], o: usize, _: &SignalBus) -> f64 {
        0.5 * self.m * x[o + 1] * x[o + 1]
    }
}

struct Spring {
    k: f64,
    pos: SignalId,
    force: SignalId,
}
impl PlantModule for Spring {
    fn name(&self) -> &str {
        "spring"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        bus.set(self.force, -self.k * bus.get(self.pos));
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
    fn stored_energy(&self, _: &[f64], _: usize, bus: &SignalBus) -> f64 {
        0.5 * self.k * bus.get(self.pos).powi(2)
    }
}

fn engine(b: f64) -> Engine {
    let mut bus = SignalBus::new();
    let pos = bus.register("mass.x", "m", "", SignalKind::Output).unwrap();
    let force = bus
        .register("spring.force", "N", "", SignalKind::Output)
        .unwrap();
    let mut p = Plant::new();
    p.add(Box::new(Mass {
        m: 0.5,
        b,
        x0: 1.0,
        pos,
        force,
    }));
    p.add(Box::new(Spring {
        k: 40.0,
        pos,
        force,
    }));
    Engine::new(p, bus, vec![], 1e-4)
}

fn residual(e: &Engine) -> f64 {
    e.bus.get(e.bus.id("energy.residual").unwrap())
}

#[test]
fn lossless_oscillator_split_across_modules_conserves_energy() {
    let mut e = engine(0.0);
    e.step_until(SimTime::from_secs_f64(2.0)).unwrap();
    assert!(residual(&e).abs() < 1e-9, "residual {}", residual(&e));
}

#[test]
fn damped_oscillator_loss_equals_lost_mechanical_energy() {
    let mut e = engine(0.8);
    e.step_until(SimTime::from_secs_f64(2.0)).unwrap();
    let loss = e.bus.get(e.bus.id("energy.loss").unwrap());
    let d_st = e.bus.get(e.bus.id("energy.stored").unwrap());
    assert!(
        loss > 1.0,
        "most of the 20 J initial energy should be dissipated, loss {loss}"
    );
    assert!(
        (loss + d_st).abs() <= 1e-9 * 20.0,
        "loss {loss} vs −ΔE_st {}",
        -d_st
    );
    assert!(residual(&e).abs() < 1e-9);
    assert_eq!(e.bus.get(e.bus.id("energy.ok").unwrap()), 1.0);
}
