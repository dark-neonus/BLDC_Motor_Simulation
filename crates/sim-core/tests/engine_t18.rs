//! P03.T18: snapshot completeness on a real engine; variable-event re-polling.

use sim_core::engine::block::{DiscreteBlock, SimError, StepCtx};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::fidelity::{FidelityConfig, StepLimits, Tier};
use sim_core::engine::plant::Plant;
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::skeleton::adapter::{SkeletonOptions, build_engine};

fn sig(e: &Engine, p: &str) -> f64 {
    e.bus.get(e.bus.id(p).unwrap())
}

#[test]
fn snapshot_with_live_param_restores_into_same_and_fresh_engine() {
    let mut e = build_engine(SkeletonOptions::default());
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 15.0,
        source: ChangeSource::Ui,
    });
    e.step_until(SimTime::from_secs_f64(0.05)).unwrap();
    e.queue(EngineCommand::SetParam {
        path: "motor.locked".into(),
        value: 1.0,
        source: ChangeSource::Ui,
    });
    e.step_until(SimTime::from_secs_f64(0.06)).unwrap();
    let snap = e.snapshot();
    e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    let (xa, ra) = (e.x.clone(), sig(&e, "energy.residual"));

    // Same engine (rewind): identical continuation, energy residual still small.
    e.restore(&snap).unwrap();
    e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    assert_eq!(e.x, xa);
    assert_eq!(sig(&e, "energy.residual"), ra);
    assert!(ra.abs() < 1e-3, "residual {ra}");

    // Fresh engine: live param (locked) comes back with the snapshot.
    let mut f = build_engine(SkeletonOptions::default());
    f.restore(&snap).unwrap();
    assert_eq!(f.get_param("motor.locked"), Some(1.0));
    f.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    assert_eq!(f.x, xa);
}

/// Variable-event block firing at k·1 ms + offset; offset is a live parameter.
struct Shifted {
    offset_ns: i64,
    out: SignalId,
}
impl DiscreteBlock for Shifted {
    fn id(&self) -> &str {
        "shifted"
    }
    fn period(&self) -> Option<SimTime> {
        None
    }
    fn priority(&self) -> u8 {
        40
    }
    fn next_event(&self, now: SimTime) -> Option<SimTime> {
        let k = (now.nanos() - self.offset_ns).div_euclid(1_000_000) + 1;
        Some(SimTime(k * 1_000_000 + self.offset_ns))
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        ctx.bus.set(self.out, ctx.t.nanos() as f64);
        Ok(())
    }
    fn reset(&mut self) {}
    fn set_param(&mut self, name: &str, v: f64) -> Result<f64, String> {
        match name {
            "offset_ns" => Ok(std::mem::replace(&mut self.offset_ns, v as i64) as f64),
            _ => Err("unknown".into()),
        }
    }
}

#[test]
fn variable_block_is_repolled_after_param_change() {
    let mut bus = SignalBus::new();
    let out = bus
        .register("test.fired_at", "ns", "", SignalKind::Diagnostic)
        .unwrap();
    let blocks: Vec<Box<dyn DiscreteBlock>> = vec![Box::new(Shifted { offset_ns: 0, out })];
    let mut e = Engine::new(Plant::new(), bus, blocks, 1e-4);
    e.step_until(SimTime(2_500_000)).unwrap(); // fired at 2.0 ms; next scheduled 3.0 ms
    e.queue(EngineCommand::SetParam {
        path: "shifted.offset_ns".into(),
        value: 300_000.0,
        source: ChangeSource::Ui,
    });
    // With re-polling the stale 3.0 ms event is replaced by 3.3 ms (2.3 ms is already past).
    e.step_until(SimTime(3_200_000)).unwrap();
    assert_eq!(
        e.bus.get(out),
        2_000_000.0,
        "stale 3.0 ms event must not fire"
    );
    e.step_until(SimTime(3_400_000)).unwrap();
    assert_eq!(e.bus.get(out), 3_300_000.0);
}

#[test]
fn fidelity_sets_dt_signals_and_tolerance() {
    let mut e = build_engine(SkeletonOptions::default());
    let lim = StepLimits {
        tau_e: 2.5e-3,
        t_ctrl: 50e-6,
        omega_max: 0.0,
        tau_bus: 0.0,
        t_pwm: 50e-6,
    };
    e.set_fidelity(FidelityConfig::preset(Tier::Detailed), &lim);
    assert_eq!(sig(&e, "sim.tier"), 2.0);
    assert!((sig(&e, "sim.dt_max") - 50e-6 / 200.0).abs() < 1e-15);
    assert!((e.dt_max - 2.5e-7).abs() < 1e-15);
    assert!(e.bus.id("energy.loss.motor").is_ok());
}
