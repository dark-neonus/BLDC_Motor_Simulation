//! Safety behaviour of the engine (P03.T17): NaN guard, input validation,
//! panic-free snapshot restore, saturating time.

use sim_core::engine::block::SimError;
use sim_core::engine::commands::{ChangeSource, EngineCommand, EngineEvent};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::SignalBus;
use sim_core::engine::time::SimTime;
use sim_core::skeleton::adapter::{SkeletonOptions, build_engine};

/// x' = k·x with a live gain k: k = huge blows the state up to inf/NaN.
struct Grow {
    k: f64,
}
impl PlantModule for Grow {
    fn name(&self) -> &str {
        "grow"
    }
    fn n_states(&self) -> usize {
        1
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![("x".into(), "-".into())]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = 1.0;
    }
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
    fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
        dx[0] = self.k * x[o];
    }
    fn set_param(&mut self, name: &str, v: f64, _: &mut [f64]) -> Result<f64, String> {
        match name {
            "k" => Ok(std::mem::replace(&mut self.k, v)),
            _ => Err("unknown".into()),
        }
    }
}

#[test]
fn non_finite_state_stops_with_numerical_error() {
    let mut p = Plant::new();
    p.add(Box::new(Grow { k: 0.0 }));
    let mut e = Engine::new(p, SignalBus::new(), vec![], 1e-3);
    e.queue(EngineCommand::SetParam {
        path: "grow.k".into(),
        value: 1e300,
        source: ChangeSource::Ui,
    });
    let r = e.step_until(SimTime::from_secs_f64(1.0));
    assert!(matches!(r, Err(SimError::Numerical { .. })), "{r:?}");
}

#[test]
fn set_signal_rejects_non_finite_and_non_input() {
    let mut e = build_engine(SkeletonOptions::default());
    for (path, v) in [("ctrl.omega_ref", f64::NAN), ("motor.omega", 1.0)] {
        e.queue(EngineCommand::SetSignal {
            path: path.into(),
            value: v,
            source: ChangeSource::Mcp,
        });
    }
    e.apply_pending();
    let ev = e.drain_events();
    assert_eq!(ev.len(), 2);
    assert!(
        ev.iter()
            .all(|x| matches!(x, EngineEvent::CommandRejected { .. })),
        "{ev:?}"
    );
}

#[test]
fn mismatched_snapshot_is_rejected_without_panic_or_change() {
    let mut e = build_engine(SkeletonOptions::default());
    e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
    let before = (e.time, e.x.clone());
    let mut bad = e.snapshot();
    bad.next_fire.clear(); // would have panicked in copy_from_slice
    assert!(e.restore(&bad).is_err());
    let mut bad = e.snapshot();
    bad.modules[0].0 = "other".into();
    assert!(e.restore(&bad).is_err());
    assert_eq!((e.time, e.x.clone()), before);
}

#[test]
fn sim_time_saturates_instead_of_overflowing() {
    let t = SimTime(i64::MAX - 5) + SimTime(100);
    assert_eq!(t, SimTime(i64::MAX));
    let e = build_engine(SkeletonOptions::default());
    // A huge step target must not overflow (the target is computed with saturating add).
    assert_eq!(e.time + SimTime(i64::MAX), SimTime(i64::MAX));
}

#[test]
fn failed_restore_rolls_back_and_invalid_dt_is_rejected() {
    let mut e = build_engine(SkeletonOptions::default());
    e.queue(EngineCommand::SetSignal {
        path: "ctrl.omega_ref".into(),
        value: 10.0,
        source: ChangeSource::Ui,
    });
    e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
    let good = e.snapshot();
    e.step_until(SimTime::from_secs_f64(0.03)).unwrap();
    let before = (e.time, e.x.clone(), e.bus.values().to_vec());

    // Block data valid, module data invalid → error after blocks were restored → roll back.
    let mut bad = good.clone();
    bad.modules[0].1 = serde_json::json!({ "nope": 1 });
    assert!(e.restore(&bad).is_err());
    assert_eq!((e.time, e.x.clone(), e.bus.values().to_vec()), before);

    let mut bad = good.clone();
    bad.dt_max = 0.0; // would make advance() loop forever
    assert!(e.restore(&bad).is_err());
    assert_eq!(e.time, before.0);
}
