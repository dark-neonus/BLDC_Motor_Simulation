//! Engine benchmarks (P03.T13): skeleton motor + 20 kHz FOC at dt = 5 µs.
//! The printed throughput (sim seconds per wall second) is the sim/real ratio baseline.

use criterion::{Criterion, criterion_group, criterion_main};
use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::time::SimTime;
use sim_core::skeleton::adapter::{SkeletonOptions, build_engine};

fn skeleton_10ms(c: &mut Criterion) {
    c.bench_function("skeleton_foc_10ms_sim_dt5us", |b| {
        b.iter_batched(
            || {
                let mut e = build_engine(SkeletonOptions::default());
                e.queue(EngineCommand::SetSignal {
                    path: "ctrl.omega_ref".into(),
                    value: 20.0,
                    source: ChangeSource::Internal,
                });
                e
            },
            |mut e| e.step_until(SimTime::from_secs_f64(0.01)).unwrap(),
            criterion::BatchSize::SmallInput,
        )
    });
}

/// Raw plant cost: 10k RK4 steps of the skeleton motor (no events: open loop, 1 ms span).
fn plant_steps(c: &mut Criterion) {
    c.bench_function("plant_rk4_steps_x10000", |b| {
        b.iter_batched(
            || {
                build_engine(SkeletonOptions {
                    open_loop_vq: Some(6.0),
                    dt_max: 1e-7,
                    ..Default::default()
                })
            },
            |mut e| e.step_until(SimTime::from_secs_f64(1e-3)).unwrap(),
            criterion::BatchSize::SmallInput,
        )
    });
}

/// Event-scheduler cost: 3 rates (20 kHz, 5 kHz, 1 kHz) over 0.1 s with an empty plant.
fn event_rates(c: &mut Criterion) {
    use sim_core::engine::block::{DiscreteBlock, SimError, StepCtx};
    use sim_core::engine::engine::Engine;
    use sim_core::engine::plant::Plant;
    use sim_core::engine::signals::SignalBus;
    struct Nop(SimTime, &'static str);
    impl DiscreteBlock for Nop {
        fn id(&self) -> &str {
            self.1
        }
        fn period(&self) -> Option<SimTime> {
            Some(self.0)
        }
        fn priority(&self) -> u8 {
            30
        }
        fn step(&mut self, _: &mut StepCtx<'_>) -> Result<(), SimError> {
            Ok(())
        }
        fn reset(&mut self) {}
    }
    c.bench_function("events_3rates_2600_events", |b| {
        b.iter_batched(
            || {
                let blocks: Vec<Box<dyn DiscreteBlock>> = vec![
                    Box::new(Nop(SimTime(50_000), "a")),
                    Box::new(Nop(SimTime(200_000), "b")),
                    Box::new(Nop(SimTime(1_000_000), "c")),
                ];
                Engine::new(Plant::new(), SignalBus::new(), blocks, 1e-3)
            },
            |mut e| e.step_until(SimTime::from_secs_f64(0.1)).unwrap(),
            criterion::BatchSize::SmallInput,
        )
    });
}

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(20);
    targets = skeleton_10ms, plant_steps, event_rates
}
criterion_main!(benches);
