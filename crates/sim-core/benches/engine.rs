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

criterion_group! {
    name = benches;
    config = Criterion::default().sample_size(20);
    targets = skeleton_10ms
}
criterion_main!(benches);
