//! Production simulation engine (P03): time base, signal bus, block/plant
//! interfaces, integrators, multi-rate scheduler, runner, snapshots, recorder.
//! Spec: `docs/docs/physics/numerics.md` (EQ-NUM-*).

pub mod block;
pub mod commands;
#[allow(clippy::module_inception)]
pub mod engine;
pub mod fidelity;
pub mod integrate;
pub mod plant;
pub mod rng;
pub mod runner;
pub mod signals;
pub mod time;
