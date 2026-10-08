//! Production simulation engine (P03): time base, signal bus, block/plant
//! interfaces, integrators, multi-rate scheduler, runner, snapshots, recorder.
//! Spec: `docs/docs/physics/numerics.md` (EQ-NUM-*).

pub mod block;
pub mod plant;
pub mod signals;
pub mod time;
