//! sim-core: the simulation engine (time base, signal bus, integrators,
//! scheduler) and all physics and control blocks. Performs no IO.

#![forbid(unsafe_code)]
pub mod energy;
pub mod engine;
pub mod skeleton;

/// Test fixtures kept for cross-checks (P05.T03 compares the stationary-frame motor
/// against the dq model).
pub mod fixtures {
    pub use crate::skeleton::model as dq_pmsm;
}
