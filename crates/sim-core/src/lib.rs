//! sim-core: the simulation engine (time base, signal bus, integrators,
//! scheduler) and all physics and control blocks. Performs no IO.

#![forbid(unsafe_code)]
pub mod build;
pub mod energy;
pub mod engine;
pub mod fixtures;
pub mod scenario_run;
pub mod skeleton;
