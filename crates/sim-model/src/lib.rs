//! sim-model: parameter types, units, schemas, constraints and YAML library IO.
//!
//! This crate describes *what* is simulated (motor, inverter, supply, ...),
//! never *how* it is stepped in time (that is `sim-core`).

#![forbid(unsafe_code)]

pub mod constraints;
pub mod io;
pub mod library;
pub mod param;
pub mod params;
pub mod scenario;
pub mod scene;
pub mod units;
pub mod winding;
pub mod wizard;
