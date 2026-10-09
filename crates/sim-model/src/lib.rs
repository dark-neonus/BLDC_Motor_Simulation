//! sim-model: parameter types, units, schemas, constraints and YAML library IO.
//!
//! This crate describes *what* is simulated (motor, inverter, supply, ...),
//! never *how* it is stepped in time (that is `sim-core`).

#![forbid(unsafe_code)]

pub mod constraints;
pub mod param;
pub mod params;
pub mod units;
pub mod winding;
