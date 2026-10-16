//! Controllers and estimators (EQ-CTRL-*), as discrete blocks that see only sensor
//! readings.

pub mod estimators;
pub mod foc;
pub mod framework;
pub mod pid;
pub mod setpoint;
