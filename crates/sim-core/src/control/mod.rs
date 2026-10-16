//! Controllers and estimators (EQ-CTRL-*), as discrete blocks that see only sensor
//! readings.

pub mod estimators;
pub mod foc;
pub mod framework;
pub mod pid;
pub mod setpoint;
pub mod six_step;

/// A scene's controller family as plain SI data (built by `skeleton::adapter`).
#[derive(Debug, Clone, PartialEq)]
pub enum ControllerSpec {
    Foc(foc::FocSpec),
    SixStep(six_step::SixStepSpec),
}
