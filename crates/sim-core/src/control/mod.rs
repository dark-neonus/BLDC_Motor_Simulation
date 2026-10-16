//! Controllers and estimators (EQ-CTRL-*), as discrete blocks that see only sensor
//! readings.

pub mod calibration;
pub mod estimators;
pub mod foc;
pub mod framework;
pub mod luau_block;
pub mod mit;
pub mod open_loop;
pub mod pid;
pub mod setpoint;
pub mod six_step;

/// A scene's controller family as plain SI data (built by `skeleton::adapter`).
#[derive(Debug, Clone, PartialEq)]
pub enum ControllerSpec {
    Foc(foc::FocSpec),
    SixStep(six_step::SixStepSpec),
    OpenLoop(open_loop::OpenLoopSpec),
    Mit(mit::MitSpec),
    Custom(CustomSpec),
}

/// A user Luau controller (P09.T10) on the FOC current loop, unless the script writes
/// voltages or duties itself.
#[derive(Debug, Clone, PartialEq)]
pub struct CustomSpec {
    pub script: String,
    pub rate_hz: f64,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
    /// The current loop underneath (cascade off).
    pub foc: foc::FocSpec,
}

impl CustomSpec {
    /// The current loop runs unless the script drives the voltage or the duties.
    pub fn needs_current_loop(&self) -> bool {
        !self.outputs.iter().any(|o| {
            o.starts_with("inverter.duty_") || o == "ctrl.foc.v_alpha" || o == "ctrl.foc.v_beta"
        })
    }
}
