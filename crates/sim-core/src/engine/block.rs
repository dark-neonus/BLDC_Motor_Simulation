//! Discrete blocks (controllers, sensors, modulators): run at fixed or
//! variable event times and exchange values through the signal bus (EQ-NUM-02).

use super::signals::SignalBus;
use super::time::SimTime;

/// Errors raised inside the simulation loop (never panic there, PLAN §4.5).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SimError {
    #[error("block `{block}`: {msg}")]
    Block { block: String, msg: String },
    #[error("numerical fault at t = {t}: {msg}")]
    Numerical { t: SimTime, msg: String },
    #[error("{0}")]
    Other(String),
}

/// What a block sees when it fires.
pub struct StepCtx<'a> {
    pub t: SimTime,
    pub bus: &'a mut SignalBus,
}

/// A discrete-time block. Fixed-rate blocks return `Some(period)`; event-driven
/// blocks (e.g. PWM edges) return `None` and implement [`DiscreteBlock::next_event`].
pub trait DiscreteBlock: Send {
    /// Stable identifier (also the RNG stream key and snapshot key).
    fn id(&self) -> &str;
    /// Fixed period, or `None` for variable-event blocks.
    fn period(&self) -> Option<SimTime>;
    /// Offset of the first firing within the period.
    fn phase(&self) -> SimTime {
        SimTime::ZERO
    }
    /// Order among blocks firing at the same instant (lower first):
    /// sensors 10, estimators 20, controllers 30, modulator 40, inverter 50.
    fn priority(&self) -> u8;
    /// For variable-event blocks: the next firing time strictly after `now`.
    fn next_event(&self, _now: SimTime) -> Option<SimTime> {
        None
    }
    /// Execute at `ctx.t`: read inputs from the bus, write outputs to it.
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError>;
    /// Return to the initial state.
    fn reset(&mut self);
}

/// Standard priorities (D-004 event ordering).
pub mod priority {
    pub const SENSOR: u8 = 10;
    pub const ESTIMATOR: u8 = 20;
    pub const CONTROLLER: u8 = 30;
    pub const MODULATOR: u8 = 40;
    pub const INVERTER: u8 = 50;
}
