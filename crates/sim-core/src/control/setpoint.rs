//! Setpoint sources (EQ-CTRL-04/07, signals.md): the UI, scenarios and MCP write the
//! active setpoints `ctrl.theta_ref` [rad], `ctrl.omega_ref` [rad/s] and
//! `ctrl.torque_ref` [N·m]. All are load-side; controllers convert with the ratio N.

use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Setpoints {
    pub theta: SignalId,
    pub omega: SignalId,
    pub torque: SignalId,
}

impl Setpoints {
    /// The setpoint signals, registered here unless another block already did (the
    /// skeleton FOC registers `ctrl.omega_ref` and `ctrl.torque_ref` until P09.T03).
    pub fn get_or_register(bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut one = |path: &str, unit: &str, desc: &str| match bus.id(path) {
            Ok(id) => Ok(id),
            Err(_) => bus.register(path, unit, desc, SignalKind::Input),
        };
        Ok(Self {
            theta: one("ctrl.theta_ref", "rad", "angle setpoint (load side)")?,
            omega: one("ctrl.omega_ref", "rad/s", "speed setpoint (load side)")?,
            torque: one("ctrl.torque_ref", "N*m", "torque setpoint (load side)")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_once_and_reuses_existing_signals() {
        let mut bus = SignalBus::new();
        let w = bus
            .register("ctrl.omega_ref", "rad/s", "", SignalKind::Input)
            .unwrap();
        let s = Setpoints::get_or_register(&mut bus).unwrap();
        assert_eq!(s.omega, w);
        assert_eq!(Setpoints::get_or_register(&mut bus).unwrap(), s);
        assert_eq!(bus.meta(s.theta).unit, "rad");
    }
}
