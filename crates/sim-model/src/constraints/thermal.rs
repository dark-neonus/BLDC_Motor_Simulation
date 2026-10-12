//! Thermal rating rule (P06.T07): a stated continuous current above what the thermal
//! network allows at 25 °C ambient (EQ-THERM-05).

use super::{Issue, Severity, rule_issue};
use crate::params::MotorParams;
use crate::thermal_limits::{ThermalModel, continuous_current};
use crate::units::Kind;

/// Ambient for the rating check [K] (25 °C).
const T_AMB: f64 = 298.15;

pub(super) fn rules(m: &MotorParams, out: &mut Vec<Issue>) {
    let (Some(model), Some(rated)) = (
        ThermalModel::from_motor(m),
        m.ratings
            .current_continuous
            .as_ref()
            .and_then(|p| p.si(Kind::Current).ok()),
    ) else {
        return;
    };
    let Some(ic) = continuous_current(&model, T_AMB) else {
        return;
    };
    // 1 %: presets round their derived ratings.
    if rated > 1.01 * ic {
        out.push(rule_issue(
            "motor.ratings_thermal",
            Severity::Warn,
            "motor.ratings.current_continuous",
            format!(
                "the rated continuous current {rated:.3} A is above the thermal limit {ic:.3} A at 25 °C ambient (EQ-THERM-05); the winding would pass its maximum temperature"
            ),
        ));
    }
}
