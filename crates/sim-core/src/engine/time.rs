//! Simulation time base: integer nanoseconds (D-004). EQ-NUM-01.

use std::fmt;

/// Simulation time / duration in integer nanoseconds.
#[derive(
    Debug,
    Clone,
    Copy,
    Default,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    serde::Serialize,
    serde::Deserialize,
)]
pub struct SimTime(pub i64);

impl SimTime {
    pub const ZERO: SimTime = SimTime(0);
    pub const NS_PER_S: i64 = 1_000_000_000;

    pub fn from_nanos(ns: i64) -> Self {
        Self(ns)
    }

    /// Nearest nanosecond to `s` seconds.
    pub fn from_secs_f64(s: f64) -> Self {
        Self((s * Self::NS_PER_S as f64).round() as i64)
    }

    pub fn as_secs_f64(self) -> f64 {
        self.0 as f64 / Self::NS_PER_S as f64
    }

    pub fn nanos(self) -> i64 {
        self.0
    }
}

impl std::ops::Add for SimTime {
    type Output = SimTime;
    fn add(self, rhs: SimTime) -> SimTime {
        SimTime(self.0.saturating_add(rhs.0))
    }
}

impl std::ops::Sub for SimTime {
    type Output = SimTime;
    fn sub(self, rhs: SimTime) -> SimTime {
        SimTime(self.0.saturating_sub(rhs.0))
    }
}

impl std::ops::AddAssign for SimTime {
    fn add_assign(&mut self, rhs: SimTime) {
        self.0 = self.0.saturating_add(rhs.0);
    }
}

impl fmt::Display for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.9} s", self.as_secs_f64())
    }
}

/// A rate converted to an integer period (D-010: rounded, actual frequency reported).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Period {
    pub period: SimTime,
    /// Frequency actually achieved by the rounded period [Hz].
    pub actual_hz: f64,
    /// Relative error |actual − requested| / requested.
    pub rel_error: f64,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PeriodError {
    #[error("frequency must be finite and > 0, got {0} Hz")]
    Invalid(f64),
    #[error("frequency {0} Hz is above the 1 GHz time-base limit")]
    TooHigh(f64),
}

/// Convert a frequency to the nearest integer-nanosecond period (D-010).
pub fn period_from_hz(hz: f64) -> Result<Period, PeriodError> {
    if !hz.is_finite() || hz <= 0.0 {
        return Err(PeriodError::Invalid(hz));
    }
    let ns = (SimTime::NS_PER_S as f64 / hz).round() as i64;
    if ns < 1 {
        return Err(PeriodError::TooHigh(hz));
    }
    let actual_hz = SimTime::NS_PER_S as f64 / ns as f64;
    Ok(Period {
        period: SimTime(ns),
        actual_hz,
        rel_error: (actual_hz - hz).abs() / hz,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_and_rounded_periods() {
        let p = period_from_hz(20_000.0).unwrap();
        assert_eq!(p.period, SimTime(50_000));
        assert_eq!(p.rel_error, 0.0);
        let p = period_from_hz(30_000.0).unwrap();
        assert_eq!(p.period, SimTime(33_333));
        // 1e9/33333 = 30000.3 Hz → ~1e-5 relative error, reported (D-010).
        assert!((p.actual_hz - 30_000.300_003).abs() < 1e-3 && p.rel_error > 1e-6);
    }

    #[test]
    fn invalid_frequencies_rejected() {
        assert!(period_from_hz(0.0).is_err());
        assert!(period_from_hz(f64::NAN).is_err());
        assert!(period_from_hz(5e9).is_err());
    }

    #[test]
    fn seconds_round_trip() {
        assert_eq!(SimTime::from_secs_f64(1.5e-6), SimTime(1500));
        assert_eq!(SimTime(2_500_000_000).as_secs_f64(), 2.5);
    }
}
