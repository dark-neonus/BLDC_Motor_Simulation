//! Fixtures for cross-checks. `dq_pmsm` is the reference dq model (P05.T03 compares
//! the stationary-frame motor against it). Until P05.T11 removes the skeleton it is
//! also used by `skeleton`; then gate this module with `#[cfg(any(test, feature = "fixtures"))]`.

pub mod dq_pmsm;
