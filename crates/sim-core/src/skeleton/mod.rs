//! Walking-skeleton physics (P01): an ideal PMSM in the dq frame, a fixed-step
//! RK4 integrator and a minimal FOC. **Temporary** — replaced by the production
//! engine and models in P03–P09. Keep the interfaces small.

pub mod adapter;
pub mod foc;
pub mod params;
pub mod scenario;
