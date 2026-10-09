//! Cogging torque (P05.T04, EQ-MOT-08): T_cog = Σ A_k sin(k·N_c·θm + φ_k), conservative
//! with potential U_cog = Σ A_k/(k·N_c)·cos(k·N_c·θm + φ_k), T_cog = −dU/dθm.
//! The torque is an internal input of the rotor; U_cog is reported as stored energy.

use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

#[derive(Debug, Clone, PartialEq)]
pub struct CoggingParams {
    /// Cogging periods per mechanical revolution, N_c = LCM(Q, 2p).
    pub n_c: f64,
    /// Harmonics k = 1, 2, …: (A_k [N·m], φ_k [rad]).
    pub terms: Vec<(f64, f64)>,
}

pub struct Cogging {
    p: CoggingParams,
    theta: SignalId,
    torque: SignalId,
}

impl Cogging {
    /// Registers `motor.torque_cog`; reads the mechanical angle `theta`.
    pub fn new(
        p: CoggingParams,
        theta: SignalId,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let torque = bus.register(
            "motor.torque_cog",
            "N*m",
            "cogging torque",
            SignalKind::Output,
        )?;
        Ok(Self { p, theta, torque })
    }

    pub fn torque_id(&self) -> SignalId {
        self.torque
    }

    pub fn torque_at(&self, th: f64) -> f64 {
        self.harmonics(th).map(|(a, _, arg)| a * arg.sin()).sum()
    }

    pub fn potential_at(&self, th: f64) -> f64 {
        self.harmonics(th)
            .map(|(a, kn, arg)| a / kn * arg.cos())
            .sum()
    }

    fn harmonics(&self, th: f64) -> impl Iterator<Item = (f64, f64, f64)> + '_ {
        self.p.terms.iter().enumerate().map(move |(i, &(a, ph))| {
            let kn = (i + 1) as f64 * self.p.n_c;
            (a, kn, kn * th + ph)
        })
    }
}

impl PlantModule for Cogging {
    fn name(&self) -> &str {
        "motor.cogging"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _t: f64, _x: &[f64], _off: usize, bus: &mut SignalBus) {
        let t = self.torque_at(bus.get(self.theta));
        bus.set(self.torque, t);
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
    fn stored_energy(&self, _x: &[f64], _off: usize, bus: &SignalBus) -> f64 {
        self.potential_at(bus.get(self.theta))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::engine::Engine;
    use crate::engine::plant::Plant;
    use crate::engine::time::SimTime;
    use crate::physics::mech::rotor::{RotorParams, RotorRigid};
    use std::f64::consts::TAU;

    fn params() -> CoggingParams {
        // 12N14P: N_c = LCM(12, 14) = 84 (EQ-MOT-08); two harmonics.
        CoggingParams {
            n_c: 84.0,
            terms: vec![(0.02, 0.3), (0.005, -1.0)],
        }
    }

    fn cog() -> Cogging {
        let mut bus = SignalBus::new();
        let th = bus
            .register("motor.theta", "rad", "", SignalKind::State)
            .unwrap();
        Cogging::new(params(), th, &mut bus).unwrap()
    }

    #[test]
    fn period_mean_and_potential() {
        let c = cog();
        let period = TAU / 84.0;
        for th in [0.0, 0.013, 0.5, 2.0] {
            // Periodic in 2π/N_c (rounding of the argument only).
            assert!((c.torque_at(th + period) - c.torque_at(th)).abs() < 1e-12);
            // T = −dU/dθ (central difference, atol 1e-8 at h = 1e-7 rad).
            let d = (c.potential_at(th + 1e-7) - c.potential_at(th - 1e-7)) / 2e-7;
            assert!((c.torque_at(th) + d).abs() < 1e-8, "{th}");
        }
        let n = 10_000;
        let mean = (0..n)
            .map(|i| c.torque_at(period * i as f64 / n as f64))
            .sum::<f64>()
            / n as f64;
        assert!(mean.abs() < 1e-15, "zero average over a period: {mean}");
    }

    #[test]
    fn unpowered_rotor_with_cogging_conserves_energy() {
        let mut bus = SignalBus::new();
        let rp = RotorParams {
            j: 2e-5,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 3.0,
        };
        let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![]).unwrap();
        let c = Cogging::new(params(), bus.id("motor.theta").unwrap(), &mut bus).unwrap();
        let rotor = rotor.with_internal(c.torque_id());
        let mut plant = Plant::new();
        plant.add(Box::new(rotor));
        plant.add(Box::new(c));
        // The cogging period in time is ~25 ms at 3 rad/s; dt = 20 µs resolves it.
        let mut e = Engine::new(plant, bus, vec![], 2e-5);
        e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
        let r = e.bus.get(e.bus.id("energy.residual").unwrap());
        assert!(r.abs() < 1e-6, "residual {r}");
        let w = e.bus.get(e.bus.id("motor.omega").unwrap());
        assert!(
            (w - 3.0).abs() > 1e-4,
            "cogging actually acted on the rotor: ω = {w}"
        );
    }
}
