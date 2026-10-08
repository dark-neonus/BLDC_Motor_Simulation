//! Deterministic randomness (EQ-NUM-08): one ChaCha8 stream per block, seeded from
//! the scene seed and the block id with a *stable* hash (FNV-1a; std's hasher is not
//! stable across Rust versions). Stream state is serialisable for snapshots.

use rand_chacha::ChaCha8Rng;
use rand_core::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};

/// FNV-1a 64-bit (stable across platforms and compiler versions).
fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// Derives per-block streams from a scene seed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RngService {
    pub scene_seed: u64,
}

impl RngService {
    pub fn new(scene_seed: u64) -> Self {
        Self { scene_seed }
    }

    /// Independent stream for `block_id`.
    pub fn stream(&self, block_id: &str) -> BlockRng {
        let mut key = self.scene_seed.to_le_bytes().to_vec();
        key.extend_from_slice(block_id.as_bytes());
        BlockRng::from_seed_u64(fnv1a(&key))
    }
}

/// A block's random stream.
#[derive(Debug, Clone)]
pub struct BlockRng {
    seed: u64,
    rng: ChaCha8Rng,
    /// Spare value from Box–Muller.
    spare: Option<f64>,
}

/// Serialisable stream state (for snapshots).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BlockRngState {
    pub seed: u64,
    pub word_pos: u128,
    pub spare: Option<f64>,
}

impl BlockRng {
    pub fn from_seed_u64(seed: u64) -> Self {
        Self {
            seed,
            rng: ChaCha8Rng::seed_from_u64(seed),
            spare: None,
        }
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        // 53 random mantissa bits.
        (self.rng.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal N(0, 1) via Box–Muller.
    pub fn normal(&mut self) -> f64 {
        if let Some(z) = self.spare.take() {
            return z;
        }
        let u1 = 1.0 - self.uniform(); // (0, 1], avoids ln(0)
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (2.0 * std::f64::consts::PI * u2).sin_cos();
        self.spare = Some(r * s);
        r * c
    }

    /// Gaussian with standard deviation `sigma`.
    pub fn gaussian(&mut self, sigma: f64) -> f64 {
        sigma * self.normal()
    }

    pub fn save(&self) -> BlockRngState {
        BlockRngState {
            seed: self.seed,
            word_pos: self.rng.get_word_pos(),
            spare: self.spare,
        }
    }

    pub fn restore(st: &BlockRngState) -> Self {
        let mut rng = ChaCha8Rng::seed_from_u64(st.seed);
        rng.set_word_pos(st.word_pos);
        Self {
            seed: st.seed,
            rng,
            spare: st.spare,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_sequence_and_streams_differ() {
        let svc = RngService::new(42);
        let (mut a1, mut a2, mut b) = (svc.stream("enc"), svc.stream("enc"), svc.stream("adc"));
        let s1: Vec<f64> = (0..5).map(|_| a1.normal()).collect();
        let s2: Vec<f64> = (0..5).map(|_| a2.normal()).collect();
        let s3: Vec<f64> = (0..5).map(|_| b.normal()).collect();
        assert_eq!(s1, s2);
        assert_ne!(s1, s3);
        assert_ne!(RngService::new(43).stream("enc").normal(), s1[0]);
    }

    #[test]
    fn save_restore_continues_identically() {
        let mut r = RngService::new(7).stream("x");
        for _ in 0..13 {
            r.normal();
        }
        let st = r.save();
        let a: Vec<f64> = (0..20).map(|_| r.normal()).collect();
        let mut r2 = BlockRng::restore(&st);
        let b: Vec<f64> = (0..20).map(|_| r2.normal()).collect();
        assert_eq!(a, b);
    }

    #[test]
    fn normal_has_unit_variance() {
        let mut r = RngService::new(1).stream("stats");
        let n = 20_000;
        let xs: Vec<f64> = (0..n).map(|_| r.normal()).collect();
        let mean = xs.iter().sum::<f64>() / n as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
        // Standard errors: mean ±0.007, std ±0.005 at n = 2e4; bounds at ~5σ.
        assert!(mean.abs() < 0.035, "mean {mean}");
        assert!((var.sqrt() - 1.0).abs() < 0.025, "std {}", var.sqrt());
    }
}
