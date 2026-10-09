//! Back-EMF shapes (EQ-MOT-03). `k(θ) = dΦ/dθ` is normalised so its fundamental is
//! −sin θ; Φ is the zero-mean integral of k (Φ = cos θ for the sinusoidal shape).

use std::f64::consts::{PI, TAU};

#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    Sinusoidal,
    /// Flat-top width w per half period [rad], 0 ≤ w ≤ π (π: square wave, 0: triangle).
    Trapezoidal {
        w: f64,
    },
    /// Extra odd harmonics: (n, b_n, φ_n) relative to the fundamental.
    Harmonics(Vec<(u32, f64, f64)>),
}

/// Fundamental of the unit odd trapezoid with flat-top width w (EQ-MOT-03).
pub fn trap_b1(w: f64) -> f64 {
    let r = (PI - w) / 2.0;
    if r < 1e-12 {
        4.0 / PI
    } else {
        4.0 / PI * r.sin() / r
    }
}

/// Unit-amplitude odd trapezoid: ramps over [0, r], flat over [r, π − r].
fn trap(theta: f64, w: f64) -> f64 {
    let r = (PI - w) / 2.0;
    let th = theta.rem_euclid(TAU);
    let (h, sign) = if th < PI { (th, 1.0) } else { (th - PI, -1.0) };
    let v = if r < 1e-12 {
        1.0
    } else if h < r {
        h / r
    } else if h > PI - r {
        (PI - h) / r
    } else {
        1.0
    };
    sign * v
}

/// ∫₀^θ trap over one period [0, 2π).
fn trap_integral(theta: f64, w: f64) -> f64 {
    let r = (PI - w) / 2.0;
    let half = |h: f64| {
        if r < 1e-12 {
            h
        } else if h < r {
            h * h / (2.0 * r)
        } else if h > PI - r {
            PI - r - (PI - h) * (PI - h) / (2.0 * r)
        } else {
            r / 2.0 + (h - r)
        }
    };
    let th = theta.rem_euclid(TAU);
    if th < PI {
        half(th)
    } else {
        (PI - r) - half(th - PI)
    }
}

impl Shape {
    /// Normalised back-EMF shape k(θ).
    pub fn k(&self, th: f64) -> f64 {
        match self {
            Shape::Sinusoidal => -th.sin(),
            Shape::Trapezoidal { w } => -trap(th, *w) / trap_b1(*w),
            Shape::Harmonics(hs) => {
                -(th.sin()
                    + hs.iter()
                        .map(|&(n, b, ph)| b * (f64::from(n) * th + ph).sin())
                        .sum::<f64>())
            }
        }
    }

    /// Normalised magnet flux Φ(θ) (zero mean, dΦ/dθ = k).
    pub fn phi(&self, th: f64) -> f64 {
        match self {
            Shape::Sinusoidal => th.cos(),
            Shape::Trapezoidal { w } => {
                // Mean of the integral over a period is (π − r)/2.
                let r = (PI - w) / 2.0;
                -(trap_integral(th, *w) - (PI - r) / 2.0) / trap_b1(*w)
            }
            Shape::Harmonics(hs) => {
                th.cos()
                    + hs.iter()
                        .map(|&(n, b, ph)| b / f64::from(n) * (f64::from(n) * th + ph).cos())
                        .sum::<f64>()
            }
        }
    }

    /// Harmonic part of k: k_h = k + sin θ (zero for the sinusoidal shape).
    pub fn k_h(&self, th: f64) -> f64 {
        match self {
            Shape::Sinusoidal => 0.0,
            _ => self.k(th) + th.sin(),
        }
    }

    /// Harmonic part of Φ: Φ − cos θ.
    pub fn phi_h(&self, th: f64) -> f64 {
        match self {
            Shape::Sinusoidal => 0.0,
            _ => self.phi(th) - th.cos(),
        }
    }

    pub fn is_sinusoidal(&self) -> bool {
        matches!(self, Shape::Sinusoidal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fourier sine coefficient of f at harmonic n (midpoint rule, 20 000 points).
    fn sine_coef(f: impl Fn(f64) -> f64, n: f64) -> f64 {
        let m = 20_000;
        (0..m)
            .map(|i| (i as f64 + 0.5) * TAU / m as f64)
            .map(|t| f(t) * (n * t).sin())
            .sum::<f64>()
            * 2.0
            / m as f64
    }

    #[test]
    fn fundamentals_are_normalised() {
        let shapes = [
            Shape::Sinusoidal,
            Shape::Trapezoidal { w: 2.0 * PI / 3.0 },
            Shape::Trapezoidal { w: 0.3 },
            Shape::Harmonics(vec![(5, 0.04, 0.3), (7, 0.02, 0.0)]),
        ];
        for s in &shapes {
            // Fundamental of k is −sin θ (EQ-MOT-03). Midpoint rule on a piecewise-linear
            // periodic function: error ~1e-8 at 20 000 points → atol 1e-6.
            let b1 = sine_coef(|t| s.k(t), 1.0);
            assert!((b1 + 1.0).abs() < 1e-6, "{s:?}: {b1}");
            // Φ is the integral of k: central difference matches (atol 1e-6 with h = 1e-5).
            for t in [0.1, 1.0, 2.5, 4.0, 5.9] {
                let d = (s.phi(t + 1e-5) - s.phi(t - 1e-5)) / 2e-5;
                assert!((d - s.k(t)).abs() < 1e-6, "{s:?} at {t}: {d} vs {}", s.k(t));
            }
            // Zero mean.
            let mean: f64 = (0..20_000)
                .map(|i| s.phi((i as f64 + 0.5) * TAU / 20_000.0))
                .sum::<f64>()
                / 20_000.0;
            assert!(mean.abs() < 1e-6, "{s:?}: mean {mean}");
        }
    }

    #[test]
    fn trapezoid_flat_top_and_classic_value() {
        let w = 2.0 * PI / 3.0;
        // b1(120°) = 12/π² (EQ-MOT-03 table); flat top normalised to π²/12 = 0.8225.
        assert!((trap_b1(w) - 12.0 / (PI * PI)).abs() < 1e-12);
        let s = Shape::Trapezoidal { w };
        let top = PI * PI / 12.0;
        for t in [PI / 6.0 + 1e-9, PI / 2.0, 5.0 * PI / 6.0 - 1e-9] {
            assert!((s.k(t) + top).abs() < 1e-9, "flat at {t}");
        }
        assert!((s.k(PI / 12.0) + top / 2.0).abs() < 1e-9, "ramp midpoint");
    }
}
