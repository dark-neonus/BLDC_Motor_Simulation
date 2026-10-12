//! Modulators (EQ-INV-04…07): voltage references → leg duties, and six-step patterns.
//! Pure functions; the controllers (P09) and the interim drive call them.

use super::leg::LegState;
use crate::physics::motor::electrical::inv_clarke;

/// Sine PWM (EQ-INV-04): d_x = ½ + v_x*/V_bus, clamped to [0, 1].
pub fn spwm(v: [f64; 3], v_bus: f64) -> [f64; 3] {
    v.map(|vx| (0.5 + vx / v_bus).clamp(0.0, 1.0))
}

/// SVPWM by min–max injection (EQ-INV-05) from phase references.
pub fn svpwm(v: [f64; 3], v_bus: f64) -> [f64; 3] {
    let (mx, mn) = v
        .iter()
        .fold((f64::MIN, f64::MAX), |(a, b), &x| (a.max(x), b.min(x)));
    let vo = -0.5 * (mx + mn);
    v.map(|vx| (0.5 + (vx + vo) / v_bus).clamp(0.0, 1.0))
}

/// Overmodulation clamp (EQ-INV-06): keep the angle, shorten to the SVPWM linear limit
/// V_bus/√3. Returns the (possibly scaled) vector and whether it was limited.
pub fn clamp_vector(va: f64, vb: f64, v_bus: f64) -> (f64, f64, bool) {
    let lim = v_bus / 3f64.sqrt();
    let mag = va.hypot(vb);
    if mag > lim {
        let k = lim / mag;
        (va * k, vb * k, true)
    } else {
        (va, vb, false)
    }
}

/// SVPWM duties from an αβ reference, with the EQ-INV-06 clamp; the flag reports
/// saturation (for anti-windup).
pub fn svpwm_ab(va: f64, vb: f64, v_bus: f64) -> ([f64; 3], bool) {
    let (a, b, sat) = clamp_vector(va, vb, v_bus);
    (svpwm(inv_clarke(a, b), v_bus), sat)
}

/// Six-step leg pattern for an electrical angle (EQ-INV-07, forward rotation): the phase
/// with the most positive back-EMF is High, the most negative Low, the third Off.
/// `advance` shifts the table earlier [rad].
pub fn six_step(theta_e: f64, advance: f64) -> [LegState; 3] {
    use LegState::*;
    // Sector 0 covers 330°…30°; each sector is 60° wide.
    let deg = (theta_e + advance).to_degrees().rem_euclid(360.0);
    let sector = ((deg + 30.0) / 60.0).floor() as usize % 6;
    // (High, Low) phase index per sector: a = 0, b = 1, c = 2.
    const HL: [(usize, usize); 6] = [(1, 2), (1, 0), (2, 0), (2, 1), (0, 1), (0, 2)];
    let (h, l) = HL[sector];
    std::array::from_fn(|k| {
        if k == h {
            High
        } else if k == l {
            Low
        } else {
            Off
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Classic sector-based SVPWM (zero vectors split equally) for the comparison.
    fn svpwm_sector(va: f64, vb: f64, v_bus: f64) -> [f64; 3] {
        let ang = vb.atan2(va).rem_euclid(2.0 * PI);
        let mag = va.hypot(vb);
        let s = (ang / (PI / 3.0)).floor() as usize % 6;
        let th = ang - s as f64 * PI / 3.0;
        let m = 3f64.sqrt() * mag / v_bus;
        let t1 = m * (PI / 3.0 - th).sin();
        let t2 = m * th.sin();
        let t0 = 1.0 - t1 - t2;
        // Active vectors V_s and V_{s+1} as (a, b, c) high patterns.
        const V: [[f64; 3]; 6] = [
            [1., 0., 0.],
            [1., 1., 0.],
            [0., 1., 0.],
            [0., 1., 1.],
            [0., 0., 1.],
            [1., 0., 1.],
        ];
        let (a, b) = (V[s], V[(s + 1) % 6]);
        std::array::from_fn(|k| t1 * a[k] + t2 * b[k] + 0.5 * t0)
    }

    #[test]
    fn v_inv_002_svpwm_matches_the_sector_reference_and_the_worked_example() {
        let v_bus = 24.0;
        for k in 0..36 {
            let th = 2.0 * PI * k as f64 / 36.0;
            let (va, vb) = (10.0 * th.cos(), 10.0 * th.sin());
            let (d, sat) = svpwm_ab(va, vb, v_bus);
            assert!(!sat);
            let r = svpwm_sector(va, vb, v_bus);
            for x in 0..3 {
                assert!((d[x] - r[x]).abs() < 1e-12, "θ = {th}: {d:?} vs {r:?}");
            }
        }
        // inverter.md worked example: 10 V at 0.3 rad.
        let (d, _) = svpwm_ab(10.0 * 0.3f64.cos(), 10.0 * 0.3f64.sin(), v_bus);
        for (x, want) in d.iter().zip([0.8519, 0.3614, 0.1481]) {
            assert!((x - want).abs() < 1e-4, "{d:?}");
        }
    }

    #[test]
    fn v_inv_003_linear_limits() {
        let v_bus = 24.0;
        // SVPWM reaches V_bus/√3 with duties exactly spanning [0, 1].
        let lim = v_bus / 3f64.sqrt();
        // Just inside the limit (at it, hypot's rounding can land a hair above).
        let r = lim * (1.0 - 1e-12);
        let mut span: f64 = 0.0;
        for k in 0..360 {
            let th = 2.0 * PI * k as f64 / 360.0;
            let (d, sat) = svpwm_ab(r * th.cos(), r * th.sin(), v_bus);
            assert!(!sat);
            span = span.max(
                d.iter().cloned().fold(f64::MIN, f64::max)
                    - d.iter().cloned().fold(f64::MAX, f64::min),
            );
        }
        assert!((span - 1.0).abs() < 1e-9, "{span}");
        let (_, _, sat) = clamp_vector(1.01 * lim, 0.0, v_bus);
        assert!(sat);
        // Sine PWM's limit is V_bus/2: a phase amplitude of V_bus/2 gives duty 1.
        let d = spwm(inv_clarke(v_bus / 2.0, 0.0), v_bus);
        assert!((d[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn six_step_table() {
        use LegState::*;
        // Mid-sector angles (deg) → (a, b, c) per EQ-INV-07.
        let cases = [
            (0.0, [Off, High, Low]),
            (60.0, [Low, High, Off]),
            (120.0, [Low, Off, High]),
            (180.0, [Off, Low, High]),
            (240.0, [High, Low, Off]),
            (300.0, [High, Off, Low]),
        ];
        for (deg, want) in cases {
            assert_eq!(six_step(f64::to_radians(deg), 0.0), want, "{deg}°");
        }
        // Advance shifts the table earlier: 25° with 10° advance is already sector 1.
        assert_eq!(six_step(25f64.to_radians(), 10f64.to_radians()), cases[1].1);
        // The High phase has the most positive back-EMF (e ∝ −sin(θ − k·120°)).
        for k in 0..360 {
            let th = (k as f64 + 0.5).to_radians();
            let e: [f64; 3] = std::array::from_fn(|x| -(th - x as f64 * 2.0 * PI / 3.0).sin());
            let legs = six_step(th, 0.0);
            let hi = (0..3).max_by(|&a, &b| e[a].total_cmp(&e[b])).unwrap();
            let lo = (0..3).min_by(|&a, &b| e[a].total_cmp(&e[b])).unwrap();
            assert_eq!(legs[hi], High, "{k}°");
            assert_eq!(legs[lo], Low, "{k}°");
        }
    }
}
