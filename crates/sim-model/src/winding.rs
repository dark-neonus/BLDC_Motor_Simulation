//! Concentrated (tooth-coil, double-layer) 3-phase windings: validity, star-of-slots
//! layout, fundamental winding factor, cogging period (EQ-MOT-08)
//! `[HendershotMiller2010]`.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
pub enum Phase {
    A,
    B,
    C,
}

/// One tooth coil: wound around tooth `tooth`, sides in slots `tooth` and `tooth+1 (mod Q)`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, schemars::JsonSchema)]
pub struct Coil {
    pub tooth: u32,
    pub slot_a: u32,
    pub slot_b: u32,
    pub phase: Phase,
    /// +1 or −1 (winding direction).
    pub polarity: i8,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum WindingError {
    #[error("slots must be a positive multiple of 3 (got {0})")]
    SlotsNotMultipleOf3(u32),
    #[error("pole pairs must be ≥ 1")]
    NoPoles,
    #[error(
        "{slots} slots / {poles} poles cannot carry a balanced 3-phase winding (Q/(3·gcd(Q,p)) must be an integer)"
    )]
    Unbalanced { slots: u32, poles: u32 },
}

fn gcd(a: u32, b: u32) -> u32 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Mechanical cogging periods per revolution: N_c = LCM(Q, 2p) (EQ-MOT-08).
pub fn cogging_periods(slots: u32, pole_pairs: u32) -> u32 {
    let poles = 2 * pole_pairs;
    slots / gcd(slots, poles) * poles
}

/// Validity of a slot/pole combination for a balanced 3-phase winding.
pub fn check(slots: u32, pole_pairs: u32) -> Result<(), WindingError> {
    if pole_pairs == 0 {
        return Err(WindingError::NoPoles);
    }
    if slots == 0 || !slots.is_multiple_of(3) {
        return Err(WindingError::SlotsNotMultipleOf3(slots));
    }
    let t = gcd(slots, pole_pairs);
    if !(slots / t).is_multiple_of(3) {
        return Err(WindingError::Unbalanced {
            slots,
            poles: 2 * pole_pairs,
        });
    }
    Ok(())
}

/// Coil EMF phasor angle of tooth k [electrical rad]: k·α, α = 2π·p/Q.
fn coil_angle(k: u32, slots: u32, pole_pairs: u32) -> f64 {
    (k as f64) * 2.0 * std::f64::consts::PI * pole_pairs as f64 / slots as f64
}

/// Star-of-slots layout: each coil goes to the phase/polarity whose 60° band contains its
/// EMF phasor. Bands start at the first coil: A+ = [0°, 60°), then C−, B+, A−, C+, B−.
/// (Phasors often sit exactly on multiples of 30°, so bands must not be centred on them.)
pub fn layout(slots: u32, pole_pairs: u32) -> Result<Vec<Coil>, WindingError> {
    check(slots, pole_pairs)?;
    const BANDS: [(Phase, i8); 6] = [
        (Phase::A, 1),
        (Phase::C, -1),
        (Phase::B, 1),
        (Phase::A, -1),
        (Phase::C, 1),
        (Phase::B, -1),
    ];
    Ok((0..slots)
        .map(|k| {
            let deg = coil_angle(k, slots, pole_pairs)
                .to_degrees()
                .rem_euclid(360.0);
            // Band index: [0°, 60°) → 0, …; +1e-9 guards angles like 59.99999999999.
            let band = (((deg + 1e-9) / 60.0).floor() as usize) % 6;
            let (phase, polarity) = BANDS[band];
            Coil {
                tooth: k,
                slot_a: k,
                slot_b: (k + 1) % slots,
                phase,
                polarity,
            }
        })
        .collect())
}

/// Fundamental winding factor k_w = k_d·k_p of the tooth-coil winding.
/// k_p = |sin(α/2)| (coil span one slot pitch); k_d = |Σ ±phasors| / n of phase A.
pub fn winding_factor(slots: u32, pole_pairs: u32) -> Result<f64, WindingError> {
    let coils = layout(slots, pole_pairs)?;
    let (mut re, mut im, mut n) = (0.0, 0.0, 0.0);
    for c in coils.iter().filter(|c| c.phase == Phase::A) {
        let a = coil_angle(c.tooth, slots, pole_pairs);
        re += c.polarity as f64 * a.cos();
        im += c.polarity as f64 * a.sin();
        n += 1.0;
    }
    let kd = re.hypot(im) / n;
    let kp = (coil_angle(1, slots, pole_pairs) / 2.0).sin().abs();
    Ok(kd * kp)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Published fundamental winding factors for tooth-coil windings (cross-checked with
    /// the emetor.com winding calculator values).
    #[test]
    fn winding_factors_match_published_values() {
        for (q, p, kw) in [
            (12, 7, 0.933),
            (9, 6, 0.866),
            (36, 21, 0.933),
            (24, 14, 0.933),
            (12, 5, 0.933),
            (9, 4, 0.945),
        ] {
            let got = winding_factor(q, p).unwrap();
            assert!((got - kw).abs() < 1e-3, "{q}N{}P: {got} vs {kw}", 2 * p);
        }
    }

    #[test]
    fn layout_is_balanced() {
        for (q, p) in [(12, 7), (36, 21), (24, 14), (9, 6)] {
            let c = layout(q, p).unwrap();
            let count = |ph: Phase| c.iter().filter(|x| x.phase == ph).count();
            assert_eq!(
                (count(Phase::A), count(Phase::B), count(Phase::C)),
                (q as usize / 3, q as usize / 3, q as usize / 3)
            );
        }
    }

    #[test]
    fn invalid_combinations_rejected_and_cogging_period() {
        assert!(matches!(check(12, 6), Err(WindingError::Unbalanced { .. }))); // 12N12P
        assert!(matches!(
            check(10, 7),
            Err(WindingError::SlotsNotMultipleOf3(10))
        ));
        assert_eq!(cogging_periods(12, 7), 84);
        assert_eq!(cogging_periods(36, 21), 252);
    }
}
