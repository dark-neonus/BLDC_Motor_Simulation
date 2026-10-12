//! Battery pack, Thevenin with one RC branch (EQ-SUP-04), built from cells.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Battery {
    /// Pack OCV(SoC) table: (SoC, volts), ascending SoC (cells × series).
    pub ocv: Vec<(f64, f64)>,
    /// Pack R0, R1 [Ω] and C1 [F].
    pub r0: f64,
    pub r1: f64,
    pub c1: f64,
    /// Pack capacity [A·h].
    pub q_ah: f64,
}

impl Battery {
    /// From cells: s in series, p in parallel (EQ-SUP-04 pack rules).
    pub fn from_cells(
        ocv_cell: &[(f64, f64)],
        s: u32,
        p: u32,
        r0: f64,
        r1: f64,
        c1: f64,
        q_cell_ah: f64,
    ) -> Self {
        let (s, p) = (f64::from(s), f64::from(p));
        Self {
            ocv: ocv_cell.iter().map(|&(x, v)| (x, s * v)).collect(),
            r0: s / p * r0,
            r1: s / p * r1,
            c1: p / s * c1,
            q_ah: p * q_cell_ah,
        }
    }

    /// OCV at a state of charge (linear interpolation, clamped at the table ends).
    pub fn ocv(&self, soc: f64) -> f64 {
        let t = &self.ocv;
        let (Some(first), Some(last)) = (t.first(), t.last()) else {
            return 0.0;
        };
        if soc <= first.0 {
            return first.1;
        }
        for w in t.windows(2) {
            let ((x0, y0), (x1, y1)) = (w[0], w[1]);
            if soc <= x1 {
                return y0 + (y1 - y0) * (soc - x0) / (x1 - x0);
            }
        }
        last.1
    }

    /// Chemical energy E_chem(SoC) = 3600·Q·∫₀^SoC OCV(s) ds [J] (exact for the
    /// piecewise-linear table; flat extrapolation outside it).
    pub fn e_chem(&self, soc: f64) -> f64 {
        let t = &self.ocv;
        let Some(first) = t.first() else { return 0.0 };
        let mut e = 0.0;
        // Below the first point: flat at its voltage.
        let lo = soc.min(first.0);
        e += first.1 * (lo - 0.0f64.min(lo));
        if soc > first.0 {
            for w in t.windows(2) {
                let ((x0, _), (x1, _)) = (w[0], w[1]);
                if soc <= x0 {
                    break;
                }
                let b = soc.min(x1);
                e += 0.5 * (self.ocv(x0) + self.ocv(b)) * (b - x0);
            }
            if let Some(last) = t.last()
                && soc > last.0
            {
                e += last.1 * (soc - last.0);
            }
        }
        3600.0 * self.q_ah * e
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_rules_and_chemical_energy() {
        let cell = [(0.0, 3.0), (0.5, 3.7), (1.0, 4.2)];
        let b = Battery::from_cells(&cell, 6, 2, 4e-3, 2e-3, 4000.0, 5.0);
        assert!((b.r0 - 12e-3).abs() < 1e-15 && (b.c1 - 4000.0 / 3.0).abs() < 1e-9);
        assert_eq!(b.q_ah, 10.0);
        assert!((b.ocv(0.25) - 6.0 * 3.35).abs() < 1e-12);
        // ∫₀¹ OCV = 6·(0.5·(3.0+3.7)/2 + 0.5·(3.7+4.2)/2) = 6·3.65.
        let full = 3600.0 * 10.0 * 6.0 * 3.65;
        assert!((b.e_chem(1.0) / full - 1.0).abs() < 1e-12);
        // dE/dSoC = 3600·Q·OCV.
        let h = 1e-6;
        let slope = (b.e_chem(0.3 + h) - b.e_chem(0.3 - h)) / (2.0 * h);
        assert!((slope / (3600.0 * 10.0 * b.ocv(0.3)) - 1.0).abs() < 1e-6);
    }
}
