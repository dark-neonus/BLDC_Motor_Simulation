//! Thermal ratings from the three-node network (EQ-THERM-05/06): continuous current
//! and the time a higher current can be held. Pure math, no engine; the UI and the
//! constraint rules use it.

use crate::params::MotorParams;
use crate::units::Kind;

/// Copper temperature coefficient α_Cu [1/K] (EQ-THERM-02).
pub const ALPHA_CU: f64 = 0.00393;
/// Reference temperature of R when the motor gives none [K] (20 °C).
pub const T_REF_DEFAULT: f64 = 293.15;

/// The network and winding data the ratings need (SI, temperatures in K).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalModel {
    /// Phase resistance at `t_ref` [Ω].
    pub r_ref: f64,
    pub t_ref: f64,
    pub r_ws: f64,
    pub r_sh: f64,
    pub r_ha: f64,
    pub c_w: f64,
    pub c_s: f64,
    pub c_h: f64,
    /// Maximum winding temperature [K].
    pub t_max: f64,
}

impl ThermalModel {
    /// From a motor's `electrical.r_phase` and `thermal` block; `None` if it has no
    /// thermal data or a value does not parse.
    pub fn from_motor(m: &MotorParams) -> Option<Self> {
        let t = m.thermal.as_ref()?;
        let g = |p: &crate::param::Param, k: Kind| p.si(k).ok();
        Some(Self {
            r_ref: g(&m.electrical.r_phase, Kind::Resistance)?,
            t_ref: match &t.t_ref {
                Some(p) => g(p, Kind::Temperature)?,
                None => T_REF_DEFAULT,
            },
            r_ws: g(&t.r_ws, Kind::ThermalResistance)?,
            r_sh: g(&t.r_sh, Kind::ThermalResistance)?,
            r_ha: g(&t.r_ha, Kind::ThermalResistance)?,
            c_w: g(&t.c_w, Kind::HeatCapacity)?,
            c_s: g(&t.c_s, Kind::HeatCapacity)?,
            c_h: g(&t.c_h, Kind::HeatCapacity)?,
            t_max: g(&t.t_max, Kind::Temperature)?,
        })
    }

    /// R(T_w) (EQ-THERM-02).
    pub fn r(&self, t_w: f64) -> f64 {
        self.r_ref * (1.0 + ALPHA_CU * (t_w - self.t_ref))
    }

    /// Series resistance winding → ambient R_th [K/W].
    pub fn r_th(&self) -> f64 {
        self.r_ws + self.r_sh + self.r_ha
    }
}

/// Continuous peak phase current Î_cont at ambient `t_amb` [K] with only copper heat
/// (EQ-THERM-05, P_fe = P_fric = 0). `None` if T_max ≤ T_a.
pub fn continuous_current(m: &ThermalModel, t_amb: f64) -> Option<f64> {
    let head = m.t_max - t_amb;
    (head > 0.0).then(|| (head / (1.5 * m.r(m.t_max) * m.r_th())).sqrt())
}

/// Time to bring the winding from ambient to T_max at a constant peak phase current
/// `i` (EQ-THERM-06): integrate EQ-THERM-01 with R(T) (RK4) and bisect the crossing
/// step to 1 µs. `None` if T_max is never reached (i ≤ Î_cont) or T_max ≤ T_a.
pub fn peak_time(m: &ThermalModel, t_amb: f64, i: f64) -> Option<f64> {
    if m.t_max <= t_amb || continuous_current(m, t_amb).is_none_or(|ic| i <= ic) {
        return None;
    }
    let f = |x: &[f64; 3]| -> [f64; 3] {
        let p_cu = 1.5 * m.r(x[0]) * i * i;
        let (q_ws, q_sh, q_ha) = (
            (x[0] - x[1]) / m.r_ws,
            (x[1] - x[2]) / m.r_sh,
            (x[2] - t_amb) / m.r_ha,
        );
        [
            (p_cu - q_ws) / m.c_w,
            (q_ws - q_sh) / m.c_s,
            (q_sh - q_ha) / m.c_h,
        ]
    };
    let rk4 = |x: &[f64; 3], h: f64| -> [f64; 3] {
        let add = |a: &[f64; 3], k: &[f64; 3], s: f64| {
            [a[0] + s * k[0], a[1] + s * k[1], a[2] + s * k[2]]
        };
        let k1 = f(x);
        let k2 = f(&add(x, &k1, h / 2.0));
        let k3 = f(&add(x, &k2, h / 2.0));
        let k4 = f(&add(x, &k3, h));
        let mut y = *x;
        for j in 0..3 {
            y[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
        }
        y
    };
    // Step: a fraction of the fastest node time constant.
    let h = 0.02 * (m.c_w * m.r_ws).min(m.c_s * m.r_sh).min(m.c_h * m.r_ha);
    let t_cap = 1e3 * (m.c_w + m.c_s + m.c_h) * m.r_th();
    let (mut t, mut x) = (0.0, [t_amb; 3]);
    while t < t_cap {
        let y = rk4(&x, h);
        if y[0] >= m.t_max {
            let (mut lo, mut hi) = (0.0, h);
            while hi - lo > 1e-6 {
                let mid = 0.5 * (lo + hi);
                if rk4(&x, mid)[0] >= m.t_max {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            return Some(t + hi);
        }
        (t, x) = (t + h, y);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    const M: ThermalModel = ThermalModel {
        r_ref: 1.0,
        t_ref: 293.15,
        r_ws: 0.5,
        r_sh: 0.3,
        r_ha: 4.0,
        c_w: 10.0,
        c_s: 20.0,
        c_h: 50.0,
        t_max: 373.15,
    };

    /// EQ-THERM-05 closed form, and its defining property: at Î_cont the steady
    /// winding temperature is exactly T_max.
    #[test]
    fn continuous_current_closed_form() {
        let t_a = 298.15;
        let ic = continuous_current(&M, t_a).unwrap();
        let expect = (75.0 / (1.5 * 1.0 * (1.0 + ALPHA_CU * 80.0) * 4.8)).sqrt();
        assert!((ic / expect - 1.0).abs() < 1e-12);
        let p = 1.5 * M.r(M.t_max) * ic * ic;
        assert!((t_a + p * M.r_th() - M.t_max).abs() < 1e-9);
        assert_eq!(continuous_current(&M, 400.0), None);
    }

    /// EQ-THERM-06: above Î_cont the time is finite, shorter for more current and at
    /// least the adiabatic lower bound; at or below Î_cont it is never reached.
    #[test]
    fn peak_time_bounds() {
        let t_a = 298.15;
        let ic = continuous_current(&M, t_a).unwrap();
        assert_eq!(peak_time(&M, t_a, ic), None);
        let (t2, t4) = (
            peak_time(&M, t_a, 2.0 * ic).unwrap(),
            peak_time(&M, t_a, 4.0 * ic).unwrap(),
        );
        assert!(t4 < t2);
        for (k, t) in [(2.0, t2), (4.0, t4)] {
            let i = k * ic;
            let bound = M.c_w * (M.t_max - t_a) / (1.5 * M.r(M.t_max) * i * i);
            assert!(t >= bound, "{t} < {bound}");
        }
        // Very high current: nearly adiabatic. With R(T) the adiabatic heating has the
        // closed form t_ad = C_w/(1.5·R_ref·α·I²)·ln(R(T_max)/R(T_a)); the network leaks a
        // little heat to the stator, so t is just above it (within 5 %).
        let i = 20.0 * ic;
        let t_ad = M.c_w / (1.5 * M.r_ref * ALPHA_CU * i * i) * (M.r(M.t_max) / M.r(t_a)).ln();
        let t = peak_time(&M, t_a, i).unwrap();
        assert!(t >= t_ad && t / t_ad < 1.05, "{t} vs {t_ad}");
    }
}
