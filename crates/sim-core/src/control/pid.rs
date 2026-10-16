//! Discrete PI / PID with anti-windup (EQ-CTRL-01/02):
//!
//! u*_k = K_p e_k + I_k + D_k (+ feed-forward),  u_k = sat(u*_k),
//! I_{k+1} = I_k + T (K_i e_k + K_b (u_k − u*_k)),
//! D_k = T_f/(T_f + T)·D_{k−1} − K_d/(T_f + T)·(y_k − y_{k−1})   (on the measurement).
//!
//! Anti-windup: `Clamping` (K_b = 0, I frozen while saturated and e pushes further),
//! `BackCalculation` (K_b = K_i/K_p) or `None`. A pure struct: the period T is passed
//! per update (the owning block's period), the limits per call when an outer limiter
//! (e.g. the FOC d-priority voltage limit) decides saturation.

use serde::{Deserialize, Serialize};

use super::framework::LoopLaw;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum AntiWindup {
    #[default]
    Clamping,
    BackCalculation,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    /// Derivative gain K_d and its filter time constant T_f [s] (K_d = 0: PI).
    pub kd: f64,
    pub tf: f64,
    /// Output limits (lo, hi) for `update`.
    pub limits: (f64, f64),
    pub aw: AntiWindup,
    /// Integrator I_k, derivative D_k and the last measurement.
    i: f64,
    d: f64,
    y_prev: Option<f64>,
    /// Whether the last output was clamped (for an inner loop or a plot).
    pub saturated: bool,
}

impl Pid {
    pub fn pi(kp: f64, ki: f64, limits: (f64, f64), aw: AntiWindup) -> Self {
        Self {
            kp,
            ki,
            kd: 0.0,
            tf: 0.0,
            limits,
            aw,
            i: 0.0,
            d: 0.0,
            y_prev: None,
            saturated: false,
        }
    }

    pub fn with_derivative(mut self, kd: f64, tf: f64) -> Self {
        (self.kd, self.tf) = (kd, tf);
        self
    }

    pub fn integrator(&self) -> f64 {
        self.i
    }

    /// One update with the controller's own limits.
    pub fn update(&mut self, r: f64, y: f64, t: f64) -> f64 {
        self.step(r, y, 0.0, self.limits, t)
    }

    /// One update with a feed-forward added before saturation and the limits of this
    /// tick (EQ-CTRL-03: saturation and anti-windup act on the total command).
    pub fn step(&mut self, r: f64, y: f64, ff: f64, (lo, hi): (f64, f64), t: f64) -> f64 {
        let e = r - y;
        if self.kd != 0.0 {
            let dy = self.y_prev.map_or(0.0, |p| y - p);
            self.d = self.tf / (self.tf + t) * self.d - self.kd / (self.tf + t) * dy;
        }
        self.y_prev = Some(y);
        let u_star = self.kp * e + self.i + self.d + ff;
        let u = u_star.clamp(lo, hi);
        self.saturated = u != u_star;
        match self.aw {
            AntiWindup::None => self.i += t * self.ki * e,
            AntiWindup::Clamping => {
                // Freeze only while the error pushes further into the active limit.
                let pushes =
                    (u_star > hi && e * self.ki > 0.0) || (u_star < lo && e * self.ki < 0.0);
                if !pushes {
                    self.i += t * self.ki * e;
                }
            }
            AntiWindup::BackCalculation => {
                let kb = if self.kp != 0.0 {
                    self.ki / self.kp
                } else {
                    0.0
                };
                self.i += t * (self.ki * e + kb * (u - u_star));
            }
        }
        u
    }

    /// Bumpless start from an output u that something else produced: I = u − K_p e − D.
    pub fn track(&mut self, u: f64, r: f64, y: f64) {
        self.i = u - self.kp * (r - y) - self.d;
        self.y_prev = Some(y);
    }

    pub fn reset(&mut self) {
        (self.i, self.d, self.y_prev, self.saturated) = (0.0, 0.0, None, false);
    }
}

impl LoopLaw for Pid {
    fn update(&mut self, r: f64, y: f64, t: f64) -> f64 {
        Pid::update(self, r, y, t)
    }
    fn reset(&mut self) {
        Pid::reset(self);
    }
    fn track(&mut self, u: f64, r: f64, y: f64) {
        Pid::track(self, u, r, y);
    }
    /// The state only: gains and limits come from the builder (and infinite limits
    /// would not survive JSON).
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "i": self.i, "d": self.d, "y_prev": self.y_prev, "saturated": self.saturated })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        (self.i, self.d) = (f("i")?, f("d")?);
        self.y_prev = v.get("y_prev").and_then(serde_json::Value::as_f64);
        self.saturated = v
            .get("saturated")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `saturated`")?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T: f64 = 1e-3;
    const INF: (f64, f64) = (f64::NEG_INFINITY, f64::INFINITY);

    /// PI on a first-order plant τẋ = −x + K·u, exact ZOH discretisation
    /// x_{k+1} = a·x_k + (1 − a)·K·u_k. The closed loop is z_{k+1} = A z_k + B r with
    /// z = (x, I); its analytic solution z_k = z_ss + A^k (z_0 − z_ss) uses
    /// A^k = (λ1^k (A − λ2) − λ2^k (A − λ1)) / (λ1 − λ2) (Cayley–Hamilton, distinct
    /// real eigenvalues). The simulated loop matches it to 1e-12.
    #[test]
    fn pi_step_response_matches_the_analytic_discrete_solution() {
        let (tau, k, kp, ki, r) = (0.05, 2.0, 0.4, 5.0, 1.0);
        let a = (-T / tau).exp();
        let b = (1.0 - a) * k;
        // x' = a x + b (kp (r − x) + I);  I' = I + T ki (r − x).
        let m = [[a - b * kp, b], [-T * ki, 1.0]];
        let (tr, det) = (m[0][0] + m[1][1], m[0][0] * m[1][1] - m[0][1] * m[1][0]);
        let disc = tr * tr - 4.0 * det;
        assert!(disc > 0.0, "pick gains with distinct real poles");
        let (l1, l2) = ((tr + disc.sqrt()) / 2.0, (tr - disc.sqrt()) / 2.0);
        assert!(l1.abs() < 1.0 && l2.abs() < 1.0, "stable");
        // Steady state: x = r, I = r/K (u = r/K holds x = r).
        let zss = [r, r / k];
        let analytic = |n: i32| -> [f64; 2] {
            let (p1, p2) = (l1.powi(n), l2.powi(n));
            let an = |i: usize, j: usize| {
                let id = if i == j { 1.0 } else { 0.0 };
                (p1 * (m[i][j] - l2 * id) - p2 * (m[i][j] - l1 * id)) / (l1 - l2)
            };
            let d0 = [0.0 - zss[0], 0.0 - zss[1]];
            [
                zss[0] + an(0, 0) * d0[0] + an(0, 1) * d0[1],
                zss[1] + an(1, 0) * d0[0] + an(1, 1) * d0[1],
            ]
        };
        let mut pi = Pid::pi(kp, ki, INF, AntiWindup::Clamping);
        let mut x = 0.0;
        // Until the slower pole has decayed to 1e-12 (|λ|^n).
        let n_end = (1e-12f64.ln() / l1.abs().max(l2.abs()).ln()).ceil() as i32;
        for n in 0..n_end {
            let z = analytic(n);
            assert!((x - z[0]).abs() < 1e-12, "n {n}: x {x} vs {}", z[0]);
            assert!((pi.integrator() - z[1]).abs() < 1e-12, "n {n}: I");
            let u = pi.update(r, x, T);
            x = a * x + b * u;
        }
        assert!((x - r).abs() < 1e-9, "no steady-state error: {x}");
    }

    /// Integrating plant ẋ = u with |u| ≤ 1: a setpoint step of 5 saturates the output
    /// for ~5 s, then the loop settles. Without anti-windup the integrator charged during
    /// saturation overshoots by 3.3 and needs 13.2 s to stay within 2 % of the setpoint.
    /// Measured: clamping 0.068 / 5.1 s, back-calculation (K_b = K_i/K_p) 0.34 / 8.5 s.
    #[test]
    fn anti_windup_recovers_faster_from_saturation() {
        let run = |aw: AntiWindup| {
            let mut pi = Pid::pi(2.0, 1.0, (-1.0, 1.0), aw);
            let (mut x, mut peak, mut last_out) = (0.0f64, 0.0f64, 0.0);
            for n in 0..40_000 {
                let u = pi.update(5.0, x, T);
                x += T * u;
                peak = peak.max(x);
                if (x - 5.0).abs() > 0.1 {
                    last_out = n as f64 * T;
                }
            }
            (peak - 5.0, last_out)
        };
        let (os_none, t_none) = run(AntiWindup::None);
        for (aw, os_max, t_max) in [
            (AntiWindup::Clamping, 0.05, 0.45),
            (AntiWindup::BackCalculation, 0.15, 0.7),
        ] {
            let (os, t) = run(aw);
            assert!(os < os_max * os_none, "{aw:?}: overshoot {os} vs {os_none}");
            assert!(t < t_max * t_none, "{aw:?}: settle {t} s vs {t_none} s");
        }
        assert!(os_none > 1.0, "the plain PI winds up: {os_none}");
    }

    #[test]
    fn clamping_freezes_only_while_pushing_into_the_limit() {
        let mut pi = Pid::pi(1.0, 10.0, (-1.0, 1.0), AntiWindup::Clamping);
        pi.update(5.0, 0.0, T);
        assert_eq!(pi.integrator(), 0.0, "saturated high, e > 0: frozen");
        assert!(pi.saturated);
        // Error reverses while still saturated: integrates (unwinds).
        pi.i = 3.0;
        pi.update(-1.0, 0.0, T);
        assert!((pi.integrator() - (3.0 - T * 10.0)).abs() < 1e-15);
        // Back-calculation: K_b = K_i/K_p pulls I by T·K_b·(u − u*).
        let mut bc = Pid::pi(2.0, 10.0, (-1.0, 1.0), AntiWindup::BackCalculation);
        bc.update(5.0, 0.0, T);
        let want = T * (10.0 * 5.0 + 5.0 * (1.0 - 10.0));
        assert!((bc.integrator() - want).abs() < 1e-15);
    }

    /// EQ-CTRL-02: the derivative acts on y, so a setpoint step gives no kick; on a ramp
    /// y = c·t it settles to −K_d·c.
    #[test]
    fn derivative_on_measurement_without_setpoint_kick() {
        let mut pid = Pid::pi(1.0, 0.0, INF, AntiWindup::None).with_derivative(0.1, 0.01);
        pid.update(0.0, 0.0, T);
        assert_eq!(pid.update(1.0, 0.0, T), 1.0, "step in r: P only");
        let c = 3.0;
        let mut u = 0.0;
        for n in 2..2000 {
            u = pid.update(0.0, c * n as f64 * T, T);
            u += c * n as f64 * T; // remove the P part (−K_p·y with r = 0)
        }
        assert!((u + 0.1 * c).abs() < 1e-9, "D → −K_d·c: {u}");
    }

    #[test]
    fn total_command_saturates_with_feed_forward_and_tracks_bumplessly() {
        let mut pi = Pid::pi(1.0, 100.0, INF, AntiWindup::Clamping);
        // ff pushes the total past the tick's limit: saturated, I frozen.
        let u = pi.step(0.5, 0.0, 2.0, (-2.0, 2.0), T);
        assert_eq!((u, pi.saturated, pi.integrator()), (2.0, true, 0.0));
        // Bumpless: after track(u), the next output (same r, y) continues from u.
        pi.track(1.5, 1.0, 0.25);
        let u = pi.update(1.0, 0.25, T);
        assert!((u - 1.5).abs() < 1e-12, "{u}");
        let saved = LoopLaw::save(&pi);
        let mut other = Pid::pi(1.0, 100.0, INF, AntiWindup::Clamping);
        LoopLaw::restore(&mut other, &saved).unwrap();
        assert_eq!(other, pi);
    }
}
