//! Minimal field-oriented control for the skeleton (P01.T02) — temporary.
//!
//! Velocity PI → i_q reference (clamped to ±I_max); d/q current PIs → v_d, v_q,
//! limited to the circle |v| ≤ V_bus/√3 (max linear SVPWM voltage). Anti-windup
//! by conditional integration (clamping): an integrator only accumulates while
//! its output is not saturated.

use super::model::PmsmParams;

/// Controller configuration.
#[derive(Debug, Clone, Copy)]
pub struct FocConfig {
    /// DC bus voltage [V].
    pub v_bus: f64,
    /// q-axis current limit [A].
    pub i_max: f64,
    /// Current-loop bandwidth [rad/s].
    pub omega_c: f64,
    /// Controller period [s].
    pub dt: f64,
}

impl FocConfig {
    /// Skeleton defaults: 24 V bus, 5 A, 1 kHz current bandwidth, 20 kHz control rate.
    pub fn skeleton() -> Self {
        Self {
            v_bus: 24.0,
            i_max: 5.0,
            omega_c: 2.0 * std::f64::consts::PI * 1000.0,
            dt: 50e-6,
        }
    }
}

/// Discrete PI gains (continuous-form u = Kp·e + Ki·∫e dt).
#[derive(Debug, Clone, Copy)]
pub struct PiGains {
    pub kp: f64,
    pub ki: f64,
}

/// Gains derived from the motor: current loop by pole-zero cancellation
/// (Kp = ωc·L, Ki = ωc·R → closed loop ≈ first order at ωc); velocity loop at
/// ωv = ωc/10 with Kp_v = ωv·J/Kt and Ki_v = Kp_v·ωv/4.
#[derive(Debug, Clone, Copy)]
pub struct FocGains {
    pub current_d: PiGains,
    pub current_q: PiGains,
    pub velocity: PiGains,
}

impl FocGains {
    pub fn design(m: &PmsmParams, cfg: &FocConfig) -> Self {
        let wc = cfg.omega_c;
        let wv = wc / 10.0;
        let kp_v = wv * m.j / m.kt();
        Self {
            current_d: PiGains {
                kp: wc * m.ld,
                ki: wc * m.r,
            },
            current_q: PiGains {
                kp: wc * m.lq,
                ki: wc * m.r,
            },
            velocity: PiGains {
                kp: kp_v,
                ki: kp_v * wv / 4.0,
            },
        }
    }
}

/// FOC controller state.
#[derive(Debug, Clone)]
pub struct Foc {
    pub cfg: FocConfig,
    pub gains: FocGains,
    /// Speed setpoint [rad/s].
    pub omega_ref: f64,
    /// Last q-current reference [A] (diagnostic).
    pub iq_ref: f64,
    int_v: f64,
    int_d: f64,
    int_q: f64,
}

impl Foc {
    pub fn new(cfg: FocConfig, gains: FocGains) -> Self {
        Self {
            cfg,
            gains,
            omega_ref: 0.0,
            iq_ref: 0.0,
            int_v: 0.0,
            int_d: 0.0,
            int_q: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.iq_ref = 0.0;
        self.int_v = 0.0;
        self.int_d = 0.0;
        self.int_q = 0.0;
    }

    /// One controller update from measured id, iq [A] and ω [rad/s]; returns (vd, vq) [V].
    pub fn update(&mut self, id: f64, iq: f64, omega: f64) -> (f64, f64) {
        let dt = self.cfg.dt;
        let g = self.gains;

        // Velocity PI → iq_ref, clamped to ±I_max.
        let e_v = self.omega_ref - omega;
        let iq_unsat = g.velocity.kp * e_v + self.int_v;
        self.iq_ref = iq_unsat.clamp(-self.cfg.i_max, self.cfg.i_max);
        if iq_unsat == self.iq_ref {
            self.int_v += g.velocity.ki * e_v * dt;
        }

        // Current PIs (id_ref = 0).
        let e_d = 0.0 - id;
        let e_q = self.iq_ref - iq;
        let vd = g.current_d.kp * e_d + self.int_d;
        let vq = g.current_q.kp * e_q + self.int_q;

        // Voltage limit circle |v| ≤ V_bus/√3; integrate only when not limited.
        let v_max = self.cfg.v_bus / 3f64.sqrt();
        let mag = vd.hypot(vq);
        if mag > v_max {
            let k = v_max / mag;
            (vd * k, vq * k)
        } else {
            self.int_d += g.current_d.ki * e_d * dt;
            self.int_q += g.current_q.ki * e_q * dt;
            (vd, vq)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::model::Pmsm;
    use super::*;

    /// Speed step 0 → 20 rad/s: enters the ±2 % band before 0.5 s and *stays* in it until
    /// the end of a 1 s run, overshoot < 20 %, and |i_q| never exceeds I_max.
    #[test]
    fn speed_step_settles_without_excess_overshoot_or_current() {
        let mp = PmsmParams::skeleton_6020();
        let cfg = FocConfig::skeleton();
        let mut foc = Foc::new(cfg, FocGains::design(&mp, &cfg));
        let mut motor = Pmsm::new(mp);
        let plant_dt = 5e-6;
        let substeps = (cfg.dt / plant_dt).round() as usize; // 10 plant steps per control step
        foc.omega_ref = 20.0;

        let mut t = 0.0;
        let mut peak: f64 = 0.0;
        let mut max_iq: f64 = 0.0;
        let mut settled_since: Option<f64> = None;
        while t < 1.0 {
            let s = motor.state;
            let (vd, vq) = foc.update(s.id, s.iq, s.omega);
            for _ in 0..substeps {
                motor.step(vd, vq, plant_dt);
                max_iq = max_iq.max(motor.state.iq.abs());
            }
            t += cfg.dt;
            let w = motor.state.omega;
            peak = peak.max(w);
            if (w - 20.0).abs() <= 0.02 * 20.0 {
                settled_since.get_or_insert(t);
            } else {
                settled_since = None;
            }
        }
        let settled = settled_since.expect("speed must settle within 2 %");
        assert!(settled < 0.5, "settled at {settled}");
        let overshoot = (peak - 20.0) / 20.0;
        assert!(overshoot < 0.20, "overshoot {:.1} %", overshoot * 100.0);
        assert!(max_iq <= cfg.i_max, "max |iq| = {max_iq}");
        eprintln!(
            "settled at {settled:.4} s, overshoot {:.2} %, max |iq| {max_iq:.3} A",
            overshoot * 100.0
        );
    }
}
