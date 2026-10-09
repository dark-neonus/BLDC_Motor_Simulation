//! abc electrical plant (P05.T01/T02): stationary-frame flux-linkage states ψα, ψβ
//! (D-013, EQ-MOT-01), currents from the flux–current relation (EQ-MOT-02, linear
//! magnetics), back-EMF shapes (EQ-MOT-03), neutral voltage (EQ-MOT-04) and torque
//! (EQ-MOT-07). Isolated neutral: i_a + i_b + i_c = 0.
//!
//! Energy (EQ-MOT-13): P_elec is the `Input` term, copper the `Loss` term, W_mag the
//! stored energy. The air-gap power T_e·ω_m is an internal transfer to the rotor.

use std::f64::consts::FRAC_PI_3;

use super::backemf::Shape;
use crate::energy::PowerKind;
use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

const SQRT3: f64 = 1.732_050_807_568_877_2;
const TWO_PI_3: f64 = 2.0 * FRAC_PI_3;

#[derive(Debug, Clone, PartialEq)]
pub struct ElectricalParams {
    /// Star-equivalent phase resistance [Ω].
    pub r: f64,
    pub ld: f64,
    pub lq: f64,
    /// Magnet flux linkage, fundamental, peak per phase [Wb] at `t_ref`.
    pub lambda: f64,
    pub pole_pairs: f64,
    pub shape: Shape,
    /// Initial rotor-frame currents and electrical angle (sets ψ₀).
    pub i_d0: f64,
    pub i_q0: f64,
    pub theta_e0: f64,
}

/// Magnet temperature input for λ(T) = λ_ref·(1 + α_Br·(T − T_ref)) (EQ-THERM-03).
#[derive(Debug, Clone, Copy)]
pub struct LambdaTemperature {
    pub t_magnet: SignalId,
    pub alpha_br: f64,
    pub t_ref: f64,
}

/// Bus signals the module reads.
#[derive(Debug, Clone, Copy)]
pub struct ElectricalInputs {
    /// Terminal voltages w.r.t. DC− (`inverter.v_a` …).
    pub v: [SignalId; 3],
    pub theta_e: SignalId,
    pub omega_e: SignalId,
}

#[derive(Debug, Clone, Copy)]
struct Out {
    psi_alpha: SignalId,
    psi_beta: SignalId,
    i: [SignalId; 3],
    i_alpha: SignalId,
    i_beta: SignalId,
    i_d: SignalId,
    i_q: SignalId,
    e: [SignalId; 3],
    v_n: SignalId,
    torque: SignalId,
    p_cu: SignalId,
}

/// Currents and rotor-frame fluxes at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Solved {
    pub i_alpha: f64,
    pub i_beta: f64,
    pub i_d: f64,
    pub i_q: f64,
    pub psi_d: f64,
    pub psi_q: f64,
}

pub struct MotorElectrical {
    p: ElectricalParams,
    inp: ElectricalInputs,
    out: Out,
    temp: Option<LambdaTemperature>,
    jump: f64,
    last_theta_e: f64,
    last_lambda: f64,
}

/// Amplitude-invariant Clarke of three phase values (EQ-CONV-01).
pub fn clarke(a: f64, b: f64, c: f64) -> (f64, f64) {
    ((2.0 * a - b - c) / 3.0, (b - c) / SQRT3)
}

/// Inverse Clarke (EQ-CONV-02), zero common mode.
pub fn inv_clarke(al: f64, be: f64) -> [f64; 3] {
    [
        al,
        -0.5 * al + 0.5 * SQRT3 * be,
        -0.5 * al - 0.5 * SQRT3 * be,
    ]
}

impl MotorElectrical {
    /// Registers the motor's electrical output signals (signals.md).
    pub fn new(
        p: ElectricalParams,
        inp: ElectricalInputs,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let mut r = |path: &str, unit: &str, d: &str, k: SignalKind| bus.register(path, unit, d, k);
        let o = SignalKind::Output;
        let out = Out {
            psi_alpha: r(
                "motor.psi_alpha",
                "Wb",
                "flux linkage α (state)",
                SignalKind::State,
            )?,
            psi_beta: r(
                "motor.psi_beta",
                "Wb",
                "flux linkage β (state)",
                SignalKind::State,
            )?,
            i: [
                r("motor.i_a", "A", "phase a current", o)?,
                r("motor.i_b", "A", "phase b current", o)?,
                r("motor.i_c", "A", "phase c current", o)?,
            ],
            i_alpha: r("motor.i_alpha", "A", "α current", o)?,
            i_beta: r("motor.i_beta", "A", "β current", o)?,
            i_d: r("motor.i_d", "A", "d-axis current", o)?,
            i_q: r("motor.i_q", "A", "q-axis current", o)?,
            e: [
                r("motor.e_a", "V", "back-EMF a", o)?,
                r("motor.e_b", "V", "back-EMF b", o)?,
                r("motor.e_c", "V", "back-EMF c", o)?,
            ],
            v_n: r("motor.v_n", "V", "neutral voltage", o)?,
            torque: r("motor.torque_em", "N*m", "electromagnetic torque", o)?,
            p_cu: r("motor.p_cu", "W", "copper loss", o)?,
        };
        let lambda = p.lambda;
        Ok(Self {
            last_theta_e: p.theta_e0,
            p,
            inp,
            out,
            temp: None,
            jump: 0.0,
            last_lambda: lambda,
        })
    }

    /// Enable λ(T) from a magnet-temperature signal (wired by P06).
    pub fn with_lambda_temperature(mut self, t: LambdaTemperature) -> Self {
        self.temp = Some(t);
        self
    }

    fn lambda(&self, bus: &SignalBus) -> f64 {
        match self.temp {
            Some(t) => self.p.lambda * (1.0 + t.alpha_br * (bus.get(t.t_magnet) - t.t_ref)),
            None => self.p.lambda,
        }
    }

    /// Harmonic magnet flux in αβ: Clarke of λ·Φ_h per phase (EQ-MOT-02).
    fn harmonic_flux(&self, th: f64, lambda: f64) -> (f64, f64) {
        if self.p.shape.is_sinusoidal() {
            return (0.0, 0.0);
        }
        let f = |x: f64| lambda * self.p.shape.phi_h(th - x * TWO_PI_3);
        clarke(f(0.0), f(1.0), f(2.0))
    }

    /// EQ-MOT-02, linear magnetics: states → currents.
    pub fn solve(&self, psi_a: f64, psi_b: f64, th: f64, lambda: f64) -> Solved {
        let (ha, hb) = self.harmonic_flux(th, lambda);
        let (a, b) = (psi_a - ha, psi_b - hb);
        let (s, c) = th.sin_cos();
        let psi_d = c * a + s * b;
        let psi_q = -s * a + c * b;
        let i_d = (psi_d - lambda) / self.p.ld;
        let i_q = psi_q / self.p.lq;
        Solved {
            i_alpha: c * i_d - s * i_q,
            i_beta: s * i_d + c * i_q,
            i_d,
            i_q,
            psi_d,
            psi_q,
        }
    }

    /// EQ-MOT-07 torque at a solved state.
    pub fn torque(&self, s: &Solved, th: f64, lambda: f64) -> f64 {
        let p = self.p.pole_pairs;
        let mut t = 1.5 * p * (s.psi_d * s.i_q - s.psi_q * s.i_d);
        if !self.p.shape.is_sinusoidal() {
            let i = inv_clarke(s.i_alpha, s.i_beta);
            t += p
                * lambda
                * (0..3)
                    .map(|x| self.p.shape.k_h(th - x as f64 * TWO_PI_3) * i[x])
                    .sum::<f64>();
        }
        t
    }

    /// Magnetic stored energy (EQ-MOT-10 linear limit): ¾(L_d i_d² + L_q i_q²).
    fn w_mag(&self, s: &Solved) -> f64 {
        0.75 * (self.p.ld * s.i_d * s.i_d + self.p.lq * s.i_q * s.i_q)
    }

    fn state(&self, x: &[f64], off: usize, bus: &SignalBus) -> (Solved, f64, f64) {
        let th = bus.get(self.inp.theta_e);
        let lambda = self.lambda(bus);
        (self.solve(x[off], x[off + 1], th, lambda), th, lambda)
    }

    fn v_alpha_beta(&self, bus: &SignalBus) -> (f64, f64) {
        let [a, b, c] = self.inp.v.map(|id| bus.get(id));
        clarke(a, b, c)
    }
}

impl PlantModule for MotorElectrical {
    fn name(&self) -> &str {
        "motor"
    }
    fn n_states(&self) -> usize {
        2
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![
            ("psi_alpha".into(), "Wb".into()),
            ("psi_beta".into(), "Wb".into()),
        ]
    }
    fn init(&self, x: &mut [f64]) {
        let p = &self.p;
        let th = p.theta_e0;
        let (s, c) = th.sin_cos();
        let (pd, pq) = (p.ld * p.i_d0 + p.lambda, p.lq * p.i_q0);
        let (ha, hb) = self.harmonic_flux(th, p.lambda);
        x[0] = c * pd - s * pq + ha;
        x[1] = s * pd + c * pq + hb;
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let (s, th, lambda) = self.state(x, off, bus);
        self.last_theta_e = th;
        self.last_lambda = lambda;
        let we = bus.get(self.inp.omega_e);
        let i = inv_clarke(s.i_alpha, s.i_beta);
        let e: [f64; 3] =
            std::array::from_fn(|k| we * lambda * self.p.shape.k(th - k as f64 * TWO_PI_3));
        let vt = self.inp.v.map(|id| bus.get(id));
        let o = self.out;
        bus.set(o.psi_alpha, x[off]);
        bus.set(o.psi_beta, x[off + 1]);
        for k in 0..3 {
            bus.set(o.i[k], i[k]);
            bus.set(o.e[k], e[k]);
        }
        bus.set(o.i_alpha, s.i_alpha);
        bus.set(o.i_beta, s.i_beta);
        bus.set(o.i_d, s.i_d);
        bus.set(o.i_q, s.i_q);
        // EQ-MOT-04 (display only).
        bus.set(
            o.v_n,
            (vt.iter().sum::<f64>() - e.iter().sum::<f64>()) / 3.0,
        );
        bus.set(o.torque, self.torque(&s, th, lambda));
        bus.set(
            o.p_cu,
            1.5 * self.p.r * (s.i_alpha * s.i_alpha + s.i_beta * s.i_beta),
        );
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        let (s, _, _) = self.state(x, off, bus);
        let (va, vb) = self.v_alpha_beta(bus);
        // EQ-MOT-01.
        dx[0] = va - self.p.r * s.i_alpha;
        dx[1] = vb - self.p.r * s.i_beta;
    }
    fn params(&self) -> Vec<(String, String)> {
        [
            ("electrical.r_phase", "ohm"),
            ("electrical.l_d", "H"),
            ("electrical.l_q", "H"),
            ("electrical.lambda_m", "Wb"),
        ]
        .iter()
        .map(|(n, u)| ((*n).into(), (*u).into()))
        .collect()
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        let p = &self.p;
        Some(match name {
            "electrical.r_phase" => p.r,
            "electrical.l_d" => p.ld,
            "electrical.l_q" => p.lq,
            "electrical.lambda_m" => p.lambda,
            _ => return None,
        })
    }
    fn set_param(&mut self, name: &str, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        if !(v.is_finite() && v > 0.0) {
            return Err(format!("`{name}` must be finite and positive, got {v}"));
        }
        // Flux is continuous; currents (and W_mag) jump with L or λ. Book the jump.
        let (th, lam_now) = (self.last_theta_e, self.last_lambda);
        let before = self.w_mag(&self.solve(x_own[0], x_own[1], th, lam_now));
        let scale = lam_now / self.p.lambda;
        let p = &mut self.p;
        let old = match name {
            "electrical.r_phase" => std::mem::replace(&mut p.r, v),
            "electrical.l_d" => std::mem::replace(&mut p.ld, v),
            "electrical.l_q" => std::mem::replace(&mut p.lq, v),
            "electrical.lambda_m" => std::mem::replace(&mut p.lambda, v),
            _ => return Err(format!("unknown parameter `{name}`")),
        };
        let lam_new = self.p.lambda * scale;
        self.last_lambda = lam_new;
        let after = self.w_mag(&self.solve(x_own[0], x_own[1], th, lam_new));
        self.jump += after - before;
        Ok(old)
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![
            ("electrical_in".into(), PowerKind::Input),
            ("copper".into(), PowerKind::Loss),
        ]
    }
    fn powers(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, p: &mut [f64]) {
        let (s, _, _) = self.state(x, off, bus);
        let (va, vb) = self.v_alpha_beta(bus);
        // EQ-MOT-13 (amplitude-invariant: factor 3/2).
        p[0] = 1.5 * (va * s.i_alpha + vb * s.i_beta);
        p[1] = 1.5 * self.p.r * (s.i_alpha * s.i_alpha + s.i_beta * s.i_beta);
    }
    fn stored_energy(&self, x: &[f64], off: usize, bus: &SignalBus) -> f64 {
        let (s, _, _) = self.state(x, off, bus);
        self.w_mag(&s)
    }
    fn take_external_energy(&mut self) -> f64 {
        std::mem::take(&mut self.jump)
    }
    fn save(&self) -> serde_json::Value {
        let p = &self.p;
        serde_json::json!({ "r": p.r, "ld": p.ld, "lq": p.lq, "lambda": p.lambda })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        let (r, ld, lq, lambda) = (f("r")?, f("ld")?, f("lq")?, f("lambda")?);
        (self.p.r, self.p.ld, self.p.lq, self.p.lambda) = (r, ld, lq, lambda);
        Ok(())
    }
}
