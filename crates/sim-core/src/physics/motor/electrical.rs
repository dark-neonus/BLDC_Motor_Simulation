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
    /// Co-energy saturation (EQ-MOT-10, Detailed tier); `None` = linear magnetics.
    pub saturation: Option<SatParams>,
}

/// EQ-MOT-10 parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SatParams {
    /// Knee current i_k [A].
    pub i_k: f64,
    /// Incremental q inductance deep in saturation L_∞ [H].
    pub l_inf: f64,
    /// Cross-saturation fraction c (0 ≤ c < 1).
    pub c: f64,
}

/// Rotor-frame magnetics: ψ_dq(i_dq), its Jacobian and the co-energy (EQ-MOT-10; the
/// linear model is the c = 0, L_∞ = L_q limit).
#[derive(Debug, Clone, Copy)]
struct Magnetics {
    ld: f64,
    lq: f64,
    lambda: f64,
    sat: Option<SatParams>,
}

impl Magnetics {
    /// (h, h', h'') of the cross-saturation function.
    fn h(&self, iq: f64) -> (f64, f64, f64) {
        match self.sat {
            None => (0.0, 0.0, 0.0),
            Some(s) => {
                let u = iq / s.i_k;
                let d = 1.0 + u * u;
                (
                    s.c * u * u / d,
                    s.c * 2.0 * u / (s.i_k * d * d),
                    s.c * 2.0 * (1.0 - 3.0 * u * u) / (s.i_k * s.i_k * d * d * d),
                )
            }
        }
    }

    fn psi(&self, id: f64, iq: f64) -> (f64, f64) {
        let (h, hp, _) = self.h(iq);
        let psi_q = match self.sat {
            None => self.lq * iq,
            Some(s) => {
                s.l_inf * iq + (self.lq - s.l_inf) * s.i_k * (iq / s.i_k).tanh()
                    - self.lambda * id * hp
            }
        };
        (self.lambda * (1.0 - h) + self.ld * id, psi_q)
    }

    /// Co-energy W'(i_d, i_q).
    fn coenergy(&self, id: f64, iq: f64) -> f64 {
        let (h, _, _) = self.h(iq);
        let q = match self.sat {
            None => 0.5 * self.lq * iq * iq,
            Some(s) => {
                let u = iq / s.i_k;
                // ln cosh(u) without overflow for large |u|.
                let lncosh = u.abs() + (-2.0 * u.abs()).exp().ln_1p() - std::f64::consts::LN_2;
                0.5 * s.l_inf * iq * iq + (self.lq - s.l_inf) * s.i_k * s.i_k * lncosh
            }
        };
        self.lambda * id * (1.0 - h) + 0.5 * self.ld * id * id + q
    }

    /// Solve ψ_dq(i) = (pd, pq) for i (closed form when linear, Newton otherwise).
    fn currents(&self, pd: f64, pq: f64) -> (f64, f64) {
        let lin = ((pd - self.lambda) / self.ld, pq / self.lq);
        let Some(s) = self.sat else { return lin };
        // Start from the linear solution every time: stateless, so snapshots restore
        // bit-identically. The Jacobian is the (symmetric) Hessian of W'.
        let (mut id, mut iq) = lin;
        // A q-current guess from the saturated asymptote converges faster deep in saturation.
        if iq.abs() > s.i_k {
            iq = (pq - (self.lq - s.l_inf) * s.i_k * iq.signum()) / s.l_inf;
        }
        // Damped Newton: halve the step until the flux residual decreases, so the
        // iteration converges from the linear guess even deep in saturation.
        let res = |id: f64, iq: f64| {
            let (fd, fq) = self.psi(id, iq);
            ((fd - pd).hypot(fq - pq), fd - pd, fq - pq)
        };
        let (mut norm, mut rd, mut rq) = res(id, iq);
        let scale = 1e-15 * (1.0 + pd.abs() + pq.abs());
        for _ in 0..100 {
            if norm <= scale {
                break;
            }
            let (_, hp, hpp) = self.h(iq);
            let sech2 = 1.0 / (iq / s.i_k).cosh().powi(2);
            let (a, b, d) = (
                self.ld,
                -self.lambda * hp,
                s.l_inf + (self.lq - s.l_inf) * sech2 - self.lambda * id * hpp,
            );
            let det = a * d - b * b;
            if !(det.is_finite() && det > 0.0) {
                break; // outside the validated (positive-definite) range
            }
            let (did, diq) = ((d * rd - b * rq) / det, (a * rq - b * rd) / det);
            let mut t = 1.0;
            loop {
                let (n2, d2, q2) = res(id - t * did, iq - t * diq);
                if n2 < norm || t < 1e-6 {
                    (id, iq, norm, rd, rq) = (id - t * did, iq - t * diq, n2, d2, q2);
                    break;
                }
                t *= 0.5;
            }
        }
        (id, iq)
    }
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
    open_phase: SignalId,
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
    /// Open-phase mode (EQ-MOT-11): the disconnected phase; state 0 is then the line
    /// flux ψ_yz of the two connected phases (y = k+1, z = k+2 mod 3).
    open: Option<usize>,
    /// Requested open phase, entered when its current crosses zero (state event), and
    /// the sign of that current at the request (so g starts positive: no false crossing).
    pending: Option<usize>,
    pending_sign: f64,
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
            open_phase: r(
                "motor.open_phase",
                "-",
                "open phase: -1 none, 0/1/2 = a/b/c",
                o,
            )?,
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
            open: None,
            pending: None,
            pending_sign: 1.0,
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

    fn magnetics(&self, lambda: f64) -> Magnetics {
        Magnetics {
            ld: self.p.ld,
            lq: self.p.lq,
            lambda,
            sat: self.p.saturation,
        }
    }

    /// EQ-MOT-02: states → currents (rotor-frame inversion of the magnetics).
    pub fn solve(&self, psi_a: f64, psi_b: f64, th: f64, lambda: f64) -> Solved {
        let (ha, hb) = self.harmonic_flux(th, lambda);
        let (a, b) = (psi_a - ha, psi_b - hb);
        let (s, c) = th.sin_cos();
        let psi_d = c * a + s * b;
        let psi_q = -s * a + c * b;
        let (i_d, i_q) = self.magnetics(lambda).currents(psi_d, psi_q);
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

    /// Magnetic stored energy (EQ-MOT-10): 3/2·(ψ_d i_d + ψ_q i_q − W'); the linear case
    /// gives ¾(L_d i_d² + L_q i_q²).
    fn w_mag(&self, s: &Solved, lambda: f64) -> f64 {
        let m = self.magnetics(lambda);
        1.5 * (s.psi_d * s.i_d + s.psi_q * s.i_q - m.coenergy(s.i_d, s.i_q))
    }

    fn state(&self, x: &[f64], off: usize, bus: &SignalBus) -> (Solved, f64, f64) {
        let th = bus.get(self.inp.theta_e);
        let lambda = self.lambda(bus);
        let s = match self.open {
            None => self.solve(x[off], x[off + 1], th, lambda),
            Some(k) => self.solve_open(x[off], th, lambda, k),
        };
        (s, th, lambda)
    }

    /// Stator flux αβ for given rotor-frame currents (EQ-MOT-02 forward direction).
    fn flux_ab(&self, i_d: f64, i_q: f64, th: f64, lambda: f64) -> (f64, f64, f64, f64) {
        let (pd, pq) = self.magnetics(lambda).psi(i_d, i_q);
        let (s, c) = th.sin_cos();
        let (ha, hb) = self.harmonic_flux(th, lambda);
        (c * pd - s * pq + ha, s * pd + c * pq + hb, pd, pq)
    }

    /// Single current path y → z with phase k open (EQ-MOT-11): state = ψ_yz.
    pub fn solve_open(&self, psi_yz: f64, th: f64, lambda: f64, k: usize) -> Solved {
        let (y, z) = ((k + 1) % 3, (k + 2) % 3);
        let (sn, cs) = th.sin_cos();
        let at = |i: f64| {
            let mut ph = [0.0; 3];
            ph[y] = i;
            ph[z] = -i;
            let (ia, ib) = clarke(ph[0], ph[1], ph[2]);
            let (id, iq) = (cs * ia + sn * ib, -sn * ia + cs * ib);
            let (fa, fb, pd, pq) = self.flux_ab(id, iq, th, lambda);
            let f = inv_clarke(fa, fb);
            (
                f[y] - f[z],
                Solved {
                    i_alpha: ia,
                    i_beta: ib,
                    i_d: id,
                    i_q: iq,
                    psi_d: pd,
                    psi_q: pq,
                },
            )
        };
        // Newton on the line flux, starting from the linear non-salient estimate
        // ψ_yz ≈ 2·L·i + magnet part.
        let l2 = self.p.ld + self.p.lq;
        let (f0, _) = at(0.0);
        let mut i = (psi_yz - f0) / l2;
        for _ in 0..50 {
            let (f, _) = at(i);
            let h = 1e-6 * (1.0 + i.abs());
            let slope = (at(i + h).0 - at(i - h).0) / (2.0 * h);
            if !(slope.is_finite() && slope > 0.0) {
                break;
            }
            let di = (f - psi_yz) / slope;
            i -= di;
            if di.abs() <= 1e-13 * (1.0 + i.abs()) {
                break;
            }
        }
        at(i).1
    }

    /// The state quantities for imposed stationary-frame currents (for analyses with
    /// ideal current sources, e.g. six-step torque ripple).
    pub fn from_currents(&self, i_alpha: f64, i_beta: f64, th: f64, lambda: f64) -> Solved {
        let (sn, cs) = th.sin_cos();
        let (i_d, i_q) = (cs * i_alpha + sn * i_beta, -sn * i_alpha + cs * i_beta);
        let (psi_d, psi_q) = self.magnetics(lambda).psi(i_d, i_q);
        Solved {
            i_alpha,
            i_beta,
            i_d,
            i_q,
            psi_d,
            psi_q,
        }
    }

    /// Phase k current from a solved state.
    fn phase_current(s: &Solved, k: usize) -> f64 {
        inv_clarke(s.i_alpha, s.i_beta)[k]
    }

    /// `motor.open_phase`: −1 reconnects, 0/1/2 opens a/b/c at its next current zero.
    fn set_open_phase(&mut self, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        let old = self.open.or(self.pending).map_or(-1.0, |k| k as f64);
        if v == -1.0 {
            self.pending = None;
            if let Some(k) = self.open.take() {
                // Rebuild ψαβ from the current state (continuous, EQ-MOT-12).
                let (th, lam) = (self.last_theta_e, self.last_lambda);
                let s = self.solve_open(x_own[0], th, lam, k);
                let (pa, pb, _, _) = self.flux_ab(s.i_d, s.i_q, th, lam);
                (x_own[0], x_own[1]) = (pa, pb);
            }
            return Ok(old);
        }
        if !matches!(v, 0.0 | 1.0 | 2.0) {
            return Err(format!("open_phase must be -1, 0, 1 or 2, got {v}"));
        }
        if self.open.is_some() {
            return Err("a phase is already open; reconnect it first".into());
        }
        let k = v as usize;
        let s = self.solve(x_own[0], x_own[1], self.last_theta_e, self.last_lambda);
        self.pending_sign = if Self::phase_current(&s, k) < 0.0 {
            -1.0
        } else {
            1.0
        };
        self.pending = Some(k);
        Ok(old)
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
        let (pd, pq) = self.magnetics(p.lambda).psi(p.i_d0, p.i_q0);
        let (ha, hb) = self.harmonic_flux(th, p.lambda);
        x[0] = c * pd - s * pq + ha;
        x[1] = s * pd + c * pq + hb;
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let (s, th, lambda) = self.state(x, off, bus);
        self.last_theta_e = th;
        self.last_lambda = lambda;
        let we = bus.get(self.inp.omega_e);
        let mut i = inv_clarke(s.i_alpha, s.i_beta);
        if let Some(k) = self.open {
            // Exact single-path currents (the αβ round trip leaves ~1e-16 in phase k).
            let (y, z) = ((k + 1) % 3, (k + 2) % 3);
            (i[k], i[z]) = (0.0, -i[y]);
        }
        let e: [f64; 3] =
            std::array::from_fn(|k| we * lambda * self.p.shape.k(th - k as f64 * TWO_PI_3));
        let vt = self.inp.v.map(|id| bus.get(id));
        let o = self.out;
        let (pa, pb, _, _) = self.flux_ab(s.i_d, s.i_q, th, lambda);
        bus.set(o.psi_alpha, pa);
        bus.set(o.psi_beta, pb);
        for k in 0..3 {
            bus.set(o.i[k], i[k]);
            bus.set(o.e[k], e[k]);
        }
        bus.set(o.i_alpha, s.i_alpha);
        bus.set(o.i_beta, s.i_beta);
        bus.set(o.i_d, s.i_d);
        bus.set(o.i_q, s.i_q);
        // EQ-MOT-04 (display only); open phase: EQ-MOT-12 from the connected pair.
        let v_n = match self.open {
            None => (vt.iter().sum::<f64>() - e.iter().sum::<f64>()) / 3.0,
            Some(k) => {
                let (y, z) = ((k + 1) % 3, (k + 2) % 3);
                0.5 * (vt[y] + vt[z] - e[y] - e[z])
            }
        };
        bus.set(o.v_n, v_n);
        bus.set(o.open_phase, self.open.map_or(-1.0, |k| k as f64));
        bus.set(o.torque, self.torque(&s, th, lambda));
        bus.set(
            o.p_cu,
            1.5 * self.p.r * (s.i_alpha * s.i_alpha + s.i_beta * s.i_beta),
        );
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        let (s, _, _) = self.state(x, off, bus);
        if let Some(k) = self.open {
            // EQ-MOT-11: dψ_yz/dt = v_yT − v_zT − 2R·i.
            let (y, z) = ((k + 1) % 3, (k + 2) % 3);
            let i = Self::phase_current(&s, y);
            dx[0] = bus.get(self.inp.v[y]) - bus.get(self.inp.v[z]) - 2.0 * self.p.r * i;
            dx[1] = 0.0;
            return;
        }
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
            ("open_phase", "-"),
        ]
        .iter()
        .map(|(n, u)| ((*n).into(), (*u).into()))
        .collect()
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        let p = &self.p;
        Some(match name {
            "open_phase" => self.open.or(self.pending).map_or(-1.0, |k| k as f64),
            "electrical.r_phase" => p.r,
            "electrical.l_d" => p.ld,
            "electrical.l_q" => p.lq,
            "electrical.lambda_m" => p.lambda,
            _ => return None,
        })
    }
    fn set_param(&mut self, name: &str, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        if name == "open_phase" {
            return self.set_open_phase(v, x_own);
        }
        if self.open.is_some() {
            return Err("reconnect the open phase before changing electrical parameters".into());
        }
        if !(v.is_finite() && v > 0.0) {
            return Err(format!("`{name}` must be finite and positive, got {v}"));
        }
        // Flux is continuous; currents (and W_mag) jump with L or λ. Book the jump.
        let (th, lam_now) = (self.last_theta_e, self.last_lambda);
        let before = self.w_mag(&self.solve(x_own[0], x_own[1], th, lam_now), lam_now);
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
        let after = self.w_mag(&self.solve(x_own[0], x_own[1], th, lam_new), lam_new);
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
        let (s, _, lambda) = self.state(x, off, bus);
        self.w_mag(&s, lambda)
    }
    fn take_external_energy(&mut self) -> f64 {
        std::mem::take(&mut self.jump)
    }
    fn n_events(&self) -> usize {
        1
    }
    fn event_functions(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, g: &mut [f64]) {
        // The pending phase's current; constant (no crossing) otherwise.
        g[0] = match self.pending {
            Some(k) => self.pending_sign * Self::phase_current(&self.state(x, off, bus).0, k),
            None => 1.0,
        };
    }
    fn on_event(
        &mut self,
        _idx: usize,
        _rising: bool,
        _t: f64,
        x_own: &mut [f64],
        _bus: &mut SignalBus,
    ) {
        let Some(k) = self.pending.take() else { return };
        let (th, lam) = (self.last_theta_e, self.last_lambda);
        let before = self.solve(x_own[0], x_own[1], th, lam);
        // ψ_yz from the phase fluxes (zero sequence cancels in the difference).
        let (y, z) = ((k + 1) % 3, (k + 2) % 3);
        let f = inv_clarke(x_own[0], x_own[1]);
        x_own[0] = f[y] - f[z];
        x_own[1] = 0.0;
        self.open = Some(k);
        // i_k ≈ 0 at the event (bisection tolerance): book the tiny W_mag difference.
        let after = self.solve_open(x_own[0], th, lam, k);
        self.jump += self.w_mag(&after, lam) - self.w_mag(&before, lam);
    }
    fn save(&self) -> serde_json::Value {
        let p = &self.p;
        serde_json::json!({ "r": p.r, "ld": p.ld, "lq": p.lq, "lambda": p.lambda, "open": self.open, "pending": self.pending })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        let (r, ld, lq, lambda) = (f("r")?, f("ld")?, f("lq")?, f("lambda")?);
        let k = |key: &str| -> Result<Option<usize>, String> {
            serde_json::from_value(v.get(key).cloned().unwrap_or_default())
                .map_err(|e| e.to_string())
        };
        let (open, pending) = (k("open")?, k("pending")?);
        (self.p.r, self.p.ld, self.p.lq, self.p.lambda) = (r, ld, lq, lambda);
        (self.open, self.pending) = (open, pending);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sat() -> Magnetics {
        Magnetics {
            ld: 2.5e-3,
            lq: 2.5e-3,
            lambda: 0.03,
            sat: Some(SatParams {
                i_k: 10.0,
                l_inf: 0.75e-3,
                c: 0.15,
            }),
        }
    }

    #[test]
    fn newton_inverts_the_saturated_flux() {
        let m = sat();
        for &(id, iq) in &[
            (0.0, 0.0),
            (0.0, 5.0),
            (-3.0, 25.0),
            (2.0, -40.0),
            (-10.0, 60.0),
        ] {
            let (pd, pq) = m.psi(id, iq);
            let (a, b) = m.currents(pd, pq);
            // Newton to 1e-13 relative step: recovered currents within 1e-9 A.
            assert!(
                (a - id).abs() < 1e-9 && (b - iq).abs() < 1e-9,
                "({id}, {iq}) → ({a}, {b})"
            );
        }
    }

    #[test]
    fn flux_is_the_gradient_of_the_coenergy() {
        let m = sat();
        let (id, iq, h) = (-2.0, 17.0, 1e-5);
        let (pd, pq) = m.psi(id, iq);
        let dd = (m.coenergy(id + h, iq) - m.coenergy(id - h, iq)) / (2.0 * h);
        let dq = (m.coenergy(id, iq + h) - m.coenergy(id, iq - h)) / (2.0 * h);
        // Central difference at h = 1e-5 A: atol 1e-9 Wb.
        assert!(
            (dd - pd).abs() < 1e-9 && (dq - pq).abs() < 1e-9,
            "{dd} {pd} / {dq} {pq}"
        );
    }
}
