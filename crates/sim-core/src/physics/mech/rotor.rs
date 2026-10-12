//! The drivetrain (D-015): rotor, gearbox, load, friction and the disturbance pulse.
//!
//! **Rigid** (EQ-MECH-02/03): one degree of freedom θm/ωm on the motor side,
//! J_eq·dω/dt = T_a + T_f + T_gl,  T_a = Σ T_internal + Σ T_external + T_L/N,
//! J_eq = J_m + J_gi + (J_go + J_load)/N²,  θ_L = θm/N; friction is lumped on ω_m.
//!
//! **Backlash** (EQ-MECH-04, chosen at build): the load side gets θ_L/ω_L, coupled by a
//! push-only dead-zone contact τ_c(δ, δ̇), δ = θm/N − θ_L:
//! (J_m + J_gi)·dωm/dt = T_motor − τ_c/N + T_f,m + T_gl,  (J_go + J_load)·dω_L/dt = τ_c + T_L + T_f,L,
//! with T_gl from τ_in = τ_c/N, and each side keeping its own friction element.
//!
//! Torques arrive as bus signals. *Internal* torques (electromagnetic, cogging, iron
//! drag) move energy between modules and need no power term here: the module that
//! produces them accounts for its side. *External* torques do work on the system and
//! are reported as `External` power. Load torques are computed here and booked by kind
//! (`load::LoadTorques`).
//!
//! States: θm, ωm, disturbance time left, [θ_L, ω_L with backlash].

use crate::energy::PowerKind;
use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

use super::backlash::Backlash;
use super::friction::{self, Friction, Mode};
use super::gearbox::Gearbox;
use super::load::{Load, LoadTorques};

#[derive(Debug, Clone, Copy)]
pub struct RotorParams {
    /// Rotor inertia [kg·m²] (`motor.mechanical.j_rotor`).
    pub j: f64,
    /// Extra fixed inertia on the motor shaft [kg·m²] (tests, couplings).
    pub j_load: f64,
    /// Viscous friction [N·m·s/rad].
    pub b: f64,
    /// Pole pairs (θe = p·θm).
    pub pole_pairs: f64,
    pub theta0: f64,
    pub omega0: f64,
}

struct Ids {
    theta: SignalId,
    omega: SignalId,
    theta_e: SignalId,
    omega_e: SignalId,
    theta_l: SignalId,
    omega_l: SignalId,
    torque_gravity: SignalId,
    torque_ext: SignalId,
    p_gear: SignalId,
    disturbance: SignalId,
    delta: SignalId,
    torque_contact: SignalId,
    p_fric: SignalId,
}

/// One friction element's evaluation.
#[derive(Clone, Copy)]
struct Side {
    /// Applied torque except this element's friction (and the gear loss).
    t_a: f64,
    /// Friction torque.
    t_f: f64,
    held: bool,
    /// Acceleration of this side.
    a: f64,
}

/// One evaluation of the drivetrain dynamics.
struct Dyn {
    motor: Side,
    /// Load side (backlash only; rigid: unused).
    load_side: Side,
    /// Gear loss torque on the motor shaft.
    t_gl: f64,
    load: LoadTorques,
    /// Σ external motor-side torques.
    t_ext_m: f64,
    /// Contact deflection, its rate, and torque (backlash; else 0).
    delta: f64,
    ddelta: f64,
    tau_c: f64,
    /// Load speed.
    w_l: f64,
}

pub struct RotorRigid {
    p: RotorParams,
    gb: Gearbox,
    load: Load,
    /// EQ-MECH-09: a live load change keeps J·ω instead of ω.
    conserve_momentum: bool,
    locked: bool,
    internal: Vec<SignalId>,
    external: Vec<SignalId>,
    ids: Ids,
    /// Energy injected by parameter changes since the last `take_external_energy`.
    jump: f64,
    /// Disturbance pulse torque on the load shaft while one runs (EQ-MECH-08); its
    /// remaining time is state 2, and a state event ends it.
    dist: f64,
    /// Length of the last commanded disturbance pulse [s] (for `get_param`).
    dist_dur: f64,
    /// Motor-bearing static/Coulomb friction (its viscous part is `p.b`).
    fr_m: Friction,
    /// Load-side friction (lumped onto the motor shaft when rigid).
    fr_l: Friction,
    mode: Mode,
    /// Load-side friction mode (backlash only).
    mode_l: Mode,
    backlash: Option<Backlash>,
}

/// Start mode of a friction element from its initial speed.
fn start_mode(w: f64) -> Mode {
    if w != 0.0 {
        Mode::Slip { dir: w.signum() }
    } else {
        Mode::Stick
    }
}

/// One friction element: held, or accelerating under T_a + T_f over inertia `j`.
fn side(fr: &Friction, mode: Mode, t_a: f64, w: f64, j: f64) -> Side {
    let (t_f, held) = if fr.is_hybrid() {
        let e = friction::eval(fr, mode, t_a, w);
        (e.torque, e.held)
    } else {
        (-fr.b * w, false)
    };
    Side {
        t_a,
        t_f,
        held,
        a: if held { 0.0 } else { (t_a + t_f) / j },
    }
}

impl RotorRigid {
    /// Registers `motor.theta`, `motor.omega`, `motor.theta_e`, `motor.omega_e`, the
    /// load-side `load.theta`, `load.omega`, `load.torque_gravity`, `load.torque_ext`,
    /// `load.disturbance` and `gearbox.p_loss`, `gearbox.delta`, `gearbox.torque_contact`.
    /// `internal` / `external` are motor-side torque signals [N·m] that must already be
    /// registered.
    pub fn new(
        p: RotorParams,
        bus: &mut SignalBus,
        internal: Vec<SignalId>,
        external: Vec<SignalId>,
    ) -> Result<Self, SignalError> {
        let mut st =
            |path: &str, unit: &str, d: &str| bus.register(path, unit, d, SignalKind::State);
        let (theta, omega) = (
            st("motor.theta", "rad", "mechanical angle")?,
            st("motor.omega", "rad/s", "mechanical speed")?,
        );
        let (theta_e, omega_e) = (
            st("motor.theta_e", "rad", "electrical angle")?,
            st("motor.omega_e", "rad/s", "electrical speed")?,
        );
        let (theta_l, omega_l) = (
            st("load.theta", "rad", "load-side angle")?,
            st("load.omega", "rad/s", "load-side speed")?,
        );
        let mut out =
            |path: &str, unit: &str, d: &str| bus.register(path, unit, d, SignalKind::Output);
        let ids = Ids {
            theta,
            omega,
            theta_e,
            omega_e,
            theta_l,
            omega_l,
            torque_gravity: out("load.torque_gravity", "N*m", "gravity torque on the load")?,
            torque_ext: out(
                "load.torque_ext",
                "N*m",
                "load torque (constant/brake/viscous)",
            )?,
            p_gear: out("gearbox.p_loss", "W", "gearbox meshing loss")?,
            disturbance: out("load.disturbance", "N*m", "disturbance torque pulse")?,
            delta: out("gearbox.delta", "rad", "backlash deflection θm/N − θ_L")?,
            torque_contact: out("gearbox.torque_contact", "N*m", "gear contact torque")?,
            p_fric: out("motor.p_fric", "W", "bearing + load friction loss (heat)")?,
        };
        Ok(Self {
            p,
            gb: Gearbox::BYPASS,
            load: Load::None,
            conserve_momentum: false,
            locked: false,
            internal,
            external,
            ids,
            jump: 0.0,
            dist: 0.0,
            dist_dur: 0.0,
            fr_m: Friction::default(),
            fr_l: Friction::default(),
            mode: start_mode(p.omega0),
            mode_l: start_mode(p.omega0),
            backlash: None,
        })
    }

    /// Add an internal torque input registered after the rotor (the motor's torque
    /// needs the rotor's angle signals first).
    pub fn with_internal(mut self, id: SignalId) -> Self {
        self.internal.push(id);
        self
    }

    /// Start with the shaft held (ω = 0); `motor.mechanical.locked` releases it.
    pub fn with_locked(mut self, locked: bool) -> Self {
        self.locked = locked;
        if locked {
            self.mode = Mode::Stick;
        }
        self
    }

    pub fn with_gearbox(mut self, gb: Gearbox) -> Self {
        self.gb = gb;
        self
    }

    /// The output-shaft load; `conserve_momentum` selects the EQ-MECH-09 rule.
    pub fn with_load(mut self, load: Load, conserve_momentum: bool) -> Self {
        self.load = load;
        self.conserve_momentum = conserve_momentum;
        self
    }

    /// Stick–slip friction (EQ-MECH-05): motor static/Coulomb (the motor's viscous part
    /// is `RotorParams::b`) and load-side friction.
    pub fn with_friction(mut self, motor: Friction, load: Friction) -> Self {
        self.fr_m = motor;
        self.fr_l = load;
        self
    }

    /// Two-inertia backlash model (EQ-MECH-04): adds θ_L, ω_L. Chosen at build.
    pub fn with_backlash(mut self, b: Backlash) -> Self {
        self.backlash = Some(b);
        self
    }

    /// Motor-side inertia J_m + J_gi (backlash) [kg·m²].
    fn j_motor_side(&self) -> f64 {
        self.p.j + self.p.j_load + self.gb.j_in
    }

    /// Load-body inertia J_go + J_load (backlash) [kg·m²].
    fn j_load_side(&self) -> f64 {
        self.gb.j_out + self.load.inertia()
    }

    fn j_eq(&self) -> f64 {
        self.gb.j_eq(self.p.j + self.p.j_load, self.load.inertia())
    }

    /// Friction element on ω_m: lumped when rigid, the motor's own with backlash.
    fn fr_motor(&self) -> Friction {
        let motor = Friction {
            b: self.p.b,
            ..self.fr_m
        };
        if self.backlash.is_some() {
            motor
        } else {
            Friction::lumped(&motor, &self.fr_l, self.gb.n)
        }
    }

    /// Load angle and speed from the own states.
    fn load_state(&self, s: &[f64]) -> (f64, f64) {
        match self.backlash {
            Some(_) => (s[3], s[4]),
            None => (s[0] / self.gb.n, s[1] / self.gb.n),
        }
    }

    /// Drivetrain dynamics from the own state slice.
    fn dynamics(&self, s: &[f64], bus: &SignalBus) -> Dyn {
        let n = self.gb.n;
        let (th, w) = (s[0], s[1]);
        let (th_l, w_l) = self.load_state(s);
        let mut load = self.load.torques(th_l, w_l);
        load.ext += self.dist;
        let t_l = load.total();
        let t_ext_m: f64 = self.external.iter().map(|&id| bus.get(id)).sum();
        let t_int: f64 = self.internal.iter().map(|&id| bus.get(id)).sum();
        let idle = Side {
            t_a: 0.0,
            t_f: 0.0,
            held: true,
            a: 0.0,
        };
        let Some(bl) = self.backlash else {
            let j_eq = self.j_eq();
            let mut m = side(
                &self.fr_motor(),
                self.mode,
                t_int + t_ext_m + t_l / n,
                w,
                j_eq,
            );
            // Gear loss from the lossless solution (EQ-MECH-03, one pass); 0 while stuck. The
            // load's share of the lumped friction counts in T_L for τ_in (EQ-MECH-05
            // clarifications): −dir·T_c,L − b_L·ω_L on the load shaft while slipping.
            let t_gl = if m.held {
                0.0
            } else {
                let dir = match self.mode {
                    Mode::Slip { dir } => dir,
                    Mode::Stick => m.t_a.signum(),
                };
                let fr_share = -dir * self.fr_l.t_c - self.fr_l.b * w_l;
                self.gb
                    .loss_torque(m.a, t_l + fr_share, self.load.inertia(), w)
            };
            m.a += t_gl / j_eq;
            return Dyn {
                motor: m,
                load_side: idle,
                t_gl,
                load,
                t_ext_m,
                delta: 0.0,
                ddelta: 0.0,
                tau_c: 0.0,
                w_l,
            };
        };
        let (delta, ddelta) = (th / n - th_l, w / n - w_l);
        let tau_c = bl.torque(delta, ddelta);
        let j_m = self.j_motor_side();
        let mut m = side(
            &self.fr_motor(),
            self.mode,
            t_int + t_ext_m - tau_c / n,
            w,
            j_m,
        );
        let t_gl = if m.held {
            0.0
        } else {
            self.gb.loss_for(tau_c / n, w)
        };
        m.a += t_gl / j_m;
        let l = side(
            &self.fr_l,
            self.mode_l,
            tau_c + t_l,
            w_l,
            self.j_load_side(),
        );
        Dyn {
            motor: m,
            load_side: l,
            t_gl,
            load,
            t_ext_m,
            delta,
            ddelta,
            tau_c,
            w_l,
        }
    }

    /// Stored energy of the own states: kinetic + gravity + contact spring.
    fn energy(&self, s: &[f64]) -> f64 {
        let (th_l, w_l) = self.load_state(s);
        let w = s[1];
        let u = self.load.potential(th_l);
        match self.backlash {
            None => 0.5 * self.j_eq() * w * w + u,
            Some(bl) => {
                0.5 * self.j_motor_side() * w * w
                    + 0.5 * self.j_load_side() * w_l * w_l
                    + u
                    + bl.energy(s[0] / self.gb.n - th_l)
            }
        }
    }

    /// Apply a change to inertia-carrying parameters with the EQ-MECH-09 rule: keep ω
    /// (D-007) or, with `conserve`, keep the momentum of the body that changed (J_eq·ω
    /// rigid, the load body's with backlash). The stored-energy jump (kinetic, potential)
    /// is booked as external energy.
    fn change_inertia(
        &mut self,
        x_own: &mut [f64],
        conserve: bool,
        f: impl FnOnce(&mut Self) -> Result<f64, String>,
    ) -> Result<f64, String> {
        let e0 = self.energy(x_own);
        let (j0, jl0) = (self.j_eq(), self.j_load_side());
        let (gb0, load0, p0) = (self.gb, self.load.clone(), self.p);
        let old = f(self)?;
        if self.backlash.is_some() && self.j_load_side() <= 0.0 {
            // The two-inertia model needs mass behind the teeth (EQ-MECH-04).
            (self.gb, self.load, self.p) = (gb0, load0, p0);
            return Err("with backlash the load side needs inertia (J_go + J_load > 0)".into());
        }
        if conserve {
            match self.backlash {
                None => x_own[1] *= j0 / self.j_eq(),
                Some(_) => x_own[4] *= jl0 / self.j_load_side(),
            }
        }
        self.jump += self.energy(x_own) - e0;
        Ok(old)
    }

    fn set_friction(&mut self, load: bool, field: &str, v: f64) -> Result<f64, String> {
        Friction::check(field, v)?;
        let f = if load { &mut self.fr_l } else { &mut self.fr_m };
        let slot = match (field, load) {
            ("static_torque", _) => &mut f.t_s,
            ("coulomb", _) => &mut f.t_c,
            ("viscous", true) => &mut f.b,
            ("viscous", false) => &mut self.p.b,
            _ => return Err(format!("unknown friction parameter `{field}`")),
        };
        Ok(std::mem::replace(slot, v))
    }

    fn set_gear(&mut self, name: &str, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        match name {
            "gearbox.efficiency" => {
                if !(v > 0.0 && v <= 1.0) {
                    return Err(format!("efficiency must be in (0, 1], got {v}"));
                }
                Ok(std::mem::replace(&mut self.gb.eta, v))
            }
            "gearbox.j_in" | "gearbox.j_out" => {
                if !(v.is_finite() && v >= 0.0) {
                    return Err(format!("inertia must be ≥ 0, got {v}"));
                }
                let input = name == "gearbox.j_in";
                self.change_inertia(x_own, false, |s| {
                    let j = if input {
                        &mut s.gb.j_in
                    } else {
                        &mut s.gb.j_out
                    };
                    Ok(std::mem::replace(j, v))
                })
            }
            "gearbox.ratio" | "gearbox.backlash" => {
                Err(format!("`{name}` changes only on a rebuild (reset)"))
            }
            _ => Err(format!("unknown parameter `{name}`")),
        }
    }

    /// Friction events of one element (`sub` 0: ω crosses zero / breakaway, 1: stuck
    /// but moving): stick, slip, or reverse (EQ-MECH-05). `wi` is its speed state index.
    fn friction_event(&mut self, load_side: bool, sub: usize, x_own: &mut [f64], bus: &SignalBus) {
        let d = self.dynamics(x_own, bus);
        let (el, wi, j, t_s) = if load_side {
            (d.load_side, 4, self.j_load_side(), self.fr_l.t_s)
        } else {
            let j = if self.backlash.is_some() {
                self.j_motor_side()
            } else {
                self.j_eq()
            };
            (d.motor, 1, j, self.fr_motor().t_s)
        };
        let mode = if load_side {
            &mut self.mode_l
        } else {
            &mut self.mode
        };
        match (sub, *mode) {
            (0, Mode::Slip { .. }) => {
                if el.t_a.abs() <= t_s {
                    // Stick: the leftover ½Jω² (bisection-sized) leaves with the jump.
                    let w = x_own[wi];
                    self.jump -= 0.5 * j * w * w;
                    x_own[wi] = 0.0;
                    *mode = Mode::Stick;
                } else {
                    *mode = Mode::Slip {
                        dir: el.t_a.signum(),
                    };
                }
            }
            (0, Mode::Stick) => {
                *mode = Mode::Slip {
                    dir: el.t_a.signum(),
                }
            }
            (1, Mode::Stick) => {
                *mode = Mode::Slip {
                    dir: x_own[wi].signum(),
                }
            }
            _ => {}
        }
    }

    /// Event functions of one friction element (see `friction_event`).
    fn friction_g(&self, el: Side, mode: Mode, fr: &Friction, w: f64, g: &mut [f64]) {
        (g[0], g[1]) = (1.0, 1.0);
        if !fr.is_hybrid() {
            return;
        }
        match mode {
            Mode::Slip { dir } => g[0] = dir * w,
            Mode::Stick => {
                g[0] = fr.t_s - el.t_a.abs();
                g[1] = 1e-12 - w.abs();
            }
        }
    }
}

impl PlantModule for RotorRigid {
    fn name(&self) -> &str {
        "motor.mechanical"
    }
    fn param_roots(&self) -> Vec<String> {
        vec!["gearbox".into(), "load".into()]
    }
    fn n_states(&self) -> usize {
        if self.backlash.is_some() { 5 } else { 3 }
    }
    fn state_names(&self) -> Vec<(String, String)> {
        let mut v = vec![
            ("theta".into(), "rad".into()),
            ("omega".into(), "rad/s".into()),
            ("disturbance_left".into(), "s".into()),
        ];
        if self.backlash.is_some() {
            v.push(("theta_load".into(), "rad".into()));
            v.push(("omega_load".into(), "rad/s".into()));
        }
        v
    }
    fn init(&self, x: &mut [f64]) {
        let w0 = if self.locked { 0.0 } else { self.p.omega0 };
        x[0] = self.p.theta0;
        x[1] = w0;
        x[2] = 0.0;
        if self.backlash.is_some() {
            x[3] = self.p.theta0 / self.gb.n;
            x[4] = w0 / self.gb.n;
        }
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let s = &x[off..off + self.n_states()];
        let (th, w) = (s[0], s[1]);
        let (th_l, w_l) = self.load_state(s);
        let p = self.p.pole_pairs;
        let ids = &self.ids;
        bus.set(ids.theta, th);
        bus.set(ids.omega, w);
        bus.set(ids.theta_e, p * th);
        bus.set(ids.omega_e, p * w);
        bus.set(ids.theta_l, th_l);
        bus.set(ids.omega_l, w_l);
        // The motor torques on the bus are from the previous stage here (the motor's
        // outputs run after the rotor's): p_loss is for display; the energy book uses
        // `powers`, which runs after all outputs.
        let d = self.dynamics(s, bus);
        let ids = &self.ids;
        bus.set(ids.torque_gravity, d.load.gravity);
        bus.set(ids.torque_ext, d.load.loss + d.load.ext - self.dist);
        bus.set(ids.p_gear, if self.locked { 0.0 } else { -d.t_gl * w });
        bus.set(ids.disturbance, self.dist);
        bus.set(ids.delta, d.delta);
        bus.set(ids.torque_contact, d.tau_c);
        let w_m = if self.locked { 0.0 } else { w };
        bus.set(ids.p_fric, -d.motor.t_f * w_m - d.load_side.t_f * d.w_l);
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        let s = &x[off..off + self.n_states()];
        let d = self.dynamics(s, bus);
        dx[2] = if self.dist != 0.0 { -1.0 } else { 0.0 };
        if self.locked || d.motor.held {
            dx[0] = 0.0;
            dx[1] = 0.0;
        } else {
            dx[0] = s[1];
            dx[1] = d.motor.a;
        }
        if self.backlash.is_some() {
            dx[3] = if d.load_side.held { 0.0 } else { s[4] };
            dx[4] = d.load_side.a;
        }
    }
    fn params(&self) -> Vec<(String, String)> {
        let mut v: Vec<(String, String)> = [
            ("j_rotor", "kg*m^2"),
            ("friction.viscous", "N*m*s/rad"),
            ("friction.static_torque", "N*m"),
            ("friction.coulomb", "N*m"),
            ("locked", "-"),
            ("gearbox.efficiency", "-"),
            ("gearbox.j_in", "kg*m^2"),
            ("gearbox.j_out", "kg*m^2"),
            ("load.friction.static_torque", "N*m"),
            ("load.friction.coulomb", "N*m"),
            ("load.friction.viscous", "N*m*s/rad"),
            ("load.disturbance.torque", "N*m"),
            ("load.disturbance.duration", "s"),
        ]
        .iter()
        .map(|(a, b)| ((*a).into(), (*b).into()))
        .collect();
        v.extend(self.load.params());
        v
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match name {
            "j_rotor" => Some(self.p.j),
            "friction.viscous" => Some(self.p.b),
            "friction.static_torque" => Some(self.fr_m.t_s),
            "friction.coulomb" => Some(self.fr_m.t_c),
            "load.friction.static_torque" => Some(self.fr_l.t_s),
            "load.friction.coulomb" => Some(self.fr_l.t_c),
            "load.friction.viscous" => Some(self.fr_l.b),
            "locked" => Some(f64::from(u8::from(self.locked))),
            "gearbox.ratio" => Some(self.gb.n),
            "gearbox.efficiency" => Some(self.gb.eta),
            "gearbox.j_in" => Some(self.gb.j_in),
            "gearbox.j_out" => Some(self.gb.j_out),
            "gearbox.backlash" => Some(self.backlash.map_or(0.0, |b| b.width)),
            "gearbox.stiffness" => self.backlash.map(|b| b.k),
            "gearbox.damping" => self.backlash.map(|b| b.d),
            "load.disturbance.torque" => Some(self.dist),
            "load.disturbance.duration" => Some(self.dist_dur),
            _ => self.load.get(name),
        }
    }
    fn set_param(&mut self, name: &str, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        if name.starts_with("gearbox.") {
            return self.set_gear(name, v, x_own);
        }
        match name {
            "load.disturbance.duration" => {
                if !(v.is_finite() && v > 0.0) {
                    return Err(format!("duration must be > 0, got {v}"));
                }
                self.dist_dur = v;
                return Ok(std::mem::replace(&mut x_own[2], v));
            }
            "load.disturbance.torque" => {
                if !v.is_finite() {
                    return Err(format!("torque must be finite, got {v}"));
                }
                if x_own[2] <= 0.0 {
                    return Err("set load.disturbance.duration first".into());
                }
                return Ok(std::mem::replace(&mut self.dist, v));
            }
            _ => {}
        }
        if let Some(field) = name.strip_prefix("load.friction.") {
            return self.set_friction(true, field, v);
        }
        if name.starts_with("load.") {
            let conserve = self.conserve_momentum;
            let name = name.to_string();
            return self.change_inertia(x_own, conserve, |s| s.load.set(&name, v));
        }
        match name {
            "j_rotor" => {
                if !(v.is_finite() && v > 0.0) {
                    return Err(format!("inertia must be positive, got {v}"));
                }
                // D-007: keep ω; the kinetic-energy change is an external jump.
                self.change_inertia(x_own, false, |s| Ok(std::mem::replace(&mut s.p.j, v)))
            }
            "friction.viscous" | "friction.static_torque" | "friction.coulomb" => {
                self.set_friction(false, &name["friction.".len()..], v)
            }
            "locked" => {
                let old = f64::from(u8::from(self.locked));
                let lock = v != 0.0;
                if lock {
                    // Stopping the shaft removes its kinetic energy.
                    let e0 = self.energy(x_own);
                    x_own[1] = 0.0;
                    self.jump += self.energy(x_own) - e0;
                }
                self.locked = lock;
                // Locking stops the shaft; releasing starts from rest.
                self.mode = Mode::Stick;
                Ok(old)
            }
            _ => Err(format!("unknown parameter `{name}`")),
        }
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        vec![
            ("friction".into(), PowerKind::Loss),
            ("gearbox".into(), PowerKind::Loss),
            ("load".into(), PowerKind::Loss),
            ("backlash".into(), PowerKind::Loss),
            ("external".into(), PowerKind::External),
        ]
    }
    fn powers(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, p: &mut [f64]) {
        let s = &x[off..off + self.n_states()];
        let d = self.dynamics(s, bus);
        let w = if self.locked { 0.0 } else { s[1] };
        let w_l = if self.locked && self.backlash.is_none() {
            0.0
        } else {
            d.w_l
        };
        p[0] = -d.motor.t_f * w - d.load_side.t_f * w_l;
        p[1] = -d.t_gl * w;
        p[2] = -d.load.loss * w_l;
        p[3] = match self.backlash {
            Some(bl) => d.tau_c * d.ddelta - bl.energy_rate(d.delta, d.ddelta),
            None => 0.0,
        };
        p[4] = d.t_ext_m * w + d.load.ext * w_l;
    }
    fn stored_energy(&self, x: &[f64], off: usize, _bus: &SignalBus) -> f64 {
        self.energy(&x[off..off + self.n_states()])
    }
    fn take_external_energy(&mut self) -> f64 {
        std::mem::take(&mut self.jump)
    }
    fn n_events(&self) -> usize {
        6
    }
    fn event_functions(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, g: &mut [f64]) {
        let s = &x[off..off + self.n_states()];
        // 0: the pulse ends when its remaining time crosses zero.
        g[0] = if self.dist != 0.0 { s[2] } else { 1.0 };
        // 1, 2: the motor-shaft friction element; 3, 4: the load side (backlash).
        // Per element: slip → ω crosses zero; stick → |T_a| crosses T_s; stick but
        // already moving (a step started past breakaway: the engine re-evaluates g at
        // each advance, so an already-negative g would never cross).
        // 5: contact, |δ| − b/2 (backlash).
        g[1..6].fill(1.0);
        let fr_m = self.fr_motor();
        let hybrid_m = !self.locked && fr_m.is_hybrid();
        let hybrid_l = self.backlash.is_some() && self.fr_l.is_hybrid();
        if !(hybrid_m || hybrid_l || self.backlash.is_some()) {
            return; // nothing to watch: skip the dynamics evaluation
        }
        let d = self.dynamics(s, bus);
        if hybrid_m {
            self.friction_g(d.motor, self.mode, &fr_m, s[1], &mut g[1..3]);
        }
        if let Some(bl) = self.backlash {
            self.friction_g(d.load_side, self.mode_l, &self.fr_l, s[4], &mut g[3..5]);
            g[5] = d.delta.abs() - 0.5 * bl.width;
        }
    }
    fn on_event(
        &mut self,
        idx: usize,
        _rising: bool,
        _t: f64,
        x_own: &mut [f64],
        bus: &mut SignalBus,
    ) {
        match idx {
            0 => {
                self.dist = 0.0;
                x_own[2] = 0.0;
            }
            1 | 2 => self.friction_event(false, idx - 1, x_own, bus),
            3 | 4 => self.friction_event(true, idx - 3, x_own, bus),
            // 5: contact change; located only (the damping term makes τ_c jump by
            // d·|δ̇| at engagement, so the step must land on it).
            _ => {}
        }
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "j": self.p.j, "b": self.p.b, "locked": self.locked,
            "eta": self.gb.eta, "j_in": self.gb.j_in, "j_out": self.gb.j_out,
            "load": self.load, "jump": self.jump, "dist": self.dist, "dist_dur": self.dist_dur,
            "fr_m": self.fr_m, "fr_l": self.fr_l, "mode": self.mode, "mode_l": self.mode_l,
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        let de = |k: &str| v.get(k).cloned().ok_or(format!("missing `{k}`"));
        let (j, b, dist, dist_dur) = (f("j")?, f("b")?, f("dist")?, f("dist_dur")?);
        let (eta, j_in, j_out, jump) = (f("eta")?, f("j_in")?, f("j_out")?, f("jump")?);
        let locked = v
            .get("locked")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `locked`")?;
        let load: Load = serde_json::from_value(de("load")?).map_err(|e| format!("`load`: {e}"))?;
        let fr_m: Friction = serde_json::from_value(de("fr_m")?).map_err(|e| e.to_string())?;
        let fr_l: Friction = serde_json::from_value(de("fr_l")?).map_err(|e| e.to_string())?;
        let mode: Mode = serde_json::from_value(de("mode")?).map_err(|e| e.to_string())?;
        let mode_l: Mode = serde_json::from_value(de("mode_l")?).map_err(|e| e.to_string())?;
        (self.p.j, self.p.b, self.locked) = (j, b, locked);
        (self.gb.eta, self.gb.j_in, self.gb.j_out) = (eta, j_in, j_out);
        (self.load, self.jump, self.dist, self.dist_dur) = (load, jump, dist, dist_dur);
        (self.fr_m, self.fr_l, self.mode, self.mode_l) = (fr_m, fr_l, mode, mode_l);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::engine::Engine;
    use crate::engine::time::SimTime;

    /// Writes a constant torque on `load.torque_ext`.
    struct Push(SignalId, f64);
    impl PlantModule for Push {
        fn name(&self) -> &str {
            "test"
        }
        fn n_states(&self) -> usize {
            0
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![]
        }
        fn init(&self, _: &mut [f64]) {}
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
            bus.set(self.0, self.1);
        }
        fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
    }

    fn engine(t_ext: f64, p: RotorParams) -> Engine {
        engine_with(t_ext, p, Gearbox::BYPASS, Load::None, false)
    }

    fn engine_with(t_ext: f64, p: RotorParams, gb: Gearbox, load: Load, conserve: bool) -> Engine {
        let mut bus = SignalBus::new();
        let tq = bus
            .register("test.torque", "N*m", "test torque", SignalKind::Output)
            .unwrap();
        let rotor = RotorRigid::new(p, &mut bus, vec![], vec![tq])
            .unwrap()
            .with_gearbox(gb)
            .with_load(load, conserve);
        let mut plant = crate::engine::plant::Plant::new();
        plant.add(Box::new(Push(tq, t_ext)));
        plant.add(Box::new(rotor));
        Engine::new(plant, bus, vec![], 1e-4)
    }

    fn sig(e: &Engine, p: &str) -> f64 {
        e.bus.get(e.bus.id(p).unwrap())
    }

    #[test]
    fn constant_torque_accelerates_linearly() {
        let p = RotorParams {
            j: 2e-4,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 0.0,
        };
        let mut e = engine(0.01, p);
        e.step_until(SimTime::from_secs_f64(0.5)).unwrap();
        // ω = T·t/J = 0.01·0.5/2e-4 = 25 rad/s; θ = ½·T·t²/J. RK4 is exact for polynomials
        // of this degree, so only rounding remains (rtol 1e-12).
        assert!((sig(&e, "motor.omega") / 25.0 - 1.0).abs() < 1e-12);
        assert!((sig(&e, "motor.theta") / 6.25 - 1.0).abs() < 1e-12);
        assert!((sig(&e, "motor.theta_e") / (7.0 * 6.25) - 1.0).abs() < 1e-12);
        assert!(
            sig(&e, "energy.residual").abs() < 1e-9,
            "external work = kinetic energy"
        );
    }

    #[test]
    fn viscous_spin_down_is_exponential() {
        let p = RotorParams {
            j: 2e-4,
            j_load: 0.0,
            b: 1e-3,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 100.0,
        };
        let mut e = engine(0.0, p);
        e.step_until(SimTime::from_secs_f64(0.4)).unwrap();
        // τ = J/B = 0.2 s → ω(0.4) = 100·e^-2. RK4 at dt = 1e-4 ≪ τ: rtol 1e-9.
        let w = sig(&e, "motor.omega");
        assert!((w / (100.0 * (-2.0f64).exp()) - 1.0).abs() < 1e-9, "{w}");
        assert!(sig(&e, "energy.residual").abs() < 1e-9);
        assert!(sig(&e, "energy.loss.friction") > 0.0);
    }

    #[test]
    fn inertia_change_keeps_speed_and_books_the_energy() {
        let p = RotorParams {
            j: 2e-4,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0: 50.0,
        };
        let mut e = engine(0.0, p);
        e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
        e.queue(crate::engine::commands::EngineCommand::SetParam {
            path: "motor.mechanical.j_rotor".into(),
            value: 4e-4,
            source: crate::engine::commands::ChangeSource::Internal,
        });
        e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        assert_eq!(sig(&e, "motor.omega"), 50.0);
        assert!(sig(&e, "energy.residual").abs() < 1e-9);
    }

    const GB: Gearbox = Gearbox {
        n: 9.0,
        eta: 1.0,
        j_in: 3e-6,
        j_out: 2e-4,
    };

    fn params(omega0: f64) -> RotorParams {
        RotorParams {
            j: 2e-5,
            j_load: 0.0,
            b: 0.0,
            pole_pairs: 7.0,
            theta0: 0.0,
            omega0,
        }
    }

    fn set(e: &mut Engine, path: &str, v: f64) {
        e.queue(crate::engine::commands::EngineCommand::SetParam {
            path: path.into(),
            value: v,
            source: crate::engine::commands::ChangeSource::Internal,
        });
    }

    /// V-MECH-003: a torque step through a rigid gearbox accelerates at T/J_eq.
    #[test]
    fn v_mech_003_torque_step_through_a_gearbox() {
        let load = Load::Flywheel { j: 0.01 };
        let j_eq = 2e-5 + 3e-6 + (2e-4 + 0.01) / 81.0;
        let mut e = engine_with(0.02, params(0.0), GB, load, false);
        e.step_until(SimTime::from_secs_f64(0.3)).unwrap();
        // RK4 is exact for a constant acceleration: rtol 1e-9 covers rounding.
        let w = sig(&e, "motor.omega");
        assert!((w / (0.02 * 0.3 / j_eq) - 1.0).abs() < 1e-9, "{w}");
        assert!((sig(&e, "load.omega") * 9.0 / w - 1.0).abs() < 1e-12);
        assert!((sig(&e, "load.theta") * 9.0 / sig(&e, "motor.theta") - 1.0).abs() < 1e-12);
        assert!(sig(&e, "energy.residual").abs() < 1e-9);
    }

    /// EQ-MECH-03: the meshing loss is (1−η)|τ_in·ω_m| whether the motor drives the
    /// flywheel or brakes it (power flowing back through the gearbox).
    #[test]
    fn v_mech_004_gear_loss_in_both_power_directions() {
        let gb = Gearbox { eta: 0.8, ..GB };
        let j_l = 0.01;
        for (t, w0) in [(0.02, 5.0), (-0.02, 50.0)] {
            let mut e = engine_with(t, params(w0), gb, Load::Flywheel { j: j_l }, false);
            e.step_until(SimTime::from_secs_f64(0.05)).unwrap();
            let w = sig(&e, "motor.omega");
            assert!(w > 1.0, "still turning forward");
            // Lossless acceleration and the input-side torque it implies (T_L = 0).
            let j_eq = gb.j_eq(2e-5, j_l);
            let a0 = t / j_eq;
            let tau_in = (gb.j_out + j_l) / 9.0 * a0 / 9.0;
            let expect = 0.2 * (tau_in * w).abs() * (w / super::super::gearbox::OMEGA_EPS).tanh();
            let p = sig(&e, "gearbox.p_loss");
            assert!((p / expect - 1.0).abs() < 1e-6, "T = {t}: {p} vs {expect}");
            assert!(sig(&e, "energy.residual").abs() < 1e-6);
        }
    }

    /// EQ-MECH-09 / D-007: a live load-inertia change keeps ω by default and J·ω with
    /// `conserve_momentum`; both book the kinetic-energy change.
    #[test]
    fn live_load_inertia_change_rules() {
        let load = Load::Flywheel { j: 0.01 };
        for conserve in [false, true] {
            let mut e = engine_with(0.0, params(30.0), GB, load.clone(), conserve);
            e.step_until(SimTime::from_secs_f64(0.01)).unwrap();
            let j0 = GB.j_eq(2e-5, 0.01);
            set(&mut e, "load.inertia", 0.02);
            e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
            let j1 = GB.j_eq(2e-5, 0.02);
            let expect = if conserve { 30.0 * j0 / j1 } else { 30.0 };
            let w = sig(&e, "motor.omega");
            assert!(
                (w / expect - 1.0).abs() < 1e-9,
                "conserve = {conserve}: {w}"
            );
            assert_eq!(e.get_param("load.inertia"), Some(0.02));
            assert!(sig(&e, "energy.residual").abs() < 1e-9);
        }
        // The ratio is fixed at build.
        let mut e = engine_with(0.0, params(0.0), GB, load, false);
        set(&mut e, "gearbox.ratio", 5.0);
        e.apply_pending();
        assert!(e.drain_events().iter().any(|ev| matches!(
            ev,
            crate::engine::commands::EngineEvent::CommandRejected { .. }
        )));
    }
}
