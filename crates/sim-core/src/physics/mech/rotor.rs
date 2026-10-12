//! Rigid drivetrain (EQ-MECH-02/03, D-015): rotor, gearbox and load share one degree of
//! freedom, θm/ωm on the motor side:
//!
//! J_eq·dω/dt = Σ T_internal + Σ T_external − B·ω + T_gl + T_L/N,
//! J_eq = J_m + J_gi + (J_go + J_load)/N²,  θ_L = θm/N.
//!
//! Torques arrive as bus signals. *Internal* torques (electromagnetic, cogging, iron
//! drag) move energy between modules and need no power term here: the module that
//! produces them accounts for its side. *External* torques do work on the system and
//! are reported as `External` power. Load torques are computed here and booked by kind
//! (`load::LoadTorques`).

use crate::energy::PowerKind;
use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

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
}

/// One evaluation of the drivetrain dynamics.
struct Dyn {
    /// dω_m/dt.
    a: f64,
    /// Gear loss torque on the motor shaft.
    t_gl: f64,
    load: LoadTorques,
    /// Σ external motor-side torques.
    t_ext_m: f64,
    /// Applied torque on the motor shaft except friction and gear loss.
    t_a: f64,
    /// Lumped friction torque (EQ-MECH-05).
    t_f: f64,
    /// Stuck: ω ≡ 0.
    held: bool,
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
    /// Motor-bearing static/Coulomb friction (its viscous part is `p.b`).
    fr_m: Friction,
    /// Load-side friction, lumped onto the motor shaft (rigid drivetrain).
    fr_l: Friction,
    mode: Mode,
}

impl RotorRigid {
    /// Registers `motor.theta`, `motor.omega`, `motor.theta_e`, `motor.omega_e`, the
    /// load-side `load.theta`, `load.omega`, `load.torque_gravity`, `load.torque_ext` and
    /// `gearbox.p_loss`. `internal` / `external` are motor-side torque signals [N·m]
    /// that must already be registered.
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
            fr_m: Friction::default(),
            fr_l: Friction::default(),
            mode: Mode::Stick,
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

    /// Stick–slip friction (EQ-MECH-05): motor static/Coulomb (`motor.b` comes from
    /// `RotorParams::b`) and load-side friction. Starts slipping if ω0 ≠ 0.
    pub fn with_friction(mut self, motor: Friction, load: Friction) -> Self {
        self.fr_m = motor;
        self.fr_l = load;
        let w0 = if self.locked { 0.0 } else { self.p.omega0 };
        self.mode = if w0 != 0.0 {
            Mode::Slip { dir: w0.signum() }
        } else {
            Mode::Stick
        };
        self
    }

    /// The friction lumped on ω_m.
    fn friction(&self) -> Friction {
        let motor = Friction {
            b: self.p.b,
            ..self.fr_m
        };
        Friction::lumped(&motor, &self.fr_l, self.gb.n)
    }

    fn hybrid(&self) -> bool {
        !self.locked && self.friction().is_hybrid()
    }

    fn j_eq(&self) -> f64 {
        self.gb.j_eq(self.p.j + self.p.j_load, self.load.inertia())
    }

    fn dynamics(&self, th: f64, w: f64, bus: &SignalBus) -> Dyn {
        let n = self.gb.n;
        let mut load = self.load.torques(th / n, w / n);
        load.ext += self.dist;
        let t_l = load.total();
        let t_ext_m: f64 = self.external.iter().map(|&id| bus.get(id)).sum();
        let t_int: f64 = self.internal.iter().map(|&id| bus.get(id)).sum();
        let j_eq = self.j_eq();
        let t_a = t_int + t_ext_m + t_l / n;
        let fr = self.friction();
        let (t_f, held) = if fr.is_hybrid() {
            let e = friction::eval(&fr, self.mode, t_a, w);
            (e.torque, e.held)
        } else {
            (-fr.b * w, false)
        };
        if held {
            // While sticking T_gl = 0 (EQ-MECH-05 clarifications).
            return Dyn {
                a: 0.0,
                t_gl: 0.0,
                load,
                t_ext_m,
                t_a,
                t_f,
                held,
            };
        }
        // Lossless acceleration first; the gear loss uses it (EQ-MECH-03, one pass).
        let a0 = (t_a + t_f) / j_eq;
        let t_gl = self.gb.loss_torque(a0, t_l, self.load.inertia(), w);
        Dyn {
            a: a0 + t_gl / j_eq,
            t_gl,
            load,
            t_ext_m,
            t_a,
            t_f,
            held,
        }
    }

    /// Apply a change to inertia-carrying parameters with the EQ-MECH-09 rule: keep ω
    /// (D-007) or, with `conserve`, keep J_eq·ω. The stored-energy jump (kinetic and
    /// potential) is booked as external energy.
    fn change_inertia(
        &mut self,
        x_own: &mut [f64],
        conserve: bool,
        f: impl FnOnce(&mut Self) -> Result<f64, String>,
    ) -> Result<f64, String> {
        let (th, w) = (x_own[0], x_own[1]);
        let (j0, u0) = (self.j_eq(), self.load.potential(th / self.gb.n));
        let old = f(self)?;
        let (j1, u1) = (self.j_eq(), self.load.potential(th / self.gb.n));
        let w1 = if conserve { w * j0 / j1 } else { w };
        x_own[1] = w1;
        self.jump += 0.5 * j1 * w1 * w1 - 0.5 * j0 * w * w + (u1 - u0);
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
            "gearbox.ratio" => Err("the gear ratio changes only on a rebuild (reset)".into()),
            _ => Err(format!("unknown parameter `{name}`")),
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
        3
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![
            ("theta".into(), "rad".into()),
            ("omega".into(), "rad/s".into()),
            ("disturbance_left".into(), "s".into()),
        ]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = self.p.theta0;
        x[1] = if self.locked { 0.0 } else { self.p.omega0 };
        x[2] = 0.0;
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let (th, w) = (x[off], x[off + 1]);
        let (n, p) = (self.gb.n, self.p.pole_pairs);
        let ids = &self.ids;
        bus.set(ids.theta, th);
        bus.set(ids.omega, w);
        bus.set(ids.theta_e, p * th);
        bus.set(ids.omega_e, p * w);
        bus.set(ids.theta_l, th / n);
        bus.set(ids.omega_l, w / n);
        // The motor torques on the bus are from the previous stage here (the motor's
        // outputs run after the rotor's): p_loss is for display; the energy book uses
        // `powers`, which runs after all outputs.
        let d = self.dynamics(th, w, bus);
        bus.set(ids.torque_gravity, d.load.gravity);
        bus.set(ids.torque_ext, d.load.loss + d.load.ext);
        bus.set(ids.p_gear, if self.locked { 0.0 } else { -d.t_gl * w });
        bus.set(ids.disturbance, self.dist);
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        dx[2] = if self.dist != 0.0 { -1.0 } else { 0.0 };
        if self.locked {
            dx[0] = 0.0;
            dx[1] = 0.0;
            return;
        }
        let (th, w) = (x[off], x[off + 1]);
        let d = self.dynamics(th, w, bus);
        dx[0] = if d.held { 0.0 } else { w };
        dx[1] = d.a;
    }
    fn params(&self) -> Vec<(String, String)> {
        let mut v = vec![
            ("j_rotor".into(), "kg*m^2".into()),
            ("friction.viscous".into(), "N*m*s/rad".into()),
            ("locked".into(), "-".into()),
            ("gearbox.efficiency".into(), "-".into()),
            ("gearbox.j_in".into(), "kg*m^2".into()),
            ("gearbox.j_out".into(), "kg*m^2".into()),
        ];
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
                self.locked = v != 0.0;
                // Locking stops the shaft; releasing starts from rest.
                self.mode = Mode::Stick;
                if self.locked {
                    // Stopping the shaft removes its kinetic energy.
                    self.jump -= 0.5 * self.j_eq() * x_own[1] * x_own[1];
                    x_own[1] = 0.0;
                }
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
            ("external".into(), PowerKind::External),
        ]
    }
    fn powers(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, p: &mut [f64]) {
        if self.locked {
            p.fill(0.0);
            return;
        }
        let (th, w) = (x[off], x[off + 1]);
        let d = self.dynamics(th, w, bus);
        let w_l = w / self.gb.n;
        p[0] = -d.t_f * w;
        p[1] = -d.t_gl * w;
        p[2] = -d.load.loss * w_l;
        p[3] = d.t_ext_m * w + d.load.ext * w_l;
    }
    fn stored_energy(&self, x: &[f64], off: usize, _bus: &SignalBus) -> f64 {
        let w = x[off + 1];
        0.5 * self.j_eq() * w * w + self.load.potential(x[off] / self.gb.n)
    }
    fn take_external_energy(&mut self) -> f64 {
        std::mem::take(&mut self.jump)
    }
    fn n_events(&self) -> usize {
        3
    }
    fn event_functions(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, g: &mut [f64]) {
        // 0: the pulse ends when its remaining time crosses zero.
        g[0] = if self.dist != 0.0 { x[off + 2] } else { 1.0 };
        // 1: slip → ω crosses zero; stick → |T_a| crosses T_s (EQ-MECH-05).
        // 2: stick but already moving (a step started past breakaway, e.g. after an edit).
        (g[1], g[2]) = (1.0, 1.0);
        if self.hybrid() {
            let (th, w) = (x[off], x[off + 1]);
            match self.mode {
                Mode::Slip { dir } => g[1] = dir * w,
                Mode::Stick => {
                    let t_a = self.dynamics(th, w, bus).t_a;
                    g[1] = self.friction().t_s - t_a.abs();
                    g[2] = 1e-12 - w.abs();
                }
            }
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
        let (th, w) = (x_own[0], x_own[1]);
        match (idx, self.mode) {
            (0, _) => {
                self.dist = 0.0;
                x_own[2] = 0.0;
            }
            (1, Mode::Slip { .. }) => {
                let t_a = self.dynamics(th, w, bus).t_a;
                if t_a.abs() <= self.friction().t_s {
                    // Stick: the leftover ½Jω² (bisection-sized) leaves with the jump.
                    self.jump -= 0.5 * self.j_eq() * w * w;
                    x_own[1] = 0.0;
                    self.mode = Mode::Stick;
                } else {
                    self.mode = Mode::Slip { dir: t_a.signum() };
                }
            }
            (1, Mode::Stick) => {
                let t_a = self.dynamics(th, w, bus).t_a;
                self.mode = Mode::Slip { dir: t_a.signum() };
            }
            (2, Mode::Stick) => self.mode = Mode::Slip { dir: w.signum() },
            _ => {}
        }
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "j": self.p.j, "b": self.p.b, "locked": self.locked,
            "eta": self.gb.eta, "j_in": self.gb.j_in, "j_out": self.gb.j_out,
            "load": self.load, "jump": self.jump, "dist": self.dist,
            "fr_m": self.fr_m, "fr_l": self.fr_l, "mode": self.mode,
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        let (j, b) = (f("j")?, f("b")?);
        let (eta, j_in, j_out, jump) = (f("eta")?, f("j_in")?, f("j_out")?, f("jump")?);
        let dist = f("dist")?;
        let locked = v
            .get("locked")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `locked`")?;
        let load: Load = serde_json::from_value(v.get("load").cloned().unwrap_or_default())
            .map_err(|e| format!("`load`: {e}"))?;
        (self.p.j, self.p.b, self.locked) = (j, b, locked);
        (self.gb.eta, self.gb.j_in, self.gb.j_out) = (eta, j_in, j_out);
        (self.load, self.jump, self.dist) = (load, jump, dist);
        let de = |k: &str| v.get(k).cloned().ok_or(format!("missing `{k}`"));
        self.fr_m = serde_json::from_value(de("fr_m")?).map_err(|e| e.to_string())?;
        self.fr_l = serde_json::from_value(de("fr_l")?).map_err(|e| e.to_string())?;
        self.mode = serde_json::from_value(de("mode")?).map_err(|e| e.to_string())?;
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
    fn torque_step_through_a_gearbox() {
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
    fn gear_loss_in_both_power_directions() {
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
