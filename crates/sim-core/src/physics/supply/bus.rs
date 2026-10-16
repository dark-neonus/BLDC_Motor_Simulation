//! DC bus with its source and brake chopper (EQ-SUP-01…05), one module `bus` that also
//! owns `supply.*` (like the drivetrain, D-015):
//!
//! C·dV/dt = i_src − i_dc − i_sw − i_ch.
//!
//! Sources: ideal (V fixed, no state, sources and sinks), lab PSU (CV/CC, cannot sink;
//! the clamp kinks are state events), battery (Thevenin 1-RC, SoC; bidirectional).
//! Chopper: V/R_b with on/off hysteresis (state events).
//!
//! States: ideal none; PSU [V]; battery [V, v₁, SoC].

use crate::energy::PowerKind;
use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

use super::battery::Battery;

#[derive(Debug, Clone, PartialEq)]
pub enum Source {
    Ideal { v: f64 },
    Psu { v_set: f64, i_lim: f64, r_out: f64 },
    Battery { pack: Battery, soc0: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Chopper {
    pub r_brake: f64,
    pub v_on: f64,
    pub v_off: f64,
}

/// PSU current (EQ-SUP-03): clamp((V_set − V)/R_o, 0, I_lim).
pub fn psu_current(v_set: f64, i_lim: f64, r_out: f64, v: f64) -> f64 {
    ((v_set - v) / r_out).clamp(0.0, i_lim)
}

/// Source resistance R_o (PSU) or R0 (battery); ∞ for the ideal source (no bus state).
fn r_src(s: &Source) -> f64 {
    match s {
        Source::Ideal { .. } => f64::INFINITY,
        Source::Psu { r_out, .. } => *r_out,
        Source::Battery { pack, .. } => pack.r0,
    }
}

struct Ids {
    v: SignalId,
    i_src: SignalId,
    mode: SignalId,
    i_ch: SignalId,
    ch_on: SignalId,
    p_brake: SignalId,
    soc: SignalId,
    v_term: SignalId,
    v_rc: SignalId,
}

pub struct DcBus {
    src: Source,
    /// Bus capacitance C_bus [F].
    c: f64,
    chopper: Option<Chopper>,
    ch_on: bool,
    /// Initial bus voltage (PSU / battery) [V].
    v0: f64,
    /// Inverter DC-link and switching-loss currents (EQ-INV-08/09).
    load: Vec<SignalId>,
    ids: Ids,
    /// R_src·C at build time: the D-018 step cap; live edits may not go below it.
    tau_build: f64,
    /// Energy injected by parameter changes (½ΔC·V²) since the last take.
    jump: f64,
}

impl DcBus {
    /// Registers `bus.v`, `supply.i`, `supply.mode`, `bus.i_chopper`, `bus.chopper_on`,
    /// `bus.p_brake`, `supply.soc`, `supply.v_terminal`, `supply.battery.v_rc`. The load
    /// currents come with `with_load` (the inverter needs `bus.v` first).
    pub fn new(
        src: Source,
        c: f64,
        chopper: Option<Chopper>,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let mut out = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Output);
        let ids = Ids {
            v: out("bus.v", "V", "DC bus voltage")?,
            i_src: out("supply.i", "A", "source current into the bus (+ supplying)")?,
            mode: out("supply.mode", "-", "PSU mode: 0 CV, 1 CC, 2 blocking")?,
            i_ch: out("bus.i_chopper", "A", "brake chopper current")?,
            ch_on: out("bus.chopper_on", "-", "1 while the brake chopper conducts")?,
            p_brake: out("bus.p_brake", "W", "brake resistor power")?,
            soc: out("supply.soc", "-", "battery state of charge")?,
            v_term: out("supply.v_terminal", "V", "source terminal voltage")?,
            v_rc: out("supply.battery.v_rc", "V", "battery RC-branch voltage")?,
        };
        let v0 = match &src {
            Source::Ideal { v } => *v,
            Source::Psu { v_set, .. } => *v_set,
            Source::Battery { pack, soc0 } => pack.ocv(*soc0),
        };
        // A bus that starts above V_on starts with the chopper conducting (EQ-SUP-05).
        let ch_on = chopper.is_some_and(|ch| v0 > ch.v_on);
        let tau_build = r_src(&src) * c;
        Ok(Self {
            src,
            c,
            chopper,
            ch_on,
            v0,
            load: vec![],
            ids,
            tau_build,
            jump: 0.0,
        })
    }

    /// Currents drawn from the bus (`inverter.i_dc`, `inverter.i_sw`).
    pub fn with_load(mut self, ids: Vec<SignalId>) -> Self {
        self.load = ids;
        self
    }

    pub fn v_id(&self) -> SignalId {
        self.ids.v
    }

    fn v(&self, s: &[f64]) -> f64 {
        match self.src {
            Source::Ideal { v } => v,
            _ => s[0],
        }
    }

    fn i_load(&self, bus: &SignalBus) -> f64 {
        self.load.iter().map(|&id| bus.get(id)).sum()
    }

    fn i_ch(&self, v: f64) -> f64 {
        match (self.chopper, self.ch_on) {
            (Some(c), true) => v / c.r_brake,
            _ => 0.0,
        }
    }

    /// Source current into the bus.
    fn i_src(&self, s: &[f64], bus: &SignalBus) -> f64 {
        let v = self.v(s);
        match &self.src {
            Source::Ideal { .. } => self.i_load(bus) + self.i_ch(v),
            Source::Psu {
                v_set,
                i_lim,
                r_out,
            } => psu_current(*v_set, *i_lim, *r_out, v),
            Source::Battery { pack, .. } => (pack.ocv(s[2]) - s[1] - v) / pack.r0,
        }
    }
}

impl PlantModule for DcBus {
    fn name(&self) -> &str {
        "bus"
    }
    fn param_roots(&self) -> Vec<String> {
        vec!["supply".into()]
    }
    fn params(&self) -> Vec<(String, String)> {
        let mut v = vec![("capacitance".to_string(), "F".to_string())];
        match self.src {
            Source::Ideal { .. } => v.push(("supply.voltage".into(), "V".into())),
            Source::Psu { .. } => {
                v.push(("supply.voltage".into(), "V".into()));
                v.push(("supply.current_limit".into(), "A".into()));
                v.push(("supply.r_out".into(), "ohm".into()));
            }
            Source::Battery { .. } => {}
        }
        v
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match (name, &self.src) {
            ("capacitance", _) => Some(self.c),
            ("supply.voltage", Source::Ideal { v }) => Some(*v),
            ("supply.voltage", Source::Psu { v_set, .. }) => Some(*v_set),
            ("supply.current_limit", Source::Psu { i_lim, .. }) => Some(*i_lim),
            ("supply.r_out", Source::Psu { r_out, .. }) => Some(*r_out),
            _ => None,
        }
    }
    fn set_param(&mut self, name: &str, v: f64, x_own: &mut [f64]) -> Result<f64, String> {
        if !(v.is_finite() && v > 0.0) {
            return Err(format!("`{name}` must be > 0, got {v}"));
        }
        // R·C below the build-time value would break the D-018 step cap.
        let check_tau = |r: f64, c: f64, tau: f64| {
            if r * c < tau * (1.0 - 1e-12) {
                Err(format!(
                    "`{name}` = {v} makes R·C = {:.3e} s, below the {:.3e} s the step size was chosen for: rebuild (reset) to apply it",
                    r * c,
                    tau
                ))
            } else {
                Ok(())
            }
        };
        let tau = self.tau_build;
        match (name, &mut self.src) {
            ("capacitance", src) => {
                check_tau(r_src(src), v, tau)?;
                if let Some(&vb) = x_own.first() {
                    // ½ΔC·V² enters the stored energy at once (EQ-ENER-02 jump).
                    self.jump += 0.5 * (v - self.c) * vb * vb;
                }
                Ok(std::mem::replace(&mut self.c, v))
            }
            ("supply.voltage", Source::Ideal { v: old }) => Ok(std::mem::replace(old, v)),
            ("supply.voltage", Source::Psu { v_set, .. }) => Ok(std::mem::replace(v_set, v)),
            ("supply.current_limit", Source::Psu { i_lim, .. }) => Ok(std::mem::replace(i_lim, v)),
            ("supply.r_out", Source::Psu { r_out, .. }) => {
                check_tau(v, self.c, tau)?;
                Ok(std::mem::replace(r_out, v))
            }
            _ => Err(format!(
                "unknown parameter `{name}` for this source (battery parameters change on a rebuild)"
            )),
        }
    }
    fn take_external_energy(&mut self) -> f64 {
        std::mem::take(&mut self.jump)
    }
    fn n_states(&self) -> usize {
        match self.src {
            Source::Ideal { .. } => 0,
            Source::Psu { .. } => 1,
            Source::Battery { .. } => 3,
        }
    }
    fn state_names(&self) -> Vec<(String, String)> {
        let mut v = vec![];
        if self.n_states() > 0 {
            v.push(("v".into(), "V".into()));
        }
        if self.n_states() == 3 {
            v.push(("v_rc".into(), "V".into()));
            v.push(("soc".into(), "-".into()));
        }
        v
    }
    fn init(&self, x: &mut [f64]) {
        if let Some(v) = x.first_mut() {
            *v = self.v0;
        }
        if let Source::Battery { soc0, .. } = self.src {
            x[1] = 0.0;
            x[2] = soc0;
        }
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let s = &x[off..off + self.n_states()];
        let v = self.v(s);
        let i_src = self.i_src(s, bus);
        let i_ch = self.i_ch(v);
        let ids = &self.ids;
        bus.set(ids.v, v);
        bus.set(ids.i_src, i_src);
        bus.set(ids.i_ch, i_ch);
        bus.set(ids.ch_on, f64::from(u8::from(self.ch_on)));
        bus.set(ids.p_brake, v * i_ch);
        bus.set(ids.v_term, v);
        let mode = match &self.src {
            Source::Psu { v_set, i_lim, .. } if v >= *v_set => 2.0,
            Source::Psu { i_lim, .. } if i_src >= *i_lim => 1.0,
            _ => 0.0,
        };
        bus.set(ids.mode, mode);
        if let Source::Battery { .. } = self.src {
            bus.set(ids.v_rc, s[1]);
            bus.set(ids.soc, s[2]);
        }
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        let s = &x[off..off + self.n_states()];
        if s.is_empty() {
            return;
        }
        let v = s[0];
        let i_src = self.i_src(s, bus);
        // EQ-SUP-01.
        dx[0] = (i_src - self.i_load(bus) - self.i_ch(v)) / self.c;
        if let Source::Battery { pack, .. } = &self.src {
            // EQ-SUP-04.
            dx[1] = i_src / pack.c1 - s[1] / (pack.r1 * pack.c1);
            dx[2] = -i_src / (3600.0 * pack.q_ah);
        }
    }
    fn power_terms(&self) -> Vec<(String, PowerKind)> {
        let mut t = match self.src {
            // The ideal source and the PSU's internal EMF feed the system; a battery's
            // energy is stored chemical energy instead (EQ-ENER-02).
            Source::Ideal { .. } => vec![("supply".into(), PowerKind::Input)],
            Source::Psu { .. } => vec![
                ("supply".into(), PowerKind::Input),
                ("supply_r".into(), PowerKind::Loss),
            ],
            Source::Battery { .. } => vec![("battery".into(), PowerKind::Loss)],
        };
        t.push(("brake".into(), PowerKind::Loss));
        t
    }
    fn powers(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, p: &mut [f64]) {
        let s = &x[off..off + self.n_states()];
        let v = self.v(s);
        let i = self.i_src(s, bus);
        let brake = v * self.i_ch(v);
        match &self.src {
            Source::Ideal { .. } => {
                p[0] = v * i;
                p[1] = brake;
            }
            Source::Psu { r_out, .. } => {
                // EMF power V_bus·i + R_o·i² (= V_set·i inside CV), loss R_o·i².
                p[0] = v * i + r_out * i * i;
                p[1] = r_out * i * i;
                p[2] = brake;
            }
            Source::Battery { pack, .. } => {
                p[0] = pack.r0 * i * i + s[1] * s[1] / pack.r1;
                p[1] = brake;
            }
        }
    }
    fn stored_energy(&self, x: &[f64], off: usize, _bus: &SignalBus) -> f64 {
        let s = &x[off..off + self.n_states()];
        match &self.src {
            Source::Ideal { .. } => 0.0,
            Source::Psu { .. } => 0.5 * self.c * s[0] * s[0],
            Source::Battery { pack, .. } => {
                0.5 * self.c * s[0] * s[0] + 0.5 * pack.c1 * s[1] * s[1] + pack.e_chem(s[2])
            }
        }
    }
    fn n_events(&self) -> usize {
        3
    }
    fn event_functions(&self, _t: f64, x: &[f64], off: usize, _bus: &SignalBus, g: &mut [f64]) {
        let s = &x[off..off + self.n_states()];
        let v = self.v(s);
        (g[0], g[1], g[2]) = (1.0, 1.0, 1.0);
        // 0, 1: the PSU clamp's kinks (blocking at V = V_set, CC at V = V_set − R_o·I_lim).
        if let Source::Psu {
            v_set,
            i_lim,
            r_out,
        } = self.src
        {
            g[0] = v_set - v;
            g[1] = i_lim * r_out - (v_set - v);
        }
        // 2: chopper hysteresis (EQ-SUP-05).
        if let Some(c) = self.chopper {
            g[2] = if self.ch_on { v - c.v_off } else { c.v_on - v };
        }
    }
    fn on_event(
        &mut self,
        idx: usize,
        _rising: bool,
        _t: f64,
        _x_own: &mut [f64],
        _bus: &mut SignalBus,
    ) {
        if idx == 2 && self.chopper.is_some() {
            self.ch_on = !self.ch_on;
        }
    }
    fn save(&self) -> serde_json::Value {
        let (v, i_lim, r_out) = match self.src {
            Source::Ideal { v } => (v, 0.0, 0.0),
            Source::Psu {
                v_set,
                i_lim,
                r_out,
            } => (v_set, i_lim, r_out),
            Source::Battery { .. } => (0.0, 0.0, 0.0),
        };
        serde_json::json!({
            "ch_on": self.ch_on, "c": self.c, "jump": self.jump,
            "v": v, "i_lim": i_lim, "r_out": r_out,
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        self.ch_on = v
            .get("ch_on")
            .and_then(serde_json::Value::as_bool)
            .ok_or("missing `ch_on`")?;
        // Live parameters arrived in P08.T10: older snapshots keep the built values.
        let f = |k: &str, old: f64| v.get(k).and_then(serde_json::Value::as_f64).unwrap_or(old);
        (self.c, self.jump) = (f("c", self.c), f("jump", 0.0));
        match &mut self.src {
            Source::Ideal { v: sv } => *sv = f("v", *sv),
            Source::Psu {
                v_set,
                i_lim,
                r_out,
            } => {
                (*v_set, *i_lim, *r_out) = (f("v", *v_set), f("i_lim", *i_lim), f("r_out", *r_out));
            }
            Source::Battery { .. } => {}
        }
        Ok(())
    }
}
