//! Three-node thermal network (EQ-THERM-01): winding → stator → housing → ambient.
//! Heat: copper loss into the winding, iron loss into the stator, bearing/load friction
//! into the housing. Temperatures are states in kelvin.
//!
//! The losses already leave the electromechanical energy balance as loss terms, so the
//! network's heat is tracked separately (energy.md EQ-ENER-01, "heat to ambient") and
//! adds no power terms here.

use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

/// Copper temperature coefficient α_Cu [1/K] (EQ-THERM-02).
pub const ALPHA_CU: f64 = 0.00393;
/// Margin below T_max for the "approaching" warning [K].
pub const WARN_MARGIN: f64 = 10.0;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalParams {
    /// Heat capacities C_w, C_s, C_h [J/K].
    pub c_w: f64,
    pub c_s: f64,
    pub c_h: f64,
    /// Thermal resistances R_ws, R_sh, R_ha [K/W].
    pub r_ws: f64,
    pub r_sh: f64,
    pub r_ha: f64,
    /// Ambient temperature T_a [K].
    pub t_amb: f64,
    /// Start temperature of all nodes [K].
    pub t_init: f64,
    /// Maximum winding temperature [K].
    pub t_max: f64,
    /// Fan-cooling factor κ [s/rad] (EQ-THERM-04).
    pub kappa: f64,
    /// Magnets sit on the housing (outrunner) or next to the stator (inrunner).
    pub outrunner: bool,
}

/// Heat-source and speed signals the network reads (absent sources count as 0).
#[derive(Debug, Clone, Copy, Default)]
pub struct ThermalInputs {
    pub p_cu: Option<SignalId>,
    pub p_fe: Option<SignalId>,
    pub p_fric: Option<SignalId>,
    pub omega: Option<SignalId>,
}

pub struct Thermal {
    p: ThermalParams,
    inp: ThermalInputs,
    t_w: SignalId,
    t_s: SignalId,
    t_h: SignalId,
    t_mag: SignalId,
    warnings: Vec<String>,
}

impl Thermal {
    /// Registers `thermal.t_winding`, `thermal.t_stator`, `thermal.t_housing`,
    /// `thermal.t_magnet` [K]. Heat inputs come with `with_inputs` (the motor that
    /// produces them needs the temperature signals first).
    pub fn new(p: ThermalParams, bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut r = |path: &str, d: &str| bus.register(path, "K", d, SignalKind::State);
        Ok(Self {
            t_w: r("thermal.t_winding", "winding temperature")?,
            t_s: r("thermal.t_stator", "stator iron temperature")?,
            t_h: r("thermal.t_housing", "housing / can temperature")?,
            t_mag: r(
                "thermal.t_magnet",
                "magnet temperature (housing or stator node)",
            )?,
            p,
            inp: ThermalInputs::default(),
            warnings: vec![],
        })
    }

    pub fn with_inputs(mut self, inp: ThermalInputs) -> Self {
        self.inp = inp;
        self
    }

    fn get(bus: &SignalBus, id: Option<SignalId>) -> f64 {
        id.map_or(0.0, |id| bus.get(id))
    }

    /// R_ha with optional fan cooling (EQ-THERM-04).
    fn r_ha(&self, bus: &SignalBus) -> f64 {
        self.p.r_ha / (1.0 + self.p.kappa * Self::get(bus, self.inp.omega).abs())
    }
}

impl PlantModule for Thermal {
    fn name(&self) -> &str {
        "motor.thermal"
    }
    fn n_states(&self) -> usize {
        3
    }
    fn state_names(&self) -> Vec<(String, String)> {
        ["t_winding", "t_stator", "t_housing"]
            .iter()
            .map(|n| ((*n).into(), "K".into()))
            .collect()
    }
    fn init(&self, x: &mut [f64]) {
        x.fill(self.p.t_init);
    }
    fn outputs(&mut self, _t: f64, x: &[f64], off: usize, bus: &mut SignalBus) {
        let (tw, ts, th) = (x[off], x[off + 1], x[off + 2]);
        bus.set(self.t_w, tw);
        bus.set(self.t_s, ts);
        bus.set(self.t_h, th);
        bus.set(self.t_mag, if self.p.outrunner { th } else { ts });
    }
    fn derivatives(&self, _t: f64, x: &[f64], off: usize, bus: &SignalBus, dx: &mut [f64]) {
        let p = &self.p;
        let (tw, ts, th) = (x[off], x[off + 1], x[off + 2]);
        let q_ws = (tw - ts) / p.r_ws;
        let q_sh = (ts - th) / p.r_sh;
        let q_ha = (th - p.t_amb) / self.r_ha(bus);
        dx[0] = (Self::get(bus, self.inp.p_cu) - q_ws) / p.c_w;
        dx[1] = (Self::get(bus, self.inp.p_fe) + q_ws - q_sh) / p.c_s;
        dx[2] = (Self::get(bus, self.inp.p_fric) + q_sh - q_ha) / p.c_h;
    }
    fn params(&self) -> Vec<(String, String)> {
        let mut v: Vec<(String, String)> = ["r_ws", "r_sh", "r_ha"]
            .iter()
            .map(|n| ((*n).into(), "K/W".into()))
            .collect();
        v.extend(
            ["c_w", "c_s", "c_h"]
                .iter()
                .map(|n| ((*n).into(), "J/K".into())),
        );
        v.push(("kappa".into(), "s/rad".into()));
        v
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        let p = &self.p;
        Some(match name {
            "r_ws" => p.r_ws,
            "r_sh" => p.r_sh,
            "r_ha" => p.r_ha,
            "c_w" => p.c_w,
            "c_s" => p.c_s,
            "c_h" => p.c_h,
            "kappa" => p.kappa,
            "t_max" => p.t_max,
            _ => return None,
        })
    }
    fn set_param(&mut self, name: &str, v: f64, _x_own: &mut [f64]) -> Result<f64, String> {
        let p = &mut self.p;
        let (slot, positive) = match name {
            "r_ws" => (&mut p.r_ws, true),
            "r_sh" => (&mut p.r_sh, true),
            "r_ha" => (&mut p.r_ha, true),
            "c_w" => (&mut p.c_w, true),
            "c_s" => (&mut p.c_s, true),
            "c_h" => (&mut p.c_h, true),
            "kappa" => (&mut p.kappa, false),
            _ => return Err(format!("unknown parameter `{name}`")),
        };
        if !(v.is_finite() && (v > 0.0 || (!positive && v == 0.0))) {
            return Err(format!(
                "`{name}` must be {}, got {v}",
                if positive { "> 0" } else { "≥ 0" }
            ));
        }
        Ok(std::mem::replace(slot, v))
    }
    fn n_events(&self) -> usize {
        2
    }
    /// 0: winding within `WARN_MARGIN` of T_max; 1: winding at T_max (rising → warning).
    fn event_functions(&self, _t: f64, x: &[f64], off: usize, _bus: &SignalBus, g: &mut [f64]) {
        g[0] = x[off] - (self.p.t_max - WARN_MARGIN);
        g[1] = x[off] - self.p.t_max;
    }
    fn on_event(
        &mut self,
        idx: usize,
        rising: bool,
        _t: f64,
        _x_own: &mut [f64],
        _bus: &mut SignalBus,
    ) {
        if !rising {
            return;
        }
        let c = self.p.t_max - 273.15;
        self.warnings.push(match idx {
            0 => format!("winding within {WARN_MARGIN} K of its limit ({c:.0} °C)"),
            _ => format!("winding over its limit ({c:.0} °C): overheating"),
        });
    }
    fn take_warnings(&mut self) -> Vec<String> {
        std::mem::take(&mut self.warnings)
    }
}
