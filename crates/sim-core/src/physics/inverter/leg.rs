//! Switching inverter legs (EQ-INV-01): each leg is High, Low or Off; the terminal
//! voltage w.r.t. DC− follows from the state, the phase current and the device drops.
//! Leg states arrive on the bus as `inverter.leg_x` (+1 High, −1 Low, 0 Off), written
//! by the PWM/commutation logic.

use crate::engine::plant::PlantModule;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LegState {
    High,
    Low,
    Off,
}

impl LegState {
    /// Bus encoding: +1 High, −1 Low, 0 Off.
    pub fn from_signal(v: f64) -> Self {
        if v > 0.5 {
            LegState::High
        } else if v < -0.5 {
            LegState::Low
        } else {
            LegState::Off
        }
    }

    pub fn to_signal(self) -> f64 {
        match self {
            LegState::High => 1.0,
            LegState::Low => -1.0,
            LegState::Off => 0.0,
        }
    }
}

/// Device parameters of the bridge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Devices {
    /// MOSFET on-resistance R_on [Ω] (both directions, synchronous rectification).
    pub r_on: f64,
    /// Diode forward drop V_f [V].
    pub v_f: f64,
}

/// Terminal voltage of one leg (EQ-INV-01), or `None` when the terminal floats (Off with
/// zero current: the motor sets it, EQ-MOT-12).
pub fn terminal_voltage(s: LegState, i: f64, v_bus: f64, d: &Devices) -> Option<f64> {
    match s {
        LegState::High => Some(v_bus - d.r_on * i),
        LegState::Low => Some(-d.r_on * i),
        LegState::Off if i > 0.0 => Some(-d.v_f),
        LegState::Off if i < 0.0 => Some(v_bus + d.v_f),
        LegState::Off => None,
    }
}

/// Whether a leg's current flows through the high side (EQ-INV-08): the High switch, or
/// the high-side diode when Off with i < 0.
pub fn high_side_conducts(s: LegState, i: f64) -> bool {
    s == LegState::High || (s == LegState::Off && i < 0.0)
}

/// Switching-mode inverter (module `inverter`, no states): writes `inverter.v_a..c` from
/// the leg states and the motor currents, and `inverter.i_dc` (EQ-INV-08).
pub struct SwitchingInverter {
    d: Devices,
    legs: [SignalId; 3],
    i: [SignalId; 3],
    v_bus: SignalId,
    /// Floating terminal: v_n + e_x (EQ-MOT-12), from the motor's signals.
    v_n: SignalId,
    e: [SignalId; 3],
    v: [SignalId; 3],
    i_dc: SignalId,
    /// `motor.open_phase` (−1 none, k: phase k floats).
    open_phase: Option<SignalId>,
    /// `motor.open_phase2`: the second floating phase (zero-current mode, EQ-MOT-12).
    open_phase2: Option<SignalId>,
    /// `motor.diode_x`: the diode a reconnect went through (−1 high, +1 low, 0 none).
    diode: Option<[SignalId; 3]>,
    /// Conducting diode of an Off leg: +1 low diode (i > 0), −1 high diode (i < 0), 0 none.
    /// Latched when the leg turns Off (or a floating terminal reaches a bound) and held
    /// until the motor reports the phase floating: the clamp must not flip with the sign
    /// of i inside a step, or the current chatters at zero instead of crossing it.
    latch: [f64; 3],
    prev: [LegState; 3],
    /// Averaged mode (EQ-INV-03): driven legs give their period-average voltage.
    averaged: Option<Averaged>,
    /// Switching loss (EQ-INV-09): (t_r + t_f, f_pwm).
    sw: Option<(f64, f64)>,
    i_sw: SignalId,
    p_loss: SignalId,
    /// Book the DC-link input V_bus·(i_dc + i_sw) here (no bus module yet).
    dc_input: bool,
    /// Six-step drive (EQ-INV-07): a High leg is H/O-PWM'd with its duty, a Low leg is L
    /// for the whole period; only the High leg switches.
    six_step: bool,
}

/// Averaged-mode data (EQ-INV-03, `fidelity.inverter_mode = averaged`).
#[derive(Debug, Clone, Copy)]
struct Averaged {
    duty: [SignalId; 3],
    /// t_d / T_pwm.
    dead_frac: f64,
    /// Current scale of the dead-time sign smoothing σ(i) = tanh(i/i_ε) [A].
    i_eps: f64,
}

/// `inverter.duty_a..c`: the existing inputs, or registered here (PWM block or averaged
/// inverter, whichever comes first).
pub fn duty_ids(bus: &mut SignalBus) -> Result<[SignalId; 3], SignalError> {
    let mut get = |x: &str| {
        let n = format!("inverter.duty_{x}");
        match bus.id(&n) {
            Ok(id) => Ok(id),
            Err(_) => bus.register(
                &n,
                "-",
                "duty (fraction of the period High)",
                SignalKind::Input,
            ),
        }
    };
    Ok([get("a")?, get("b")?, get("c")?])
}

/// Period-average terminal voltage of a complementary PWM leg (EQ-INV-03), including the
/// dead-time error with σ(i) = tanh(i/i_ε).
/// Effective dead fraction with pulse dropping (EQ-INV-03): min(t_d/T, d, 1 − d).
pub fn dead_eff(d: f64, dead_frac: f64) -> f64 {
    dead_frac.min(d).min(1.0 - d).max(0.0)
}

pub fn averaged_voltage(
    d: f64,
    i: f64,
    v_bus: f64,
    dev: &Devices,
    dead_frac: f64,
    i_eps: f64,
) -> f64 {
    let sigma = (i / i_eps).tanh();
    let dead_frac = dead_eff(d, dead_frac);
    d * v_bus
        - dev.r_on * i
        - sigma * dead_frac * (v_bus + 2.0 * dev.v_f - 2.0 * dev.r_on * i.abs())
}

/// Averaged six-step High leg (EQ-INV-07): H for d, O otherwise. `branch` +1: the
/// current flows into the motor (i > 0), the low diode clamps the O part to −V_f;
/// −1: out of the motor (i < 0) through the high diode, V_bus + V_f. Returns (v̄_xT, its
/// DC-link current). When the current reaches zero the phase floats instead (the leg is
/// floatable, EQ-MOT-12): no branch conducts.
pub fn six_step_high_voltage(d: f64, i: f64, v_bus: f64, dev: &Devices, branch: f64) -> (f64, f64) {
    let on = d * (v_bus - dev.r_on * i);
    if branch < 0.0 {
        (on + (1.0 - d) * (v_bus + dev.v_f), i)
    } else {
        (on - (1.0 - d) * dev.v_f, d * i)
    }
}

/// Lower clamp of a floatable leg at zero current (EQ-MOT-12): −V_f for an Off leg (d = 0),
/// the H/O on-average d·V_bus − (1 − d)·V_f for an averaged six-step High leg.
pub fn lower_clamp(d: f64, v_bus: f64, v_f: f64) -> f64 {
    d * v_bus - (1.0 - d) * v_f
}

impl SwitchingInverter {
    /// Six-step drive (EQ-INV-07): High legs are H/O-PWM'd (no complementary low side),
    /// Low legs stay L. In switching mode the PWM block produces the H/O edges
    /// (`Pwm::with_six_step`).
    pub fn with_six_step(mut self) -> Self {
        self.six_step = true;
        self
    }

    /// Switching loss (EQ-INV-09) as an extra bus current i_sw = f_pwm·Σ ½|i_x|(t_r + t_f)
    /// over the legs that switch.
    pub fn with_switching_loss(mut self, t_rise_fall: f64, f_pwm: f64) -> Self {
        self.sw = Some((t_rise_fall, f_pwm));
        self
    }

    /// The DC bus is a module of its own that books the source input: keep only the loss.
    pub fn without_dc_input(mut self) -> Self {
        self.dc_input = false;
        self
    }

    /// Switch to the averaged model (EQ-INV-03): legs that are not Off follow their duty
    /// `inverter.duty_x` (complementary PWM); Off legs keep the diode/floating rule.
    pub fn averaged(
        mut self,
        bus: &mut SignalBus,
        dead_frac: f64,
        i_eps: f64,
    ) -> Result<Self, SignalError> {
        self.averaged = Some(Averaged {
            duty: duty_ids(bus)?,
            dead_frac,
            i_eps,
        });
        Ok(self)
    }

    /// Registers `inverter.leg_a..c` (inputs), `inverter.v_a..c` and `inverter.i_dc`. The
    /// motor's current/EMF/neutral signals are attached with `with_motor` (the motor needs
    /// the terminal voltages first).
    pub fn new(d: Devices, v_bus: SignalId, bus: &mut SignalBus) -> Result<Self, SignalError> {
        let mut leg = |x: &str| {
            bus.register(
                &format!("inverter.leg_{x}"),
                "-",
                "leg state: +1 High, −1 Low, 0 Off",
                SignalKind::Input,
            )
        };
        let legs = [leg("a")?, leg("b")?, leg("c")?];
        // Start Low (the PWM's zero vector); 0.0 would read as Off and float a phase.
        for id in legs {
            bus.set(id, LegState::Low.to_signal());
        }
        let mut volt = |x: &str| {
            bus.register(
                &format!("inverter.v_{x}"),
                "V",
                "terminal voltage w.r.t. DC−",
                SignalKind::Output,
            )
        };
        let v = [volt("a")?, volt("b")?, volt("c")?];
        let i_dc = bus.register("inverter.i_dc", "A", "DC-link current", SignalKind::Output)?;
        let i_sw = bus.register(
            "inverter.i_sw",
            "A",
            "switching-loss bus current",
            SignalKind::Output,
        )?;
        let p_loss = bus.register("inverter.p_loss", "W", "inverter loss", SignalKind::Output)?;
        Ok(Self {
            d,
            legs,
            i: [v_bus; 3],
            v_bus,
            v_n: v_bus,
            e: [v_bus; 3],
            v,
            i_dc,
            open_phase: None,
            open_phase2: None,
            diode: None,
            latch: [0.0; 3],
            prev: [LegState::Low; 3],
            averaged: None,
            sw: None,
            i_sw,
            p_loss,
            dc_input: true,
            six_step: false,
        })
    }

    pub fn outputs_ids(&self) -> [SignalId; 3] {
        self.v
    }

    /// Attach the motor's `motor.i_x`, `motor.e_x` and `motor.v_n` signals.
    pub fn with_motor(mut self, bus: &SignalBus) -> Result<Self, SignalError> {
        let id = |n: &str| bus.id(n);
        self.i = [id("motor.i_a")?, id("motor.i_b")?, id("motor.i_c")?];
        self.e = [id("motor.e_a")?, id("motor.e_b")?, id("motor.e_c")?];
        self.v_n = id("motor.v_n")?;
        self.open_phase = Some(id("motor.open_phase")?);
        self.open_phase2 = bus.id("motor.open_phase2").ok();
        self.diode = Some([
            id("motor.diode_a")?,
            id("motor.diode_b")?,
            id("motor.diode_c")?,
        ]);
        Ok(self)
    }

    /// Initial leg states (written once; later the PWM/commutation block owns them).
    pub fn set_legs(&self, bus: &mut SignalBus, s: [LegState; 3]) {
        for (id, s) in self.legs.iter().zip(s) {
            bus.set(*id, s.to_signal());
        }
    }
}

/// Below this phase current [A] the fallback diode choice uses the terminal voltage,
/// not the sign of i (round-off).
const LATCH_I_MIN: f64 = 1e-9;
impl SwitchingInverter {
    /// Whether a leg in state `s` lets its phase float at zero current (EQ-MOT-12): Off,
    /// or an averaged six-step High leg (H/O: no current path while the switch is off).
    fn floatable(&self, s: LegState) -> bool {
        s == LegState::Off || (self.six_step && self.averaged.is_some() && s == LegState::High)
    }

    /// The lower clamp of leg k at zero current (`lower_clamp`).
    fn lower_clamp_of(&self, k: usize, v_bus: f64, bus: &SignalBus) -> f64 {
        let s = LegState::from_signal(bus.get(self.legs[k]));
        let d = match (self.averaged, self.six_step && s == LegState::High) {
            (Some(a), true) => bus.get(a.duty[k]).clamp(0.0, 1.0),
            _ => 0.0,
        };
        lower_clamp(d, v_bus, self.d.v_f)
    }

    /// The conducting branch of an averaged six-step High leg: the motor's reconnect
    /// diode, else the current's sign (+1 at zero: the drive pushes current in).
    fn high_branch(&self, k: usize, i: f64, bus: &SignalBus) -> f64 {
        let from_motor = self.diode.map_or(0.0, |d| bus.get(d[k]));
        if from_motor != 0.0 {
            from_motor
        } else if i < -LATCH_I_MIN {
            -1.0
        } else {
            1.0
        }
    }

    /// The conducting diode of Off leg k that is not floating (−1 high, +1 low), without
    /// mutating state (D-017): the motor's reconnect diode, else the latch set at the
    /// turn-off (`after_discrete`), else the current's sign, or at i ≈ 0 the bound the
    /// terminal is nearer.
    fn diode_of(&self, k: usize, i: f64, v_bus: f64, bus: &SignalBus) -> f64 {
        let from_motor = self.diode.map_or(0.0, |d| bus.get(d[k]));
        if from_motor != 0.0 {
            from_motor
        } else if self.latch[k] != 0.0 {
            self.latch[k]
        } else if i.abs() > LATCH_I_MIN {
            i.signum()
        } else if bus.get(self.v_n) + bus.get(self.e[k]) >= 0.5 * v_bus {
            -1.0
        } else {
            1.0
        }
    }
}

impl PlantModule for SwitchingInverter {
    fn name(&self) -> &str {
        "inverter"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _x: &mut [f64]) {}
    fn outputs(&mut self, _t: f64, _x: &[f64], _off: usize, bus: &mut SignalBus) {
        let v_bus = bus.get(self.v_bus);
        let open = self.open_phase.map_or(-1.0, |id| bus.get(id));
        let open2 = self.open_phase2.map_or(-1.0, |id| bus.get(id));
        let floating = |k: usize| open == k as f64 || open2 == k as f64;
        let lo: [f64; 3] = std::array::from_fn(|k| self.lower_clamp_of(k, v_bus, bus));
        let mut i_dc = 0.0;
        for (k, &lo_k) in lo.iter().enumerate() {
            let s = LegState::from_signal(bus.get(self.legs[k]));
            let i = bus.get(self.i[k]);
            let mut dc = None;
            let v = if floating(k) && self.floatable(s) {
                // Floating: v_n + e_x (EQ-MOT-12); i = 0, no diode conducts.
                // max/min, not clamp: a bus below −2V_f (no reverse clamp yet, P10.T09)
                // would invert the bounds, and clamp panics on that.
                dc = Some(0.0);
                (bus.get(self.v_n) + bus.get(self.e[k]))
                    .min(v_bus + self.d.v_f)
                    .max(lo_k)
            } else if let (Some(a), true) = (self.averaged, s != LegState::Off) {
                let d = bus.get(a.duty[k]).clamp(0.0, 1.0);
                match (self.six_step, s) {
                    (true, LegState::High) => {
                        let branch = self.high_branch(k, i, bus);
                        let (v, idc) = six_step_high_voltage(d, i, v_bus, &self.d, branch);
                        dc = Some(idc);
                        v
                    }
                    (true, _) => {
                        dc = Some(0.0);
                        terminal_voltage(LegState::Low, i, v_bus, &self.d).unwrap_or(0.0)
                    }
                    (false, _) => averaged_voltage(d, i, v_bus, &self.d, a.dead_frac, a.i_eps),
                }
            } else if s != LegState::Off {
                terminal_voltage(s, i, v_bus, &self.d).unwrap_or(0.0)
            } else {
                terminal_voltage(s, self.diode_of(k, i, v_bus, bus), v_bus, &self.d).unwrap_or(0.0)
            };
            bus.set(self.v[k], v);
            if let Some(idc) = dc {
                i_dc += idc;
            } else if let (Some(a), true) = (self.averaged, s != LegState::Off) {
                // EQ-INV-08 averaged: the dead time changes the effective high-side duty.
                let d = bus.get(a.duty[k]).clamp(0.0, 1.0);
                i_dc += (d - (i / a.i_eps).tanh() * dead_eff(d, a.dead_frac)) * i;
            } else if s == LegState::High
                || (s == LegState::Off && !floating(k) && self.diode_of(k, i, v_bus, bus) < 0.0)
            {
                i_dc += i;
            }
        }
        if open2 >= 0.0 {
            // Zero-current mode: both floating terminals at v_n + e_x with v_n set by the
            // driven leg j alone (i = 0 everywhere), so v_m − v_j = e_m − e_j holds exactly.
            let (k, m) = (open as usize, open2 as usize);
            let j = 3 - k - m;
            let v_n = bus.get(self.v[j]) - bus.get(self.e[j]);
            for x in [k, m] {
                let v = (v_n + bus.get(self.e[x]))
                    .min(v_bus + self.d.v_f)
                    .max(lo[x]);
                bus.set(self.v[x], v);
            }
        }
        bus.set(self.i_dc, i_dc);
        // Switching loss: every leg that is not Off switches once per period (EQ-INV-09);
        // in six-step only the High (PWM'd) leg switches.
        let switches = |s: LegState| match self.six_step {
            true => s == LegState::High,
            false => s != LegState::Off,
        };
        let i_sw = self.sw.map_or(0.0, |(t_sw, f)| {
            (0..3)
                .filter(|&k| switches(LegState::from_signal(bus.get(self.legs[k]))))
                .map(|k| 0.5 * bus.get(self.i[k]).abs() * t_sw * f)
                .sum()
        });
        bus.set(self.i_sw, i_sw);
        let p_out: f64 = (0..3)
            .map(|k| bus.get(self.v[k]) * bus.get(self.i[k]))
            .sum();
        bus.set(self.p_loss, v_bus * (i_dc + i_sw) - p_out);
    }
    fn derivatives(&self, _t: f64, _x: &[f64], _off: usize, _bus: &SignalBus, _dx: &mut [f64]) {}
    fn power_terms(&self) -> Vec<(String, crate::energy::PowerKind)> {
        use crate::energy::PowerKind;
        let loss = ("inverter".into(), PowerKind::Loss);
        if self.dc_input {
            vec![("dc_link".into(), PowerKind::Input), loss]
        } else {
            vec![loss]
        }
    }
    fn powers(&self, _t: f64, _x: &[f64], _off: usize, bus: &SignalBus, p: &mut [f64]) {
        // From this pass's outputs: P_dc = V_bus·(i_dc + i_sw), P_loss = P_dc − Σ v_x·i_x
        // (conduction, diodes and switching, EQ-INV-09).
        let loss = bus.get(self.p_loss);
        if self.dc_input {
            p[0] = bus.get(self.v_bus) * (bus.get(self.i_dc) + bus.get(self.i_sw));
            p[1] = loss;
        } else {
            p[0] = loss;
        }
    }
    fn after_discrete(&mut self, _t: f64, _x: &mut [f64], bus: &SignalBus) {
        for k in 0..3 {
            let s = LegState::from_signal(bus.get(self.legs[k]));
            if s != LegState::Off {
                self.latch[k] = 0.0;
            } else if self.prev[k] != LegState::Off {
                // Turned Off: the current keeps its direction through the matching diode.
                let i = bus.get(self.i[k]);
                self.latch[k] = if i == 0.0 { 0.0 } else { i.signum() };
            }
            self.prev[k] = s;
        }
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "latch": self.latch,
            "prev": self.prev.map(LegState::to_signal),
            "r_on": self.d.r_on, "v_f": self.d.v_f,
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let arr = |k: &str| -> Result<[f64; 3], String> {
            serde_json::from_value(v.get(k).cloned().unwrap_or_default())
                .map_err(|e| format!("`{k}`: {e}"))
        };
        self.latch = arr("latch")?;
        self.prev = arr("prev")?.map(LegState::from_signal);
        let f = |k: &str| {
            v.get(k)
                .and_then(serde_json::Value::as_f64)
                .ok_or(format!("missing `{k}`"))
        };
        (self.d.r_on, self.d.v_f) = (f("r_on")?, f("v_f")?);
        Ok(())
    }
    fn params(&self) -> Vec<(String, String)> {
        vec![("r_on".into(), "ohm".into()), ("v_f".into(), "V".into())]
    }
    fn get_param(&self, name: &str) -> Option<f64> {
        match name {
            "r_on" => Some(self.d.r_on),
            "v_f" => Some(self.d.v_f),
            _ => None,
        }
    }
    fn set_param(&mut self, name: &str, v: f64, _x: &mut [f64]) -> Result<f64, String> {
        let slot = match name {
            "r_on" => &mut self.d.r_on,
            "v_f" => &mut self.d.v_f,
            _ => return Err(format!("unknown parameter `{name}`")),
        };
        if !(v.is_finite() && v >= 0.0) {
            return Err(format!("`{name}` must be ≥ 0, got {v}"));
        }
        Ok(std::mem::replace(slot, v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const D: Devices = Devices {
        r_on: 0.01,
        v_f: 0.8,
    };

    #[test]
    fn v_inv_005_terminal_voltage_per_state() {
        // On-resistance drop in both directions.
        assert_eq!(
            terminal_voltage(LegState::High, 5.0, 24.0, &D),
            Some(24.0 - 0.05)
        );
        assert_eq!(terminal_voltage(LegState::Low, -5.0, 24.0, &D), Some(0.05));
        // Off: the diode set by the current direction conducts.
        assert_eq!(terminal_voltage(LegState::Off, 2.0, 24.0, &D), Some(-0.8));
        assert_eq!(terminal_voltage(LegState::Off, -2.0, 24.0, &D), Some(24.8));
        assert_eq!(terminal_voltage(LegState::Off, 0.0, 24.0, &D), None);
        assert!(high_side_conducts(LegState::Off, -1.0) && !high_side_conducts(LegState::Off, 1.0));
        for s in [LegState::High, LegState::Low, LegState::Off] {
            assert_eq!(LegState::from_signal(s.to_signal()), s);
        }
    }

    /// Pulse dropping (EQ-INV-03): the averaged voltage is exact at d = 0 and d = 1 and
    /// continuous near them.
    #[test]
    fn averaged_pulse_dropping_matches_the_full_low_and_high_legs() {
        let f = 0.01;
        for i in [3.0, -3.0] {
            assert_eq!(averaged_voltage(0.0, i, 24.0, &D, f, 0.01), -D.r_on * i);
            assert_eq!(
                averaged_voltage(1.0, i, 24.0, &D, f, 0.01),
                24.0 - D.r_on * i
            );
            // Continuous: a tiny duty is close to d = 0.
            let v = averaged_voltage(1e-6, i, 24.0, &D, f, 0.01);
            assert!((v + D.r_on * i).abs() < 1e-4, "{v}");
        }
        // In the linear range the full dead fraction applies.
        assert_eq!(dead_eff(0.5, f), f);
        assert_eq!(
            (dead_eff(0.004, f), dead_eff(0.997, f)),
            (0.004, 1.0 - 0.997)
        );
    }

    /// EQ-INV-07 averaged six-step High leg: i > 0 → d(V − R_on i) − (1 − d)V_f with
    /// i_dc = d·i; i < 0 → d(V − R_on i) + (1 − d)(V + V_f) with i_dc = i.
    #[test]
    fn six_step_high_leg_average() {
        let (d, v) = (0.3, 24.0);
        let (vp, ip) = six_step_high_voltage(d, 2.0, v, &D, 1.0);
        assert!((vp - (d * (v - D.r_on * 2.0) - (1.0 - d) * D.v_f)).abs() < 1e-12);
        assert!((ip - d * 2.0).abs() < 1e-12);
        let (vn, i_n) = six_step_high_voltage(d, -2.0, v, &D, -1.0);
        assert!((vn - (d * (v + D.r_on * 2.0) + (1.0 - d) * (v + D.v_f))).abs() < 1e-12);
        assert!((i_n + 2.0).abs() < 1e-12);
    }

    /// Review P07 #4 / P08.T10: a terminal reconnected at the high bound uses the high
    /// diode even when the round-off current is a hair positive; the choice comes from
    /// the motor (`motor.diode_x`) and `outputs` mutates nothing (D-017).
    #[test]
    fn reconnect_latch_follows_the_bound_not_round_off() {
        let mut bus = SignalBus::new();
        let v_bus = bus.register("bus.v", "V", "", SignalKind::Output).unwrap();
        let mut reg = |n: &str| bus.register(n, "-", "", SignalKind::Output).unwrap();
        let (ia, ib, ic) = (reg("motor.i_a"), reg("motor.i_b"), reg("motor.i_c"));
        let (ea, _, _) = (reg("motor.e_a"), reg("motor.e_b"), reg("motor.e_c"));
        let (vn, open) = (reg("motor.v_n"), reg("motor.open_phase"));
        let (da, _, _) = (
            reg("motor.diode_a"),
            reg("motor.diode_b"),
            reg("motor.diode_c"),
        );
        let mut inv = SwitchingInverter::new(D, v_bus, &mut bus)
            .unwrap()
            .with_motor(&bus)
            .unwrap();
        bus.set(v_bus, 24.0);
        bus.set(open, -1.0);
        bus.set(inv.legs[0], LegState::Off.to_signal());
        // Phase a just reached V_bus + V_f (v_n + e_a = 24.8), i_a = +1e-15 from round-off.
        bus.set(vn, 12.0);
        bus.set(ea, 12.8);
        bus.set(ia, 1e-15);
        bus.set(ib, -1.0);
        bus.set(ic, 1.0);
        // Without the motor's word: the fallback picks the bound the terminal is at.
        inv.outputs(0.0, &[], 0, &mut bus);
        assert_eq!(bus.get(inv.v[0]), 24.8, "high diode (fallback)");
        // The motor's reconnect diode decides, even against the terminal estimate.
        bus.set(vn, 0.0);
        bus.set(da, -1.0);
        inv.outputs(0.0, &[], 0, &mut bus);
        assert_eq!(bus.get(inv.v[0]), 24.8, "high diode (motor)");
        // outputs() keeps no state: the latch is untouched.
        assert_eq!(inv.latch, [0.0; 3]);
    }
}
