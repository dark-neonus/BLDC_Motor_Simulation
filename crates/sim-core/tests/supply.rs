//! P07.T06–T09: DC bus, PSU, battery and brake chopper (EQ-SUP-01…05) under a scripted
//! DC-link load current.

use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::supply::battery::Battery;
use sim_core::physics::supply::bus::{Chopper, DcBus, Source};

/// Writes `inverter.i_dc` = f(t).
struct Load(SignalId, fn(f64) -> f64);
impl PlantModule for Load {
    fn name(&self) -> &str {
        "load"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, t: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        bus.set(self.0, (self.1)(t));
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
    // The load's energy leaves (or enters) the system here.
    fn power_terms(&self) -> Vec<(String, sim_core::energy::PowerKind)> {
        vec![("load".into(), sim_core::energy::PowerKind::External)]
    }
    fn powers(&self, _: f64, _: &[f64], _: usize, bus: &SignalBus, p: &mut [f64]) {
        p[0] = -bus.get(bus.id("bus.v").unwrap()) * bus.get(self.0);
    }
}

const C: f64 = 470e-6;

fn rig(src: Source, chopper: Option<Chopper>, f: fn(f64) -> f64, dt: f64) -> Engine {
    let mut bus = SignalBus::new();
    let i_dc = bus
        .register("inverter.i_dc", "A", "", SignalKind::Output)
        .unwrap();
    let dc = DcBus::new(src, C, chopper, &mut bus)
        .unwrap()
        .with_load(vec![i_dc]);
    let mut plant = Plant::new();
    plant.add(Box::new(Load(i_dc, f)));
    plant.add(Box::new(dc));
    Engine::new(plant, bus, vec![], dt)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn run(e: &mut Engine, t: f64) {
    e.step_until(SimTime::from_secs_f64(t)).unwrap();
}

/// T06: with no source current (PSU limit 0) a constant 2 A discharge drops V linearly.
#[test]
fn constant_discharge_is_linear() {
    let src = Source::Psu {
        v_set: 24.0,
        i_lim: 0.0,
        r_out: 0.02,
    };
    let mut e = rig(src, None, |_| 2.0, 5e-6);
    for t in [1e-3, 2e-3, 3e-3] {
        run(&mut e, t);
        assert!(
            (sig(&e, "bus.v") - (24.0 - 2.0 * t / C)).abs() < 1e-9,
            "{t}"
        );
    }
    assert!(sig(&e, "energy.residual").abs() < 1e-9);
}

/// T07: 8 A from a 5 A PSU → CC at I_lim and the bus sags at 3 A/C; a regen current
/// cannot be sunk → blocking, the bus rises at 2 A/C. Energy closes throughout.
#[test]
fn v_sup_001_psu_current_limit_and_blocking() {
    let src = Source::Psu {
        v_set: 24.0,
        i_lim: 5.0,
        r_out: 0.02,
    };
    let mut e = rig(src, None, |t| if t < 0.002 { 8.0 } else { -2.0 }, 5e-6);
    run(&mut e, 1e-3);
    assert_eq!(sig(&e, "supply.mode"), 1.0, "CC");
    assert!((sig(&e, "supply.i") - 5.0).abs() < 1e-12);
    let v1 = sig(&e, "bus.v");
    run(&mut e, 1.5e-3);
    assert!(((v1 - sig(&e, "bus.v")) / (3.0 * 0.5e-3 / C) - 1.0).abs() < 1e-6);
    run(&mut e, 0.01);
    assert_eq!(sig(&e, "supply.mode"), 2.0, "blocking");
    assert_eq!(sig(&e, "supply.i"), 0.0);
    let v2 = sig(&e, "bus.v");
    assert!(v2 > 24.0);
    run(&mut e, 0.011);
    assert!(((sig(&e, "bus.v") - v2) / (2.0 * 1e-3 / C) - 1.0).abs() < 1e-6);
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}

fn b_ocv(e: &Engine) -> f64 {
    pack().ocv(sig(e, "supply.soc"))
}

fn pack() -> Battery {
    let cell = [(0.0, 3.0), (0.5, 3.7), (1.0, 4.2)];
    Battery::from_cells(&cell, 6, 1, 4e-3, 2e-3, 4000.0, 5.0)
}

/// T08 / V-SUP-004: a 10 A step sags by I·R0 at once (after the fast bus RC), the RC
/// branch charges and then relaxes with τ = R1·C1, and SoC falls by ∫I dt / (3600·Q).
/// Exact references (rtol 1e-6) include the bus: its capacitor carries C·dV/dt, and the
/// source current follows a load step R0·C later.
#[test]
fn v_sup_004_battery_sag_relaxation_and_soc() {
    let b = pack();
    let (r0, r1, c1, q) = (b.r0, b.r1, b.c1, b.q_ah);
    let tau = r1 * c1;
    // The load steps back to 0 at τ/10 (the closure cannot capture τ).
    assert!((tau - 8.0).abs() < 1e-12);
    let src = Source::Battery { pack: b, soc0: 0.8 };
    // dt = 5 µs ≈ τ_bus/2 (D-018: RK4 is stable to ~2.8 τ; accuracy checked here).
    let mut e = rig(src, None, |t| if t < 0.8 { 10.0 } else { 0.0 }, 5e-6);
    // Bus RC = R0·C ≈ 11 µs: after 0.2 ms the bus sits on OCV − i·R0 − v1. With the
    // fast bus mode eliminated (error O(τ_bus/τ) on a 1e-6 correction), the source
    // current is i = (I + C·v1/τ)/(1 + κ), κ = C·(1/C1 + OCV'/(3600·Q)) (its capacitor
    // carries C·dV/dt), and v1 obeys dv1/dt = a·I − b·v1, a = 1/(C1(1+κ)),
    // b = (1 − C/(C1(1+κ)))/τ, with I applied δ = R0·C late (the bus RC's lag).
    let slope = 6.0 * (4.2 - 3.7) / 0.5; // OCV' above SoC 0.5
    let kappa = C * (1.0 / c1 + slope / (3600.0 * q));
    let (a, bb) = (
        1.0 / (c1 * (1.0 + kappa)),
        (1.0 - C / (c1 * (1.0 + kappa))) / tau,
    );
    run(&mut e, 2e-4);
    let v1 = sig(&e, "supply.battery.v_rc");
    let sag = b_ocv(&e) - v1 - sig(&e, "bus.v");
    let i = (10.0 + C * v1 / tau) / (1.0 + kappa);
    assert!((sag / (i * r0) - 1.0).abs() < 1e-6, "{sag} vs {}", i * r0);
    let d = r0 * C;
    let t1 = tau / 10.0;
    run(&mut e, t1);
    let charged = |t: f64| 10.0 * a / bb * (1.0 - (-bb * (t - d)).exp());
    let v = sig(&e, "supply.battery.v_rc");
    assert!(
        (v / charged(t1) - 1.0).abs() < 1e-6,
        "charging: {v} vs {}",
        charged(t1)
    );
    let soc = sig(&e, "supply.soc");
    let want_soc = 0.8 - 10.0 * t1 / (3600.0 * q);
    assert!((soc - want_soc).abs() < 1e-6, "{soc} vs {want_soc}");
    // Relaxation (I = 0 from t_off + δ): v1 = v1(t_off + δ)·e^{−b(t − t_off − δ)}. The
    // scripted load is 0 at t1, which RK4's last stage of the step ending there samples
    // with weight 1/6: the load effectively stops at t_off = t1 − h/6 (h = 5 µs).
    let t_off = t1 - 5e-6 / 6.0;
    let t2 = t1 + tau / 10.0;
    run(&mut e, t2);
    let want = charged(t_off + d) * (-bb * (t2 - t_off - d)).exp();
    let v = sig(&e, "supply.battery.v_rc");
    assert!((v / want - 1.0).abs() < 1e-6, "relaxation: {v} vs {want}");
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}

/// T09: regen into a blocking PSU with a chopper: the bus stays within the hysteresis band
/// (≤ V_on plus a step's worth), the chopper cycles, and the dumped energy is booked.
#[test]
fn chopper_clamps_regen() {
    let src = Source::Psu {
        v_set: 24.0,
        i_lim: 5.0,
        r_out: 0.02,
    };
    let ch = Chopper {
        r_brake: 5.0,
        v_on: 28.0,
        v_off: 27.0,
    };
    let mut e = rig(src, Some(ch), |_| -3.0, 2e-6);
    let mut cycles = 0;
    let mut prev = 0.0;
    for k in 1..=2000 {
        run(&mut e, k as f64 * 1e-5);
        let v = sig(&e, "bus.v");
        assert!(v <= 28.0 + 1e-6, "{v}");
        let on = sig(&e, "bus.chopper_on");
        if on == 1.0 && prev == 0.0 {
            cycles += 1;
        }
        prev = on;
        if k > 500 {
            assert!(v >= 27.0 - 1e-6, "{v}");
        }
    }
    assert!(cycles >= 3, "{cycles}");
    assert!(sig(&e, "energy.loss.brake") > 0.0);
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}

/// V-SUP-005: constant discharge: 3600·Q·ΔSoC = ∫i_src dt = I·t + C·ΔV_bus exactly (RK4
/// keeps this linear invariant), rtol 1e-9.
#[test]
fn v_sup_005_battery_soc_integration() {
    let b = pack();
    let q = b.q_ah;
    let src = Source::Battery { pack: b, soc0: 0.8 };
    let mut e = rig(src, None, |_| 10.0, 5e-6);
    let v0 = sig(&e, "bus.v");
    run(&mut e, 0.5);
    let charge = 3600.0 * q * (0.8 - sig(&e, "supply.soc"));
    let want = 10.0 * 0.5 + C * (sig(&e, "bus.v") - v0);
    assert!((charge / want - 1.0).abs() < 1e-9, "{charge} vs {want}");
}

/// Review P07 #13: a bus that starts above V_on starts with the chopper on.
#[test]
fn chopper_starts_on_above_v_on() {
    let ch = Chopper {
        r_brake: 5.0,
        v_on: 25.0,
        v_off: 24.5,
    };
    let src = Source::Psu {
        v_set: 30.0,
        i_lim: 1.0,
        r_out: 0.02,
    };
    let mut e = rig(src, Some(ch), |_| 0.0, 5e-6);
    run(&mut e, 1e-5);
    assert_eq!(sig(&e, "bus.chopper_on"), 1.0);
}

/// D-019 / P07 review #11: bus and PSU parameters are readable and live-settable under
/// their flat paths; a C change books ½ΔC·V² (energy closes); R·C below the build-time
/// value (the D-018 step cap) is refused.
#[test]
fn bus_and_supply_parameters_live() {
    use sim_core::engine::commands::{ChangeSource, EngineCommand};
    let src = Source::Psu {
        v_set: 24.0,
        i_lim: 5.0,
        r_out: 0.02,
    };
    let mut e = rig(src, None, |_| 2.0, 5e-6);
    assert_eq!(e.get_param("bus.capacitance"), Some(C));
    assert_eq!(e.get_param("supply.r_out"), Some(0.02));
    assert_eq!(e.get_param("supply.voltage"), Some(24.0));
    run(&mut e, 1e-3);
    let set = |e: &mut Engine, path: &str, value: f64| {
        e.queue(EngineCommand::SetParam {
            path: path.into(),
            value,
            source: ChangeSource::Internal,
        });
    };
    set(&mut e, "bus.capacitance", 2.0 * C);
    set(&mut e, "supply.current_limit", 1.0);
    // With C doubled, R = 0.005 gives half the build-time R·C.
    set(&mut e, "supply.r_out", 0.005);
    run(&mut e, 3e-3);
    assert_eq!(e.get_param("bus.capacitance"), Some(2.0 * C));
    assert_eq!(e.get_param("supply.current_limit"), Some(1.0));
    assert_eq!(
        e.get_param("supply.r_out"),
        Some(0.02),
        "R·C below the build value: refused"
    );
    // The 2 A load now exceeds the 1 A limit: CC mode, the bus sags.
    assert_eq!(sig(&e, "supply.mode"), 1.0);
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
}

/// A bus snapshot from before the live parameters (P08.T10) restores with the built
/// values (P08 review m8).
#[test]
fn pre_p08_bus_snapshot_restores() {
    let src = Source::Psu {
        v_set: 24.0,
        i_lim: 5.0,
        r_out: 0.02,
    };
    let mut e = rig(src.clone(), None, |_| 2.0, 5e-6);
    run(&mut e, 1e-3);
    let mut snap = e.snapshot();
    let (_, st) = snap
        .modules
        .iter_mut()
        .find(|(n, _)| n == "bus")
        .expect("bus module");
    for k in ["c", "jump", "v", "i_lim", "r_out"] {
        st.as_object_mut().unwrap().remove(k);
    }
    let mut f = rig(src, None, |_| 2.0, 5e-6);
    f.restore(&snap).unwrap();
    assert_eq!(f.get_param("bus.capacitance"), Some(C));
    assert_eq!(f.get_param("supply.current_limit"), Some(5.0));
    assert_eq!(f.get_param("supply.r_out"), Some(0.02));
}
