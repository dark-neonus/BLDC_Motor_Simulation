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
fn psu_current_limit_and_blocking() {
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

fn pack() -> Battery {
    let cell = [(0.0, 3.0), (0.5, 3.7), (1.0, 4.2)];
    Battery::from_cells(&cell, 6, 1, 4e-3, 2e-3, 4000.0, 5.0)
}

/// T08: a 10 A step sags by I·R0 at once (after the fast bus RC), the RC branch relaxes
/// with R1·C1, and SoC falls by ∫I dt / (3600·Q).
#[test]
fn battery_sag_relaxation_and_soc() {
    let b = pack();
    let (r0, r1, c1) = (b.r0, b.r1, b.c1);
    let ocv0 = b.ocv(0.8);
    let src = Source::Battery { pack: b, soc0: 0.8 };
    // dt = 5 µs ≈ τ_bus/2 (D-018: RK4 is stable to ~2.8 τ; accuracy checked here).
    let mut e = rig(src, None, |_| 10.0, 5e-6);
    // Bus RC = R0·C ≈ 11 µs: after 0.2 ms the bus sits on OCV − I·R0 − v1 (v1 still tiny).
    run(&mut e, 2e-4);
    let sag = ocv0 - sig(&e, "bus.v");
    let v1 = sig(&e, "supply.battery.v_rc");
    assert!((sag - (10.0 * r0 + v1)).abs() < 1e-6 * ocv0, "{sag}");
    assert!(v1 < 0.01 * 10.0 * r1);
    // RC branch: v1(t) → I·R1·(1 − e^{−t/R1C1}); τ = 48 s · … check at t = τ/10.
    let tau = r1 * c1;
    run(&mut e, tau / 10.0);
    let want = 10.0 * r1 * (1.0 - (-0.1f64).exp());
    assert!(
        (sig(&e, "supply.battery.v_rc") / want - 1.0).abs() < 1e-3,
        "{}",
        sig(&e, "supply.battery.v_rc")
    );
    let soc = sig(&e, "supply.soc");
    let want_soc = 0.8 - 10.0 * (tau / 10.0) / (3600.0 * 5.0);
    assert!((soc - want_soc).abs() < 1e-6, "{soc} vs {want_soc}");
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
