//! V-THERM-002 (P06.T07), network-only form: holding the continuous current from
//! `sim_model::thermal_limits` on the engine's thermal network settles at T_max (atol
//! 1 K). The copper loss is restated here; `v_therm.rs` runs the same case through the
//! motor's own R(T) path.

use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::thermal::{Thermal, ThermalInputs, ThermalParams};
use sim_model::thermal_limits::{ThermalModel, continuous_current};

/// Copper loss of a peak phase current `i` at the winding temperature (EQ-THERM-02).
struct Copper {
    m: ThermalModel,
    i: f64,
    t_w: SignalId,
    p_cu: SignalId,
}

impl PlantModule for Copper {
    fn name(&self) -> &str {
        "copper"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        let p = 1.5 * self.m.r(bus.get(self.t_w)) * self.i * self.i;
        bus.set(self.p_cu, p);
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

#[test]
fn v_therm_002_continuous_current_settles_at_t_max() {
    // The 6020-class preset's estimated network (presets/motors/6020-outrunner.yaml).
    let m = ThermalModel {
        r_ref: 1.8,
        t_ref: 293.15,
        r_ws: 0.5,
        r_sh: 0.3,
        r_ha: 4.91583,
        c_w: 27.027,
        c_s: 75.348,
        c_h: 168.48,
        t_max: 398.15,
    };
    let t_amb = 298.15;
    let ic = continuous_current(&m, t_amb).unwrap();
    let mut bus = SignalBus::new();
    let p_cu = bus
        .register("motor.p_cu", "W", "", SignalKind::Output)
        .unwrap();
    let p = ThermalParams {
        c_w: m.c_w,
        c_s: m.c_s,
        c_h: m.c_h,
        r_ws: m.r_ws,
        r_sh: m.r_sh,
        r_ha: m.r_ha,
        t_amb,
        t_init: t_amb,
        t_max: m.t_max,
        kappa: 0.0,
        outrunner: true,
    };
    let th = Thermal::new(p, &mut bus)
        .unwrap()
        .with_inputs(ThermalInputs {
            p_cu: Some(p_cu),
            ..Default::default()
        });
    let copper = Copper {
        m,
        i: ic,
        t_w: bus.id("thermal.t_winding").unwrap(),
        p_cu,
    };
    let mut plant = Plant::new();
    plant.add(Box::new(th));
    plant.add(Box::new(copper));
    let mut e = Engine::new(plant, bus, vec![], 0.5);
    // Slowest time constant ≈ (C_w + C_s + C_h)·R_ha ≈ 1.3 ks: 20 ks is steady.
    e.step_until(SimTime::from_secs_f64(20_000.0)).unwrap();
    let t_w = e.bus.get(e.bus.id("thermal.t_winding").unwrap());
    assert!(
        (t_w - m.t_max).abs() < 1.0,
        "T_w = {t_w} K vs {} K",
        m.t_max
    );
}
