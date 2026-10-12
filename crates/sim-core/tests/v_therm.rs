//! V-THERM catalog cases (validation-catalog.md) for the thermal network and its
//! couplings.

use sim_core::engine::commands::EngineEvent;
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, MotorElectrical, RTemperature,
};
use sim_core::physics::thermal::{Thermal, ThermalInputs, ThermalParams};

/// Writes fixed values on the given signals.
struct Fixed(Vec<(SignalId, f64)>);

impl PlantModule for Fixed {
    fn name(&self) -> &str {
        "fixed"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        for &(id, v) in &self.0 {
            bus.set(id, v);
        }
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

const TP: ThermalParams = ThermalParams {
    c_w: 27.0,
    c_s: 75.0,
    c_h: 168.0,
    r_ws: 0.5,
    r_sh: 0.3,
    r_ha: 4.9,
    t_amb: 298.15,
    t_init: 298.15,
    t_max: 398.15,
    kappa: 0.0,
    outrunner: true,
};

fn network(heat: [f64; 3], p: ThermalParams) -> Engine {
    let mut bus = SignalBus::new();
    let ids: Vec<SignalId> = ["motor.p_cu", "motor.p_fe", "motor.p_fric"]
        .iter()
        .map(|n| bus.register(n, "W", "", SignalKind::Output).unwrap())
        .collect();
    let th = Thermal::new(p, &mut bus)
        .unwrap()
        .with_inputs(ThermalInputs {
            p_cu: Some(ids[0]),
            p_fe: Some(ids[1]),
            p_fric: Some(ids[2]),
            omega: None,
        });
    let mut plant = Plant::new();
    plant.add(Box::new(Fixed(ids.into_iter().zip(heat).collect())));
    plant.add(Box::new(th));
    Engine::new(plant, bus, vec![], 1e-2)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

type M3 = [[f64; 3]; 3];

fn mul(a: &M3, b: &M3) -> M3 {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    c
}

/// e^{A·t} by scaling and squaring with a 20-term Taylor series (exact to round-off).
fn expm(a: &M3, t: f64) -> M3 {
    let norm: f64 = a.iter().flatten().map(|x| x.abs()).sum::<f64>() * t;
    let s = norm.log2().ceil().max(0.0) as i32 + 4;
    let h = t / 2f64.powi(s);
    let mut term = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut sum = term;
    for k in 1..=20 {
        term = mul(&term, a);
        for row in term.iter_mut() {
            for x in row.iter_mut() {
                *x *= h / k as f64;
            }
        }
        for i in 0..3 {
            for j in 0..3 {
                sum[i][j] += term[i][j];
            }
        }
    }
    for _ in 0..s {
        sum = mul(&sum, &sum);
    }
    sum
}

/// V-THERM-001: constant heat (α_Cu = 0: no motor) → node temperatures equal the exact
/// solution of the linear network, T(t) = T_ss + e^{At}(T₀ − T_ss), rtol 1e-6.
#[test]
fn v_therm_001_linear_network_matches_exact_solution() {
    let heat = [10.0, 2.0, 1.0];
    let p = TP;
    let mut e = network(heat, p);
    let a: M3 = [
        [-1.0 / (p.r_ws * p.c_w), 1.0 / (p.r_ws * p.c_w), 0.0],
        [
            1.0 / (p.r_ws * p.c_s),
            -(1.0 / p.r_ws + 1.0 / p.r_sh) / p.c_s,
            1.0 / (p.r_sh * p.c_s),
        ],
        [
            0.0,
            1.0 / (p.r_sh * p.c_h),
            -(1.0 / p.r_sh + 1.0 / p.r_ha) / p.c_h,
        ],
    ];
    // Steady state (temperature rise over ambient) through the series chain.
    let total: f64 = heat.iter().sum();
    let th_ss = total * p.r_ha;
    let ts_ss = th_ss + (heat[0] + heat[1]) * p.r_sh;
    let tw_ss = ts_ss + heat[0] * p.r_ws;
    let ss = [tw_ss, ts_ss, th_ss];
    for t in [5.0, 60.0, 600.0, 3000.0] {
        e.step_until(SimTime::from_secs_f64(t)).unwrap();
        let m = expm(&a, t);
        for (i, name) in ["thermal.t_winding", "thermal.t_stator", "thermal.t_housing"]
            .iter()
            .enumerate()
        {
            let exact = ss[i] - (0..3).map(|k| m[i][k] * ss[k]).sum::<f64>();
            let rise = sig(&e, name) - p.t_amb;
            assert!(
                (rise - exact).abs() <= 1e-6 * exact.abs(),
                "{name} at {t} s: {rise} vs {exact}"
            );
        }
    }
    assert_eq!(sig(&e, "thermal.t_magnet"), sig(&e, "thermal.t_housing"));
}

/// Locked motor at a DC voltage with the winding held at `t_w`: steady current V/R(T).
fn steady_current(t_w: f64) -> f64 {
    let mut bus = SignalBus::new();
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let tw = bus
        .register("thermal.t_winding", "K", "", SignalKind::Output)
        .unwrap();
    let rp = RotorParams {
        j: 1e-4,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: 7.0,
        theta0: 0.0,
        omega0: 0.0,
    };
    let rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .unwrap()
        .with_locked(true);
    let inp = ElectricalInputs {
        v,
        theta_e: bus.id("motor.theta_e").unwrap(),
        omega_e: bus.id("motor.omega_e").unwrap(),
    };
    let ep = ElectricalParams {
        r: 1.0,
        ld: 2.5e-3,
        lq: 2.5e-3,
        lambda: 0.03,
        pole_pairs: 7.0,
        shape: Shape::Sinusoidal,
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus)
        .unwrap()
        .with_r_temperature(RTemperature {
            t_winding: tw,
            t_ref: 293.15,
        });
    let mut plant = Plant::new();
    plant.add(Box::new(Fixed(vec![
        (v[0], 1.0),
        (v[1], 0.0),
        (v[2], 0.0),
        (tw, t_w),
    ])));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    let mut e = Engine::new(plant, bus, vec![], 1e-5);
    // τ = L/R = 2.5 ms: 0.1 s is 40 τ.
    e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    sig(&e, "motor.i_a")
}

/// V-THERM-003: winding at 100 °C vs 20 °C → R ratio 1 + 0.00393·80 = 1.3144, rtol 1e-9
/// (seen as the inverse ratio of the steady locked-rotor current).
#[test]
fn v_therm_003_resistance_follows_winding_temperature() {
    let ratio = steady_current(293.15) / steady_current(373.15);
    assert!((ratio / 1.3144 - 1.0).abs() < 1e-9, "{ratio}");
}

/// Overheat warnings: approaching (T_max − 10 K) and over T_max, each raised once.
#[test]
fn overheat_warnings_fire_once_each() {
    let p = ThermalParams { t_max: 320.0, ..TP };
    let mut e = network([30.0, 0.0, 0.0], p);
    e.step_until(SimTime::from_secs_f64(2000.0)).unwrap();
    assert!(sig(&e, "thermal.t_winding") > 320.0);
    let w: Vec<String> = e
        .drain_events()
        .into_iter()
        .filter_map(|ev| match ev {
            EngineEvent::Warning { source, msg, .. } if source == "motor.thermal" => Some(msg),
            _ => None,
        })
        .collect();
    assert_eq!(w.len(), 2, "{w:?}");
    assert!(w[0].contains("within") && w[1].contains("over"));
}
