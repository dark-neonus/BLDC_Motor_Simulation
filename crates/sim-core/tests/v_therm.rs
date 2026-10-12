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
    ElectricalInputs, ElectricalParams, LambdaTemperature, MotorElectrical, RTemperature,
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

/// Locked motor (rotor at θ_e = π/2, so a phase-a current is pure i_q) at a DC
/// voltage on phase a, magnet held at `t_m` with α_Br = −0.0012/K: steady torque.
fn locked_torque(t_m: f64) -> f64 {
    let mut bus = SignalBus::new();
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let tm = bus
        .register("thermal.t_magnet", "K", "", SignalKind::Output)
        .unwrap();
    let rp = RotorParams {
        j: 1e-4,
        j_load: 0.0,
        b: 0.0,
        pole_pairs: 7.0,
        theta0: std::f64::consts::FRAC_PI_2 / 7.0,
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
        theta_e0: std::f64::consts::FRAC_PI_2,
        saturation: None,
    };
    let motor = MotorElectrical::new(ep, inp, &mut bus)
        .unwrap()
        .with_lambda_temperature(LambdaTemperature {
            t_magnet: tm,
            alpha_br: -0.0012,
            t_ref: 293.15,
            t_start: t_m,
        });
    let mut plant = Plant::new();
    plant.add(Box::new(Fixed(vec![
        (v[0], 1.0),
        (v[1], 0.0),
        (v[2], 0.0),
        (tm, t_m),
    ])));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    let mut e = Engine::new(plant, bus, vec![], 1e-5);
    e.step_until(SimTime::from_secs_f64(0.1)).unwrap();
    sig(&e, "motor.torque_em")
}

/// EQ-THERM-03: magnets at 80 °C vs 20 °C → K_t (torque at the same current) lower by
/// α_Br·60 K = 7.2 %, rtol 1e-9.
#[test]
fn lambda_temperature_lowers_torque_per_amp() {
    let (cold, hot) = (locked_torque(293.15), locked_torque(353.15));
    // i_a = 2/3 A is −i_q here: T = 1.5·p·λ·i_q = −0.21 N·m at 20 °C.
    assert!((cold + 0.21).abs() < 1e-9, "{cold}");
    assert!(
        (hot / cold / (1.0 - 0.0012 * 60.0) - 1.0).abs() < 1e-9,
        "{hot} / {cold}"
    );
}

/// V-THERM-002 through the motor's own R(T): a locked motor fed V = 1.5·R(T_max)·Î_cont
/// on phase a (i_a = 2V/(3R) with a floating neutral) has its one equilibrium at
/// T_w = T_max, i_a = Î_cont. The capacitances are scaled by 1e-4 so the network
/// settles in seconds; Î_cont (EQ-THERM-05) does not depend on them. atol 1 K.
#[test]
fn v_therm_002_continuous_current_through_the_motor() {
    use sim_model::thermal_limits::{ThermalModel, continuous_current};
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
    let v = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let p = ThermalParams {
        c_w: m.c_w * 1e-4,
        c_s: m.c_s * 1e-4,
        c_h: m.c_h * 1e-4,
        r_ws: m.r_ws,
        r_sh: m.r_sh,
        r_ha: m.r_ha,
        t_amb,
        t_init: t_amb,
        t_max: m.t_max,
        kappa: 0.0,
        outrunner: true,
    };
    let th = Thermal::new(p, &mut bus).unwrap();
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
        r: m.r_ref,
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
            t_winding: bus.id("thermal.t_winding").unwrap(),
            t_ref: m.t_ref,
        });
    let th = th.with_inputs(ThermalInputs {
        p_cu: Some(bus.id("motor.p_cu").unwrap()),
        ..Default::default()
    });
    let v_a = 1.5 * m.r(m.t_max) * ic;
    let mut plant = Plant::new();
    plant.add(Box::new(Fixed(vec![(v[0], v_a), (v[1], 0.0), (v[2], 0.0)])));
    plant.add(Box::new(th));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    let mut e = Engine::new(plant, bus, vec![], 2e-5);
    // Slowest time constant ≈ 0.13 s here: 3 s is steady.
    e.step_until(SimTime::from_secs_f64(3.0)).unwrap();
    let t_w = sig(&e, "thermal.t_winding");
    assert!(
        (t_w - m.t_max).abs() < 1.0,
        "T_w = {t_w} K vs {} K",
        m.t_max
    );
    let i_a = sig(&e, "motor.i_a");
    assert!((i_a / ic - 1.0).abs() < 1e-3, "i_a = {i_a} vs Î_cont {ic}");
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

/// Review M2: live thermal edits survive a snapshot restored into a fresh engine.
#[test]
fn thermal_edits_survive_snapshot_restore() {
    use sim_core::engine::commands::{ChangeSource, EngineCommand};
    use sim_core::engine::snapshot::Snapshot;
    let mut a = network([10.0, 1.0, 0.5], TP);
    a.step_until(SimTime::from_secs_f64(50.0)).unwrap();
    a.queue(EngineCommand::SetParam {
        path: "motor.thermal.r_ha".into(),
        value: 2.0,
        source: ChangeSource::Internal,
    });
    a.step_until(SimTime::from_secs_f64(60.0)).unwrap();
    let snap = Snapshot::from_bytes(&a.snapshot().to_bytes().unwrap()).unwrap();
    let mut b = network([10.0, 1.0, 0.5], TP);
    b.restore(&snap).unwrap();
    assert_eq!(b.get_param("motor.thermal.r_ha"), Some(2.0));
    a.step_until(SimTime::from_secs_f64(200.0)).unwrap();
    b.step_until(SimTime::from_secs_f64(200.0)).unwrap();
    assert_eq!(a.snapshot().x, b.snapshot().x);
}
