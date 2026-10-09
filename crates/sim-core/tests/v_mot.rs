//! V-MOT validation catalog (docs/docs/physics/validation-catalog.md), one test per ID
//! (P05.T08). Tolerances are the catalog's.

use std::f64::consts::PI;

use sim_core::engine::commands::{ChangeSource, EngineCommand};
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::cogging::{Cogging, CoggingParams};
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, MotorElectrical, SatParams, clarke, inv_clarke,
};
use sim_core::physics::motor::iron_loss::{IronLoss, IronLossParams};

/// V-MOT-005 lives in its own file (shared with the P05.T03 history).
#[path = "support/abc_dq.rs"]
mod v_mot_005;

const P: f64 = 7.0;
const R: f64 = 1.0;
const L: f64 = 2.5e-3;
const LAMBDA: f64 = 0.03;

type VFn = Box<dyn Fn(f64, f64, f64) -> [f64; 3] + Send>;

/// Terminal voltages as a function of (t, θe, ωe).
struct Source {
    ids: [SignalId; 3],
    th: SignalId,
    w: SignalId,
    f: VFn,
}

impl PlantModule for Source {
    fn name(&self) -> &str {
        "inverter"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, t: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        let v = (self.f)(t, bus.get(self.th), bus.get(self.w));
        for (id, vk) in self.ids.iter().zip(v) {
            bus.set(*id, vk);
        }
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

struct Case {
    shape: Shape,
    sat: Option<SatParams>,
    j: f64,
    b: f64,
    omega0: f64,
    locked: bool,
    i_q0: f64,
    cogging: Option<CoggingParams>,
    iron: Option<IronLossParams>,
    v: VFn,
    dt: f64,
}

impl Default for Case {
    fn default() -> Self {
        Self {
            shape: Shape::Sinusoidal,
            sat: None,
            j: 1e-4,
            b: 0.0,
            omega0: 0.0,
            locked: false,
            i_q0: 0.0,
            cogging: None,
            iron: None,
            v: Box::new(|_, _, _| [0.0; 3]),
            dt: 1e-5,
        }
    }
}

fn build(c: Case) -> Engine {
    let mut bus = SignalBus::new();
    let ids = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let rp = RotorParams {
        j: c.j,
        j_load: 0.0,
        b: c.b,
        pole_pairs: P,
        theta0: 0.0,
        omega0: c.omega0,
    };
    let mut rotor = RotorRigid::new(rp, &mut bus, vec![], vec![])
        .unwrap()
        .with_locked(c.locked);
    let (th, w) = (
        bus.id("motor.theta_e").unwrap(),
        bus.id("motor.omega_e").unwrap(),
    );
    let ep = ElectricalParams {
        r: R,
        ld: L,
        lq: L,
        lambda: LAMBDA,
        pole_pairs: P,
        shape: c.shape,
        i_d0: 0.0,
        i_q0: c.i_q0,
        theta_e0: 0.0,
        saturation: c.sat,
    };
    let motor = MotorElectrical::new(
        ep,
        ElectricalInputs {
            v: ids,
            theta_e: th,
            omega_e: w,
        },
        &mut bus,
    )
    .unwrap();
    rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let mut plant = Plant::new();
    let theta_m = bus.id("motor.theta").unwrap();
    let omega_m = bus.id("motor.omega").unwrap();
    let cog = c
        .cogging
        .map(|p| Cogging::new(p, theta_m, &mut bus).unwrap());
    let fe = c.iron.map(|p| IronLoss::new(p, omega_m, &mut bus).unwrap());
    if let Some(m) = &cog {
        rotor = rotor.with_internal(m.torque_id());
    }
    if let Some(m) = &fe {
        rotor = rotor.with_internal(m.torque_id());
    }
    plant.add(Box::new(rotor));
    plant.add(Box::new(Source { ids, th, w, f: c.v }));
    plant.add(Box::new(motor));
    if let Some(m) = cog {
        plant.add(Box::new(m));
    }
    if let Some(m) = fe {
        plant.add(Box::new(m));
    }
    Engine::new(plant, bus, vec![], c.dt)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

fn run(e: &mut Engine, t: f64) {
    e.step_until(SimTime::from_secs_f64(t)).unwrap();
}

/// dq voltages applied at the model's own θe (an ideal source in the rotor frame).
fn dq(vd: f64, vq: f64) -> VFn {
    Box::new(move |_, th, _| {
        let (s, c) = th.sin_cos();
        inv_clarke(c * vd - s * vq, s * vd + c * vq)
    })
}

fn close(x: f64, want: f64, atol: f64, rtol: f64) -> bool {
    (x - want).abs() <= atol + rtol * want.abs()
}

#[test]
fn v_mot_001_locked_rotor_rl_step() {
    let mut e = build(Case {
        locked: true,
        v: dq(0.0, 1.0),
        ..Default::default()
    });
    for t in [1e-3, 2.5e-3, 5e-3, 1e-2] {
        run(&mut e, t);
        let want = 1.0 / R * (1.0 - (-t * R / L).exp());
        assert!(
            close(sig(&e, "motor.i_q"), want, 0.005 / R, 1e-6),
            "t = {t}"
        );
    }
}

#[test]
fn v_mot_002_free_spin_steady_speed() {
    let (v, b) = (6.0, 1e-4);
    let mut e = build(Case {
        j: 1e-5,
        b,
        v: dq(0.0, v),
        ..Default::default()
    });
    run(&mut e, 1.0);
    // Steady state of EQ-MOT-06 + B·ω: solve for ω by bisection on the torque balance.
    let torque_gap = |w: f64| {
        let we = P * w;
        // 0 = −R i_d + ωe L i_q; 0 = v − R i_q − ωe L i_d − ωe λ.
        let iq = (v - we * LAMBDA) * R / (R * R + we * we * L * L);
        1.5 * P * LAMBDA * iq - b * w
    };
    let (mut lo, mut hi) = (0.0, v / (P * LAMBDA));
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if torque_gap(mid) > 0.0 {
            lo = mid
        } else {
            hi = mid
        }
    }
    assert!(
        close(sig(&e, "motor.omega"), lo, 0.0, 5e-3),
        "{} vs {lo}",
        sig(&e, "motor.omega")
    );
}

#[test]
fn v_mot_003_open_circuit_back_emf() {
    // "Open circuit": terminals follow the back-EMF, so no current flows.
    let w_m = 100.0;
    let emf: VFn = Box::new(|_, th, we| {
        std::array::from_fn(|k| -we * LAMBDA * (th - k as f64 * 2.0 * PI / 3.0).sin())
    });
    let mut e = build(Case {
        j: 1e9,
        omega0: w_m,
        v: emf,
        ..Default::default()
    });
    let mut peak: f64 = 0.0;
    let mut t = 0.0;
    while t < 1.5 * 2.0 * PI / (P * w_m) {
        t += 1e-6;
        run(&mut e, t);
        peak = peak.max(sig(&e, "motor.e_a") - sig(&e, "motor.e_b"));
        assert!(
            sig(&e, "motor.i_a").abs() < 1e-9,
            "open circuit: no current"
        );
    }
    let ke = 3f64.sqrt() * P * LAMBDA;
    assert!(close(peak, ke * w_m, 0.0, 1e-6), "{peak}");
}

#[test]
fn v_mot_004_stall_torque() {
    let iq = 2.0;
    let mut e = build(Case {
        locked: true,
        i_q0: iq,
        v: dq(0.0, R * iq),
        ..Default::default()
    });
    run(&mut e, 0.01);
    assert!(close(
        sig(&e, "motor.torque_em"),
        1.5 * P * LAMBDA * iq,
        0.0,
        1e-9
    ));
}

#[test]
fn v_mot_006_kv_conversion() {
    let kv =
        sim_model::units::parse_quantity("100 rpm/V", sim_model::units::Kind::VelocityConstant)
            .unwrap();
    let p = 14.0;
    let lambda = 1.0 / (kv * 3f64.sqrt() * p); // EQ-CONV-13
    assert!(close(lambda, 3.9381e-3, 0.0, 1e-5), "{lambda}");
    assert!(close(1.5 * p * lambda, 82.699e-3, 0.0, 1e-5));
}

#[test]
fn v_mot_007_cogging_unpowered() {
    // Unpowered: the terminals follow the back-EMF (no current), so only cogging acts.
    let emf: VFn = Box::new(|_, th, we| {
        std::array::from_fn(|k| -we * LAMBDA * (th - k as f64 * 2.0 * PI / 3.0).sin())
    });
    let cog = CoggingParams {
        n_c: 84.0,
        terms: vec![(0.02, 0.3)],
    };
    let mut e = build(Case {
        j: 2e-5,
        omega0: 3.0,
        cogging: Some(cog),
        v: emf,
        dt: 2e-5,
        ..Default::default()
    });
    run(&mut e, 1.0);
    assert!(
        sig(&e, "energy.residual").abs() < 1e-6,
        "{}",
        sig(&e, "energy.residual")
    );
    assert!((sig(&e, "motor.omega") - 3.0).abs() > 1e-4, "cogging acted");
}

#[test]
fn v_mot_008_iron_loss_spin_down() {
    let (j, k_hy, k_ed, w0) = (2e-4, 2e-4, 1e-6, 50.0);
    let emf: VFn = Box::new(|_, th, we| {
        std::array::from_fn(|k| -we * LAMBDA * (th - k as f64 * 2.0 * PI / 3.0).sin())
    });
    let fe = IronLossParams {
        k_hy,
        k_ed,
        pole_pairs: P,
    };
    let mut e = build(Case {
        j,
        omega0: w0,
        iron: Some(fe),
        v: emf,
        dt: 1e-4,
        ..Default::default()
    });
    run(&mut e, 2.0);
    // Reference: integrate J·ω' = T_fe(ω) (with the tanh) by RK4 at dt = 1e-5.
    let tfe = |w: f64| -(k_hy * P * (w / 0.01).tanh() + k_ed * P * P * w);
    let (mut w, h) = (w0, 1e-5);
    for _ in 0..200_000 {
        let k1 = tfe(w) / j;
        let k2 = tfe(w + 0.5 * h * k1) / j;
        let k3 = tfe(w + 0.5 * h * k2) / j;
        let k4 = tfe(w + h * k3) / j;
        w += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    assert!(
        close(sig(&e, "motor.omega"), w, 1e-9, 1e-4),
        "{} vs {w}",
        sig(&e, "motor.omega")
    );
}

#[test]
fn v_mot_009_saturation_linear_limit() {
    let lin = SatParams {
        i_k: 10.0,
        l_inf: L,
        c: 0.0,
    };
    let drive = || dq(1.0, 8.0);
    let (mut a, mut b) = (
        build(Case {
            sat: Some(lin),
            v: drive(),
            ..Default::default()
        }),
        build(Case {
            v: drive(),
            ..Default::default()
        }),
    );
    for k in 1..=50 {
        let t = k as f64 * 2e-3;
        run(&mut a, t);
        run(&mut b, t);
        for n in ["motor.i_d", "motor.i_q", "motor.omega"] {
            assert!(close(sig(&a, n), sig(&b, n), 1e-12, 0.0), "{n} at {t}");
        }
    }
}

#[test]
fn v_mot_010_saturated_torque_per_amp() {
    let sat = SatParams {
        i_k: 20.0,
        l_inf: 0.75e-3,
        c: 0.05,
    };
    let iq = 2.0 * sat.i_k;
    let mut e = build(Case {
        sat: Some(sat),
        locked: true,
        v: dq(0.0, R * iq),
        ..Default::default()
    });
    run(&mut e, 0.05);
    let u = sig(&e, "motor.i_q") / sat.i_k;
    let h = sat.c * u * u / (1.0 + u * u);
    let tpa = sig(&e, "motor.torque_em") / sig(&e, "motor.i_q");
    assert!(close(tpa, 1.5 * P * LAMBDA * (1.0 - h), 0.0, 1e-3));
    assert!(close(sig(&e, "motor.i_q"), iq, 0.0, 1e-3));
    assert!(sig(&e, "energy.residual").abs() < 1e-3);
}

#[test]
fn v_mot_011_open_phase_c() {
    let drive: VFn = Box::new(|t, _, _| {
        std::array::from_fn(|k| {
            12.0 + 8.0 * (2.0 * PI * 30.0 * t - k as f64 * 2.0 * PI / 3.0).cos()
        })
    });
    let mut e = build(Case {
        b: 1e-4,
        v: drive,
        dt: 2e-6,
        ..Default::default()
    });
    run(&mut e, 0.05);
    e.queue(EngineCommand::SetParam {
        path: "motor.open_phase".into(),
        value: 2.0,
        source: ChangeSource::Internal,
    });
    for k in 1..=500 {
        run(&mut e, 0.05 + k as f64 * 1e-4);
        let (ia, ib) = (sig(&e, "motor.i_a"), sig(&e, "motor.i_b"));
        assert!(ia.is_finite() && ib.is_finite());
        if sig(&e, "motor.open_phase") == 2.0 {
            assert!(close(sig(&e, "motor.i_c"), 0.0, 1e-12, 0.0));
            assert!(close(ia, -ib, 1e-12, 0.0));
        }
    }
    assert_eq!(sig(&e, "motor.open_phase"), 2.0);
    assert!(sig(&e, "energy.residual").abs() < 1e-3);
}

/// Torque ripple of ideal 120° block currents (+I to the phase with the largest k,
/// −I to the smallest) over one electrical period, excluding `skip` rad around each
/// commutation.
fn six_step_ripple(shape: Shape, skip: f64) -> f64 {
    let mut bus = SignalBus::new();
    let ids = ["inverter.v_a", "inverter.v_b", "inverter.v_c"]
        .map(|n| bus.register(n, "V", "", SignalKind::Output).unwrap());
    let th = bus
        .register("motor.theta_e", "rad", "", SignalKind::State)
        .unwrap();
    let w = bus
        .register("motor.omega_e", "rad/s", "", SignalKind::State)
        .unwrap();
    let ep = ElectricalParams {
        r: R,
        ld: L,
        lq: L,
        lambda: LAMBDA,
        pole_pairs: P,
        shape: shape.clone(),
        i_d0: 0.0,
        i_q0: 0.0,
        theta_e0: 0.0,
        saturation: None,
    };
    let m = MotorElectrical::new(
        ep,
        ElectricalInputs {
            v: ids,
            theta_e: th,
            omega_e: w,
        },
        &mut bus,
    )
    .unwrap();
    let (mut tmax, mut tmin) = (f64::MIN, f64::MAX);
    let n = 36_000;
    for i in 0..n {
        let t = 2.0 * PI * i as f64 / n as f64;
        let k: [f64; 3] = std::array::from_fn(|x| shape.k(t - x as f64 * 2.0 * PI / 3.0));
        let hi = (0..3).max_by(|&a, &b| k[a].total_cmp(&k[b])).unwrap();
        let lo = (0..3).min_by(|&a, &b| k[a].total_cmp(&k[b])).unwrap();
        // Commutation happens where the choice of phases changes: every 60° from 30°
        // for −sin-type shapes. Skip a window around those angles.
        let to_boundary = ((t - PI / 6.0).rem_euclid(PI / 3.0))
            .min(PI / 3.0 - (t - PI / 6.0).rem_euclid(PI / 3.0));
        if to_boundary < skip {
            continue;
        }
        let mut cur = [0.0; 3];
        cur[hi] = 1.0;
        cur[lo] = -1.0;
        let (ia, ib) = clarke(cur[0], cur[1], cur[2]);
        let s = m.from_currents(ia, ib, t, LAMBDA);
        let te = m.torque(&s, t, LAMBDA);
        tmax = tmax.max(te);
        tmin = tmin.min(te);
    }
    (tmax - tmin) / tmax
}

#[test]
fn v_mot_012_six_step_on_sinusoidal_machine() {
    let r = six_step_ripple(Shape::Sinusoidal, 0.0);
    assert!(close(r, 1.0 - (PI / 6.0).cos(), 0.005, 0.0), "{r}");
}

#[test]
fn v_mot_013_six_step_on_trapezoidal_machine() {
    let r = six_step_ripple(Shape::Trapezoidal { w: 2.0 * PI / 3.0 }, 1e-3);
    assert!(r.abs() < 0.005, "{r}");
}
