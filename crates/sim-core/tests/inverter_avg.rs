//! P07.T03 / V-INV-avg: the averaged inverter (EQ-INV-03) agrees with the switching
//! model (PWM edges + dead time, EQ-INV-01/02) on period-averaged currents.

use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::rng::RngService;
use sim_core::engine::signals::{SignalBus, SignalId, SignalKind};
use sim_core::engine::time::SimTime;
use sim_core::physics::inverter::leg::{Devices, LegState, SwitchingInverter, duty_ids};
use sim_core::physics::inverter::pwm::{Pwm, PwmConfig};
use sim_core::physics::mech::rotor::{RotorParams, RotorRigid};
use sim_core::physics::motor::backemf::Shape;
use sim_core::physics::motor::electrical::{
    ElectricalInputs, ElectricalParams, LegInputs, MotorElectrical,
};
use sim_core::physics::sensors::adc::{Adc, AdcConfig, Channel};

const D: Devices = Devices {
    r_on: 0.02,
    v_f: 0.8,
};
const V_BUS: f64 = 24.0;
const PWM: PwmConfig = PwmConfig {
    period_ns: 50_000,
    dead_ns: 500,
};
const DUTY: [f64; 3] = [0.7, 0.4, 0.4];

struct Bus(SignalId);
impl PlantModule for Bus {
    fn name(&self) -> &str {
        "bus"
    }
    fn n_states(&self) -> usize {
        0
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![]
    }
    fn init(&self, _: &mut [f64]) {}
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, bus: &mut SignalBus) {
        bus.set(self.0, V_BUS);
    }
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, _: &mut [f64]) {}
}

fn rig(averaged: bool) -> Engine {
    rig_f(averaged, 20e3)
}

fn rig_f(averaged: bool, f_loss: f64) -> Engine {
    rig_adc(averaged, f_loss, false)
}

/// With `adc`, a 12-bit two-shunt ADC (±20 A) samples at `pwm.center` (period T_pwm,
/// phase 0).
fn rig_adc(averaged: bool, f_loss: f64, adc: bool) -> Engine {
    let mut bus = SignalBus::new();
    let vb = bus.register("bus.v", "V", "", SignalKind::Output).unwrap();
    let mut inv = SwitchingInverter::new(D, vb, &mut bus)
        .unwrap()
        .with_switching_loss(60e-9, f_loss);
    if averaged {
        inv = inv
            .averaged(&mut bus, PWM.dead_ns as f64 / PWM.period_ns as f64, 0.01)
            .unwrap();
    }
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
        v: inv.outputs_ids(),
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
    let legs = ["inverter.leg_a", "inverter.leg_b", "inverter.leg_c"].map(|n| bus.id(n).unwrap());
    let motor = MotorElectrical::new(ep, inp, &mut bus)
        .unwrap()
        .with_legs(LegInputs {
            legs,
            v_bus: vb,
            v_f: D.v_f,
            r_on: D.r_on,
            ho_duty: None,
            dead_time: PWM.dead_ns as f64 * 1e-9,
        })
        .with_internal_terminals();
    let rotor = rotor.with_internal(bus.id("motor.torque_em").unwrap());
    let inv = inv.with_motor(&bus).unwrap();
    for (id, d) in duty_ids(&mut bus).unwrap().iter().zip(DUTY) {
        bus.set(*id, d);
    }
    let mut blocks: Vec<Box<dyn sim_core::engine::block::DiscreteBlock>> = if averaged {
        // Averaged: every leg is driven by its duty.
        inv.set_legs(&mut bus, [LegState::High; 3]);
        vec![]
    } else {
        vec![Box::new(Pwm::new(PWM, &mut bus).unwrap())]
    };
    if adc {
        let ch = Channel {
            full_scale: 20.0,
            ..Default::default()
        };
        let cfg = AdcConfig {
            bits: 12,
            phase: [ch; 3],
            shunts: 2,
            bus: None,
            noise: 0.0,
            bus_noise: 0.0,
            latency_ns: 0,
            bypass: false,
        };
        let period = SimTime::from_nanos(PWM.period_ns);
        blocks.push(Box::new(
            Adc::new(&cfg, period, &RngService::new(1), &mut bus).unwrap(),
        ));
    }
    let mut plant = Plant::new();
    plant.add(Box::new(Bus(vb)));
    plant.add(Box::new(rotor));
    plant.add(Box::new(motor));
    plant.add(Box::new(inv));
    Engine::new(plant, bus, blocks, 1e-6)
}

fn sig(e: &Engine, n: &str) -> f64 {
    e.bus.get(e.bus.id(n).unwrap())
}

#[test]
fn v_inv_avg_locked_rotor_matches_switching_period_averages() {
    let (mut sw, mut av) = (rig(false), rig(true));
    // τ = L/R = 2.5 ms: 40 ms is 16 τ.
    let t0 = 0.04;
    sw.step_until(SimTime::from_secs_f64(t0)).unwrap();
    av.step_until(SimTime::from_secs_f64(t0)).unwrap();
    // Period average of the switching currents (200 samples per 50 µs period).
    let n = 200;
    let mut mean = [0.0; 3];
    for k in 1..=n {
        sw.step_until(SimTime::from_nanos((t0 * 1e9) as i64 + k * 250))
            .unwrap();
        for (x, name) in ["motor.i_a", "motor.i_b", "motor.i_c"].iter().enumerate() {
            mean[x] += sig(&sw, name) / n as f64;
        }
    }
    let rated = sig(&av, "motor.i_a").abs();
    assert!(rated > 3.0, "a real operating point: {rated} A");
    for (x, name) in ["motor.i_a", "motor.i_b", "motor.i_c"].iter().enumerate() {
        let a = sig(&av, name);
        assert!(
            (mean[x] - a).abs() <= 0.01 * rated,
            "{name}: switching {} vs averaged {a}",
            mean[x]
        );
    }
    // The dead-time error is visible: without it the current would be ~5 % higher.
    let ideal = (DUTY[0] - DUTY[1]) * V_BUS / (1.5 * (1.0 + D.r_on));
    assert!(
        sig(&av, "motor.i_a") < 0.97 * ideal,
        "{} vs {ideal}",
        sig(&av, "motor.i_a")
    );
}

/// P07.T05: DC input = phase output + inverter loss (conduction, diodes, switching) in
/// both modes, so the balance closes; the switching-loss current scales with f_pwm.
#[test]
fn v_inv_007_008_losses_close_the_balance_and_scale_with_f_pwm() {
    for averaged in [false, true] {
        let mut e = rig(averaged);
        e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        let r = sig(&e, "energy.residual");
        assert!(r.abs() < 1e-6, "averaged = {averaged}: residual {r}");
        assert!(sig(&e, "energy.loss.inverter") > 0.0);
        assert!(sig(&e, "inverter.p_loss") > 0.0);
    }
    // Averaged mode, from the EQ-INV-09 terms per leg (not from P_dc − Σ v·i): MOSFET
    // conduction R_on·i², corrected for the dead-time share 2·(t_d/T)·σ·R_on·|i|·i the
    // diodes carry instead, diode conduction 2·V_f·(t_d/T)·σ·i, and switching V·i_sw with
    // i_sw = f_pwm·Σ ½|i|(t_r + t_f).
    let mut e = rig(true);
    e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
    let f = PWM.dead_ns as f64 / PWM.period_ns as f64;
    let (mut cond, mut i_sw) = (0.0, 0.0);
    for name in ["motor.i_a", "motor.i_b", "motor.i_c"] {
        let i = sig(&e, name);
        let s = (i / 0.01).tanh();
        cond += D.r_on * i * i - 2.0 * f * s * D.r_on * i.abs() * i + 2.0 * f * s * D.v_f * i;
        i_sw += 20e3 * 0.5 * i.abs() * 60e-9;
    }
    let want = cond + V_BUS * i_sw;
    let p = sig(&e, "inverter.p_loss");
    assert!(
        (p / want - 1.0).abs() < 1e-9,
        "p_loss {p} vs EQ-INV-09 {want}"
    );
    let i_sw = |f: f64| {
        let mut e = rig_f(true, f);
        e.step_until(SimTime::from_secs_f64(0.02)).unwrap();
        sig(&e, "inverter.i_sw")
    };
    let (a, b) = (i_sw(20e3), i_sw(40e3));
    assert!(a > 0.0 && (b / a - 2.0).abs() < 1e-12, "{a} {b}");
}

/// V-SENS-005: in switching mode the ADC samples at `pwm.center` (the middle of the
/// zero vector), where the ripple is at its midpoint: the reading matches the period
/// average within 1 % of the operating current.
#[test]
fn v_sens_005_center_sampling_reads_the_period_average() {
    let mut e = rig_adc(false, 20e3, true);
    let t0 = 0.04;
    e.step_until(SimTime::from_nanos((t0 * 1e9) as i64 + 1))
        .unwrap();
    let read = ["sensors.adc.i_a", "sensors.adc.i_b", "sensors.adc.i_c"].map(|n| sig(&e, n));
    // Average over the period centred on the sample, [t0 − T/2, t0 + T/2): the zero
    // vector the ADC samples is centred on t0 (fine grid, 250 ns).
    let mut e = rig_adc(false, 20e3, true);
    let half = PWM.period_ns / 2;
    let start = (t0 * 1e9) as i64 - half;
    e.step_until(SimTime::from_nanos(start)).unwrap();
    let n = 200;
    let mut mean = [0.0; 3];
    for k in 1..=n {
        e.step_until(SimTime::from_nanos(start + k * 250)).unwrap();
        for (x, name) in ["motor.i_a", "motor.i_b", "motor.i_c"].iter().enumerate() {
            mean[x] += sig(&e, name) / n as f64;
        }
    }
    let rated = mean[0].abs();
    assert!(rated > 3.0, "{rated}");
    for x in 0..3 {
        assert!(
            (read[x] - mean[x]).abs() <= 0.01 * rated,
            "phase {x}: ADC {} vs mean {}",
            read[x],
            mean[x]
        );
    }
}
