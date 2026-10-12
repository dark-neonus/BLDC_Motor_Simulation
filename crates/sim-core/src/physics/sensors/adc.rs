//! Current and bus-voltage ADC with shunt amplifiers (EQ-SENS-06):
//!
//! i_meas = sat_[−I_fs, I_fs](Q_Δ((1 + g_err)·i_x + i_off + n)),  Δ = 2·I_fs/2^b,
//!
//! one pipeline per channel (own RNG substream). Two shunts measure a and b and
//! reconstruct i_c = −i_a − i_b; three shunts measure all. The bus channel uses the same
//! model with its own full scale. Sampling: period T_pwm, phase 0 (= `pwm.center`, the
//! middle of the zero vector) in switching mode, the controller period otherwise.

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::rng::RngService;
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

use super::pipeline::{PipelineConfig, SensorPipeline};

/// One ADC channel: full scale ±fs, gain error, offset (signal units).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Channel {
    pub full_scale: f64,
    pub gain_error: f64,
    pub offset: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdcConfig {
    pub bits: u32,
    /// Phase channels a, b, c (c unused with two shunts).
    pub phase: [Channel; 3],
    /// 2 or 3.
    pub shunts: u32,
    /// Bus-voltage channel (needs `bus.v`).
    pub bus: Option<Channel>,
    /// Current noise σ [A], bus noise σ [V].
    pub noise: f64,
    pub bus_noise: f64,
    pub latency_ns: i64,
    pub bypass: bool,
}

impl AdcConfig {
    fn pipe(&self, ch: &Channel, sigma: f64, rng: &RngService, id: &str) -> SensorPipeline {
        let cfg = PipelineConfig {
            bypass: self.bypass,
            latency_ns: self.latency_ns,
            noise_sigma: sigma,
            step: 2.0 * ch.full_scale / 2f64.powi(self.bits as i32),
            wrap: None,
            range: Some((-ch.full_scale, ch.full_scale)),
        };
        let (g, off) = (ch.gain_error, ch.offset);
        SensorPipeline::new(cfg, rng.stream(id)).with_error(Box::new(move |x| g * x + off))
    }
}

pub struct Adc {
    period: SimTime,
    phase: SimTime,
    shunts: u32,
    i: [SignalId; 3],
    v_bus: Option<SignalId>,
    out: [SignalId; 3],
    out_bus: Option<SignalId>,
    pipes: Vec<SensorPipeline>,
    bus_pipe: Option<SensorPipeline>,
}

impl Adc {
    /// Reads `motor.i_a..c` (and `bus.v` with a bus channel); registers
    /// `sensors.adc.i_a..c` (and `sensors.adc.v_bus`).
    pub fn new(
        cfg: &AdcConfig,
        period: SimTime,
        rng: &RngService,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let i = [
            bus.id("motor.i_a")?,
            bus.id("motor.i_b")?,
            bus.id("motor.i_c")?,
        ];
        let v_bus = match cfg.bus {
            Some(_) => Some(bus.id("bus.v")?),
            None => None,
        };
        let mut reg = |p: &str, u: &str, d: &str| bus.register(p, u, d, SignalKind::Output);
        let out = [
            reg("sensors.adc.i_a", "A", "measured phase-a current")?,
            reg("sensors.adc.i_b", "A", "measured phase-b current")?,
            reg(
                "sensors.adc.i_c",
                "A",
                "measured phase-c current (−a − b with two shunts)",
            )?,
        ];
        let out_bus = match cfg.bus {
            Some(_) => Some(reg("sensors.adc.v_bus", "V", "measured bus voltage")?),
            None => None,
        };
        let n = if cfg.shunts == 3 { 3 } else { 2 };
        let pipes = (0..n)
            .map(|k| {
                let id = ["sensors.adc.i_a", "sensors.adc.i_b", "sensors.adc.i_c"][k];
                cfg.pipe(&cfg.phase[k], cfg.noise, rng, id)
            })
            .collect();
        let bus_pipe = cfg
            .bus
            .map(|ch| cfg.pipe(&ch, cfg.bus_noise, rng, "sensors.adc.v_bus"));
        Ok(Self {
            period,
            phase: SimTime::from_nanos(0),
            shunts: n as u32,
            i,
            v_bus,
            out,
            out_bus,
            pipes,
            bus_pipe,
        })
    }

    /// First sample offset within the period (0 = `pwm.center`).
    pub fn with_phase(mut self, phase: SimTime) -> Self {
        self.phase = phase;
        self
    }
}

impl DiscreteBlock for Adc {
    fn id(&self) -> &str {
        "sensors.adc"
    }
    fn period(&self) -> Option<SimTime> {
        Some(self.period)
    }
    fn phase(&self) -> SimTime {
        self.phase
    }
    fn priority(&self) -> u8 {
        priority::SENSOR
    }
    fn step(&mut self, ctx: &mut StepCtx<'_>) -> Result<(), SimError> {
        let t = ctx.t.nanos();
        let mut y = [0.0; 3];
        for (k, p) in self.pipes.iter_mut().enumerate() {
            y[k] = p.sample(t, ctx.bus.get(self.i[k]));
        }
        if self.shunts == 2 {
            y[2] = -y[0] - y[1];
        }
        for (id, v) in self.out.iter().zip(y) {
            ctx.bus.set(*id, v);
        }
        if let (Some(p), Some(src), Some(out)) = (&mut self.bus_pipe, self.v_bus, self.out_bus) {
            let v = p.sample(t, ctx.bus.get(src));
            ctx.bus.set(out, v);
        }
        Ok(())
    }
    fn reset(&mut self) {
        self.pipes.iter_mut().for_each(SensorPipeline::reset);
        if let Some(p) = &mut self.bus_pipe {
            p.reset();
        }
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({
            "pipes": self.pipes.iter().map(SensorPipeline::save).collect::<Vec<_>>(),
            "bus": self.bus_pipe.as_ref().map(SensorPipeline::save),
        })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let pipes = v
            .get("pipes")
            .and_then(serde_json::Value::as_array)
            .ok_or("missing `pipes`")?;
        if pipes.len() != self.pipes.len() {
            return Err(format!(
                "{} ADC channels, snapshot has {}",
                self.pipes.len(),
                pipes.len()
            ));
        }
        for (p, s) in self.pipes.iter_mut().zip(pipes) {
            p.restore(s)?;
        }
        if let (Some(p), Some(s)) = (&mut self.bus_pipe, v.get("bus").filter(|s| !s.is_null())) {
            p.restore(s)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::engine::Engine;
    use crate::engine::plant::Plant;

    fn cfg() -> AdcConfig {
        let ch = Channel {
            full_scale: 20.0,
            ..Default::default()
        };
        AdcConfig {
            bits: 12,
            phase: [ch; 3],
            shunts: 2,
            bus: Some(Channel {
                full_scale: 60.0,
                ..Default::default()
            }),
            noise: 0.0,
            bus_noise: 0.0,
            latency_ns: 0,
            bypass: false,
        }
    }

    /// Phase currents and the bus voltage set directly; one sample at t = 0.
    fn read(c: &AdcConfig, i: [f64; 3], v: f64) -> ([f64; 3], f64) {
        let mut bus = SignalBus::new();
        let ids = ["motor.i_a", "motor.i_b", "motor.i_c"]
            .map(|n| bus.register(n, "A", "", SignalKind::Input).unwrap());
        let vb = bus.register("bus.v", "V", "", SignalKind::Input).unwrap();
        let adc = Adc::new(
            c,
            SimTime::from_nanos(50_000),
            &RngService::new(9),
            &mut bus,
        )
        .unwrap();
        for (id, x) in ids.iter().zip(i) {
            bus.set(*id, x);
        }
        bus.set(vb, v);
        let mut e = Engine::new(Plant::new(), bus, vec![Box::new(adc)], 1e-6);
        e.step_until(SimTime::from_nanos(1)).unwrap();
        let g = |n: &str| e.bus.get(e.bus.id(n).unwrap());
        (
            [
                g("sensors.adc.i_a"),
                g("sensors.adc.i_b"),
                g("sensors.adc.i_c"),
            ],
            g("sensors.adc.v_bus"),
        )
    }

    #[test]
    fn quantisation_saturation_and_two_shunt_reconstruction() {
        let d: f64 = 40.0 / 4096.0;
        let (y, v) = read(&cfg(), [3.0, -25.0, 22.0], 24.0);
        assert_eq!(y[0], d * (3.0 / d).round());
        assert_eq!(y[1], -20.0, "clamped at −I_fs");
        assert_eq!(y[2], -y[0] - y[1], "reconstructed, not measured");
        let dv: f64 = 60.0 / 2048.0;
        assert_eq!(v, dv * (24.0 / dv).round());
        // Three shunts: c is measured (and clamps on its own).
        let (y, _) = read(&AdcConfig { shunts: 3, ..cfg() }, [3.0, -25.0, 22.0], 24.0);
        assert_eq!(y[2], 20.0);
    }

    #[test]
    fn gain_and_offset_errors() {
        let mut c = cfg();
        c.bits = 24;
        c.phase[0].gain_error = 0.02;
        c.phase[0].offset = 0.1;
        let (y, _) = read(&c, [5.0, 0.0, 0.0], 0.0);
        let d = 40.0 / 2f64.powi(24);
        assert!((y[0] - (1.02 * 5.0 + 0.1)).abs() <= d);
        // Two shunts: the offset on a shows up in the reconstructed c.
        assert!((y[2] + y[0]).abs() <= d);
        let (y, _) = read(&AdcConfig { bypass: true, ..c }, [5.0, 0.0, 0.0], 0.0);
        assert_eq!(y[0], 5.0);
    }

    /// V-SENS-006: an offset i_off on phase a (two shunts, c reconstructed) puts an error
    /// vector of fixed length 2·i_off/√3 in αβ, i.e. an i_d, i_q ripple at ωe with that
    /// amplitude (amplitude-invariant Clarke/Park, EQ-MOT conventions).
    #[test]
    fn v_sens_006_offset_gives_dq_ripple_at_omega_e() {
        use crate::physics::motor::electrical::clarke;
        let mut c = cfg();
        c.bits = 24;
        let off = 0.1;
        c.phase[0].offset = off;
        let (amp, iq0) = (5.0, 5.0);
        let (mut d_err, mut q_err) = (vec![], vec![]);
        for k in 0..360 {
            let th = (k as f64).to_radians();
            // Pure i_q = 5 A at θe: i_x = −I·sin(θ − x·120°).
            let i: [f64; 3] = std::array::from_fn(|x| {
                -amp * (th - x as f64 * 2.0 * std::f64::consts::PI / 3.0).sin()
            });
            let (y, _) = read(&c, i, 0.0);
            let (al, be) = clarke(y[0], y[1], y[2]);
            let (s, co) = th.sin_cos();
            let (id, iq) = (co * al + s * be, -s * al + co * be);
            d_err.push(id);
            q_err.push(iq - iq0);
        }
        let want = 2.0 * off / 3f64.sqrt();
        let pk = |v: &[f64]| v.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        assert!(
            (pk(&d_err) / want - 1.0).abs() < 0.05,
            "i_d ripple {}",
            pk(&d_err)
        );
        assert!(
            (pk(&q_err) / want - 1.0).abs() < 0.05,
            "i_q ripple {}",
            pk(&q_err)
        );
        // One period per electrical turn: |error| is constant (a rotating vector).
        for (d, q) in d_err.iter().zip(&q_err) {
            assert!((d.hypot(*q) / want - 1.0).abs() < 1e-3);
        }
    }
}
