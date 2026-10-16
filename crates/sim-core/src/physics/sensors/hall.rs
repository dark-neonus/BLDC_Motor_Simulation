//! Three digital Hall sensors (EQ-SENS-02):
//!
//! h_x = [cos(θe − (x+1)·120° − δ_x) > (h/2)·s_x],  s_x = +1 while h_x = 0, −1 while 1.
//!
//! θe passes through the common pipeline first (latency, angle noise; EQ-SENS-01).
//! Outputs: `sensors.hall.a/.b/.c`, the code 4h_a + 2h_b + h_c (`sensors.hall.code`) and
//! the sector 1–6 in rotation order (`sensors.hall.sector`, 0 for the invalid codes 0/7).

use crate::engine::block::{DiscreteBlock, SimError, StepCtx, priority};
use crate::engine::signals::{SignalBus, SignalError, SignalId, SignalKind};
use crate::engine::time::SimTime;

use super::pipeline::SensorPipeline;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct HallConfig {
    /// Placement errors δ_a, δ_b, δ_c [rad electrical].
    pub offsets: [f64; 3],
    /// Hysteresis h (fraction of the normalised field).
    pub hysteresis: f64,
}

/// Hall states from θe, given the previous states (for the hysteresis).
pub fn hall_states(theta_e: f64, cfg: &HallConfig, prev: [bool; 3]) -> [bool; 3] {
    std::array::from_fn(|x| {
        let axis = (x as f64 + 1.0) * 2.0 * std::f64::consts::PI / 3.0;
        let c = (theta_e - axis - cfg.offsets[x]).cos();
        let s = if prev[x] { -1.0 } else { 1.0 };
        c > 0.5 * cfg.hysteresis * s
    })
}

/// Code 4h_a + 2h_b + h_c.
pub fn code(h: [bool; 3]) -> u8 {
    4 * u8::from(h[0]) + 2 * u8::from(h[1]) + u8::from(h[2])
}

/// Sector 1–6 in rotation order (1: 330°…30°, code 1; then 5, 4, 6, 2, 3); 0 if invalid.
pub fn sector(code: u8) -> u8 {
    match code {
        1 => 1,
        5 => 2,
        4 => 3,
        6 => 4,
        2 => 5,
        3 => 6,
        _ => 0,
    }
}

pub struct HallBlock {
    cfg: HallConfig,
    period: SimTime,
    theta_e: SignalId,
    out: [SignalId; 3],
    code: SignalId,
    sector: SignalId,
    pipe: SensorPipeline,
    state: [bool; 3],
    /// First-sample offset within the period.
    phase: SimTime,
}

impl HallBlock {
    /// Reads `motor.theta_e`; registers the Hall outputs. `pipe` should not quantise or
    /// wrap (the angle is used through cos).
    pub fn new(
        cfg: HallConfig,
        period: SimTime,
        pipe: SensorPipeline,
        bus: &mut SignalBus,
    ) -> Result<Self, SignalError> {
        let theta_e = bus.id("motor.theta_e")?;
        let mut reg = |p: &str, d: &str| bus.register(p, "-", d, SignalKind::Output);
        let out = [
            reg("sensors.hall.a", "Hall sensor A (0/1)")?,
            reg("sensors.hall.b", "Hall sensor B (0/1)")?,
            reg("sensors.hall.c", "Hall sensor C (0/1)")?,
        ];
        let code = reg("sensors.hall.code", "Hall code 4a + 2b + c (0, 7 invalid)")?;
        let sector = reg(
            "sensors.hall.sector",
            "Hall sector 1–6 (0 if the code is invalid)",
        )?;
        Ok(Self {
            cfg,
            period,
            theta_e,
            out,
            code,
            sector,
            pipe,
            state: [false; 3],
            phase: SimTime::from_nanos(0),
        })
    }
}

impl HallBlock {
    /// First sample offset within the period (default 0).
    pub fn with_phase(mut self, phase: SimTime) -> Self {
        self.phase = phase;
        self
    }
}

impl DiscreteBlock for HallBlock {
    fn id(&self) -> &str {
        "sensors.hall"
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
        let th = self.pipe.sample(ctx.t.nanos(), ctx.bus.get(self.theta_e));
        // Bypass: the ideal sensor set (no placement error, no hysteresis).
        let cfg = if self.pipe.config().bypass {
            HallConfig::default()
        } else {
            self.cfg
        };
        self.state = hall_states(th, &cfg, self.state);
        for (id, h) in self.out.iter().zip(self.state) {
            ctx.bus.set(*id, f64::from(u8::from(h)));
        }
        let c = code(self.state);
        ctx.bus.set(self.code, f64::from(c));
        ctx.bus.set(self.sector, f64::from(sector(c)));
        Ok(())
    }
    fn reset(&mut self) {
        self.pipe.reset();
        self.state = [false; 3];
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({ "pipe": self.pipe.save(), "state": self.state })
    }
    fn restore(&mut self, v: &serde_json::Value) -> Result<(), String> {
        let st = v.get("state").cloned().ok_or("missing `state`")?;
        self.state = serde_json::from_value(st).map_err(|e| e.to_string())?;
        self.pipe.restore(v.get("pipe").ok_or("missing `pipe`")?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::inverter::leg::LegState;
    use crate::physics::inverter::modulation::six_step;

    /// Codes per 1° step from 0.5° (no boundary hits), with the state carried over.
    fn sweep(cfg: &HallConfig, dir: f64) -> Vec<(f64, u8)> {
        let mut st = hall_states(0.0, &HallConfig::default(), [false; 3]);
        (0..720)
            .map(|k| {
                let deg = dir * (k as f64 + 0.5);
                st = hall_states(deg.to_radians(), cfg, st);
                (deg, code(st))
            })
            .collect()
    }

    /// V-SENS-004: over one electrical turn the code changes at 30° + 60°k and follows
    /// the EQ-SENS-02 table.
    #[test]
    fn v_sens_004_codes_change_every_60_degrees_at_the_six_step_boundaries() {
        let codes = sweep(&HallConfig::default(), 1.0);
        let changes: Vec<f64> = codes
            .windows(2)
            .filter(|w| w[0].1 != w[1].1)
            .map(|w| w[1].0 - 0.5)
            .collect();
        let want: Vec<f64> = (0..12).map(|k| 30.0 + 60.0 * k as f64).collect();
        assert_eq!(changes, want);
        // The table of EQ-SENS-02, and agreement with the six-step pairs of EQ-INV-07:
        // the High leg is the code's "+" phase.
        // (θe, code, sector, "+" phase, "−" phase); a = 0, b = 1, c = 2.
        for (deg, want_code, want_sector, hi, lo) in [
            (0.0, 1, 1, 1, 2),
            (60.0, 5, 2, 1, 0),
            (120.0, 4, 3, 2, 0),
            (180.0, 6, 4, 2, 1),
            (240.0, 2, 5, 0, 1),
            (300.0, 3, 6, 0, 2),
        ] {
            let c = code(hall_states(
                f64::to_radians(deg),
                &HallConfig::default(),
                [false; 3],
            ));
            assert_eq!((c, sector(c)), (want_code, want_sector), "{deg}°");
            let legs = six_step(f64::to_radians(deg), 0.0);
            assert_eq!(
                (legs[hi], legs[lo]),
                (LegState::High, LegState::Low),
                "{deg}°"
            );
        }
        assert_eq!((sector(0), sector(7)), (0, 0));
    }

    #[test]
    fn offsets_shift_their_sensor_and_hysteresis_splits_the_edges() {
        // δ_a = 10°: sensor a's edges (30° and 210°) move to 40° and 220°.
        let cfg = HallConfig {
            offsets: [10f64.to_radians(), 0.0, 0.0],
            hysteresis: 0.0,
        };
        let edges_a = |codes: &[(f64, u8)]| -> Vec<f64> {
            codes
                .windows(2)
                .filter(|w| (w[0].1 & 4) != (w[1].1 & 4))
                .map(|w| (w[1].0.abs() - 0.5).rem_euclid(360.0))
                .collect()
        };
        assert_eq!(edges_a(&sweep(&cfg, 1.0)), [40.0, 220.0, 40.0, 220.0]);
        // h = 0.2, forward: a turns on where cos = +0.1, arccos(0.1) = 84.26° before its
        // axis (120°), i.e. 35.74°, and off where cos = −0.1, 95.74° after it, i.e.
        // 215.74° (instead of 30° and 210°); the 1° grid sees them at the next degree.
        let cfg = HallConfig {
            offsets: [0.0; 3],
            hysteresis: 0.2,
        };
        let (on, off) = (
            120.0 - 0.1f64.acos().to_degrees(),
            120.0 + (-0.1f64).acos().to_degrees(),
        );
        assert!((on - 35.74).abs() < 0.01 && (off - 215.74).abs() < 0.01);
        let fwd = edges_a(&sweep(&cfg, 1.0));
        assert_eq!(fwd[..2], [on.ceil(), off.ceil()]);
    }

    #[test]
    fn block_writes_code_and_sector_and_bypass_ignores_the_offsets() {
        use crate::engine::engine::Engine;
        use crate::engine::plant::Plant;
        use crate::engine::rng::RngService;
        use crate::physics::sensors::pipeline::PipelineConfig;
        let run = |bypass: bool| {
            let mut bus = SignalBus::new();
            let th = bus
                .register("motor.theta_e", "rad", "", SignalKind::Input)
                .unwrap();
            let cfg = HallConfig {
                offsets: [0.0, 0.0, 40f64.to_radians()],
                hysteresis: 0.0,
            };
            let pipe = SensorPipeline::new(
                PipelineConfig {
                    bypass,
                    ..Default::default()
                },
                RngService::new(1).stream("sensors.hall"),
            );
            let hb = HallBlock::new(cfg, SimTime::from_nanos(50_000), pipe, &mut bus).unwrap();
            bus.set(th, 100f64.to_radians());
            let mut e = Engine::new(Plant::new(), bus, vec![Box::new(hb)], 1e-5);
            e.step_until(SimTime::from_nanos(60_000)).unwrap();
            let g = |n: &str| e.bus.get(e.bus.id(n).unwrap());
            (g("sensors.hall.code"), g("sensors.hall.sector"))
        };
        // 100°: ideal code 4 (sector 3). δ_c = 40° moves c's falling edge from 90° to
        // 130°, so with the offset c is still on: code 5 (sector 2). Bypass ignores it.
        assert_eq!(run(true), (4.0, 3.0));
        assert_eq!(run(false), (5.0, 2.0));
    }
}
