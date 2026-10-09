//! State-event localisation (P03.T15, EQ-NUM-05, V-NUM-003).

use std::sync::{Arc, Mutex};

use sim_core::engine::block::SimError;
use sim_core::engine::engine::Engine;
use sim_core::engine::plant::{Plant, PlantModule};
use sim_core::engine::signals::SignalBus;
use sim_core::engine::time::SimTime;

const G: f64 = 9.81;

/// Ball (height y, velocity v) bouncing with restitution e; records impact times.
struct Ball {
    e: f64,
    impacts: Arc<Mutex<Vec<f64>>>,
}
impl PlantModule for Ball {
    fn name(&self) -> &str {
        "ball"
    }
    fn n_states(&self) -> usize {
        2
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![("y".into(), "m".into()), ("v".into(), "m/s".into())]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = 1.0;
        x[1] = 0.0;
    }
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
    fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
        dx[0] = x[o + 1];
        dx[1] = -G;
    }
    fn n_events(&self) -> usize {
        1
    }
    fn event_functions(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, g: &mut [f64]) {
        g[0] = x[o];
    }
    fn on_event(&mut self, _: usize, rising: bool, t: f64, x: &mut [f64], _: &mut SignalBus) {
        if !rising {
            x[0] = 0.0;
            x[1] *= -self.e;
            self.impacts.lock().unwrap().push(t);
        }
    }
}

fn ball_engine(e: f64) -> (Engine, Arc<Mutex<Vec<f64>>>) {
    let impacts = Arc::new(Mutex::new(Vec::new()));
    let mut p = Plant::new();
    p.add(Box::new(Ball {
        e,
        impacts: Arc::clone(&impacts),
    }));
    (Engine::new(p, SignalBus::new(), vec![], 1e-3), impacts)
}

#[test]
fn bouncing_ball_impacts_located_to_a_nanosecond() {
    let e = 0.9;
    let (mut eng, impacts) = ball_engine(e);
    eng.step_until(SimTime::from_secs_f64(6.0)).unwrap();
    // Analytic: first impact √(2h/g); then intervals 2·v_k/g with v_k = e^k·√(2gh).
    let v1 = (2.0 * G * 1.0f64).sqrt();
    let mut t = v1 / G;
    let mut expected = vec![t];
    let mut v = v1;
    for _ in 0..9 {
        v *= e;
        t += 2.0 * v / G;
        expected.push(t);
    }
    let got = impacts.lock().unwrap().clone();
    assert!(got.len() >= 10, "only {} impacts", got.len());
    // T15 criterion is per-impact localisation (≤ 1 ns). Absolute times accumulate earlier
    // impacts' errors, so check the first impact and each bounce *interval* separately.
    assert!(
        (got[0] - expected[0]).abs() <= 1e-9,
        "impact 0: {} vs {}",
        got[0],
        expected[0]
    );
    for k in 1..10 {
        let (dg, de) = (got[k] - got[k - 1], expected[k] - expected[k - 1]);
        assert!((dg - de).abs() <= 1e-9, "interval {k}: {dg} vs {de}");
    }
}

#[test]
fn zeno_cascade_is_stopped_with_an_error() {
    let (mut eng, _) = ball_engine(0.5);
    // Bounces accumulate at t∞ = (1+e)/(1−e)·√(2h/g) ≈ 1.36 s; running past it must not hang.
    let r = eng.step_until(SimTime::from_secs_f64(3.0));
    assert!(matches!(r, Err(SimError::Numerical { .. })), "{r:?}");
}

/// Thermostat with hysteresis: heater on below 20, off above 22 (states: T, on-flag held in module).
struct Thermostat {
    on: bool,
    switches: Arc<Mutex<Vec<(f64, f64)>>>,
}
impl PlantModule for Thermostat {
    fn name(&self) -> &str {
        "thermo"
    }
    fn n_states(&self) -> usize {
        1
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![("T".into(), "°C".into())]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = 18.0;
    }
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
    fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
        let heat = if self.on { 5.0 } else { 0.0 };
        dx[0] = heat - 0.2 * (x[o] - 10.0);
    }
    fn n_events(&self) -> usize {
        2
    }
    fn event_functions(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, g: &mut [f64]) {
        g[0] = x[o] - 22.0; // rising → switch off
        g[1] = x[o] - 20.0; // falling → switch on
    }
    fn on_event(&mut self, idx: usize, rising: bool, _: f64, x: &mut [f64], _: &mut SignalBus) {
        if idx == 0 && rising && self.on {
            self.on = false;
            self.switches.lock().unwrap().push((x[0], 22.0));
        } else if idx == 1 && !rising && !self.on {
            self.on = true;
            self.switches.lock().unwrap().push((x[0], 20.0));
        }
    }
}

#[test]
fn hysteresis_switches_at_exact_thresholds() {
    let sw = Arc::new(Mutex::new(Vec::new()));
    let mut p = Plant::new();
    p.add(Box::new(Thermostat {
        on: true,
        switches: Arc::clone(&sw),
    }));
    let mut eng = Engine::new(p, SignalBus::new(), vec![], 1e-2);
    eng.step_until(SimTime::from_secs_f64(60.0)).unwrap();
    let sw = sw.lock().unwrap();
    assert!(sw.len() >= 4, "{} switches", sw.len());
    for (temp, thr) in sw.iter() {
        // dT/dt ≈ 3 K/s, crossing located to 1 ns → < 1e-8 K.
        assert!((temp - thr).abs() < 1e-8, "{temp} vs {thr}");
    }
}

/// Module with one event g = x − level, x' = 1 (crosses `level` at t = level).
struct Ramp {
    name: &'static str,
    level: f64,
    hits: Arc<Mutex<Vec<(String, f64)>>>,
}
impl PlantModule for Ramp {
    fn name(&self) -> &str {
        self.name
    }
    fn n_states(&self) -> usize {
        1
    }
    fn state_names(&self) -> Vec<(String, String)> {
        vec![("x".into(), "-".into())]
    }
    fn init(&self, x: &mut [f64]) {
        x[0] = 0.0;
    }
    fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
    fn derivatives(&self, _: f64, _: &[f64], _: usize, _: &SignalBus, dx: &mut [f64]) {
        dx[0] = 1.0;
    }
    fn n_events(&self) -> usize {
        1
    }
    fn event_functions(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, g: &mut [f64]) {
        g[0] = x[o] - self.level;
    }
    fn on_event(&mut self, _: usize, _: bool, t: f64, _: &mut [f64], _: &mut SignalBus) {
        self.hits.lock().unwrap().push((self.name.to_string(), t));
    }
}

type Hits = Arc<Mutex<Vec<(String, f64)>>>;

fn ramps(levels: &[(&'static str, f64)], dt_max: f64) -> (Engine, Hits) {
    let hits = Arc::new(Mutex::new(Vec::new()));
    let mut p = Plant::new();
    for &(name, level) in levels {
        p.add(Box::new(Ramp {
            name,
            level,
            hits: Arc::clone(&hits),
        }));
    }
    (Engine::new(p, SignalBus::new(), vec![], dt_max), hits)
}

#[test]
fn simultaneous_crossings_in_two_modules_both_fire_once() {
    let (mut e, hits) = ramps(&[("a", 0.3337), ("b", 0.3337)], 1e-2);
    e.step_until(SimTime::from_secs_f64(1.0)).unwrap();
    let h = hits.lock().unwrap();
    assert_eq!(h.len(), 2, "{h:?}");
    assert!(h.iter().all(|(_, t)| (t - 0.3337).abs() <= 1e-9));
}

#[test]
fn crossing_exactly_at_substep_end_and_interval_end_fires_once() {
    // dt_max 0.01 → a substep ends at 0.25; the target 0.5 lands exactly on the second level.
    let (mut e, hits) = ramps(&[("sub", 0.25), ("end", 0.5)], 1e-2);
    e.step_until(SimTime::from_secs_f64(0.5)).unwrap();
    e.step_until(SimTime::from_secs_f64(0.7)).unwrap();
    let h = hits.lock().unwrap();
    let count = |n: &str| h.iter().filter(|(m, _)| m == n).count();
    assert_eq!((count("sub"), count("end")), (1, 1), "{h:?}");
}
