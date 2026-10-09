//! Runner (P03.T08): owns an [`Engine`] on a dedicated thread, paces sim time to
//! wall time × time scale, applies commands between events, and publishes a status
//! snapshot plus engine events. Paused → blocks on the command channel (0 % CPU).

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, SyncSender, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde::Serialize;

use super::commands::{EngineCommand, EngineEvent};
use super::engine::Engine;
use super::time::SimTime;

const CHUNK: Duration = Duration::from_millis(1);
const PUBLISH_EVERY: Duration = Duration::from_millis(16);
/// Largest sim-time slice per `step_until` call (keeps commands responsive at max speed).
const MAX_SLICE: SimTime = SimTime(1_000_000); // 1 ms

/// Builds a fresh engine (used at start and on reset).
pub type EngineFactory = Box<dyn Fn() -> Engine + Send>;

#[derive(Debug, Clone, PartialEq)]
pub enum RunnerCommand {
    Play,
    Pause,
    Reset,
    /// Advance by this much sim time, then stay paused.
    StepTime(SimTime),
    /// Advance by `n` events, then stay paused.
    StepEvents(usize),
    /// Sim seconds per wall second; `f64::INFINITY` = as fast as possible.
    TimeScale(f64),
    Engine(EngineCommand),
    Shutdown,
}

/// Published status.
#[derive(Debug, Clone, Default, Serialize)]
pub struct RunnerStatus {
    pub t: f64,
    pub running: bool,
    pub time_scale: f64,
    /// Achieved sim/real ratio (EWMA).
    pub real_ratio: f64,
    /// True when the engine cannot keep up with the requested time scale.
    pub lagging: bool,
    /// Bus values in registration order (names via the engine's registry).
    pub values: Vec<f64>,
    /// Last error (numerical fault etc.); the runner pauses on error.
    pub error: Option<String>,
    /// Paced-loop iterations so far (diagnostic; must not grow while paused).
    pub iterations: u64,
}

struct Envelope {
    cmd: RunnerCommand,
    reply: Option<Sender<RunnerStatus>>,
}

pub struct RunnerHandle {
    tx: Sender<Envelope>,
    status: Arc<Mutex<RunnerStatus>>,
    thread: Option<JoinHandle<()>>,
    /// Signal metadata of the engine (paths, units), fixed after build.
    pub signals: Vec<super::signals::SignalMeta>,
}

impl RunnerHandle {
    /// Spawn the runner. `events` receives engine events (dropped when the buffer is full).
    pub fn spawn(
        factory: EngineFactory,
        events: Option<SyncSender<EngineEvent>>,
    ) -> std::io::Result<Self> {
        let engine = factory();
        let signals = engine.bus.all().to_vec();
        let status = Arc::new(Mutex::new(RunnerStatus {
            time_scale: 1.0,
            ..Default::default()
        }));
        let (tx, rx) = mpsc::channel();
        let shared = Arc::clone(&status);
        let thread = std::thread::Builder::new()
            .name("sim-engine".into())
            .spawn(move || {
                Runner {
                    engine,
                    factory,
                    running: false,
                    scale: 1.0,
                    ratio: 0.0,
                    lagging: false,
                    error: None,
                    iterations: 0,
                    shared,
                    events,
                }
                .run(rx)
            })?;
        Ok(Self {
            tx,
            status,
            thread: Some(thread),
            signals,
        })
    }

    pub fn send(&self, cmd: RunnerCommand) {
        let _ = self.tx.send(Envelope { cmd, reply: None });
    }

    /// Send and wait (≤ 1 s) for the status right after the command was applied.
    pub fn request(&self, cmd: RunnerCommand) -> Option<RunnerStatus> {
        let (tx, rx) = mpsc::channel();
        self.tx
            .send(Envelope {
                cmd,
                reply: Some(tx),
            })
            .ok()?;
        rx.recv_timeout(Duration::from_secs(1)).ok()
    }

    pub fn status(&self) -> RunnerStatus {
        self.status
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

impl Drop for RunnerHandle {
    fn drop(&mut self) {
        self.send(RunnerCommand::Shutdown);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct Runner {
    engine: Engine,
    factory: EngineFactory,
    running: bool,
    scale: f64,
    ratio: f64,
    lagging: bool,
    error: Option<String>,
    iterations: u64,
    shared: Arc<Mutex<RunnerStatus>>,
    events: Option<SyncSender<EngineEvent>>,
}

impl Runner {
    fn snapshot(&self) -> RunnerStatus {
        RunnerStatus {
            t: self.engine.time.as_secs_f64(),
            running: self.running,
            time_scale: self.scale,
            real_ratio: self.ratio,
            lagging: self.lagging,
            values: self.engine.bus.values().to_vec(),
            error: self.error.clone(),
            iterations: self.iterations,
        }
    }

    fn publish(&mut self) {
        if let Some(tx) = &self.events {
            for ev in self.engine.drain_events() {
                let _ = tx.try_send(ev); // never block the engine on a slow consumer
            }
        } else {
            self.engine.drain_events();
        }
        *self.shared.lock().unwrap_or_else(|e| e.into_inner()) = self.snapshot();
    }

    /// Run an engine call; on error pause and record it (never panic).
    fn guarded(&mut self, f: impl FnOnce(&mut Engine) -> Result<(), super::block::SimError>) {
        if let Err(e) = f(&mut self.engine) {
            self.error = Some(e.to_string());
            self.running = false;
        }
    }

    /// Apply one command; returns false on shutdown. Returns true if pacing must restart.
    fn apply(&mut self, env: Envelope) -> (bool, bool) {
        let mut restart = false;
        match env.cmd {
            RunnerCommand::Play => {
                self.running = self.error.is_none();
                restart = true;
            }
            RunnerCommand::Pause => self.running = false,
            RunnerCommand::Reset => {
                self.engine = (self.factory)();
                self.error = None;
                self.ratio = 0.0;
                self.lagging = false;
                restart = true;
            }
            RunnerCommand::StepTime(dt) => {
                self.running = false;
                self.guarded(|e| e.step_time(dt));
            }
            RunnerCommand::StepEvents(n) => {
                self.running = false;
                self.guarded(|e| e.step_events(n));
            }
            RunnerCommand::TimeScale(s) if s > 0.0 => {
                // Slow-motion floor 1e-4 (POLISHED_IDEA §6.1); ∞ = as fast as possible.
                self.scale = s.max(1e-4);
                restart = true;
            }
            RunnerCommand::TimeScale(_) => {}
            RunnerCommand::Engine(c) => {
                self.engine.queue(c);
                if !self.running {
                    self.engine.apply_pending();
                }
            }
            RunnerCommand::Shutdown => return (false, false),
        }
        self.publish();
        if let Some(r) = env.reply {
            let _ = r.send(self.snapshot());
        }
        (true, restart)
    }

    fn run(mut self, rx: Receiver<Envelope>) {
        self.publish();
        loop {
            if !self.running {
                match rx.recv() {
                    Ok(env) => {
                        if !self.apply(env).0 {
                            return;
                        }
                    }
                    Err(_) => return,
                }
            } else if !self.run_paced(&rx) {
                return;
            }
        }
    }

    /// Pace until paused or a restart is needed. Returns false on shutdown.
    fn run_paced(&mut self, rx: &Receiver<Envelope>) -> bool {
        let mut wall0 = Instant::now();
        let mut sim0 = self.engine.time;
        let mut last_pub = Instant::now();
        let mut last_ratio = (Instant::now(), self.engine.time);
        while self.running {
            let chunk = Instant::now();
            let target = if self.scale.is_finite() {
                sim0 + SimTime::from_secs_f64(wall0.elapsed().as_secs_f64() * self.scale)
            } else {
                SimTime(i64::MAX)
            };
            while self.running && self.engine.time < target && chunk.elapsed() < CHUNK {
                let to = (self.engine.time + MAX_SLICE).min(target);
                self.guarded(|e| e.step_until(to));
            }
            self.lagging = self.scale.is_finite() && self.engine.time < target;
            self.iterations += 1;
            if self.lagging {
                // Can't keep up: rebase pacing so we don't sprint to catch up later.
                wall0 = Instant::now();
                sim0 = self.engine.time;
            }
            if last_ratio.0.elapsed() >= Duration::from_millis(100) {
                let r = (self.engine.time - last_ratio.1).as_secs_f64()
                    / last_ratio.0.elapsed().as_secs_f64();
                self.ratio = if self.ratio == 0.0 {
                    r
                } else {
                    0.7 * self.ratio + 0.3 * r
                };
                last_ratio = (Instant::now(), self.engine.time);
            }
            if last_pub.elapsed() >= PUBLISH_EVERY {
                self.publish();
                last_pub = Instant::now();
            }
            let wait = if self.engine.time >= target {
                CHUNK
            } else {
                Duration::ZERO
            };
            match rx.recv_timeout(wait) {
                Ok(env) => {
                    let (alive, restart) = self.apply(env);
                    if !alive {
                        return false;
                    }
                    if restart {
                        return true;
                    }
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return false,
            }
            // Drain any further queued commands without waiting.
            loop {
                match rx.try_recv() {
                    Ok(env) => {
                        let (alive, restart) = self.apply(env);
                        if !alive {
                            return false;
                        }
                        if restart {
                            return true;
                        }
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return false,
                }
            }
        }
        self.publish();
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::plant::{Plant, PlantModule};
    use crate::engine::signals::SignalBus;

    struct Decay;
    impl PlantModule for Decay {
        fn name(&self) -> &str {
            "decay"
        }
        fn n_states(&self) -> usize {
            1
        }
        fn state_names(&self) -> Vec<(String, String)> {
            vec![("x".into(), "-".into())]
        }
        fn init(&self, x: &mut [f64]) {
            x[0] = 1.0;
        }
        fn outputs(&mut self, _: f64, _: &[f64], _: usize, _: &mut SignalBus) {}
        fn derivatives(&self, _: f64, x: &[f64], o: usize, _: &SignalBus, dx: &mut [f64]) {
            dx[0] = -x[o];
        }
    }

    fn factory() -> EngineFactory {
        Box::new(|| {
            let mut p = Plant::new();
            p.add(Box::new(Decay));
            Engine::new(p, SignalBus::new(), vec![], 1e-4)
        })
    }

    #[test]
    fn paced_slow_motion_follows_wall_time() {
        let h = RunnerHandle::spawn(factory(), None).unwrap();
        h.request(RunnerCommand::TimeScale(0.1));
        let start = Instant::now();
        h.request(RunnerCommand::Play);
        std::thread::sleep(Duration::from_millis(300));
        let s = h.request(RunnerCommand::Pause).unwrap();
        let wall = start.elapsed().as_secs_f64();
        assert!(
            (s.t - 0.1 * wall).abs() <= 0.1 * 0.1 * wall,
            "sim {} vs expected {}",
            s.t,
            0.1 * wall
        );
        // Paused: time frozen.
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(h.status().t, s.t);
    }

    #[test]
    fn max_speed_runs_faster_than_real_time() {
        let h = RunnerHandle::spawn(factory(), None).unwrap();
        h.request(RunnerCommand::TimeScale(f64::INFINITY));
        h.request(RunnerCommand::Play);
        std::thread::sleep(Duration::from_millis(400));
        let s = h.request(RunnerCommand::Pause).unwrap();
        assert!(
            s.t > 1.0,
            "sim time {} after 0.4 s wall (a 1-state plant should be ≫ real time)",
            s.t
        );
    }

    #[test]
    fn paused_runner_does_not_spin() {
        let h = RunnerHandle::spawn(factory(), None).unwrap();
        h.request(RunnerCommand::Play);
        std::thread::sleep(Duration::from_millis(50));
        let a = h.request(RunnerCommand::Pause).unwrap().iterations;
        std::thread::sleep(Duration::from_millis(150));
        assert_eq!(h.status().iterations, a, "paced loop iterated while paused");
    }

    #[test]
    fn step_and_reset() {
        let h = RunnerHandle::spawn(factory(), None).unwrap();
        let s = h
            .request(RunnerCommand::StepTime(SimTime::from_secs_f64(0.5)))
            .unwrap();
        assert!((s.t - 0.5).abs() < 1e-12 && !s.running);
        let s = h.request(RunnerCommand::Reset).unwrap();
        assert_eq!(s.t, 0.0);
    }
}
