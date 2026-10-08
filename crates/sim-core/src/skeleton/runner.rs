//! Skeleton simulation runner (P01.T03) — temporary, replaced in P03.T08.
//!
//! A dedicated OS thread owns the motor + controller. Commands arrive over a
//! channel; the latest state is published to a shared cell at most every 16 ms.
//! Sim time is paced to wall-clock × time_scale. While paused the thread blocks
//! on the channel (no busy-wait).

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use super::foc::{Foc, FocConfig, FocGains};
use super::model::{Pmsm, PmsmParams};

/// Plant integration step [s].
const PLANT_DT: f64 = 5e-6;
/// How often the published snapshot is refreshed while running.
const PUBLISH_EVERY: Duration = Duration::from_millis(16);
/// Wall-time budget of one pacing chunk.
const CHUNK: Duration = Duration::from_millis(1);

/// Commands accepted by the runner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Command {
    Play,
    Pause,
    Reset,
    /// Mechanical speed setpoint [rad/s].
    SetTargetSpeed(f64),
    /// Sim seconds per wall second; `f64::INFINITY` = as fast as possible.
    SetTimeScale(f64),
    Shutdown,
}

/// Published state. Serialized with the dotted signal paths of CONVENTIONS §3
/// (e.g. `motor.omega`), which is the wire format for REST, WS and MCP.
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize)]
pub struct StateSnapshot {
    /// Sim time [s].
    #[serde(rename = "sim.t")]
    pub t: f64,
    /// Mechanical speed [rad/s].
    #[serde(rename = "motor.omega")]
    pub omega: f64,
    /// Mechanical angle [rad].
    #[serde(rename = "motor.theta")]
    pub theta: f64,
    /// d/q currents [A].
    #[serde(rename = "motor.i_d")]
    pub i_d: f64,
    #[serde(rename = "motor.i_q")]
    pub i_q: f64,
    /// Phase currents [A].
    #[serde(rename = "motor.i_a")]
    pub i_a: f64,
    #[serde(rename = "motor.i_b")]
    pub i_b: f64,
    #[serde(rename = "motor.i_c")]
    pub i_c: f64,
    /// Speed setpoint [rad/s].
    #[serde(rename = "ctrl.omega_ref")]
    pub omega_ref: f64,
    #[serde(rename = "sim.running")]
    pub running: bool,
    #[serde(rename = "sim.time_scale")]
    pub time_scale: f64,
    /// Achieved sim-time / wall-time ratio (EWMA).
    #[serde(rename = "sim.real_ratio")]
    pub sim_real_ratio: f64,
}

/// A command plus an optional channel on which the runner replies with the state
/// right after applying it (so callers never return stale state).
struct Envelope {
    cmd: Command,
    reply: Option<Sender<StateSnapshot>>,
}

/// Handle to a running simulation thread.
pub struct SimHandle {
    tx: Sender<Envelope>,
    latest: Arc<Mutex<StateSnapshot>>,
    thread: Option<JoinHandle<()>>,
}

impl SimHandle {
    /// Spawn the runner thread with the skeleton motor and controller.
    pub fn spawn() -> std::io::Result<Self> {
        let (tx, rx) = mpsc::channel();
        let latest = Arc::new(Mutex::new(StateSnapshot {
            time_scale: 1.0,
            ..Default::default()
        }));
        let shared = Arc::clone(&latest);
        let thread = std::thread::Builder::new()
            .name("sim-runner".into())
            .spawn(move || Runner::new(shared).run(rx))?;
        Ok(Self {
            tx,
            latest,
            thread: Some(thread),
        })
    }

    /// Send a command without waiting (ignored if the thread has exited).
    pub fn send(&self, cmd: Command) {
        let _ = self.tx.send(Envelope { cmd, reply: None });
    }

    /// Send a command and wait (≤ 1 s) for the state right after it was applied.
    /// Blocking: from async code call it via `tokio::task::spawn_blocking`.
    pub fn request(&self, cmd: Command) -> Option<StateSnapshot> {
        let (tx, rx) = mpsc::channel();
        self.tx
            .send(Envelope {
                cmd,
                reply: Some(tx),
            })
            .ok()?;
        rx.recv_timeout(Duration::from_secs(1)).ok()
    }

    /// Latest published state.
    pub fn state(&self) -> StateSnapshot {
        // A poisoned lock only means a panic elsewhere; the data is still a valid snapshot.
        *self.latest.lock().unwrap_or_else(|e| e.into_inner())
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.send(Command::Shutdown);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

struct Runner {
    motor: Pmsm,
    foc: Foc,
    t: f64,
    running: bool,
    time_scale: f64,
    ratio: f64,
    shared: Arc<Mutex<StateSnapshot>>,
}

impl Runner {
    fn new(shared: Arc<Mutex<StateSnapshot>>) -> Self {
        let mp = PmsmParams::skeleton_6020();
        let cfg = FocConfig::skeleton();
        Self {
            motor: Pmsm::new(mp),
            foc: Foc::new(cfg, FocGains::design(&mp, &cfg)),
            t: 0.0,
            running: false,
            time_scale: 1.0,
            ratio: 0.0,
            shared,
        }
    }

    /// Apply a command, publish and reply; returns false on shutdown.
    fn apply(&mut self, env: Envelope) -> bool {
        let keep_running = self.apply_cmd(env.cmd);
        if let Some(reply) = env.reply {
            let _ = reply.send(self.snapshot());
        }
        keep_running
    }

    fn apply_cmd(&mut self, cmd: Command) -> bool {
        match cmd {
            Command::Play => self.running = true,
            Command::Pause => self.running = false,
            Command::Reset => {
                self.motor.state = Default::default();
                self.foc.reset();
                self.t = 0.0;
            }
            Command::SetTargetSpeed(w) => self.foc.omega_ref = w,
            Command::SetTimeScale(s) if s > 0.0 => self.time_scale = s,
            Command::SetTimeScale(_) => {}
            Command::Shutdown => return false,
        }
        self.publish();
        true
    }

    /// One controller period: controller update, then plant substeps (ZOH).
    fn control_step(&mut self) {
        let s = self.motor.state;
        let (vd, vq) = self.foc.update(s.id, s.iq, s.omega);
        let n = (self.foc.cfg.dt / PLANT_DT).round() as usize;
        for _ in 0..n {
            self.motor.step(vd, vq, PLANT_DT);
        }
        self.t += self.foc.cfg.dt;
    }

    fn publish(&self) {
        *self.shared.lock().unwrap_or_else(|e| e.into_inner()) = self.snapshot();
    }

    fn snapshot(&self) -> StateSnapshot {
        let s = self.motor.state;
        let theta_e = self.motor.params.p * s.theta;
        let (sin, cos) = theta_e.sin_cos();
        // Inverse Park, then inverse Clarke (amplitude-invariant).
        let i_alpha = s.id * cos - s.iq * sin;
        let i_beta = s.id * sin + s.iq * cos;
        let k = 3f64.sqrt() / 2.0;
        StateSnapshot {
            t: self.t,
            omega: s.omega,
            theta: s.theta,
            i_d: s.id,
            i_q: s.iq,
            i_a: i_alpha,
            i_b: -0.5 * i_alpha + k * i_beta,
            i_c: -0.5 * i_alpha - k * i_beta,
            omega_ref: self.foc.omega_ref,
            running: self.running,
            time_scale: self.time_scale,
            sim_real_ratio: self.ratio,
        }
    }

    fn run(mut self, rx: Receiver<Envelope>) {
        self.publish();
        loop {
            if !self.running {
                // Paused: block until a command arrives (0 % CPU).
                match rx.recv() {
                    Ok(env) => {
                        if !self.apply(env) {
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

    /// Run while playing, pacing sim time to wall time × scale.
    /// Returns `false` on shutdown, `true` when paused or when pacing must restart.
    fn run_paced(&mut self, rx: &Receiver<Envelope>) -> bool {
        let wall0 = Instant::now();
        let sim0 = self.t;
        let scale = self.time_scale;
        let mut last_publish = Instant::now();
        let mut last_ratio = (Instant::now(), self.t);
        while self.running {
            let chunk_start = Instant::now();
            let target = sim0 + wall0.elapsed().as_secs_f64() * scale;
            // Step until caught up, or until the chunk budget is used (then we are lagging).
            while self.t < target && chunk_start.elapsed() < CHUNK {
                self.control_step();
            }
            if last_ratio.0.elapsed() >= Duration::from_millis(100) {
                let r = (self.t - last_ratio.1) / last_ratio.0.elapsed().as_secs_f64();
                self.ratio = if self.ratio == 0.0 {
                    r
                } else {
                    0.7 * self.ratio + 0.3 * r
                };
                last_ratio = (Instant::now(), self.t);
            }
            if last_publish.elapsed() >= PUBLISH_EVERY {
                self.publish();
                last_publish = Instant::now();
            }
            // Sleep only when ahead of wall time; always check for commands.
            let wait = if self.t >= target {
                CHUNK
            } else {
                Duration::ZERO
            };
            match rx.recv_timeout(wait) {
                Ok(env) => {
                    let cmd = env.cmd;
                    if !self.apply(env) {
                        return false;
                    }
                    if matches!(cmd, Command::SetTimeScale(_) | Command::Reset) {
                        return true; // restart pacing from the new scale / time origin
                    }
                }
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => return false,
            }
        }
        self.publish();
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wait(ms: u64) {
        std::thread::sleep(Duration::from_millis(ms));
    }

    #[test]
    fn play_pace_pause_reset() {
        let h = SimHandle::spawn().expect("spawn runner");
        h.request(Command::SetTimeScale(1.0));
        h.request(Command::SetTargetSpeed(10.0));
        let start = Instant::now();
        h.request(Command::Play);
        wait(200);
        let s = h.request(Command::Pause).expect("reply");
        let wall = start.elapsed().as_secs_f64();
        assert!(!s.running);
        // Sim time follows the *measured* wall time × scale within ±10 % (P01.T03 criterion).
        assert!(
            (s.t - wall).abs() <= 0.1 * wall,
            "sim t = {}, wall = {wall}",
            s.t
        );
        assert!(
            s.omega > 1.0,
            "motor should be spinning, omega = {}",
            s.omega
        );

        // Paused: time does not advance.
        wait(100);
        assert_eq!(h.state().t, s.t);

        let r = h.request(Command::Reset).expect("reply");
        assert_eq!(r.t, 0.0);
        assert_eq!(r.omega, 0.0);
        assert_eq!(r.i_q, 0.0);
    }

    #[test]
    fn request_returns_state_after_the_command() {
        let h = SimHandle::spawn().expect("spawn runner");
        let s = h.request(Command::SetTargetSpeed(7.5)).expect("reply");
        assert_eq!(s.omega_ref, 7.5);
        let s = h.request(Command::Play).expect("reply");
        assert!(s.running);
    }
}
