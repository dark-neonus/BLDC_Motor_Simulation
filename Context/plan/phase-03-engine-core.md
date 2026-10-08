# Phase 03 — Simulation engine core

> **Goal:** the production engine that all physics and control blocks plug into: time base, signal bus, block and plant interfaces, integrators, multi-rate event scheduler, fidelity config, live commands, runner with time scaling, snapshots, recorder and RNG. Port the skeleton onto it.
> **Depends on:** P02.
> **Read first:** `docs/docs/physics/numerics.md` (EQ-NUM-*), [DECISIONS D-002, D-004, D-006](DECISIONS.md), [CONVENTIONS §6](CONVENTIONS.md).
> **Exit criteria:**
> - The skeleton motor + FOC runs on the new engine, and all P01 tests (Rust, Python, E2E) still pass.
> - RK4 shows 4th-order convergence, verified against diffsol.
> - Snapshot → restore → continue is bit-identical.
> - Criterion baseline recorded.
>
> **Design sketch (refine, don't contradict without a decision):**
> ```
> Engine { time: SimTime, plant: Plant (continuous, one state vector), blocks: Vec<Box<dyn DiscreteBlock>>,
>          events: BinaryHeap<(SimTime, priority, block_idx)>, bus: SignalBus, rng: RngService, cfg: FidelityConfig }
> step_until(t_target): loop { t_next = min(next_time_event, t_target); plant.integrate(t → t_next, dt_max)
>                                 (stops early at a located state event); fire all events at t_next in priority order; }
> plant derivative evaluation (every RK stage):
>   1. outputs pass: each PlantModule computes its algebraic outputs from the FULL state x into a stage bus,
>      in a fixed module order (mechanical → electrical → inverter/bus → thermal …), e.g. θe/ω for the motor, V_bus for the inverter;
>   2. derivatives pass: each module computes dx for its slice, reading the stage bus + the ZOH discrete inputs.
> ```
> Discrete inputs (switch states, duties, setpoints, external torques) are held constant between events (ZOH). **Continuous coupling between modules (θ/ω → motor, V_bus → inverter, T → R(T), the two backlash inertias) must go through the per-stage outputs pass, never through values stale since the last event.**

- [x] **P03.T01** — Core types: `SimTime`, `SignalId`, `SignalBus` + registry
  - **Depends:** P02
  - **Do:**
    - `SimTime(i64 ns)` with helpers (`from_secs_f64`, `as_secs_f64`, `period_from_hz`, which **rounds** to the nearest ns per D-010 and reports the actual frequency).
    - `SignalBus`: preallocated `Vec<f64>`. Registry `name → SignalId` with metadata `{path, unit, description, help_id, kind: state|input|output|diagnostic}`. Registration happens only at build time, and the bus is frozen before running.
  - **Files:** `crates/sim-core/src/engine/{time,signals}.rs`
  - **Verify:** Unit tests: 20 kHz → 50 000 ns exactly; 30 kHz → 33 333 ns with an actual-frequency report; duplicate registration error; lookup by path.
  - **Done when:** Tests pass. Paths follow CONVENTIONS §3.

- [x] **P03.T02** — Block and plant interfaces
  - **Depends:** P03.T01
  - **Do:**
    - Trait `DiscreteBlock { fn id(&self)->&str; fn period(&self)->SimTime; fn phase(&self)->SimTime; fn priority(&self)->u8; fn step(&mut self, ctx: &mut StepCtx) -> Result<(), SimError>; fn reset(&mut self); fn snapshot/restore }`. `StepCtx` gives bus read/write, time and RNG.
    - Trait `PlantModule` for continuous subsystems, each owning a slice of the global state vector: `fn n_states()`, `fn outputs(t, x_full, stage_bus)` (algebraic outputs from the full state, called every RK stage in fixed module order), `fn derivatives(t, x_full, stage_bus, dx_slice)`, `fn state_events(x_full, stage_bus) -> &[f64]` (zero-crossing functions, see T15), `fn on_state_event(idx, …)`, `fn power_terms() -> &[PowerTerm]` (see T16).
    - `Plant` composes modules into one state vector, with a state layout registry (names + units) for debugging.
    - Support for **variable-event blocks** (e.g. PWM edges, whose next fire time depends on duty): `fn next_event(&self, now) -> Option<SimTime>`.
  - **Files:** `crates/sim-core/src/engine/{block,plant}.rs`
  - **Verify:** Compile + toy tests: a counter block at 1 kHz plus an exponential-decay module; **two coupled modules** (a mass-spring split across two modules, the spring force passed via `outputs`) reproduce the single-module solution to 1e-12, proving per-stage coupling.
  - **Done when:** The interfaces are documented with rustdoc examples.

- [x] **P03.T03** — Integrators: RK4 and exponential/semi-implicit RL step
  - **Depends:** P03.T02
  - **Do:**
    - RK4 over the plant state vector (no allocation per step: scratch buffers in the integrator).
    - An exponential integrator option for linear RL substates as specified in EQ-NUM.
    - `integrate(plant, x, t0, t1, dt_max)` takes `ceil((t1−t0)/dt_max)` equal substeps.
  - **Files:** `crates/sim-core/src/engine/integrate.rs`
  - **Verify:** Tests:
    - exponential decay exact comparison
    - harmonic oscillator energy drift bound over 1e4 periods
    - **convergence order**: error ratio when halving dt ≈ 16 (±10 %) for RK4
    - RL exponential step exact for constant input
  - **Done when:** All pass.

- [x] **P03.T04** — diffsol cross-check harness (dev-dependency)
  - **Depends:** P03.T03
  - **Do:** Add `diffsol` as a dev-dependency (or a test-only crate `crates/sim-core-xcheck` if it pulls heavy deps). Solve the same test ODEs (decay, oscillator, a nonlinear Van der Pol, and the skeleton PMSM open-loop) with diffsol at tolerance 1e-10 and compare with RK4 at a fine dt.
  - **Files:** `crates/sim-core/tests/xcheck_diffsol.rs`
  - **Verify:** `cargo nextest run -p sim-core xcheck` — max relative error < 1e-6.
  - **Done when:** Passing. If diffsol's build is heavy, gate it behind the feature `xcheck`, run it in `just check` (not `check-fast`), and record the decision.
  - **If stuck:** Check docs.rs for the locked diffsol version. Its API differs between 0.x versions (builder vs problem structs).

- [x] **P03.T05** — Multi-rate event scheduler
  - **Depends:** P03.T03
  - **Do:**
    - Implement `Engine::step_until`. Fixed-period blocks are scheduled with integer periods + phase; variable-event blocks are polled for `next_event`.
    - Coincident events are ordered by priority (D-004), then by registration order.
    - The plant integrates exactly to each event time.
    - Expose `step_events(n)` and `step_time(dt)` for single-stepping.
  - **Files:** `crates/sim-core/src/engine/{engine,scheduler}.rs`
  - **Verify:** Tests:
    - 20 kHz + 5 kHz + 1 kHz blocks fire the exact counts over 1 s (20000/5000/1000)
    - coincident-event order is deterministic
    - a variable-event block firing at irregular times is honored exactly
  - **Done when:** Tests pass.

- [x] **P03.T06** — Fidelity configuration
  - **Depends:** P03.T05
  - **Do:** `FidelityConfig { tier: Ideal|Standard|Detailed, dt_max, inverter_mode: Averaged|Switching, enable_cogging, enable_iron_loss, enable_saturation, enable_thermal, enable_sensor_nonideal, … }` with tier presets that the user can override per flag. Default `dt_max` rules from EQ-NUM (e.g. min(τ_e/20, T_pwm/200) in switching mode). Expose the resulting values as signals `sim.dt_max`, `sim.tier`.
  - **Files:** `crates/sim-core/src/engine/fidelity.rs`
  - **Verify:** Unit tests for the dt rules.
  - **Done when:** Presets match POLISHED_IDEA §3.2.

- [x] **P03.T07** — Command queue and live parameter changes
  - **Depends:** P03.T05
  - **Do:**
    - `Command` enum: SetParam(path, value), SetTarget, Fault on/off, Load scene, Play/Pause/Step/Reset, TimeScale, Snapshot save/restore, Subscribe/Unsubscribe signals.
    - **Interim param registry** (until P04.T13 routes edits through `apply_edit`): modules register their live-settable params as `{path, unit, getter, setter}` at build time; `SetParam` looks them up there.
    - Applied **only between events** (never mid-integration).
    - A param-change hook per module (e.g. load mass change applies D-007).
    - Each applied change produces an `EngineEvent::ParamChanged {path, old, new, source}` for broadcast.
  - **Files:** `crates/sim-core/src/engine/commands.rs`
  - **Verify:** Test: changing a param mid-run takes effect at the next event boundary and emits exactly one event.
  - **Done when:** Passing.

- [x] **P03.T08** — Runner: pacing, time scale, step, sim/real ratio
  - **Depends:** P03.T07
  - **Do:**
    - Replace the skeleton runner. A dedicated thread runs chunks; time scale ranges from 1e-4 (slow motion) to `max` (unpaced).
    - Measure `sim.real_ratio` (EWMA). Overrun means the runner can't keep up: set the `sim.lagging` flag, never block the commands channel.
    - Paused: block on the command channel (0 % CPU).
    - Publish state via a lock-free latest-value cell plus a bounded broadcast for events.
  - **Files:** `crates/sim-core/src/engine/runner.rs`
  - **Verify:** Tests: scale 0.1 → sim ≈ 0.1× wall time; `max` → ratio ≫ 1; paused CPU ~0 (measure the loop iteration count while paused).
  - **Done when:** Passing.

- [x] **P03.T09** — Snapshots (save/restore full state)
  - **Depends:** P03.T08, P03.T11
  - **Do:** Serialize the engine state with serde + MessagePack: time, plant state, every block's internal state (`snapshot()/restore()`), RNG states, event queue, fidelity config and scene parameters. Snapshots are kept in memory (named) and can be exported to a file.
  - **Files:** `crates/sim-core/src/engine/snapshot.rs`
  - **Verify:** Test: run 0.1 s (with a noisy block using the RNG) → snapshot → run 0.1 s (A); restore → run 0.1 s (B); A == B bit-identical.
  - **Done when:** Passing.

- [x] **P03.T10** — Recorder & stream decimation
  - **Depends:** P03.T05
  - **Do:**
    - Per-subscription ring buffers.
    - **Full-rate capture** mode for export: every plant step or every event, selectable, with a memory cap and a warning.
    - **Stream** mode: min/max decimation per time bucket so plots show spikes and ripple even when decimated.
    - Output frames `{t0, dt_bucket, signals: {path: [min..], [max..]}}`.
  - **Files:** `crates/sim-core/src/engine/recorder.rs`
  - **Verify:** Test: a 20 kHz square ripple decimated to 100 buckets/s keeps the correct min/max envelope.
  - **Done when:** Passing.

- [x] **P03.T11** — Deterministic RNG service
  - **Depends:** P03.T02
  - **Do:** `RngService` with a scene seed. Each block gets a `ChaCha8Rng` seeded by `hash(scene_seed, block_id)`. Normal distribution via `rand_distr`. RNG state is included in snapshots.
  - **Files:** `crates/sim-core/src/engine/rng.rs`
  - **Verify:** Test: the same seed gives the same sequences; different blocks give independent streams; RNG state serialize → deserialize continues identically. (Full-engine snapshot continuity is tested in T09.)
  - **Done when:** Passing.

- [ ] **P03.T12** — Port the skeleton onto the engine
  - **Depends:** P03.T06, P03.T08, P03.T09, P03.T10, P03.T11
  - **Do:** Re-express the skeleton dq motor as a `PlantModule` and the FOC as `DiscreteBlock`s on the bus. Switch the CLI and server to the new engine. Delete `skeleton/runner.rs`. **Keep** the skeleton dq model as a test fixture under `crates/sim-core/src/fixtures/dq_pmsm.rs` (`#[cfg(any(test, feature = "fixtures"))]`), because P05.T03 needs it for the abc ≡ dq equivalence test. Keep the skeleton FOC block (it becomes the interim drive in P05.T11).
  - **Verify:** `just test && just validate && just e2e` — all P01 tests green.
  - **Done when:** Green, and no code path uses the old runner.

- [ ] **P03.T13** — Criterion benchmarks
  - **Depends:** P03.T12
  - **Do:** Benchmarks: plant steps/s (skeleton motor), events/s with 3 rates, and full engine sim/real ratio at dt = 5 µs. Add `just bench`.
  - **Files:** `crates/sim-core/benches/engine.rs`
  - **Verify:** `just bench` runs. Record the numbers in LOG.
  - **Done when:** The baseline is recorded.

- [x] **P03.T15** — State events (zero-crossing location)
  - **Depends:** P03.T05
  - **Do:** Implement EQ-NUM state events. After each integration substep, evaluate every module's `state_events()` functions. On a sign change, locate the crossing by bisection (or Illinois/regula falsi) on the dense RK4 interpolant or by re-integration, to a time tolerance (default 1 ns, as in EQ-NUM). Integrate exactly to the crossing, call `on_state_event`, then continue. Guard against Zeno chatter (max events per µs → diagnostic event). Needed by diode conduction/floating phases (P05.T07, P07.T01), Karnopp stick/slip (P06.T03), backlash contact (P06.T02), the chopper hysteresis (P07.T09) and PSU CV/CC switching (P07.T07).
  - **Files:** `crates/sim-core/src/engine/state_events.rs`
  - **Verify:** Tests:
    - a bouncing ball (restitution) has its impact times located to ≤ 1 ns vs the analytic solution over 10 bounces
    - a relay/hysteresis thermostat switches at the exact thresholds
    - a Zeno case triggers the guard, not a hang
  - **Done when:** Passing.

- [~] **P03.T16** — Energy accounting framework (accumulator + residual)
  - **Depends:** P03.T02, P03.T05
  - **Do:** Implement the engine side of EQ-ENER now, so every physics phase can test conservation:
    - Each module reports `PowerTerm {path, kind: Input|Output|Loss|External, watts}` and `stored_energy()`.
    - An `EnergyAccumulator` plant module integrates all power terms as **extra states** (same integrator → consistent accuracy).
    - Signals `energy.in`, `energy.out`, `energy.loss.<module>`, `energy.stored`, `energy.external`, `energy.residual` (normalized per EQ-ENER), `energy.ok`.
    - Discrete energy injections (live param changes per D-007, disturbance pulses) are recorded as `External`.
  - **Files:** `crates/sim-core/src/energy.rs`
  - **Verify:** Tests:
    - a lossless oscillator split across two modules → residual < 1e-9
    - a damped one → loss integral = initial energy − final within 1e-9
  - **Done when:** Passing. P05–P09 use `energy.residual` in their tests; P10.T04 adds the engine-wide property test.

- [ ] **P03.T14** — Phase gate
  - **Depends:** P03.T01, P03.T02, P03.T03, P03.T04, P03.T05, P03.T06, P03.T07, P03.T08, P03.T09, P03.T10, P03.T11, P03.T12, P03.T13, P03.T15, P03.T16
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-03-done`.
