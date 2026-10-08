# Phase 09 — Control system

> **Goal:**
> - The four controller families with per-loop rates, enable/bypass and limits: six-step + halls, open-loop sinusoidal, FOC cascaded PI (sensored and sensorless) and MIT/impedance.
> - Encoder offset calibration.
> - The sandboxed Luau custom controller block.
> - Auto-tune.
> - Controller presets (soft/medium/stiff).
>
> **Depends on:** P08.
> **Read first:** `docs/docs/physics/control.md` (EQ-CTRL-*), POLISHED_IDEA §5.
> **Exit criteria:**
> - All V-CTRL cases pass.
> - Each family holds an arm at 90° or tracks speed in a preset scene, as appropriate.
> - A Luau custom controller runs at 20 kHz in a release build at ≥ 1× real time in the averaged Standard tier.
> - Auto-tune on any motor preset produces a stable current loop within ±20 % of the target bandwidth.

- [ ] **P09.T01** — Controller block framework
  - **Depends:** P08
  - **Do:**
    - Common traits/utilities: `LoopConfig { enabled, rate_hz, limits }`, a setpoint source (`ctrl.setpoint.*` signals fed by the UI, scenarios or MCP), and ramp/rate limiters.
    - "Bypass" semantics for loops: a disabled outer loop passes its setpoint straight to the inner loop's reference.
    - A controller family is a set of blocks wired on the bus.
    - A `ControllerFactory` builds a family from `ControllerParams`.
  - **Files:** `crates/sim-core/src/control/{mod,framework,setpoint}.rs`
  - **Verify:** Test: disabling the position loop makes the velocity setpoint drive the velocity loop directly.
  - **Done when:** Passing.

- [ ] **P09.T02** — Discrete PI/PID with anti-windup
  - **Depends:** P09.T01
  - **Do:** Per EQ-CTRL: discretization method, clamping and back-calculation anti-windup, a filtered derivative, and output limits. A pure struct with no bus coupling.
  - **Files:** `crates/sim-core/src/control/pid.rs`
  - **Verify:** Tests:
    - step response of the PI on a first-order plant matches the analytic discrete solution
    - windup recovery is faster with anti-windup than without (quantified)
  - **Done when:** Passing.

- [ ] **P09.T03** — FOC current loop (sensored)
  - **Depends:** P09.T02
  - **Do:**
    - Measured currents (ADC) → Clarke/Park with the encoder angle (or `est.theta`).
    - PI on d and q, decoupling feed-forward, voltage-limit circle with d-priority, inverse Park → SVPWM duties → inverter.
    - Runs on the `pwm.center` trigger or at its configured rate.
    - **Replaces the interim skeleton FOC** (P05.T11/P07.T12): switch the default scene to it and delete the skeleton FOC block (`grep INTERIM` → none left after P09).
  - **Files:** `crates/sim-core/src/control/foc/current_loop.rs`
  - **Verify:** Tests:
    - closed-loop current bandwidth (V-CTRL) within ±15 % of design
    - i_d stays ≈ 0 under a speed change
    - voltage saturation is handled without windup
  - **Done when:** Passing.

- [ ] **P09.T04** — Velocity & position loops (cascade)
  - **Depends:** P09.T03
  - **Do:** Velocity PI → iq_ref (with the current limit); position P/PI → velocity ref (with the velocity limit). Each loop has its own rate, enable flag and ramp. Gear ratio aware (the position setpoint is on the load side when a gearbox is present).
  - **Files:** `crates/sim-core/src/control/foc/{velocity_loop,position_loop}.rs`
  - **Verify:** Tests:
    - speed step settling time per the design
    - position hold of the arm at 90° under gravity with a steady-state error explained by the loop type (P → error ∝ load; PI → 0)
  - **Done when:** Passing.

- [ ] **P09.T05** — Six-step with halls
  - **Depends:** P09.T02
  - **Do:** Hall sector → commutation table → six-step leg pattern (P07.T04). Duty from either direct duty or a speed PI. Optional advance angle.
  - **Files:** `crates/sim-core/src/control/six_step.rs`
  - **Verify:** Tests:
    - runs a trapezoidal motor to steady speed
    - torque ripple % within the V-CTRL expected band
    - the floating-phase current decays through the diodes
  - **Done when:** Passing.

- [ ] **P09.T06** — Open-loop sinusoidal (V/f)
  - **Depends:** P09.T02
  - **Do:** A rotating voltage vector with a commanded frequency ramp and voltage amplitude (V/f law or fixed V). No feedback.
  - **Files:** `crates/sim-core/src/control/open_loop.rs`
  - **Verify:** Tests:
    - at a low load it follows the commanded speed
    - with a high load step it **loses sync** (the rotor slips poles: detected via the angle error), as documented
  - **Done when:** Passing.

- [ ] **P09.T07** — MIT / impedance mode
  - **Depends:** P09.T03
  - **Do:** τ = Kp(θ*−θ) + Kd(ω*−ω) + τ_ff → iq_ref = τ/(Kt·N·η) per EQ-CTRL, running at its own rate on top of the current loop.
  - **Files:** `crates/sim-core/src/control/mit.rs`
  - **Verify:** Tests:
    - static deflection under a known load torque = τ_load/Kp (V-CTRL)
    - damping ratio from Kd matches the second-order formula
  - **Done when:** Passing.

- [ ] **P09.T08** — Sensorless FOC (startup + handoff)
  - **Depends:** P09.T04, P08.T07
  - **Do:** Startup sequence: align → open-loop ramp → handoff to the observer angle when `est.valid` holds for N ms, then back to open loop if it is lost. Uses `est.theta`/`est.omega` instead of the encoder.
  - **Files:** `crates/sim-core/src/control/foc/sensorless.rs`
  - **Verify:** Test: starts from standstill and reaches the speed target; the handoff is smooth (bounded current spike, specified).
  - **Done when:** Passing.

- [ ] **P09.T09** — Encoder offset calibration routine
  - **Depends:** P09.T03
  - **Do:** Align the rotor with a d-axis current and record the electrical offset; optionally check direction and pole pairs by slow rotation. Results are written to controller params. Also used to *demonstrate* the "wrong offset" fault in P10.
  - **Files:** `crates/sim-core/src/control/calibration.rs`
  - **Verify:** Test: a random true offset is recovered within 1° electrical.
  - **Done when:** Passing.

- [ ] **P09.T10** — Luau custom controller block
  - **Depends:** P09.T01
  - **Do:**
    1. `mlua` with features `luau` + `vendored`. `Lua::sandbox(true)`, memory limit and an interrupt-based instruction budget per call (exceeding it → block error, auto-pause).
    2. API: the user script defines `function init(params) return state end` and `function step(inputs, state, dt) return outputs end`.
       - `inputs`: a table of the subscribed signals declared in the block config.
       - `outputs`: a table of declared writable signals (e.g. `ctrl.v_d`, `ctrl.v_q` or `ctrl.duty_a..c` or `ctrl.iq_ref`).
       - Configured rate.
    3. Precompile once; reuse input/output tables (no allocation per call where possible).
    4. Errors show the Luau line number.
    5. State is snapshot-able if it contains only numbers, strings, booleans and tables (else the snapshot warns).
    6. Document the API in `docs/docs/control/custom-block.md`.
  - **Files:** `crates/sim-core/src/control/luau_block.rs`, docs page
  - **Verify:** Tests:
    - a Luau PI matching the Rust PI gives identical outputs (1e-12)
    - infinite loop → interrupted with an error, not a hang
    - memory bomb → error
    - benchmark at 20 kHz logged
  - **Done when:** Passing.
  - **If stuck:** If the mlua sandbox/interrupt API differs at the locked version, check its docs.rs page and `tests/luau.rs` in the mlua repo.

- [ ] **P09.T11** — Auto-tune
  - **Depends:** P09.T04, P09.T07
  - **Do:** Run in a **separate engine instance** (D-006) copied from the current scene:
    1. Measure R (DC voltage injection), L (voltage step response or HF injection), and J + friction (torque step / speed ramp), per EQ-CTRL.
    2. Compute gains: current loop (pole-zero cancellation, bandwidth = f_ctrl/10 by default, user-adjustable), velocity loop (from J, bandwidth separation ≥ 5×), position loop, and MIT Kp/Kd suggestions.
    3. Return the measured values + gains + predicted bandwidths. Apply only on user confirmation (`apply: true`).
  - **Files:** `crates/sim-core/src/control/autotune.rs`
  - **Verify:** Tests:
    - on each motor preset the measured R/L/J are within 5 % of the true params
    - the resulting current-loop bandwidth is within ±20 % of the target
    - all loops are stable (no oscillation growth over 1 s)
  - **Done when:** Passing.

- [ ] **P09.T12** — Controller presets & preset scenes
  - **Depends:** P09.T05, P09.T06, P09.T07, P09.T08, P09.T11
  - **Do:** `presets/controllers/`: six-step-{soft,medium,stiff}, open-loop-{slow,fast}, foc-{soft,medium,stiff} (sensored), foc-sensorless, mit-{soft,medium,stiff}. Gains are expressed **relative to the motor** (bandwidth targets → auto-computed at scene build), so they work on any motor preset. Update the example scenes.
  - **Files:** `presets/controllers/*.yaml`, `presets/scenes/*.yaml`
  - **Verify:** A test runs every controller preset × 3 motor presets for 1 s: no instability, no protection trips.
  - **Done when:** Passing.

- [ ] **P09.T13** — V-CTRL sweep
  - **Depends:** P09.T12, P09.T10, P09.T09
  - **Do:** Implement all V-CTRL catalog cases as tests.
  - **Verify:** `cargo nextest run -p sim-core v_ctrl`
  - **Done when:** Green.

- [ ] **P09.T14** — Phase gate
  - **Depends:** P09.T01, P09.T02, P09.T03, P09.T04, P09.T05, P09.T06, P09.T07, P09.T08, P09.T09, P09.T10, P09.T11, P09.T12, P09.T13
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-09-done`.
