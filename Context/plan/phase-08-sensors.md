# Phase 08 — Sensors & estimators

> **Goal:** all sensor models as discrete blocks with a shared non-ideality pipeline (error → quantization → noise → latency → sample-and-hold), plus velocity estimation and the sensorless flux observer.
> **Depends on:** P07.
> **Read first:** `docs/docs/physics/sensors.md` (EQ-SENS-*), `control.md` (estimators), [CONVENTIONS §6 (RNG)](CONVENTIONS.md).
> **Exit criteria:** All V-SENS cases pass. Every sensor can be bypassed (ideal) and has update rate, latency, noise and resolution parameters. The observer converges above its minimum speed and fails below it, as documented.

- [ ] **P08.T01** — Sensor pipeline framework
  - **Depends:** P07
  - **Do:**
    - A generic `SensorPipeline` (composable stages: deterministic error fn, quantizer, seeded Gaussian noise, delay line in update periods or ns, sample-and-hold).
    - `bypass: true` → outputs the true value with no delay.
    - Each sensor block has a configurable rate (fixed period) or is triggered by an event (e.g. `pwm.center`).
  - **Files:** `crates/sim-core/src/physics/sensors/pipeline.rs`
  - **Verify:** Tests:
    - quantization step correct
    - delay exactly N samples
    - noise std within statistical bounds (seeded, 1e4 samples, ±5 %)
    - bypass is exact
  - **Done when:** Passing.

- [ ] **P08.T02** — Hall sensors
  - **Depends:** P08.T01
  - **Do:** 3 digital outputs from θe with per-sensor placement offset, optional hysteresis, and a sector output `sensors.hall.sector` (1–6).
  - **Files:** `crates/sim-core/src/physics/sensors/hall.rs`
  - **Verify:** Test: transitions every 60° electrical at the correct angles; offsets shift them.
  - **Done when:** Passing.

- [ ] **P08.T03** — On-axis magnetic encoder (+ chip presets)
  - **Depends:** P08.T01
  - **Do:** Per EQ-SENS: bits, INL harmonics, eccentricity first-harmonic error, air-gap → noise approximation, update rate and latency. Presets AS5600/AS5047P/MT6701 load from `presets/sensors/`. Signals `sensors.encoder.angle_raw`, `sensors.encoder.angle` (rad), `sensors.encoder.error` (true − measured, diagnostic).
  - **Files:** `crates/sim-core/src/physics/sensors/encoder_magnetic.rs`
  - **Verify:** Tests:
    - 14-bit resolution step = 2π/16384
    - eccentricity error amplitude and period
    - the air-gap outside the recommended range increases noise as specified
  - **Done when:** Passing.

- [ ] **P08.T04** — Incremental optical encoder
  - **Depends:** P08.T01
  - **Do:** CPR, ×4 quadrature counting, index pulse, count → angle (relative until index). Optional missed-count fault hook for P10.
  - **Files:** `crates/sim-core/src/physics/sensors/encoder_incremental.rs`
  - **Verify:** Test: 1000 CPR → 4000 counts/rev, sign by direction, index once per rev.
  - **Done when:** Passing.

- [ ] **P08.T05** — Current & voltage ADC
  - **Depends:** P08.T01
  - **Do:** Phase current shunts (2 or 3 configurable) and bus voltage. Bits, range, gain/offset errors, noise and saturation. Sampled on the `pwm.center` event (switching mode) or at the controller rate (averaged mode).
  - **Files:** `crates/sim-core/src/physics/sensors/adc.rs`
  - **Verify:** Tests:
    - in switching mode, center sampling gives the average current (ripple midpoint) within tolerance
    - saturation clamps
    - an offset error produces the expected dq ripple at ωe (diagnostic test)
  - **Done when:** Passing.

- [ ] **P08.T06** — Velocity estimation (difference + LPF, and PLL)
  - **Depends:** P08.T03, P08.T04
  - **Do:** Estimator blocks taking an angle signal → `est.omega`, `est.theta`. Options: finite difference + 1st-order LPF, or a PLL (Kp, Ki) per EQ-CTRL.
  - **Files:** `crates/sim-core/src/control/estimators/velocity.rs`
  - **Verify:** Tests:
    - a constant-speed ramp is tracked with the expected lag
    - PLL step response bandwidth matches the design formula
  - **Done when:** Passing.

- [ ] **P08.T07** — Sensorless nonlinear flux observer + PLL
  - **Depends:** P08.T05, P08.T06
  - **Do:** Implement the [Lee2010] observer per EQ-CTRL using the measured currents and the commanded voltages. Gain γ. A PLL for speed. Outputs `est.theta`, `est.omega`, `est.valid` (a validity heuristic: speed above a threshold and flux magnitude near λ).
  - **Files:** `crates/sim-core/src/control/estimators/flux_observer.rs`
  - **Verify:** Tests:
    - at ≥ 10 % of rated speed, the angle error converges below 5° electrical within the specified time
    - at very low speed, the error grows and `est.valid = false` (documented failure)
  - **Done when:** Passing.

- [ ] **P08.T08** — V-SENS sweep
  - **Depends:** P08.T02, P08.T03, P08.T04, P08.T05, P08.T07
  - **Do:** Implement all V-SENS catalog cases as tests.
  - **Verify:** `cargo nextest run -p sim-core v_sens`
  - **Done when:** Green.

- [ ] **P08.T09** — Phase gate
  - **Depends:** P08.T01, P08.T02, P08.T03, P08.T04, P08.T05, P08.T06, P08.T07, P08.T08
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-08-done`.
