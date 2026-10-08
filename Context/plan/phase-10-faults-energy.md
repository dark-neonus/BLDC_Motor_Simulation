# Phase 10 — Faults, protection, energy balance & warnings

> **Goal:** live fault injection, configurable protection logic, an engine-wide energy-balance residual (integrated as states), the NaN guard, and a warnings/event system. After this phase the **physics core is feature-complete for v1**.
> **Depends on:** P09.
> **Read first:** POLISHED_IDEA §5.4, §10; `docs/docs/physics/energy.md` (EQ-ENER-*).
> **Exit criteria:**
> - Every fault in POLISHED_IDEA §5.4 can be toggled live and in scenarios.
> - Protections trip and recover as configured.
> - `energy.residual` stays below the tolerance over a randomized scenario property test.
> - NaN anywhere → auto-pause with a diagnostic event, not a crash.

- [ ] **P10.T01** — Fault injection framework
  - **Depends:** P09
  - **Do:** A `FaultId` registry with params and on/off state. Faults are toggled by `Command::Fault`, by a scenario action or by the API. Each fault has metadata (title, description, help id) for the UI. Signals `fault.<id>.active`.
  - **Files:** `crates/sim-core/src/faults/mod.rs`
  - **Verify:** Test: toggling emits events and changes state at the next event boundary.
  - **Done when:** Passing.

- [ ] **P10.T02** — Implement the faults
  - **Depends:** P10.T01
  - **Do:**
    - `phase_open.{a,b,c}` (terminal disconnect → P05.T07)
    - `encoder.dropout` (holds the last value / outputs a fixed error code)
    - `encoder.glitch` (random spikes, seeded)
    - `ctrl.wrong_pole_pairs` (the controller uses a wrong p)
    - `ctrl.wrong_encoder_offset` (adds a configurable offset)
    - `mech.stall` (locks the rotor: infinite friction / constraint)
    - `supply.current_limit` (temporarily lowers the PSU I_lim)
    - `sensors.noise_burst` (multiplies sensor noise for a duration)
  - **Files:** `crates/sim-core/src/faults/*.rs`
  - **Verify:** A test per fault with its expected observable effect (e.g. wrong offset → reduced torque per amp ∝ cos(offset), with the formula in the test comment).
  - **Done when:** Passing.

- [ ] **P10.T03** — Protection logic
  - **Depends:** P10.T01
  - **Do:**
    - Protections OC (phase current), OV, UV (bus), OT (winding), each with a threshold, filter time, action (disable PWM → all legs Off; or brake → low-side on) and latch vs auto-retry.
    - Configurable and disableable.
    - Signals `protect.<id>.tripped`; events on trip/clear.
  - **Files:** `crates/sim-core/src/protection.rs`
  - **Verify:** Tests:
    - OC trips within filter time + 1 control period
    - OV trips during regen without a chopper
    - latch requires a reset
    - disabled protections never trip
  - **Done when:** Passing.

- [ ] **P10.T04** — Engine-wide energy-balance audit & property test
  - **Depends:** P10.T01
  - **Do:** The accumulator exists since P03.T16. Now audit that **every** module from P05–P10 (incl. faults, protections, chopper, Luau-driven control) reports all its power terms and stored energies (the checklist from EQ-ENER), and that fault/protection transitions don't leak energy. Then add the property test below.
  - **Files:** `crates/sim-core/tests/energy_property.rs`
  - **Verify:** Property test (proptest, ~200 cases): random preset motor × controller × load × supply, random setpoint sequence for 1 s → |residual| < tolerance. On failure, proptest shrinks to a minimal case: log it and fix the leaking block.
  - **Done when:** Passing in Ideal and Standard tiers. In Detailed/switching mode, the tolerance follows EQ-ENER (documented).

- [ ] **P10.T05** — NaN/Inf guard & diagnostic auto-pause
  - **Depends:** P10.T04
  - **Do:** After each event, check the state vector and bus for non-finite values (cheap check). On detection: roll back to the last good state (kept from the previous event), pause, and emit `SimEvent::NumericalFault {time, signals, suspects, hint}` with hints (dt too large? param units?).
  - **Files:** `crates/sim-core/src/engine/guard.rs`
  - **Verify:** Test: an absurd param (L = 1e-12 H with a large dt) → pause + event, no panic.
  - **Done when:** Passing.

- [ ] **P10.T06** — Warnings system
  - **Depends:** P10.T03
  - **Do:** Non-fatal warnings with dedup/rate limiting:
    - over rated voltage / current / temperature
    - the thermal continuous limit exceeded for > X s
    - the encoder air gap outside the recommended range
    - sim lagging real time
    - observer invalid while in use

    Each warning: id, severity, message (beginner-friendly), help id. Exposed as an event stream + `warnings.active` list.
  - **Files:** `crates/sim-core/src/warnings.rs`
  - **Verify:** Tests per warning.
  - **Done when:** Passing.

- [ ] **P10.T07** — Fault/protection scenarios & determinism test
  - **Depends:** P10.T02, P10.T03, P10.T05, P10.T06
  - **Do:** Scenarios in `presets/scenarios/` with asserts:
    - `phase-loss.yaml`
    - `wrong-offset.yaml`
    - `regen-overvoltage.yaml` (with and without chopper)
    - `stall-overcurrent.yaml`
    - `psu-current-limit.yaml`

    Add a determinism test: run every scenario twice and the Parquet outputs are byte-identical.
  - **Verify:** `bldc-sim run-scenario` asserts pass; `cargo nextest run determinism`.
  - **Done when:** Green.

- [ ] **P10.T08** — Phase gate
  - **Depends:** P10.T01, P10.T02, P10.T03, P10.T04, P10.T05, P10.T06, P10.T07
  - **Do:** PLAN §8 checklist. Also: confirm every catalog case with `side: rust` or `both` has a Rust test (`grep` the catalog IDs vs the test names; script it in `scripts/catalog_coverage.py` and add it to `just check`).
  - **Done when:** Tagged `phase-10-done`.
