# Phase 07 — Power electronics & power supply

> **Goal:** the inverter in both modes (switching with PWM edge events and dead time; averaged), the modulators, losses, the DC bus, lab PSU (CV/CC, cannot sink), battery, brake chopper and regeneration.
> **Depends on:** P06.
> **Read first:** `docs/docs/physics/inverter.md` (EQ-INV-*), `supply.md` (EQ-SUP-*), `numerics.md` (variable events).
> **Exit criteria:**
> - All V-INV and V-SUP cases pass.
> - Averaged and switching modes agree on period-averaged currents.
> - A regen scenario shows the bus voltage rising with a PSU, and being clamped with the chopper.
> - Sim/real ratio in switching mode is measured and logged.

- [ ] **P07.T01** — Inverter leg model (switching)
  - **Depends:** P06
  - **Do:**
    - Per-leg state High/Low/Off.
    - Terminal voltage from the switch state, the current sign (diode conduction when Off), R_ds,on and V_f.
    - Floating detection (Off with i = 0 → open-phase mode from P05.T07).
    - Writes v_a, v_b, v_c to the bus for the motor module.
    - The interim "ideal voltage source" from P05 stays available as `inverter.mode = ideal` for tests.
  - **Files:** `crates/sim-core/src/physics/inverter/leg.rs`
  - **Verify:** Tests:
    - Off with positive current → terminal clamped to −V_f (low diode)
    - with negative current → V_bus + V_f
    - correct R_ds,on drop
  - **Done when:** Passing.

- [ ] **P07.T02** — PWM generator with edge events, dead time, shadow registers
  - **Depends:** P07.T01
  - **Do:**
    - Center-aligned carrier at f_pwm.
    - Compare values are latched at the period start/center (shadow registers, as on real MCUs).
    - Generates **variable events** for each switch edge, including dead-time insertion (both switches Off for t_dead).
    - Emits a `pwm.center` event for ADC sampling (used by P08).
  - **Files:** `crates/sim-core/src/physics/inverter/pwm.rs`
  - **Verify:** Tests:
    - edge times exact to the ns for given duties
    - dead-time interval present on every transition
    - duty changes mid-period apply next period
  - **Done when:** Passing.

- [ ] **P07.T03** — Averaged inverter
  - **Depends:** P07.T01
  - **Do:** Duty → average terminal voltages, including conduction drop and optional dead-time average error (sign of current) per EQ-INV. Leg Off in averaged mode → diode rule as in switching mode (so six-step floating phases behave correctly). Mode selected by `fidelity.inverter_mode`.
  - **Files:** `crates/sim-core/src/physics/inverter/averaged.rs`
  - **Verify:** Test V-INV averaged ≡ switching: over a steady-state run, the period-averaged phase currents agree within 1 % of rated current.
  - **Done when:** Passing.

- [ ] **P07.T04** — Modulators: SPWM, SVPWM, six-step patterns
  - **Depends:** P07.T02
  - **Do:** Pure functions: (v_α, v_β, V_bus) → duties for SPWM and SVPWM (min-max injection), with an overmodulation clamp (keep the angle, scale the magnitude) and six-step leg patterns per sector (High/Low/Off). These are used by the controllers in P09.
  - **Files:** `crates/sim-core/src/physics/inverter/modulation.rs`
  - **Verify:** Tests:
    - max linear amplitude: SVPWM = V_bus/√3, SPWM = V_bus/2
    - SVPWM duties match the sector-based reference for 36 angles
    - six-step table correct
  - **Done when:** Passing.

- [ ] **P07.T05** — Inverter losses & DC-link current
  - **Depends:** P07.T03, P07.T04
  - **Do:** Conduction, switching and diode losses (switching mode: per edge; averaged: per formula). DC-link current per EQ-INV. Report losses to the energy terms. Signals `inverter.p_loss_*` and `inverter.i_dc`.
  - **Files:** `crates/sim-core/src/physics/inverter/losses.rs`
  - **Verify:** Tests:
    - P_in(dc) = P_out(phases) + P_loss within the energy tolerance in both modes
    - switching loss scales linearly with f_pwm
  - **Done when:** Passing.

- [ ] **P07.T06** — DC bus capacitor
  - **Depends:** P07.T05
  - **Do:** Plant module: V_bus state, C·dV/dt = i_src − i_dc − i_chopper. Report the stored energy ½CV². Signal `bus.v`.
  - **Files:** `crates/sim-core/src/physics/supply/bus.rs`
  - **Verify:** Test: a constant discharge current gives a linear voltage drop.
  - **Done when:** Passing.

- [ ] **P07.T07** — Lab PSU (CV/CC, unidirectional) and ideal source
  - **Depends:** P07.T06
  - **Do:** PSU per EQ-SUP: setpoint V, current limit, output resistance, CC mode, **cannot sink** (i_src ≥ 0). Mode signal `supply.mode` (CV/CC). The ideal source is for tests.
  - **Files:** `crates/sim-core/src/physics/supply/psu.rs`
  - **Verify:** Tests:
    - load above the limit → CC at I_lim, voltage sags
    - regen with no chopper → V_bus rises per the energy balance (V-SUP regen case)
  - **Done when:** Passing.

- [ ] **P07.T08** — Battery model
  - **Depends:** P07.T06
  - **Do:** The model chosen in P02.T07 (Thevenin 1-RC or Tremblay). SoC state; OCV(SoC) table from presets; bidirectional. Signals `supply.soc`, `supply.v_terminal`.
  - **Files:** `crates/sim-core/src/physics/supply/battery.rs`
  - **Verify:** Tests:
    - instantaneous sag = I·R0
    - RC relaxation time constant
    - SoC integration = ∫I dt / capacity
  - **Done when:** Passing.

- [ ] **P07.T09** — Brake chopper & over-voltage behavior
  - **Depends:** P07.T07
  - **Do:** Chopper with hysteresis (on above V_on, off below V_off), resistor R_brake, power to energy terms. Signals `bus.chopper_on`, `bus.p_brake`.
  - **Files:** `crates/sim-core/src/physics/supply/chopper.rs`
  - **Verify:** Test: a regen spin-down with the chopper keeps V_bus ≤ V_on + margin, and the energy dumped matches the kinetic energy minus losses.
  - **Done when:** Passing.

- [ ] **P07.T10** — V-INV/V-SUP sweep, performance check
  - **Depends:** P07.T03, P07.T04, P07.T05, P07.T08, P07.T09
  - **Do:** Implement all V-INV and V-SUP catalog cases as tests. Add a criterion benchmark for switching mode (20 kHz PWM, dt per tier rule) and log the sim/real ratio (release).
  - **Verify:** `cargo nextest run -p sim-core v_inv v_sup`; `just bench`.
  - **Done when:** Green, numbers logged. If switching mode is < 0.05× real time, add a P19 optimization task and note it.

- [ ] **P07.T12** — Rewire the interim drive through the real inverter & supply
  - **Depends:** P07.T03, P07.T04, P07.T07
  - **Do:** The skeleton FOC's v_α/v_β now go through SVPWM (P07.T04) → averaged inverter (default) or switching inverter → DC bus → lab PSU. The ideal source remains as `inverter.mode = ideal` for tests. Update the default scene, V-SKEL scenarios (with expected values updated only if the physics legitimately changes — note it in LOG) and the server.
  - **Verify:** `just test && just validate && just e2e` green.
  - **Done when:** Green, and the UI still spins the motor.

- [ ] **P07.T11** — Phase gate
  - **Depends:** P07.T01, P07.T02, P07.T03, P07.T04, P07.T05, P07.T06, P07.T07, P07.T08, P07.T09, P07.T10, P07.T12
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-07-done`.
