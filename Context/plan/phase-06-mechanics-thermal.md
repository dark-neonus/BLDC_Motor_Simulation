# Phase 06 — Mechanics & thermal

> **Goal:** the rotor/gearbox/load mechanical plant (including backlash, Karnopp friction and arm-under-gravity loads), the external disturbance input for the mouse "push", and the thermal network coupled to motor R and λ.
> **Depends on:** P05.
> **Read first:** `docs/docs/physics/mechanical.md` (EQ-MECH-*), `thermal.md` (EQ-THERM-*), [DECISIONS D-007](DECISIONS.md).
> **Exit criteria:** All V-MECH and V-THERM cases pass. Holding torque at standstill visibly heats the winding in a scenario (`presets/scenarios/hold-heats-motor.yaml`), and the temperature trajectory matches the analytic first-order approximation within the stated tolerance.

- [ ] **P06.T01** — Rotor + rigid gearbox + reflected inertia
  - **Depends:** P05
  - **Do:**
    - Extend/replace the minimal rotor from P05.T10: mechanical plant states θm, ωm (motor side).
    - Rigid gearbox (backlash = 0): load side θ_load = θm/N, reflected inertia and load torques.
    - Efficiency depends on the power-flow direction, with the smooth formulation from EQ-MECH.
    - Bypass gearbox = ratio 1, η = 1, J_gb = 0.
  - **Files:** `crates/sim-core/src/physics/mech/{rotor,gearbox}.rs`
  - **Verify:** Tests:
    - torque step acceleration = T/(J_m + J_gb + J_load/N²)
    - back-driven efficiency asymmetry per EQ
  - **Done when:** Passing.

- [ ] **P06.T02** — Backlash two-inertia model
  - **Depends:** P06.T01
  - **Do:** When backlash > 0, add load-side states θ_L, ω_L with a dead-zone contact spring-damper per EQ-MECH. Switch between the rigid and two-inertia formulations at scene build (not live), and document this in the UI help.
  - **Files:** `crates/sim-core/src/physics/mech/backlash.rs`
  - **Verify:** Tests:
    - lost-motion angle on a direction reversal = backlash
    - contact energy dissipation bounded
    - no chatter-induced NaN at dt_max
  - **Done when:** Passing.

- [ ] **P06.T03** — Karnopp friction (static/Coulomb/viscous)
  - **Depends:** P06.T01
  - **Do:** Implement per EQ-MECH, with the stick state handled via the velocity band and event logic from EQ-NUM. Friction applies on the motor bearing and optionally on the load side.
  - **Files:** `crates/sim-core/src/physics/mech/friction.rs`
  - **Verify:** Tests:
    - applied torque < static friction → stays stuck (ω ≡ 0, θ constant)
    - above it → breakaway; deceleration on coast matches Coulomb + viscous analytic
    - friction dissipation is accounted in energy
  - **Done when:** Passing.

- [ ] **P06.T04** — Arm + mass gravity load
  - **Depends:** P06.T01
  - **Do:**
    - Load params: arm length, arm mass, point mass and distance d, mounting offset angle.
    - Gravity torque and inertia per EQ-MECH. Gravitational potential energy is reported.
    - Live changes of mass/d/length via the param hook, honoring D-007 (default keep ω; option to conserve momentum).
  - **Files:** `crates/sim-core/src/physics/mech/load_arm.rs`
  - **Verify:** Tests:
    - unpowered small-angle oscillation period = 2π√(J_total/(m_eff·g·d_eff)) within 0.5 %
    - large-amplitude energy conservation without friction (residual < 1e-6)
    - live mass change follows the D-007 rule
  - **Done when:** Passing.

- [ ] **P06.T05** — Constant/viscous loads & disturbance torque input
  - **Depends:** P06.T01
  - **Do:** Constant-torque and viscous load modules. A `load.disturbance` input: torque pulse `{torque, duration}` or an impulse (converted to a pulse over max(1 ms, 5·dt)) — this is used by the mouse "push/flick". It is commanded via `Command::Disturbance`.
  - **Files:** `crates/sim-core/src/physics/mech/load_basic.rs`
  - **Verify:** Test: an impulse J_imp on a free rotor → Δω = J_imp/J_total.
  - **Done when:** Passing.

- [ ] **P06.T06** — Thermal network + R(T), λ(T) coupling
  - **Depends:** P05
  - **Do:**
    - 3-node thermal plant module (winding, stator, housing → ambient). Heat inputs from copper/iron/friction losses.
    - Each step, the motor module reads `thermal.t_winding` to compute R(T), and the magnet temperature (approximately the stator/housing node, as specified) to compute λ(T).
    - Signals: `thermal.t_winding`, `thermal.t_stator`, `thermal.t_housing` (stored in K, displayed in °C).
    - Overheat warning thresholds.
  - **Files:** `crates/sim-core/src/physics/thermal.rs`
  - **Verify:** Test: a step heat input → node time constants match the analytic solution of the linear network (eigenvalues); R(T) feedback raises the copper loss as specified. (The continuous-current check is in T07.)
  - **Done when:** Passing.

- [ ] **P06.T07** — Continuous/peak torque derivation helpers
  - **Depends:** P06.T06
  - **Do:** Functions `continuous_current(params, T_ambient)` and `peak_time(params, I)` (the time to reach T_max from ambient at current I), used by the constraint warnings (P04.T05 rule) and the UI.
    - Also add the constraint rule (P04.T05 framework) "ratings vs thermal continuous limit" (Warn) using these functions.
  - **Files:** `crates/sim-model/src/thermal_limits.rs` (pure math, no engine), `crates/sim-model/src/constraints/thermal.rs`
  - **Verify:** Unit tests in sim-model against the closed form (EQ-THERM). An integration test **in sim-core** (`crates/sim-core/tests/thermal_limits.rs`, sim-core may depend on sim-model): running I_cont until steady state gives T_max ± 1 K.
  - **Done when:** Passing.

- [ ] **P06.T08** — V-MECH/V-THERM sweep + "hold heats motor" scenario
  - **Depends:** P06.T02, P06.T03, P06.T04, P06.T05, P06.T07
  - **Do:** Implement all V-MECH and V-THERM catalog cases as Rust tests. Write the scenario `presets/scenarios/hold-heats-motor.yaml` (arm with mass held at 90° using the **interim drive** from P05.T11: the skeleton FOC current loop with a fixed i_q command sized to the gravity torque, and its velocity/position loops disabled; record temperatures; assert the final temperature band).
  - **Verify:** `cargo nextest run -p sim-core v_mech v_therm`; the scenario passes its asserts via the CLI.
  - **Done when:** Green.

- [ ] **P06.T10** — P05 review carry-overs
  - **Depends:** P05
  - **Do:**
    1. Saturation and open-phase Newton solves report non-convergence (non-PD Jacobian, non-positive slope, iteration cap) as `SimError::Numerical` or an engine warning instead of returning unconverged currents silently (PLAN §4 rule 5). Use a step-size stop as in `solve_open`, so a 1e-15 residual floor does not cost ~2000 evaluations.
    2. When the λ(T) hook is wired (P06.T06), check that `energy.residual` stays within r_tol over a heating run (energy.md: the unbooked magnet work is neglected); book it if it does not.
    3. Strengthen the V-MOT tests:
       - V-MOT-006 uses the production Kv → λ derivation.
       - V-MOT-011 checks i_c ≈ 0 at the switch from the Clarke-derived currents and the a–b path time constant 2L/2R.
       - V-MOT-007 gets the period and zero-mean checks (atol 1e-9).
       - V-MOT-009 drives above 2·i_k and compares torque.
       - V-MOT-008 passes through ω = 0.
    4. Missing tests:
       - a harmonic-shape EMF with a triplen (no torque with an isolated neutral), energy closing;
       - a live `l_d`/`lambda_m` edit books ΔW_mag (residual within tolerance);
       - saturation in open-phase mode;
       - production `build_engine` with iron loss.
  - **Files:** `crates/sim-core/src/physics/motor/electrical.rs`, `crates/sim-core/tests/{v_mot,motor_*}.rs`
  - **Verify:** `cargo nextest run -p sim-core` green, with the new tests listed.
  - **Done when:** Green.

- [ ] **P06.T09** — Phase gate
  - **Depends:** P06.T01, P06.T02, P06.T03, P06.T04, P06.T05, P06.T06, P06.T07, P06.T08, P06.T10
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-06-done`.
