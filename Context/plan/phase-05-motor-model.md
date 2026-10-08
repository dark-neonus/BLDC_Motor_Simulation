# Phase 05 — Motor electromagnetic model

> **Goal:** the production motor plant module per D-002. abc phase-current states with isolated neutral, general back-EMF shape, torque, cogging, iron loss, saturation (Detailed) and open-phase support, all reporting energy terms.
> **Depends on:** P04.
> **Read first:** `docs/docs/physics/motor.md` (EQ-MOT-*), `numerics.md`, `energy.md`, [DECISIONS D-002](DECISIONS.md).
> **Exit criteria:**
> - All V-MOT cases from the catalog are implemented as Rust tests and pass.
> - abc ≡ dq equivalence holds for the sinusoidal case.
> - The skeleton dq plant is removed and replaced.
>
> **Keeping the app green:** P05.T10 adds a minimal rigid rotor (the full mechanics come in P06). P05.T11 builds the **interim drive path**: the skeleton FOC (kept from P03.T12) → inverse Park → an "ideal voltage source" module writing phase voltages. This keeps the server, UI, E2E and V-SKEL validation working until P07 (real inverter) and P09 (real FOC) replace them.

- [ ] **P05.T01** — abc electrical plant module
  - **Depends:** P05.T10
  - **Do:** `MotorElectrical: PlantModule`:
    - States ψ_α, ψ_β (stationary-frame flux linkages, D-013). Currents come from the flux–current relation (EQ-MOT-02, closed form in the linear case); i_c = −i_a − i_b.
    - Inputs: terminal voltages v_a, v_b, v_c from the bus, θe and ω from the mechanical states.
    - Neutral voltage per EQ-MOT; di/dt with Ls = L − M.
    - Outputs: phase currents, back-EMFs, line voltages, i_d/i_q (computed per CONVENTIONS), electrical power.
  - **Files:** `crates/sim-core/src/physics/motor/electrical.rs`
  - **Verify:** V-MOT locked-rotor RL test in abc (a phase-to-phase step) → τ = Ls/R.
  - **Done when:** Passing.

- [ ] **P05.T02** — Back-EMF shape functions & torque
  - **Depends:** P05.T01
  - **Do:**
    - Shape functions: sinusoidal, trapezoidal (flat-top width param), harmonic table.
    - Torque T = p·λ·Σ f_x(θe)·i_x (EQ-MOT singularity-free form).
    - Temperature-dependent λ hook (filled in by P06 thermal).
    - Signals: `motor.e_a..c`, `motor.torque_em`.
  - **Files:** `crates/sim-core/src/physics/motor/{backemf,torque}.rs`
  - **Verify:** Tests:
    - back-EMF LL peak at ω = Ke,LL,pk·ω_m
    - stall torque with i_q = I → Kt·I
    - trapezoidal shape has the correct flat-top width and fundamental normalization
  - **Done when:** Passing.

- [ ] **P05.T03** — abc ≡ dq equivalence test
  - **Depends:** P05.T02
  - **Do:** Run the abc model with sinusoidal back-EMF and the skeleton dq model (the test fixture from P03.T12) with identical sinusoidal voltages and initial conditions, and compare i_d, i_q, ω and T over 0.2 s. Afterwards no production code path uses the dq plant (it stays as a fixture only).
  - **Files:** `crates/sim-core/tests/abc_dq_equivalence.rs`
  - **Verify:** Difference ≤ 1e-9 abs + 1e-6 rel at the same dt (CONVENTIONS §7; i_d ≈ 0 needs the abs floor).
  - **Done when:** Passing; the skeleton is gone from production code.

- [ ] **P05.T04** — Cogging torque
  - **Depends:** P05.T02
  - **Do:** T_cog = Σ A_k sin(k·N_c·θm + φ_k), using N_c from `winding.rs`. Enabled by `fidelity.enable_cogging`. Default amplitude per preset (typically 1–5 % of rated torque; cite the source in `_sources.md`). Cogging is conservative (it has an associated potential energy, report it).
  - **Files:** `crates/sim-core/src/physics/motor/cogging.rs`
  - **Verify:** Tests: period = 2π/N_c; zero average over a period; energy conservation of the unpowered rotor with cogging and no friction (`energy.residual` < 1e-6 over 1 s).
  - **Done when:** Passing.

- [ ] **P05.T05** — Iron losses
  - **Depends:** P05.T02
  - **Do:** P_iron = k_h·|ωe| + k_e·ωe², applied as a drag torque P_iron/|ω_m| with a smooth regularization near ω = 0 per EQ-MOT. Loss heat goes to the stator thermal node (P06). Enabled by fidelity flag.
  - **Files:** `crates/sim-core/src/physics/motor/iron_loss.rs`
  - **Verify:** Test: no-load spin-down deceleration matches the analytic integration of the drag law.
  - **Done when:** Passing.

- [ ] **P05.T06** — Saturation (Detailed tier)
  - **Depends:** P05.T03
  - **Do:** Implement the energy-consistent saturation per EQ-MOT (e.g. ψq(iq) curve and the co-energy torque), with the inductance matrix/flux formulation as specified. Default curves: knee at ~1.5–2× rated current, based on the preset.
  - **Files:** `crates/sim-core/src/physics/motor/saturation.rs`
  - **Verify:** Tests:
    - below the knee, behaves like the linear model within 1 %
    - above it, torque per amp drops per the curve
    - energy residual is within tolerance for a current pulse into the saturation region
  - **Done when:** Passing.
  - **If stuck:** If the abc formulation with saturation is too complex, implement saturation in a dq-derived inductance for sinusoidal machines only. Record a decision and document it in the spec.

- [ ] **P05.T07** — Open-phase support
  - **Depends:** P05.T01
  - **Do:** When a phase terminal is disconnected (fault, or a floating inverter leg with zero current after the diodes stop conducting), switch the model to a single-current-state mode (current flows only between the two connected phases) per EQ-MOT. Transitions are event-handled at current zero crossing.
  - **Files:** `crates/sim-core/src/physics/motor/electrical.rs`
  - **Verify:** Test: open phase C during operation → i_c stays 0 and i_a = −i_b, with no NaN and energy within tolerance.
  - **Done when:** Passing.

- [ ] **P05.T08** — Energy terms & V-MOT test sweep
  - **Depends:** P05.T04, P05.T05, P05.T06, P05.T07
  - **Do:** Report copper loss, iron loss, magnetic stored energy, cogging potential and air-gap power through `energy_terms()`. Implement **all V-MOT catalog cases** as Rust tests (`crates/sim-core/tests/v_mot.rs`), named by ID.
  - **Verify:** `cargo nextest run -p sim-core v_mot` all green.
  - **Done when:** Every V-MOT ID has a test.

- [ ] **P05.T10** — Minimal rigid rotor module
  - **Depends:** P04
  - **Do:** `RotorRigid: PlantModule` with states θm, ωm; J·dω/dt = T_em + T_cog − T_drag − B·ω + T_ext (EQ-MECH, rigid case only, no gearbox/load/Karnopp yet). Outputs θm, ωm, θe = p·θm via the per-stage outputs pass. Kinetic energy and viscous loss are reported to the energy framework (P03.T16). P06.T01 extends or replaces it.
  - **Files:** `crates/sim-core/src/physics/mech/rotor.rs`
  - **Verify:** Test: a constant torque gives ω = T·t/J (B = 0); a viscous spin-down gives the exponential with τ = J/B.
  - **Done when:** Passing.

- [ ] **P05.T11** — Interim drive path (keeps the app working until P07/P09)
  - **Depends:** P05.T01, P05.T02
  - **Do:** An `IdealVoltageSource` module (`inverter.mode = ideal`): phase voltages = the commanded values, with no bus or losses (V_bus constant for limits). Adapt the skeleton FOC block to output v_α/v_β → v_a, v_b, v_c (inverse Clarke). Rebuild the default scene, server, CLI and V-SKEL scenarios on **abc motor + rigid rotor + ideal source + skeleton FOC**. Mark the skeleton FOC `// INTERIM: replaced in P09.T03`.
  - **Files:** `crates/sim-core/src/physics/inverter/ideal.rs`, skeleton FOC adaptation
  - **Verify:** `just test && just validate && just e2e` all green (the same P01 behaviors now on the abc model).
  - **Done when:** Green.

- [ ] **P05.T09** — Phase gate
  - **Depends:** P05.T01, P05.T02, P05.T03, P05.T04, P05.T05, P05.T06, P05.T07, P05.T08, P05.T10, P05.T11
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-05-done`.
