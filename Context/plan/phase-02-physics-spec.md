# Phase 02 — Physics specification (docs-first)

> **Goal:** write the complete, referenced specification of every model, equation, convention, numerical method and validation case **before** implementing physics.
> The spec is the single source of truth for the Rust core (P03–P10), the independent Python reference model (P11.T01) and the user-facing docs (P18).
> **Depends on:** P01.
> **Read first:** [CONVENTIONS §5](CONVENTIONS.md), [DECISIONS D-002, D-004, D-007](DECISIONS.md), POLISHED_IDEA §3, §5, §10.
> **Output location:** `docs/docs/physics/*.md`. Each equation gets an ID and an anchor, `### EQ-<AREA>-<NN> — Title {/* #eq-area-nn */}` (MDX comment form, CONVENTIONS §8).
> Areas: `CONV`, `MOT`, `MECH`, `THERM`, `INV`, `SUP`, `SENS`, `CTRL`, `NUM`, `ENER`. Validation cases use `V-<AREA>-<NNN>`.
> **Exit criteria:**
> - Every model in POLISHED_IDEA §3, §5.2 and §5.4 has equations with symbols, units, assumptions and ≥ 1 reference.
> - The validation catalog lists ≥ 40 cases with expected values or formulas and tolerances.
> - The docs build is clean.
> - The phase-end reviewer finds no blocker-level physics errors.
>
> **Writing rules for every page:**
> - A **symbol table** (symbol, meaning, unit, code name).
> - **Assumptions**, stated explicitly.
> - Equations in LaTeX.
> - **Parameter ranges** for typical low-RPM motors.
> - A **References** section (book chapter/page, paper DOI or app note).
> - Use web search to confirm any formula you are not certain of, and prefer primary sources.
> - When sources disagree, state both and pick one with a reason.
>
> Do NOT invent physics. If something is a modeling approximation (e.g. encoder noise vs air gap), label it **Approximation** and explain why it is good enough for learning.

- [x] **P02.T01** — References page
  - **Depends:** P01
  - **Do:** Create `docs/docs/physics/references.md`. Collect and verify (title, authors, year, ISBN/DOI/URL) at minimum:
    - Krishnan, *Permanent Magnet Synchronous and Brushless DC Motor Drives* (CRC 2010)
    - Mohan, *Advanced Electric Drives* (Wiley 2014)
    - Hendershot & Miller, *Design of Brushless Permanent-Magnet Machines* (2010)
    - Holmes & Lipo, *Pulse Width Modulation for Power Converters* (2003)
    - Lee, Hong, Nam, Ortega, Praly, Astolfi, "Sensorless control of surface-mount PMSMs based on a nonlinear observer", IEEE TPEL 2010
    - Karnopp, "Computer simulation of stick-slip friction in mechanical dynamic systems", 1985
    - Tremblay & Dessaint, "Experimental validation of a battery dynamic model for EV applications", 2009
    - TI/ST/Microchip FOC & SVPWM application notes
    - ODrive / SimpleFOC / VESC documentation
    - ams AS5047P / AS5600 and MT6701 datasheets
    - Hairer/Nørsett/Wanner (RK methods)

    Give each a short key (`[Krishnan2010]`) for citation.
  - **Files:** `docs/docs/physics/references.md`
  - **Verify:** Every entry has a resolvable identifier (check URLs/DOIs).
  - **Done when:** The page builds and the keys are used consistently.

- [x] **P02.T02** — Conventions (EQ-CONV-*)
  - **Depends:** P02.T01
  - **Do:** Write `conventions.md`:
    - Frames: abc, αβ, dq.
    - Amplitude-invariant Clarke/Park, with matrices and inverses.
    - Electrical vs mechanical angle; pole pairs; rotation and power sign conventions.
    - **Motor constant relations** with derivations: λ_m → Kt = 1.5pλ, Ke,LL,pk = √3pλ, Kv = 60/(2π·Ke,LL,pk), Kt ≈ 8.27/Kv.
    - A table converting common datasheet variants (Kv by RMS LL, Kv measured as six-step no-load rpm per DC volt, Kt per RMS amp) to λ_m.
    - Star/delta equivalents; R/L line-to-line vs phase.
    - Temperature units.
  - **Files:** `docs/docs/physics/conventions.md`
  - **Verify:** A numeric worked example: Kv = 100 rpm/V, p = 14 → λ, Kt, Ke. Recompute by hand in the page.
  - **Done when:** Conventions match CONVENTIONS.md §5 exactly. If you find an error there, fix both and add a decision.

- [x] **P02.T03** — Motor electromagnetic model (EQ-MOT-*)
  - **Depends:** P02.T02
  - **Do:** Write `motor.md`:
    1. abc voltage equations with self L and mutual M, isolated star neutral (neutral voltage expression, i_a + i_b + i_c = 0, two independent states).
    2. Back-EMF shape functions f_x(θe), normalized so the fundamental amplitude = 1:
       - sinusoidal
       - trapezoidal with flat-top width parameter (120° classic)
       - harmonic table
    3. Torque from power balance T = Σ e_x·i_x / ω_m, written in the singularity-free form T = p·λ·Σ f_x(θe)·i_x (show the equivalence).
    4. dq reduction for the sinusoidal case and the **equivalence statement** used as a test (V-MOT-equivalence).
    5. Cogging torque T_cog = Σ A_k sin(k·N_c·θm + φ_k), N_c = LCM(slots, 2p).
    6. Iron loss model (hysteresis ∝ ωe, eddy ∝ ωe²) as power and equivalent drag torque. **Approximation** label.
    7. Saturation (Detailed tier): energy-consistent formulation, e.g. q-axis inductance as a function of current via a flux-linkage curve ψq(iq), with co-energy-based torque. State how abc integration uses it; if a position-dependent inductance matrix is needed, specify it.
    8. Open-phase behavior (a terminal disconnected).
  - **Files:** `docs/docs/physics/motor.md`
  - **Verify:** Each equation has units. A dimensional check table is included.
  - **Done when:** Complete, with references.

- [ ] **P02.T04** — Mechanical model (EQ-MECH-*)
  - **Depends:** P02.T02
  - **Do:** Write `mechanical.md`:
    - Rotor inertia (from geometry: rotor as a cylinder/shell, inner vs outer rotor formulas).
    - Rigid gearbox: reflected inertia J_load/N²; efficiency applied **depending on power-flow direction** (motor driving vs back-driven), using a smooth formulation to avoid chatter.
    - Backlash: two-inertia model with dead-zone contact spring-damper (stiffness/damping parameters and their typical values).
    - Friction: Karnopp model (static, Coulomb, viscous, velocity band). Stick condition and the holding behavior must be specified precisely.
    - Loads: arm (uniform rod mass + point mass at d) under gravity: τ_g = −(m_rod·L/2 + m·d)·g·sin θ_load. Inertia J = m_rod·L²/3 + m·d². Plus constant torque, viscous load and external disturbance torque input.
    - **Live parameter change rule** (D-007: keep ω by default; optional momentum conservation, with formula).
  - **Files:** `docs/docs/physics/mechanical.md`
  - **Verify:** Small-angle pendulum period formula is included (used by V-MECH tests).
  - **Done when:** Complete, with references.

- [ ] **P02.T05** — Thermal model (EQ-THERM-*)
  - **Depends:** P02.T02
  - **Do:** Write `thermal.md`:
    - 3-node RC network (winding → stator iron → housing → ambient) with heat inputs: copper loss → winding, iron loss → stator, friction → housing (state the assumption).
    - **Magnet temperature node per topology** (state the approximation): inrunner magnets on the rotor inside the stator, outrunner magnets on the rotor can near the housing/air. Either add a rotor node or map to an existing node with justification.
    - Copper: R(T) = R_ref·(1 + α_Cu·(T − T_ref)), α_Cu = 0.00393 1/K.
    - NdFeB: λ(T) = λ_ref·(1 + α_Br·(T − T_ref)), α_Br ≈ −0.0012 1/K (check the source; give the range for grades).
    - Typical thermal resistances/capacitances by motor size, with sources or a clearly labeled estimate method.
    - Continuous current/torque from the steady state at T_winding,max. Peak current time limit from the thermal time constant.
  - **Files:** `docs/docs/physics/thermal.md`
  - **Verify:** The steady-state formula is solvable in closed form (show it, including the R(T) feedback).
  - **Done when:** Complete.

- [ ] **P02.T06** — Inverter & modulation (EQ-INV-*)
  - **Depends:** P02.T03
  - **Do:** Write `inverter.md`:
    - Three-leg two-level inverter. Per-leg states: high, low, off (both off → diode conduction set by current sign; floating when current = 0).
    - Terminal voltage computation including diode drop and MOSFET R_ds,on.
    - Dead time; center-aligned PWM carrier; shadow-register update at the period boundary.
    - Averaged model (duty → average voltage, optional dead-time average error).
    - SPWM; SVPWM via min-max injection (show equivalence to sector-based SVPWM); overmodulation clamp; six-step patterns.
    - Losses: conduction I²R_ds,on; switching ½·V·I·(t_r + t_f)·f_sw; diode V_f·I.
    - DC-link current i_dc = Σ S_x·i_x (switching) or the averaged equivalent.
  - **Files:** `docs/docs/physics/inverter.md`
  - **Verify:** A worked example of SVPWM duties for one vector.
  - **Done when:** Complete.

- [ ] **P02.T07** — Power supply & DC bus (EQ-SUP-*)
  - **Depends:** P02.T06
  - **Do:** Write `supply.md`:
    - Bus capacitor C·dV/dt = i_src − i_dc − i_chopper.
    - Lab PSU CV/CC: unidirectional (cannot sink), output resistance, CC fold-back, transition logic, and how regen current raises V_bus.
    - Battery: Thevenin 1-RC or the Tremblay model (choose and justify). SoC integration; OCV(SoC) curves for Li-ion/LiPo/LiFePO4 presets; can sink current (charging).
    - Brake chopper with hysteresis thresholds and resistor power.
    - Over/undervoltage definitions.
    - Ideal source (for tests).
  - **Files:** `docs/docs/physics/supply.md`
  - **Verify:** Regen energy example: a flywheel spinning down into the bus capacitor gives the final V by energy balance.
  - **Done when:** Complete.

- [ ] **P02.T08** — Sensors (EQ-SENS-*)
  - **Depends:** P02.T03
  - **Do:** Write `sensors.md`:
    - Common sensor pipeline: true value → error model → quantization → noise (Gaussian, seeded) → latency (pure delay) → sample-and-hold at the update rate.
    - Halls: 3 signals at 120° electrical, placement offsets, state → sector table.
    - On-axis magnetic encoder: bits, INL as harmonic error, magnet eccentricity → first-harmonic angle error. Air gap → noise σ (**Approximation**, based on datasheet recommended-gap ranges). Chip presets with datasheet numbers (AS5600 12-bit, AS5047P 14-bit, MT6701 14-bit; update rates/latency from datasheets).
    - Incremental optical encoder: CPR, quadrature ×4, index.
    - ADC: bits, range, gain/offset error, noise, sampling instant (PWM center), saturation.
  - **Files:** `docs/docs/physics/sensors.md`
  - **Verify:** Chip parameters are cited to datasheets.
  - **Done when:** Complete.

- [ ] **P02.T09** — Control algorithms & estimators (EQ-CTRL-*)
  - **Depends:** P02.T06, P02.T08
  - **Do:** Write `control.md`:
    - Discrete PI/PID (form, discretization, anti-windup by clamping and by back-calculation, derivative filter).
    - FOC: Clarke/Park on measured currents, PI d/q, decoupling feed-forward, voltage-limit circle and d-priority, inverse Park → SVPWM.
    - Cascade velocity/position with enable/disable semantics; setpoint ramps.
    - Six-step: hall-based commutation table, duty control.
    - Open-loop sinusoidal (V/f).
    - MIT/impedance law τ = Kp(θ*−θ) + Kd(ω*−ω) + τ_ff → iq_ref = τ/Kt (with gear ratio).
    - Velocity estimation (difference + LPF, PLL).
    - Nonlinear flux observer [Lee2010] + PLL, with expected low-speed failure and the reason.
    - Sensorless startup (align, open-loop ramp, handoff).
    - Encoder offset calibration.
    - **Auto-tune:** R measurement (DC injection), L measurement (voltage step or HF injection), inertia (torque step / acceleration), and gain formulas: current loop by pole-zero cancellation, velocity loop from J with bandwidth separation, position loop.
  - **Files:** `docs/docs/physics/control.md`
  - **Verify:** Closed-loop bandwidth formula for the current loop with the pole-zero-cancellation design is stated (used by V-CTRL tests).
  - **Done when:** Complete.

- [ ] **P02.T10** — Energy balance (EQ-ENER-*)
  - **Depends:** P02.T03, P02.T04, P02.T05, P02.T06, P02.T07
  - **Do:** Write `energy.md`:
    - Every power term: source/battery output; chopper; inverter conduction, switching and diode losses; copper; iron; friction; gearbox loss; load work (gravity potential, external).
    - Stored energies: bus ½CV², magnetic ½(L−M)Σi² (or the saturation-consistent co-energy), kinetic ½Jω² for each inertia, backlash spring, gravitational potential, battery chemical (via OCV integral), thermal (if counted).
    - Residual definition: E_in − (E_out + E_loss + ΔE_stored), normalized by the total throughput energy. Target |residual| < 1e-3 (state why, based on integrator error).
  - **Files:** `docs/docs/physics/energy.md`
  - **Verify:** Every power term maps to exactly one block that reports it.
  - **Done when:** Complete.

- [ ] **P02.T11** — Numerical methods (EQ-NUM-*)
  - **Depends:** P02.T03
  - **Do:** Write `numerics.md`:
    - Time base (D-004).
    - Hybrid scheme: integrate the continuous plant between discrete events with zero-order-hold inputs.
    - RK4 (Butcher tableau).
    - Exponential/semi-implicit treatment of stiff RL electrical states (formula and stability argument).
    - Step-size rules per tier (dt_max relative to τ_e = L/R and the PWM period).
    - Energy integration as extra states.
    - Event ordering priorities; the friction stick event.
    - diffsol reference configuration (method, tolerances).
    - Determinism requirements.
  - **Files:** `docs/docs/physics/numerics.md`
  - **Verify:** A stability-limit derivation for RK4 on di/dt = −(R/L)·i is included (|λ·dt| ≤ 2.785).
  - **Done when:** Complete.

- [ ] **P02.T12** — Validation catalog (V-*)
  - **Depends:** P02.T03, P02.T04, P02.T05, P02.T06, P02.T07, P02.T08, P02.T09, P02.T10, P02.T11
  - **Do:** Write `validation-catalog.md`. Each case gets: ID, purpose, **side** (`rust` = Rust unit test, `python` = CLI-level pytest, `both`), scenario description (params, inputs, duration), expected result (formula/value), tolerance with justification, and fidelity tier. At least 40 cases covering:
    - Locked-rotor RL; no-load speed; stall torque; back-EMF amplitude vs speed; abc ≡ dq for sinusoidal.
    - Six-step torque ripple, both cases stated separately: trapezoidal back-EMF with ideal 120° currents (≈ 0 % apart from commutation transients), and **sinusoidal** back-EMF driven six-step (ripple = 1 − cos 30° ≈ 13.4 %). Cogging period.
    - Pendulum period; friction holding / breakaway; gearbox reflected inertia; backlash dead-zone.
    - Thermal step time constants; continuous current.
    - Averaged ≡ switching period-average; SVPWM max linear voltage Vdc/√3; dead-time voltage error sign.
    - Regen bus voltage rise; PSU CC limit; battery sag = I·R0.
    - Encoder quantization step; hall sector transitions.
    - PI current-loop bandwidth; velocity step settling; MIT stiffness (static deflection under load = τ/Kp).
    - Observer convergence above min speed.
    - Energy residual for all scenarios; determinism (two runs bit-identical).
  - **Files:** `docs/docs/physics/validation-catalog.md`
    - **Every formula states its conventions explicitly**: which voltage (DC bus, line-to-line peak, phase peak, RMS) and which current (peak phase = i_q, RMS, DC). Example: no-load speed ω = V_LL,pk/Ke,LL,pk (= Kv·V_LL,pk in rpm), *not* "Kv·V_bus" without qualification; stall torque under FOC = Kt·i_q; a six-step DC-bus formulation is listed separately.
    - Tolerances use `atol + rtol·|expected|` (CONVENTIONS §7).
  - **Verify:** `just plan-check` still passes; the docs build is clean.
  - **Done when:** ≥ 40 cases, each traceable to EQ IDs.

- [ ] **P02.T14** — Signal & parameter name catalog
  - **Depends:** P02.T03, P02.T04, P02.T05, P02.T06, P02.T07, P02.T08, P02.T09, P02.T10
  - **Do:** Write `docs/docs/physics/signals.md`: the **canonical list** of every signal path (CONVENTIONS §3) produced by each model (e.g. `motor.i_a`, `motor.omega`, `motor.theta_e`, `load.theta`, `bus.v`, `thermal.t_winding`, `ctrl.foc.iq_ref`, `energy.residual`) with unit, meaning, producing block and EQ ID, plus the top-level parameter paths per component. The Rust code, the refmodel (P11.T01), the help registry and the UI all use these names. Rename any skeleton (P01) names that differ, in code and in this file, in the same task.
  - **Files:** `docs/docs/physics/signals.md`
  - **Verify:** `grep -rhoE '"(motor|load|bus|ctrl|sensors|thermal|energy|supply|inverter)\.[a-z0-9_.]+"' crates/ | sort -u` is a subset of the catalog.
  - **Done when:** The catalog is complete for every model in P02.

- [ ] **P02.T13** — Phase gate (physics review is critical here)
  - **Depends:** P02.T01, P02.T02, P02.T03, P02.T04, P02.T05, P02.T06, P02.T07, P02.T08, P02.T09, P02.T10, P02.T11, P02.T12, P02.T14
  - **Do:** PLAN §8 checklist. In the reviewer prompt, add: "Focus on physics correctness, sign conventions, unit consistency, missing assumptions and whether an independent implementer could code every model from these pages alone without guessing."
  - **Verify:** All blocker/major findings are resolved.
  - **Done when:** Tagged `phase-02-done`. **P11.T01 (reference model) is now startable in parallel.**
