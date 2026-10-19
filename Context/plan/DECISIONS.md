# Decision Log

> ADR-style, append-only. Number sequentially. To reverse a decision, add a new one that supersedes it; don't edit old ones.
> Format: **D-NNN — Title** · date · status (accepted / superseded by D-xxx) · context → decision → consequences.

## D-001 — Local backend, Rust core, single binary · 2026-10-08 · accepted
- **Context:** Switching-level PWM needs roughly 1e5–1e6 solver steps/s. Agents and the UI need one source of truth.
- **Decision:** The simulation runs in a native Rust process (`bldc-sim serve`) that hosts REST + WS + MCP and serves the embedded web UI. The browser is a view only.
- **Consequences:** Agents can drive the sim with no tab open. UI interaction pays localhost WS latency (negligible). See TOOLS_TO_USE.md.

## D-002 — Unified abc phase model as the single motor formulation · 2026-10-08 · accepted
- **Context:** We need trapezoidal and sinusoidal back-EMF, six-step drives with floating phases, open-phase faults and switching inverters. A dq model only handles the sinusoidal, balanced case.
- **Decision:** The plant integrates **phase currents in the abc frame** (isolated star neutral, two independent currents) with a general back-EMF shape function. dq quantities are *computed* for display and control, never integrated separately. The sinusoidal case is tested against the analytic dq model (EQ-MOT-equivalence test).
- **Consequences:** One model, no divergence between tiers. Saliency (Ld ≠ Lq) needs a position-dependent inductance matrix (Detailed tier, P05.T05).

## D-003 — `uom` at boundaries, `f64` SI inside hot state vectors · 2026-10-08 · accepted
- **Context:** The user chose compile-time units. `uom` with nalgebra state vectors and integrators would be very verbose.
- **Decision:** `sim-model` types, block constructors and public functions use `uom` quantities. ODE state vectors and inner math use `f64` SI with `// [unit]` comments. Conversion happens once, at block construction or parameter change.
- **Consequences:** Unit errors are caught where humans/agents enter values. Inner math stays readable.

## D-004 — Time base in integer nanoseconds · 2026-10-08 · accepted
- **Context:** Multi-rate blocks (e.g. 20 kHz, 5 kHz, 1 kHz, PWM edges) accumulate floating-point drift and give ambiguous event ordering.
- **Decision:** `SimTime(i64)` in **nanoseconds**. All rates are converted to integer periods. The integrator works in seconds (`f64`) between events. Coincident events are ordered by a fixed block priority (sensors → estimators → controllers → modulator → inverter).
- **Consequences:** Exact, reproducible event timing. Max rate is 1 GHz (more than enough), and the max sim duration is about 292 years.

## D-005 — Validation Python pinned to 3.13 · 2026-10-08 · accepted
- **Context:** The system has Python 3.14. Scientific wheels (pyarrow, scipy) may lag for the newest Python.
- **Decision:** `validation/` uses Python 3.13 via uv (`requires-python = ">=3.13,<3.14"`). Revisit when all deps ship 3.14 wheels.

## D-006 — Single shared simulation instance · 2026-10-08 · accepted
- **Context:** Local single-user app. The UI and agents should see the *same* running simulation.
- **Decision:** The server owns exactly one live engine instance. All UI tabs and MCP clients observe and control it. Batch runs (`run-scenario`, auto-tune measurements and validation) use separate, short-lived engine instances that don't disturb the live one.
- **Consequences:** No session management is needed. Changes made by an agent show live in the UI (the event channel broadcasts changes).

## D-007 — Live load-mass change preserves angular velocity · 2026-10-08 · accepted (revisit in P02.T04)
- **Context:** When the user changes mass or arm length while running, the inertia changes instantly.
- **Decision:** Default: keep ω (the mass is "already moving with the arm"). Optional scene flag `load.on_change: conserve_momentum`. The P02 spec documents both.

## D-008 — Docs widgets are analytic, client-side only · 2026-10-08 · accepted
- **Context:** The docs site is static and must work without the server.
- **Decision:** Interactive widgets in the docs compute simple analytic curves in TS (e.g. back-EMF vs angle) and reuse app UI components, but never call the simulator. Lessons that need the sim live in the app's lesson panel.

## D-009 — Docs embedded in the binary at `/docs` from the start · 2026-10-08 · accepted
- **Context:** Tooltips, MCP resources (`bldc://docs/llms.txt`) and offline use need the docs reachable from the running app before P18.
- **Decision:** From P01.T10, `just build` also builds the docs and embeds them (feature `embed-docs`) at `/docs`. The help base URL defaults to `/docs` (dev: `localhost:3000`).

## D-010 — Frequencies become integer-ns periods by rounding · 2026-10-08 · accepted
- **Context:** Many real rates (30 kHz = 33 333.3 ns) don't have an exact integer-ns period (D-004).
- **Decision:** Round to the nearest ns. Expose the **actual** frequency as a diagnostic signal and warn when the relative error is > 1e-6. Don't reject.

## D-011 — Pinned toolchain versions · 2026-10-08 · accepted
- **Context:** P00.T03 needs reproducible toolchains.
- **Decision:** `mise.toml` pins rust 1.99.0 (with `rust-toolchain.toml`: rustfmt, clippy), node 24.21.0 (LTS), pnpm 12.10.1, python 3.13.16, uv 0.12.23, just 1.58.0, lefthook 2.2.0, gh 2.102.0, cargo-nextest 0.9.146, cargo-insta 1.49.0. These were the latest at this date.
- **Consequences:** `mise install` reproduces the environment. Bump versions deliberately, with a new decision entry.

## D-012 — State wire format uses dotted signal paths; commands reply with post-apply state · 2026-10-09 · accepted
- **Context:** The P01 review found the skeleton JSON/msgpack keys were bare (`omega`, `running`) and that REST/MCP returned state after a 5 ms sleep (racy).
- **Decision:** REST, WS (msgpack frames `{type, v, ...}`) and MCP (`structured_content`) all serialize state with the CONVENTIONS §3 paths (`sim.t`, `motor.omega`, `ctrl.omega_ref`, `sim.running` …). Commands go to the runner with a reply channel, and the response is the state right after the command was applied (`SimHandle::request`).
- **Consequences:** These interfaces are stable from P01 on (P12 extends them). UI and tests index by dotted keys.

## D-013 — Electrical states are stationary-frame flux linkages (ψα, ψβ) · 2026-10-09 · accepted (refines D-002)
- **Context:** The spec needs saliency (Ld ≠ Lq), magnetic saturation and back-EMF harmonics in one energy-consistent model. With current states this needs position/current-dependent inductance matrices and their derivatives.
- **Decision:** The motor plant integrates **ψα, ψβ** with dψ/dt = v − R·i (EQ-MOT-01). Currents follow by inverting the flux–current relation: closed form when linear, 2-D Newton with saturation (EQ-MOT-02/10). With an isolated neutral this is equivalent to integrating (ia, ib). It is still one stationary-frame model, as D-002 requires; dq quantities are only computed, never integrated. Open-phase mode integrates the line flux ψ_ab (EQ-MOT-11).
- **Consequences:** Saliency and saturation need no dL/dθ terms. Energy accounting is exact by construction (verified numerically: the residual scales with the integrator order). P05.T01/T06 implement flux states.

## D-014 — Constraint rules are code with issue paths; three planned rules dropped · 2026-10-09 · accepted (P04 gate review)
- **Context:** P04.T05 asked for declarative rule records `{id, inputs, outputs, kind, …}` and listed "λ plausibility vs size class" and "L ≥ M sanity". The phase-end review flagged these as missing.
- **Decision:** Rules stay plain Rust functions (`constraints::validate`, `components::validate_*`, `scene::validate_scene`), each tested by name; every issue carries the dotted parameter path and `help_id`. Stable rule ids are added with the help registry work (P05.T12 step 4). **λ vs size class** is replaced by the global Kv range warning: one size class spans > 10× in Kv depending on the winding (gimbal 4108 ≈ 27 rpm/V vs drone 4108 ≈ 380 rpm/V), so a class-based λ bound would either never fire or reject real motors. **L ≥ M** does not apply: the model has no mutual-inductance parameter (only L_d, L_q). The **PSU limit** is not compared with the phase-current limit: a DC-side limit and a phase limit differ by the modulation index.
- **Consequences:** No rule engine to maintain; the help registry gets ids later.

## D-015 — One drivetrain module owns all mechanical states and the gearbox/load parameters · 2026-10-12 · accepted
- **Context:** P06 adds the gearbox, loads, friction and backlash. A live load-mass change must change ω (`conserve_momentum`, EQ-MECH-09), and the gear loss (EQ-MECH-03) needs the lossless acceleration of the whole drivetrain. A separate zero-state gearbox or load module could do neither, because only the owner of a state may change it and the per-stage outputs pass runs in module order.
- **Decision:** The rotor module (`motor.mechanical`) is the drivetrain. It holds θm/ωm (plus θL/ωL with backlash), the gearbox, the load, friction and the disturbance pulse, and reports their energy. `PlantModule::param_roots` lets it also own `gearbox.*` and `load.*`: those paths reach `set_param` unchanged, and paths under the module name arrive relative to it. Routing picks the longest matching root, as before.
- **Consequences:** CONVENTIONS §3 paths stay unchanged (`load.mass`, `gearbox.efficiency`). One module carries the whole EQ-MECH energy book, so the drivetrain code is split across `mech/{gearbox,load_arm,load_basic,friction,backlash}.rs` helpers.

## D-016 — Backlash without a stiffness uses 1 N·m/arcmin and ζ = 0.1 · 2026-10-12 · accepted
- **Context:** EQ-MECH-04 turns on the two-inertia model whenever `gearbox.backlash > 0`, but the gearbox presets (P04.T10) give backlash and no contact stiffness or damping, and the spec gives no default.
- **Decision:** Missing `gearbox.stiffness` defaults to 1 N·m/arcmin (≈ 3438 N·m/rad, the low end of the spec's small-planetary range), with a Warn issue (`gearbox.backlash_stiffness`). Missing `gearbox.damping` is d_c = 2ζ√(k_c·J_red) with ζ = 0.1 (mid-range of the spec's 0.05…0.2). The presets now carry estimated stiffnesses (presets/gearboxes/_sources.md). The build caps dt_max at 1/(20·√(k_c/J_red)) (numerics.md step table). New unit kind `torsional_stiffness` (N·m/rad, N·m/arcmin).
- **Consequences:** Scenes with a backlash gearbox (arm-servo) now run the two-inertia model, with a smaller step when the contact is stiff.

## D-017 — Modules react to discrete input changes through `after_discrete`; Off legs latch their diode · 2026-10-12 · accepted
- **Context:** Inverter leg states change at block ticks (PWM edges, commutation, commands). The motor must open a phase when its leg turns Off and reconnect it when the leg is driven again, which changes its own states (ψ_αβ ↔ ψ_yz). A state event cannot see such a step change, because the engine evaluates g again after the discrete change. Separately, an Off leg clamped by the instantaneous sign of i chatters at i ≈ 0 inside RK stages instead of crossing zero.
- **Decision:** `PlantModule::after_discrete(t, x_own, bus)` runs after blocks fire, after commands are applied, and once at engine start; the engine then refreshes outputs. The switching inverter latches the conducting diode when a leg turns Off (from the current direction), or when a floating terminal reaches a bound, and holds it until the motor reports the phase floating. The motor's pending-phase zero crossing is therefore a clean state event, and its diode-bound events (EQ-MOT-12) reconnect with the matching latch.
- **Consequences:** Leg-driven floating works in both directions without chatter. Modules that need discrete reactions use the hook and do not mutate state in `outputs`.

## D-018 — The DC bus is an explicit RK4 state with dt ≤ τ_bus/2, not τ_bus/20 · 2026-10-12 · accepted
- **Context:** EQ-NUM-04 limits the step to τ_bus/20. With the presets (20 mΩ PSU, 24 mΩ 6S pack, 470 µF) τ_bus ≈ 9–11 µs, so that would mean sub-µs steps, 10× slower than today. P07.T06 asked for an exact or semi-implicit bus update instead. Operator splitting would desynchronise the bus state from the RK4-integrated energy terms.
- **Decision:** keep V_bus (and the battery states) as ordinary RK4 states and cap dt at τ_bus/2. RK4 is stable on the real axis to |hλ| ≈ 2.8, and the energy terms are integrated by the same scheme, so the balance stays consistent. The supply tests run at dt ≈ τ_bus/2 and meet their tolerances (battery sag to 1e-6·OCV, residual < 1e-6).
- **Consequences:** the build applies `min(dt_max, τ_bus/2)` (P07.T12). Much stiffer sources (R_o·C ≲ 2 µs) would make the step small; that is a P19 optimisation (exponential update) if a preset needs it.

## D-019 — Supply parameters use flat `supply.*` paths · 2026-10-12 · accepted
- **Context:** supply.md and signals.md named the source parameters `supply.psu.*` / `supply.battery.*`, while the YAML schema, presets, validation and build use flat names under the `kind` tag (`supply.voltage`, `supply.current_limit`, `supply.r_out`, `supply.series`, `supply.r0_cell`, …). One ID namespace is required (CONVENTIONS §3).
- **Decision:** keep the flat paths (the source kind is the `kind` field, so the names do not collide); update the spec pages. Battery *signals* keep their own names (`supply.soc`, `supply.battery.v_rc`).
- **Consequences:** no YAML migration. `DcBus` exposes `bus.capacitance`, `supply.voltage`, `supply.current_limit`, `supply.r_out` live; a change that makes R·C smaller than at build time is refused (the D-018 step cap was computed from it): it needs a rebuild. Battery parameters change on a rebuild only.

## D-020 — Parameters use their scene paths (`controller.*`, `fidelity.*`); signals keep `ctrl.*` · 2026-10-16 · accepted
- **Context:** the scene YAML puts the controller under `controller:` and the tier/seed under `fidelity:` / `seed`, while CONVENTIONS §3 and signals.md listed the parameter roots as `ctrl.*` and `sim.*`. P09 adds the first live controller parameters (loop enable/bypass, gains), so one of the two had to give (one ID namespace).
- **Decision:** a parameter's path is its scene path: `controller.position.enabled`, `controller.current.bandwidth`, `fidelity.tier`, `seed` (as D-019 did for `supply.*`). Runtime **signals** keep the `ctrl.*` root (`ctrl.omega_ref`, `ctrl.foc.iq_ref`, …); `sim.*` stays the root of actions (`sim.play`) and engine signals. User's choice.
- **Consequences:** no YAML migration. CONVENTIONS §3 and signals.md list `controller` and `fidelity` as parameter roots. The interim skeleton parameter `ctrl.mode` disappears with the skeleton FOC (P09.T03).

## D-021 — V-CTRL-002 designs the velocity loop at ω_c/20, not ω_c/10 · 2026-10-19 · accepted
- **Context:** V-CTRL-002 checks the EQ-CTRL-04 PI design by its step overshoot. At the default ω_v = ω_c/10 the current loop's lag is not negligible, and the overshoot drifts several points from the design value, so the case would test the cascade's interaction instead of the design.
- **Decision:** the case uses ω_v = ω_c/20, where the current loop adds ≈ 0.8 points (13.5 % expected, atol 2 %). The default FOC design keeps ω_c/10.
- **Consequences:** the catalog entry states ω_c/20; the spec (control.md) explains the lag term. Found in P09.T13, recorded at the P09 gate (review m3).
