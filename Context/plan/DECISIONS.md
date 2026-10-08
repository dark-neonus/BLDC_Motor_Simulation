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
