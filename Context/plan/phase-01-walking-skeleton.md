# Phase 01 — Walking skeleton

> **Goal:** the thinnest end-to-end slice through **every** layer, to prove the architecture early:
> ideal PMSM + simple FOC (Rust) → CLI to Parquet → Python analytic test; server → WS stream → React UI with one plot and a spinning rotor; MCP tool → same sim; docs site with math and llms.txt.
> **Depends on:** P00.
> **Read first:** [TOOLS_TO_USE.md §1](../TOOLS_TO_USE.md), [DECISIONS D-001, D-004, D-006](DECISIONS.md), [CONVENTIONS §2–3](CONVENTIONS.md).
> **Exit criteria:**
> - `just dev` → open `localhost:5173` → press Play → the rotor spins and the speed plot rises to the target.
> - The MCP integration test calls `set_target_speed` and the state follows. Optional 👤 manual: the user confirms the UI follows when using the inspector.
> - `just validate` runs 2 analytic tests green.
> - `pnpm -C docs build` succeeds with KaTeX and `llms.txt`.
> - `just build` produces one binary that serves the UI.
>
> **Note:** Skeleton internals (hardcoded params, dq model) are **temporary** and get replaced in P03–P09. Keep the *interfaces* (CLI contract, WS framing, REST paths, MCP tool names) close to the final ones in P12, because those survive.

- [x] **P01.T01** — Ideal PMSM dq model + RK4 (temporary)
  - **Depends:** P00
  - **Do:**
    1. In `sim-core/src/skeleton/`, implement the dq model: `did/dt = (vd − R·id + ωe·Lq·iq)/Ld`, `diq/dt = (vq − R·iq − ωe·Ld·id − ωe·λ)/Lq`, `T = 1.5·p·λ·iq`, `J·dω/dt = T − B·ω`, `dθ/dt = ω`, with ωe = p·ω.
    2. Fixed-step RK4. Hardcoded illustrative 6020-class params (CONVENTIONS §5 units): p = 14, R = 1.0 Ω (phase), L = 2.5 mH (phase), λ = 0.03 Wb (→ Kt ≈ 0.63 N·m/A, Kv ≈ 13 rpm/V), J = 2.5e-4 kg·m², B = 1e-4 N·m·s/rad. Real values arrive with the presets in P04.
  - **Files:** `crates/sim-core/src/skeleton/{mod,model,rk4}.rs`
  - **Verify:** `cargo nextest run -p sim-core skeleton`
    - Locked rotor (ω forced 0), `vq` = 1 V step: `iq(t) = (V/R)(1−e^{−tR/L})` within 1e-6 relative at `dt = τ/100`.
    - Free spin at constant `vd = 0`, `vq = V`: the final ω matches the steady-state solution of the model equations with all derivatives set to 0. Solve that test-side with a scalar bisection on ω. With B → 0 it tends to ω ≈ V/(p·λ). Tolerance 1e-4 relative.
  - **Done when:** Both analytic tests pass.

- [x] **P01.T02** — Minimal FOC: current PI + velocity PI, voltage limit
  - **Depends:** P01.T01
  - **Do:**
    - Ideal DC bus V_bus = 24 V (ideal source).
    - PI current loops: id_ref = 0; iq_ref from a velocity PI, clamped to ±I_max = 5 A. Output vd, vq clamped to the circle |v| ≤ V_bus/√3. Anti-windup by clamping.
    - The controller runs every 50 µs (20 kHz) while the plant steps at 5 µs (zero-order hold).
    - Current gains by pole-zero cancellation: Kp = ωc·L, Ki = ωc·R, with ωc = 2π·1 kHz.
    - Velocity gains: ωv = ωc/10, Kp_v = ωv·J/Kt, Ki_v = Kp_v·ωv/4.
    - The no-load speed limit at 24 V is ≈ (24/√3)/(p·λ) ≈ 33 rad/s, so speed targets must stay below ~25 rad/s.
  - **Files:** `crates/sim-core/src/skeleton/foc.rs`
  - **Verify:** Test: a speed step 0 → 20 rad/s settles within 2 % in < 0.5 s, overshoot < 20 %, |i_q| never exceeds I_max.
  - **Done when:** The test passes and the step response is logged.

- [x] **P01.T03** — Sim runner thread with commands and state snapshots
  - **Depends:** P01.T02
  - **Do:**
    - A dedicated OS thread owns the model.
    - Commands arrive over a `crossbeam-channel` or `std::sync::mpsc`: `Play`, `Pause`, `Reset`, `SetTargetSpeed(f64)`, `SetTimeScale(f64)`.
    - Pace sim time to wall-clock × time_scale in chunks of ~1 ms wall time.
    - Publish the latest `StateSnapshot { t, omega, theta, id, iq, ia, ib, ic, sim_real_ratio }` via `arc-swap` or a `watch` channel, at most every 16 ms.
    - Never block on a slow consumer.
  - **Files:** `crates/sim-core/src/skeleton/runner.rs`
  - **Verify:** Test: play 0.2 s at scale 1 → sim time ≈ 0.2 s ± 10 %; pause stops time; reset zeroes the state.
  - **Done when:** The test passes; there are no busy-wait CPU spins while paused (use a blocking recv when paused).

- [x] **P01.T04** — CLI `run-scenario` (minimal YAML → CSV + Parquet)
  - **Depends:** P01.T02
  - **Do:**
    1. `bldc-sim run-scenario <file.yaml> --out <dir>`.
    2. Minimal YAML: `duration`, `dt`, `record_every`, `actions: [{t, set_target_speed}]`, plus optional `lock_rotor: true`, `vq_open_loop`.
    3. Write `signals.parquet` (arrow + parquet crates; columns `t, motor.omega, motor.i_q, …` using final signal names) and `signals.csv`, plus `meta.json` (version, scenario hash, seed).
    4. Use `serde-saphyr` for YAML.
  - **Files:** `crates/bldc-sim/src/cli/run_scenario.rs`, `validation/scenarios/skeleton_*.yaml`
  - **Verify:** `cargo run -p bldc-sim -- run-scenario validation/scenarios/skeleton_locked_rotor.yaml --out /tmp/claude-1000/out && ls /tmp/claude-1000/out`
  - **Done when:** Files are produced and the column names follow CONVENTIONS §3.

- [x] **P01.T05** — Server: REST + WebSocket stream (axum)
  - **Depends:** P01.T03
  - **Do:**
    1. `bldc-sim serve [--port 8787]`.
    2. Routes:
       - `GET /api/health`
       - `GET /api/state`
       - `POST /api/sim/play|pause|reset`
       - `POST /api/sim/time_scale {value}`
       - `POST /api/ctrl/target_speed {value}`
       - `GET /api/stream` (WS)
    3. WS: on connect, push MessagePack frames at 30 Hz: `{type:"state", ...snapshot}`.
    4. Use `tower-http` tracing. Graceful shutdown on Ctrl-C.
  - **Files:** `crates/sim-api/src/{lib,routes,ws}.rs`, `crates/bldc-sim/src/cli/serve.rs`
  - **Verify:** `cargo run -p bldc-sim -- serve &` then `curl -s localhost:8787/api/health` and `curl -X POST localhost:8787/api/sim/play`. Add an integration test using `reqwest` + `tokio-tungstenite` that receives ≥ 5 frames.
  - **Done when:** The integration test passes.

- [x] **P01.T06** — Minimal MCP server (rmcp, streamable HTTP at `/mcp`)
  - **Depends:** P01.T05
  - **Do:**
    1. Mount the rmcp streamable-HTTP service at `/mcp` in the same axum router.
    2. Tools:
       - `get_state` (returns the snapshot as JSON text plus a structured result)
       - `sim_control {action: play|pause|reset}`
       - `set_target_speed {rad_per_s}`

       Tools share the runner handle with REST (same instance, D-006).
    3. Write clear tool descriptions.
    4. Add `.mcp.json` at the repo root: `{"mcpServers":{"bldc-sim":{"type":"http","url":"http://localhost:8787/mcp"}}}`.
  - **Files:** `crates/sim-api/src/mcp.rs`, `.mcp.json`
  - **Verify:** A Rust integration test with the rmcp client: `list_tools` contains the 3 tools; `set_target_speed` then `get_state` shows the changed target. Manual: `npx @modelcontextprotocol/inspector` → connect to `http://localhost:8787/mcp`.
  - **Done when:** The test passes.
  - **If stuck:** Copy the structure of the rmcp `examples/servers` streamable-HTTP example at the locked version (TROUBLESHOOTING §2).

- [x] **P01.T07** — Web skeleton: WS client, play/pause, one uPlot, spinning Pixi rotor
  - **Depends:** P01.T05
  - **Do:**
    1. Vite proxy `/api` (with `ws: true`) and `/mcp` → 8787.
    2. `web/src/api/ws.ts`: decode MessagePack (`@msgpack/msgpack`), auto-reconnect.
    3. Zustand store for the latest state.
    4. Buttons Play/Pause/Reset and a target-speed input, each with a `data-agent-id` (`action:sim.play` …) and an `aria-label`. Add a numeric readout of the speed with `data-agent-id="signal:motor.omega"` (used by the E2E test).
    5. A uPlot chart of `motor.omega` and `motor.i_q` (rolling 10 s).
    6. A PixiJS canvas: a circle stator, a rotor with 2·p alternating colored magnet segments, rotated by `theta` each animation frame (render loop outside React state).
  - **Files:** `web/src/**`, `web/vite.config.ts`
  - **Verify:** `just dev`, open the browser: Play → the plot rises and the rotor spins. `pnpm -C web test --run` passes.
  - **Done when:** It works manually. Note a screenshot path in LOG (Playwright in T11).

- [x] **P01.T08** — Docs skeleton: Docusaurus + KaTeX + Mermaid + llms.txt
  - **Depends:** P00
  - **Do:**
    1. Scaffold non-interactively into the not-yet-existing dir: `timeout 600 pnpm create docusaurus@latest docs classic --typescript --skip-install < /dev/null`, then `pnpm install` at the root (it is a pnpm workspace member).
    2. Add `remark-math` + `rehype-katex` (+ KaTeX CSS), `@docusaurus/theme-mermaid`, the llms.txt plugin (`docusaurus-plugin-llms`) and a local search plugin (`@easyops-cn/docusaurus-search-local`).
    3. Create `docs/docs/physics/conventions.md` as a stub page with one equation and an `{#eq-conv-01}` anchor.
    4. Remove the template blog.
  - **Files:** `docs/**`
  - **Verify:** `pnpm -C docs build`; `build/llms.txt` exists; the equation renders in `pnpm -C docs serve`.
  - **Done when:** The build is clean with no broken links (`onBrokenLinks: 'throw'`, `onBrokenAnchors: 'throw'`).

- [x] **P01.T09** — First Python validation tests
  - **Depends:** P01.T04
  - **Do:**
    1. `validation/tests/conftest.py`: locate the binary via env `BLDC_SIM_BIN` (default `target/release/bldc-sim`), a `run_scenario(path) -> polars.DataFrame` helper, and a tmp dir.
    2. Tests:
       - `test_v_skel_001_locked_rotor_rl`: compare `i_q(t)` to the analytic RL step; max abs error < 0.5 % of the final value.
       - `test_v_skel_002_no_load_speed`: compare steady speed to the analytic value within 0.5 %.
    3. `just validate` builds release first, then runs pytest.
  - **Files:** `validation/tests/{conftest,test_skeleton}.py`, `justfile`
  - **Verify:** `just validate` → 2 passed (+ smoke).
  - **Done when:** Green.

- [ ] **P01.T10** — `just dev` / `just build` single binary with embedded UI
  - **Depends:** P01.T07
  - **Do:**
    - `just dev` runs `cargo run -p bldc-sim -- serve` and `pnpm -C web dev` concurrently, killing both on Ctrl-C (use a just recipe with `trap`, or `pnpm dlx concurrently`).
    - `just build` = `pnpm -C web build` + `pnpm -C docs build` → `cargo build --release -p bldc-sim`, with `sim-api` embedding `web/dist` (feature `embed-ui`) and `docs/build` at `/docs` (feature `embed-docs`, D-009) via `rust-embed`, so dev builds don't need either. Serve the SPA fallback to `index.html`.
    - `bldc-sim serve --open` opens the browser (`open` crate).
  - **Files:** `justfile`, `crates/sim-api/src/static_files.rs`
  - **Verify:** `just build && ./target/release/bldc-sim serve` → `localhost:8787` shows the UI and works without Vite; `localhost:8787/docs/llms.txt` is served.
  - **Done when:** The single binary works.

- [ ] **P01.T11** — Playwright smoke E2E
  - **Depends:** P01.T10
  - **Do:**
    1. Add Playwright to `web/`. The config starts the release binary as `webServer` on port 8787.
    2. Test: open the page, click `[data-agent-id="action:sim.play"]`, wait 1 s, read the displayed speed (`[data-agent-id="signal:motor.omega"]` text) and assert > 0.
    3. Save a screenshot to `test-results/`.
    4. Add the `just e2e` recipe and include it in `just check`.
  - **Files:** `web/playwright.config.ts`, `web/e2e/smoke.spec.ts`
  - **Verify:** `just e2e` passes headless.
  - **Done when:** Green. CI e2e job enabled (uncomment in ci.yml).
  - **If stuck:** TROUBLESHOOTING §3 (headless WebGL).

- [ ] **P01.T12** — Phase gate
  - **Depends:** P01.T01, P01.T02, P01.T03, P01.T04, P01.T05, P01.T06, P01.T07, P01.T08, P01.T09, P01.T10, P01.T11
  - **Do:** Run the PLAN.md §8 checklist. Measure and record in LOG the release-build sim/real ratio of the skeleton at dt = 5 µs (baseline for P19).
  - **Verify:** All exit criteria demonstrated.
  - **Done when:** Tagged `phase-01-done`.
