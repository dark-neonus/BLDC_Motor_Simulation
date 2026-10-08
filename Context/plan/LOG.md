# Work Log

> Append-only journal. Newest entry at the **bottom**. One entry per session or task batch (format: [CONVENTIONS.md §9](CONVENTIONS.md#9-commits--logs)).
> At session start, read the last 3 entries.

## 2026-10-08 — Planning (agent: Claude)
- Did: polished the idea (POLISHED_IDEA.md), selected tools (TOOLS_TO_USE.md), wrote the plan (PLAN.md + plan/), and recorded initial decisions D-001…D-008.
- Notes: The user approved subagents only for the independent reference model (P11.T01), phase-end reviews and graphify extraction. Ask before any other subagent use. Git: commits directly on `main`.
- Next: P00.T01

## 2026-10-08 — Plan dry-run review (agent: Claude + cold-review subagent)
- Did: a fresh-context subagent dry-ran the whole plan. 10 blockers and ~30 smaller issues were fixed:
  - per-RK-stage module coupling (P03.T02); state events (new P03.T15); energy accumulator moved to P03 (new P03.T16)
  - minimal rotor + interim drive path keeping the app green through P05–P09 (P05.T10/T11, P07.T12)
  - `validate-file` CLI; skeleton FOC params (V_bus 24 V, λ 0.03 Wb); bash vs fish + mise shims; pipx graphify; non-interactive scaffolding; `--no-tests=pass`
  - signal catalog (new P02.T14); URL deep links (new P13.T12); docs search; library-folder API/UI; assert exit codes; dependency-order fixes
  - decisions D-009 (docs embedded) and D-010 (ns rounding)
- Next: P00.T01

## 2026-10-08 — P00.T01–T03 (agent: Claude)
- Did:
  - T01: all prerequisites present; the plan was committed in 92df09c.
  - T02: mise 2026.10.4 installed. With the user's approval, added mise to `~/.config/fish/config.fish` (activate) and to `~/.bashrc` + `~/.profile` (shims on PATH).
  - T03: pinned the toolchain in `mise.toml` + `rust-toolchain.toml` (D-011); `mise install` took ~220 s.
- Verified: `bash -lc` resolves cargo/node/pnpm/just/uv/gh/lefthook/nextest/insta through the mise shims, with the expected versions.
- Notes: the system `tail` is not GNU (rejects `-3`); use `tail -n 3`. Inside the repo, `python` resolves to the mise 3.13.
- Next: P00.T04

## 2026-10-08 — P00.T04–T11 (agent: Claude)
- Did:
  - T04: skeleton dirs. The existing Python-template `.gitignore` was extended, and `lib/`/`lib64/` were anchored to the root (`web/src/lib` is not ignored).
  - T05: Cargo workspace (4 crates).
  - T06: Vite React TS (create-vite 9.2.1, run non-interactively); oxlint replaced by Biome 2.5; Vitest 5.
  - T07: uv project with Python 3.13.16.
  - T08: justfile.
  - T09: lefthook (verified that it rejects a mis-formatted .rs).
  - T10: CLAUDE.md + AGENTS.md symlink.
  - T11: graphify 0.9.80 (pipx, upgraded). Whole-repo graph at root `.` (510 nodes, 755 edges, 58 communities; 13 dangling edges from cross-chunk ids). The Context-only graph backup is in the session scratchpad. Post-commit/post-checkout hooks are installed and the merge driver is registered.
- Notes:
  - `graphify claude install` added a CLAUDE.md section **and** `.claude/settings.json` PreToolUse hooks (graph-query reminders before Bash/Grep/Read). Kept; reported to the user.
  - The doc extraction cost ~216k subagent tokens.
  - The pnpm store is at `/mnt/D/.pnpm-store` (separate filesystem from home).
- Verified: `just check` green; `graphify query` returns nodes; `graphify hook status` shows installed.
- Next: P00.T12

## 2026-10-09 — P00.T12–T14, Phase 00 gate (agent: Claude)
- Did:
  - T12: CI workflow (rust/web/python).
  - T13: the remote already existed (`origin` = github.com/dark-neonus/BLDC_Motor_Simulation). The user approved pushing at phase gates.
  - T14 gate: the fresh-clone test (`mise install && just setup && just check-fast`) passed.
- Phase-end review (subagent) found 3 majors + 10 minors. Fixed:
  - unanchored Python-template `.gitignore` dir rules (`env/`, `build/`, `var/`, `parts/` …) that ignored plausible source paths
  - CI installing every tool / compiling cargo tools (now per-job `install_args` + cargo-binstall 1.25.2 pinned + pnpm/uv caches + plan-check in CI)
  - `just setup` now runs `mise trust`; `just dev` server detection; first Rust test (`crates/bldc-sim/tests/cli.rs`)
  - explicit `#![forbid(unsafe_code)]`; `web/.gitignore` `logs` anchored; dead graphify `.gitattributes` rule removed
- Left to the user (agent config): `.claude/settings.json` hard-codes the graphify path; the CLAUDE.md graphify wording.
- Verified: CI run 37841687048 green (rust, web, python); local `just check-fast` green.
- Usage guard: `~/.local/share/claude-usage-guard/usage_guard.py` (calibrated 65% / 40 min).
- Next: P01.T01

## 2026-10-09 — P01.T01–T07, T09 (agent: Claude)
- Did:
  - T01: skeleton dq PMSM + RK4 with analytic tests.
  - T02: FOC (step 0→20 rad/s settles in 20 ms, 15.6 % overshoot, max i_q 3.9 A).
  - T03: runner thread (±10 % pacing, 10/10 runs).
  - T04: `run-scenario` CLI → parquet/csv/meta.
  - T05: axum REST + MessagePack WS at 30 Hz (sim/real 1.00 in debug).
  - T06: MCP via rmcp 3.5.1 streamable HTTP (`/mcp`) + `.mcp.json`.
  - T07: web skeleton (uPlot, Pixi 8, msgpack WS, zustand).
  - T09: Python V-SKEL-001/002 green.
- Notes:
  - sha2 0.11 digests lack `LowerHex` (manual hex).
  - Biome a11y requires ARIA roles: readouts use `role="status"`, panels use `<section>`. Add this to the P13.T04 convention.
  - The usage guard was recalibrated to the user's 80 % reading.
- Verified: `cargo nextest` 9/9, vitest 3/3, `just validate` 4/4; Vite proxy smoke test (page, REST, WS frames).
- Next: P01.T08 (docs), T10, T11, T12 gate.

## 2026-10-09 — P01.T08, T10–T12, Phase 01 gate (agent: Claude)
- Did:
  - T08: Docusaurus 3.10 (baseUrl `/docs/`, KaTeX bundled locally, Mermaid, local search, llms.txt). Heading anchors must use `{/* #id */}` (CONVENTIONS §8 updated).
  - T10: single binary with the UI and docs embedded (`--features embed`), `serve --open`, `just build`/`just run`.
  - T11: Playwright smoke test against the release binary + CI e2e job. Screenshot: `web/test-results/smoke.png` (rotor with 28 magnets, plot, readouts at 20 rad/s).
- Baseline (release, plant dt 5 µs, FOC 20 kHz, max time scale): **78× real time**.
- Phase-end review: 4 majors + 13 minors. Fixed:
  - Ctrl-C hang with an open MCP session (CancellationToken + test)
  - MCP structured results / enum action / real errors
  - MCP test proves "state follows" and that REST shares the instance
  - plot history wiped while paused (skip equal-t, sim-time window)
  - scenario dt validation; FOC test checks the speed stays in band to 1 s
  - atol+rtol tolerances (Rust + Python); runner test measures wall time at scale 1
  - command replies via channel (D-012); dotted wire keys (D-012); SPA fallback 404 for missing files and `/api`
  - KaTeX CSS version-matched and local; UI error surfacing
  - `SimHandle::spawn` returns `Result`; separate d/q current gains
- Not fixed: #15 (`thiserror` in the skeleton scenario) — the skeleton code is temporary and replaced in P03/P04.
- pnpm 12 blocks dependency build scripts: `allowBuilds` in pnpm-workspace.yaml (@swc/core allowed, core-js denied).
- Verified: `just check` green (11 Rust, 5 vitest, 4 pytest, 1 e2e, web + docs builds).
- Next: P02.T01

## 2026-10-09 — CI note (agent: Claude)
- The CI run for the phase-01 push (37847777743) failed: rust (`cargo fmt --all --check`) and python (`uv sync --frozen`). Both pass locally and in a fresh clone, and the previous run (37841687048) was green with the same config. The logs need GitHub auth (gh not logged in).
- Added `scripts/ci-run.sh`, which publishes the failing step's output as a public `::error` annotation. Verify it at the next push (P02 gate): `curl …/check-runs/<job-id>/annotations`. Also consider bumping the Node-20 actions (deprecation warning).

## 2026-10-09 — P02 physics spec + gate (agent: Claude)
- Did: wrote every page in `docs/docs/physics/` (references, conventions, motor, mechanical, thermal, inverter, supply, sensors, control, energy, numerics, signals, 61-case validation catalog). Key formulas were checked numerically before publishing (scratch scripts). D-013: flux-linkage states.
- Phase review (subagent): 2 blockers, 7 majors, 13 minors — all fixed except none deferred:
  - backlash contact sign; battery energy double-count; dead-time error with 2 diode intervals; τ63 as the current-loop metric
  - bus RC step limit; all-off rectification mode; rigid-drivetrain lumped friction + paths
  - MIT law without η; six-step H/O leg timing + averages; and the minors (symbols, g, N², Kt_dc, codes, names)
- Verified: `just check` green; docs build clean.
- Token budget: the docs graphify re-extraction is deferred and batched at the P04 gate (code graph is kept current by the post-commit hook).
- Next: P03.T01

## 2026-10-09 — P03.T01–T13, T15, T16 (agent: Claude)
- Did: engine core:
  - SimTime / SignalBus; DiscreteBlock / PlantModule with per-stage coupling
  - RK4 + exact RL step; diffsol cross-check
  - multi-rate scheduler; fidelity rules; command queue / live params / events
  - runner; snapshots; recorder; RNG; state events (bisection 0.1 ns, Zeno guard); energy accounting
  - skeleton ported onto the engine (energy-accounted); server, MCP and CLI on the engine runner
- Baseline: 16× real time (skeleton FOC, dt 5 µs, release).
- CI fixes: rustup components, `needs_binary` marker, validate job.
- Spec: energy residual normalisation now includes stored-energy magnitude (lossless systems).
- Phase-03 review: 7 majors + 12 minors, no blockers for the skeleton. Recorded as P03.T17–T20; the gate (T14) waits for them.
- `just check` currently fails only on `ruff format` of `validation/refmodel/` (written by the P11.T01 subagent, still running in the background).
- Stopped at ~93.5 % plan usage (user limit 97 %).
- Next: P03.T17
