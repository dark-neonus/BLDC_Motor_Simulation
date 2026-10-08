# Phase 19 — Hardening & release v1.0

> **Goal:** performance to target, a complete E2E and agent-journey coverage, resilience, packaging as a single binary, a green CI, and the final review → v1.0.
> **Depends on:** P18.
> **Read first:** LOG entries with benchmark numbers (P01.T12, P03.T13, P07.T10, P14.T08); POLISHED_IDEA §12 (scope).
> **Exit criteria:**
> - Performance targets met (or the shortfall explicitly accepted by the user).
> - `just check` green, including E2E.
> - A fresh-machine install works from the README.
> - The final review has no blockers.
> - Tag `v1.0.0` (user-approved).
>
> **Performance targets** (release build, dev machine, 12 cores):
> | Mode | Target sim/real |
> |---|---|
> | Ideal or Standard tier, averaged inverter, FOC 20 kHz | ≥ 5× |
> | Detailed tier, switching inverter 20 kHz | ≥ 0.2× (slow motion is the use case; ratio displayed honestly) |
>
> UI: ≥ 50 fps with viz (10k particles) + 4 plot panels on a real GPU (👤 confirmed by the user, PLAN §2 manual checks).

- [ ] **P19.T01** — Performance profiling & optimization
  - **Depends:** P18
  - **Do:**
    1. Profile with `cargo flamegraph` / `perf` on the benchmark scenarios.
    2. Typical wins:
       - avoid bus lookups by name in hot paths (resolve the IDs at build)
       - no allocation per event
       - batch event handling
       - inline small physics functions
       - reduce RK4 stage overhead
       - exponential integrator for RL in the averaged tiers
       - (optional) run auto-tune/batch on other threads
    3. Re-measure after each change. Physics outputs must stay identical (goldens).
  - **Files:** `crates/sim-core/**`
  - **Verify:** `just bench` meets the targets; `just validate` is unchanged.
  - **Done when:** The targets are met, or the shortfall is documented and the user decides.

- [ ] **P19.T02** — Full E2E user journeys
  - **Depends:** P19.T01
  - **Do:** Playwright suites:
    - first-run (empty user library)
    - load preset → run → change mass live → observe
    - six-step vs FOC comparison
    - duplicate & edit motor → save → reload app → still there
    - datasheet wizard
    - scenario run with asserts
    - fault injection + protection trip + reset
    - recording export
    - lesson run
    - layout save/restore
    - theme toggle
  - **Files:** `web/e2e/journeys/*.spec.ts`
  - **Verify:** `just e2e` green, 3 consecutive runs (flakiness check).
  - **Done when:** Green and stable.

- [ ] **P19.T03** — Agent journey with UI open (MCP + browser)
  - **Depends:** P19.T02
  - **Do:** A combined test: the Playwright browser is open on the app; a Node MCP client (`@modelcontextprotocol/sdk`) performs the P12.T08 journey plus `ui_command` highlights. Assert that the UI reflects each step (param highlights, plots updated, toasts). This is the automated proof of the "agent can work with the open app" requirement.
  - **Files:** `web/e2e/agent-journey.spec.ts`
  - **Verify:** `just e2e -g agent-journey` green.
  - **Done when:** Passing.

- [ ] **P19.T04** — Accessibility & keyboard pass
  - **Depends:** P19.T02
  - **Do:** Run `@axe-core/playwright` on every panel/layout and fix serious/critical issues. Ensure keyboard navigation for the main controls, focus rings in both themes, and `prefers-reduced-motion` (particles slowed or off).
  - **Files:** `web/e2e/a11y.spec.ts`
  - **Verify:** The axe run has no serious/critical violations.
  - **Done when:** Passing.

- [ ] **P19.T05** — Resilience & error handling
  - **Depends:** P19.T02
  - **Do:** Cases:
    - server restart → the UI reconnects and resyncs state
    - a corrupted YAML in the user library → listed with an error badge, the app keeps working
    - a Luau runtime error → block disabled + event, sim paused, not crashed
    - a numerical fault → auto-pause with a hint (P10.T05) surfaced in the UI
    - disk full / permission denied on save → a clear message
    - a very large recording → the memory cap warning works
  - **Files:** various, plus tests
  - **Verify:** An E2E or integration test per case.
  - **Done when:** Passing.

- [ ] **P19.T06** — Packaging & first-run experience
  - **Depends:** P19.T01
  - **Do:**
    - `just build` → `target/release/bldc-sim` containing the web UI, docs and builtin presets.
    - `bldc-sim` with no args = `serve --open`.
    - The first run creates the user dirs and prints the URL + MCP config snippet.
    - Optional `just install` → copies to `~/.local/bin`.
    - The README covers: install (from source with mise/just), run, connect an agent, where files live, troubleshooting link.
  - **Files:** `README.md`, `justfile`, `crates/bldc-sim/src/main.rs`
  - **Verify:** Fresh-clone test in `/tmp/claude-1000/…` following only the README → works.
  - **Done when:** Passing.

- [ ] **P19.T07** — CI pipeline complete
  - **Depends:** P19.T02
  - **Do:** Enable all jobs:
    - rust, web, python, validate, e2e (headless Chromium), docs build
    - schema/openapi/help/eq-id/docs-gen no-diff checks
    - a nightly job with the thorough hypothesis profile + benchmarks (record the trends as an artifact)

    Skip this task (`[-]`) if no remote exists (user decision from P00.T13).
  - **Files:** `.github/workflows/*.yml`
  - **Verify:** The CI run is green on `main`.
  - **Done when:** Green, or skipped with the reason.

- [ ] **P19.T08** — Final docs/validation refresh & changelog
  - **Depends:** P19.T05, P19.T06
  - **Do:** Regenerate all generated docs, screenshots and the validation report. Write `CHANGELOG.md` (v1.0.0 features, mirrored from POLISHED_IDEA §12 "In scope"). Update the "Future" list with anything deferred during implementation (`[-]` tasks).
  - **Files:** `CHANGELOG.md`, docs
  - **Verify:** `just check` green.
  - **Done when:** Complete.

- [ ] **P19.T09** — Final gate & 👤 USER release approval
  - **Depends:** P19.T01, P19.T02, P19.T03, P19.T04, P19.T05, P19.T06, P19.T07, P19.T08
  - **Do:**
    1. PLAN §8 checklist with a **whole-project** review: the review subagent diff range is `phase-00-done..HEAD`, focused on physics correctness, the agent-accessibility requirements, and POLISHED_IDEA coverage (a table mapping each feature → where it is implemented and tested).
    2. Present the summary to the user; on approval, `git tag v1.0.0`.
  - **Done when:** The user approves and the tag is created.
