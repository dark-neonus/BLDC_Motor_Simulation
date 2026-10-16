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

## 2026-10-09 — P11.T01 reference model delivered (subagent)
- `validation/refmodel/`: 47 self-tests green (~32 s), ruff clean. Isolation confirmed: the agent's file list contains only the physics docs, CONVENTIONS §5/§6 and pyproject; grep of the refmodel finds no reference to the crates.
- Scope: rigid drivetrain, linear magnetics, averaged inverter, ideal sensors. Not covered: saturation, backlash, switching inverter, phase-open fault, all-off rectification, sensors/observer/auto-tune.
- 20 spec questions are in `validation/refmodel/SPEC_QUESTIONS.md` (Q-01…Q-20). **P11.T01 stays [~] until they are answered in the spec** (task "Done when").
- Next: P03.T17, then resolve the SPEC_QUESTIONS.

## 2026-10-09 — P03.T17–T19 (agent: Claude)
- Did:
  - T17: NaN guard, input-only and finite SetSignal, validate-then-restore, saturating SimTime.
  - T18: snapshots now carry the energy book, dt, fidelity, FOC integrators and the motor's live `locked` param (the test caught the missing FOC state); variable-event re-poll + warnings; `set_fidelity` → `sim.dt_max`/`sim.tier` + tier tolerance; `energy.loss.<module>`; `sim_core::fixtures::dq_pmsm`.
  - T19: hot-path gating (x_save, outputs), precomputed term counts, persistent Zeno window, runner TimeScale floor 1e-4 + lag rebase + reset clears ratio + paused-spin test, flat capture buffer.
- Bench: 16× → **19.4×** real time.
- Deferred (by decision, not forgotten):
  - event direction mask and per-module Zeno: add if profiling shows bisection cost (P19.T01).
  - RNG in `StepCtx`: blocks own their `BlockRng` from `RngService` (simpler, and snapshotted per block).
  - exponential RL step as an integrator option: P05 decides whether the flux-state motor needs it.
- Next: P03.T20, then the P03 gate.

## 2026-10-09 — P03.T20 + Phase 03 gate (agent: Claude)
- T20: event edge-case tests (simultaneous crossings, substep end, t1), per-impact interval check (≤ 1 ns), External/jump/floor energy tests, xcheck through the engine, runner snapshot save/restore commands, plant and 3-rate event benches.
- Gate review (focused re-review of the T17–T20 commits): 7 findings, all fixed:
  - restore rollback on block/module data errors; dt_max validation (restore + set_fidelity)
  - Zeno window reset on restore; runner clears the error after a successful restore
  - saturating capture sizing; warning on silent repoll disable
  - dq model moved to `fixtures/dq_pmsm.rs` (cfg gate to be added when the skeleton is removed, P05.T11)
- Verified: `just check` green; 60 Rust tests.
- Next: P04.T01; also answer `validation/refmodel/SPEC_QUESTIONS.md` (P11.T01).

## 2026-10-09 — P11.T01 closed: spec questions answered (agent: Claude)
- All 20 refmodel SPEC_QUESTIONS were answered in the spec pages (signals, numerics, inverter, mechanical, thermal, control, energy); see the status line in `validation/refmodel/SPEC_QUESTIONS.md`.
- Decisions that differ from the refmodel's literal reading:
  - Q-05: switching loss only on switching legs
  - Q-08: no `energy.out`
  - Q-14: all setpoints load-side
  - Q-17: commands before the tick
  - Q-18: no gearbox friction
- The refmodel was updated for Q-05/14/17; 47/47 self-tests green.
- Follow-up for Rust (when the modules exist): `energy.loss.<term>` names (currently per module; switch to term names in P05/P10); setpoints load-side with N (P09).

## 2026-10-09 — P04.T01–T03 (agent: Claude)
- Did:
  - T01: `sim-model::units` (whitelisted table, Unicode normalisation, kind errors) + 56-case `schemas/units-fixture.yaml`.
  - T02: `Param` (short/long YAML forms, provenance).
  - T03: parameter types for motor, gearbox, loads, inverter, supply, bus/chopper, sensors, controllers, protection, fidelity (serde, deny_unknown_fields, tagged enums). JSON Schema derivation moved to P04.T07 (noted in the task).
- CI for the phase-03 push: **all 5 jobs green** (37903922496). This confirms the rustfmt-component and needs_binary fixes.
- Stopped at ~45 % usage (user limit for this window: 50 %).
- P04.T04 also done: star-of-slots layout + winding factor (6 published values incl. 12N14P 0.933, 9N8P 0.945), balance, validity, cogging LCM.
- Next: P04.T05 (constraint graph & apply_edit).

## 2026-10-09 — P04.T06 YAML IO
- `sim_model::io`: load/save/parse_yaml (serde-saphyr, line/col errors prefixed with file), schema modeline on save, `schema_version` with TooNew error and a `migrate()` hook (v1 identity).
- `bldc-sim validate-file`: motor files only for now (detected by `electrical:` key); other types join in T10/T11. Unknown-field errors list the expected fields (serde), no extra fuzzy matcher.

## 2026-10-09 — P04.T07 JSON Schemas
- schemars 1.2 derives on all params types; `Param` uses `#[schemars(with = "ParamRepr")]` → anyOf(number|string, {value, source, note}).
- `bldc-sim dump-schemas` writes motor/gearbox/load/inverter/supply/sensors/controller; scene/scenario join in T11. `just gen-schemas`, `just schemas-check` (in `just check`) and a CI step.
- `check-jsonschema` on presets is wired when presets exist (P04.T09). Sanity-checked: valid sample passes, unknown field rejected.

## 2026-10-09 — P04.T08 Library
- `sim_model::library::Library`: builtin (rust-embed over `presets/**/*.yaml`, or a dir in tests), user (`$XDG_DATA_HOME|~/.local/share/bldc-sim/library`, created per kind), extra folders from `~/.config/bldc-sim/config.yaml` `library_extra:` (ids `extraN:`; read-only).
- Ops: list(kind, filter), load (raw text), save_as, duplicate (sets `identity.name`, keeps the modeline; comments are not preserved in the copy), rename, delete. Names restricted to `[A-Za-z0-9._-]` (no path traversal); `_`-prefixed files (e.g. `_sources`) are hidden.

## 2026-10-09 — P04.T09 Motor presets
- 9 generic outrunner classes (2804 … 12020) in `presets/motors/`, generated from a class table (LL→phase halving, λ derived from Kv). Sources, ranges and the Kt·I check in `_sources.md`; 8010 and 12020 are interpolated/scaled (no exact-size datasheet found); no inrunner variants (not common in these classes).
- `estimation.md` now holds EQ-EST-01 (outrunner J), EQ-EST-02 (L from τ_e), EQ-EST-03 (thermal from mass/size); P04.T12 extends it.
- `just presets-check` (check-jsonschema + validate-file) in `just check`; a library test validates all embedded motor presets.

## 2026-10-09 — P04.T10 Component presets
- 26 presets: 2 inverters, 3 PSUs + 5 batteries (LiPo 4S/6S, Li-ion 12S, LiFePO4 4S/16S), 5 gearboxes, 4 loads, 6 sensors, 1 placeholder FOC controller. Sources in `presets/{sensors,supplies}/_sources.md`.
- Component files carry the schema modeline but **no `schema_version`** (types have no such field; `deny_unknown_fields`); a missing version is read as current. Revisit if a component format ever needs a migration.
- `validate-file` picks the type from the folder name; non-motor files get a typed parse only (no unit-kind checks yet; those come with each module's constraint rules). `presets-check` runs check-jsonschema per kind.

## 2026-10-09 — P04.T11 Scenes & scenarios
- `scene.rs`: `Component<T>` = `{preset, overrides{dotted.path: value}}` | inline T (untagged); `Scene::resolve(&Library)` → `ResolvedScene` (overrides go through the typed parse, so unknown fields are still rejected). `initial` holds theta/omega/temperature/ambient.
- `scenario.rs`: scene (ref or inline), `record`, `sample`, sequential `timeline` of externally tagged actions (`set`, `target`, `fault`, `disturbance`, `wait`, `assert{signal, op, value, tol, at?, window?}`, `snapshot`, `stop`).
- Presets: scenes gimbal-hold, arm-servo, six-step-demo; scenario arm-step. `validate-file` resolves scene/scenario references and runs the motor rules; schemas scene/scenario added.

## 2026-10-09 — P04.T12 Datasheet wizard
- `wizard::{start, WizardState::{questions, answer, accept_suggestions, finish}}`. Slots are a required input; questions only when relevant (poles meaning — auto-resolved by plausibility q ≥ 1/4; Kv definition; Kt basis; R/L LL vs phase; star/delta). Provenance: datasheet / derived / estimated, `Estimate{path, rule, confidence}`.
- New rules EQ-EST-04 (mass from envelope) and EQ-EST-05 (envelope from size class). Test uses the CubeMars AK10-9 V2 spec table: derived Kt 0.0827 vs listed 0.095 (−13 %, within 20 %), Kt·50 A = 4.13 N·m vs 38/9 = 4.22 N·m (−2 %).
- Lesson: Python heredocs writing LaTeX must use raw strings (`\r`, `\f`, `\a` became control chars).

## 2026-10-09 — P04.T13 Engine from scene, scenario asserts
- `sim_core::build::build_engine(&ResolvedScene) -> BuiltScene{engine, model, ratio, ctrl_dt}` maps the scene onto the skeleton dq motor + FOC (R, L, λ, p, J + reflected gearbox/load inertia, supply voltage incl. battery OCV at soc_init, FOC rate/bandwidth/current limit). Only `foc` builds before P09. `DqMotor` now exposes r/ld/lq/lambda/j/b as live params and saves them in snapshots.
- `SceneModel::edit` routes `motor.*` edits through `apply_edit` and emits `SetParam`s; rejected edits fail the run (exit 1).
- `sim_core::scenario_run::run`: sequential timeline → timed events; asserts at a time or over a window (checked at every sample); velocity targets with ramps (load side × N); `fault`/`disturbance` warn until P10/P05; position/torque targets error until P09.
- CLI `run-scenario`: both formats (full = has `timeline:`), `asserts.json`, pass/fail table, exit 0/1/2, `--no-asserts`. New preset scenario `gimbal-spin` (runs, passes).
- **Deferred:** the live server (sim-api) still runs the default skeleton engine without a scene, so live `SetParam` does not yet pass through `apply_edit`; wire `SceneModel::edit` in when the server loads scenes.

## 2026-10-09 — Session stop before P04.T14 (usage limit 60 %)
- `just check` (full gate) green after P04.T13. Remaining for the gate: phase-end review subagent (pre-approved), graphify doc re-extraction (deferred from P04 start), fix findings, tag `phase-04-done`, push.

## 2026-10-09 — P04 phase gate
- **Exit criteria evidence:**
  - All presets load and validate: `just presets-check` → check-jsonschema ok for 9 kinds; `validate-file presets/*/*.yaml` → 0 Reject, 0 Warn (35 files).
  - Every constraint rule has a test: `constraints::tests::every_rule_fires` (11 cases) + named tests for Kv derive, slot/pole, units/signs, locked derived fields, dedup; `components::tests::bad_components_are_rejected`; `scene::tests::scene_rules_fire`. Rule-list deviations in D-014.
  - `dump-schemas` writes schemas; regenerating gives no diff (`just schemas-check`, CI step); presets validate with `uvx check-jsonschema`.
  - `apply_edit` Kv→λ→Kt: Kv 100 rpm/V, p = 14 → λ 3.9381 mWb, Kt 82.699 mN·m/A (`kv_edit_derives_lambda_kt_ke`); overrides reach the engine (`motor_constant_overrides_reach_the_engine`).
  - Units fixture: `units::tests::shared_fixture_parses_exactly` (56 cases).
  - `just check` green: 94 Rust tests, web 5, Python 4 (+47 refmodel self-tests), e2e 1, docs + web builds.
- **Review:** 1 blocker, 8 major, 14 minor findings. All blocker/major fixed (commit e5fa0b1); minors fixed except field doc comments, quantity-valued asserts and live-server `apply_edit` routing → **P05.T12**; declarative rule ids → D-014 / P05.T12.
- **graphify:** incremental update with doc re-extraction (35 docs, 2 agents): 2417 nodes, 4329 edges, 296 communities.
- **Phase summary:** sim-model now holds units, provenance params, all component types, winding layout, constraint rules + apply_edit, YAML IO with schemas, the library (builtin/user/extra), 9 motor + 26 component presets with sources, scenes/scenarios, and the datasheet wizard; sim-core builds an engine from a scene and runs scenarios with asserts (CLI exit 0/1/2).

## 2026-10-09 — P05.T10 Rigid rotor
- `physics::mech::rotor::RotorRigid` (module `motor.mechanical`, so `motor.mechanical.j_rotor` routes to it): states θm, ωm; outputs motor.theta/omega/theta_e/omega_e; torque inputs split into internal (no power term) and external (`External` "load" term); viscous loss "friction"; D-007 inertia change keeps ω and books ½ΔJω² as an external jump; `locked` param.
- Prep: `energy.loss.<term>` per loss-term name (signals.md); `SetParam` routes to the module/block with the longest dotted-name prefix.

## 2026-10-09 — P05.T01/T02 abc electrical model, back-EMF shapes, torque
- `physics::motor::electrical::MotorElectrical` (module `motor`): ψα/ψβ states (D-013), linear EQ-MOT-02 inversion incl. harmonic magnet flux, EQ-MOT-01 derivatives, outputs per signals.md (ψ, i_abc/αβ/dq, e_abc, v_n, torque_em, p_cu); EQ-MOT-07 torque (dq term + harmonic term); energy: `electrical_in` Input, `copper` Loss, W_mag stored; L/λ edits keep flux and book ΔW_mag; `with_lambda_temperature` hook for P06 (EQ-THERM-03). Torque lives in `electrical.rs` (no separate torque.rs).
- `backemf::Shape`: sinusoidal / trapezoidal(w) / harmonics with k, Φ, k_h, Φ_h; tests: fundamental −sin θ for all shapes, Φ′ = k, zero mean, b1(120°) = 12/π², flat top 0.8225.
- Rotor builders `with_internal`, `with_locked` (the motor torque signal exists only after the motor is built).
- Tests (`tests/motor_abc.rs`): locked-rotor RL τ = L/R (rtol 1e-8), LL back-EMF peak = Ke·ω (rtol 1e-6), stall torque = Kt·i_q, trapezoidal energy closure < 1e-6.
- Line voltages are not output (not in signals.md); v_ab = v_aT − v_bT is available from the inverter signals.

## 2026-10-09 — P05.T03 abc ≡ dq, P05.T11 interim drive path
- `tests/abc_dq_equivalence.rs`: salient (L_d ≠ L_q) machine, smooth dq voltage program, abc engine (rotor + source doing inverse Park at its own θe + motor) vs the dq fixture as an engine module (`Pmsm::derivatives`, per-stage voltages; ZOH `step` cannot be matched exactly). All of i_d, i_q, ω, T within 1e-9 + 1e-6·|ref| every ms for 0.2 s.
- `physics::inverter::ideal::IdealVoltageSource` (INTERIM until P07): inverter.v_x = invClarke(ctrl.foc.v_α/β) + V_bus/2. The skeleton adapter now builds rotor + ideal source + abc motor + skeleton FOC (INTERIM until P09.T03); the FOC block outputs ctrl.foc.v_alpha/v_beta (inverse Park at the tick, ZOH). The dq plant is fixture-only; `PmsmParams` remains as the skeleton's parameter bag.
- Rotor: `j_load` (reflected inertia) separate from `j_rotor`; FOC gains designed on the total.
- `just validate` (V-SKEL), `just e2e` and the gimbal-spin scenario pass unchanged. Bench not re-run (perf check at the gate).

## 2026-10-09 — P05.T04 Cogging
- `physics::motor::cogging::Cogging` (module `motor.cogging`, stateless): reads motor.theta, writes motor.torque_cog (rotor internal input), U_cog as stored energy (EQ-MOT-08). Tests: period 2π/N_c, zero mean, T = −dU/dθ, unpowered cogging rotor energy residual < 1e-6 over 1 s.
- Wired via `SkeletonOptions::cogging`; `build::tier()` maps the scene tier; added when `FidelityConfig::preset(tier).enable_cogging` (Standard/Detailed) and the motor lists terms. N_c from `winding::cogging_periods`.
- Presets: one estimated harmonic at 2 % of Kt·I_peak each (`_sources.md`, [HendershotMiller2010] range 0.5–5 %).

## 2026-10-09 — P05.T05 Iron loss
- `physics::motor::iron_loss::IronLoss` (module `motor.magnetic` → params `motor.magnetic.k_hy/k_ed`): EQ-MOT-09 drag with tanh(ω/ω_ε), ω_ε = 0.01 rad/s; writes motor.torque_fe (new in signals.md) and motor.p_fe; `iron` Loss term = −T_fe·ω. Tests: spin-down vs analytic (a + bω) drag (rtol 1e-9), energy residual, dissipative through ω = 0.
- Wired via `SkeletonOptions::iron`, gated by `enable_iron_loss` and present coefficients (none of the presets has them yet: k_hy/k_ed need spin-down data). Coefficients are bare SI numbers (no unit kind for W·s/rad).

## 2026-10-09 — P05.T06 Saturation
- EQ-MOT-10 co-energy model inside `MotorElectrical` (`SatParams`, Detailed tier via `SkeletonOptions::saturation`): ψ_dq(i), W', W_mag = 3/2(ψ·i − W'); damped Newton from the linear solution (stateless → snapshot-safe), Hessian Jacobian.
- Debugging lesson: the first test curve (i_k = 10 A, c = 0.15) was not positive definite at reachable i_d, so Newton stalled and the energy residual was flat in dt (model error, not integrator). Added the missing validity rule in sim-model (`saturation_rules`: PD over ±3·i_k, L_∞ ≤ L_q, tested) and documented it in motor.md. A rustfmt-reflowed constant had silently kept the old curve: re-check edits after formatting.
- Tests: Newton round trip (deep saturation), ψ = ∇W', below knee ≈ linear within 1 %, above knee torque = 3/2·p·λ(1−h)·i_q, energy residual < 1e-6 for a pulse into saturation on a free rotor.

## 2026-10-09 — P05.T07 Open phase
- `MotorElectrical`: `motor.open_phase` param (−1 reconnect, 0/1/2 request) → state event on the phase current (g scaled by its sign at the request, so no false crossing) → single-path mode with state ψ_yz, dψ_yz/dt = v_yT − v_zT − 2R·i, i by 1-D Newton through the full magnetics (saturation/harmonics included); ΔW_mag at the switch booked as a jump; exact i_k = 0, i_z = −i_y outputs; v_n per EQ-MOT-12; reconnect rebuilds ψαβ from (i, θe). Snapshots carry open/pending.
- Test: 30 Hz drive, open c mid-run (sinusoidal and trapezoidal): i_c ≡ 0, i_a = −i_b, no NaN, residual < 1e-6, continuous reconnect.
- Not yet: diode-clamped zero-current mode and automatic reconnection when the floating terminal exceeds the rails (EQ-MOT-11/12 inverter part) — needs the real inverter (P07).

## 2026-10-09 — P05.T08 Energy terms & V-MOT sweep
- `tests/v_mot.rs`: V-MOT-001…013 each a named test with the catalog tolerances (005 = the P05.T03 equivalence test, moved to `tests/support/abc_dq.rs` and included). "Open circuit"/"unpowered" cases drive the terminals with the back-EMF so no current flows. Six-step ripple (012/013) uses ideal block currents through the new `MotorElectrical::from_currents` + `torque`.
- Energy terms: copper (`energy.loss.copper`), iron (`energy.loss.iron`), magnetic W_mag and U_cog (stored), electrical input. Air-gap power T_e·ω is an internal transfer between motor and rotor, so it has no term of its own (the balance closes without it).

## 2026-10-09 — P05.T12 (part 1/2): live scene + quantity asserts
- Server: `default_app_state()` loads `builtin:scenes/gimbal-hold` (derived + checked), keeps it in `AppState.scene` (Arc<Mutex>); the runner factory builds from it, so resets keep edits; falls back to the skeleton if the scene cannot load. `AppState::apply_param` → `SceneModel::edit` (apply_edit) → engine commands; REST `POST /api/param {path, value}` (422 + reason on reject) and MCP `set_param`. Only `motor.*` live so far. `SceneModel::edit` takes the ChangeSource.
- `Assert.value`/`tol` are `Param`s resolved with the signal's registered unit (`units::kind_of_unit`); test with rpm against rad/s.
- e2e still green with the gimbal scene as the live default.

## 2026-10-09 — P05.T12 (part 2/2): field docs, rule ids
- Doc comments on all 168 `sim-model::params` fields (units, EQ refs; `gearbox.ratio` = motor turns per output turn) → schema descriptions.
- `Issue.rule`: stable rule id (default = the path when it is the only rule there; named ids `value.unit/finite/positive/range`, `motor.kv_range`, `motor.saliency_ratio`, `motor.time_constant`, `motor.winding_valid`, `motor.saturation_*`, `motor.peak_above_continuous`, `motor.t_max_above_ref`, `mech.coulomb_le_static`, `motor.entered_constant_present`, `wizard.kt_matches_kv`). Test: two rules on one path keep distinct ids.

## 2026-10-09 — P05 gate prep: performance
- Bench `skeleton_foc_10ms_sim_dt5us`: abc model made it 2.44 ms (4.1× real time, was 19.4× on the dq plant). Exact-match solve cache (outputs → derivatives/powers of the same RK stage), ψ output from the state in normal mode, sinusoidal EMFs from one sin_cos: **1.62 ms (6.2× real time)**. No hard target in the spec; revisit in the perf phase.

## 2026-10-09 — P05 phase gate
- **Exit criteria evidence:**
  - All V-MOT cases pass with catalog tolerances: `tests/v_mot.rs` V-MOT-001…013 (13 tests); the reviewer checked each tolerance against validation-catalog.md.
  - abc ≡ dq: V-MOT-005 (`tests/support/abc_dq.rs`), salient machine, i_d/i_q/ω/T within 1e-9 + 1e-6·|ref| every ms for 0.2 s.
  - Skeleton dq plant removed from production: `fixtures` (dq model + its RK4) is `#[cfg(any(test, feature = "fixtures"))]`; integration tests enable the feature through a self dev-dependency; `PmsmParams` lives in `skeleton::params` as the FOC design bag.
  - `just check` green: 133 Rust tests, web 5, Python 4, e2e 1, docs + web builds. Bench `skeleton_foc_10ms_sim_dt5us` ≈ 6.2× real time (abc model; was 19.4× on the dq skeleton).
- **Review:** 4 major, 7 minor findings. Majors fixed:
  - dq plant gated out of production (cc38300);
  - scene `emf_shape` now reaches the motor; the test fails on the old code (07cad41);
  - MotorElectrical snapshots now hold last θe/λ, pending sign and jump, `restore` clears the cache, and the engine refreshes outputs on restore; a mid-fault bit-identical restore test fails on the old code (28e8798);
  - live edits commit to the scene only after the engine accepts them (`RunnerCommand::ApplyNow`, reply `rejected`); build-time motor fields (shape, cogging, saturation, winding) are refused with "reset"; k_hy/k_ed map to SetParam (0c47502).
- **Minors:**
  - Fixed: exact j_rotor in edits; an open request at zero current takes effect at once; doc fixes (EQ-MOT-10 wording, EQ-MOT-12 P07 status note, engine-only live params in signals.md, iron coefficients documented as bare SI) (26b976d).
  - λ(T) energy is already covered by energy.md ("neglected").
  - Solver diagnostics, V-MOT test strengthening and missing tests → **P06.T10**.
- **graphify:** AST update via the commit hook / `graphify update .`. Doc re-extraction is skipped this gate because only 3 spec pages changed (motor, signals, estimation); it is due at the P06 gate.
- **Phase summary:** the production plant is now the stationary-frame abc motor:
  - flux-linkage states;
  - sinusoidal, trapezoidal and harmonic EMF;
  - EQ-MOT-07 torque;
  - co-energy saturation with a PD validity rule;
  - open-phase single-path mode with an event-timed entry;
  - conservative cogging;
  - iron-loss drag.
  
  It runs with a rigid rotor and an ideal voltage source under the interim skeleton FOC. Energy closes for every term, and the server holds a live scene whose edits pass the constraint rules and the engine.

## 2026-10-12 — P06.T01 Rigid drivetrain: gearbox + load slot (D-015)
- D-015: the rotor module (`motor.mechanical`) is the drivetrain and owns `gearbox.*` / `load.*` through the new `PlantModule::param_roots` (engine routing: longest root; extra roots get the full path). `Engine::get_param` uses the same routing.
- `mech::gearbox::Gearbox` (N, η, J_gi, J_go; `j_eq`, EQ-MECH-03 `loss_torque` from the lossless acceleration, ω_ε = 1e-3). `mech::load::{Load, LoadTorques}`: torques split into gravity (stored U), loss and external; T01 has `Flywheel` (arm → its inertia only, INTERIM until T04).
- Rotor signals now include load.theta/omega, load.torque_gravity/torque_ext, gearbox.p_loss (display; the motor torque on the bus lags one stage in outputs, the energy book uses `powers`). Power terms: friction, gearbox, load (losses), external (renamed from "load").
- Live: gearbox.efficiency, gearbox.j_in/j_out (keep ω), load.* via the EQ-MECH-09 rule (keep ω, or J·ω with `on_change: conserve_momentum`), ΔE booked external; gearbox.ratio is build-only.
- Build: `SkeletonOptions{gearbox, load, conserve_momentum}` replace `extra_j`; FOC gains use `gearbox.j_eq`. arm-servo now sees its gearbox η.
- Tests: V-MECH-003 torque step through N = 9 (rtol 1e-9), gear loss = (1−η)|τ_in ω| driving and braking (rtol 1e-6), live load-inertia rules both ways (rtol 1e-9, residual < 1e-9), ratio edit rejected.

## 2026-10-12 — P06.T04 Arm + mass gravity load
- `mech::load_arm::Arm` (EQ-MECH-06): M_eff, J_load, T_g = −g·M_eff·sin θ_L, U_g reported as stored energy (gravity has no power term; its work is −dU/dt). `Load::Arm`; live `load.arm_length/arm_mass/mass/distance` through the EQ-MECH-09 rule (ΔU included in the booked jump).
- Scene: optional `load.mount_angle` (initial θ_L; the spec's mounting offset) → rotor θ0 = N·angle. Schemas regenerated. Build maps the arm fully (replaces the T01 interim inertia-only mapping).
- `tests/v_mech.rs`: V-MECH-001 period 0.77961 s (spec value reproduced to 1e-5, simulated rtol 5e-4), V-MECH-002 150° swing for 15 s: residual < 1e-6 and stored energy constant, V-MECH-009 keep_speed / conserve_momentum through N = 9 (rtol 1e-9) with ΔE booked.

## 2026-10-12 — P06.T05 Basic loads + disturbance pulse
- `mech::load_basic` (EQ-MECH-07): constant (external work), brake −T₀·tanh(ω_L/1e-3) and viscous −c·ω_L (losses); `Load::{Constant, Brake, Viscous}` with live `load.torque` / `load.coefficient`; build maps them.
- Disturbance (EQ-MECH-08): the drivetrain gets a 3rd state, the pulse's remaining time (dτ/dt = −1 while running), and a state event ends the pulse exactly. Params `load.disturbance.duration` then `load.disturbance.torque`; signal `load.disturbance`; work booked external. `EngineCommand::Disturbance{torque, duration}` and `Impulse{h}` (Δt = max(1 ms, 5·dt_max)). Scenario `disturbance:` actions now drive it (was a warning).
- Tests: V-MECH-008 impulse through N = 9: Δω_L = H/J_tot,L rtol 1e-6, residual < 1e-9; brake/viscous/constant analytic speeds with energy closing.

## 2026-10-12 — P06.T03 Stick–slip friction (EQ-MECH-05)
- `mech::friction`: `Friction{t_s, t_c, b}`, rigid lumping T_s,m + T_s,L/N, T_c,m + T_c,L/N, b_m + b_L/N²; `Mode::{Slip{dir}, Stick}`; `eval` gives the friction torque and whether the shaft is held (stick with |T_a| ≤ T_s → T_f = −T_a; stick past breakaway → slips toward T_a).
- Drivetrain events (only when T_s or T_c > 0, so pure viscous runs stay event-free): slip → ω crosses zero (stick, or reverse when |T_a| > T_s); stick → |T_a| crosses T_s; stick → |ω| > 1e-12 catches a step that starts past breakaway (the engine re-evaluates g at each advance, so an already-negative g would never cross). On sticking, the bisection-sized ½Jω² leaves through the external jump (no loss-jump API; magnitude ~1e-20 J). T_gl = 0 while held.
- Live: motor.mechanical.friction.{static_torque, coulomb, viscous}, load.friction.*; locking/unlocking resets to stick. Build maps the motor's friction (viscous stays in `PmsmParams::b`) and the arm's load friction.
- Tests: V-MECH-006 0.98·T_s → θ and ω bit-identical over 1 s; V-MECH-007 1.2·T_s breakaway (speed rtol 1e-3), Coulomb + viscous coast-down vs the analytic law (rtol 1e-3), re-sticks and stays bit-identical, residual < 1e-3.

## 2026-10-12 — P06.T02 Backlash two-inertia model (EQ-MECH-04)
- `mech::backlash::Backlash{width, k, d}`: push-only dead-zone contact τ_c(δ, δ̇), spring energy ½k(|δ|−b/2)², `dt_limit` = 1/(20√(k/J_red)), `damping_for(k, ζ, J_red)`.
- Drivetrain rewritten around a `Dyn` evaluation: rigid (as before) or backlash (`with_backlash`, chosen at build): states θm, ωm, τ_dist, θ_L, ω_L; motor side (J_m + J_gi) with T_gl from τ_in = τ_c/N, load side (J_go + J_load); each side keeps its own stick–slip element (rigid: lumped). Six event slots: pulse end, motor friction ×2, load friction ×2, contact |δ| − b/2. Energy: stored = both kinetic + U_g + contact spring; new Loss term `backlash` = τ_c·δ̇ − d/dt(spring) ≥ 0. EQ-MECH-09 with backlash conserves the load body's momentum. Signals gearbox.delta / gearbox.torque_contact.
- D-016: missing stiffness → 1 N·m/arcmin with a Warn (`gearbox.backlash_stiffness`), missing damping → ζ = 0.1; build caps dt_max by the contact limit. New unit kind `torsional_stiffness` (N·m/rad, N·m/arcmin; units fixture +2). Gearbox presets get estimated stiffness (presets/gearboxes/_sources.md, new). mechanical.md notes the defaults.
- Tests: V-MECH-005 PD-driven slow reversal through N = 9, b = 0.01 rad: lost motion at the stroke mid-points = b + 2T_c/k within 1e-4 rad (the peak δ includes a ~2e-4 rad re-engagement impact, so peaks are not used), no NaN, backlash loss ≥ 0, residual < 1e-6; sim-model warns for backlash without stiffness.
- `.gitignore`: `.VSCodeCounter/` (editor extension output; Biome follows .gitignore and failed on it).

## 2026-10-12 — P06.T06 Thermal network + R(T), λ(T)
- `physics::thermal::Thermal` (module `motor.thermal`, EQ-THERM-01): states T_w, T_s, T_h [K]; heat from `motor.p_cu` → winding, `motor.p_fe` → stator, new `motor.p_fric` (drivetrain friction power, motor + load side) → housing; R_ha with fan cooling κ (EQ-THERM-04); `thermal.t_magnet` = housing (outrunner) or stator (inrunner). Live r_*/c_*/kappa. No main-balance terms: energy.md tracks heat separately (thermal.md wording fixed: booking ΣC·T too would double-count the losses).
- Overheat warnings: state events at T_max − 10 K and T_max (rising) → `EngineEvent::Warning`, through a new `PlantModule::take_warnings` hook collected after events.
- Motor: `with_r_temperature` (R(T), EQ-THERM-02, α_Cu 0.00393) used in dψ/dt, open-phase law, p_cu and the copper power term; `LambdaTemperature.t_start` so the initial flux uses λ(T_start) (otherwise a 25 °C start gave ~0.07 A at t = 0, caught by the server rejection test).
- Build: thermal when the tier enables it and the motor has `thermal`; ambient from `initial.ambient` (default 25 °C), start from `initial.temperature`, t_ref default 20 °C, α_Br default −0.0012, κ bare SI. Thermal precedes the motor in the plant (temperatures feed R/λ in the same stage).
- Tests (`tests/v_therm.rs`): V-THERM-001 network vs the exact solution e^{At} (scaling-and-squaring) at 5…3000 s, rtol 1e-6; V-THERM-003 R(100 °C)/R(20 °C) = 1.3144 via the steady locked-rotor current, rtol 1e-9; both warnings fire once.

## 2026-10-12 — P06.T07 Continuous/peak current helpers + thermal rating rule
- `sim_model::thermal_limits`: `ThermalModel::from_motor` (r_phase at t_ref + thermal block), `continuous_current(m, T_a)` = EQ-THERM-05 closed form (copper only), `peak_time(m, T_a, Î)` = EQ-THERM-06 by RK4 on the network with R(T) and a 1 µs bisection of the crossing step (`None` at or below Î_cont).
- Rule `motor.ratings_thermal` (Warn, `constraints/thermal.rs`): rated continuous current > 1.01·Î_cont at 25 °C. No preset trips it.
- Tests: closed form + its defining property (steady T_w = T_max exactly); peak time decreasing in current, above the quick bound, and within 5 % of the exact adiabatic R(T) solution at 20·Î_cont (the quick bound with R(T_max) is ~15 % short at high current because R rises during heating: test expectation corrected, not the code); rule fires / stays quiet; sim-core `tests/thermal_limits.rs` V-THERM-002: the engine network held at Î_cont settles at T_max ± 1 K (6020 preset network).
- `just check` green (155 Rust tests).

## 2026-10-12 — P06.T08 V-MECH/V-THERM sweep + hold-heats-motor
- All V-MECH (001–009) and V-THERM (001–004) cases have tests: `tests/v_mech.rs`, rotor unit tests renamed `v_mech_003_*` / `v_mech_004_*`, `tests/v_therm.rs`, `tests/thermal_limits.rs` (002), `validation/tests/test_thermal.py` (004). `cargo nextest run -p sim-core v_mech v_therm` → 12 tests.
- Interim torque mode for the skeleton FOC (INTERIM until P09): block param `ctrl.mode` (0 velocity, 1 torque) and input `ctrl.torque_ref` (load side) → i_q = T/(N·K_t), speed loop off. Scenario `target: {kind: torque}` switches the mode (no ramp); velocity targets switch back. Position targets still error (P09).
- `presets/scenarios/hold-heats-motor.yaml`: 6020 motor holds 0.75 kg at 20 cm horizontal (θ_L = 90°, gravity torque 1.52 N·m) for 30 s, load friction holds it as K_t drops with magnet temperature. CLI: 5/5 asserts pass, T_w 25 → 33.9 °C, ω exactly 0 after the first sag (10.6 s wall, release). Python V-THERM-004: T_w(t) matches the exact linear network driven by the mean copper loss within 5 % of the rise at 5–30 s.
- Removed my T04 `load.mount_angle`: the scene already has `initial.theta` / `initial.omega`, which the build ignored; it now uses them (θ_L, ω_L load side). Presets fixed to EQ-CONV-05 (0 = hanging down): arm-servo starts at 0°, arm-step targets 90° (was −90°/0°).
- `just check` green apart from schemas-check before commit (regenerated schema).

## 2026-10-12 — P06.T10 P05 review carry-overs
1. Flux→current solves: `Magnetics::currents` returns a convergence flag (residual ≤ 1e-9·flux **and** a positive-definite Jacobian at the solution: an exact one-shot solve at i_q = 0 can sit in the indefinite region, found while writing the test), step-size stop at the round-off floor; the open-phase 1-D solve checks its final residual. Failures set a `Cell` flag in the `&self` passes; the motor reports one warning per run through `take_warnings`, which the engine now also collects after every `advance` (not only after events). Test: i_d → +45 A on the motor.md non-PD curve (i_k 10 A, c 0.15) → exactly one warning; the valid curve → none.
2. λ(T) energy: the 30 s hold-heats run keeps |energy.residual| ≤ 1.2e-11 (recorded in the scenario, asserted < 1e-6 in V-THERM-004). energy.md's "neglected" stands; nothing to book.
3. V-MOT: 006 through `apply_edit` (production Kv → λ, Kt); 007 period 2π/N_c and zero mean (atol 1e-9); 008 second case in the tanh region near ω = 0; 009 locked at i_q → 25 A > 2·i_k with torque compared; 011 a–b path after an at-rest open: i(t) = (1/R)(1 − e^{−tR/L}) rtol 1e-6 and Clarke-derived i_c = 0.
4. New tests: triplen harmonics (3rd, 9th) give torque exactly K_t·I with pure q current (rtol 1e-9) while a 5th ripples; harmonic-shape energy closes (< 1e-6); live l_d / λ_m edits book ΔW_mag (residual < 1e-6); saturation in open-phase mode (i_c = 0, residual < 1e-6, no warning); production `build_engine` with iron loss from scene k_hy/k_ed (loss booked, residual < 1e-3). Snapshot-during-fault was already covered at the P05 gate.

## 2026-10-12 — P06 phase gate
- **Exit criteria evidence:**
  - All V-MECH and V-THERM cases pass with catalog tolerances: `cargo nextest run -p sim-core v_mech v_therm` (V-MECH-001…009 incl. 003/004 in rotor.rs, V-THERM-001/003 + warnings; V-THERM-002 in `tests/thermal_limits.rs`; V-THERM-004 in `validation/tests/test_thermal.py`). The Python halves of the "both" cases V-MECH-001 / V-THERM-002 belong to the P11 reference-model work.
  - Holding torque heats the winding: `presets/scenarios/hold-heats-motor.yaml` passes 5/5 asserts through the CLI (T_w 25 → 33.9 °C over 30 s, ω_L exactly 0 after the first ~0.004 rad sag); V-THERM-004 matches T_w(t) to the exact linear network driven by the mean copper loss within 5 % of the rise; |energy.residual| ≤ 1.2e-11 over the heating run.
  - `just check` green: 167 Rust tests, web 5, Python 5, e2e 1, docs + web builds, schemas/presets checks. Bench `skeleton_foc_10ms_sim_dt5us` 1.99 ms (5.0× real time) vs 1.82 ms at phase-05-done on the same machine (+9 % for the drivetrain); a 2.5 ms reading was contention with the reviewer's concurrent cargo runs (re-measured; per-commit bisect T01 1.70 → T05 1.87 → T06 2.06 → T10 1.98 ms). `perf` needs a lower `perf_event_paranoid` (sudo), so profiling was by bisection only.
- **Review:** 1 blocker, 4 major, 10 minor. Fixed (72ffa49):
  - B1 backlash with no load-side inertia (presets have no `j_out`) divided by zero: builds rigid when J_go + J_load ≤ 0, and live edits that would make it 0 are rejected (with state restored);
  - M1 `initial.theta` now sets θe0 = p·N·θ_L0 (the motor started 180° off, i_d = −2λ/L_d, in hold-heats-motor);
  - M2 thermal live params and the motor's `warned` flag in snapshots;
  - M4 V-MECH-005 also runs at the build's contact dt cap;
  - minors: contact-event comment, locked start ω_L0, `thermal.t_magnet` as Output, the stick-event energy now described as it is booked (mechanical.md), disturbance params in signals.md, `peak_time` cap documented.
  - M3 was a misreading: `energy.residual` is already normalised by the energy throughput, so `< 1e-3` is the catalog's rtol 1e-3.
  - The remaining minors (τ_in load-friction share, snapshot versioning, `get_param` gaps, bumpless torque-mode switch, V-THERM-002 through the motor, λ(T) torque-drop test) → **P07.T13**.
- **graphify:** AST via the commit hook; doc re-extraction of the 55 changed docs/presets with 3 Haiku agents (per the user's cheaper-model guidance), merged incrementally: 2899 nodes, 5611 edges, 235 communities. One extraction agent flagged that `presets/supplies/liion-12s.yaml` reuses the LiPo OCV table → checked in P07 (supply models).
- **Phase summary:** the mechanical plant is one drivetrain module (D-015): rigid gearbox with direction-independent meshing loss, two-inertia backlash with push-only contact (D-016 defaults), hybrid stick–slip friction per side, arm/constant/brake/viscous/flywheel loads with EQ-MECH-09 live changes, and an event-timed disturbance pulse (`Disturbance`/`Impulse` commands). The three-node thermal network feeds R(T) and λ(T) back into the motor, raises overheat warnings and drives the sim-model rating helpers (continuous current, peak time, ratings rule). The interim FOC gained a torque mode for the hold-heats scenario.

## 2026-10-12 — P07.T01 Switching inverter legs + automatic floating (D-017)
- `physics::inverter::leg`: `LegState` (bus `inverter.leg_x`: +1 H, −1 L, 0 O), `terminal_voltage` (EQ-INV-01: R_on drop both ways, Off → diode by current sign, None = floating), `high_side_conducts` (EQ-INV-08). `SwitchingInverter` (module `inverter`, no states): writes `inverter.v_a..c` and `inverter.i_dc`; an Off leg uses a **latched** diode (from the current direction when it turned Off, or the bound a floating terminal reached) until the motor reports the phase floating, then v_n + e_x clamped to [−V_f, V_bus + V_f]. Without the latch the clamp flipped with sign(i) inside RK stages and the current chattered at +1e-3 A instead of crossing zero (found in the test).
- Engine: `PlantModule::after_discrete` hook (after blocks fire, after commands, once at start) + outputs refresh (D-017).
- Motor: `with_legs(LegInputs)` → Off leg requests an open (pending zero crossing, or at once at zero current), a driven leg reconnects; two diode-bound state events (v_kT = v_n + e_k from same-pass bus values reaching V_bus + V_f or −V_f) reconnect with the pending sign the diode implies (EQ-MOT-12). `motor.open_phase` is now a physical-disconnect fault (`fault_open`, never auto-reconnects; saved in snapshots). Two+ Off legs: first one floats, one warning (zero-current mode → P10). motor.md implementation note updated.
- Tests (`tests/inverter_leg.rs`): R_on drops and the diode rule (unit); A/B/C driven steady current V_bus/(1.5(R+R_on)) rtol 1e-6, A Off → v_a = −V_f, current reaches zero and the phase floats with i_a ≡ 0, A High → reconnects; over-speed with A Off (EMF 42 V vs 12 V bus): the terminal stays within the diode bounds, the high diode conducts (i_a < 0), the phase floats and reconnects.

## 2026-10-12 — P07.T04 Modulators
- `physics::inverter::modulation`: `spwm` (EQ-INV-04), `svpwm` min–max injection (EQ-INV-05), `clamp_vector` / `svpwm_ab` with the EQ-INV-06 overmodulation clamp and a saturation flag, `six_step` legs per EQ-INV-07 (sector table, optional advance). Done before T02/T03 (pure functions, no dependency in practice).
- Tests: SVPWM = classic sector-based reference at 36 angles (< 1e-12) and the inverter.md worked example (0.8519/0.3614/0.1481); linear limits (SVPWM duty span reaches 1 at V_bus/√3, clamp beyond; SPWM duty 1 at V_bus/2); six-step table at mid-sector angles, advance, and High/Low = max/min back-EMF at 360 angles.

## 2026-10-12 — P07.T02 PWM generator
- `physics::inverter::pwm::Pwm` (block `inverter.pwm`, variable events, priority INVERTER): center-aligned carrier; duties `inverter.duty_x` latched at each period start t_k (shadow registers; t_k = middle of the all-Low zero vector = `pwm.center`, counted in `inverter.pwm_count` for the P08 ADC); each turn-on goes through Off for the dead time; pulse dropping within t_d of 0 or 1 (and the dead time also applies when a period starts full-High from Low). Edge times are integer ns.
- Tests (engine with the block alone, sampled with `step_until(t + 1 ns)` = state after the events at t): d = 0.6 → Off at 10 µs, High at 10.5 µs, Off at 40 µs, Low at 40.5 µs; d = 0.2 → [20, 30] µs; d = 0 → no pulse; no direct H↔L switch anywhere; a duty changed mid-period applies from the next period; d = 0.995 stays High with the entry dead time.
- Live f_pwm / dead_time edits are not routed yet (block id `inverter.pwm` vs the CONVENTIONS paths `inverter.f_pwm` / `inverter.dead_time`); build-time values for now.

## 2026-10-12 — P07.T03 Averaged inverter
- `SwitchingInverter::averaged(dead_frac, i_eps)`: the same module in averaged mode (EQ-INV-03): legs that are not Off give d·V_bus − R_on·i − σ(i)·(t_d/T)(V_bus + 2V_f − 2R_on|i|), σ = tanh(i/i_ε); Off legs keep the latched-diode / floating rule of the switching mode, so floating behaves the same in both. i_dc averaged per EQ-INV-08 ((d − σ t_d/T)·i). `duty_ids` registers or reuses `inverter.duty_x` (shared with the PWM block).
- Not yet: the averaged six-step H/O leg (EQ-INV-07 blend with w = (1+σ)/2) — it needs a per-leg six-step mode signal and comes with the six-step controller (P09).
- Test V-INV-avg (`tests/inverter_avg.rs`): locked rotor, d = (0.7, 0.4, 0.4), 24 V, 20 kHz, t_d = 0.5 µs: the switching model's period-averaged phase currents (200 samples over one period after 16 τ) equal the averaged model's within 1 % of the 4.6 A operating current; the dead-time error is visible (current < 0.97 × the no-dead-time value).

## 2026-10-12 — P07.T05 Inverter losses + DC-link current
- Inverter (in `leg.rs`, not a separate losses.rs: the loss is one line on top of the terminal voltages): `with_switching_loss(t_r + t_f, f_pwm)` → `inverter.i_sw` = f_pwm·Σ ½|i_x|(t_r + t_f) over legs that are not Off (EQ-INV-09, both modes); `inverter.p_loss` = V_bus(i_dc + i_sw) − Σ v_x i_x (conduction + diodes + switching, exact by EQ-INV-01/08). Power terms: Input `dc_link` = V_bus(i_dc + i_sw) (until a bus module takes it: `without_dc_input`) and Loss `inverter`.
- Motor `with_internal_terminals()`: the terminal power is an internal flow, so the motor books only copper (no double input).
- Tests: residual < 1e-6 with switching and averaged inverters (locked rotor, 20 ms), `energy.loss.inverter` > 0; i_sw doubles exactly with f_pwm.

## 2026-10-12 — P07.T06/T07/T08 (+T09 chopper module) DC bus, PSU, battery, chopper
- `physics::supply::bus::DcBus` (module `bus`, extra root `supply`): C·dV/dt = i_src − Σ load − i_ch (EQ-SUP-01); `Source::{Ideal (no state, sources/sinks), Psu (clamp((V_set − V)/R_o, 0, I_lim), mode CV/CC/blocking, kinks as state events), Battery (Thevenin 1-RC + SoC, EQ-SUP-04)}`; optional `Chopper` (V/R_b, on above V_on / off below V_off, state events). Signals per signals.md (bus.v, supply.i, supply.mode, bus.i_chopper, bus.chopper_on, bus.p_brake, supply.soc, supply.v_terminal, supply.battery.v_rc). Energy: ideal/PSU input (PSU as EMF behind R_o with loss R_o i²), battery has no input but stores E_chem(SoC) + ½C₁v₁², losses R₀i² + v₁²/R₁, brake V²/R_b; ½CV² stored. Load currents attached with `with_load` (inverter i_dc, i_sw).
- `supply::battery::Battery`: pack from cells (s, p rules), OCV interpolation, exact E_chem for the piecewise-linear table.
- D-018: bus as an explicit state with dt ≤ τ_bus/2 instead of τ/20.
- Tests (`tests/supply.rs`, scripted i_dc): linear discharge (1e-9), PSU CC at 5 A with the sag rate 3 A/C and blocking with the rise rate 2 A/C (rtol 1e-6), battery I·R₀ sag after the bus RC (1e-6·OCV), RC relaxation at t = τ/10 (rtol 1e-3) and SoC = ∫I dt/3600Q, chopper cycles inside [V_off, V_on] with brake energy booked; residual < 1e-6 in each. The motor regen spin-down with the chopper is part of the T10 sweep.

## 2026-10-12 — P07.T12 Interim drive through the real inverter & supply
- `skeleton::adapter::Drive::{Ideal (default for the skeleton and V-SKEL), Inverter(InverterOptions)}`: the FOC block writes SVPWM duties (EQ-INV-05/06, on the measured `bus.v`) after its inverse Park; averaged inverter (legs held driven) or switching (PWM block + latched-diode legs); `DcBus` with the scene's source and chopper first in the plant, the inverter after the motor; the motor follows the legs with internal terminals and the inverter books only its loss (`without_dc_input`).
- Build: `fidelity.inverter_mode` (ideal / averaged / switching, else the tier's: Ideal and Standard averaged, Detailed switching) → `Drive`; device values from the inverter preset; source from `scene.supply` (ideal / PSU / battery from cells); `bus.capacitance` default 470 µF (supply.md's example; no scene sets it yet); chopper from `scene.bus.chopper`. dt cap: τ_bus/2 (D-018) and T_pwm/200 in switching mode (EQ-NUM step table).
- All scenes (server default, scenario tests, hold-heats) now run through averaged inverter + PSU. No expected values changed: V-SKEL keeps the ideal source; hold-heats, scenario and e2e tests pass unchanged.
- Test: the 4108 scene in averaged and switching mode tracks 20 rad/s, the PSU bus stays 22–25 V (a brief regen after the ramp lifts a blocking PSU slightly above 24 V), the inverter loss is booked, residual < 1e-3. `just check` green (184 Rust tests).

## 2026-10-12 — P07.T09/T10 Regen + V-INV/V-SUP sweep, performance
- Every V-INV and V-SUP case has a test (`cargo nextest run -p sim-core v_inv v_sup` → 13):
  - V-INV-001: FOC steady state, averaged vs switching scene, mean i_d/i_q at the PWM-center control ticks agree within 0.05 A = 1 % of the 5 A rating.
  - V-INV-002/003/005/007/008: existing tests renamed to their ids.
  - V-INV-004: switching dead-time error of the period-average terminal voltage = ∓(t_d/T)(V_bus + 2V_f − 2R_on|i|) for both signs, rtol 2 %.
  - V-INV-006: a–b ripple with phase c disconnected, t_d = 0, = V·D(1−D)T/(2L), rtol 5 %.
  - V-SUP-001/004: renamed.
  - V-SUP-002: flywheel braked by the FOC into a blocking PSU (low-R 8010 at 60 rad/s, since the 4108 burns everything in its 5.6 Ω copper and the bus never rises) rises above 24.5 V but stays under √(V₁² + Jω₀²/C); residual < 1e-3.
  - V-SUP-003 (= P07.T09 verify): the same with a chopper keeps the bus ≤ V_on + 0.01 V, the brake takes part of the kinetic energy, residual < 1e-3.
  - V-SUP-005: 3600·Q·ΔSoC = I·t + C·ΔV to rtol 1e-9.
- Found and fixed: inverter legs registered as 0.0 (= Off) floated phase a at engine start; legs now start Low.
- Bench (release, 10 ms sim, 20 kHz FOC):
  - skeleton/ideal 2.00 ms (5.0× real time);
  - averaged inverter + PSU at dt 4.7 µs 3.34 ms (3.0×);
  - switching at the T_pwm/200 rule (0.25 µs) 52.1 ms (0.19×), above the 0.05× threshold, so no P19 task.

## 2026-10-12 — P07.T13 P06 review carry-overs
- Gear loss: the meshing loss sees T_L plus the load side's share of the lumped friction (EQ-MECH-05 clarifications).
- Snapshot format v2 (`SNAPSHOT_VERSION`): P06/P07 modules added fields, so a v1 snapshot is rejected with a clear "snapshot format v1 … module data changed" instead of "missing …". No migration; snapshots are session artifacts.
- `get_param`: `gearbox.stiffness`/`damping` (with backlash) and `load.disturbance.duration` (the last commanded pulse length; the remaining time is the state `disturbance_left`).
- Interim FOC: torque → velocity restarts the speed integrator from the present i_q (`Foc::bumpless_to_velocity`); the P term still acts on the speed error at the switch (cancelling it as well made the loop push the wrong way while recovering). Comment: the torque command uses K_t(λ_ref) and ignores η and λ(T).
- Tests: V-THERM-002 through the motor's own R(T) (locked motor at V = 1.5·R(T_max)·Î_cont, network capacitances ×1e-4; T_w within 1 K, i_a = Î_cont to 1e-3); λ(T) at 80 °C lowers the torque per amp by 7.2 % (rtol 1e-9).
- Li-ion OCV: no published numeric full-cell NMC table found (the papers have curves only in figures; two sites blocked). By the user's choice: the provenance is now explicit (the NMC table is the Li-ion preset's; LiPo borrows it as an estimated proxy), and the search for a measured table is carried to P11.T07.

## 2026-10-12 — P07.T11 Phase 07 gate
- **Exit criteria (evidence):**
  - All V-INV and V-SUP cases pass: `cargo nextest run -p sim-core` (195 Rust tests in `just check`, all green); every catalog id has a test (V-INV-001…008, V-SUP-001…005).
  - Averaged vs switching agree on period-averaged currents: `inverter_avg.rs` (4.6 A operating point, 1 %) and V-INV-001, now loaded (0.45 N·m brake, i_q ≈ 1.5 A), within 0.05 A.
  - Regen: V-SUP-002 (bus rises above 24.5 V with a blocking PSU, under the lossless bound) and V-SUP-003 (chopper holds ≤ V_on + 0.01 V at 10 µs sampling; the resistor takes > 50 % of the kinetic energy, E_brake = ∫p_brake dt within the sampling bound).
  - Switching mode measured at 0.19× real time (P07.T10 entry).
- **Review** (phase-end subagent; 6 majors, 8 minors). Fixed:
  - PWM: at T − high < 2·t_d the Low turn-on ended after the period and was dropped (leg stuck Off, then a direct Low→High switch). Edges at or after the new period start now carry over; a conflicting start transition waits t_d after them. Sweep test d = 0.97…1.0.
  - Supply/bus validation: `supply.r_out`, `r0_cell`, `r1_cell` must be > 0; new `validate_bus` (C_bus > 0, R_b > 0, 0 < V_off < V_on, rating > 0); the build refuses a zero bus step cap.
  - Bus step rule: numerics.md EQ-NUM-04 and `fidelity::dt_max` now use τ_bus/2 (D-018), one rule everywhere.
  - Diode latch at a reconnect: chosen from the bound the terminal reached when |i| ≤ 1e-9 A (round-off sign), unit test.
  - V-SUP-004 at the catalog's rtol 1e-6: exact references with the bus eliminated (κ = C(1/C₁ + OCV'/3600Q), source current lag R₀C), charging and a new relaxation phase (I = 0, decay with τ = R₁C₁); the rig's load step is sampled by RK4's last stage with weight 1/6 (effective stop t₁ − h/6).
  - V-INV-007: averaged p_loss against the EQ-INV-09 terms computed independently (rtol 1e-9). V-INV-001 loaded. V-SUP-003 as above.
  - Minors fixed: chopper starts on above V_on; signals.md (`inverter.pwm_count`, averaged legs held +1); INVARIANT comment on the front-end constructors; the SVPWM saturation flag is noted as P09's anti-windup.
  - Carried to **P08.T10**: averaged pulse dropping (spec first), two-legs-Off warning/tracking, open-phase fault vs leg float, supply parameter paths + DcBus params, latch out of `outputs()`.
- `just check` green.

## 2026-10-12 — P08.T01 Sensor pipeline framework
- `physics::sensors::pipeline::SensorPipeline` (EQ-SENS-01): y_k = sat(Q(x(t_k − τ) + e(x(t_k − τ)) + n_k)), held between samples. History with integer-ns timestamps and linear interpolation (latency N periods → exactly the sample N back); optional error fn, seeded Gaussian noise (block RNG stream), quantisation step, wrap (angles), range clamp. `bypass` returns x and draws no noise. Save/restore includes the history and the RNG state.
- `SensorBlock`: fixed-rate discrete block (priority SENSOR) around a pipeline; PWM-centre triggering = period T_pwm, phase 0. Bypass is exact at each firing; continuous bypass (consumers read x(t)) is for the sensor builders.
- Tests: quantisation/wrap/range, delay of exactly 3 samples and the half-period midpoint, noise σ within 5 % over 1e4 seeded samples (mean within 4σ/√n), bypass exact, error at the delayed value, block snapshot/restore reproduces the noisy sequence.

## 2026-10-12 — P08.T02 Hall sensors
- `physics::sensors::hall` (EQ-SENS-02): `hall_states(θe, cfg, prev)` with placement offsets δ_x and hysteresis h (on at +h/2, off at −h/2 of the normalised field), `code` = 4a + 2b + c, `sector` 1–6 in rotation order (0 for codes 0/7). `HallBlock` (id `sensors.hall`, fixed rate) runs θe through the pipeline (latency, angle noise), writes `sensors.hall.{a,b,c,code,sector}`; bypass = ideal set (no offsets or hysteresis). `sensors.hall.sector` added to signals.md.
- Tests: codes change every 60° at 30° + 60°k and match the EQ-SENS-02 table and the EQ-INV-07 six-step pairs; δ_a = 10° moves a's edges to 40°/220°; h = 0.2 moves them to 35.74°/215.74°; the block writes code/sector and bypass ignores a 40° offset.

## 2026-10-12 — P08.T03 On-axis magnetic encoder
- Spec first (sensors.md EQ-SENS-03): the datasheets give only the INL peak, so the shape is fixed: harmonics (0.25, 1, 0.5) with phases (0, π/3, π/2), scaled to the INL peak; eccentricity slope k_ecc = 0.8°/mm (AS5047P: ±0.8° → ±1.2° at 0.5 mm), φ_ecc = 0; outputs `angle_raw` [0, 2π), `angle` (turn-counted), `error` = wrap(θm − raw). signals.md updated.
- `physics::sensors::encoder_magnetic`: `MagneticEncoderConfig` (bits, INL, displacement, mount offset, σ_nom, air gap, latency, bypass) → pipeline with error fn θ0 + e_INL + e_ecc, noise σ(z) = σ_nom·max(1, (z/z_max)³), INL(z) = INL·(1 + max(0, z_min − z)/z_min), Δ = 2π/2^b, wrap 2π; `MagneticEncoder` block (id `sensors.encoder`, reads `motor.theta`).
- `build::magnetic_encoder(&EncoderParams)` → config + period (default 10 kHz when the preset gives no rate).
- Presets aligned with the spec's chip table: AS5047P INL 0.34° (estimated) → 0.8° (datasheet), noise 0.068°; AS5600 INL 1° now datasheet, noise 0.015°; MT6701 noise 0.05° (estimated). `_sources.md` updated.
- Tests: V-SENS-001 (14-bit ramp steps exactly 2π/16384, every code boundary once), eccentricity −a sin θ with a = 0.4° at 0.5 mm, INL peak = 0.8°, air gap (σ ×8 at 2·z_max within 5 % over 1e4 samples, INL ×1.5 at z_min/2), block unwrap/error over three turns, presets build with the table values.

## 2026-10-12 — P08.T04 Incremental optical encoder
- `physics::sensors::encoder_incremental` (EQ-SENS-05): `IncrementalEncoder` (id `sensors.encoder`, reads `motor.theta` through the pipeline) counts c = ⌊4N/2π·(θ − θ_z)⌋ from the power-up angle θ_z; relative until the first crossing of θ_index (any turn, either direction), then anchored there: θ = θ_index + (c − c_index)·Δ. Outputs `sensors.encoder.{count, index, indexed, angle_raw, angle, error}` (signals.md updated). `drop_counts(m)` is the P10.T02 missed-count hook. Bypass: the true angle.
- Tests: 1000 CPR → 12000 counts over three turns, index exactly once per turn at the first sample past θ_index, absolute within one count after it; backward the count falls by 4000 per turn with one index pulse; relative → anchored at the index; missed counts; bypass. Samples sit half a count off the edges (a sample exactly on an edge is a floor() tie).

## 2026-10-12 — P08.T05 Current & voltage ADC
- `physics::sensors::adc` (EQ-SENS-06): `Adc` block (id `sensors.adc`) with one pipeline per channel (RNG substreams `sensors.adc.i_a` …): sat_[±fs](Q_Δ((1 + g)·i + off + n)), Δ = 2·fs/2^b; two shunts reconstruct i_c = −i_a − i_b, three measure all; optional bus-voltage channel (`bus.v` → `sensors.adc.v_bus`) with its own full scale. Period T_pwm with phase 0 samples at `pwm.center`; the controller period in averaged mode.
- Tests: quantisation, clamping at ±fs, two-shunt reconstruction vs a measured third shunt, gain/offset, bypass; V-SENS-005 (switching rig, 12-bit two-shunt ADC at the PWM centre = period average centred on the sample within 1 % of the 4.6 A operating current); V-SENS-006 (offset 0.1 A on a → αβ error vector of constant length 2·i_off/√3, i_d and i_q ripple at ωe within 5 %; the Python half is P11).

## 2026-10-12 — P08.T06 Velocity estimation
- New `control` module. `control::estimators::velocity` (EQ-CTRL-08): `DiffLpf` (wrap-safe difference + first-order LPF, α = T/(T + τ_f)), `Pll::design(T, ω_n, ζ)` (K_p = 2ζω_n, K_i = ω_n²; θ̂ unwrapped, initialised to the first sample), `VelocityEstimator` block (id `est.velocity`, priority ESTIMATOR) on any angle signal → `est.theta`, `est.omega`; snapshot/restore.
- control.md: θ̂ after a PLL step is the prediction for the next sample (one-sample lead, no ramp lag); found by the ramp test.
- Tests: the LPF on a wrapped ramp follows ω0·(1 − (1 − α)^k) to 1e-9; the PLL tracks a ramp with zero steady-state error and ω̂ = ω0; |H| at 2.48·ω_n = 1/√2 and at ω_n/10 ≈ 1 (2 %); block restore reproduces the estimates.

## 2026-10-12 — P08.T07 Sensorless flux observer
- `control::estimators::flux_observer` (EQ-CTRL-09, [Lee2010]): `FluxObserver` (forward Euler at the control rate from the measured i_αβ and the voltage applied over the last period; x starts at L_s·i, i.e. η = 0; θ̂e = atan2 η; PLL for ω̂e; valid = |ω̂e| > ω_min and ||η| − λ| < 0.2λ; default γ = 1000/λ²). `FluxObserverBlock` (id `est.observer`) reads three phase-current signals (Clarke inside) and v_α, v_β by path → `est.theta`, `est.omega` (electrical), `est.valid`.
- Spec: control.md now states the convergence time the task asked to verify (below 5° within 100 ms from η = 0 at ≥ 10 % rated speed; measured ~55 ms at 200 rad/s, λ = 10 mWb); signals.md: `est.*` are in the frame of the estimator's input.
- Tests (synthetic PMSM at constant speed, Euler-consistent voltages): 200 and 2000 rad/s converge below 5° after 100 ms, valid, ω̂ within 1 %; 3 % R error at 2000 rad/s stays below 5° on average; at 1.4 rad/s with 3 % R error the mean error exceeds 20° and `valid` is false (documented failure); block smoke test.

## 2026-10-12 — P08.T08 V-SENS sweep
- Every V-SENS catalog case has a test named by its id; `cargo nextest run -p sim-core v_sens` → 7/7 pass: 001 (encoder resolution), 002 (latency, pipeline), 003 (noise σ and seed), 004 (Halls), 005 (ADC at pwm.center, switching rig), 006 (offset → dq ripple; the Python half is P11), 007 (quadrature counts and index). Tests renamed, no expectations changed.

## 2026-10-12 — P08.T10 P07 review carry-overs (minors)
1. Averaged pulse dropping: inverter.md EQ-INV-03 first (effective dead fraction min(t_d/T, d, 1 − d), also in EQ-INV-08), then `leg::dead_eff`; exact at d = 0 (−R_on·i) and d = 1 (V − R_on·i), continuous. Test.
2. Two legs Off: the motor tracks a pending zero crossing per Off leg (events 0–2; diode bounds moved to 3–4), the first to cross opens. The warning fires only when two legs stay Off for more than 10 µs across discrete events (dead time is shorter). V-INV-004 (b and c share their dead time) asserts no warning.
3. `motor.open_phase` fault over a leg-driven float: takes it over (same phase) or reconnects and requests the new one; test with the leg turned back on (the fault holds the phase open).
4. D-019: supply parameters keep the flat `supply.*` paths of the YAML (supply.md, signals.md fixed). `DcBus` exposes `bus.capacitance`, `supply.voltage`, `supply.current_limit`, `supply.r_out` (get/set; C change books ½ΔC·V²; R·C below the build value is refused, D-018 step cap). Test: live edits, CC after a lower limit, residual < 1e-6, refused R.
5. The Off-leg diode is no longer latched in `outputs()`: the motor publishes the reconnect diode per phase (`motor.diode_x`, set at the event, cleared at the float and when the leg turns on); the inverter reads it, else its turn-off latch (`after_discrete`), else the sign of i / the nearer bound, all without mutation (D-017). Motor snapshots carry `diode`, per-phase `pending`; older single-pending snapshots still restore.
- `just check-fast` green.

## 2026-10-16 — P08.T12 Phase 08 review fixes
Phase-end review (P08.T09): 2 majors, 8 minors, no blockers. M1 (no sensor/estimator built from a scene) → P08.T11 (sensors) and P09.T15 (estimators, with m7), user's choice. The rest here:
- M2: `FluxObserver.x` is `Option<[f64; 2]>` (the NaN sentinel serialised as null and broke any snapshot before the first step). Test: serialise/restore a fresh observer, identical updates after.
- m1: `motor.open_phase` is refused while another phase's leg is Off (one floating phase at a time; zero-current mode is P10); a leg going Off while a fault holds a phase open warns. The motor remembers the Off legs of the last discrete event (`leg_off`, in snapshots). Test.
- m2: `enter_open` clears only the opening phase's reconnect diode.
- m3: inverter.md states the averaged deviation inside the pulse-dropping band (switching leg stays Low/High; averaged off by 2d(V_bus + V_f − R_on|i|) for the sign that gains on-time, 2d(R_on|i| − V_f) for the other). Tests: PWM keeps the leg Low/High at half the dead fraction; averaged deviation matches the stated expressions and bounds.
- m4: the two-legs-Off warning threshold is max(2·t_d, 1 µs), with t_d passed in `LegInputs.dead_time`. Test: 15 µs is quiet at t_d = 10 µs, warns at t_d = 0.
- m5: `SensorPipeline::reset` restores the initial RNG (same noise as a fresh run). Test.
- m6: control.md EQ-CTRL-09 table re-measured with its conditions stated (it gave no setup and did not reproduce): 700/140/14/1.4 rad/s → 0.2°/23°/52°/147°, valid fraction 1/0.97/0/0; `est.valid` does not detect parameter-mismatch angle error (|η| is driven to λ), only gross flux error (true flux ≥ 1.3× assumed clears it at 700 rad/s). Tests for the table (±3 %) and the |η| check replace the old low-speed test whose `!valid` held through the speed gate alone.
- m8: `PlantModule::params` doc matches D-015 (names under a param root are full paths); bus snapshots without the P08.T10 keys restore with the built values. Test.
- `just check-fast` green (234 tests).

## 2026-10-16 — P08.T11 Build the sensors from a scene (review M1)
- `physics::sensors::SensorSpec` (plain data: Hall, magnetic/incremental encoder, ADC, each with period and phase; `blocks()` builds them on their own substreams of the scene seed). `SkeletonOptions.sensors`; `try_build_engine_with` returns the signal error (e.g. a missing input) instead of panicking, `build_engine_with` keeps its signature.
- `build::sensors`: `update_rate` when given; else the ADC at `pwm.center` (T_pwm, phase 0) in switching mode and at the controller rate otherwise, the magnetic encoder at 10 kHz, the Halls and incremental encoder at the controller rate. Bypass samples at exactly the controller period (sensors run before controllers at the same instant, so each controller step reads x(t)); sensors.md states the rules.
- Schema: `sensors.adc.offsets` / `gain_errors` (per phase, override the scalars), `sensors.adc.bus` (full_scale, gain_error, offset, noise; refused at build without a DC bus); validation checks sensor noise in the sensor's unit. Schemas regenerated.
- Hall and encoder blocks take a first-sample phase (`with_phase`).
- Tests (`tests/sensor_scenes.rs`): every sensor preset builds and publishes; bypass equals motor.theta / motor.i_a (and −i_a − i_b for c) at controller instants despite INL, noise, offsets and a 1 kHz `update_rate`; sampling per mode and rate; per-phase errors and the bus channel reach the readings, bus channel refused in ideal mode; seeded noise repeats.
- `just check-fast` green (239 tests).

## 2026-10-16 — P08 phase gate (Sensors)
- `just check` exit 0: 239 Rust tests, web 5/5, e2e 1/1, validation suite, docs build, schemas and presets check, plan check.
- Exit criteria:
  - All V-SENS cases pass: `cargo nextest run -p sim-core v_sens` → 001–007 pass (the Python half of V-SENS-006 is P11).
  - Every sensor is bypassable and has rate, latency, noise and resolution: the blocks (P08.T01–T05) and, since P08.T11, scenes build them (`tests/sensor_scenes.rs`: presets publish, bypass equals the true value at every controller step, sampling per mode/rate).
  - Observer converges above its minimum speed and fails below it as documented: `converges_above_ten_percent_speed`, `low_speed_error_table_matches_the_spec` (EQ-CTRL-09 table re-measured with its conditions), `valid_drops_on_a_wrong_flux_above_omega_min`. Evidence run: 20/20 sensor/observer tests pass.
- Phase-end review: 2 majors, 8 minors, no blockers. M1 (nothing built sensors from a scene) → P08.T11 (sensors) + P09.T15 (estimators and m7, T from the block period, γλ²T check), user's choice; M2 and m1–m6, m8 fixed in P08.T12. Nits left: unipolar bus-voltage ADC range, Hall initial state inside the hysteresis band.
- Phase summary: common sensor pipeline (EQ-SENS-01), Halls (02), magnetic encoder with chip presets and air-gap law (03/04), quadrature encoder with index (05), shunt/bus ADC (06), velocity estimators (EQ-CTRL-08) and the sensorless flux observer (09); P07 carry-overs (pulse dropping, per-leg pending zeros, D-019 flat supply paths, live bus parameters, diode choice out of `outputs()`).
- graphify: hook rebuilt the code graph; 14 changed docs re-extracted (3 Haiku agents) and merged → 3677 nodes, 6974 edges, 327 communities.

## 2026-10-16 — P09.T01 Controller block framework
- D-020 (user's choice): a parameter's path is its scene path (`controller.position.enabled`, `fidelity.tier`, `seed`), signals keep `ctrl.*`; CONVENTIONS §3 and signals.md updated. The plan's `ctrl.setpoint.*` wording replaced by the spec's load-side `ctrl.theta_ref/omega_ref/torque_ref`.
- `control::setpoint::Setpoints::get_or_register` (reuses the skeleton's signals until P09.T03).
- `control::framework`: `RampLimiter`, `LoopConfig::from_hz` (validated rate, ordered limits, ramp > 0), `LoopLaw` trait (update/reset/track for bumpless re-enable/save/restore) with `Proportional` (the EQ-CTRL-04 position P), `Scaled` signals (unit/ratio gains), `LoopBlock` (id = parameter root, live `<id>.enabled`; enabled: ramped reference → law → clamp; disabled: passes the inner quantity's setpoint, the law tracks it, the ramp restarts from the measurement). `PlantInfo`, `Family`, `build_family` (refuses each family with the task that brings it until it lands).
- New signal `ctrl.omega_cmd` (velocity-loop reference, motor side) in signals.md.
- Tests: ramp limiter; config validation; position P over velocity P with N = 4: enabled → ω_cmd = K_p·N·(θ* − θ_L), live `controller.position.enabled = 0` → ω_cmd = N·ω_ref drives the velocity loop (verify), a disabled velocity loop passes the torque setpoint; re-enable ramps from the measurement, save/restore reproduces outputs, bad values refused; factory refusal names the task.
- `just check-fast` green (245 tests).

## 2026-10-16 — P09.T02 Discrete PI/PID with anti-windup
- `control::pid::Pid` (EQ-CTRL-01/02): u* = K_p e + I + D (+ ff), u = sat(u*), I += T(K_i e + K_b(u − u*)); `Clamping` (default; I frozen only while e pushes further into the active limit), `BackCalculation` (K_b = K_i/K_p, 0 for K_p = 0), `None`; filtered derivative on the measurement. `step(r, y, ff, limits, T)` saturates the total command with per-tick limits (EQ-CTRL-03 clarification); `track(u, r, y)` for bumpless starts; T is passed per update (the owning block's period, no second copy). `LoopLaw` impl; save/restore carry the state only (gains/limits come from the builder; infinite limits do not survive JSON).
- Tests: PI on an exactly discretised first-order plant equals the closed-form z_k = z_ss + A^k(z_0 − z_ss) (Cayley–Hamilton) to 1e-12 until the slow pole decays to 1e-12, no steady-state error; windup on an integrating plant with |u| ≤ 1 and a 5-unit step: no anti-windup overshoots 3.3 and settles (2 %) in 13.2 s, clamping 0.068 / 5.1 s, back-calculation 0.34 / 8.5 s (asserted < 5 %/45 % and < 15 %/70 % of the no-anti-windup values); clamping freezes only while pushing; back-calculation increment; no derivative kick, D → −K_d·c on a ramp; feed-forward saturation, bumpless track, state restore.
- `just check-fast` green (250 tests).

## 2026-10-16 — P09.T03 FOC current loop (sensored)
- control.md EQ-CTRL-03 first: timing step 5 (the loop runs at `pwm.center` or its own rate; the command computed at t_k applies from t_{k+1} and is held a period, the 1.5·T the angle advance compensates).
- `control::foc::current_loop::FocCurrentLoop` (block `controller.current`, D-020): phase currents (ADC or true) → Clarke → Park at θ̂e = gain·angle + offset; PI d/q (P09.T02, clamping by default) with decoupling feed-forward; d-priority voltage limit V_max = m_max·V_bus/√3 (bus measured or fixed) with saturation on the total command; inverse Park at θ̂e + 1.5·T·ω̂e; SVPWM duties for an inverter or α/β for the ideal source; ω̂e from a signal or difference + LPF of θ̂e. Gains K_p = ω_c L_{d,q}, K_i = ω_c R, default ω_c = 2π f/20; live `controller.current.bandwidth` [Hz] re-designs them. `FocSignals::register` (new measured `ctrl.foc.i_d`, `ctrl.foc.i_q`, documented in signals.md). Snapshot: PI states, the pending command, the speed filter.
- Plan: replacing the skeleton FOC moved to P09.T04 (every scene needs the speed loop, which lands there).
- Tests (`tests/foc_current.rs`, 7-pole-pair motor, ideal 24 V source, 20 kHz): V-CTRL-001 τ63 = 153 µs vs 1/ω_c = 159 µs (−4 %, rtol 15 %), no steady-state error; |i_d| < 2 % of i_q while accelerating to ω_e > 250 rad/s; at the voltage limit (20 A asked, 13.2 A reached, flag set) a drop to 5 A settles within 5 % in 0.95 ms with back-calculation (slew-limited 0.90 ms), 1.75 ms with clamping (integrator rebuilds from ~0), 26.9 ms without anti-windup; live bandwidth and snapshot restore.
- `just check-fast` green (254 tests).

## 2026-10-16 — P09.T04 Velocity & position loops; the skeleton FOC is gone
- Spec first: control.md EQ-CTRL-04 defaults (ω_v = ω_c/10, ω_p = ω_v/10; explicit kp/ki override each designed gain; position P unless ki), load-side `controller.limits.speed` and `controller.limits.ramp` (rad/s², new unit kind `angular_acceleration`: rad/s^2, rpm/s, deg/s^2, in the shared units fixture), feedback sources, scenario modes; the 13.5 % overshoot is for an ideal current loop: an independent continuous model with a first-order current loop and the 1.5·T delay gives 15.2 % at ω_c/10 (sim 15.8 %, the sampled velocity hold) and 14.3 % at ω_c/20, so V-CTRL-002 now states ω_v = ω_c/20 (atol 2 % kept).
- `control::foc`: `velocity_loop` (design K_p = ω_v J/K_t, K_i = K_p ω_v/4; `Prefiltered` PI behind 1/(1 + 4s/ω_v)), `position_loop` (motor-side gains ×N; P-over-P error formula), `FocSpec`/`LoopSpec`/`MotorModel`/`FocIo` and `foc::build` (position → velocity → current `LoopBlock`/`FocCurrentLoop`; no velocity loop → a disabled one passing τ*/(N K_t)). Current loop: open-loop v_q test mode applies at once with a 0.5·T lead (a voltage source, not a computation).
- Skeleton FOC deleted (`skeleton/foc.rs`, `FocBlock`, `ctrl.mode`; no INTERIM left in crates). The adapter registers `ctrl.foc.*` via `FocSignals`, picks feedback (ADC currents and encoder angle when fitted, else the true values; true speeds until P09.T15), builds the family from `SkeletonOptions.controller` (`FocSpec::skeleton()` by default, `v_supply` for the ideal source) and starts `ctrl.theta_ref` at the initial load angle. `build.rs` maps `controller: { family: foc, … }` to `FocSpec` (the current loop's default ω_c is now the spec's 2π f/20; sensorless → P09.T08); other families report their task.
- Scenario runner: velocity/torque/position targets switch `controller.position.enabled` / `controller.velocity.enabled`; position targets set `ctrl.theta_ref` (no ramp) and need a scene with a position loop.
- Fix in the P08.T11 sensor build: the Ideal tier bypasses every sensor (`enable_sensor_nonideal`). `sensor_scenes` ADC test compares at the sample instant (the FOC now acts on the readings).
- Tests (`tests/foc_cascade.rs`): V-CTRL-002 at ω_c/20 (overshoot within 2 % of 13.5 %, 2 % settling within 15 % of the design's closed form, ≈ 0 with the prefilter) and 15–16.5 % at ω_c/10; arm at 90° on a rigid 1:9 gear, Ideal tier: P over PI → |error| < 1 µrad, P over P (K_p,v = 0.5) → τ_L/(N² K_t K_p,v K_p,p) within 2 %; arm-servo preset from hanging down to 90°: loop error < 0.2 mrad, true load error within INL/N + backlash/2 + twist, at rest. Unit test of the design overshoot with/without prefilter.
- `just check-fast` green (256 tests), `just validate` 5/5.

## 2026-10-16 — P09.T05 Six-step with Halls
- Spec first (control.md EQ-CTRL-05): signed command u ∈ [−1, 1] (`ctrl.duty_ref` or the speed PI; sign = direction), Hall interpolation for the advance, speed-PI design on the DC-equivalent model (k_e = p·λ·mean(k_max − k_min), 3√3/π sinusoidal; K_p = ω_s τ_m k_e/V, K_i = ω_s k_e/V), a duty cap for `limits.current` from the DC model **with the inverter** (H/O freewheels through the low diode: |u| ≤ (k_e|ω| + V_f + 2(R + R_on)I)/(V + V_f); without V_f a stalled motor got 0.3 A of a 5 A limit), and the conditions of the flat-torque claim (sector ≥ 10·L/R, negligible speed ripple; near no-load speed a small speed ripple is a large current ripple — measured).
- EQ-INV-07 was specified but not implemented in P07 (only the sector table): `SwitchingInverter::with_six_step` (averaged High leg = H/O form with its DC current, Low leg = L, switching loss on the High leg only; `six_step_high_voltage`) and `Pwm::with_six_step` (`inverter.pattern_x`: centred H/O pulses, no dead time between H and O, dead time on a direct H→L, from L the high switch waits t_d).
- `control::six_step`: `SixStepBlock` (block `controller`, live `controller.advance`; Hall sector or true angle; reverse swaps High/Low; invalid code → all Off; `ctrl.six_step.sector`), `DutyLimit`, `emf_shape_ll`, `speed_design`, `SixStepSpec`/`build` (speed loop `controller.speed_loop` as a framework `LoopBlock`, bypass `ctrl.duty_ref`). `ControllerSpec` { Foc, SixStep } in the adapter options; six-step on the ideal drive is refused (it cannot float a phase). `LoopBlock::with_reference_limit` (speed limit on the speed reference, used by both families). Scenario velocity targets drive `controller.speed_loop` on six-step scenes; torque targets are refused there.
- Found: a bus driven below zero (six-step at high duty on a 5 A PSU, before the cap) made the floating-terminal `clamp(−V_f, V_bus + V_f)` panic. The clamp is now min/max (no panic); the missing bus reverse clamp through the bridge diodes is P10.T09.
- Tests (`tests/six_step.rs`, 5010 with a 120° trapezoidal EMF): the six-step demo (Halls with hysteresis/latency, flywheel, current cap) holds 150 rad/s within 1 % after 3 s; Ideal tier, duty 0.045 against a 0.05 N·m brake with a flywheel-sized rotor (10 ms sectors = 10·τ_e): torque within 5 % of its mean outside ±20° of the commutations, the Off phase diode-clamped (−V_f or V_bus + V_f) while its current flows and floating after each commutation; negative command spins backwards; ideal drive refused; switching mode: H/O pulses on the High leg, Low leg L, spins. Unit tests: H/O PWM pattern, averaged H/O formula, 3√3/π, Hall interpolation.
- `just check-fast` green (264 tests), `just validate` 5/5.

## 2026-10-16 — P09.T06 Open-loop V/f
- Spec first (control.md EQ-CTRL-06): ω*_e = N·p·`ctrl.omega_ref` ramped at `controller.accel` (load side, rad/s²) and clamped to the speed limit; k_vf in V per electrical rad/s; the vector at the centre of its hold; loss of synchronism also by **pole slip** (unwrapped error moved 2π from its first value). Found by the V-CTRL-009 test: a fast-slipping rotor passes |wrap| < 90° every turn, so the "> 90° for an electrical period" rule alone never fired.
- `control::open_loop`: `OpenLoopBlock` (block `controller`, live `controller.v0`/`k_vf`/`accel`; SVPWM duties for an inverter or v_α/v_β for the ideal source; true angle for the diagnostic only), `OpenLoopSignals` (`ctrl.open_loop.v_alpha/v_beta/theta_ref/omega_ref/sync_lost`, signals.md), `OpenLoopSpec`; `ControllerSpec::OpenLoop`; the adapter registers each family's voltage-command signals (the ideal source reads FOC's or open loop's). Validation: `controller.k_vf` (back-EMF constant kind), `controller.accel` (angular acceleration). Scenario velocity targets drive open loop through `ctrl.omega_ref` directly.
- Engine: `DiscreteBlock::take_warnings` (reported as engine warnings after each block step).
- Tests (`tests/open_loop.rs`, 5010 on the ideal 24 V source, k_vf = 1.05·λ, V_0 = 0.3 V, 0.02 N·m brake): follows 50 rad/s (mean over 0.5 s within 1 %, in sync, ω*_e = 700 rad/s); V-CTRL-009: a 3 N·m step (pull-out ≈ 2 N·m) latches `sync_lost`, raises the warning, and the mean speed leaves the command by > 10 %.
- `just check-fast` green (266 tests), `just validate` 5/5.

## 2026-10-16 — P09.T07 MIT / impedance mode
- The plan's "i_q* = τ/(K_t·N·η)" contradicts EQ-CTRL-07 (no efficiency factor: gear loss is speed-dependent and zero at standstill); the spec wins, the law uses τ*/(N K_t).
- `control::mit`: `MitBlock` (block `controller`, live `controller.kp`/`kd`/`tau_ff`; τ* = K_p(θ* − θ_L) + K_d(ω* − ω_L) + τ_ff + `ctrl.torque_ref`, i_q* clamped to the current limit, written to `ctrl.foc.iq_ref`), `MitSpec`, `damping_ratio`. `FocSpec.cascade` (false: the current loop alone); `ControllerSpec::Mit`; the adapter feeds the load angle (encoder/N or the true load angle) and load speed (`load.omega` or motor/N, true until P09.T15). Validation: `controller.kp` torsional stiffness, `controller.kd` rotational damping. Scenario targets on MIT scenes are plain setpoints (no loop switching).
- Tests (`tests/mit.rs`, 5010 + flywheel, N = 1, ω_n = 2π·5 rad/s, ζ = 0.3, Ideal tier): V-CTRL-003 deflection under a constant 0.1 N·m = τ_L/K_p within 2 %; the step-overshoot ζ matches K_d/(2√(K_p J)) within 5 %; `controller.kp` live.
- `just check-fast` green (268 tests), `just validate` 5/5.

## 2026-10-16 — P09.T15 Estimators built from a scene (P08 review M1, m7)
- Spec first (control.md EQ-CTRL-08/09 "In a scene"): one estimator owns `est.*` — the flux observer when `sensors.observer` is given (or the controller is sensorless, P09.T08), else a velocity estimator on the encoder (PLL ω_n = 2π f_s/50, ζ = 1, or difference + LPF with τ_f = 10 samples); consumers use the estimate (velocity loop `est.omega`, current loop ω̂e = p·`est.omega`, MIT `est.omega`/N), six-step on Halls its Hall-interval speed `ctrl.six_step.omega`; observer defaults γ = 1000/λ², ω_min = 0.05·V_bus/(√3 λ), PLL ω_n = 2π f_c/100; γλ²T ≥ 2 refused. T always from the block period (m7).
- Schema: `sensors.velocity` (method pll | diff_lpf, bandwidth, zeta, tau_f) and `sensors.observer` (gamma, omega_min, pll_bandwidth, pll_zeta) replace `sensors.observer_gamma` (unused by any preset); validated; schemas regenerated; signals.md roots and `ctrl.six_step.omega`.
- `control::estimators`: `EstimatorSpec`/`VelocitySpec`/`ObserverSpec`, `velocity_block`, `observer_block` (currents: ADC when fitted; voltage: `ctrl.foc.v_alpha/v_beta` read before the current loop updates it, i.e. over the last period; L_s = (L_d + L_q)/2). Adapter: `SkeletonOptions.estimators`, speeds routed to consumers. Six-step publishes the Hall-interval speed (bounded by 60°/(time since the last edge), so it decays when the rotor stops) and its speed loop uses it.
- Found with real feedback: the preset scenes' loops were tuned against the true speed. arm-servo (encoder PLL + compliant, backlashed gear) rang at a 60 Hz velocity loop (±22 % speed ripple; ±0.5 % with true speed, ±3 % with a rigid gear) → 30 Hz / position 5 Hz (≈ ±4 %); gimbal-hold (AS5600 polled at 1 kHz) limit-cycled at 30 Hz → 15 Hz / 3 Hz. Commented in the scenes; P09.T12 retunes the presets. Tests adjusted to what they test: the units test asserts on `ctrl.omega_ref`; the gearbox setpoint test checks the mean speed over the last 0.5 s.
- Tests (`tests/estimator_scenes.rs`): AS5047P scene regulates 40 rad/s on the PLL speed (est and true means within 1 %); an observer in a running FOC scene: angle error < 5°, `est.valid`, est.omega = p·ω within 5 %; γ = 1e10 refused (γλ²T); six-step demo at 100 rad/s on the Hall speed (true within 2 %, Hall speed within 2 % of true).
- `just check-fast` green (272 tests), `just validate` 5/5.

## 2026-10-16 — P09.T08 Sensorless FOC (startup + hand-off)
- Spec first (control.md EQ-CTRL-10 "Details"): one supervisor owns the sequence, the velocity PI and the current loop's angle/speed; align (i_d = I_align at θ = 0), I/f (i_q = I_if·sign, θ* integrates ω*_e ramped at a), blend θ = θ* + w·wrap(θ̂ − θ*) over t_blend, closed loop on θ̂ with the velocity PI started bumplessly from I_if, fallback to I/f from the last estimate when `est.valid` is lost for t_valid. Defaults I_align = I_if = 0.5·I_lim, t_align 0.3 s, a = 0.25·K_t I_if/(N J), ω_handoff = 3·ω_min, t_valid 20 ms, t_blend 50 ms. The "bounded current spike" is specified: i_q* continuous across the hand-off, measured |i| below the current limit through the whole start.
- Schema: `controller.startup` (align_current, align_time, current, accel, handoff_speed, valid_time, blend_time). A sensorless FOC always gets the flux observer (P09.T15).
- `control::foc::sensorless`: `SensorlessBlock` (block `controller.sensorless`; `ctrl.sensorless.theta/omega/state`, signals.md), `StartupOptions::resolve`, `velocity_pi`; `FocSpec.sensorless`, `foc::build` builds supervisor + current loop on its angle (`FocIo.omega_min` from the observer).
- Test (`tests/sensorless.rs`, 4108 with no position sensor, averaged inverter, 4 A limit): states 0 → 1 → 2 → 3, 30 rad/s within 2 % after 3 s, peak |i| < 4 A, i_q* jump at the switch < 0.1 A; commanded to stop, it falls back to I/f.
- `just check-fast` green (273 tests), `just validate` 5/5.
