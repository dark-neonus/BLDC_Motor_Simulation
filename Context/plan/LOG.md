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
