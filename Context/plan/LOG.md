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
