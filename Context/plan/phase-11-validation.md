# Phase 11 — Independent validation suite

> **Goal:** prove the Rust core right with evidence that does not depend on the Rust code:
> - an independent Python reference model written from the spec by an isolated subagent
> - analytic tests from the catalog
> - Rust-vs-reference cross-checks
> - property tests
> - preset consistency checks
> - golden scenario regressions
> - a published validation report
>
> **Depends on:** P11.T01 needs only P02. The rest need P10.
> **Read first:** `docs/docs/physics/validation-catalog.md`, [CONVENTIONS §6–7 (Python)](CONVENTIONS.md), [PLAN §5 subagent policy](../PLAN.md#5-subagent-policy).
> **Exit criteria:**
> - `just validate` runs all V-* Python cases, the cross-checks and the property tests green.
> - The report is generated into the docs.
> - CI runs `validate` (if a remote exists).

- [~] **P11.T01** — 🤖 SUBAGENT: independent Python reference model
  - **Depends:** P02
  - **Do:** Spawn a **`general-purpose` subagent** (pre-approved) with the prompt below. Run it in the background and continue with other tasks if any are available. When it returns, the main agent reviews the code for isolation violations (`grep -r "crates/" validation/refmodel` must be empty, and the agent's report must list only docs files read) and runs its self-tests.
    > You are writing an **independent reference implementation** of a BLDC/PMSM motor drive simulation in Python, used to validate a separate Rust implementation that you must **never look at**.
    >
    > **Allowed to read:** `docs/docs/physics/**` (the spec), `Context/plan/CONVENTIONS.md` §5 and §6-Python, and `validation/pyproject.toml`.
    > **Forbidden:** anything under `crates/`, `web/`, `presets/`, `target/`, and git history. Do not run `bldc-sim`. If the spec is ambiguous, **do not guess**: write the question to `validation/refmodel/SPEC_QUESTIONS.md` and pick the most literal reading, marking it in code with `# SPEC-AMBIGUITY: <question id>`.
    >
    > **Build** `validation/refmodel/` (package) implementing:
    > - **Plant:** abc motor model with isolated neutral, back-EMF shapes, cogging, iron loss, Karnopp friction, rigid gearbox, arm gravity load, thermal network with R(T)/λ(T), DC bus + PSU CV/CC (unidirectional) + battery + chopper, **averaged** inverter with SVPWM/SPWM and six-step leg patterns.
    > - **Discrete controllers:** FOC current/velocity/position PI with anti-windup, MIT mode, six-step with ideal halls, open-loop V/f — as per `control.md`.
    > - **Integration:** `scipy.integrate.solve_ivp` (method `Radau` or `DOP853`, `rtol=1e-9, atol=1e-12`) between discrete controller ticks with zero-order hold, exactly as specified in `numerics.md` (event timing). Energy terms are integrated as extra states.
    >
    > Every function cites its EQ ID in a docstring. Use NumPy/SciPy only. Use **exactly** the signal and parameter names from `docs/docs/physics/signals.md`.
    >
    > **Interface:** `simulate(params: RefParams, setpoints: list[Action], t_end: float, record: list[str]) -> polars.DataFrame`. Columns use the signal names from the spec (CONVENTIONS §3); time in s, SI units. `RefParams` is a flat dataclass tree (SI) documented in `validation/refmodel/README.md`.
    >
    > **Self-tests** in `validation/refmodel/tests/`: implement the analytic V-* cases from `validation-catalog.md` that apply to your scope, against your own model.
    >
    > Run `cd validation && uv run pytest refmodel -q` and `uv run ruff check refmodel` until green.
    >
    > **Final report:** files created; list of every file you read (to prove isolation); spec questions raised; any catalog cases you could not implement and why.
  - **Files:** `validation/refmodel/**`
  - **Verify:**
    - `cd validation && uv run pytest refmodel -q` green.
    - Isolation check: grep + the agent's read-list.
    - Each `SPEC_QUESTIONS.md` item is answered by improving the spec (docs) and then the refmodel, with a LOG note.
  - **Done when:** The refmodel self-tests are green and all spec questions are resolved.

- [ ] **P11.T02** — CLI output contract & resolved-scene export
  - **Depends:** P10
  - **Do:**
    - Freeze the Parquet contract: column per recorded signal (CONVENTIONS §3 names), `t` in s, file metadata `{bldc_sim_version, scene_hash, seed, tier, inverter_mode}`.
    - Add `bldc-sim resolve-scene <scene|scenario> --json` → a fully resolved, **SI, flat** parameter JSON (no library refs), so Python can configure the refmodel without parsing our YAML formats.
    - Document both in `docs/docs/reference/cli.md`.
  - **Files:** `crates/bldc-sim/src/cli/*.rs`, `docs/docs/reference/cli.md`
  - **Verify:** A Rust test checks the schema stability (insta snapshot of the column list + metadata keys).
  - **Done when:** Passing.

- [ ] **P11.T03** — Adapter: resolved scene → `RefParams`
  - **Depends:** P11.T01, P11.T02
  - **Do:** `validation/adapters/scene_to_ref.py` maps the resolved JSON to the refmodel dataclasses, plus scenario timeline → refmodel actions. Fail loudly on any unmapped field (no silent defaults).
  - **Files:** `validation/adapters/*.py`
  - **Verify:** A pytest round-trip on all preset scenes: the mapping is complete (or explicitly listed as out of refmodel scope).
  - **Done when:** Passing.

- [ ] **P11.T04** — Analytic catalog tests against the Rust CLI
  - **Depends:** P11.T02
  - **Do:** Implement every catalog case marked `side: python` or `both` as `validation/tests/test_v_<area>.py`, running the release CLI on scenario files under `validation/scenarios/` and comparing with the analytic formulas. Tolerances are exactly as in the catalog.
  - **Files:** `validation/tests/test_v_*.py`, `validation/scenarios/*.yaml`
  - **Verify:** `just validate -k test_v_` green.
  - **Done when:** Every applicable catalog ID has a Python test (coverage script from P10.T08 extended to Python).

- [ ] **P11.T05** — Rust vs reference cross-checks
  - **Depends:** P11.T03, P11.T04
  - **Do:** A scenario set (≥ 12: each controller family × 2 motors, plus thermal, regen, gearbox/backlash-free, cogging) run in both. Compare per-signal NRMSE and max error with tolerances per signal class (currents, speed, angle, temperature, bus voltage), justified in the test docstring. On failure, the plot script `validation/report/plot_compare.py` writes overlay PNGs.
  - **Files:** `validation/tests/test_crosscheck.py`, `validation/report/plot_compare.py`
  - **Verify:** `just validate -k crosscheck` green.
  - **Done when:** Green with no loosened tolerances (PLAN §4.2).
  - **If stuck:** A mismatch means the spec, the Rust code or the refmodel is wrong. Locate it by comparing the shortest failing scenario signal by signal (TROUBLESHOOTING §5.7). Fix the spec ambiguity first.

- [ ] **P11.T06** — Property-based tests (hypothesis)
  - **Depends:** P11.T04
  - **Do:** Random valid motor params (within preset ranges) × random setpoints. Invariants:
    - energy residual < tolerance
    - no NaN
    - steady no-load speed ≈ V/(p·λ) bound
    - stall torque = Kt·I within tolerance
    - determinism (two runs equal)

    Keep it fast: limited examples by default, `--hypothesis-profile=thorough` for CI nightly.
  - **Files:** `validation/tests/test_properties.py`
  - **Verify:** `just validate -k properties`.
  - **Done when:** Green.

- [ ] **P11.T07** — Preset consistency checks
  - **Depends:** P11.T04
  - **Do:** For each generic motor preset: derived no-load speed at rated V, stall torque, Kt·I_peak vs peak torque, and continuous torque from thermal, compared with the ranges recorded in `presets/motors/_sources.md` (machine-readable front matter added there). Flag outliers.
  - **Files:** `validation/tests/test_presets.py`
  - **Verify:** `just validate -k presets`.
  - **Done when:** All presets are within their documented ranges.

- [ ] **P11.T08** — Golden scenario regressions
  - **Depends:** P11.T04
  - **Do:** Golden Parquet outputs for key scenarios in `validation/goldens/` (small: decimated). Compare with tolerance. Add `just validate --update-goldens` (writes new goldens + requires a LOG note per CONVENTIONS §7).
  - **Files:** `validation/tests/test_goldens.py`, `validation/goldens/*`
  - **Verify:** Green; changing a physics constant deliberately makes it fail (then revert).
  - **Done when:** Green.

- [ ] **P11.T09** — Validation report → docs
  - **Depends:** P11.T05, P11.T06, P11.T07, P11.T08
  - **Do:** `validation/report/build_report.py` runs the suite with JSON output (`pytest --junitxml` or `pytest-json-report`) and writes `docs/docs/validation/report.md` (a table per catalog ID: pass/fail, measured vs expected, tolerance; embedded comparison plots in `docs/static/validation/`). Add `just validate-report`. Enable the `validate` job in CI.
  - **Files:** `validation/report/*`, `docs/docs/validation/report.md`
  - **Verify:** `just validate-report && pnpm -C docs build`.
  - **Done when:** The report renders in the docs.

- [ ] **P11.T10** — Phase gate
  - **Depends:** P11.T01, P11.T02, P11.T03, P11.T04, P11.T05, P11.T06, P11.T07, P11.T08, P11.T09
  - **Do:** PLAN §8 checklist. The reviewer focus is on test independence, weak tolerances and untested catalog IDs.
  - **Done when:** Tagged `phase-11-done`.
