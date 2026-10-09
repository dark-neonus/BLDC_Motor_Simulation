# Phase 04 — Parameter model, units, schemas & presets

> **Goal:** `sim-model` becomes the authoritative description of every configurable thing: typed parameters with units and provenance, a constraint graph that makes contradictory input impossible, YAML IO with JSON Schemas, library folders, the generic preset library, scene and scenario formats, and the datasheet-wizard logic.
> **Depends on:** P03.
> **Read first:** POLISHED_IDEA §4, [CONVENTIONS §3–4](CONVENTIONS.md), `docs/docs/physics/conventions.md`, [DECISIONS D-003](DECISIONS.md).
> **Exit criteria:**
> - All preset YAML files load and pass validation.
> - Every constraint rule has a test.
> - `bldc-sim dump-schemas` writes schemas, and every preset validates against its schema with an independent validator (`uvx check-jsonschema`).
> - `apply_edit` propagates Kv→λ→Kt correctly.
> - The units fixture passes in Rust.

- [x] **P04.T01** — Units module and shared fixture
  - **Depends:** P03
  - **Do:**
    - `sim-model::units`: re-export the `uom` SI types used.
    - A **whitelisted unit table** (unit string → quantity kind → SI factor/offset). Include:
      - rpm, rad/s, deg, rad, mH, µH/uH, H, mΩ/mOhm, Ω/Ohm
      - Wb, mWb, V, A, N·m/N*m/Nm, kgf·cm/kgf*cm, g·cm², kg·cm², kg·m²
      - °C/degC, K, mm, cm, m, g, kg, Hz, kHz, µs/us, ms, s, W, J, F, µF/uF, Ah, mAh
      - rpm/V, V/krpm, N·m/A
    - `parse_quantity("100 rpm/V", expected_kind)` and `format_quantity`.
    - Create `schemas/units-fixture.yaml` (≥ 40 input → SI pairs, including tricky ones like `"45 °C"` → 318.15 K and `"2 kgf*cm"`).
  - **Files:** `crates/sim-model/src/units.rs`, `schemas/units-fixture.yaml`
  - **Verify:** `cargo nextest run -p sim-model units` (the fixture-driven test passes).
  - **Done when:** Passing. Wrong-kind input gives a helpful error ("expected inductance, got resistance").

- [x] **P04.T02** — Provenance wrapper `Param<T>`
  - **Depends:** P04.T01
  - **Do:** `Param<T> { value: T, source: Measured|Datasheet|Estimated|Derived|Default, note: Option<String> }` with serde that accepts both shorthand `kv: "100 rpm/V"` (source = Default/Datasheet per context) and long form `kv: {value: "100 rpm/V", source: datasheet, note: "seller page"}`.
  - **Files:** `crates/sim-model/src/param.rs`
  - **Verify:** Round-trip tests for both forms.
  - **Done when:** Passing.

- [x] **P04.T03** — Parameter types for all components
  - **Depends:** P04.T02
  - **Do:** Define serde + `schemars::JsonSchema` structs. **Every field gets a doc comment**, which becomes the schema description and seeds the help registry.
    - `MotorParams`:
      - identity: name, size class, topology inner/outer
      - electrical: R, L/M or Ls, λ, the user-entered Kv/Kt form, connection
      - magnetic: back-EMF shape, cogging harmonics, saturation curve
      - winding: slots, pole pairs, layout
      - geometry: stator OD/ID, stack, airgap, magnet thickness/arc, rotor dims
      - mechanical: J_rotor, mass, friction
      - thermal: network R/C, T_max
      - ratings: V, I_cont, I_peak, rpm
    - Also: `GearboxParams`, `LoadParams` (arm, constant, viscous), `InverterParams`, `SupplyParams` (psu | battery, bus C, chopper), `SensorParams` (halls, encoder variants, ADC, observer), `ControllerParams` (family + loops + limits + rates + custom Luau), `FaultParams`, `ProtectionParams`, `FidelityParams`.
    - Field names follow the namespace (CONVENTIONS §3).
  - **Files:** `crates/sim-model/src/params/*.rs`
  - **Verify:** `cargo build -p sim-model`; a test that serializes defaults to YAML and back.
  - **Done when:** It covers POLISHED_IDEA §3–§5 fully.

- [x] **P04.T04** — Winding/topology computations
  - **Depends:** P04.T03
  - **Do:** Implement and test:
    - Slot/pole validity for 3-phase (slots divisible by 3; q = slots/(3·2p) rules; gcd condition for balanced windings).
    - Winding layout via the **star-of-slots** method (coil → phase and polarity).
    - Fundamental winding factor.
    - Cogging period N_c = LCM(slots, 2p).

    The layout is exposed for visualization (P14) via a function `winding_layout(slots, p) -> Vec<Coil {slot_a, slot_b, phase, polarity}>`.
  - **Files:** `crates/sim-model/src/winding.rs`
  - **Verify:** Tests against published values: 12N14P winding factor ≈ 0.933; 9N12P ≈ 0.866; 36N42P ≈ 0.933; 24N28P ≈ 0.933. Cross-check with an online calculator (e.g. emetor.com) and cite it in the test comment. Invalid combos (e.g. 12N12P) are rejected.
  - **Done when:** Passing.

- [x] **P04.T05** — Constraint graph & `apply_edit`
  - **Depends:** P04.T04
  - **Do:**
    1. Declarative rules, each `{id, inputs, outputs, kind: Derive|Reject|Warn, explanation, help_id}`:
       - Kv ↔ Ke ↔ Kt ↔ λ (one is the "entered" form, the rest derived and locked)
       - R/L line-to-line ↔ phase; star/delta
       - slot/pole validity; winding factor
       - geometry → J_rotor, mass, thermal C estimates (Estimated, overridable)
       - L ≥ M sanity; positive values; λ plausibility vs size class (Warn)
       - (the "ratings vs thermal continuous limit" rule is added later in P06.T07, once the thermal math exists)
       - controller current limit ≤ inverter/PSU limits (Warn)
       - encoder bits plausible
    2. `apply_edit(model, path, value) -> EditResult { model, changed: Vec<(path, old, new)>, issues: Vec<Issue{severity, path, message, help_id}> }`. Rejects leave the model unchanged.
    3. `validate(model) -> Vec<Issue>`.
  - **Files:** `crates/sim-model/src/constraints/*.rs`
  - **Verify:** A test per rule. Property test (proptest): any sequence of valid edits leaves the model with `validate()` free of Reject issues.
  - **Done when:** Passing. Explanations are written for beginners.

- [x] **P04.T06** — YAML IO, schema version & migrations
  - **Depends:** P04.T03
  - **Do:**
    - `load_yaml<T>(path)` / `save_yaml(path, &T)` with serde-saphyr.
    - Files start with `# yaml-language-server: $schema=<relative path to schemas/X.schema.json>` and include `schema_version: 1`.
    - A migration hook (`migrate(v_from, value)`) for future renames.
    - Errors include file:line:col and the param path.
    - CLI subcommand `bldc-sim validate-file <files...>`: load + `validate()`, print issues (file, path, severity, message), exit code 1 on any Reject or parse error. Used by preset tasks and CI.
  - **Files:** `crates/sim-model/src/io.rs`
  - **Verify:** Tests: a malformed file gives a line/col error; an unknown field is rejected (`deny_unknown_fields`) with a "did you mean" suggestion if cheap; `validate-file` exit codes are correct.
  - **Done when:** Passing.

- [x] **P04.T07** — JSON Schema generation
  - **Depends:** P04.T06
  - **Do:** First derive/implement `schemars::JsonSchema` for all `sim-model::params` types (deferred from P04.T03; `Param` needs a manual impl accepting number | quantity string | `{value, source, note}`). Then `bldc-sim dump-schemas --out schemas/` writes one schema per file type (motor, gearbox, load, inverter, supply, sensors, controller, scene, scenario; `lesson` is added in P18.T09). Add the `just gen-schemas` recipe. CI check: regenerating produces no diff.
  - **Files:** `crates/bldc-sim/src/cli/dump_schemas.rs`, `schemas/*.schema.json`
  - **Verify:** `just gen-schemas && git diff --exit-code schemas/`; `uvx check-jsonschema --schemafile schemas/motor.schema.json presets/motors/*.yaml` (once T09 exists; wire it into `just check`).
  - **Done when:** Schemas are committed and the check is wired into `just check`.

- [x] **P04.T08** — Library locations & operations
  - **Depends:** P04.T06
  - **Do:**
    - Library sources: **builtin** (`presets/`, embedded in the binary via rust-embed and read-only), **user** (`~/.local/share/bldc-sim/library/`, created on first run) and **extra** folders listed in `~/.config/bldc-sim/config.yaml`.
    - Operations: list(kind, filter), load(id), save_as(user), duplicate(builtin → user, with a new name), delete(user only), rename.
    - IDs: `builtin:motors/8010-outrunner`, `user:motors/my-8010`.
  - **Files:** `crates/sim-model/src/library.rs`
  - **Verify:** Tests with temp dirs: builtin is read-only; duplicate preserves content and changes the name; extra folders are listed.
  - **Done when:** Passing.

- [ ] **P04.T09** — Generic motor preset research & files
  - **Depends:** P04.T05, P04.T08
  - **Do:**
    1. For classes 2804, 4108, 5010, 6010, 6020, 8010, 8108, 10015 and 12020 (outrunner gimbal/robot types; plus inrunner variants where common), research **typical** values from ≥ 2 public datasheets per class (web search): slots/poles, Kv, phase R, L, mass, rotor inertia (if listed), rated/peak current and torque.
    2. Record ranges and sources in `presets/motors/_sources.md`.
    3. Pick representative values (median-ish) and write `presets/motors/<class>-<topology>.yaml`. Missing values are estimated by the documented rules (source: estimated).
  - **Files:** `presets/motors/*.yaml`, `presets/motors/_sources.md`
  - **Verify:** `cargo run -p bldc-sim -- validate-file presets/motors/*.yaml` → no Reject issues. Check derived Kt·I_peak against the datasheet peak torque within ±30 % (record the comparison in `_sources.md`).
  - **Done when:** ≥ 9 motor presets, all validated, with sources documented. These are **generic classes**, not specific products (user decision).

- [ ] **P04.T10** — Other component presets
  - **Depends:** P04.T08
  - **Do:** YAML presets:
    - **inverters:** small 24 V/10 A and medium 48 V/40 A generic MOSFET stages (R_ds,on, t_r/t_f, dead time, PWM 20/40 kHz)
    - **supplies:** lab PSU 24 V/5 A, 30 V/10 A, 48 V/10 A; batteries 4S/6S/12S Li-ion/LiPo/LiFePO4 with OCV curves cited
    - **gearboxes:** planetary 1:4/1:6/1:9, cycloidal 1:20/1:50, with typical efficiency and backlash
    - **loads:** arm 20 cm with 0.5 kg; arm 15 cm with 2 kg; flywheel; constant-torque brake
    - **sensors:** AS5600, AS5047P, MT6701, generic hall set, 1000-CPR optical, 12-bit shunt ADC
    - **controllers:** placeholders until P09.T12
  - **Files:** `presets/**`
  - **Verify:** `validate-file` on all files.
  - **Done when:** All validate, and the sources for chips/batteries are cited in `_sources.md` files.

- [ ] **P04.T11** — Scene & scenario formats
  - **Depends:** P04.T03
  - **Do:**
    - **Scene** = components (each a library reference *or* inline params, with overrides) + fidelity + seed + initial conditions.
    - **Scenario** = scene ref + `record:` signal list + `timeline:` of actions:
      - `set {path, value}`, `target {kind: position|velocity|torque, value, ramp}`
      - `fault {id, on/off}`, `disturbance {torque, duration}`
      - `wait`, `assert {signal, op, value, tol, at|window}`
      - `snapshot`, `stop`
    - Asserts make scenarios usable as regression tests.
  - **Files:** `crates/sim-model/src/{scene,scenario}.rs`, `presets/scenes/*.yaml`, `presets/scenarios/*.yaml`
  - **Verify:** Round-trip tests. 3 example scenes (gimbal-hold, arm-servo, six-step-demo) validate.
  - **Done when:** Passing.

- [ ] **P04.T12** — Datasheet wizard logic (backend)
  - **Depends:** P04.T05, P04.T09
  - **Do:**
    - `wizard::start(inputs) -> WizardState` with a list of **convention questions** where inputs are ambiguous (Kv definition variant; R line-to-line or phase; star/delta; "poles" means pole count or pole pairs?).
    - `answer(q, a)`; `finish() -> MotorParams` with provenance (Datasheet for inputs, Derived for computed, Estimated for filled-in values).
    - Estimation rules from the size class (inertia from geometry, L from typical τ_e by class, thermal from class), each documented in `docs/docs/physics/estimation.md` (create it, with EQ-EST IDs).
    - Each estimate gets a confidence (low/med/high).
  - **Files:** `crates/sim-model/src/wizard.rs`, `docs/docs/physics/estimation.md`
  - **Verify:** Test: entering a real public datasheet (record which) produces a model whose derived Kt/peak torque match the datasheet within tolerance.
  - **Done when:** Passing.

- [ ] **P04.T13** — Engine builds from a scene; scenario asserts & CLI exit codes
  - **Depends:** P04.T05, P04.T08, P04.T11
  - **Do:** `sim-core::build_engine(scene: &ResolvedScene) -> Engine` (resolve library refs in `sim-model`, convert uom → f64 per D-003). Live `SetParam` commands route through `apply_edit` first (so constraints are enforced at runtime too). Wire the CLI `run-scenario` to the full scenario format (keep skeleton scenarios working, or migrate them). Scenario `assert` actions are evaluated by the engine at their time/window. The CLI prints a pass/fail table, writes `asserts.json` next to the outputs, and **exits with code 2 if any assert fails** (1 = error, 0 = all passed). `--no-asserts` skips them.
  - **Files:** `crates/sim-core/src/build.rs`, CLI
  - **Verify:** `just validate` still green; a new scenario using `presets/scenes/gimbal-hold.yaml` runs (with the skeleton physics until P05); a scenario with a deliberately false assert exits 2.
  - **Done when:** Passing.

- [ ] **P04.T14** — Phase gate
  - **Depends:** P04.T01, P04.T02, P04.T03, P04.T04, P04.T05, P04.T06, P04.T07, P04.T08, P04.T09, P04.T10, P04.T11, P04.T12, P04.T13
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-04-done`.
