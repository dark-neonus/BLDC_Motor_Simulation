# Conventions

> Binding for all agents. If you need to break a convention, add a decision in [DECISIONS.md](DECISIONS.md) first.

## 1. Repository layout

```
/
├── CLAUDE.md                 # agent entry point → points to Context/PLAN.md (AGENTS.md symlinks to it)
├── Context/                  # idea, tools, plan (this folder's parent)
├── Cargo.toml                # Rust workspace
├── crates/
│   ├── sim-model/            # parameter types, units, schemas, constraints, YAML IO + library folders (local file IO only; no physics stepping, no network)
│   ├── sim-core/             # engine: time base, signal bus, integrators, scheduler, all physics + control blocks (no IO)
│   ├── sim-api/              # axum REST/WS server, MCP (rmcp), exposes sim-model's library over HTTP, embedded web/docs assets
│   └── bldc-sim/             # the binary: CLI subcommands (serve, run-scenario, dump-openapi, dump-schemas, validate-file)
├── web/                      # React + Vite frontend (pnpm workspace member)
├── docs/                     # Docusaurus site (pnpm workspace member); physics spec lives in docs/docs/physics/
├── help/registry.yaml        # `?` tooltip registry (single source for app + docs)
├── presets/                  # built-in read-only YAML library: motors/ inverters/ supplies/ sensors/ gearboxes/ loads/ controllers/ scenes/ scenarios/ lessons/
├── schemas/                  # generated JSON Schemas (do not edit by hand) + units-fixture.yaml
├── validation/               # Python uv project: refmodel/, tests/, scenarios/, report/
├── scripts/                  # small helper scripts (codegen, checks)
├── mise.toml · justfile · lefthook.yml · pnpm-workspace.yaml · package.json · biome.json · rust-toolchain.toml
└── .github/workflows/ci.yml
```

Dependency direction: `sim-model ← sim-core ← sim-api ← bldc-sim`. `sim-core` never does file or network IO.

## 2. Ports & paths

| What | Value |
|---|---|
| Server (REST, WS, MCP) | `http://localhost:8787` — `/api/*`, `/api/stream` (WS), `/mcp` |
| Web dev server | `http://localhost:5173` (proxies `/api` and `/mcp` to 8787) |
| Docs dev server | `http://localhost:3000` |
| User library | `directories` crate data dir → Linux `~/.local/share/bldc-sim/library/` |
| User config | `~/.config/bldc-sim/config.yaml` (extra library folders, UI prefs) |

## 3. The shared ID namespace (most important convention)

Every parameter, signal, action and panel has **one dotted path**, used identically in YAML keys, the REST API, MCP tools, the help registry and UI attributes.

| Kind | Path example | UI attribute | Help id |
|---|---|---|---|
| Parameter | `motor.electrical.kv` | `data-agent-id="param:motor.electrical.kv"` | `param:motor.electrical.kv` |
| Signal | `motor.i_a`, `ctrl.foc.iq_ref`, `energy.residual` | `data-agent-id="signal:motor.i_a"` | `signal:motor.i_a` |
| Action | `sim.play`, `fault.phase_open.a` | `data-agent-id="action:sim.play"` | `action:sim.play` |
| Panel | `plots`, `viz`, `control-diagram` | `data-agent-id="panel:plots"` | `panel:plots` |

Rules:
- Path segments are `snake_case`. Top-level roots: `sim`, `motor`, `inverter`, `supply`, `bus`, `sensors`, `ctrl`, `gearbox`, `load`, `thermal`, `fault`, `protect`, `energy`, `scenario`.
- Paths are **stable API**. Renaming one requires a decision entry and a YAML migration.
- Every interactive UI element has a `data-agent-id` **and** an `aria-label`. The Playwright audit (P13.T04) enforces this.

## 4. Units

- **Internally SI everywhere** (rad, rad/s, A, V, Ω, H, Wb, N·m, kg·m², K, W, J, s). Temperatures are stored in **K** and displayed in °C by default.
- **Rust:** `uom` types at the **model/API boundary** (`sim-model` types, block constructors, public functions). Inside hot ODE/state-vector code, plain `f64` in SI with a `// [unit]` comment on each field. See decision D-003.
- **YAML:** a value is either a bare number (meaning SI) or a quantity string `"<number> <unit>"` from the whitelist in `sim-model::units` (e.g. `"100 rpm/V"`, `"2.5 mH"`, `"45 °C"`). Saved files use quantity strings in the user's chosen display unit.
- **Frontend:** parses input with math.js units and sends SI numbers to the API. `schemas/units-fixture.yaml` lists (input string → SI value) pairs. Rust and TS both test against it, so the two parsers can't drift.

## 5. Physics conventions (summary; the full spec is `docs/docs/physics/conventions.md`, EQ-CONV-*)

- `p` = **pole pairs** (never "poles" in code; the UI shows both). θe = p·θm.
- **Amplitude-invariant** Clarke/Park (k = 2/3). The d-axis is aligned with the rotor magnet flux.
- **Canonical magnet constant:** λ_m = peak phase flux linkage [Wb]. Everything else is derived:
  - Kt = 1.5·p·λ_m [N·m per A of peak phase current = iq]
  - Ke,LL,pk = √3·p·λ_m [V·s/rad, line-to-line peak]
  - Kv = 60 / (2π·Ke,LL,pk) [rpm/V]  ⇒  Kt ≈ 8.27 / Kv
  - Datasheets differ (RMS, phase vs LL, six-step DC). The wizard asks which convention applies.
- Resistance/inductance are stored as **per-phase, star-equivalent** values. Delta windings are converted to star equivalents, and line-to-line measurements are halved (star).
- Positive rotation: counter-clockwise viewed from the output-shaft end.
- Power sign: positive = motoring (electrical → mechanical). Bus current is positive when drawn from the supply.
- Gravity angle: the load arm angle is θ_load = 0 when pointing straight down (stable equilibrium) and positive CCW.

## 6. Code style

### Rust
- Edition 2024, stable toolchain pinned in `rust-toolchain.toml`. `#![forbid(unsafe_code)]` in `sim-model` and `sim-core`.
- `thiserror` for library errors, `anyhow` only in `bldc-sim` main.
- **No `unwrap()`/`expect()` in non-test code** except on provable invariants, with a `// INVARIANT:` comment.
- Physics: one function per equation, named after it, with a doc comment citing the spec ID:
  ```rust
  /// Electromagnetic torque, dq frame. EQ-MOT-04.
  fn torque_dq(p: f64, lambda_m: f64, l_d: f64, l_q: f64, i_d: f64, i_q: f64) -> f64 { ... }
  ```
- Hot loop: no heap allocation per step (preallocate), no locks. Commands come via a channel and are applied at step boundaries.
- Randomness only via the engine RNG service (`rand_chacha`, seeded per block from the scene seed).
- `cargo clippy --all-targets -- -D warnings` must pass.

### TypeScript
- `strict: true`. Biome for lint+format. Function components and hooks. State in Zustand stores (`web/src/state/`).
- API types are **generated** (`web/src/api/schema.d.ts` from OpenAPI). Never hand-write API types.
- Pixi and uPlot code lives in imperative modules (`web/src/viz/`, `web/src/plots/`) with thin React wrappers. Keep the render loop out of React state.

### Python
- Python 3.13 (pinned for wheel availability), `uv` project in `validation/`. `ruff` for lint+format. Type hints everywhere.
- The reference model (`validation/refmodel/`) must not import anything from the Rust side. It only reads its outputs.

## 7. Tests

- Rust test names describe the physics: `locked_rotor_current_rises_with_rl_time_constant`. The doc comment cites the EQ/V IDs.
- Float comparisons use `approx` with **explicit, justified tolerances** (comment why that tolerance). Always give **both** an absolute floor and a relative tolerance (`|a−b| ≤ atol + rtol·|b|`), so signals near zero (t ≈ 0, i_d ≈ 0) don't produce meaningless relative errors. Python uses the same form (`numpy.allclose(atol=, rtol=)`).
- Validation tests are named after the catalog ID: `test_v_mot_001_no_load_speed`.
- Golden/snapshot updates (`cargo insta review`, `just validate --update-goldens`) require a LOG note explaining why outputs changed.

## 8. Docs & help

- Physics spec pages give each equation an ID anchor: `### EQ-MOT-04 — Electromagnetic torque (dq) {#eq-mot-04}`.
- Help registry entry:
  ```yaml
  param:motor.electrical.kv:
    title: Velocity constant (Kv)
    short: |
      How many rpm the motor spins per volt with no load. Lower Kv = more torque per amp, less speed. $K_t \approx 8.27/K_v$.
    doc: /docs/fundamentals/motor-constants#kv
  ```
  `short` is ≤ 3 sentences, plain words first, math optional.

## 9. Commits & logs

- Commit: `PNN.TMM: <imperative summary>`. One task per commit where practical.
- LOG.md entry format:
  ```markdown
  ## 2026-10-09 — P03.T05 (agent: Claude)
  - Did: …
  - Verified: `cargo nextest run -p sim-core scheduler` → 14 passed
  - Notes/surprises: …
  - Next: P03.T06
  ```
