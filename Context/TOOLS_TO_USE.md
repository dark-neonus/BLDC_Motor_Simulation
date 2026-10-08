# Tools to Use

> Companion to [POLISHED_IDEA.md](POLISHED_IDEA.md). Decided 2026-10-08 via Q&A.
> † = decided by Claude without a question (conventional/low-risk choice). Exact versions get pinned during planning.

---

## 1. Architecture at a glance

```
┌──────────── Browser (React UI) ─────────────┐        ┌──────── AI agent ────────┐
│ PixiJS viz · uPlot plots · React Flow loop  │        │  MCP client (Claude etc.)│
└───────────────┬─────────────────────────────┘        └────────────┬─────────────┘
                │ REST (commands) + WebSocket (MessagePack stream)  │ MCP (streamable HTTP)
┌───────────────▼───────────────────────────────────────────────────▼─────────────┐
│                 Rust single binary  (axum server + rmcp MCP)                     │
│   sim-core: physics · multi-rate scheduler · fixed-step solvers · Luau blocks    │
│   sim-model: parameters · constraint graph · YAML presets/scenarios              │
│   headless CLI: run-scenario → Parquet/CSV                                       │
└──────────────────────────────────────┬───────────────────────────────────────────┘
                                       │ Parquet
                     Python validation suite (independent reference model)
```

**Why a local backend:** one source of truth for UI and agents. Native speed and multiple cores. The sim keeps running when no tab is open.

---

## 2. Simulation core & server — Rust

| Tool | Purpose | Why |
|---|---|---|
| **Rust (stable)**, Cargo workspace | Core, server, CLI | C-level speed, no GC pauses (switching-PWM mode near real time), strong types catch logic errors. |
| **Custom fixed-step integrators** (RK4 + semi-implicit) | Real-time loop | Predictable step-by-step execution inside a **multi-rate discrete scheduler** (controllers, PWM, sensors at exact times). Each integrator is validated against diffsol and SciPy. |
| **diffsol** | Adaptive/stiff ODE reference, offline high-accuracy runs | Maintained, peer-reviewed (JOSS 2026), provides an independent check of our integrators. |
| **nalgebra** † | Small vectors/matrices (Clarke/Park, state vectors) | De-facto Rust linear algebra, fixed-size types without heap use. |
| **uom** | Compile-time SI units | Zero runtime cost. Mixing volts and amps fails to compile. |
| **mlua** (Luau mode) | Custom controller block | Small embeddable language, real sandbox, fast enough for 20 kHz loops. |
| **serde + serde-saphyr** | YAML presets/scenarios | `serde_yaml` is deprecated; serde-saphyr is its maintained continuation. |
| **schemars** † | JSON Schema from Rust types | Validates YAML files and gives editor autocompletion. One source of truth for file formats. |
| **axum + tokio + tower-http** † | REST + WebSocket server | Standard async Rust web stack. |
| **rmcp** | MCP server inside the binary | Official Rust MCP SDK. Agents drive the *running* instance directly. |
| **utoipa** † | OpenAPI spec from Rust handlers | The API doc is generated, never hand-written. It also feeds TS type generation. |
| **rmp-serde** † | MessagePack for the WebSocket stream | Compact binary, much cheaper than JSON at high signal rates. |
| **arrow + parquet** † / **csv** † | Recording output | Parquet for the validation pipeline. CSV/JSON for user export. |
| **rust-embed** † | Bundle the built UI into the binary | `just run` → one process → open `localhost`. |
| **clap** †, **directories** †, **tracing** †, **thiserror/anyhow** † | CLI, OS user-data paths, logging, errors | Standard, boring, reliable. |

**Rust testing:** `cargo-nextest` (runner), `approx` (float comparison), `proptest` (property-based: e.g. energy is always conserved), `insta` (snapshot tests), `criterion` (benchmarks that track the real-time factor). **Lint:** `rustfmt`, `clippy`.

---

## 3. Frontend — TypeScript

| Tool | Purpose | Why |
|---|---|---|
| **React + Vite + TypeScript (strict)** | UI framework, dev server | Biggest ecosystem, best known by AI agents, instant reloads. |
| **pnpm** † | Package manager | Fast, strict, disk-efficient. |
| **shadcn/ui (Radix) + Tailwind CSS** | Components + Claude-like dark/light orange theme | Accessible by default (ARIA, good for agents), fully themeable via CSS variables, no lock-in. |
| **Dockview** | IDE-style dockable panels | Rearrangeable/saveable layouts (e.g. "Learning", "Tuning"). |
| **PixiJS** | Motor visualization | WebGL/WebGPU 2D. 10k+ current particles at 60 fps. |
| **uPlot** | Live plots | Fastest web time-series plotter, made for streaming. |
| **React Flow (@xyflow/react)** † | Control-loop block diagram | The leading node/diagram editor library for React. |
| **Zustand** † + **TanStack Query** † | Client state + REST data | Minimal global state; cached, typed server calls. |
| **openapi-typescript** † + **@msgpack/msgpack** † | Typed API client, stream decoding | TS types generated from the Rust OpenAPI spec, so front and back can't drift. |
| **math.js (units)** | Unit-aware input (`300rpm`, `2 kgf*cm`) | Mature unit parser/converter. |
| **CodeMirror 6** † | Luau editor in the custom block | Much lighter than Monaco, has Lua syntax highlighting. |
| **KaTeX** † | Math inside `?` tooltips | Fast LaTeX rendering, same engine as the docs. |

**Testing:** **Vitest** + Testing Library (units/components), **Playwright** (E2E; also checks that agent-facing element IDs stay stable). **Lint/format:** **Biome** †.

---

## 4. Documentation

| Tool | Purpose | Why |
|---|---|---|
| **Docusaurus** | Docs site from Markdown/MDX in the repo | React-based, so docs can embed **live widgets reused from the app**. Mature and actively developed. |
| **remark-math + rehype-katex** † | LaTeX equations | Standard math pipeline. |
| **Mermaid** (Docusaurus theme) † | Diagrams in Markdown | Text-based diagrams that agents can read and write. |
| **llms.txt plugin** † | AI-agent index of docs | Agents find the right page without crawling. |
| **Help registry (YAML)** † | Single source for `?` tooltips | Each id holds short text + doc anchor. Consumed by both app and docs, so they can't diverge. |

---

## 5. Validation suite — Python

| Tool | Purpose | Why |
|---|---|---|
| **uv** | Python env & deps | Already installed, fast, lockfile-based. |
| **pytest** + **hypothesis** † | Test runner, property-based tests | Standard. |
| **NumPy + SciPy (`solve_ivp`)** | **Independent reference model** | Different language, solver and author from the Rust core, so it is a real cross-check. |
| **pyarrow / polars** † | Read the Rust Parquet output | Fast, columnar. |
| **matplotlib** † | Plots in the validation report (published in the docs) | Standard. |
| **ruff** † | Lint/format | Fast, all-in-one. |

**Bridge:** the Rust **headless CLI** (`run-scenario scenario.yaml → signals.parquet`). No bindings, so the code under test stays independent of the reference.

---

## 6. Dev environment & CI

| Tool | Purpose | Why |
|---|---|---|
| **mise** | Pins Rust / Node / pnpm / Python versions | One `mise install` gives an identical toolchain everywhere. |
| **just** | Task runner | `just dev`, `just test`, `just validate`, `just docs`, `just check`. |
| **lefthook** † | Git pre-commit hooks | Fast format/lint before each commit. |
| **GitHub Actions** | CI | Rust tests, Python validation, Vitest, Playwright, docs build on every push/PR. |

---

## 7. Implementation workflow

- **graphify** skill: build and query a knowledge graph of code + docs during implementation.
- **Subagents** (used where they pay off):
  - **Independent reference model**: written by a separate agent that never sees the Rust physics code, only the documented equations. This keeps the validation honest.
  - Parallel, well-separated work: docs pages, frontend components vs core modules, reviews (`/code-review`) of finished milestones.
  - Broad codebase searches (Explore agent) once the repo grows.
