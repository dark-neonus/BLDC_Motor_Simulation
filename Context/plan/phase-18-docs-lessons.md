# Phase 18 — Documentation content & guided lessons

> **Goal:**
> - Complete, beginner-friendly docs (POLISHED_IDEA §9) built on the P02 spec.
> - Generated references (API, MCP, file formats, CLI).
> - Live analytic widgets.
> - Help registry with no TODOs.
> - Guided lessons runnable in the app.
> - An agent-oriented `llms.txt`.
>
> **Depends on:** P17, P11.
> **Read first:** POLISHED_IDEA §7.5, §9; [CONVENTIONS §8](CONVENTIONS.md); [DECISIONS D-008](DECISIONS.md).
> **Exit criteria:**
> - `scripts/check_help.py` passes **without** `--allow-todo`.
> - The docs build has no broken links/anchors.
> - All 6 lessons run end-to-end in Playwright.
> - `llms.txt` lists every page.
> - Docs are reachable from the app (`/docs` served by the binary, or a configured URL).

- [ ] **P18.T01** — Docs information architecture & landing
  - **Depends:** P17, P11
  - **Do:** Sidebar per POLISHED_IDEA §9: Getting started, UI guide, Motor fundamentals, Physics & math (the P02 spec pages), Power electronics, Sensors, Control theory, Lessons, For agents (API/MCP), File formats, Validation report. Landing page with "Start here" paths for "I want to learn", "I want to simulate my motor" and "I'm an agent". The docs theme matches the app palette (Claude-like, light/dark).
  - **Files:** `docs/sidebars.ts`, `docs/src/css/custom.css`, `docs/docs/index.md`
  - **Verify:** `pnpm -C docs build` clean.
  - **Done when:** The structure is in place with stubs.

- [ ] **P18.T02** — Motor fundamentals pages
  - **Depends:** P18.T01
  - **Do:** Beginner-first pages (plain-language intro → diagram → math → "try it in the app" link):
    - what a BLDC/PMSM is
    - BLDC vs PMSM (back-EMF shapes)
    - poles, pole pairs & slots
    - Kv/Kt/Ke/λ (with the datasheet-convention table)
    - back-EMF
    - cogging
    - inrunner vs outrunner
    - geometry naming (`DDHH`)
    - torque vs speed vs heat (why low-RPM torque motors are different)

    Mermaid/SVG diagrams.
  - **Files:** `docs/docs/fundamentals/*.md`
  - **Verify:** Build + each page has ≥ 1 diagram and a "try it" link.
  - **Done when:** Complete.

- [ ] **P18.T03** — Physics pages polish & sync with code
  - **Depends:** P18.T01
  - **Do:** Re-read the P02 pages against the final implementation:
    - every EQ ID used in code exists in the docs, and vice versa (script `scripts/check_eq_ids.py`: grep `EQ-[A-Z]+-\d+` in `crates/` and `validation/` vs the docs anchors)
    - add friendly intros to each page

    Add the check to `just check`.
  - **Files:** `docs/docs/physics/*`, `scripts/check_eq_ids.py`
  - **Verify:** `python3 scripts/check_eq_ids.py` passes.
  - **Done when:** Passing.

- [ ] **P18.T04** — Power electronics, sensors, control theory pages
  - **Depends:** P18.T01
  - **Do:** Beginner explanations of:
    - inverter & PWM, SVPWM, dead time
    - power supplies vs batteries, regen & brake resistors (why PSUs can be damaged)
    - halls vs encoders (with an on-axis magnetic encoder photo/diagram), ADC & current sensing, sensorless observers
    - block diagrams, PI/PID, cascaded loops, FOC step-by-step, six-step, impedance control, tuning & auto-tune
    - the custom Luau block (already written in P09.T10; link it)
  - **Files:** `docs/docs/{power,sensors,control}/*.md`
  - **Verify:** Build clean.
  - **Done when:** Complete.

- [ ] **P18.T05** — UI guide with generated screenshots
  - **Depends:** P18.T01
  - **Do:** A Playwright script `web/e2e/docs-screenshots.spec.ts` (tagged, not part of normal e2e) captures each panel in both themes into `docs/static/img/ui/`. Write the UI guide pages: layout, viz, plots, control diagram, params & units, library & editors, wizard, scenarios, faults, playground, clock & slow motion, status bar & energy indicator.
  - **Files:** `docs/docs/ui/*.md`, `web/e2e/docs-screenshots.spec.ts`, `just docs-screenshots`
  - **Verify:** `just docs-screenshots && pnpm -C docs build`.
  - **Done when:** Complete.

- [ ] **P18.T06** — Generated references: API, MCP, CLI, file formats
  - **Depends:** P18.T01
  - **Do:** Scripts generating Markdown:
    - **REST:** from `openapi.json` (or embed Scalar on a docs page)
    - **MCP tools:** from a `bldc-sim dump-mcp-tools` subcommand (add it), listing name, description and input schema
    - **CLI:** from `bldc-sim --help` output per subcommand
    - **File formats:** from `schemas/*.schema.json`, with field tables + examples from presets

    Wire into `just gen-docs` (and check for no diff in `just check`).
  - **Files:** `scripts/gen_docs_*.py`, `docs/docs/reference/*`
  - **Verify:** `just gen-docs && git diff --exit-code docs/docs/reference/`.
  - **Done when:** Passing.

- [ ] **P18.T07** — Live analytic widgets in docs
  - **Depends:** P18.T02
  - **Do:** Per D-008, MDX components computed in TS, reusing app UI primitives via a small shared package `web/src/shared` (or `packages/ui` if cleaner — record a decision). Widgets:
    - back-EMF shape vs angle (sine ↔ trapezoid slider)
    - Kv ↔ Kt converter
    - RL step response
    - pendulum torque vs angle
    - SVPWM hexagon
    - PI step response on a first-order plant
  - **Files:** `docs/src/components/widgets/*`
  - **Verify:** Build + a Vitest per widget's math function.
  - **Done when:** ≥ 6 widgets embedded in the relevant pages.

- [ ] **P18.T08** — Help registry completion
  - **Depends:** P18.T02, P18.T03, P18.T04, P18.T05
  - **Do:** Write/review every help entry (short, plain words first; math optional) and point each `doc` link to the right anchor. Remove `--allow-todo` from `just check` and CI.
  - **Files:** `help/registry.yaml`
  - **Verify:** `python3 scripts/check_help.py` (strict) passes.
  - **Done when:** Passing.

- [ ] **P18.T09** — Lesson format & lesson runner panel
  - **Depends:** P18.T01
  - **Do:**
    - **Lesson YAML** (`presets/lessons/*.yaml`): title, goal, doc page link, scene, plot preset, layout, and steps. Each step has: text (markdown), optional `ui_command`s (highlight/open), an optional action (load scenario, set param), and an optional **check** (a signal condition the learner should reach, e.g. "make the arm hold 90° with < 2° error").
    - **Lessons panel:** list, a step-through runner, check feedback (✓/hint), and "show me" (auto-performs the step). Implement `open_lesson` ui_command (from P13.T10).
  - **Files:** `crates/sim-model/src/lesson.rs`, `web/src/panels/lessons/*`
  - **Verify:** Vitest for the runner state machine. Schema generated.
  - **Done when:** Passing.

- [ ] **P18.T10** — The six lessons
  - **Depends:** P18.T09, P18.T07
  - **Do:** Write lessons + their doc pages:
    1. *What is back-EMF?*
    2. *Kv vs Kt*
    3. *Why FOC beats six-step* (torque ripple comparison)
    4. *Tuning your first PI loop* (current then velocity; auto-tune comparison)
    5. *Why holding torque heats the motor* (thermal; continuous vs peak)
    6. *What regen does to your power supply* (PSU vs battery vs chopper)

    Each runs in the app and has a docs page with a "Open in app" link.
  - **Files:** `presets/lessons/*.yaml`, `docs/docs/lessons/*.md`
  - **Verify:** Playwright runs each lesson with "show me" for all steps; all checks pass.
  - **Done when:** Passing.

- [ ] **P18.T11** — `llms.txt`, "For agents" page, docs served by the app
  - **Depends:** P18.T06
  - **Do:**
    - Configure the llms plugin to emit `llms.txt` + `llms-full.txt`.
    - Write `docs/docs/agents.md`: how to connect via MCP (`.mcp.json`), the recommended tool flow, the ID namespace, `window.__bldc` for browser automation, `ui_command` to show things to the user, and common recipes.
    - Embed the built docs into the binary at `/docs` (rust-embed, feature `embed-docs`) so tooltip links work offline. The help base URL defaults to `/docs`.
  - **Files:** `docs/docusaurus.config.ts`, `docs/docs/agents.md`, `crates/sim-api/src/static_files.rs`
  - **Verify:** `just build` → `localhost:8787/docs/llms.txt` is served; a tooltip "Read more" opens the embedded docs page at the right anchor.
  - **Done when:** Passing.

- [ ] **P18.T12** — Phase gate
  - **Depends:** P18.T01, P18.T02, P18.T03, P18.T04, P18.T05, P18.T06, P18.T07, P18.T08, P18.T09, P18.T10, P18.T11
  - **Do:** PLAN §8 checklist. The reviewer also reads 3 random beginner pages and rates clarity for a non-expert, with concrete rewrite suggestions.
  - **Done when:** Tagged `phase-18-done`.
