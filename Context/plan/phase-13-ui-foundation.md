# Phase 13 — UI foundation

> **Goal:** the app shell every feature panel plugs into:
> - Claude-like dark/light theme
> - Dockview layouts
> - generated API client + WS client + stores
> - the agent-ID convention enforced by tests
> - the help registry with `?` tooltips
> - unit-aware inputs
> - schema-driven parameter panels
> - status bar, clock toolbar, and the handling of server `ui_command`s
>
> **Depends on:** P12.
> **Read first:** POLISHED_IDEA §7, [CONVENTIONS §3, §6-TS, §8](CONVENTIONS.md), `docs/docs/api/*`.
> **Exit criteria:**
> - The app shows the dockable layout, theme toggle, a parameters panel generated from the API with working unit inputs and constraint messages, `?` tooltips with doc links, a status bar and clock controls.
> - The Playwright audit finds every interactive element has `data-agent-id` + `aria-label`.
> - An agent `ui_command` focuses an element.

- [ ] **P13.T01** — Tailwind + shadcn/ui + Claude-like theme
  - **Depends:** P12
  - **Do:**
    1. Tailwind v4 + shadcn/ui init.
    2. Theme tokens as CSS variables for light and dark: warm neutral backgrounds (light ≈ `#FAF9F5`, dark ≈ `#262624` / `#1F1E1D`), text ≈ `#141413` / `#F5F4EE`, **accent orange** ≈ `#D97757` (hover darker), muted borders, success/warn/danger.
    3. Check contrast ≥ WCAG AA for text (verify with a contrast checker and note the ratios in a comment).
    4. Theme toggle: system/light/dark, persisted in localStorage (try/catch). Fonts: system UI sans + a mono for numbers.
  - **Files:** `web/src/styles/theme.css`, `web/src/components/ui/*`, `web/src/components/ThemeToggle.tsx`
  - **Verify:** Visual check in both themes. A Vitest test for the persistence fallback when localStorage throws.
  - **Done when:** Both themes look cohesive. A screenshot of each is saved in `docs/static/img/ui/`.

- [ ] **P13.T02** — API client, WS client, stores
  - **Depends:** P13.T01
  - **Do:**
    - `openapi-fetch` client typed by the generated `schema.d.ts`; TanStack Query hooks for REST.
    - WS client v1: MessagePack, reconnect with backoff, subscription manager (ref-counted signal subscriptions from components), dispatch by message type.
    - Zustand stores: `simStatus`, `params` (tree + edit results), `events/warnings`, `signals` (ring buffers per subscribed signal, outside React render), `viz` (latest frame).
  - **Files:** `web/src/api/*`, `web/src/state/*`
  - **Verify:** Vitest with a mock WS server (`mock-socket`): reconnect, subscribe/unsubscribe ref-counting, event dispatch.
  - **Done when:** Passing.

- [ ] **P13.T03** — Dockview shell & saved layouts
  - **Depends:** P13.T01
  - **Do:** Dockview with panel registry: `viz`, `plots`, `control-diagram`, `params`, `library`, `scenario`, `faults`, `lessons`, `log` (events). Built-in layouts "Playground" (default), "Learning", "Tuning". Users can save custom layouts (localStorage, try/catch) and reset. Every panel container has `data-agent-id="panel:<id>"`.
  - **Files:** `web/src/layout/*`
  - **Verify:** Playwright: switch layouts, drag a panel, reload → layout persisted.
  - **Done when:** Passing.

- [ ] **P13.T04** — Agent-ID convention: helper + audit test
  - **Depends:** P13.T03
  - **Do:**
    - Helper `aid(kind, path)` returning `{ 'data-agent-id': ..., 'aria-label': ... }` props.
    - A Playwright audit spec that walks every panel in every layout and asserts that all `button, input, select, [role=slider], [role=tab], a, [contenteditable]` have both attributes and that the IDs are unique on the page.
    - Expose `window.__bldc` (read-only debug API: current params, status, list of agent ids) for agents using browser automation.
  - **Files:** `web/src/lib/aid.ts`, `web/e2e/agent-ids.spec.ts`
  - **Verify:** `just e2e -g agent-ids`.
  - **Done when:** Passing, and part of `just check`.

- [ ] **P13.T05** — Help registry pipeline + `<HelpTip>`
  - **Depends:** P13.T02
  - **Do:**
    1. `help/registry.yaml` (CONVENTIONS §8). Seed entries from the schema descriptions (script `scripts/seed_help.py` adds missing ids with a `TODO` marker) and from the signal registry.
    2. Generate `web/src/help/registry.gen.ts` (`just gen-help`).
    3. `<HelpTip id>`: a `?` icon → Radix tooltip/popover with title, short markdown (KaTeX for math), and a "Read more →" link to the docs URL (the docs base URL is configurable; the dev default is `localhost:3000`, prod is the docs served by the binary at `/docs` if embedded).
    4. Check script `scripts/check_help.py`: every param/signal/action id has an entry with no `TODO`, and every `doc` link resolves to an existing docs anchor (parse the docs build output). Wire it into `just check`. Allow TODOs until P18 via a flag `--allow-todo`.
  - **Files:** `help/registry.yaml`, `scripts/{seed_help,check_help}.py`, `web/src/help/*`
  - **Verify:** `just gen-help && python3 scripts/check_help.py --allow-todo`.
  - **Done when:** Tooltips render. The check runs in CI with `--allow-todo` (removed in P18).

- [ ] **P13.T06** — Unit-aware numeric input
  - **Depends:** P13.T02
  - **Do:** `<QuantityInput path>`:
    - parses with math.js units (accepts `300rpm`, `2 kgf*cm`, bare numbers in the current display unit)
    - per-field display-unit dropdown (remembered per quantity kind)
    - sends SI values to `/api/params/edit`
    - shows the EditResult issues inline (reject = red with an explanation, warn = amber)
    - shows a provenance badge (measured/datasheet/estimated/derived)
    - shows a locked state for derived fields, with a "why locked?" tooltip naming the source field

    - A **normalization layer** before math.js: Unicode → ASCII (`°C`→`degC`, `N·m`→`N*m`, `Ω`→`ohm`, `µ`→`u`).
    - Register missing units with `math.createUnit` (e.g. `rpm` if absent, `kgf`). Handle the affine `°C` explicitly.

    Test against `schemas/units-fixture.yaml` (the same fixture as Rust).
  - **Files:** `web/src/components/QuantityInput.tsx`, `web/src/lib/units.ts`
  - **Verify:** Vitest fixture test passes 100 %; component tests for reject/warn rendering.
  - **Done when:** Passing.

- [ ] **P13.T07** — Schema-driven parameter panel
  - **Depends:** P13.T05, P13.T06
  - **Do:** Render `GET /api/params` as collapsible sections (motor/electrical, …) with the right widget per type (quantity, enum select, bool switch, slider for live-tweak-friendly params, nested objects). Every field has a `<HelpTip>`. Search/filter box. Live updates via `ParamChanged` events, with a brief highlight when changed by another source (e.g. "changed by agent").
  - **Files:** `web/src/panels/params/*`
  - **Verify:** Playwright: edit Kv → Kt and λ update and are locked; enter an invalid pole/slot combo → rejected with an explanation; an MCP edit appears highlighted in the UI.
  - **Done when:** Passing.

- [ ] **P13.T08** — Status bar
  - **Depends:** P13.T02
  - **Do:** Shows sim time, sim/real ratio (colored when lagging), fidelity tier + dt, energy-balance indicator (green/amber/red with the residual in its tooltip), active faults/protections chips, warnings count, and connection state. Each item has a help tip.
  - **Files:** `web/src/layout/StatusBar.tsx`
  - **Verify:** Playwright: inject a fault via the API → the chip appears; stop the server → disconnected state; restart → recovers.
  - **Done when:** Passing.

- [ ] **P13.T09** — Clock toolbar
  - **Depends:** P13.T02
  - **Do:** Play/pause/step (with a step-size selector: 1 event, 1 PWM period, 1 ms, 10 ms)/reset; log-scale time-scale slider (1e-4× … 1× … max) with presets (1/1000, 1/100, 1/10, 1×, max); fidelity selector; snapshot save/restore menu. Keyboard shortcuts (space = play/pause, `.` = step).
  - **Files:** `web/src/layout/ClockToolbar.tsx`
  - **Verify:** Playwright: slow motion at 1/1000 → the sim time advances ≈ 1 ms per wall second (±20 %).
  - **Done when:** Passing.

- [ ] **P13.T10** — Handle server `ui_command`s
  - **Depends:** P13.T04
  - **Do:** Implement `focus`, `open_panel`, `highlight` (orange pulse outline), `set_layout`, `toast`, and `open_lesson` (stub until P18) by resolving `data-agent-id` targets.
  - **Files:** `web/src/agent/uiCommands.ts`
  - **Verify:** Playwright + REST: POST `/api/ui/command {highlight: "param:motor.electrical.kv"}` → the element is highlighted and scrolled into view.
  - **Done when:** Passing.

- [ ] **P13.T12** — Predictable URL state & deep links
  - **Depends:** P13.T03, P13.T10
  - **Do:** The URL reflects the navigable UI state so agents and users can link to it: `?layout=tuning&panel=params&focus=param:motor.electrical.kv&lesson=<id>`. On load, apply it (via the same code as `ui_command`s). On change, update it with `history.replaceState` (no reload). Document the URL scheme in `docs/docs/agents.md` (stub until P18.T11).
  - **Files:** `web/src/agent/urlState.ts`
  - **Verify:** Playwright: open a deep link → the right layout and panel open and the param is focused; change the layout → the URL updates.
  - **Done when:** Passing.

- [ ] **P13.T11** — Phase gate
  - **Depends:** P13.T01, P13.T02, P13.T03, P13.T04, P13.T05, P13.T06, P13.T07, P13.T08, P13.T09, P13.T10, P13.T12
  - **Do:** PLAN §8 checklist. Include screenshots of both themes in the LOG entry (paths).
  - **Done when:** Tagged `phase-13-done`.
