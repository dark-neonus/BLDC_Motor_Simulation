# Phase 17 — Library, model editors, wizard, scenarios & faults UI

> **Goal:** the UI for managing everything that lives on disk and for experiments:
> - library browser
> - motor editor with constraints and provenance
> - datasheet import wizard
> - scene composer
> - live playground controls (supply/load/gearbox)
> - scenario timeline editor/runner
> - faults & protections panel
> - warnings toasts
>
> **Depends on:** P14, P16.
> **Read first:** POLISHED_IDEA §4, §5.4, §6.2–6.3; P04 (library, constraints, wizard), P12 routes.
> **Exit criteria:** A user can duplicate a builtin motor, edit it (with constraint enforcement), save it to the user library, build a scene with it, run a scenario with timeline actions and asserts, toggle faults live, and create a motor from datasheet numbers via the wizard. All through the UI and equally through MCP.

- [ ] **P17.T01** — Library browser
  - **Depends:** P14, P16
  - **Do:** Tree by kind (motors, scenes, scenarios, controllers, sensors, supplies, gearboxes, loads, lessons) and source (builtin 🔒 / user / extra folders). Search/filter by size class/topology. Actions: open, load into scene, duplicate to user, rename/delete (user only, with confirmation), reveal path. A "Manage folders" dialog adds/removes extra library folders (via `/api/library/folders`).
  - **Files:** `web/src/panels/library/*`
  - **Verify:** Playwright: duplicate `builtin:motors/8010-outrunner` → it appears under user; delete asks for confirmation; builtin delete is disabled.
  - **Done when:** Passing.

- [ ] **P17.T02** — Motor editor
  - **Depends:** P17.T01
  - **Do:**
    - Sectioned editor (identity, electrical, magnetic, winding, geometry, mechanical, thermal, ratings) reusing QuantityInput with constraint messages.
    - "Entered as" selector for the motor constant (Kv/Kt/Ke/λ), with the derived ones locked.
    - Convention notes, provenance badges (editable source + note).
    - **Live geometry preview** (reuses the P14 builder, static).
    - Derived performance summary: Kt, no-load speed at V, stall torque, continuous torque.
    - Save / save-as.
  - **Files:** `web/src/panels/editors/MotorEditor.tsx`
  - **Verify:** Playwright: change slots to an invalid combination → rejected with an explanation; change Kv → Kt updates; save → the file exists with the correct YAML (read via the API).
  - **Done when:** Passing.

- [ ] **P17.T03** — Datasheet import wizard UI
  - **Depends:** P17.T02
  - **Do:** A stepper:
    1. size class & topology
    2. enter known values (any subset), each with "where to find this on a datasheet" help
    3. convention questions from the backend
    4. review: a table of every parameter with value, source and confidence; estimated ones highlighted, overridable
    5. save to the user library → open in the editor or load into the scene
  - **Files:** `web/src/panels/editors/DatasheetWizard.tsx`
  - **Verify:** Playwright: enter the datasheet from the P04.T12 test → the result matches the backend test values.
  - **Done when:** Passing.

- [ ] **P17.T04** — Scene composer
  - **Depends:** P17.T01
  - **Do:** Choose components per slot (motor, inverter, supply, sensors, gearbox, load, controller) from the library, with inline overrides, fidelity tier and seed. Apply to the live engine (confirm if running) and save as a scene.
  - **Files:** `web/src/panels/editors/SceneComposer.tsx`
  - **Verify:** Playwright: switch the PSU → battery preset → the status shows the battery SoC signal available.
  - **Done when:** Passing.

- [ ] **P17.T05** — Playground controls (supply / load / gearbox quick panel)
  - **Depends:** P17.T04
  - **Do:** A compact "Playground" panel with sliders for the most-tweaked live params:
    - load mass, distance, arm length
    - supply voltage, current limit
    - setpoint (position/velocity/torque)
    - controller stiffness
    - time scale

    Each slider shows its value with units and a help tip, and is throttled to the API.
  - **Files:** `web/src/panels/playground/*`
  - **Verify:** Playwright: move the mass slider → the `load.mass` param event fires → the arm sags more under a P-only position loop (steady-state error increases).
  - **Done when:** Passing.

- [ ] **P17.T06** — Scenario timeline editor & runner
  - **Depends:** P17.T04
  - **Do:**
    - A list/timeline editor of actions (P04.T11 types) with add/edit/reorder, signal and param pickers, and assert editors.
    - Run on the live engine with a progress bar; timeline markers are drawn on the plots (vertical lines with labels).
    - Results: assert pass/fail list.
    - Save / save-as.
  - **Files:** `web/src/panels/scenario/*`
  - **Verify:** Playwright: build "t=0 target 90°, t=1 add 1 kg, t=2 assert |error| < 5°" → run → result shown; markers visible.
  - **Done when:** Passing.

- [ ] **P17.T07** — Faults & protections panel
  - **Depends:** P17.T01
  - **Do:** A list of faults with toggles and param fields (e.g. offset angle, burst duration), each with a "what to expect" help text. Protections with thresholds, enable and latch reset, and status lights.
  - **Files:** `web/src/panels/faults/*`
  - **Verify:** Playwright: toggle phase-open A → the status chip appears and the viz shows phase A dead.
  - **Done when:** Passing.

- [ ] **P17.T08** — Warnings & event toasts, event log panel
  - **Depends:** P17.T07
  - **Do:** Toasts for warnings/protection trips/numerical faults (deduped, with a help link). The `log` panel lists all events with source (ui/mcp/scenario) and time, and is filterable.
  - **Files:** `web/src/panels/log/*`, `web/src/components/Toasts.tsx`
  - **Verify:** Playwright: trigger OV in a regen scenario → a toast appears + a log entry.
  - **Done when:** Passing.

- [ ] **P17.T09** — Phase gate
  - **Depends:** P17.T01, P17.T02, P17.T03, P17.T04, P17.T05, P17.T06, P17.T07, P17.T08
  - **Do:** PLAN §8 checklist. Also re-run the P12.T08 agent journey with the UI open and confirm visually that every agent action is reflected.
  - **Done when:** Tagged `phase-17-done`.
