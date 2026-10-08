# Phase 16 — Control-loop editor

> **Goal:** the conventional control-theory block diagram (Setpoint → Σ → Controller → Modulator → Inverter → Motor → Gearbox → Load, with Sensors/Estimator feedback) as an interactive React Flow view. Every block is configurable, bypassable and rate-settable; clicking a wire plots its signal. Also: controller preset selection, auto-tune UI, and the Luau custom-block editor.
> **Depends on:** P15.
> **Read first:** POLISHED_IDEA §5.1–5.3; `crates/sim-model` ControllerParams; `docs/docs/control/custom-block.md`.
> **Exit criteria:** The diagram reflects the current scene. Toggling a loop/sensor bypass changes the sim live. Wire click → plot. Auto-tune runs and applies with a diff preview. A Luau controller can be written, validated and enabled from the UI.

- [ ] **P16.T01** — Diagram from scene (React Flow, fixed topology)
  - **Depends:** P15
  - **Do:**
    - Build nodes/edges from the scene's component config (no free-form wiring — fixed topology per the user's decision).
    - Custom node components styled with the theme, showing the block name, a key parameter summary, a rate badge (e.g. "20 kHz") and a bypass state.
    - Edges are labeled with signal names. Auto-layout left→right with the feedback path below.
  - **Files:** `web/src/panels/control/*`
  - **Verify:** Playwright: for the FOC scene, the expected nodes exist with correct rates; for six-step, the nodes differ accordingly.
  - **Done when:** Passing.

- [ ] **P16.T02** — Block config side panel
  - **Depends:** P16.T01
  - **Do:** Selecting a node opens its parameters (reusing the schema-driven panel from P13.T07, filtered to the block's param prefix): rate, enable/bypass toggle, limits, gains, and help.
  - **Files:** `web/src/panels/control/BlockInspector.tsx`
  - **Verify:** Playwright: change the current-loop rate → the API param changes → the badge updates.
  - **Done when:** Passing.

- [ ] **P16.T03** — FOC cascade expansion
  - **Depends:** P16.T01
  - **Do:** The controller node expands (toggle) into position → velocity → current loop sub-blocks with their own enable switches and rates. Disabled loops are drawn as bypass wires.
  - **Files:** `web/src/panels/control/CascadeNode.tsx`
  - **Verify:** Playwright: disable the position loop → the bypass is drawn, and the sim behaves as P09.T01 specifies.
  - **Done when:** Passing.

- [ ] **P16.T04** — Wire click → plot
  - **Depends:** P16.T01
  - **Do:** Clicking an edge calls `usePlotSignal` with the edge's signal(s). Hovering shows the live value.
  - **Verify:** Playwright: click the `iq_ref` wire → the plot shows `ctrl.foc.iq_ref`.
  - **Done when:** Passing.

- [ ] **P16.T05** — Controller preset selector & auto-tune UI
  - **Depends:** P16.T02
  - **Do:**
    - **Preset selector:** family + variant (soft/medium/stiff), with an apply confirmation.
    - **Auto-tune dialog:**
      1. target bandwidths (defaults from P09.T11)
      2. run, with progress events
      3. a results table (measured R/L/J vs current params, proposed vs current gains, predicted bandwidths)
      4. apply/cancel
  - **Files:** `web/src/panels/control/{PresetPicker,AutotuneDialog}.tsx`
  - **Verify:** Playwright: run auto-tune on the default scene → apply → the gains change and the sim stays stable.
  - **Done when:** Passing.

- [ ] **P16.T06** — Luau custom block editor
  - **Depends:** P16.T02
  - **Do:**
    - A "Custom controller" node option. CodeMirror 6 editor with Lua highlighting and a template (PI example).
    - Declared inputs/outputs pickers (from the signal registry, restricted to the allowed writable signals) and a rate.
    - **Validate** (compile + one dry-run call via the API) with line-numbered errors.
    - Enable. Runtime errors show as an event + editor marker.
    - An API reference sidebar (from the docs page).
  - **Files:** `web/src/panels/control/LuauEditor.tsx`, API route `/api/ctrl/custom/validate`
  - **Verify:** Playwright: paste the PI template → validate OK → enable → the speed tracks; a syntax error shows the line number.
  - **Done when:** Passing.

- [ ] **P16.T07** — Phase gate
  - **Depends:** P16.T01, P16.T02, P16.T03, P16.T04, P16.T05, P16.T06
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-16-done`.
