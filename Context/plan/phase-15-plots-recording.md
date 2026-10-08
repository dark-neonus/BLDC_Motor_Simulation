# Phase 15 — Live plots & recording

> **Goal:** fast multi-panel live plotting of any signal with cursors and time-window control, a reusable "plot this signal" hook, and recording/export UI.
> **Depends on:** P13.
> **Read first:** POLISHED_IDEA §6.4, §7.1; `docs/docs/api/websocket.md`; P03.T10 (decimation).
> **Exit criteria:** Plot any signal (searchable picker with help) in ≥ 4 synced panels at 60 fps. Cursors read values. Pause/scroll works. Recordings export to CSV/JSON with the full rate and correct units in the headers.

- [ ] **P15.T01** — Plot panel with uPlot, multiple synced panels
  - **Depends:** P13
  - **Do:**
    - Imperative uPlot module + React wrapper. A panel holds N stacked plots with a synced x-axis (time) and synced cursor.
    - Series colors from the theme (phase A/B/C consistent with the viz).
    - The y-axis label shows the display unit. Each series' legend has a `<HelpTip>`.
  - **Files:** `web/src/plots/*`, `web/src/panels/plots/PlotsPanel.tsx`
  - **Verify:** Playwright: add 4 plots with 3 signals each; the fps probe ≥ 30 headless.
  - **Done when:** Passing.

- [ ] **P15.T02** — Signal picker
  - **Depends:** P15.T01
  - **Do:** Searchable list from `/api/signals` grouped by root (motor, ctrl, sensors …), showing unit and description, drag-to-plot or click-to-add. Subscribes via the WS subscription manager (ref-counted).
  - **Files:** `web/src/plots/SignalPicker.tsx`
  - **Verify:** Vitest + Playwright: adding/removing signals changes the WS subscriptions correctly.
  - **Done when:** Passing.

- [ ] **P15.T03** — Streaming buffers, time window, pause/scroll, cursors
  - **Depends:** P15.T01
  - **Do:**
    - Client ring buffers from the min/max frames, drawn as an envelope band + mean line when decimated.
    - Window presets (100 ms, 1 s, 10 s, 60 s); **auto window** = time-scale aware.
    - Pause plotting (independent of the sim) and scroll back through the buffer.
    - Cursors: two vertical cursors with Δt and value readouts per series.
  - **Files:** `web/src/plots/buffer.ts`, `web/src/plots/cursors.ts`
  - **Verify:** Vitest for the buffer and decimation rendering logic; a Playwright cursor readout matches the value from `read_signals`.
  - **Done when:** Passing.

- [ ] **P15.T04** — "Plot this signal" hook (for other panels)
  - **Depends:** P15.T02
  - **Do:** `usePlotSignal()` → `plot(path, {panel?})`, used by the control diagram wires (P16), param panel signals and lessons. If the plots panel is closed, open it.
  - **Files:** `web/src/plots/usePlotSignal.ts`
  - **Verify:** Vitest.
  - **Done when:** Passing.

- [ ] **P15.T05** — Recording & export UI
  - **Depends:** P15.T02
  - **Do:** A record button with a signal selection (default: the plotted signals), rate mode (full / decimated) and a memory estimate. Stop → list of recordings → export CSV/JSON download (via the API) or save to a path. CSV headers include units (`motor.i_a [A]`).
  - **Files:** `web/src/panels/plots/Recording.tsx`
  - **Verify:** Playwright: record 1 s → export CSV → parse it; the row count matches the expected rate.
  - **Done when:** Passing.

- [ ] **P15.T06** — Plot presets per layout/lesson
  - **Depends:** P15.T03
  - **Do:** Named plot configurations ("FOC currents", "Speed loop", "Thermal", "Bus & supply", "Encoder error") selectable from a menu and referenced by lessons. Stored as YAML in `presets/plots/`.
  - **Files:** `presets/plots/*.yaml`, `web/src/plots/presets.ts`
  - **Verify:** Playwright: select a preset → the expected signals are plotted.
  - **Done when:** Passing.

- [ ] **P15.T07** — Phase gate
  - **Depends:** P15.T01, P15.T02, P15.T03, P15.T04, P15.T05, P15.T06
  - **Do:** PLAN §8 checklist.
  - **Done when:** Tagged `phase-15-done`.
