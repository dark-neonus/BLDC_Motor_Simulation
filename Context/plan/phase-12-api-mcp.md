# Phase 12 — Full API, streaming & MCP

> **Goal:**
> - A complete, documented control surface over the single live engine (D-006): REST for commands and queries, WebSocket for signal/viz/event streams, a server→UI command channel, and MCP tools that map 1:1 to the API.
> - Everything the UI can do, an agent can do.
>
> **Depends on:** P10.
> **Read first:** POLISHED_IDEA §8, [CONVENTIONS §3](CONVENTIONS.md), [DECISIONS D-006](DECISIONS.md), P01 skeleton routes (keep their paths).
> **Exit criteria:**
> - The OpenAPI spec is generated and served.
> - The integration tests cover every endpoint and MCP tool.
> - A scripted MCP session can load a scene, set params, run, inject a fault and read signals, and a connected WS client sees every change as events.

- [ ] **P12.T01** — API design document
  - **Depends:** P10
  - **Do:** Write `docs/docs/api/overview.md`:
    - resources and routes
    - error format (`{error: {code, message, path?, help_id?}}`)
    - WS message types
    - the event model
    - the MCP tool list with the mapping to routes

    Review it against POLISHED_IDEA §8 (every UI action exists). Add a decision if anything deviates from the skeleton paths.
  - **Files:** `docs/docs/api/overview.md`
  - **Verify:** A table maps each POLISHED_IDEA feature to an endpoint/tool.
  - **Done when:** Complete.

- [ ] **P12.T02** — REST: sim, params, faults, protections
  - **Depends:** P12.T01
  - **Do:** Routes:
    - Sim control: `/api/sim/{play,pause,step,reset,time_scale,fidelity}`, `/api/sim/snapshots` (list/save/restore/export)
    - Params: `GET /api/params` (tree with values, units, provenance, locked/derived flags, help ids), `POST /api/params/edit {path, value}` → `EditResult` (P04.T05)
    - Setpoints: `/api/ctrl/setpoint {kind, value, ramp}`
    - Disturbance: `/api/load/disturbance`
    - Faults and protections: `/api/faults`, `/api/protections` (list/toggle/configure)
    - Warnings: `/api/warnings`
  - **Files:** `crates/sim-api/src/routes/*.rs`
  - **Verify:** Integration tests per route (reqwest).
  - **Done when:** Passing.

- [ ] **P12.T03** — REST: library, scenes, scenarios, recording, wizard, auto-tune, help
  - **Depends:** P12.T02
  - **Do:** Routes:
    - Library: `/api/library` (list/get/save_as/duplicate/delete/rename), plus `/api/library/folders` (list/add/remove the extra library folders; persisted to `~/.config/bldc-sim/config.yaml`)
    - Scene: `/api/scene` (load by id, get current, save current as)
    - Scenarios: `/api/scenarios/run|stop|status` (runs on the **live** engine with a progress event; a headless batch variant runs on a separate instance and returns a recording id)
    - Recording: `/api/recording/start|stop|list|export?format=csv|json` (download) and `export_path` (save to disk)
    - Wizard: `/api/wizard/start|answer|finish`
    - Auto-tune: `/api/autotune/run|apply`
    - Help: `/api/help/{id}` (from the help registry, once P13 exists; until then returns the schema descriptions)
    - Signals: `/api/signals` (registry metadata)
  - **Files:** `crates/sim-api/src/routes/*.rs`
  - **Verify:** Integration tests per route; the library tests use a temp user dir.
  - **Done when:** Passing.

- [ ] **P12.T04** — OpenAPI generation & API docs UI
  - **Depends:** P12.T03
  - **Do:** Annotate the handlers with `utoipa`. `bldc-sim dump-openapi > web/src/api/openapi.json`. Serve `/api/openapi.json` and a docs UI at `/api/docs` (utoipa-scalar or swagger-ui). Implement `just gen-api` = dump-openapi + `openapi-typescript` → `web/src/api/schema.d.ts`. CI check: no diff after regenerating.
  - **Files:** `crates/sim-api/src/openapi.rs`, `justfile`
  - **Verify:** `just gen-api && git diff --exit-code web/src/api/`.
  - **Done when:** Passing.

- [ ] **P12.T05** — WebSocket protocol v1
  - **Depends:** P12.T02
  - **Do:** `/api/stream` with MessagePack messages.
    - Client → server: `subscribe {signals[], max_rate_hz}`, `unsubscribe`, `viz {enabled, fps}`.
    - Server → client:
      - `signals` frames (decimated min/max from P03.T10)
      - `viz` frames at the requested fps (rotor angle, phase currents, leg states, coil currents, arm angle, encoder angle, est angle, i_d/i_q vectors)
      - `event` (ParamChanged incl. source = ui|mcp|scenario, Fault, Protection, Warning, Scenario progress, NumericalFault)
      - `status` (time, ratio, tier, energy ok)
    - Versioned (`v: 1`). Slow-consumer handling: drop old frames, never block the engine.
  - **Files:** `crates/sim-api/src/ws/*.rs`, `docs/docs/api/websocket.md`
  - **Verify:** Integration tests: subscribe → frames arrive at ≈ the requested rate; a param edit via REST produces an event on the WS.
  - **Done when:** Passing.

- [ ] **P12.T06** — Server → UI command channel (for agents)
  - **Depends:** P12.T05
  - **Do:** `ui_command` WS message type: `focus {target: data-agent-id}`, `open_panel {panel}`, `highlight {target, ms}`, `set_layout {name}`, `open_lesson {id}`, `toast {text}`. REST route `/api/ui/command` broadcasts to connected UIs. Lets an agent show the user things.
  - **Files:** `crates/sim-api/src/routes/ui.rs`
  - **Verify:** Integration test: POST → the WS client receives a `ui_command`.
  - **Done when:** Passing (the UI handling comes in P13).

- [ ] **P12.T07** — MCP tools (complete)
  - **Depends:** P12.T03, P12.T05, P12.T06
  - **Do:** rmcp tools mirroring the API:
    - `get_status`, `sim_control`, `set_time_scale`, `set_fidelity`
    - `list_params`, `get_params {prefix}`, `edit_param {path, value}`
    - `set_setpoint`, `apply_disturbance`
    - `list_faults`, `set_fault`, `list_protections`, `configure_protection`, `get_warnings`
    - `library_list`, `library_get`, `library_save_as`, `library_duplicate`, `library_folders`, `load_scene`, `save_scene`
    - `run_scenario`, `scenario_status`
    - `record_start`, `record_stop`, `export_recording {path, format}`
    - `read_signals {signals, window_s, max_points}` (returns compact JSON + a summary: min/max/mean/last)
    - `list_signals`
    - `snapshot_save`, `snapshot_restore`
    - `autotune_run`, `autotune_apply`
    - `wizard_*`
    - `ui_command`
    - `get_help {id}` (tooltip text + docs URL)

    Tool descriptions are written for an LLM: what it does, units (SI, with examples like "rad/s; 100 rpm = 10.47"), and side effects. Also expose MCP resources: `bldc://docs/llms.txt`, `bldc://signals`, `bldc://params`.
  - **Files:** `crates/sim-api/src/mcp/*.rs`
  - **Verify:** rmcp client integration tests per tool. Manual check with `npx @modelcontextprotocol/inspector`.
  - **Done when:** Passing.

- [ ] **P12.T08** — Scripted agent journey test
  - **Depends:** P12.T07
  - **Do:** A Rust integration test acting like an agent over MCP: load scene `arm-servo` → set the load mass to 1 kg at 0.15 m → setpoint 90° → run 2 s → read signals (assert the arm angle is ≈ 90° within tolerance) → inject `ctrl.wrong_encoder_offset` → observe the torque-per-amp drop → reset. A WS client attached in parallel must see all the corresponding events.
  - **Files:** `crates/sim-api/tests/agent_journey.rs`
  - **Verify:** `cargo nextest run -p sim-api agent_journey`.
  - **Done when:** Passing.

- [ ] **P12.T09** — Phase gate
  - **Depends:** P12.T01, P12.T02, P12.T03, P12.T04, P12.T05, P12.T06, P12.T07, P12.T08
  - **Do:** PLAN §8 checklist. Update `.mcp.json` and `CLAUDE.md` with how an agent connects.
  - **Done when:** Tagged `phase-12-done`.
