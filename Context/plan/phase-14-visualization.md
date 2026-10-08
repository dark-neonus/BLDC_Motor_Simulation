# Phase 14 — Motor visualization (PixiJS)

> **Goal:** the real-time motor cross-section described in POLISHED_IDEA §7.1:
> - geometry generated from parameters (inner/outer rotor, slots, coils by phase, magnets)
> - coil energization and polarity
> - **current particles**
> - rotating rotor
> - load arm with mass and gravity
> - vector overlays
> - direct manipulation (drag the target angle, push/flick the arm)
>
> **Depends on:** P13.
> **Read first:** POLISHED_IDEA §6.2, §7.1; `docs/docs/api/websocket.md` (viz frames); `crates/sim-model/src/winding.rs` (layout API).
> **Exit criteria:**
> - The visualization renders any motor preset correctly (slot/pole count, inner/outer).
> - Particles reflect current magnitude and sign.
> - The arm moves with gravity.
> - Dragging the target angle moves the servo.
> - Flicking the arm applies a disturbance.
> - ≥ 50 fps with 10k particles on the dev machine (measured).

- [ ] **P14.T01** — Geometry data endpoint + pure geometry builder
  - **Depends:** P13
  - **Do:**
    - Add `GET /api/viz/geometry` returning dimensions (stator OD/ID, tooth/slot shape params, airgap, magnet arc/thickness, rotor dims, topology) and the **winding layout** from `sim-model::winding` (coil → slots, phase, polarity). This is the single source; the UI never recomputes windings.
    - In TS, `buildGeometry(geo) → { teeth[], slots[], coils[] (with path polylines for particles), magnets[], rotorShapes }` as pure, unit-tested functions.
  - **Files:** `crates/sim-api/src/routes/viz.rs`, `web/src/viz/geometry.ts`
  - **Verify:** Vitest: 12N14P outrunner → 12 teeth, 14 magnets, coil phase order matches the API; inner-rotor variant mirrors radially.
  - **Done when:** Passing.

- [ ] **P14.T02** — Pixi scene: stator, coils, magnets, rotor rotation
  - **Depends:** P14.T01
  - **Do:**
    - Pixi `Application` in an imperative module with a thin React wrapper; resizes with the Dockview panel.
    - Layers: stator iron, coils colored by phase (A/B/C palette consistent with the plots), magnets N/S (two theme-aware colors), rotor rotating from viz frames.
    - Interpolate the angle between frames for smoothness. Slow motion uses the frame times correctly.
    - Theme-aware colors from CSS variables.
  - **Files:** `web/src/viz/{scene,layers}.ts`, `web/src/panels/viz/VizPanel.tsx`
  - **Verify:** Manual on 3 presets. Playwright reads `window.__bldcViz.state` (rotor angle, coil count) and asserts it against the API.
  - **Done when:** Passing.

- [ ] **P14.T03** — Coil energization & inverter leg states
  - **Depends:** P14.T02
  - **Do:** Coil fill intensity ∝ |i_phase|/I_peak; polarity markers (⊙/⊗ or arrows) from the current sign × coil polarity. A small inverter schematic overlay (3 legs, high/low/off states highlighted) — collapsible. Floating phases are shown distinctly.
  - **Files:** `web/src/viz/coils.ts`, `web/src/viz/inverterOverlay.ts`
  - **Verify:** Six-step demo at 1/1000 time scale: exactly two phases are energized per sector and the floating one is marked (Playwright reads the overlay state).
  - **Done when:** Passing.

- [ ] **P14.T04** — Current particles
  - **Depends:** P14.T03
  - **Do:**
    - A particle system with pooled sprites (ParticleContainer). Particles travel along each coil's path polyline.
    - Speed ∝ i (signed, so the direction reverses with the current); density ∝ |i| (spawn/despawn to the target count).
    - Particle speed scales with the sim time scale, so slow motion shows slow particles and the visual time is consistent.
    - Cap the total particles (setting; default 10k).
    - A "particles off" toggle for low-end machines.
  - **Files:** `web/src/viz/particles.ts`
  - **Verify:** Vitest for the kinematics (position along the path, direction reversal). A Playwright perf probe reads the fps from `window.__bldcViz.fps` for 5 s at 10k particles and logs it (assert ≥ 30 in headless swiftshader; record the real-GPU number manually in LOG).
  - **Done when:** Passing + the manual fps is noted.

- [ ] **P14.T05** — Load arm, mass, gravity, gearbox indicator
  - **Depends:** P14.T02
  - **Do:** Draw the output shaft, arm (length to scale with a zoom-aware scale bar), mass circle sized by mass, gravity arrow, and target-angle ghost. When a gearbox is present, show a gear icon with the ratio and draw the arm at the **load-side** angle. Shows the live torque on the arm (optional label).
  - **Files:** `web/src/viz/load.ts`
  - **Verify:** Playwright: a pendulum swing with no power → the arm angle oscillates; the readout matches the `load.theta` signal.
  - **Done when:** Passing.

- [ ] **P14.T06** — Direct manipulation: drag target, push/flick arm
  - **Depends:** P14.T05
  - **Do:**
    - **Drag target handle** (servo/MIT modes) → throttled `POST /api/ctrl/setpoint` (position) while dragging.
    - **Push/flick the arm:** pointer drag on the arm → on release compute the angular velocity of the gesture → `POST /api/load/disturbance` with a torque pulse sized to impart a proportional Δω (documented formula, clamped). Hold-and-push applies a continuous torque while held.
    - Cursor and help tooltips explain both. Every handle has a `data-agent-id`.
  - **Files:** `web/src/viz/interaction.ts`
  - **Verify:** Playwright: drag the handle to 90° → the setpoint becomes 90° and the arm follows; flick → a disturbance event is observed and the arm reacts.
  - **Done when:** Passing.

- [ ] **P14.T07** — Overlays: vectors and sensor ghosts
  - **Depends:** P14.T02
  - **Do:** Toggles:
    - stator current vector (from i_α, i_β) and the rotor d-axis
    - the encoder-measured angle as a ghost rotor marker vs the true angle (shows quantization, latency and faults)
    - the observer angle in sensorless mode
    - the hall sector wedge
  - **Files:** `web/src/viz/overlays.ts`
  - **Verify:** Playwright: with the wrong-offset fault on, the ghost marker offset equals the fault offset (read from the exposed state).
  - **Done when:** Passing.

- [ ] **P14.T08** — Zoom/pan, hover help, performance pass
  - **Depends:** P14.T04, P14.T06, P14.T07
  - **Do:** Wheel zoom, drag pan (empty space), fit button. Hover tooltips on parts (tooth, coil, magnet, arm → help ids). Profile with the Chrome performance panel: no per-frame allocations in hot paths, and the render is decoupled from WS frame arrival.
  - **Files:** `web/src/viz/*`
  - **Verify:** 👤 Ask the user to confirm the fps counter shows ≥ 50 at 10k particles on their GPU; record their answer in LOG.
  - **Done when:** Met, or a P19 optimization task is added with numbers.

- [ ] **P14.T09** — Phase gate
  - **Depends:** P14.T01, P14.T02, P14.T03, P14.T04, P14.T05, P14.T06, P14.T07, P14.T08
  - **Do:** PLAN §8 checklist + a short screen recording or screenshots in `docs/static/img/ui/`.
  - **Done when:** Tagged `phase-14-done`.
