# BLDC Motor Simulator — Polished Idea

> Status: **idea complete** → next step is planning, task breakdown and tool selection.
> Source: [Raw_Idea.md](Raw_Idea.md), refined through Q&A on 2026-10-08.

---

## 1. Vision

A local, browser-based **simulator and learning lab for brushless permanent-magnet motors**, focused on
**high-torque, low-RPM motors** (gimbal motors, robot actuators, 6020 / 8010 / 10015-class motors).

It must let a beginner:

1. **Learn** how BLDC/PMSM motors work, what each parameter means, and how it affects behavior.
2. **Reproduce** the behavior of a real motor from its datasheet or parameters.
3. **Play** with the motor live: drive it with different controllers, power supplies, loads and sensors, and see what happens, including what goes wrong.

The physics must be **correct and validated** (tests plus live self-consistency checks), not just "looks right".
Every adjustable field explains itself through a `?` tooltip that links to full docs.
The whole app must be easy for an **AI agent** to drive.

---

## 2. Core principles

| Principle | Meaning |
|---|---|
| **Correct first** | Equations come from established references. Every formula is unit-tested, cross-checked and documented. Conventions (phase vs line-to-line, peak vs RMS, amplitude- vs power-invariant transforms) are explicit and consistent. |
| **Explain everything** | Every input, selector and plotted signal has a `?` with a short but complete explanation and a link to the full doc page. |
| **Consistent by construction** | It is impossible to enter physically contradictory motor parameters (see §4.3). |
| **Live & tactile** | Everything can be changed while the simulation runs. You can drag targets and push loads with the mouse. |
| **Agent-native** | Every UI action is also an API call, exposed via MCP. The UI uses stable semantic IDs. |
| **Honest about fidelity** | The active model fidelity and the sim-time / real-time ratio are always visible. |

---

## 3. Motor & physics model

### 3.1 Motor types
- **One unified permanent-magnet brushless model** covering:
  - **BLDC**: trapezoidal back-EMF, typically six-step driven.
  - **PMSM**: sinusoidal back-EMF, typically FOC driven.
  - Real motors sit in between, so the **back-EMF shape is a parameter** (trapezoidal ↔ sinusoidal, or a harmonic table).
- **Inner-rotor (inrunner)** and **outer-rotor (outrunner)** topologies.
- Future (out of scope for v1): brushed DC as a reference baseline, hybrid stepper.

### 3.2 Fidelity tiers (user-selectable, shown in the UI)
| Tier | Contents | Use |
|---|---|---|
| **Ideal** | Textbook dq/abc equations, no losses beyond R | Learning the basics |
| **Standard** (default) | + back-EMF shape, cogging torque, Coulomb + viscous friction, iron losses, thermal model | Realistic behavior that matches datasheets |
| **Detailed** | + magnetic saturation (L vs current), switching-level inverter, sensor non-idealities at full detail | Close-to-hardware studies, slower |
| *(future)* FEM | Offline field-level model hook | Architecture kept open, not v1 |

Always visible: **active tier**, **solver step**, **sim-time / real-time ratio** (e.g. `0.42× real time`).

### 3.3 Mechanical model
- Rotor inertia (computed from geometry or entered).
- **Gearbox / transmission** (optional, bypassable): ratio, efficiency, backlash, gearbox inertia. Planetary and cycloidal presets.
- **Loads** (combinable):
  - **Arm + mass + gravity**: arm length, arm mass, point mass at an adjustable distance, mounting angle. Gravity torque changes with angle. Load inertia is included automatically.
  - **Friction / viscous / constant torque**.
- Future: torsional spring / compliant coupling, custom torque profiles.

### 3.4 Thermal model
- Lumped thermal network: winding → stator → housing → ambient.
- Winding resistance rises with temperature. Magnet strength (Kt) drops with temperature.
- Continuous vs peak torque limits come out of the model. Overheat warnings.
- Key lesson for low-RPM use: **holding torque at standstill heats the motor.**

### 3.5 Power electronics (inverter / driver)
- **Selectable model**:
  - **Averaged**: ideal average phase voltages, fast and smooth.
  - **Switching**: real PWM with configurable frequency, dead-time, current ripple, MOSFET conduction and switching losses.
- Configurable PWM frequency, modulation (SPWM / SVPWM), dead-time, MOSFET parameters.

### 3.6 Power supply
- **Lab PSU (CV/CC)**: set voltage, current limit. It sags or folds back when the limit is hit.
- **Battery pack**: chemistry, cell count (S/P), internal resistance, state of charge, voltage sag.
- **DC bus**: bus capacitance, **regeneration** (braking raises the bus voltage), optional **brake resistor / chopper**, overvoltage behavior.
- The motor can be run at any voltage, including above or below its rating, with warnings shown.

### 3.7 Sensors
Every sensor has: **resolution, noise, latency, update rate**, and can be bypassed (ideal feedback).
- **Hall sensors**: 3 digital, 60° electrical resolution, placement error.
- **Encoders**:
  - **On-axis magnetic encoder**: a diametric magnet glued to the shaft end and a single chip on a PCB. Presets for typical chips (e.g. AS5600 12-bit, AS5047P 14-bit, MT6701). Models air gap, magnet misalignment (eccentricity error), nonlinearity, bit resolution.
  - Optical / incremental encoder (CPR).
- **Current & voltage sensing**: shunt + ADC with bits, gain, offset, noise, sampling instant.
- **Sensorless observer**: back-EMF-based position estimate. It is expected to fail at low speed, which is educational.

---

## 4. Motors, presets & parameters

### 4.1 Preset library
- **Generic geometry classes**, not specific products: e.g. 2804, 4108, 5010, 6010, 6020, 8010, 8108, 10015, 12020…
  (Naming convention: `DDHH` = stator diameter (mm) × stator stack height (mm). Explained in the docs.)
- Each class has typical values: pole pairs, slot count, Kv/Kt, R, L, inertia, mass, thermal constants, rated/peak current.
- Both inrunner and outrunner variants where meaningful. Emphasis on torque / low-RPM classes.
- Presets also exist for: controllers, encoders, power supplies, gearboxes, loads, full **scenes** (motor + driver + supply + load + controller).

### 4.2 User models
- Create new, **duplicate & modify** an existing preset, or import a datasheet (§4.4).
- **Storage**: YAML files validated against a schema.
  - Built-in presets: read-only, in the app folder.
  - User models: user data folder.
  - Extra library folders: configurable (e.g. a shared/team folder or a git repo).
- Each file records **provenance** per parameter: `measured` / `datasheet` / `estimated` / `derived`.

### 4.3 Cross-constrained parameters
Parameters form a **dependency graph** of independent inputs and derived values:
- Electrical constants: **Kv ↔ Ke ↔ Kt ↔ flux linkage λ** are one quantity in different forms. Editing one updates the others. The convention used is shown.
- Topology: pole count must be even. Slot/pole combination must be valid for 3-phase windings. Winding factor is computed.
- Geometry → mass, rotor inertia and thermal mass estimates (overridable, flagged as estimates).
- Ratings must be consistent with the thermal model (e.g. rated current vs continuous thermal limit).
- UI behavior: derived fields are shown as computed/locked. Out-of-range or contradictory input is **rejected with an explanation**. Plausible-but-unusual values give a **warning**, not a block.

### 4.4 Datasheet import wizard
- Enter what a seller provides (Kv or Kt, phase/line resistance, pole count, weight, size class, rated torque/current/voltage…).
- The wizard asks for the convention when it is ambiguous (e.g. "Is this resistance phase-to-phase?"), fills in the missing values with physically consistent estimates, and marks each value's confidence.
- Result: a ready-to-simulate user model.

---

## 5. Control system

### 5.1 Control-loop diagram
A standard **block diagram**, rendered in the conventional control-theory style:

```
Setpoint ─► (Σ) ─► Controller ─► Modulator/Driver ─► Inverter ─► Motor ─► Gearbox ─► Load
             ▲                                                     │
             └────────── Sensors / Estimator (feedback) ◄──────────┘
```

- Every block is **configurable**, **bypassable** (ideal pass-through) where meaningful, and has its own **execution rate**:
  - Controller loop frequencies (e.g. current loop 20 kHz, velocity 5 kHz, position 1 kHz, as on a real MCU).
  - PWM frequency (inverter), sensor update rates and latencies.
- Plus an optional **custom controller block**: user-written code with a defined input/output interface, run in a sandbox at its configured rate.
- Users can click any wire in the diagram to plot that signal.

### 5.2 Controller presets
| Preset | Notes |
|---|---|
| **Six-step + halls** | Simplest BLDC drive. Visible torque ripple. |
| **Open-loop sinusoidal** | Rotates a voltage vector blindly, stepper-like. Can lose sync. |
| **FOC, cascaded PI** | Current (torque) → velocity → position loops. Each loop can be enabled or disabled. Sensored or sensorless. |
| **Impedance / MIT-mode** | `τ = Kp·(θ* − θ) + Kd·(ω* − ω) + τ_ff`. Robot-actuator style, ideal for holding angles under load. |

Each preset comes in variants with different gain sets (soft / medium / stiff).
Configurable limits: current limit, velocity limit, voltage limit, ramp rates.

### 5.3 Tuning
- **Auto-tune**: measures the plant and suggests gains for the current/velocity/position loops.
- Future: step-response metrics, Bode / frequency response, motor parameter identification routine.

### 5.4 Faults & protection
- Live fault injection: phase disconnect, encoder dropout/glitch, wrong pole-pair setting, wrong encoder offset, stalled rotor, PSU current limit hit, sensor noise burst.
- Protection logic: overcurrent, overvoltage (regen), undervoltage, overtemperature. Each can be configured or disabled.

---

## 6. Simulation runtime & experiments

### 6.1 Simulation clock
- Independent sim clock: **play / pause / single-step / reset**.
- **Time-scale slider**: slow motion (e.g. 1000× slower, to watch individual PWM cycles) ↔ real time ↔ as fast as possible.
- **Snapshots**: save and restore the full simulation state.
- Multi-rate execution: physics solver step, plus discrete blocks running at their own frequencies.

### 6.2 Live playground (primary mode)
- Change any parameter **while running**: load mass, load distance, supply voltage, gains, limits…
- **Direct manipulation** in the visualization:
  - drag the **target angle** (servo mode), or set target speed/torque,
  - **push/flick the arm** with the mouse to apply a disturbance torque.

### 6.3 Scripted scenarios
- A timeline of actions, e.g. `t=0 speed 100 rpm → t=2 s add 1 kg @ 10 cm → t=4 s hold 90°`.
- Saved as files (YAML), replayable, shareable. Scenarios are the basis for lessons and tests.

### 6.4 Recording & export
- Record any set of signals. Export to **CSV / JSON**.
- Future: overlay/compare runs, automatic characterization sweeps (torque-speed curve, efficiency map).

---

## 7. User interface

### 7.1 Main views
- **Motor visualization (real time)**:
  - Motor cross-section geometry: stator, slots, coils, magnets, rotor (inner or outer).
  - Which coils are energized, and their polarity.
  - **Current flow shown as moving particles**: speed and density proportional to current, direction from the sign.
  - Rotor rotation, field/current vector overlay (optional).
  - **Load arm and mass**, moving dynamically, with gravity shown.
  - Zoom, pan, and slow motion via the sim clock.
- **Live plots**: any signal, multiple panels, adjustable time window, pause/scroll, cursors.
- **Control-loop diagram editor** (§5.1).
- **Parameter panels** with `?` tooltips, units, provenance badges.
- **Library browser** for motors, scenes, scenarios and presets.
- **Status bar**: sim time, sim/real ratio, fidelity tier, energy balance, active faults/protections.

### 7.2 Help system
- `?` next to every field, selector and signal: a short but complete explanation, plus a link to the full doc page.

### 7.3 Units
- SI internally.
- Display toggle per field: rpm ↔ rad/s, deg ↔ rad, N·m ↔ kgf·cm, kg·m² ↔ kg·cm², etc.
- **Unit-aware input**: typing `300rpm` or `2 kgf*cm` just works.

### 7.4 Design
- Claude-like palette: warm neutrals, **orange accent**, **dark & light themes**.
- Smooth, modern, uncluttered UI/UX. Panels are resizable and rearrangeable.

### 7.5 Guided lessons
- A lesson is a doc page, a preloaded scene/scenario, and step-by-step prompts ("now increase the load, watch the current plot").
- Example lessons: *What is back-EMF?*, *Kv vs Kt*, *Why FOC beats six-step*, *Tuning your first PI loop*, *Why holding torque heats the motor*, *What regen does to your power supply*.

---

## 8. AI-agent accessibility

- **Public API**: every UI action and state is available via a documented local API (HTTP + WebSocket for streaming). It covers reading state and signals, setting parameters, loading presets, running scenarios, fetching recorded data and controlling the sim clock.
- **MCP server** wrapping that API, so an agent can drive the *currently open* app instance directly ("enable fault X", "what's the winding temperature?", "set load to 2 kg at 15 cm").
- **UI is agent-friendly**: stable semantic element IDs, accessible labels, and a URL/state structure that is predictable to navigate.
- The UI and the API share **one source of truth**, so a change made by an agent is visible live in the UI, and vice versa.

---

## 9. Documentation

- **Markdown files in the repo** (readable by agents directly), built into a **searchable static docs site** with LaTeX math and diagrams.
- An **`llms.txt`** index for AI agents.
- Sections:
  - **Getting started & UI guide**
  - **Motor fundamentals**: BLDC vs PMSM, poles/slots, Kv/Kt/Ke, back-EMF, cogging, inrunner vs outrunner, geometry naming
  - **Physics & math**: the exact equations used, conventions, assumptions, fidelity tiers
  - **Power electronics**: inverter, PWM, SVPWM, dead-time, power supplies, regen
  - **Sensors**: halls, magnetic encoders, ADCs, observers
  - **Control theory**: block diagrams, PI/PID, cascaded loops, FOC, impedance control, tuning
  - **Lessons**
  - **API & MCP reference**
  - **File formats** (motor/scene/scenario YAML schemas)
  - **Validation report**: what is tested and how
- Every `?` tooltip deep-links to its doc section.

---

## 10. Correctness & validation

- Use established numerical libraries/solvers. Do not hand-roll an ODE integrator without validating it.
- **Unit tests for every formula**, with documented references.
- **Analytic-solution tests**, e.g. locked-rotor current step = RL time constant; no-load speed = Kv·V_LL,peak; stall torque = Kt·i_q. Each formula states its voltage/current convention (see the validation catalog, P02.T12).
- **Energy-conservation checks**: electrical in = mechanical out + copper/iron/switching/friction losses + stored energy. Shown **live in the UI** as an energy-balance indicator.
- **Cross-check** against an independent reference implementation of the core equations.
- **Datasheet regression tests**: generic presets must reproduce typical published curves within tolerance.
- **Scenario-based regression tests**: saved scenarios with expected outputs.

---

## 11. Deployment

- **Local web app**: started with one command, opened in a browser at `localhost`.
- Reads and writes model/scenario files on the local disk.
- The API and MCP server run alongside the app.

---

## 12. Scope summary

### In scope (v1)
Unified BLDC/PMSM model (inner/outer rotor) · fidelity tiers · thermal · gearbox · arm+mass+gravity and friction loads ·
selectable inverter model · lab PSU / battery / regen + brake resistor · halls, magnetic & optical encoders, ADCs, sensorless observer ·
4 controller families with presets · block-diagram editor + custom code block · auto-tune · fault injection & protection ·
sim clock with time scaling & snapshots · live playground with direct manipulation · scripted scenarios · CSV/JSON export ·
datasheet wizard · cross-constrained parameters · generic-size preset library (YAML) · real-time visualization with current particles ·
live plots · `?` help everywhere · unit-aware inputs · guided lessons · API + MCP · Markdown docs site + llms.txt ·
full validation suite + live energy balance · Claude-style dark/light UI.

### Future / out of scope for v1
FEM-level magnetics · brushed DC & stepper motors · spring/compliant & custom-profile loads · step-response metrics ·
Bode analysis · parameter-ID routine · run comparison overlays · characterization sweeps · hosted multi-user deployment ·
specific commercial motor presets.

---

## 13. Open questions for the planning phase

1. **Where the simulation runs**: browser-side (TypeScript/WASM) vs a local backend (e.g. Python with scientific libraries). This affects performance, how validation is done, and agent API design.
2. **Solver choice**: fixed-step vs adaptive; how to handle multi-rate discrete blocks and switching-level PWM efficiently.
3. **Visualization tech**: 2D canvas/WebGL cross-section (proposed primary) vs an additional 3D view.
4. **Custom controller block language & sandboxing.**
5. **Docs tooling** (e.g. MkDocs Material vs Docusaurus) and how docs pages map to in-app tooltips.
6. Reference sources for equations, generic preset values and validation data.

---

## 14. Process notes

- Next steps: **planning → full task breakdown → tool selection → implementation → testing.**
- During implementation, use the **graphify** skill to build and query a knowledge graph of the codebase and docs.
