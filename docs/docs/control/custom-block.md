---
title: Custom controller (Luau)
---

# Custom controller block (Luau)

:::tip In plain words
You can write your own controller as a small [Luau](https://luau.org) script. The simulator calls it at a fixed rate with the signals you asked for, and writes back the signals it returns. A runaway script (endless loop, huge allocation) stops the simulation with an error instead of freezing it.
:::

## In a scene

```yaml
controller:
  family: custom
  rate: 20 kHz                                  # how often step() runs
  inputs: [ctrl.omega_ref, motor.omega]         # signals the script reads
  outputs: [ctrl.foc.iq_ref]                    # signals the script writes
  limits: { current: 4 A }
  script: |
    function init(p)
      local wv = 2 * math.pi * 20               -- EQ-CTRL-04 design at 20 Hz
      local kp = wv * p.j / p.kt
      return { i = 0, kp = kp, ki = kp * wv / 4, lim = p.i_max }
    end

    function step(inp, s, dt)
      local e = inp["ctrl.omega_ref"] - inp["motor.omega"]
      local us = s.kp * e + s.i
      local u = math.clamp(us, -s.lim, s.lim)
      if u == us then s.i = s.i + dt * s.ki * e end   -- clamping anti-windup
      return { ["ctrl.foc.iq_ref"] = u }
    end
```

The script runs on top of the FOC current loop (EQ-CTRL-03) at the same rate, so writing `ctrl.foc.iq_ref` / `ctrl.foc.id_ref` commands currents. If the outputs include `inverter.duty_a..c` or `ctrl.foc.v_alpha` / `ctrl.foc.v_beta`, no current loop is built: the script drives the duties (inverter) or the voltage (ideal source) itself.

## The API

| Function | Called | Returns |
|---|---|---|
| `init(params)` | once at build, and on reset | the state (any value; usually a table) |
| `step(inputs, state, dt)` | every `1/rate` seconds | a table of outputs, optionally a new state as a second value |

- `inputs` is a table keyed by the signal paths in `inputs:` (`inputs["motor.omega"]`). It is the same table every call; do not keep references to it.
- `outputs` keys are the paths in `outputs:`. A missing key leaves that signal unchanged; a non-number is an error.
- `dt` is the period in seconds.
- `state` is passed back unchanged unless `step` returns a second value. Tables are passed by reference, so updating fields in place works.
- `params` holds `i_max` (A), `ratio` (N), `kt` (N·m/A), `pole_pairs`, `r` (Ω), `ld`, `lq` (H), `lambda` (Wb) and `j` (kg·m², motor side): the controller's model of the plant.
- Signals are SI (rad, rad/s, A, V, N·m). Setpoints are load-side (`ctrl.omega_ref`, `ctrl.theta_ref`, `ctrl.torque_ref`), as for the built-in controllers.

## Limits and errors

- The VM is sandboxed (Luau `sandbox` mode: no file or OS access, read-only standard library).
- Memory limit: 16 MB. An allocation beyond it is an error.
- Instruction budget: 1 000 000 interrupts (loop iterations and calls) per call to `step` or `init`. Exceeding it is an error: an endless loop stops the run instead of hanging it.
- Errors stop the simulation (the UI pauses) and name the script line, e.g. `custom:3: attempt to perform arithmetic on a nil value`. A syntax error is reported when the scene is built. A script without `init` or `step` is refused.

## Snapshots

The state is saved in snapshots when it contains only numbers, strings, booleans and tables. A state holding functions or other values is not saved; restoring such a snapshot warns and restarts the script from `init`.

## Performance

The PI above costs about 2 µs per call in a release build (11 µs in a debug build), well inside the 50 µs of a 20 kHz period. A scene with it runs at ≈ 1.6× real time in the averaged Standard tier (release). Validation case V-CTRL-010 checks that this PI produces the same outputs as the built-in Rust PI to 1e-12.
