---
title: Signal and parameter catalog
---

# Signal and parameter catalog

These are the canonical dotted paths (CONVENTIONS §3). They are used identically in the engine bus, the REST/WS/MCP APIs, the Parquet columns, the help registry and the UI `data-agent-id`s. All values are SI.

## Signals

| Path | Unit | Meaning | Producer | Spec |
|---|---|---|---|---|
| `sim.t` | s | simulation time | engine | EQ-NUM-01 |
| `sim.running`, `sim.time_scale`, `sim.real_ratio`, `sim.dt_max`, `sim.tier` | –, –, –, s, – | clock / fidelity status | runner | EQ-NUM |
| `motor.theta`, `motor.omega` | rad, rad/s | mechanical angle, speed | rotor | EQ-MECH-02 |
| `motor.theta_e`, `motor.omega_e` | rad, rad/s | electrical angle, speed | rotor | EQ-CONV-05 |
| `motor.psi_alpha`, `motor.psi_beta` | Wb | flux-linkage states | motor | EQ-MOT-01 |
| `motor.i_a`, `motor.i_b`, `motor.i_c` | A | phase currents | motor | EQ-MOT-02 |
| `motor.i_alpha`, `motor.i_beta`, `motor.i_d`, `motor.i_q` | A | transformed currents | motor | EQ-CONV-01/03 |
| `motor.e_a`, `motor.e_b`, `motor.e_c` | V | back-EMF | motor | EQ-MOT-03 |
| `motor.v_n` | V | neutral voltage | motor | EQ-MOT-04 |
| `motor.torque_em`, `motor.torque_cog`, `motor.torque_fe` | N·m | electromagnetic / cogging / iron-drag torque | motor | EQ-MOT-07/08/09 |
| `motor.p_cu`, `motor.p_fe` | W | copper / iron loss | motor | EQ-MOT-09/13 |
| `motor.open_phase` | – | open-phase mode (none/a/b/c) | motor | EQ-MOT-11 |
| `gearbox.delta`, `gearbox.torque_contact`, `gearbox.p_loss` | rad, N·m, W | backlash deflection, contact torque, loss | gearbox | EQ-MECH-03/04 |
| `load.theta`, `load.omega` | rad, rad/s | load-side angle, speed | mech | EQ-MECH-02/04 |
| `load.torque_gravity`, `load.torque_ext`, `load.disturbance` | N·m | load torques | load | EQ-MECH-06…08 |
| `thermal.t_winding`, `thermal.t_stator`, `thermal.t_housing`, `thermal.t_magnet` | K | temperatures | thermal | EQ-THERM-01 |
| `inverter.leg_a`, `inverter.leg_b`, `inverter.leg_c` | – | leg state H/L/O | inverter | EQ-INV-01 |
| `inverter.duty_a`, `inverter.duty_b`, `inverter.duty_c` | – | duties | modulator | EQ-INV-02 |
| `inverter.v_a`, `inverter.v_b`, `inverter.v_c` | V | terminal voltages | inverter | EQ-INV-01/03 |
| `inverter.i_dc`, `inverter.i_sw`, `inverter.p_loss` | A, A, W | DC link, switching current, loss | inverter | EQ-INV-08/09 |
| `bus.v`, `bus.i_chopper`, `bus.chopper_on`, `bus.p_brake` | V, A, –, W | DC bus and chopper | bus | EQ-SUP-01/05 |
| `supply.i`, `supply.mode`, `supply.soc`, `supply.v_terminal`, `supply.battery.v_rc` | A, –, –, V, V | source state | supply | EQ-SUP-02…04 |
| `sensors.hall.a`, `.b`, `.c`, `sensors.hall.code` | – | Hall outputs and code | halls | EQ-SENS-02 |
| `sensors.encoder.angle`, `sensors.encoder.error` | rad | measured angle, true − measured | encoder | EQ-SENS-03/05 |
| `sensors.adc.i_a`, `.i_b`, `.i_c`, `sensors.adc.v_bus` | A, V | measured currents / bus | ADC | EQ-SENS-06 |
| `est.theta`, `est.omega`, `est.valid` | rad, rad/s, – | estimator outputs | estimators | EQ-CTRL-08/09 |
| `ctrl.omega_ref`, `ctrl.theta_ref`, `ctrl.torque_ref` | rad/s, rad, N·m | active setpoints | setpoint | EQ-CTRL-04/07 |
| `ctrl.foc.id_ref`, `ctrl.foc.iq_ref`, `ctrl.foc.v_d`, `ctrl.foc.v_q`, `ctrl.foc.v_alpha`, `ctrl.foc.v_beta`, `ctrl.foc.saturated` | A, V, – | FOC internals (αβ = commands after inverse Park, held until the next tick) | FOC | EQ-CTRL-03 |
| `ctrl.six_step.sector`, `ctrl.open_loop.theta_ref` | –, rad | controller internals | controllers | EQ-CTRL-05/06 |
| `fault.<id>.active`, `protect.<id>.tripped` | – | fault / protection flags | faults | P10 |
| `energy.in`, `energy.loss`, `energy.loss.<term>`, `energy.stored`, `energy.external`, `energy.residual`, `energy.ok` | J, J, J, J, J, –, – | energy accounting | engine | EQ-ENER |

**Clarifications (spec Q&A, P11.T01):**
- Parquet/CSV outputs use the column `t` [s] for time (the bus signal `sim.t` is the same value).
- **All setpoints are load-side** (`ctrl.theta_ref`, `ctrl.omega_ref`, `ctrl.torque_ref`); controllers convert with the gear ratio $N$. Without a gearbox they equal motor-side values.
- `energy.loss.<term>` uses these term names: `copper`, `iron`, `friction`, `gearbox`, `load`, `inverter`, `chopper`, `battery`. There is no `energy.out` signal.
- `inverter.leg_x` reports H/L/O for switched legs and six-step; it is undefined (null) for averaged PWM legs.
- `ctrl.six_step.sector` is 0…5 in the order of the EQ-INV-07 table (0 = 330°…30°), and −1 for an invalid Hall code.

## Parameter roots

Parameter paths are `<component>.<group>.<name>` and live under these roots:

`sim.*` (fidelity, seed), `motor.{identity,electrical,magnetic,winding,geometry,mechanical,thermal,ratings}.*`, `gearbox.*` (ratio, efficiency, backlash, stiffness, inertias; no separate gearbox friction: its loss is η), `load.*`, `inverter.*`, `supply.{psu,battery}.*`, `bus.*`, `sensors.{hall,encoder,adc,observer}.*`, `ctrl.{foc,six_step,open_loop,mit,custom,limits}.*`, `fault.*`, `protect.*`.

The authoritative list of parameter fields is the generated JSON Schema (`schemas/*.schema.json`, P04.T07). Field names must match the paths used in the physics pages, e.g. `motor.electrical.lambda_m` and `motor.electrical.r_phase`.
