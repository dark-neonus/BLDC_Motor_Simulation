# refmodel — independent Python reference model

A from-spec implementation of the BLDC/PMSM drive in `docs/docs/physics/`, used to validate
the Rust engine. It was written from the physics pages only (no Rust code was read).

```python
from refmodel import RefParams, Action, simulate

P = RefParams()  # validation-catalog motor "M1", 24 V ideal source
P.ctrl.kind = "foc"
P.ctrl.foc.mode = "velocity"
df = simulate(
    P,
    [Action(0.01, "ctrl.omega_ref", 10.0)],
    t_end=0.1,
    record=["motor.omega", "motor.i_q", "energy.residual"],
    record_dt=1e-4,
)
```

`simulate(params, actions, t_end, record, record_dt) -> polars.DataFrame` returns column `t`
(s) plus the requested signals, named exactly as in `signals.md`, SI units.

## Model scope

| Block | Equations | Notes |
|---|---|---|
| Motor | EQ-MOT-01/02/03/04/05/07/08/09, EQ-CONV-* | ψαβ states; linear magnetics (L_d ≠ L_q allowed); shapes `sinusoidal`, `trapezoidal(w)`, `harmonics`. Saturation (EQ-MOT-10) not implemented. |
| Open phase | EQ-MOT-11/12, EQ-INV-01 | Off legs: diode-low / diode-high / floating sub-modes with state events (i = 0, terminal bounds). Floating uses the non-salient EQ-MOT-12 voltage. Two or more floating legs → zero-current mode; the line-EMF diode clamp of the all-off case is **not** implemented. |
| Mechanics | EQ-MECH-02/03/05/06/07/08/09 | Rigid drivetrain only (no backlash). Hybrid stick–slip on the lumped body (exact stick, events for ω = 0 and \|T_a\| = T_s). |
| Thermal | EQ-THERM-01..04 | R(T_w), λ(T_mag); T_mag = T_h (outrunner) / T_s (inrunner). |
| Supply / bus | EQ-SUP-01..05 | ideal, PSU CV/CC/BLOCKING (mode events), battery 1-RC Thevenin, chopper with hysteresis events. |
| Inverter | EQ-INV-03..09 | **Averaged** only: dead-time term with σ = tanh(i/i_ε), SPWM/SVPWM, overmodulation clamp, six-step H/O legs. |
| Control | EQ-CTRL-01/03/04/05/06/07 | Discrete, ZOH, ideal (bypass) sensors; FOC current / velocity (optional prefilter) / position cascade, MIT, six-step with ideal Halls (EQ-SENS-02), open-loop V/f. |
| Numerics / energy | EQ-NUM-02/05, EQ-ENER-01..03 | `solve_ivp` (DOP853 default, rtol 1e-9, atol 1e-12) between controller ticks / actions; state events terminal; energy terms are integrated states; battery bookkeeping (no input term, E_chem stored). |

Event order at one instant (EQ-NUM-02): controller tick → queued actions → mode settling.
Records that fall on an event time show post-event values. The integrator step is capped
at `record_dt` (dense-output accuracy).

Test fixtures (not in the spec): `mech.mode = "prescribed"` (kinematic constant speed; the
fixture's work is booked as external energy), `ctrl.kind = "ideal_voltage"` (continuous
ideal αβ voltage source `f(t, ctx)`, its power is the input), `thermal.p_inject`.

## Actions

`Action(t, path, value, mode=None, duration=None)`: sets a dotted parameter path at time `t`
(e.g. `"ctrl.omega_ref"`, `"bus.chopper_enabled"`). Special paths:
`load.disturbance.impulse` (H [N·m·s] → pulse H/Δt, Δt = max(1 ms, 5·sim.dt_max), EQ-MECH-08),
`load.disturbance.torque` (with `duration`), and `load.mass|distance|arm_mass|arm_length`
(EQ-MECH-09, `mode="keep_speed"` default or `"conserve_momentum"`, ΔE booked external).

## Parameters (`RefParams`, all SI)

| Field | Unit | Spec |
|---|---|---|
| `sim.method`, `sim.rtol`, `sim.atol` | –, –, – | EQ-NUM-07 |
| `sim.max_step` | s | integrator cap (also capped at `record_dt`) |
| `sim.dt_max` | s | EQ-MECH-08 pulse width |
| `sim.r_tol`, `sim.e_floor` | –, J | EQ-ENER-03 |
| `motor.pole_pairs` | – | p, EQ-CONV-05 |
| `motor.r_phase` | Ω | R at T_ref, EQ-MOT-01, EQ-THERM-02 |
| `motor.l_d`, `motor.l_q` | H | EQ-MOT-02 |
| `motor.lambda_m` | Wb | λ_m at T_ref, EQ-MOT-02, EQ-THERM-03 |
| `motor.emf_shape`, `motor.trap_width`, `motor.harmonics` | –, rad, ((n, b_n, φ_n)…) | EQ-MOT-03 |
| `motor.j_rotor` | kg·m² | J_m, EQ-MECH-02 |
| `motor.friction_static`, `_coulomb`, `_viscous` | N·m, N·m, N·m·s/rad | EQ-MECH-05 |
| `motor.cogging_nc`, `motor.cogging` | –, ((A_k [N·m], φ_k [rad])…) | EQ-MOT-08 |
| `motor.k_hy`, `motor.k_ed`, `motor.omega_eps_fe` | W·s/rad, W·s²/rad², rad/s | EQ-MOT-09 |
| `motor.alpha_br`, `motor.topology` | 1/K, – | EQ-THERM-03, thermal.md assumption 5 |
| `motor.theta0`, `motor.omega0`, `motor.i_alpha0`, `motor.i_beta0` | rad, rad/s, A, A | initial state |
| `mech.mode`, `mech.omega_prescribed` | –, rad/s | fixture |
| `gearbox.ratio`, `gearbox.efficiency` | –, – | N, η, EQ-MECH-02/03 |
| `gearbox.j_in`, `gearbox.j_out` | kg·m² | EQ-MECH-02 |
| `gearbox.omega_eps` | rad/s | EQ-MECH-03 |
| `load.arm_mass`, `load.arm_length`, `load.mass`, `load.distance` | kg, m, kg, m | EQ-MECH-06 |
| `load.g` | m/s² | EQ-MECH-06 |
| `load.kind`, `load.torque`, `load.viscous`, `load.omega_eps` | –, N·m, N·m·s/rad, rad/s | EQ-MECH-07 |
| `load.friction_static`, `_coulomb`, `_viscous` | N·m, N·m, N·m·s/rad | EQ-MECH-05 (lumped /N, /N²) |
| `load.theta0` | rad | initial load angle (θ_m = N θ_L) |
| `thermal.enabled` | – | off: R, λ at reference values, temperatures frozen |
| `thermal.c_w`, `c_s`, `c_h` | J/K | EQ-THERM-01 |
| `thermal.r_ws`, `r_sh`, `r_ha`, `kappa` | K/W, K/W, K/W, s/rad | EQ-THERM-01/04 |
| `thermal.t_amb`, `t_ref`, `t_init` | K | EQ-THERM-01..03 |
| `thermal.alpha_cu` | 1/K | EQ-THERM-02 (0.00393) |
| `thermal.p_inject` | W | fixture heat into the winding |
| `inverter.f_pwm`, `dead_time` | Hz, s | EQ-INV-03 |
| `inverter.r_on`, `v_f`, `t_rise`, `t_fall` | Ω, V, s, s | EQ-INV-03/09 |
| `inverter.i_eps` | A | σ(i), EQ-INV-03 |
| `inverter.modulation` | `svpwm` / `spwm` | EQ-INV-04/05 |
| `bus.capacitance`, `bus.v0` | F, V | EQ-SUP-01 |
| `bus.chopper_enabled`, `r_brake`, `v_on`, `v_off` | –, Ω, V, V | EQ-SUP-05 |
| `supply.kind` | `ideal` / `psu` / `battery` | EQ-SUP-02..04 |
| `supply.v_set`, `i_lim`, `r_o` | V, A, Ω | EQ-SUP-02/03 |
| `supply.series`, `parallel` | – | EQ-SUP-04 pack |
| `supply.ocv_soc`, `ocv_cell` | –, V (per cell) | EQ-SUP-04 OCV table (piecewise linear) |
| `supply.r0_cell`, `r1_cell`, `c1_cell`, `q_cell_ah` | Ω, Ω, F, A·h | EQ-SUP-04 |
| `supply.soc0`, `v_rc0` | –, V | initial state |
| `ctrl.kind` | `none` / `foc` / `mit` / `six_step` / `open_loop` / `ideal_voltage` | |
| `ctrl.f_ctrl` | Hz | controller rate (period rounded to ns, EQ-NUM-01) |
| `ctrl.omega_ref`, `ctrl.theta_ref` | rad/s, rad | setpoints (velocity loop: motor side; MIT/position: load side) |
| `ctrl.i_max`, `ctrl.omega_max` | A, rad/s | limits |
| `ctrl.foc.mode`, `id_ref`, `iq_ref` | –, A, A | EQ-CTRL-03 |
| `ctrl.foc.omega_c`, `anti_windup`, `m_max`, `delay_comp` | rad/s, –, –, – | EQ-CTRL-01/03 (default ω_c = 2π f_ctrl/20) |
| `ctrl.foc.omega_v`, `kp_v`, `ki_v`, `vel_div`, `prefilter` | rad/s, A·s/rad, A/rad, –, – | EQ-CTRL-04 (default ω_v = ω_c/10) |
| `ctrl.foc.omega_p`, `kp_p`, `ki_p`, `pos_div` | rad/s, 1/s, 1/s², – | EQ-CTRL-04 (default ω_p = ω_v/5) |
| `ctrl.six_step.duty`, `direction`, `advance`, `speed_pi`, `kp`, `ki` | –, ±1, rad, –, s/rad, 1/rad | EQ-CTRL-05 |
| `ctrl.open_loop.omega_e_ref`, `accel`, `v0`, `k_vf` | rad/s, rad/s², V, V·s/rad | EQ-CTRL-06 |
| `ctrl.mit.kp`, `kd`, `tau_ff` | N·m/rad, N·m·s/rad, N·m | EQ-CTRL-07 |
| `ctrl.ideal_voltage` | callable | fixture |

## Signals

Supported: all `motor.*` except `motor.v_n` in zero-current mode (NaN); `gearbox.delta` (0,
rigid), `gearbox.p_loss`; `load.*`; `thermal.*`; `inverter.*` (averaged values);
`bus.*`; `supply.*`; `sensors.hall.*` (ideal); the `ctrl.*` signals of the active
controller (held values, null otherwise); `energy.in|out|stored|external|residual|ok` and
`energy.loss.{copper,iron,friction,gearbox,load,inverter,chopper,battery}`.
Not supported: `gearbox.torque_contact`, encoder/ADC/estimator signals, `fault.*`,
`protect.*`, `sim.*` status signals.

## Self-tests

`uv run pytest refmodel -q` (≈ 40 s). Test names carry the validation-catalog IDs.
Ambiguities are listed in `SPEC_QUESTIONS.md`.
