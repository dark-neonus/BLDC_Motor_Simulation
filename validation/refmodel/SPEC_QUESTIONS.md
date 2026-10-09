# Spec questions raised by the Python reference model

Each question has the reading the refmodel implements (the most literal one).

**Status (2026-10-09): all answered in the spec.** Readings were adopted as written except: Q-05 (only switching legs), Q-08 (`energy.out` dropped), Q-14 (all setpoints load-side), Q-17 (commands *before* the tick), Q-18 (`gearbox.friction.*` dropped; gearbox loss via η only). The refmodel was updated for Q-05/Q-14/Q-17. The code
marks the place with `# SPEC-AMBIGUITY: Q-xx`.

| ID | Spec | Question | Reading implemented |
|---|---|---|---|
| Q-01 | signals.md | The time signal is `sim.t`; the refmodel brief asks for a column `t`. Should Parquet outputs use `t` or `sim.t`? | Column `t` (seconds). |
| Q-02 | EQ-MECH-03 + mechanical.md assumption 4 | In the rigid case friction is lumped on ω_m, but τ_in uses T_L "incl. load friction". Is the load-bearing share of the lumped friction part of T_L in τ_in? And while sticking? | Slipping: the lumped element is split back into motor part (T_s,m, T_c,m, b_m) and load part (T_c,L, b_L); the load part is in T_L. Sticking: T_gl = 0 (tanh(0)). |
| Q-03 | EQ-INV-07 (averaged six-step) | The H/O leg average is given separately for i > 0 and i < 0 with no rule at i = 0 (sliding/DCM). How is the sign switched? | Blended with σ(i) = tanh(i/i_ε) like EQ-INV-03: weight w = (1+σ)/2 on the i>0 formula; i_dc contribution w·d·i + (1−w)·i. |
| Q-04 | EQ-MOT-09 / EQ-THERM-01 | P_fe = k_hy\|ω_e\| + k_ed ω_e² differs from −T_fe ω_m (tanh vs abs). Which one heats the stator and is booked as loss? | −T_fe ω_m for both (so the energy balance and the heat agree). |
| Q-05 | EQ-INV-09 | i_sw sums ½\|i_x\|(t_r+t_f) over all legs, but in six-step only the High leg switches. | Literal sum over all three legs in every mode. |
| Q-06 | EQ-ENER-03 | Which terms enter E_thr = ∫Σ\|P_k\|? With a battery, is \|V_bus i_src\| (internal flow) included? | Only the EQ-ENER-01 table terms (input, external, every loss); battery internal flow not included. |
| Q-07 | signals.md `inverter.leg_x` | H/L/O is undefined for an averaged PWM leg (FOC). | Null for averaged PWM legs; six-step reports the commanded state (H for the PWM'd H/O leg, L, O). |
| Q-08 | signals.md `energy.out` | `energy.out` is listed but EQ-ENER never defines it. | Sum of all loss terms (energy leaving the system as heat). |
| Q-09 | signals.md `energy.loss.<module>` | Module names are not listed. | `copper, iron, friction, gearbox, load, inverter, chopper, battery`. |
| Q-10 | EQ-THERM-01 | Does P_fric (to the housing node) include the viscous part b·ω² of the bearing / load friction? | Yes: P_fric = −(T_f,m ω_m + T_f,L ω_L) with the full EQ-MECH-05 slipping law. |
| Q-11 | EQ-MECH-07 | ω_ε for the `brake` load is not given. | `load.omega_eps` = 1e-3 rad/s (same as EQ-MECH-03). |
| Q-12 | EQ-INV-03 | i_ε "≈ 1 % of rated current" — there is no rated-current parameter. | Explicit `inverter.i_eps` [A], default 0.01 A. |
| Q-13 | EQ-CTRL-01/03 | Is the PI saturation / anti-windup applied to u* = Kp e + I or to u* + feed-forward? | To the total (PI + decoupling feed-forward), after the d-priority limiter. |
| Q-14 | EQ-CTRL-07 / signals.md | `ctrl.omega_ref` is the motor-side setpoint for the velocity loop but ω* in MIT is load-side. Same signal for both? Which J sets the MIT default gains (EQ-CTRL-12)? | Same signal; in MIT mode it is load-side ω*. MIT gains are explicit parameters (no default from J). |
| Q-15 | EQ-CTRL-05 / EQ-NUM-02 | Does six-step commutate on Hall edges (asynchronous events) or only at controller ticks? | Only at controller ticks (Halls read at the tick, ZOH). |
| Q-16 | signals.md `ctrl.six_step.sector` | Numbering not defined. | 0..5 in EQ-INV-07 table order (0 = 330°…30°); −1 for an invalid Hall code. |
| Q-17 | EQ-NUM-02 | Queued commands at the same instant as a controller tick: before or after the tick? | After (literal loop order): a setpoint change at a tick time is first used at the next tick. |
| Q-18 | mechanical.md assumption 4 | `gearbox.friction.*` exists as a parameter root, but the rigid lumping formula only combines motor and load friction. | Gearbox friction is not modelled in the rigid case. |
| Q-19 | V-MECH-002 / EQ-ENER-03 | For an unpowered frictionless swing all power terms are 0, so E_thr = 0 and the residual is (drift in J)/E_floor. Is "< 1e-6" meant with that floor? | Yes, literal (drift < 1 pJ); the refmodel reaches ~4e-8. |
| Q-20 | EQ-MECH-03 + EQ-CTRL-07 | With η < 1 the tanh mesh loss acts as a viscous drag (1−η)\|τ_in\|/ω_ε below ω_ε, so an MIT hold with η < 1 creeps to τ_L/K_p with a time constant of seconds. Intended? | Implemented literally; V-CTRL-003 self-test uses η = 1. |
