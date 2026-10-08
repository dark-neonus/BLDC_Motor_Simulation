---
title: Validation catalog
---

# Validation catalog

Every case has an ID, which is used verbatim in the test name (`test_v_mot_001_…` in Python, `v_mot_001_…` in Rust).

**Side:**
- `rust` — Rust unit/integration test.
- `python` — CLI-level pytest that compares the output against a formula.
- `both` — both of the above.

**Tolerance.** All tolerances are $|a - b| \le atol + rtol\cdot|b|$ (CONVENTIONS §7).

**Default test motor ("M1").** Unless stated otherwise: $p = 14$, $R = 1.0\ \Omega$, $L_s = 2.5$ mH, $\lambda_m = 0.03$ Wb, $J = 2.5\cdot10^{-4}$ kg·m², $B = 10^{-4}$ N·m·s/rad, $V_{bus} = 24$ V.

**Conventions in this page.**
- Voltages: "LL pk" = line-to-line peak; "phase pk" = phase peak; "DC" = bus.
- Currents: $\hat I$ = peak phase current (= $i_q$ when $i_d = 0$).

## Motor (EQ-MOT, EQ-CONV)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-MOT-001 | both | Ideal | locked rotor, ideal source, $v_q$ = 1 V step | $i_q(t) = (V/R)(1 - e^{-tR/L_s})$ | atol 0.5 % of $V/R$, rtol 1e-6 |
| V-MOT-002 | both | Ideal | free spin, $v_d = 0$, $v_q = V$ (phase pk) | steady $\omega$ = root of the steady-state equations; $\to V/(p\lambda_m)$ as $B \to 0$ | rtol 5e-3 |
| V-MOT-003 | rust | Ideal | driven at constant $\omega_m$, open circuit | LL pk back-EMF $= K_e\omega_m = \sqrt3 p\lambda_m\omega_m$ | rtol 1e-6 |
| V-MOT-004 | rust | Ideal | locked rotor, $i_q = \hat I$ held | $T_e = K_t\hat I = 1.5p\lambda_m\hat I$ | rtol 1e-9 |
| V-MOT-005 | rust | Ideal | stationary-frame model vs dq reference, same sinusoidal voltages (EQ-MOT-06) | $i_d, i_q, \omega, T$ agree | atol 1e-9, rtol 1e-6 |
| V-MOT-006 | rust | Ideal | $K_v$ = 100 rpm/V, $p$ = 14 converted (EQ-CONV-13) | $\lambda_m$ = 3.9381 mWb, $K_t$ = 82.699 mN·m/A | rtol 1e-5 |
| V-MOT-007 | rust | Standard | unpowered rotor with cogging, no friction | cogging period $2\pi/N_c$; zero mean; energy residual < 1e-6 | atol 1e-9 |
| V-MOT-008 | rust | Standard | no-load spin-down with iron loss only | $\omega(t)$ matches integrating $J\dot\omega = T_{fe}(\omega)$ | rtol 1e-4 |
| V-MOT-009 | rust | Detailed | saturation with $c = 0$, $L_\infty = L_q$ | identical to the linear model | atol 1e-12 |
| V-MOT-010 | rust | Detailed | $i_q$ step to $2\,i_k$ | torque per amp $= 1.5p\lambda_m(1 - h(i_q))$; energy residual < 1e-3 | rtol 1e-3 |
| V-MOT-011 | rust | Standard | phase C opened while running | $i_c \equiv 0$, $i_a = -i_b$, no NaN, energy residual < 1e-3 | atol 1e-12 |
| V-MOT-012 | both | Standard | six-step on a **sinusoidal** machine, ideal 120° currents, constant speed | torque ripple $(T_{max} - T_{min})/T_{max} = 1 - \cos 30° = 13.4\,\%$ | atol 0.5 % |
| V-MOT-013 | rust | Standard | six-step on an ideal **trapezoidal** machine ($w$ = 120°), ideal currents | ripple excluding commutation windows ≈ 0 | atol 0.5 % |

## Mechanical (EQ-MECH)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-MECH-001 | both | Ideal | unpowered arm, 0.01 rad release, $N$ = 1 | period $T_0 = 2\pi\sqrt{J_{tot}/(gM_{eff})}$ | rtol 5e-4 |
| V-MECH-002 | rust | Ideal | large-angle swing, no friction | energy residual < 1e-6 over 10 periods | – |
| V-MECH-003 | rust | Ideal | torque step $T$ through a rigid gearbox $N$ | $\dot\omega_m = T/J_{eq}$ (EQ-MECH-02) | rtol 1e-9 |
| V-MECH-004 | rust | Standard | gearbox $\eta$ = 0.8, drive vs back-drive | loss $= (1-\eta)\lvert\tau_{in}\omega_m\rvert$ in both directions | rtol 1e-6 |
| V-MECH-005 | rust | Standard | backlash $b$, motor reverses slowly | load lags by $b$ (lost motion) | atol 1e-4 rad |
| V-MECH-006 | rust | Standard | torque $0.98\,T_s$ on a stuck body | $\theta$ unchanged (bit-identical) | exact |
| V-MECH-007 | rust | Standard | torque $1.2\,T_s$ then released | moves, decelerates per Coulomb + viscous, re-sticks; energy closes | rtol 1e-3 |
| V-MECH-008 | rust | Ideal | impulse $H$ on a free system | $\Delta\omega_L = H/J_{tot,L}$ | rtol 1e-6 |
| V-MECH-009 | rust | Ideal | live mass change, `keep_speed` vs `conserve_momentum` | $\omega$ kept / $\omega J$ kept; $\Delta E$ booked external | rtol 1e-9 |

## Thermal (EQ-THERM)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-THERM-001 | rust | Standard | constant heat into the winding, $\alpha_{Cu}$ = 0 | node temperatures = analytic solution of the linear 3-node ODE (eigen-decomposition) | rtol 1e-6 |
| V-THERM-002 | both | Standard | hold $\hat I_{cont}$ (EQ-THERM-05) to steady state | $T_w = T_{w,max}$ | atol 1 K |
| V-THERM-003 | rust | Standard | winding at 100 °C vs 20 °C | $R$ ratio 1.3144 | rtol 1e-9 |
| V-THERM-004 | both | Standard | `hold-heats-motor` scenario | $T_w$ in the asserted band; rises while $\omega$ = 0 | scenario asserts |

## Inverter (EQ-INV)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-INV-001 | rust | Detailed | switching vs averaged, FOC steady state | period-averaged $i_x$ agree | atol 1 % of rated current |
| V-INV-002 | rust | – | SVPWM duties vs sector-based reference, 36 angles | equal | atol 1e-12 |
| V-INV-003 | rust | – | max linear amplitude | SVPWM $V_{bus}/\sqrt3$, SPWM $V_{bus}/2$ | rtol 1e-12 |
| V-INV-004 | both | Detailed | dead time, $i > 0$ vs $i < 0$ | average terminal voltage error $\mp(t_d/T)(V_{bus} + V_f)$ | rtol 2 % |
| V-INV-005 | rust | Detailed | leg Off, current $\pm$ | terminal clamps to $-V_f$ / $V_{bus} + V_f$ | atol 1e-9 V |
| V-INV-006 | rust | Detailed | locked rotor, single leg pair at duty $D$ | ripple $\Delta i = V_{bus}D(1-D)T/(2L_s)$ for the a–b path | rtol 5 % |
| V-INV-007 | rust | Standard | power check in both modes | $V_{bus}(i_{dc} + i_{sw}) = \sum v_{xT}i_x + P_{inv}$ | rtol 1e-6 |
| V-INV-008 | rust | – | switching loss vs $f_{pwm}$ | linear in $f_{pwm}$ | rtol 1e-9 |

## Supply (EQ-SUP)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-SUP-001 | both | Standard | PSU, load above $I_{lim}$ | $i_{src} = I_{lim}$, $V_{bus}$ sags, mode = CC | atol 1e-6 A |
| V-SUP-002 | both | Standard | flywheel brake into PSU (blocking), no chopper | $V_{bus,final} \le \sqrt{V_1^2 + J\omega_0^2/C}$; energy closes | rtol 1e-3 |
| V-SUP-003 | rust | Standard | same with chopper | $V_{bus} \le V_{on}$ + one event's overshoot; dumped energy ≈ kinetic − losses | rtol 1e-3 |
| V-SUP-004 | rust | Standard | battery current step $I$ | instant sag $= I R_0$; RC relaxation time $R_1C_1$ | rtol 1e-6 |
| V-SUP-005 | rust | Standard | battery constant discharge | $\Delta SoC = \int I\,dt/(3600Q)$ | rtol 1e-9 |

## Sensors (EQ-SENS)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-SENS-001 | rust | – | encoder $b$ bits, slow ramp | step size $2\pi/2^b$ | exact |
| V-SENS-002 | rust | – | latency $\tau$ | reading $= x(t - \tau)$ | atol 1e-12 |
| V-SENS-003 | rust | – | noise $\sigma$, 1e4 seeded samples | sample std within ±5 % of $\sigma$; same seed gives the same sequence | statistical |
| V-SENS-004 | rust | – | Halls over one electrical turn | transitions at 30° + 60°k, code table of EQ-SENS-02 | exact |
| V-SENS-005 | rust | Detailed | ADC sampled at `pwm.center` | reading ≈ period-average current | atol 1 % of rated |
| V-SENS-006 | both | Standard | ADC offset on phase a | $i_d, i_q$ ripple at $\omega_e$ with amplitude $\propto$ offset | rtol 5 % |
| V-SENS-007 | rust | Standard | quadrature 1000 CPR | 4000 counts/rev; index once per rev | exact |

## Control and estimation (EQ-CTRL)

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-CTRL-001 | both | Standard | FOC current-loop step | closed-loop bandwidth ≈ $\omega_c$ | rtol 15 % |
| V-CTRL-002 | both | Standard | velocity step, PI design EQ-CTRL-04 | overshoot 13.5 % (no prefilter), ≈ 0 with prefilter | atol 2 % |
| V-CTRL-003 | both | Standard | MIT hold under load $\tau_L$ | static deflection $\tau_L/K_p$ | rtol 2 % |
| V-CTRL-004 | rust | Standard | PLL, $\zeta$ = 1 | −3 dB bandwidth $2.48\,\omega_n$ | rtol 5 % |
| V-CTRL-005 | rust | Standard | flux observer, $\omega_e \ge$ 10 % rated, 3 % $R$ error | angle error < 5° after settling | – |
| V-CTRL-006 | rust | Standard | flux observer near standstill | `est.valid` = false (documented failure) | – |
| V-CTRL-007 | rust | Standard | encoder calibration with a random true offset | recovered offset error < 1° electrical | – |
| V-CTRL-008 | both | Standard | auto-tune on each motor preset | $R, L, J$ within 5 %; achieved $\omega_c$ within ±20 % | rtol |
| V-CTRL-009 | rust | Standard | open-loop V/f with a load step above pull-out | loss of synchronism detected | – |
| V-CTRL-010 | rust | Standard | Luau PI vs Rust PI on the same inputs | identical outputs | atol 1e-12 |

## Energy, numerics, determinism

| ID | Side | Tier | Scenario | Expected | Tolerance |
|---|---|---|---|---|---|
| V-ENER-001 | both | all | every scenario in `presets/scenarios/` | $\lvert r\rvert < r_{tol}$ (EQ-ENER-03) throughout | per tier |
| V-NUM-001 | rust | – | RK4 on decay/oscillator, halving $h$ | error ratio ≈ 16 | ±10 % |
| V-NUM-002 | rust | – | RK4 vs diffsol on the skeleton PMSM | agree | rtol 1e-6 |
| V-NUM-003 | rust | – | bouncing ball event localisation | impact times within 1 ns | atol 1 ns |
| V-DET-001 | both | all | every scenario run twice | byte-identical Parquet output | exact |

Total: 62 cases.

## Skeleton cases (P01, kept until replaced)

V-SKEL-001 (same as V-MOT-001) and V-SKEL-002 (same as V-MOT-002) were implemented in P01.T09. P11 renames them to their V-MOT IDs when the full scenario format lands.
