---
title: Control algorithms and estimators
---

# Control algorithms and estimators

:::tip In plain words
A controller compares what you want (the setpoint) with what a sensor says (the feedback) and decides what voltage to apply. FOC stacks three such loops: current inside speed inside position. Other controllers covered here: six-step, open-loop, impedance ("MIT") control, the sensorless observer, and the auto-tuner.
:::

All controllers are discrete blocks running at their own rate $f = 1/T$ (EQ-NUM), and they see only sensor readings (EQ-SENS).

## PI / PID

### EQ-CTRL-01 — Discrete PI with anti-windup {/* #eq-ctrl-01 */}

$$
u_k^{*} = K_p e_k + I_k,\qquad u_k = \mathrm{sat}(u_k^{*}),\qquad
I_{k+1} = I_k + T\big(K_i e_k + K_b\,(u_k - u_k^{*})\big)
$$

There are two anti-windup modes:

- `clamping` (default): $K_b = 0$, and $I$ is not updated while $u_k \ne u_k^*$ and $e_k$ would push further into the limit.
- `back_calculation`: $K_b = K_i/K_p$.

### EQ-CTRL-02 — Filtered derivative (PID) {/* #eq-ctrl-02 */}

The derivative acts on the measurement $y$ (not the error) to avoid kicks on setpoint steps:

$$
D_k = \frac{T_f}{T_f + T}D_{k-1} - \frac{K_d}{T_f + T}\,(y_k - y_{k-1})
$$

## Field-oriented control (FOC)

### EQ-CTRL-03 — Current loop {/* #eq-ctrl-03 */}

1. Measured $i_a, i_b$ → Clarke → Park with $\hat\theta_e = p\,\theta_{meas} + \hat\theta_{e,0}$ (encoder or estimator).
2. PI on $d$ and $q$ (EQ-CTRL-01) plus decoupling feed-forward: $v_d^{ff} = -\hat\omega_e L_q i_q$, $v_q^{ff} = \hat\omega_e(L_d i_d + \lambda_m)$.
3. Voltage limit with d priority: $V_{max} = m_{max}V_{bus}/\sqrt3$ ($m_{max} = 0.95$), $v_d \leftarrow \mathrm{clamp}(v_d, \pm V_{max})$, $|v_q| \le \sqrt{V_{max}^2 - v_d^2}$. The limiter reports saturation to the PIs.
4. Inverse Park using the angle advanced by $1.5\,T\,\hat\omega_e$, which compensates the computation plus PWM delay. Then SVPWM (EQ-INV-05).
5. **Timing.** The loop runs at `pwm.center` (period $T = T_{pwm}$, phase 0) or at its own rate. The command computed from the samples at $t_k$ is applied from $t_{k+1}$ (one period of computation delay, like an MCU writing shadow registers) and held for a period, so it acts on average $1.5\,T$ after the sample: the delay the angle advance compensates.

**Gain design** by pole-zero cancellation:

$$
K_p = \omega_c L,\qquad K_i = \omega_c R \;\Rightarrow\; \frac{i}{i^*} \approx \frac{\omega_c}{s + \omega_c}
$$

The default is $\omega_c = 2\pi f_{ctrl}/20$. The d axis uses $L_d$ and the q axis $L_q$. With the 1.5-period delay the loop keeps about 63° phase margin. **Measured quantity:** the step-response time constant $\tau_{63} \approx 1/\omega_c$ (the −3 dB bandwidth comes out at about $2.2\,\omega_c$ because of the delay, so it is not used as the metric).

**Clarifications.** PI saturation and anti-windup apply to the **total** command (PI output plus decoupling feed-forward), after the d-priority limiter. Six-step reads the Halls and commutates at controller ticks (Hall edges are not asynchronous events). The MIT gains are explicit parameters; EQ-CTRL-12 only *suggests* them from $J_{tot,L}$.

### EQ-CTRL-04 — Velocity and position loops {/* #eq-ctrl-04 */}

**Velocity PI → $i_q^*$** (clamped to the current limit). With $J$ the total reflected inertia:

$$
K_{p,v} = \frac{\omega_v J}{K_t},\qquad K_{i,v} = \frac{K_{p,v}\,\omega_v}{4},\qquad \omega_v \le \omega_c/5
$$

- The closed loop has a double pole at $\omega_v/2$.
- The PI zero gives **13.5 % step overshoot** with an ideal current loop; the optional setpoint prefilter $\dfrac{1}{1 + 4s/\omega_v}$ removes it (verified numerically). The current loop's lag (≈ $1/\omega_c$ plus the $1.5\,T$ delay) adds overshoot: about 14.3 % at $\omega_v = \omega_c/20$ and 15–16 % at $\omega_v = \omega_c/10$ (continuous model with a first-order current loop and the delay: 15.2 %; the sampled velocity loop's hold adds ≈ 0.6 points).

**Position P (or PI) → $\omega^*$** on the load-side angle: $K_{p,p} = \omega_p$, $\omega_p \le \omega_v/5$.

- The output is converted to motor speed (×$N$) and clamped to the speed limit.
- Loop semantics: a disabled outer loop passes its setpoint straight through, and each loop has its own rate and ramp limiter.
- A P-only position loop under constant load torque holds a steady error of $\tau_L/(N^2\,K_t\,K_{p,v}\,K_{p,p})$ when the velocity loop is P-only, and zero error with a PI velocity loop.

**Defaults and units.** Without a `bandwidth`, $\omega_v = \omega_c/10$ and $\omega_p = \omega_v/10$; explicit `kp`/`ki` override the design (velocity in A per rad/s and A per rad, position in load rad/s per rad and per rad·s). The position loop is P unless a `ki` is given. `controller.limits.speed` (load side) clamps $\omega^*$ to $\pm N\,\omega_{lim}$ on the motor side; `controller.limits.current` clamps $i_q^*$; `controller.limits.ramp` (load side, rad/s²) limits how fast the velocity reference may change. A disabled velocity loop passes the torque setpoint as $i_q^* = \tau^*/(N K_t)$. Feedback: the position loop reads the load angle (the encoder angle / $N$ when an encoder is fitted), the velocity loop the motor speed (`est.omega` when an estimator runs, EQ-CTRL-08/09; the true speed without a speed sensor).

**Modes from scenarios.** A velocity target disables the position loop; a torque target disables the velocity loop; a position target enables both. $\theta^*$ starts at the initial load angle, so a scene with a position loop holds its initial pose.

## Other controllers

### EQ-CTRL-05 — Six-step with Hall sensors {/* #eq-ctrl-05 */}

1. The Hall code selects the leg pattern (EQ-SENS-02 → EQ-INV-07); without Halls the controller uses the true angle (ideal feedback).
2. Command $u \in [-1, 1]$: either set directly (`ctrl.duty_ref`) or from a speed PI whose output is limited to the setpoint's direction ($[0, 1]$ for $\omega^* > 0$, $[-1, 0]$ for $\omega^* < 0$, 0 at $\omega^* = 0$): slowing within a direction coasts, and only a commanded reversal plugs (within the current cap). The duty is $|u|$. (A signed PI plugged at every overshoot and turned the speed into a sawtooth limit cycle: found in the P09.T12 preset sweep.)
3. The sign of $u$ is the direction: negative swaps the High and Low legs.
4. An optional advance angle shifts commutation by up to 30°. With Halls the angle inside a sector is interpolated: $\hat\theta_e$ = start of the sector + $\hat\omega_e\,(t - t_{edge})$, clamped to the sector, with $\hat\omega_e = 60°/\Delta t$ of the last sector.

**Speed-PI design** on the DC-equivalent model of the two conducting phases: $J\dot\omega = k_e i - \tau_L$, $u V_{bus} = 2R\,i + k_e\,\omega$, with $k_e = p\,\lambda_m\,\overline{(k_{max} - k_{min})}$ (the sector-average line-to-line EMF shape, EQ-MOT-03; $3\sqrt3/\pi$ for a sinusoidal machine). Then $\omega/u = (V_{bus}/k_e)/(1 + s\tau_m)$, $\tau_m = 2RJ/k_e^2$, and pole-zero cancellation gives $K_p = \omega_s \tau_m k_e/V_{bus}$, $K_i = \omega_s k_e/V_{bus}$ (closed loop $\approx \omega_s/(s + \omega_s)$; default $\omega_s$ = rate/50·2π).

**Current limit.** There is no current loop; the duty is capped from the DC model including the inverter so that the current cannot exceed `controller.limits.current` $I_{lim}$. With H/O PWM the High leg freewheels through the low diode during O (EQ-INV-07), so the line voltage is $d(V_{bus} + V_f) - V_f - R_{on}(1 + d)\,i$, and $|u| \le \big(k_e|\hat\omega| + V_f + 2(R + R_{on}) I_{lim}\big)/(V_{bus} + V_f)$ (measured bus) while motoring ($u$ and $\hat\omega$ of the same sign). Braking with the reversed pattern (plugging), the back-EMF adds to the applied voltage, so the cap is $\max\big(0,\ V_f + 2(R + R_{on}) I_{lim} - k_e|\hat\omega|\big)/(V_{bus} + V_f)$: above that speed the controller coasts and the current returns through the diodes (found: a speed overshoot on a 5010 plugged it at 68 A with a 20 A limit). In the cap $k_e$ is the sector's **minimum** line EMF, $p\lambda_m\min(k_{max} - k_{min})$ (1.5 instead of the mean $3\sqrt3/\pi$ for a sinusoidal machine): on a low-R motor most of the voltage is back-EMF, and the ±13 % EMF variation across a sector of a sinusoidal machine let the current exceed a mean-based cap by 50 % near the sector edges. At low duty the diode drop dominates: a cap without it would leave a stalled motor with a fraction of $I_{lim}$. Validation: on a trapezoidal machine (120° flat top) the line-to-line EMF is flat across each sector, so the torque between commutations is flat (within 5 % of the mean outside ±20° of each commutation) **when** a sector lasts many electrical time constants ($\pi/(3\omega_e) \ge 10\,L/R$, so the current settles) and the inertia keeps the speed ripple negligible. Otherwise the torque ramps within each sector: near the no-load speed most of the voltage is back-EMF, so a small speed ripple is a large current ripple. The floating phase's current decays through a diode to zero after each commutation (EQ-INV-07, EQ-MOT-12).

**At $u = 0$ (coasting)** the High leg is commanded Off and the Low leg stays on, so two phases float (the zero-current mode, EQ-MOT-12). The drive does not brake. A current flows only when a line EMF exceeds the diode drop: then the Low switch and a low diode short that pair, a real effect at speed. Being unidirectional, the drive slows down only through its load; a frictionless flywheel keeps its speed.

### EQ-CTRL-06 — Open-loop V/f {/* #eq-ctrl-06 */}

$$
\theta^*_e = \int \omega^*_e\,dt\ \ (\text{ramped at } a),\qquad
|v| = V_0 + k_{vf}\,|\omega^*_e|,\qquad
v_{\alpha\beta} = |v|\,[-\sin\theta^*_e,\ \cos\theta^*_e]
$$

**Limitation.** Without feedback there is no damping beyond the winding resistance: on a low-R motor V/f hunts and slips poles at mid speeds even unloaded (verified: the 5010, R = 25 mΩ, loses synchronism with both open-loop presets; the 2804 and 8108 stay in sync). Real V/f drives add a current-based stabilizer; this controller is the plain law. Defaults relative to the motor (presets): $k_{vf} = 1.1\,\lambda_m$ (10 % above the back-EMF line) and $V_0 = 2R \cdot 0.3\,I_{lim}$ (a boost current through the line resistance). There is no feedback. $\omega^*_e = N p\,$`ctrl.omega_ref` (load side → electrical), ramped at `controller.accel` (load side, rad/s²) and clamped to the speed limit; $k_{vf}$ is in V per electrical rad/s. The vector is applied at the centre of its hold interval ($\theta^* + \tfrac12\omega^* T$). **Loss of synchronism** is flagged (`ctrl.open_loop.sync_lost`, latched, and a warning) when $|\mathrm{wrap}(\theta^*_e - \theta_e)| > \pi/2$ for more than one electrical period, **or** when the rotor slips a pole: the unwrapped error (the sum of the wrapped per-tick changes of $\theta^*_e - \theta_e$) has moved by $2\pi$ from its first value. The second rule is needed because a fast-slipping rotor passes through $|\mathrm{wrap}| < \pi/2$ every turn, so the first never fires. The true angle serves this diagnostic only.

### EQ-CTRL-07 — Impedance / MIT mode {/* #eq-ctrl-07 */}

$$
\tau^* = K_p(\theta^* - \theta_L) + K_d(\omega^* - \omega_L) + \tau_{ff},\qquad
i_q^* = \mathrm{clamp}\!\Big(\frac{\tau^*}{N\,K_t}\Big)
$$

- Validation (V-CTRL-mit): static deflection under load is $\tau_L/K_p$.
- Damping ratio: $\zeta = K_d / (2\sqrt{K_p J_{L,tot}})$.
- The law runs on top of the current loop at its own rate.
- Gains relative to the motor (presets): without `kp`, $K_p = J_{L,tot}\,\omega_{mit}^2$ from `bandwidth`; without `kd`, $K_d = 2\zeta\sqrt{K_p J_{L,tot}}$ with `damping` ζ (default 0.7), as EQ-CTRL-12 suggests.
- Cogging is a load too: a hold stops anywhere $K_p|e| \le \tau_{cog}$ (EQ-MOT-08), so the static error can reach $\tau_{cog}/K_p$. On a bare rotor $K_p = J\omega_{mit}^2$ is tiny: a 5 Hz hold on a 2804 gives $\tau_{cog}/K_p \approx 0.5$ rad. The presets therefore start at 10 Hz (soft) and go to 25 and 50 Hz.
- No efficiency factor is applied: gear loss is speed-dependent and zero at standstill (EQ-MECH-03), so holding torque transmits without loss.

## Estimators

### EQ-CTRL-08 — Velocity estimation {/* #eq-ctrl-08 */}

**(a) Difference + low-pass:**

$$
\hat\omega_k = (1-\alpha)\hat\omega_{k-1} + \alpha\,\frac{\mathrm{wrap}(\theta_k - \theta_{k-1})}{T},\qquad \alpha = \frac{T}{T + \tau_f}
$$

**(b) PLL:**

$$
e = \mathrm{wrap}(\theta_{meas} - \hat\theta),\qquad
\hat\omega \mathrel{+}= T K_i e,\qquad
\hat\theta \mathrel{+}= T(\hat\omega + K_p e)
$$

with $K_p = 2\zeta\omega_n$ and $K_i = \omega_n^2$. For $\zeta = 1$ the −3 dB bandwidth is $2.48\,\omega_n$ (verified). With this update order, $\hat\theta$ after step $k$ is the prediction for the next sample, $\theta(t_k + T)$; on a constant-speed ramp the loop (type 2) settles with no lag beyond that.

**In a scene** (`sensors.velocity`): with an encoder fitted and no flux observer, a velocity estimator runs at the encoder's rate on `sensors.encoder.angle` (mechanical) — a PLL by default with $\omega_n = 2\pi f_s/50$, $\zeta = 1$, or difference + LPF with $\tau_f$ = 10 samples. Its T is the sample period. Its `est.omega` is the velocity loop's feedback and $p\,$`est.omega` the current loop's $\hat\omega_e$ (MIT: `est.omega`/N). Six-step on Halls uses its Hall-interval speed $\hat\omega_e/p$ (`ctrl.six_step.omega`). Without a speed sensor the true speeds are used (ideal feedback).

### EQ-CTRL-09 — Nonlinear flux observer `[Lee2010]` {/* #eq-ctrl-09 */}

$$
\eta = x - L_s\,i_{\alpha\beta},\qquad
\dot x = v_{\alpha\beta} - R\,i_{\alpha\beta} + \frac{\gamma}{2}\,\eta\,\big(\lambda_m^2 - |\eta|^2\big),\qquad
\hat\theta_e = \mathrm{atan2}(\eta_\beta, \eta_\alpha)
$$

- Speed comes from a PLL on $\hat\theta_e$. The radial convergence rate is $\gamma\lambda_m^2$; default $\gamma = 1000/\lambda_m^2$.
- **In a scene** (`sensors.observer`): built when the controller is sensorless or the section is given (then it replaces the encoder's velocity estimator: one estimator owns `est.*`). It runs at the current-loop rate (its T is that period) on the phase currents (ADC when fitted) and the applied command `ctrl.foc.v_alpha/v_beta` read before the current loop updates it, i.e. the voltage over the last period; $L_s = (L_d + L_q)/2$. Defaults: $\gamma = 1000/\lambda_m^2$, $\omega_{min} = 0.05\,V_{bus}/(\sqrt3\,\lambda_m)$, PLL $\omega_n = 2\pi f_c/100$, $\zeta = 1$. Forward Euler on the radial dynamics needs $\gamma\lambda_m^2 T < 2$; a larger γ is refused at build.
- `est.valid` is set when $|\hat\omega_e| > \omega_{min}$ and $\big||\eta| - \lambda_m\big| < 0.2\lambda_m$.
- **Convergence:** started from $\eta = 0$ ($x = L_s i$), with the default $\gamma$ and exact parameters, the angle error falls below 5° within 100 ms at $\omega_e \ge 10\,\%$ of rated speed and stays there (P08.T07 test: about 55 ms at 200 rad/s for $\lambda_m$ = 10 mWb). The integration is forward Euler at the control rate, with the voltage applied over the last period.
- **Expected low-speed failure:** the back-EMF shrinks with speed while errors in $R$, offsets and dead time do not. Verified (P08 test) with a 3 % resistance error, $\lambda_m$ = 10 mWb, $L_s$ = 1 mH, $R$ = 0.5 Ω, a pure $i_q$ = 2 A, $T$ = 50 µs, default $\gamma$, PLL $\omega_n = 2\pi\cdot100$ rad/s, $\zeta = 1$; run for 40 electrical radians (at least 0.3 s, at most 4 s), mean over the second half:

  | $\omega_e$ [rad/s] | 700 | 140 | 14 | 1.4 |
  |---|---|---|---|---|
  | mean angle error | 0.2° | 23° | 52° | 147° |
  | `est.valid` (fraction of samples) | 1 | 0.97 | 0 | 0 |

  `est.valid` does **not** detect the angle error caused by a parameter mismatch: the observer drives $|\eta|$ to $\lambda_m$ by construction, so the magnitude check catches only gross divergence (verified at 700 rad/s: a true flux 30 % or more above the assumed $\lambda_m$ clears the flag on every sample; a true flux of half or 0.7× the assumed one makes it flicker on about half), and in practice the flag is a speed gate. Controllers must not rely on it alone below a few times $\omega_{min}$.

### EQ-CTRL-10 — Sensorless startup {/* #eq-ctrl-10 */}

1. **Align:** $i_d = I_{align}$ at $\theta = 0$ for $t_{align}$.
2. **Open-loop current ramp (I/f):** $i_q^*$ fixed, $\theta^*$ ramped.
3. **Hand-off** when $|\omega^*| > \omega_{handoff}$ and `est.valid` has held for $t_{valid}$. The angle is blended over $t_{blend}$.
4. **Fallback** to step 2 if the estimate is lost.

**Details** (one supervisor block owns the sequence, the velocity PI and the angle the current loop uses; `ctrl.sensorless.state` 0 align, 1 I/f, 2 blend, 3 closed loop):

- Align: $i_d^* = I_{align}$, $i_q^* = 0$ at $\theta = 0$; the rotor turns to the d axis.
- I/f: $i_d^* = 0$, $i_q^* = I_{if}\,\mathrm{sign}(\omega^*)$, and $\theta^*$ integrates $\omega^*_e$, which ramps at $a$ towards $N p\,$`ctrl.omega_ref`. A loaded rotor lags θ* by the angle at which $K_t I_{if}\cos\delta$ carries the load.
- Blend: $\theta = \theta^* + w\,\delta$ with $w$ rising linearly 0 → 1 over $t_{blend}$, where $\delta$ is $\hat\theta_e - \theta^*$ **unwrapped** (wrapped at the start of the blend, then the wrapped per-tick change is accumulated); $i_q^*$ stays $I_{if}$. Wrapping $\delta$ every tick jumps it by $2\pi$ when it crosses ±π, which turns the current vector by $w\cdot 2\pi$ in one tick (found: a 48 A spike with a 20 A limit on a 5010 whose I/f lag was ≈ 100°).
- Closed loop: $\theta = \hat\theta_e$, the velocity PI (EQ-CTRL-04) on $\hat\omega_e/p$, started from $i_q^* = I_{if}$ (bumpless).
- Fallback: `est.valid` false for $t_{valid}$ in closed loop → I/f with $\theta^* = \hat\theta_e$ and $\omega^*_e = \hat\omega_e$.
- Reference: `ctrl.omega_ref` is clamped to `controller.limits.speed` and rate-limited by `controller.limits.ramp` (EQ-CTRL-02) before every step uses it, as in the cascade.
- A speed loop only: `controller.position` together with `sensorless: true` is refused at build (the observer has no angle at standstill, so a position hold cannot work).
- Defaults: $I_{align} = I_{if} = 0.5\,I_{lim}$, $t_{align}$ = 0.3 s, $a = 0.25\,K_t I_{if}/(N J)$ (load side), $\omega_{handoff} = 3\,\omega_{min}$, $t_{valid}$ = 20 ms, $t_{blend}$ = 50 ms.
- **Bound** (verified): $i_q^*$ is continuous across the hand-off, and the measured current magnitude stays below `controller.limits.current` through the whole start.

### EQ-CTRL-11 — Encoder offset calibration {/* #eq-ctrl-11 */}

1. Inject $i_d = I_{cal}$ at $\theta_e = 0$. The rotor's d-axis aligns with phase a.
2. Read the encoder and set $\hat\theta_{e,0} = -p\,\theta_{raw} \bmod 2\pi$.
3. Pole-pair and direction check: sweep $\theta_e$ through $n$ electrical turns and measure $\Delta\theta_m$. Then $\hat p = 2\pi n/|\Delta\theta_m|$, and the sign of $\Delta\theta_m$ gives the direction.

**In the simulator** (a routine run on the engine, `control::calibration`): the velocity loop is switched off (its bypass gives $i_q^* = 0$), the current loop is forced to the commanded angle (`ctrl.foc.angle_forced` = 1, `ctrl.foc.angle_cmd`), $i_d^* = I_{cal}$ (default $0.5\,I_{lim}$) for $t_{align}$ (default 0.5 s), then $\hat\theta_{e,0}$ is written to `ctrl.foc.angle_offset`, which the current loop adds every tick ($\hat\theta_e = p\,\theta_{enc} + \hat\theta_{e,0}$; initial value `controller.encoder_offset`, default 0). The sweep runs $n + 1$ turns at one electrical turn per second ($n = 2$), and $\Delta\theta_m$ is read between the commanded angles $\pi$ and $\pi + 2\pi n$: the rotor lags the command by the same angle at both, while a reading after the sweep stops would include the d-axis spring's barely damped swing (found: $\hat p$ = 10.6 for 11). The swing excited at the start of the sweep still leaves ≈ 0.5 % ($\hat p$ = 10.94), so $\hat p$ is rounded to identify $p$. The recovered offset is only as good as the encoder: its INL times $p$ is an electrical error (0.8° INL × 11 pole pairs ≈ 9°), and a load or cogging torque during the alignment shifts the rest angle.

## Auto-tune

### EQ-CTRL-12 — Measurement and gain computation {/* #eq-ctrl-12 */}

Runs on a copy of the scene (D-006).

**Measurements:**

| Quantity | Method |
|---|---|
| $R$ | two DC levels on the α axis: $R = \Delta v / \Delta i$ (the difference cancels dead time and diode offsets) |
| $L$ | voltage step, fit $i(t) = (V/R)(1 - e^{-tR/L})$ |
| $J$ | two torque levels: $J = K_t(i_2 - i_1)/(\dot\omega_2 - \dot\omega_1)$ (the difference cancels friction) |
| $\lambda_m$ | spin with $i \approx 0$: $\lambda_m = v_q/\omega_e$ |

**Gains:**

| Loop | Default |
|---|---|
| current | $\omega_c = 2\pi f_{ctrl}/20$ |
| velocity | $\omega_v = \omega_c/10$ |
| position | $\omega_p = \omega_v/5$ |
| MIT | $K_p = J\,\omega_{mit}^2$ with $\omega_{mit} = \omega_v/5$; $K_d = 2 \cdot 0.7\sqrt{K_p J}$ |

The gains are applied only after the user confirms. Validation: the measured values are within 5 % of the true parameters, and the achieved bandwidth is within ±20 % of the target.

**What is measured** (the simulator's routine, `control::autotune`): R is the resistance the current loop sees, the winding's $R(T)$ at its present temperature plus the switch $R_{on}$ (so the "true" R for the check is $R(T) + R_{on}$; the result reports the winding temperature). λ is the slope $dv_q/d\omega_e$ at constant current during the J test, not $v_q/\omega_e$ at $i \approx 0$: the dead-time voltage the current loop compensates is a constant offset in $v_q$ (≈ 0.65 V on a 5010 at 48 V, three times its back-EMF at the coasting speed), and only the slope removes it. J uses +I then −I windows (friction cancels, the speed comes back down), each stopped at half the voltage limit. The test currents are 25 %/50 % (R, L) and 30 % (J, λ) of `controller.limits.current`: on a low-R, low-L motor, a limit far below its rating leaves the dead-time voltage dominant and the current ragged (measured: 5010 at 4 A gave λ +15 %; at its 25 A rating +0.4 %). The achieved bandwidth is checked small-signal: a q step between two currents of the same sign (25 % → 50 %), timed from the change of $i_q^*$ (τ63 ≈ $1/\omega_c$). A step from zero is slower (8010: 3.6×) because the phase currents leave zero and the dead-time voltage appears during the rise: a large-signal effect, not the loop's bandwidth.

## References

`[Krishnan2010]` (FOC, six-step), `[Mohan2014]` (current-loop design), `[Lee2010]`, `[Ortega2011]` (observer), `[ODrive]`, `[VESC]`, `[SimpleFOC]` (practical tuning and calibration). See [References](./references.md).
