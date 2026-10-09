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
- The PI zero gives **13.5 % step overshoot**; the optional setpoint prefilter $\dfrac{1}{1 + 4s/\omega_v}$ removes it (verified numerically).

**Position P (or PI) → $\omega^*$** on the load-side angle: $K_{p,p} = \omega_p$, $\omega_p \le \omega_v/5$.

- The output is converted to motor speed (×$N$) and clamped to the speed limit.
- Loop semantics: a disabled outer loop passes its setpoint straight through, and each loop has its own rate and ramp limiter.
- A P-only position loop under constant load torque holds a steady error of $\tau_L/(N^2\,K_t\,K_{p,v}\,K_{p,p})$ when the velocity loop is P-only, and zero error with a PI velocity loop.

## Other controllers

### EQ-CTRL-05 — Six-step with Hall sensors {/* #eq-ctrl-05 */}

1. The Hall code selects the leg pattern (EQ-SENS-02 → EQ-INV-07).
2. Duty: either set directly, or from a speed PI with output clamped to $[0, 1]$.
3. Direction swaps the High and Low legs.
4. An optional advance angle shifts commutation by up to 30°.

### EQ-CTRL-06 — Open-loop V/f {/* #eq-ctrl-06 */}

$$
\theta^*_e = \int \omega^*_e\,dt\ \ (\text{ramped at } a),\qquad
|v| = V_0 + k_{vf}\,|\omega^*_e|,\qquad
v_{\alpha\beta} = |v|\,[-\sin\theta^*_e,\ \cos\theta^*_e]
$$

There is no feedback. **Loss of synchronism** is flagged when $|\mathrm{wrap}(\theta^*_e - \theta_e)| > \pi/2$ for more than one electrical period.

### EQ-CTRL-07 — Impedance / MIT mode {/* #eq-ctrl-07 */}

$$
\tau^* = K_p(\theta^* - \theta_L) + K_d(\omega^* - \omega_L) + \tau_{ff},\qquad
i_q^* = \mathrm{clamp}\!\Big(\frac{\tau^*}{N\,K_t}\Big)
$$

- Validation (V-CTRL-mit): static deflection under load is $\tau_L/K_p$.
- Damping ratio: $\zeta = K_d / (2\sqrt{K_p J_{L,tot}})$.
- The law runs on top of the current loop at its own rate.
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

with $K_p = 2\zeta\omega_n$ and $K_i = \omega_n^2$. For $\zeta = 1$ the −3 dB bandwidth is $2.48\,\omega_n$ (verified).

### EQ-CTRL-09 — Nonlinear flux observer `[Lee2010]` {/* #eq-ctrl-09 */}

$$
\eta = x - L_s\,i_{\alpha\beta},\qquad
\dot x = v_{\alpha\beta} - R\,i_{\alpha\beta} + \frac{\gamma}{2}\,\eta\,\big(\lambda_m^2 - |\eta|^2\big),\qquad
\hat\theta_e = \mathrm{atan2}(\eta_\beta, \eta_\alpha)
$$

- Speed comes from a PLL on $\hat\theta_e$. The radial convergence rate is $\gamma\lambda_m^2$; default $\gamma = 1000/\lambda_m^2$.
- `est.valid` is set when $|\hat\omega_e| > \omega_{min}$ and $\big||\eta| - \lambda_m\big| < 0.2\lambda_m$.
- **Expected low-speed failure:** the back-EMF shrinks with speed while errors in $R$, offsets and dead time do not. Verified with a 3 % resistance error:

  | $\omega_e$ [rad/s] | 700 | 140 | 14 | 1.4 |
  |---|---|---|---|---|
  | mean angle error | 0.5° | 13° | 126° | 64° |

### EQ-CTRL-10 — Sensorless startup {/* #eq-ctrl-10 */}

1. **Align:** $i_d = I_{align}$ at $\theta = 0$ for $t_{align}$.
2. **Open-loop current ramp (I/f):** $i_q^*$ fixed, $\theta^*$ ramped.
3. **Hand-off** when $|\omega^*| > \omega_{handoff}$ and `est.valid` has held for $t_{valid}$. The angle is blended over $t_{blend}$.
4. **Fallback** to step 2 if the estimate is lost.

### EQ-CTRL-11 — Encoder offset calibration {/* #eq-ctrl-11 */}

1. Inject $i_d = I_{cal}$ at $\theta_e = 0$. The rotor's d-axis aligns with phase a.
2. Read the encoder and set $\hat\theta_{e,0} = -p\,\theta_{raw} \bmod 2\pi$.
3. Pole-pair and direction check: sweep $\theta_e$ through $n$ electrical turns and measure $\Delta\theta_m$. Then $\hat p = 2\pi n/|\Delta\theta_m|$, and the sign of $\Delta\theta_m$ gives the direction.

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

## References

`[Krishnan2010]` (FOC, six-step), `[Mohan2014]` (current-loop design), `[Lee2010]`, `[Ortega2011]` (observer), `[ODrive]`, `[VESC]`, `[SimpleFOC]` (practical tuning and calibration). See [References](./references.md).
