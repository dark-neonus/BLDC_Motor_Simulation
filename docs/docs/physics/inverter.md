---
title: Inverter and modulation
---

# Inverter and modulation

:::tip In plain words
A motor driver connects each motor wire either to **+** (high), to **−** (low), or to nothing (off), thousands of times per second (PWM). By choosing how long each wire spends high, it creates any average voltage it wants. This page explains those switches, the tiny safety pause between them (dead time), the shortcut "averaged" model, the patterns used by FOC (SVPWM) and six-step, and where the losses go.
:::

## Assumptions

1. A two-level, three-leg inverter. Each leg has two MOSFETs, each with an anti-parallel diode.
2. A MOSFET that is on is a resistor $R_{on}$ in both current directions (synchronous rectification). A diode is a constant drop $V_f$.
3. Switching is **instantaneous** in the electrical solution. Switching energy is accounted separately as a bus-current draw (EQ-INV-09).
4. Gate-driver delays other than dead time are ignored.

## Symbols

| Symbol | Meaning | Unit | Code / signal |
|---|---|---|---|
| $V_{bus}$ | DC bus voltage | V | `bus.v` |
| $S_x \in \{H, L, O\}$ | state of leg $x$: High, Low, Off | – | `inverter.leg_a` … |
| $d_x$ | duty of leg $x$ (fraction of the period High) | – | `inverter.duty_a` … |
| $T_{pwm} = 1/f_{pwm}$ | PWM period | s | `inverter.f_pwm` |
| $t_d$ | dead time | s | `inverter.dead_time` |
| $R_{on}$, $V_f$ | MOSFET on-resistance, diode drop | Ω, V | `inverter.r_on`, `inverter.v_f` |
| $t_r, t_f$ | switching rise and fall times | s | `inverter.t_rise`, `inverter.t_fall` |
| $i_x$ | phase current, positive **into** the motor | A | `motor.i_a` … |

Typical values for small low-voltage drives: $R_{on} = 2\ldots20$ mΩ, $V_f = 0.7\ldots1.0$ V, $t_r, t_f = 10\ldots100$ ns, $t_d = 0.2\ldots1$ µs, $f_{pwm} = 20\ldots40$ kHz.

## Switching model

### EQ-INV-01 — Terminal voltage of a leg {/* #eq-inv-01 */}

Terminal voltage w.r.t. the DC− rail:

$$
v_{xT} =
\begin{cases}
V_{bus} - R_{on}\,i_x & S_x = H \\
-R_{on}\,i_x & S_x = L \\
-V_f & S_x = O,\ i_x > 0 \quad\text{(low-side diode conducts)}\\
V_{bus} + V_f & S_x = O,\ i_x < 0 \quad\text{(high-side diode conducts)}\\
\text{floating, set by the motor (EQ-MOT-12)} & S_x = O,\ i_x = 0
\end{cases}
$$

A leg that is Off with zero current puts the motor into open-phase mode (EQ-MOT-11). A floating terminal that tries to leave $[-V_f,\ V_{bus} + V_f]$ turns its diode on (a state event), which ends open-phase mode.

### EQ-INV-02 — Center-aligned PWM with dead time {/* #eq-inv-02 */}

The carrier is center-aligned (counter counts up then down) with period $T_{pwm}$. Within period $k$ (from $t_k = kT_{pwm}$), leg $x$ is commanded High during

$$
\Big[\,t_k + \tfrac{1-d_x}{2}T_{pwm},\ \ t_k + \tfrac{1+d_x}{2}T_{pwm}\Big]
$$

and Low otherwise. **Dead time** delays every turn-on by $t_d$:

- Low → Off at the edge, Off → High $t_d$ later;
- High → Off at the edge, Off → Low $t_d$ later.

During that gap the leg follows the diode rule of EQ-INV-01. Duties are clamped to $[0, 1]$, and a duty within $t_d/T_{pwm}$ of 0 or 1 produces no pulse (pulse dropping).

- **Shadow registers:** the duties are latched at $t_k$, so a new duty takes effect at the next period, like on real MCUs.
- **Events:** each switch edge is a scheduled time event (EQ-NUM). The event `pwm.center` fires at $t_k$, the middle of the all-Low interval (zero vector $V_0$). There the low-side shunts conduct and the current ripple is at its midpoint, so ADCs sample there (EQ-SENS).

## Averaged model

### EQ-INV-03 — Period-averaged terminal voltage {/* #eq-inv-03 */}

For fast simulation (`fidelity.inverter_mode = averaged`), the switching inside each period is replaced by its average:

$$
\bar v_{xT} = d_x\,V_{bus} - R_{on}\,i_x - \sigma(i_x)\,\frac{t_d}{T_{pwm}}\,\big(V_{bus} + V_f\big),
\qquad \sigma(i) = \tanh(i/i_\epsilon)
$$

The last term is the **dead-time voltage error**:

- $i_x > 0$: the high side loses $t_d$ of on-time per period;
- $i_x < 0$: it gains $t_d$.

$\sigma$ smooths the sign with $i_\epsilon$ ≈ 1 % of rated current. A leg commanded Off uses the same diode / floating rule as EQ-INV-01, so six-step floating phases behave the same in both modes.

Validation (V-INV-avg): in steady state, the period-averaged phase currents of the switching model match the averaged model within 1 % of rated current.

## Modulation

### EQ-INV-04 — Sine PWM {/* #eq-inv-04 */}

From phase voltage references $v_x^*$:

$$
d_x = \tfrac12 + \frac{v_x^*}{V_{bus}}
$$

The linear range is $|v^*| \le V_{bus}/2$.

### EQ-INV-05 — SVPWM by min–max injection {/* #eq-inv-05 */}

Add the same offset to all three references:

$$
v_o = -\tfrac12\Big(\max_x v_x^* + \min_x v_x^*\Big),\qquad
d_x = \tfrac12 + \frac{v_x^* + v_o}{V_{bus}}
$$

- This is identical to classic sector-based SVPWM with the zero vectors split equally `[HolmesLipo2003]`. Verified: the two methods give the same duties to within $4\cdot10^{-16}$ at 36 angles.
- The common offset cancels in the phase voltages (it lands on the isolated neutral).
- The linear range is $|v^*_{\alpha\beta}| \le V_{bus}/\sqrt3$, 15.5 % more than sine PWM.

**Worked example.** $V_{bus} = 24$ V and a reference of amplitude 10 V at $\theta = 0.3$ rad:

| Quantity | Value |
|---|---|
| $v_a^*, v_b^*, v_c^*$ | 9.5534, −2.2174, −7.3360 V |
| $v_o$ | −1.1087 V |
| $d_a, d_b, d_c$ | 0.8519, 0.3614, 0.1481 |

### EQ-INV-06 — Overmodulation clamp {/* #eq-inv-06 */}

If the requested vector is longer than the linear limit, keep its **angle** and shorten it:

$$
v^*_{\alpha\beta} \leftarrow v^*_{\alpha\beta}\cdot\min\!\Big(1,\ \frac{V_{bus}/\sqrt3}{|v^*_{\alpha\beta}|}\Big)
$$

The controller is told about the saturation so it can stop integrating (anti-windup, EQ-CTRL).

### EQ-INV-07 — Six-step commutation table {/* #eq-inv-07 */}

Six-step drives two phases: the phase with the most positive back-EMF goes High (PWM'd), the one with the most negative goes Low, and the third is Off. Derived from the EMF convention $e_x \propto -\sin(\theta_e - x\cdot120°)$ (EQ-MOT-03) for forward rotation:

| $\theta_e$ range | High | Low | Off |
|---|---|---|---|
| 330° … 30° | B | C | A |
| 30° … 90° | B | A | C |
| 90° … 150° | C | A | B |
| 150° … 210° | C | B | A |
| 210° … 270° | A | B | C |
| 270° … 330° | A | C | B |

- PWM is applied to the high-side switch. The low side stays on for the whole sector, and during the PWM off-time the current freewheels through the low-side diode of the High leg.
- Reverse rotation swaps High and Low.
- An optional advance angle shifts the table earlier.
- Hall sensors give these sectors directly (EQ-SENS).

## Losses and DC-link current

### EQ-INV-08 — DC-link current {/* #eq-inv-08 */}

**Switching model.** The bus current is the current through the high side, including the high-side diodes:

$$
i_{dc} = \sum_x i_x\cdot\big[S_x = H \ \lor\ (S_x = O \land i_x < 0)\big]
$$

**Averaged model.** The dead time changes the effective high-side duty:

$$
\bar i_{dc} = \sum_x \Big(d_x - \sigma(i_x)\frac{t_d}{T_{pwm}}\Big)\,i_x
$$

### EQ-INV-09 — Losses {/* #eq-inv-09 */}

| Loss | Formula | How it is accounted |
|---|---|---|
| MOSFET conduction | $R_{on}\,i_x^2$ for each conducting switch | implicit in the terminal voltages |
| diode conduction | $V_f\,\lvert i_x\rvert$ while a diode conducts; averaged: $V_f\,\lvert i_x\rvert\,t_d/T_{pwm}$ | implicit in the terminal voltages |
| switching | $\tfrac12 V_{bus}\lvert i_x\rvert(t_r + t_f)$ per leg per PWM period | an extra bus current $i_{sw} = f_{pwm}\sum_x \tfrac12\lvert i_x\rvert(t_r + t_f)$, the same in both modes |

The conduction terms follow automatically: with EQ-INV-01/08, $V_{bus}\,i_{dc} - \sum_x v_{xT}\,i_x$ is exactly the conduction plus diode loss. This was checked case by case for every leg state.

Switching loss is drawn from the bus as $i_{sw}$ because the electrical solution switches ideally. Its loss power is $V_{bus}\,i_{sw}$, so the energy balance stays exact.

Total inverter loss: $P_{inv} = V_{bus}(i_{dc} + i_{sw}) - \sum_x v_{xT}\,i_x$. The heat is reported but not fed into the motor's thermal network: the inverter has no thermal model in v1.

## Dimensional check

| Equation | Check |
|---|---|
| EQ-INV-01 | V − Ω·A = V ✓ |
| EQ-INV-03 | – · V − Ω·A − (s/s)·V = V ✓ |
| EQ-INV-09 | $f\cdot$A·s = A ✓; V·A = W ✓ |

## References

- `[HolmesLipo2003]`: carrier PWM, SVPWM and zero-sequence injection, dead-time effects.
- `[Mohan2014]` ch. 6: SVPWM, average models.
- `[Krishnan2010]` ch. 9: six-step BLDC commutation.

See [References](./references.md).
