---
title: Sensors
---

# Sensors

:::tip In plain words
A controller never sees the true motor state. It sees **sensor readings**, which are delayed, coarse (finite resolution), noisy and slightly wrong. A magnetic encoder glued to the shaft (AS5600, AS5047P, MT6701), three Hall switches, or current-sensing resistors all show this in different amounts. Making these imperfections visible and adjustable is one of the main teaching goals of the simulator.
:::

## The sensor pipeline

### EQ-SENS-01 — Common non-ideality pipeline {/* #eq-sens-01 */}

Every sensor turns the true quantity $x(t)$ into a reading $y_k$ at its sample instants $t_k$ (fixed update rate, or triggered by an event such as `pwm.center`):

$$
y_k = \mathrm{sat}\Big(Q\big(\,x(t_k - \tau) + e\big(x(t_k-\tau)\big) + n_k\,\big)\Big),\qquad n_k \sim \mathcal N(0,\sigma^2)
$$

The reading is held (zero-order hold) until $t_{k+1}$.

| Term | Meaning |
|---|---|
| $\tau$ | **latency**: the reading describes the past. The true signal history is kept in a buffer and read with linear interpolation when $\tau$ is not a multiple of the event grid. |
| $e(\cdot)$ | deterministic error: nonlinearity, eccentricity, gain/offset |
| $n_k$ | white Gaussian noise from the block's seeded RNG (deterministic per scene seed, CONVENTIONS §6) |
| $Q$ | quantisation $Q(x) = \Delta\,\mathrm{round}(x/\Delta)$; angles wrap to $[0, 2\pi)$ |
| $\mathrm{sat}$ | range limits (ADC full scale) |

**Bypass** (`…bypass = true`) returns $y(t) = x(t)$ exactly and continuously: no delay, no sampling, no noise. This is the "ideal feedback" switch on every sensor block.

## Hall sensors

### EQ-SENS-02 — Three digital Hall sensors {/* #eq-sens-02 */}

$$
h_x = \Big[\cos\!\big(\theta_e - (x+1)\cdot120° - \delta_x\big) > \tfrac{h}{2}\cdot s_x\Big],\qquad x = a, b, c = 0, 1, 2
$$

- $\delta_x$ is the placement error of each sensor (electrical degrees).
- $h$ is the magnetic hysteresis, in units of the normalised field. $s_x = +1$ while $h_x = 0$ and $-1$ while $h_x = 1$, so the output switches on at $+h/2$ and off at $-h/2$.

With $\delta = h = 0$ the codes change exactly at the six-step boundaries of EQ-INV-07:

| $\theta_e$ | $(h_a, h_b, h_c)$ | code | six-step pair |
|---|---|---|---|
| 330°…30° | 0 0 1 | 1 | B+ C− |
| 30°…90° | 1 0 1 | 5 | B+ A− |
| 90°…150° | 1 0 0 | 4 | C+ A− |
| 150°…210° | 1 1 0 | 6 | C+ B− |
| 210°…270° | 0 1 0 | 2 | A+ B− |
| 270°…330° | 0 1 1 | 3 | A+ C− |

Codes 0 and 7 never occur in a healthy sensor set; the controller treats them as a fault. Resolution is 60° electrical, i.e. $60°/p$ mechanical (4.3° for $p = 14$).

## Magnetic angle encoders

### EQ-SENS-03 — On-axis magnetic encoder {/* #eq-sens-03 */}

A diametrically magnetised magnet sits on the shaft end and a Hall-array chip faces it across an air gap $z$:

$$
\theta_{meas} = Q_{b}\Big(\theta_m(t_k - \tau) + \theta_{0} + e_{INL}(\theta_m) + e_{ecc}(\theta_m) + n_k\Big),
\qquad \Delta = \frac{2\pi}{2^{b}}
$$

| Term | Model |
|---|---|
| $\theta_0$ | mounting offset (unknown to the controller until calibration, P09.T09) |
| $e_{INL}$ | intrinsic nonlinearity $\sum_{n=1}^{3} a_n\sin(n\theta_m + \varphi_n)$, peak scaled to the datasheet INL |
| $e_{ecc}$ | magnet–chip lateral displacement $r_d$: first-harmonic error $a_{ecc}\sin(\theta_m + \varphi_{ecc})$. **Approximation:** $a_{ecc}$ scales linearly with $r_d$, from the datasheet's "INL at displacement" figure |
| $\sigma(z)$ | noise grows outside the recommended air gap (EQ-SENS-04) |

### EQ-SENS-04 — Air gap and noise (approximation) {/* #eq-sens-04 */}

The magnet's field at the chip falls roughly like a dipole far from the magnet, $B \propto z^{-3}$. The chips work over a field window $[B_{min}, B_{max}]$, which corresponds to a recommended gap $[z_{min}, z_{max}]$ for a given magnet. The model:

$$
\sigma(z) = \sigma_{nom}\cdot\max\!\Big(1,\ \big(z/z_{max}\big)^{3}\Big),
\qquad
a_{INL}(z) = a_{INL,nom}\cdot\big(1 + k_{sat}\,\max(0,\ z_{min} - z)/z_{min}\big)
$$

- **Too far:** weak field, noise rises as $B_{min}/B$.
- **Too close:** the AGC saturates and nonlinearity grows; $k_{sat} = 1$ by default.
- A warning is raised outside $[z_{min}, z_{max}]$ (P10.T06).

This is a teaching approximation of the trend, not a magnetic field solution.

### Encoder chip presets

Values from the manufacturer datasheets `[AS5047P]` `[AS5600]` `[MT6701]`:

| Parameter | AS5047P | AS5600 | MT6701 |
|---|---|---|---|
| core resolution | 14 bit | 12 bit | 14 bit |
| update rate | ~4.5 MHz internal (222 ns), read over SPI | 150 µs sampling | ~5 µs system delay, 14-bit SSI |
| latency $\tau$ | 90–110 µs over SPI; 1.5–1.9 µs with DAEC on ABI/UVW | 150 µs + output filter (settling 2.2 / 1.1 / 0.55 / 0.286 ms for SF = 00…11) | ≈ 5 µs |
| INL (optimal placement) | ±0.8° (±1.0° over temperature) | ±1° | ±1° typ. (±1.5° over temperature) |
| INL with 0.5 mm magnet displacement | ±1.2° | – | – |
| noise (1σ) | 0.068° (0.052° with DAEC off) | 0.015° (SF=00) … 0.043° (SF=11) | not specified; preset uses 0.05° (**estimate**) |
| field window $B_z$ | 35–70 mT | 30–90 mT | – |
| max speed | 28 krpm | – | 55 krpm |

**DAEC** (AS5047P dynamic angle-error compensation) extrapolates the angle by $\omega\,t_{delay}$. It is modelled as latency $t_{delay,DAEC}$ plus 0.016° extra rms noise, matching the datasheet note.

AS5600's slow filter is modelled as a first-order low-pass with time constant $\tau_f = T_{settl}/4$ (**approximation**, from the settling time to the 14-bit step), in series with the 150 µs sampling.

## Incremental optical encoder

### EQ-SENS-05 — Quadrature encoder {/* #eq-sens-05 */}

With $N_{CPR}$ lines per revolution, quadrature decoding gives $4N_{CPR}$ counts:

$$
c_k = \Big\lfloor \frac{4N_{CPR}}{2\pi}\big(\theta_m(t_k) - \theta_{z}\big) \Big\rfloor,
\qquad \theta_{meas} = \frac{2\pi}{4N_{CPR}}\,c_k
$$

- The reading is **relative** until the index pulse is seen. The index pulse fires once per revolution at $\theta_m \equiv \theta_{index}$, and then the absolute offset is known.
- Missed counts (fault `encoder.missed_counts`, P10.T02) drop $m$ counts at random edges.
- Typical 1000 CPR gives 4000 counts/rev (0.09°).

## Current and voltage sensing

### EQ-SENS-06 — ADC with shunt amplifier {/* #eq-sens-06 */}

$$
i_{meas} = \mathrm{sat}_{[-I_{fs},\,I_{fs}]}\Big(Q_{\Delta}\big((1 + g_{err})\,i_x + i_{off} + n\big)\Big),
\qquad \Delta = \frac{2I_{fs}}{2^{b}}
$$

- **Sampling instant:** at `pwm.center` in switching mode. That is the middle of the zero vector, where low-side shunts conduct and the ripple is at its midpoint, so the reading approximates the period-average current. In averaged mode the ADC samples at the controller rate.
- **Two-shunt** layout: $i_c = -i_a - i_b$ is reconstructed (its error includes both channels). **Three-shunt** layout: all three are measured.
- **Bus voltage:** the same model with a divider and its own full scale.
- **What it teaches** (validation case V-SENS-offset): an offset $i_{off}$ on one phase produces an $i_d, i_q$ ripple at $\omega_e$, i.e. torque ripple at the electrical frequency.

Typical: 12-bit ADC, $I_{fs}$ = 1.2–2 × peak current, offsets of a few LSB, gain error ±1 %.

## Estimators

Velocity estimation (difference + low-pass, PLL) and the sensorless flux observer are signal-processing blocks, not physical sensors, so they are specified in [Control](./control.md) (EQ-CTRL).

## References

- `[AS5047P]`, `[AS5600]`, `[MT6701]`: chip datasheets (values in the table above).
- `[Krishnan2010]` ch. 9: Hall-sensor commutation.
- `[Mohan2014]` ch. 7: current sensing for FOC.

See [References](./references.md).
