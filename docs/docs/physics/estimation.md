---
title: Parameter estimation
---

# Parameter estimation

:::tip In plain words
Motor datasheets for gimbal and robot motors often list only Kv, resistance and weight. To simulate the motor you also need its inductance, how heavy its rotor feels (inertia) and how fast it heats up. These rules give **reasonable guesses** from the size and weight. Every guessed value is marked `estimated`, and a measured or datasheet value always replaces it.
:::

This page holds the rules used by the motor presets (P04.T09). The datasheet wizard (P04.T12) adds more.

## Symbols

| Symbol | Meaning | Unit |
|---|---|---|
| $m$ | motor mass | kg |
| $D$ | outer diameter | m |
| $H$ | motor height | m |
| $R$ | phase resistance (star-equivalent) | Ω |
| $\tau_e$ | electrical time constant $L/R$ | s |

### EQ-EST-01 — Rotor inertia of an outrunner {/* #eq-est-01 */}

$$
J_{rotor} \approx 0.35\, m \left(\frac{D}{2}\right)^2
$$

The rotating can and magnets carry roughly 40 % of the mass at roughly 90 % of the outer radius ($0.4 \cdot 0.9^2 \approx 0.32$). Checked against CubeMars RO100 (0.71 kg, Φ108 mm): the rule gives 7200 g·cm², and the listings give 5290–8700 g·cm².

### EQ-EST-02 — Inductance from a typical time constant {/* #eq-est-02 */}

$$
L_d = L_q \approx \tau_e\, R,\qquad
\tau_e \approx \begin{cases} 0.3\ \text{ms} & \text{gimbal windings } (R_{LL} > 1\ \Omega)\\ 1\ \text{ms} & \text{drone / robot windings} \end{cases}
$$

Surface magnets give $L_d \approx L_q$. Published pairs span 0.3–4 ms (CubeMars AK80 0.34 ms, RO100 0.96 ms, RI100 2.9 ms, AK10-9 3.7 ms, DJI GM6020 3.2 ms), so this is a rough guess and the value matters mainly for current-loop tuning.

### EQ-EST-03 — Thermal network from mass and size {/* #eq-est-03 */}

$$
\begin{aligned}
C_w &= 0.15\,m \cdot 385, & C_s &= 0.35\,m \cdot 460, & C_h &= 0.40\,m \cdot 900 \quad [\mathrm{J/K}]\\
R_{ha} &= \frac{1}{h A},\ A = \pi D H + \tfrac{\pi}{2} D^2,\ h = 12\ \mathrm{W/(m^2 K)}, & R_{ws} &= 0.5\ \mathrm{K/W}, & R_{sh} &= 0.3\ \mathrm{K/W}
\end{aligned}
$$

Mass fractions (copper 15 %, iron 35 %, body 40 %) and specific heats follow the [thermal model](./thermal.md#eq-therm-01) table. $h$ is natural convection plus radiation. $R_{ws}$ and $R_{sh}$ are mid-range values from that table. $T_{w,max} = 100$ °C unless the datasheet gives a value.

## Dimensional check

| Equation | Check |
|---|---|
| EQ-EST-01 | kg · m² ✓ |
| EQ-EST-02 | s · Ω = H ✓ |
| EQ-EST-03 | kg · J/(kg·K) = J/K; 1/(W/(m²K) · m²) = K/W ✓ |
