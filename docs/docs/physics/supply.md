---
title: Power supply and DC bus
---

# Power supply and DC bus

:::tip In plain words
The driver runs from a **DC bus**: a capacitor fed by a lab power supply or a battery. When the motor *brakes*, it works as a generator and pushes energy back into the bus. A battery can absorb that energy. A lab supply **cannot**, so the bus voltage climbs, sometimes high enough to damage things. A **brake resistor** (chopper) burns the excess energy as heat. This page models all of that.
:::

## Assumptions

1. One DC bus node with capacitance $C_{bus}$ (driver capacitors plus supply output capacitance).
2. Supply wiring resistance is included in the source's output resistance. Cable inductance is ignored.
3. Exactly one source is active per scene: `ideal`, `psu` or `battery`. A brake chopper is optional.

## Symbols

| Symbol | Meaning | Unit | Code / signal |
|---|---|---|---|
| $V_{bus}$ | bus voltage (**state**) | V | `bus.v` |
| $C_{bus}$ | bus capacitance | F | `bus.capacitance` |
| $i_{src}$ | current from the source into the bus (+ = supplying) | A | `supply.i` |
| $i_{dc}, i_{sw}$ | inverter DC-link and switching-loss currents (EQ-INV-08/09) | A | `inverter.i_dc`, `inverter.i_sw` |
| $i_{ch}$ | brake chopper current | A | `bus.i_chopper` |
| $V_{set}, I_{lim}, R_o$ | PSU voltage setpoint, current limit, output resistance | V, A, Ω | `supply.psu.*` |
| $SoC$ | battery state of charge (**state**, 0…1) | – | `supply.soc` |
| $v_1$ | battery RC-branch voltage (**state**) | V | `supply.battery.v_rc` |

## Bus

### EQ-SUP-01 — DC bus capacitor {/* #eq-sup-01 */}

$$
C_{bus}\,\dot V_{bus} = i_{src} - i_{dc} - i_{sw} - i_{ch}
$$

Stored energy: $\tfrac12 C_{bus}V_{bus}^2$. Typical small drives have $C_{bus} = 100\ \mu\mathrm F\ldots2\ \mathrm{mF}$.

## Sources

### EQ-SUP-02 — Ideal source (tests) {/* #eq-sup-02 */}

$V_{bus} \equiv V_{set}$, an algebraic constraint: the bus capacitor state is removed and $i_{src} = i_{dc} + i_{sw} + i_{ch}$. Its power can be positive or negative (it sources and sinks freely).

### EQ-SUP-03 — Lab power supply (CV/CC, cannot sink) {/* #eq-sup-03 */}

$$
i_{src} = \mathrm{clamp}\!\left(\frac{V_{set} - V_{bus}}{R_o},\ 0,\ I_{lim}\right)
$$

| Mode (`supply.mode`) | Condition | Behaviour |
|---|---|---|
| `CV` | $0 < i_{src} < I_{lim}$ | constant voltage; the bus sits $R_o\,i_{src}$ below $V_{set}$ |
| `CC` | $i_{src} = I_{lim}$ | constant current; the bus voltage **sags** while the load wants more |
| `BLOCKING` | $V_{bus} \ge V_{set}$ | the supply cannot sink current; regenerated energy only charges $C_{bus}$, so $V_{bus}$ **rises** |

- Mode changes are state events, so the kinks in the clamp are located exactly (EQ-NUM).
- Typical $R_o$ (supply plus leads): 20–100 mΩ.
- **Approximation:** real supplies have slower CC loops, foldback and their own OVP. The `BLOCKING` rise above $V_{set}$ is what makes a lesson about regen damaging a PSU possible (P18.T10), and the overvoltage warnings and protection use it (EQ-SUP-06).

### EQ-SUP-04 — Battery (Thevenin, one RC branch) {/* #eq-sup-04 */}

The battery terminals are the bus, so the battery current follows from the bus voltage:

$$
i_{src} = \frac{OCV(SoC) - v_1 - V_{bus}}{R_0},\qquad
\dot v_1 = \frac{i_{src}}{C_1} - \frac{v_1}{R_1 C_1},\qquad
\dot{SoC} = -\frac{i_{src}}{3600\,Q_{Ah}}
$$

- **Bidirectional:** $i_{src} < 0$ charges the battery, so regenerated energy is absorbed.
- Pack values are built from cells: $s$ in series and $p$ in parallel give $OCV = s\cdot ocv_{cell}$, $R_0 = (s/p)\,r_{0,cell}$, $R_1 = (s/p)\,r_{1,cell}$, $C_1 = (p/s)\,c_{1,cell}$ and $Q = p\,q_{cell}$.
- **Why this model and not Tremblay's:** the 1-RC Thevenin model `[Chen2006]` is symmetric for charge and discharge, has a clean energy definition, and is the standard choice for simulation. Tremblay's datasheet-curve method `[Tremblay2009]` is used in the presets to *fit* $OCV(SoC)$ and $r_0$ from manufacturer discharge curves.
- **Energy:** chemical energy $E_{chem}(SoC) = 3600\,Q_{Ah}\int_0^{SoC}OCV(s)\,ds$; RC storage $\tfrac12 C_1 v_1^2$; losses $R_0 i^2 + v_1^2/R_1$. Verified numerically: a 200 s constant-current discharge closes the balance to $1.3\cdot10^{-6}$.

Per-cell open-circuit voltage, typical tables (**approximation**; the presets cite their sources in `presets/supplies/_sources.md`, P04.T10):

| Chemistry | SoC 0 | 0.1 | 0.2 | 0.5 | 0.8 | 0.9 | 1.0 |
|---|---|---|---|---|---|---|---|
| Li-ion / LiPo (NMC/LCO) | 3.00 | 3.40 | 3.55 | 3.70 | 3.95 | 4.05 | 4.20 |
| LiFePO4 | 2.50 | 3.10 | 3.20 | 3.28 | 3.32 | 3.35 | 3.60 |

## Brake chopper

### EQ-SUP-05 — Brake chopper with hysteresis {/* #eq-sup-05 */}

$$
i_{ch} = \begin{cases} V_{bus}/R_b & \text{chopper on}\\ 0 & \text{off}\end{cases},
\qquad
\text{on when } V_{bus} > V_{on},\ \text{off when } V_{bus} < V_{off}\ \ (V_{off} < V_{on})
$$

- The threshold crossings are state events.
- Dissipated power is $V_{bus}^2/R_b$ (signals `bus.chopper_on`, `bus.p_brake`).
- The resistor must stay below its energy and power rating. A warning is raised if the time-averaged power exceeds the rating.

### Regeneration example (V-SUP-regen)

A flywheel with $J = 2.5\cdot10^{-4}$ kg·m² spinning at 30 rad/s stores $\tfrac12 J\omega^2 = 0.1125$ J. If it is braked to rest into a 470 µF bus at 24 V, with a PSU (blocking) and no chopper, the upper bound (lossless) is

$$
V_2 = \sqrt{V_1^2 + \frac{2E}{C_{bus}}} = \sqrt{24^2 + \frac{0.225}{470\cdot10^{-6}}} = 32.48\ \mathrm V
$$

With losses the real value is lower. The validation case checks that the final bus voltage stays at or below 32.48 V and that the energy balance closes.

## Voltage limits

### EQ-SUP-06 — Over- and under-voltage {/* #eq-sup-06 */}

Defined on $V_{bus}$ with a filter time $t_{flt}$ (P10.T03):

- **OV** if $V_{bus} > V_{OV}$ for longer than $t_{flt}$;
- **UV** if $V_{bus} < V_{UV}$ for longer than $t_{flt}$.

Defaults: $V_{OV}$ = the lower of the inverter's rating and 1.25·$V_{set}$; $V_{UV}$ = 0.7·$V_{set}$, or the battery's cut-off voltage. The protection logic decides the action (disable PWM, latch).

## Dimensional check

| Equation | Check |
|---|---|
| EQ-SUP-01 | F·V/s = C/s = A ✓ |
| EQ-SUP-04 | V/Ω = A ✓; A/F = V/s ✓; A/(A·h · s/h) = 1/s ✓ |

## References

- `[Chen2006]`: Thevenin battery model with $OCV(SoC)$.
- `[Tremblay2009]`: fitting battery parameters from datasheet discharge curves.
- `[Krishnan2010]` ch. 2: converter DC link and dynamic braking.

See [References](./references.md).
