---
title: Parameter estimation
---

# Parameter estimation

:::tip In plain words
Motor datasheets for gimbal and robot motors often list only Kv, resistance and weight. To simulate the motor you also need its inductance, how heavy its rotor feels (inertia) and how fast it heats up. These rules give **reasonable guesses** from the size and weight. Every guessed value is marked `estimated`, and a measured or datasheet value always replaces it.
:::

These rules are used by the motor presets and by the datasheet wizard. Each estimate is reported with a **confidence**: *medium* when all its inputs come from the datasheet, *low* when one of them is itself estimated. Every rule here is a rough engineering guess, so none is rated *high*.

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

The rotating can and magnets carry roughly 40 % of the mass at roughly 90 % of the outer radius ($0.4 \cdot 0.9^2 \approx 0.32$); the coefficient is rounded up to 0.35 to include the hub and shaft. Checked against CubeMars RO100 (0.71 kg, Φ108 mm): the rule gives 7200 g·cm², and the listings give 5290–8700 g·cm².

**Inrunner.** The rotor is a solid cylinder of about 30 % of the mass with a diameter of about 0.55 $D$: $J \approx 0.3\,m \cdot \tfrac12 (0.55\,D/2)^2 \approx 0.045\,m\,(D/2)^2$. This is an order-of-magnitude guess (confidence low).

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

### EQ-EST-04 — Mass from the outer envelope {/* #eq-est-04 */}

$$
m \approx \rho_{eff}\,\frac{\pi}{4} D^2 H,\qquad \rho_{eff} \approx 2200\ \mathrm{kg/m^3}
$$

$\rho_{eff}$ is an average over copper, iron, magnets, aluminium and air. Checks with the EQ-EST-05 envelope: 2804 class 40 g (listed 39 g); 10015 class 650 g (630 g); 4108 class 95 g (124 g); 8108 class 334 g (238–250 g; thin flat motors weigh less).

### EQ-EST-05 — Envelope from the size class {/* #eq-est-05 */}

For a size class `DDHH` (stator diameter $d$ and stack height $h$ in mm):

$$
D \approx 1.07\,d + 5\ \mathrm{mm},\qquad H \approx h + 15\ \mathrm{mm}
$$

$D$ adds the magnets, back iron and can around the stator. $H$ adds end turns, bearings and the base. Fitted to the presets: 2804 → 35 mm (listed 35), 4108 → 49 mm (47), 8108 → 92 mm (88), 10015 → 112 mm (106).

## Datasheet wizard conventions

The wizard asks only the questions whose answer changes the model:

| Question | Effect |
|---|---|
| Pole number = magnets or pole pairs | $p$; asked only if both readings give a plausible winding ($q = N_s/(3\cdot 2p) \ge 1/4$) |
| Kv: no-load DC (hobby) or per V RMS line-to-line | RMS-based Kv is divided by $\sqrt2$ to get the LL-peak Kv (EQ-CONV-11) |
| Kt per peak or RMS amp | RMS-based Kt is divided by $\sqrt2$ |
| R, L measured line-to-line or per phase | star-equivalent phase value = LL / 2 (star and delta); a delta phase winding / 3 |
| Star or delta | stored for display; the model always uses star-equivalent values |

## Dimensional check

| Equation | Check |
|---|---|
| EQ-EST-01 | kg · m² ✓ |
| EQ-EST-02 | s · Ω = H ✓ |
| EQ-EST-03 | kg · J/(kg·K) = J/K; 1/(W/(m²K) · m²) = K/W ✓ |
| EQ-EST-04 | kg/m³ · m³ = kg ✓ |
| EQ-EST-05 | mm ✓ |
