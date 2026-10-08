---
title: Thermal model
---

# Thermal model

:::tip In plain words
Current in the copper makes heat ($I^2R$). The heat flows from the coils into the iron, then into the motor's outer body, then into the air. The hotter the copper, the higher its resistance, so it heats even faster. The hotter the magnets, the weaker they get, so you need more current for the same torque. This is why **holding a heavy arm still** can overheat a motor even though nothing moves: all the electrical power becomes heat.
:::

## Assumptions

1. **Three lumped thermal nodes** (each a single temperature):
   - **winding** $T_w$;
   - **stator iron** $T_s$;
   - **outer body** $T_h$: the housing for an inrunner, the rotating can that carries the magnets for an outrunner.
2. Heat flows only along the chain winding → stator → outer body → ambient. Radiation is folded into the outer-body-to-ambient resistance.
3. The ambient temperature $T_a$ is constant (configurable).
4. Heat sources:

   | Source | Goes into node |
   |---|---|
   | copper loss $P_{cu}$ | winding |
   | iron loss $P_{fe}$ | stator |
   | bearing and load-side friction | outer body |
   | gearbox loss | not modelled; it leaves the motor's thermal network |

5. **Magnet temperature** (**approximation**):

   | Topology | $T_{mag}$ | Why |
   |---|---|---|
   | outrunner | $T_h$ | the magnets are bonded to the can |
   | inrunner | $T_s$ | the rotor sits millimetres from the stator across a thin air gap; slightly conservative |

6. Temperatures are stored in kelvin and shown in °C. The equations below use differences, so either scale works.

## Symbols

| Symbol | Meaning | Unit | Code / signal |
|---|---|---|---|
| $T_w, T_s, T_h$ | node temperatures | K | `thermal.t_winding`, `thermal.t_stator`, `thermal.t_housing` |
| $C_w, C_s, C_h$ | heat capacities | J/K | `motor.thermal.c_*` |
| $R_{ws}, R_{sh}, R_{ha}$ | thermal resistances between nodes / to ambient | K/W | `motor.thermal.r_*` |
| $\alpha_{Cu}$ | copper temperature coefficient | 1/K | constant 0.00393 |
| $\alpha_{Br}$ | magnet remanence temperature coefficient | 1/K | `motor.magnetic.alpha_br` (default −0.0012) |
| $T_{w,max}$ | maximum winding temperature | K | `motor.thermal.t_max` |

## Network

### EQ-THERM-01 — Three-node thermal network {/* #eq-therm-01 */}

$$
\begin{aligned}
C_w\,\dot T_w &= P_{cu} - \frac{T_w - T_s}{R_{ws}} \\[2pt]
C_s\,\dot T_s &= P_{fe} + \frac{T_w - T_s}{R_{ws}} - \frac{T_s - T_h}{R_{sh}} \\[2pt]
C_h\,\dot T_h &= P_{fric} + \frac{T_s - T_h}{R_{sh}} - \frac{T_h - T_a}{R_{ha}}
\end{aligned}
$$

$P_{cu}$ is the copper loss of EQ-MOT-13 (evaluated with the temperature-dependent $R$), $P_{fe}$ is EQ-MOT-09, and $P_{fric}$ is the friction loss (EQ-MECH-05) of the motor bearings and the load.

The energy stored in the network ($\sum C_i T_i$) and the heat flow to ambient are reported to the energy balance (EQ-ENER).

### EQ-THERM-02 — Copper resistance vs temperature {/* #eq-therm-02 */}

$$
R(T_w) = R_{ref}\,\big(1 + \alpha_{Cu}\,(T_w - T_{ref})\big),\qquad \alpha_{Cu} = 0.00393\ \mathrm{K^{-1}}
$$

The value is for annealed copper (100 % IACS) at $T_{ref} = 20$ °C, valid from −100 to 200 °C `[CDA-Copper]`. A winding at 100 °C has about 31 % more resistance than at 20 °C.

### EQ-THERM-03 — Magnet strength vs temperature {/* #eq-therm-03 */}

$$
\lambda_m(T_{mag}) = \lambda_{m,ref}\,\big(1 + \alpha_{Br}\,(T_{mag} - T_{ref})\big),\qquad \alpha_{Br} \approx -0.0012\ \mathrm{K^{-1}}
$$

- Sintered NdFeB has a reversible coefficient of −0.12 %/K across common grades (N35 … UH) `[ArnoldNeo]`. Bonded NdFeB and ferrite differ, and the preset sets the value.
- $K_t$, $K_e$ and $K_v$ follow $\lambda_m$ (EQ-CONV-09…11). At 80 °C a motor gives about 7 % less torque per amp than at 20 °C.
- **Irreversible demagnetisation** is not modelled. Instead a **warning** is raised when $T_{mag}$ exceeds the grade's maximum operating temperature (N ≈ 80 °C, H ≈ 120 °C, SH ≈ 150 °C, UH ≈ 180 °C) and when the current is large enough to demagnetise (P10.T06).

### EQ-THERM-04 — Optional speed-dependent cooling {/* #eq-therm-04 */}

A spinning outrunner can fans itself. **Approximation:**

$$
R_{ha}(\omega_m) = \frac{R_{ha,0}}{1 + \kappa\,|\omega_m|}
$$

The default is $\kappa = 0$ (still air, the conservative choice for low-RPM use). An outrunner preset may set a small $\kappa$ (e.g. $0.01\ \mathrm{s/rad}$).

## Ratings derived from the model

### EQ-THERM-05 — Continuous current and torque {/* #eq-therm-05 */}

At steady state ($\dot T = 0$) the heat passes through the chain in series. With $R_{th} = R_{ws} + R_{sh} + R_{ha}$ and $P_{cu} = \tfrac32 R(T_w)\,\hat I^2$ (peak phase current $\hat I$):

$$
T_w - T_a = \tfrac32\,R(T_w)\,\hat I^2\,R_{th} + P_{fe}\,(R_{sh} + R_{ha}) + P_{fric}\,R_{ha}
$$

Setting $T_w = T_{w,max}$ makes $R$ known, so the continuous current has a closed form:

$$
\hat I_{cont} = \sqrt{\frac{T_{w,max} - T_a - P_{fe}(R_{sh}+R_{ha}) - P_{fric}R_{ha}}{\tfrac32\,R_{ref}\big(1 + \alpha_{Cu}(T_{w,max} - T_{ref})\big)\,R_{th}}},
\qquad
T_{cont} = K_t(T_{mag,ss})\,\hat I_{cont}
$$

$T_{mag,ss}$ is the steady-state magnet node temperature at that current. Checked numerically: integrating the network at $\hat I_{cont}$ settles at $T_w = 99.996$ °C for a 100 °C target.

**Thermal runaway.** A steady state exists only if $\tfrac32\,\hat I^2 R_{ref}\,\alpha_{Cu}\,R_{th} < 1$. Above that, the rising resistance makes the heat grow faster than the cooling, and the simulator raises an overheat warning early.

### EQ-THERM-06 — Peak-current time limit {/* #eq-therm-06 */}

The time a current $\hat I > \hat I_{cont}$ can be held, starting from ambient, before $T_w$ reaches $T_{w,max}$ is found by integrating EQ-THERM-01 (with EQ-THERM-02) and bisecting on time (`peak_time()`, P06.T07). A quick lower bound assumes all copper heat stays in the winding:

$$
t_{peak} \gtrsim \frac{C_w\,(T_{w,max} - T_a)}{\tfrac32 R(T_{w,max})\,\hat I^2}
$$

## Parameter values

Gimbal and robot motor datasheets rarely give thermal data. The presets therefore use **estimates**, labelled as such and with the method recorded:

| Quantity | Estimate |
|---|---|
| $C_w$ | $m_{Cu}\,c_{Cu}$, $c_{Cu} = 385$ J/(kg·K) |
| $C_s$ | $m_{Fe}\,c_{Fe}$, $c_{Fe} \approx 460$ J/(kg·K) |
| $C_h$ | $m_{body}\,c$, $c_{Al} \approx 900$ J/(kg·K) |
| $R_{ha}$ | $1/(h\,A)$, natural convection plus radiation $h \approx 10\ldots15$ W/(m²·K), $A$ = outer surface |
| $R_{ws}$ | 0.2–2 K/W (thin slot liner) |
| $R_{sh}$ | 0.1–1 K/W (press fit / glued) |
| $T_{w,max}$ | 100 °C default (conservative for hobby magnet-wire and N-grade magnets) |

Example, 8010-class outrunner: $A \approx 0.02$ m² gives $R_{ha} \approx 4$ K/W and $R_{th} \approx 5$ K/W. That allows about 15 W of continuous copper loss at 25 °C ambient.

## Dimensional check

| Equation | Check |
|---|---|
| EQ-THERM-01 | J/K · K/s = W; W − K/(K/W) = W ✓ |
| EQ-THERM-05 | $\sqrt{\mathrm{K}/(\Omega\cdot\mathrm{K/W})} = \sqrt{\mathrm{W}/\Omega} = \mathrm A$ ✓ |

## References

- `[CDA-Copper]`: temperature coefficient of resistance of annealed copper.
- `[ArnoldNeo]`: reversible temperature coefficients of sintered NdFeB.
- `[HendershotMiller2010]`: thermal paths and ratings in PM machines.

See [References](./references.md).
