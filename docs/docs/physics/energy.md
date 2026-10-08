---
title: Energy balance
---

# Energy balance

:::tip In plain words
Energy can't appear from nowhere. Everything the supply delivers ends up as motion, stored energy (spinning, height, magnetic field, charged capacitor, heat) or losses. The simulator adds all of these up continuously. If the total doesn't match, something in the model is wrong, so the **energy indicator** in the status bar is a live correctness check.
:::

## EQ-ENER-01 — Power terms {/* #eq-ener-01 */}

Each block reports its terms, and the engine integrates each one as an extra state (P03.T16):

| Term | Kind | Source |
|---|---|---|
| source power $V_{bus}\,i_{src}$, **PSU / ideal source only** | input | EQ-SUP-02/03 |
| battery: **no input term**; its energy is the stored $E_{chem}$ (below) | – | EQ-SUP-04 |
| battery internal losses $R_0 i^2 + v_1^2/R_1$ | loss | EQ-SUP-04 |
| brake chopper $V_{bus}^2/R_b$ | loss | EQ-SUP-05 |
| inverter conduction + diode + switching | loss | EQ-INV-09 |
| copper $R\sum i_x^2$ | loss | EQ-MOT-13 |
| iron $-T_{fe}\,\omega_m$ | loss | EQ-MOT-09 |
| friction (all bodies, incl. stick events) | loss | EQ-MECH-05 |
| gearbox meshing $-T_{gl}\,\omega_m$ | loss | EQ-MECH-03 |
| load dissipation (brake, viscous) | loss | EQ-MECH-07 |
| external work (constant load, disturbance, live changes) | external | EQ-MECH-07/08/09 |
| heat to ambient | (thermal subsystem, tracked separately) | EQ-THERM-01 |

## EQ-ENER-02 — Stored energies {/* #eq-ener-02 */}

$$
E_{st} = \tfrac12 C_{bus}V_{bus}^2 + W_{mag} + \sum \tfrac12 J\omega^2 + U_{cog} + U_g + \tfrac12 k_c(|\delta| - b/2)_+^2 + \tfrac12 C_1 v_1^2 + E_{chem}(SoC)
$$

$W_{mag}$ is from EQ-MOT-10 (it reduces to $\tfrac12 L_s\sum i^2$ when linear). With an ideal source, $E_{chem}$ and $\tfrac12 C_{bus}V_{bus}^2$ are omitted.

**Battery bookkeeping.** For a battery the chemical energy $E_{chem}(SoC)$, $\tfrac12 C_1 v_1^2$ and the $R_0$/$R_1$ losses are inside the system, so $V_{bus}i_{src}$ is an internal flow and is **not** booked as input. Otherwise it would be counted twice.

**Temperature-dependent magnets.** A change of $\lambda_m(T)$ at fixed $\psi$ moves a small amount of energy that no term books. It is neglected: thermal time constants are seconds to minutes, and the effect is far below $r_{tol}$.

## EQ-ENER-03 — Residual {/* #eq-ener-03 */}

$$
r(t) = \frac{E_{in} + E_{ext} - E_{loss} - \big(E_{st}(t) - E_{st}(0)\big)}{\max\big(E_{thr}(t),\ E_{floor}\big)}
$$

- $E_{thr} = \int \sum |P_k|\,dt$ is the total energy throughput.
- $E_{floor} = 10^{-6}$ J avoids dividing by zero at start.
- Signals: `energy.residual`, and `energy.ok` = $|r| < r_{tol}$.

**Tolerance.** $r_{tol} = 10^{-3}$ for the Ideal and Standard tiers: RK4 with the step rules of EQ-NUM gives errors far below this. The Detailed tier with switching events uses $r_{tol} = 5\cdot10^{-3}$, because state-event localisation and the switching-loss averaging (EQ-INV-09) add bounded error.

**Purely electrical check.** For the motor alone, $P_{elec} = P_{cu} + \dot W_{mag} + T_e\omega_m$ holds exactly in the continuous model (EQ-MOT-13). Its numerical residual scales with the integrator order, which was verified for the saturation model.

## References

See [Motor](./motor.md), [Inverter](./inverter.md), [Supply](./supply.md), [Mechanical](./mechanical.md), [Thermal](./thermal.md).
