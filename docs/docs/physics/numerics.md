---
title: Numerical methods
---

# Numerical methods

:::tip In plain words
The computer can't solve the equations continuously, so it takes many tiny time steps. This page says how big those steps are, how the controllers (which run at fixed rates like 20 kHz) fit between them, how sudden events (a diode turning on, friction sticking) are located exactly, and how the run stays repeatable.
:::

## EQ-NUM-01 — Time base {/* #eq-num-01 */}

- Sim time is an integer number of **nanoseconds** (D-004).
- Rates become integer periods by rounding (D-010), and the actual frequency is reported.
- Between events the integrator works in `f64` seconds.

## EQ-NUM-02 — Hybrid scheme {/* #eq-num-02 */}

```
loop:
  t_next = min(next time event, target)
  integrate the continuous plant t → t_next in ≤ dt_max substeps (EQ-NUM-03),
      stopping early at a located state event (EQ-NUM-05)
  fire all events at t_next in priority order:
      sensors → estimators → controllers → modulator → inverter
  apply queued commands (parameter changes, faults)
```

- Discrete outputs (duties, switch states, setpoints) are held constant between events (zero-order hold).
- Each RK stage first evaluates the algebraic outputs of every module from the full state, in a fixed order, and then the derivatives (P03 design note). Modules are therefore coupled consistently within a step.

## EQ-NUM-03 — RK4 {/* #eq-num-03 */}

Classic 4th order `[Hairer1993]`:

$$
x_{n+1} = x_n + \tfrac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)
$$

- Global error is $O(h^4)$, verified by halving the step: the error ratio ≈ 16.
- **Stability** for $\dot x = -x/\tau$: $|h/\tau| \le 2.785$.
- **Accuracy** needs $h \le \tau/20$ (electrical $\tau_e = L/R$; mechanical and contact modes $\tau = 1/\omega$).

## EQ-NUM-04 — Step-size rules {/* #eq-num-04 */}

| Tier / mode | $dt_{max}$ |
|---|---|
| Ideal / Standard, averaged | $\min(\tau_e/20,\ T_{ctrl}/2,\ 1/(20\,\omega_{max}))$ |
| switching inverter | additionally $\le T_{pwm}/200$; edges are events |
| backlash contact | additionally $\le 1/(20\sqrt{k_c/J_{red}})$ |

$\omega_{max}$ is the highest mechanical or electrical natural frequency present. These limits are computed by `FidelityConfig` (P03.T06), and the user can tighten them.

**Optional stiff electrical step.** For linear RL substates with constant input over the step, the exact exponential update is

$$
i_{n+1} = i_\infty + (i_n - i_\infty)\,e^{-hR/L}
$$

It is stable for any $h$ (Ideal/Standard tiers only; P03.T03).

## EQ-NUM-05 — State events {/* #eq-num-05 */}

Modules expose zero-crossing functions $g(x)$:

| Event | Zero-crossing function |
|---|---|
| diode turn-on/off | EQ-MOT-12, EQ-INV-01 |
| friction stick/slip | EQ-MECH-05 |
| backlash contact | $\lvert\delta\rvert - b/2$ |
| PSU mode | EQ-SUP-03 |
| chopper hysteresis | EQ-SUP-05 |

- A sign change within a substep is located by bisection to 1 ns, the step is redone up to the event, the module handler runs, and integration continues.
- **Zeno guard:** more than 100 events per simulated µs from one module pauses the run with a diagnostic.

## EQ-NUM-06 — Energy integration {/* #eq-num-06 */}

Power terms are integrated as extra states with the same integrator (EQ-ENER-01). Their accuracy therefore matches the physical states.

## EQ-NUM-07 — Reference solutions {/* #eq-num-07 */}

Cross-checks use **diffsol** (BDF/ESDIRK, `rtol = 1e-10`) in Rust and SciPy `solve_ivp` (`DOP853` or `Radau`, `rtol = 1e-9`, `atol = 1e-12`) in Python. Both integrate between the same discrete events.

## EQ-NUM-08 — Determinism {/* #eq-num-08 */}

Same scene + scenario + seed gives bit-identical output on the same machine and build. This requires:

- all randomness from the seeded RNG service, with one stream per block;
- no hash-map iteration in the hot path;
- no wall-clock time inside the simulation;
- a fixed event order (EQ-NUM-02).

## References

`[Hairer1993]`. See [References](./references.md).
