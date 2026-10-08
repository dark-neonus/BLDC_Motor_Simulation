---
title: Mechanical model
---

# Mechanical model

:::tip In plain words
Everything that spins: the rotor, an optional gearbox, and the load (an arm with a weight, or a brake or fan-like load). This page covers how torque turns into acceleration, how gearboxes multiply torque and lose some of it, how gear "play" (backlash) makes the load lag, and how friction can hold a shaft completely still until you push hard enough.
:::

Conventions: positive rotation is counter-clockwise from the output-shaft end. The load arm angle is $0$ pointing straight down ([Conventions](./conventions.md#eq-conv-05)).

## Assumptions

1. Rigid bodies on fixed axes; shaft torsion is ignored except where the gearbox contact (backlash) model is enabled.
2. The gearbox has a fixed ratio $N \ge 1$ (reduction). Load-side angle $\theta_L = \theta_m / N$ when rigid.
3. Gravity acts downward in the plane of the arm ($g = 9.80665\ \mathrm{m/s^2}$).
4. Friction is lumped per body (motor bearings, gearbox, load bearing).

## Symbols

| Symbol | Meaning | Unit | Code / signal |
|---|---|---|---|
| $J_m$ | rotor inertia (motor side) | kg·m² | `motor.mechanical.j_rotor` |
| $J_{gi}, J_{go}$ | gearbox inertia at input / output | kg·m² | `gearbox.j_in`, `gearbox.j_out` |
| $N$, $\eta$ | gear ratio, efficiency | –, – | `gearbox.ratio`, `gearbox.efficiency` |
| $b_{bl}$ | backlash (total, load side) | rad | `gearbox.backlash` |
| $k_c, d_c$ | gear contact stiffness, damping | N·m/rad, N·m·s/rad | `gearbox.stiffness`, `gearbox.damping` |
| $T_s, T_c, b$ | static, Coulomb, viscous friction | N·m, N·m, N·m·s/rad | `motor.mechanical.friction.*` |
| $m_a, L_a$ | arm mass, length | kg, m | `load.arm.mass`, `load.arm.length` |
| $m, d$ | point mass and its distance from the shaft | kg, m | `load.mass`, `load.distance` |
| $\theta_L, \omega_L$ | load angle and speed | rad, rad/s | `load.theta`, `load.omega` |

## Rotor inertia

### EQ-MECH-01 — Rotor inertia from geometry {/* #eq-mech-01 */}

$$
J_{inrunner} \approx \tfrac12\, m_r\, r_o^2,\qquad
J_{outrunner} \approx \tfrac12\, m_r\,(r_o^2 + r_i^2)
$$

- Inrunner: solid cylinder of mass $m_r$ and radius $r_o$.
- Outrunner: thick shell (rotor can plus magnets) between $r_i$ and $r_o$.

The geometry-based estimates (mass from volumes and densities) are in [estimation](./estimation.md) (P04.T12). Datasheet values override them.

## Rigid drivetrain

### EQ-MECH-02 — Motor-side equation of motion (rigid gearbox) {/* #eq-mech-02 */}

All inertias are reflected to the motor side:

$$
J_{eq}\,\dot\omega_m = T_e + T_{cog} + T_{fe} + T_{f,m} + T_{gl} + \frac{T_{L}}{N},
\qquad
J_{eq} = J_m + J_{gi} + \frac{J_{go} + J_{load}}{N^2},\qquad \dot\theta_m = \omega_m
$$

- $T_L$ is the sum of external load-side torques (gravity, loads, disturbance, load friction).
- $J_{load}$ comes from EQ-MECH-05.
- "Bypass gearbox" means $N = 1$, $\eta = 1$, $J_{gi} = J_{go} = 0$.

### EQ-MECH-03 — Gearbox efficiency loss {/* #eq-mech-03 */}

Meshing loss is proportional to the torque transmitted through the gearbox, and always opposes motion:

$$
T_{gl} = -(1-\eta)\,\big|\tau_{in}\big|\,\tanh\!\frac{\omega_m}{\omega_\epsilon}
$$

Here $\tau_{in}$ is the input-side transmitted torque from the lossless solution of EQ-MECH-02:

$$
\tau_{in} = \frac{1}{N}\Big(\frac{J_{go} + J_{load}}{N}\,\dot\omega_m^{(0)} - T_L\Big)
$$

where $\dot\omega_m^{(0)}$ is the acceleration computed with $T_{gl} = 0$.

- **Approximation:** using the lossless $\tau_{in}$ (one correction pass, no iteration) keeps the model explicit. $T_{gl}\,\omega_m \le 0$ always, so energy accounting stays exact: the loss power reported is $-T_{gl}\,\omega_m$.
- Forward drive and back-driving both lose a fraction $1 - \eta$ of the transmitted power, which matches the "same efficiency both ways" datasheet value. Self-locking gearboxes (worm) are out of scope.
- $\omega_\epsilon \approx 10^{-3}$ rad/s smooths the sign change.

## Backlash

### EQ-MECH-04 — Two-inertia backlash model {/* #eq-mech-04 */}

When `gearbox.backlash > 0`, the load gets its own states $\theta_L$ and $\omega_L$, coupled by a dead-zone contact. Define the load-side deflection $\delta = \theta_m/N - \theta_L$:

$$
\tau_c =
\begin{cases}
0, & |\delta| \le b_{bl}/2 \\[4pt]
\max\!\Big(0,\; \mathrm{sgn}(\delta)\big[k_c(|\delta| - b_{bl}/2) + d_c\,\mathrm{sgn}(\delta)\,\dot\delta\big]\Big)\,\mathrm{sgn}(\delta), & |\delta| > b_{bl}/2
\end{cases}
$$

$$
(J_m + J_{gi})\,\dot\omega_m = T_{motor\ side} - \frac{\tau_c}{N} + T_{gl},\qquad
(J_{go} + J_{load})\,\dot\omega_L = \tau_c + T_L
$$

- The $\max(0,\cdot)$ makes the contact **push-only**: damping can never make the teeth "stick" together.
- Entering and leaving contact ($|\delta| = b_{bl}/2$) are **state events** (EQ-NUM).
- Stored energy while in contact: $\tfrac12 k_c(|\delta| - b_{bl}/2)^2$.
- Typical values: small planetary gearboxes have backlash of 10–30 arcmin and torsional stiffness of 1–10 N·m/arcmin (≈ 3.4·10³–3.4·10⁴ N·m/rad).
- Damping: $d_c = 2\zeta\sqrt{k_c J_{red}}$ with $\zeta \approx 0.05\ldots0.2$ and $J_{red}$ the reduced inertia of the two sides.
- Numerics: the contact frequency $\sqrt{k_c/J_{red}}$ limits the time step (EQ-NUM).
- **Switching:** rigid vs backlash is chosen when the scene is built, not live, because the number of states changes.

## Friction

### EQ-MECH-05 — Stick–slip friction (Karnopp-type hybrid) {/* #eq-mech-05 */}

For each body with speed $\omega$ and net applied torque $T_a$ (all torques except its own friction):

**Slipping:**

$$
T_f = -\,\mathrm{sgn}(\omega)\,T_c - b\,\omega
$$

**Sticking:** the body is held exactly still ($\omega \equiv 0$, $\theta$ constant) as long as $|T_a| \le T_s$. In that state $T_f = -T_a$.

**Transitions** (state events):

- **slip → stick:** $\omega$ crosses zero while $|T_a| \le T_s$. The remaining kinetic energy $\tfrac12 J\omega^2$ (tiny, at most one step's worth) is booked as friction loss.
- **stick → slip:** $|T_a| > T_s$. Motion starts in the direction of $T_a$.

Why a hybrid stick state instead of Karnopp's original velocity band `[Karnopp1985]`? It keeps the exact "holds still" behaviour that holding-torque lessons need, and it avoids the band's slow creep. Verified numerically: a torque of $0.98\,T_s$ leaves $\theta$ exactly unchanged, a pulse above $T_s$ moves the shaft and it sticks again, and the energy balance closes to integrator accuracy.

Typical values: motor bearings $T_c \approx 0.1$–1 % of rated torque; gearboxes have much more ($T_c$ up to a few % of output torque).

## Loads

### EQ-MECH-06 — Arm with a point mass under gravity {/* #eq-mech-06 */}

A uniform rod (mass $m_a$, length $L_a$, pivoted at one end) carries a point mass $m$ at distance $d$. With arm angle $\varphi = \theta_L + \varphi_{mount}$ measured from straight down:

$$
M_{eff} = \frac{m_a L_a}{2} + m\,d,\qquad
T_g = -\,g\,M_{eff}\sin\varphi,\qquad
U_g = g\,M_{eff}\,(1 - \cos\varphi),\qquad
J_{load} = \frac{m_a L_a^2}{3} + m\,d^2
$$

Small-angle period of the **unpowered** system (rigid gearbox, no friction), validation case V-MECH-pendulum:

$$
T_0 = 2\pi\sqrt{\frac{J_{load} + J_{go} + N^2 (J_m + J_{gi})}{g\,M_{eff}}}
$$

Checked numerically: for $m_a = 0.1$ kg, $L_a = 0.2$ m, $m = 0.5$ kg, $d = 0.15$ m, $J_m = 2.5\cdot10^{-4}$ kg·m² and $N = 1$, theory gives 0.77948 s and the simulation 0.77950 s at 0.01 rad amplitude.

### EQ-MECH-07 — Simple load torques {/* #eq-mech-07 */}

| Load | Torque on the load shaft | Energy |
|---|---|---|
| `constant` (e.g. a hanging weight on a drum) | $T = T_0$ (fixed direction) | external work $\int T_0\,\omega_L\,dt$ |
| `brake` (opposes motion) | $T = -T_0\tanh(\omega_L/\omega_\epsilon)$ | dissipated |
| `viscous` (fan/pump-like at low speed) | $T = -c\,\omega_L$ | dissipated |

### EQ-MECH-08 — Disturbance torque (mouse push / flick) {/* #eq-mech-08 */}

`load.disturbance` applies a torque pulse $T_{dist}$ for a duration $\Delta t$ on the load shaft. An **impulse** $H$ (N·m·s, from a flick gesture) becomes the pulse $T_{dist} = H/\Delta t$ with $\Delta t = \max(1\ \mathrm{ms},\ 5\,dt_{max})$.

- On a free rigid system: $\Delta\omega_L = H / J_{tot,L}$ (validation case V-MECH-impulse).
- Its work $\int T_{dist}\,\omega_L\,dt$ is booked as external energy.

## Live parameter changes

### EQ-MECH-09 — Changing mass, distance or arm length while running {/* #eq-mech-09 */}

Changing these changes $J_{load}$ and $M_{eff}$ instantly.

- **Default `keep_speed`** (D-007): $\omega_L$ stays the same, as if the new mass were already moving with the arm.
- **Option `conserve_momentum`:** $\omega_L^{+} = \omega_L^{-}\,J^{-}/J^{+}$ (the mass is added at rest relative to the ground).

In both cases the jump in stored energy is booked as external energy so the balance stays exact:

$$
\Delta E = \tfrac12 J^{+}(\omega_L^{+})^2 - \tfrac12 J^{-}(\omega_L^{-})^2 + \Delta U_g
$$

## Dimensional check

| Equation | Check |
|---|---|
| EQ-MECH-02 | kg·m²·rad/s² = N·m ✓ |
| EQ-MECH-04 | N·m/rad · rad = N·m ✓; N·m·s/rad · rad/s = N·m ✓ |
| EQ-MECH-06 | m/s² · kg·m = N·m ✓; $U_g$: N·m = J ✓ |
| EQ-MECH-08 | N·m·s / s = N·m ✓ |

## References

- `[Karnopp1985]`: stick–slip friction simulation.
- `[Krishnan2010]` ch. 1: load torque characteristics and drive dynamics.
- Gearbox stiffness and backlash ranges: typical datasheets for small planetary and cycloidal reducers (the values used in presets are cited in `presets/gearboxes/_sources.md`, P04.T10).

See [References](./references.md).
