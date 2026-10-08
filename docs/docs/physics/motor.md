---
title: Motor (electromagnetic model)
---

# Motor — electromagnetic model

:::tip In plain words
A brushless motor is three coils (phases) and a ring of magnets. When the magnets move past the coils they induce a voltage, the **back-EMF**. When current flows in the coils the motor produces **torque**. This page is the exact set of equations the simulator uses for that: how voltages turn into currents, how currents turn into torque, plus the side effects (cogging, iron losses, saturation, a disconnected phase).
:::

Conventions (frames, signs, $\lambda_m$, $K_t$, $K_e$, $K_v$) are defined in [Conventions](./conventions.md). Implementation decisions: D-002 (one stationary-frame model) and D-013 (flux-linkage states).

## Assumptions

1. **Lumped model:** three identical phases, star-connected with an **isolated neutral**. Delta windings use their star equivalent (EQ-CONV-14).
2. The magnets are a flux source $\lambda_m\,\Phi(\theta_e)$ that does **not** depend on current, except through the optional cross-saturation term (EQ-MOT-10). Temperature dependence of $\lambda_m$ and $R$ is in [Thermal](./thermal.md).
3. Magnetic behaviour is described in the rotor frame by flux functions $\psi_d(i_d,i_q)$ and $\psi_q(i_d,i_q)$ derived from one **co-energy** function. This guarantees energy conservation.
4. Not modelled: eddy currents in the magnets, end-winding effects, inter-turn faults, phase-unbalance from manufacturing, and skin effect at PWM frequency.

## Symbols

| Symbol | Meaning | Unit | Code / signal |
|---|---|---|---|
| $\psi_\alpha,\psi_\beta$ | stator flux linkage, stationary frame (**the integrated states**) | Wb | `motor.psi_alpha`, `motor.psi_beta` |
| $v_{aT},v_{bT},v_{cT}$ | terminal voltages w.r.t. the DC− rail (from the inverter) | V | `inverter.v_a` … |
| $v_n$ | star-point (neutral) voltage w.r.t. DC− | V | `motor.v_n` |
| $R$ | phase resistance (star-equivalent, temperature-corrected) | Ω | `motor.electrical.r_phase` |
| $L_d, L_q$ | unsaturated d/q inductance ($L_d = L_q = L_s$ for non-salient) | H | `motor.electrical.l_d`, `…l_q` |
| $\lambda_m$ | magnet flux linkage, fundamental, peak per phase | Wb | `motor.electrical.lambda_m` |
| $\Phi(\theta)$ | normalised magnet-flux shape, fundamental amplitude 1 | – | `motor.magnetic.emf_shape` |
| $k(\theta) = d\Phi/d\theta$ | normalised back-EMF shape | – | |
| $T_e$ | electromagnetic torque | N·m | `motor.torque_em` |
| $N_c$ | cogging periods per mechanical revolution | – | derived |

Typical values for the low-RPM motors this simulator focuses on (gimbal and robot-actuator class, 2804…12020): $p = 7\ldots21$, $R = 0.05\ldots20\ \Omega$, $L_s = 20\ \mu\mathrm{H}\ldots10\ \mathrm{mH}$, $\lambda_m = 2\ldots50\ \mathrm{mWb}$, electrical time constant $L_s/R \approx 0.1\ldots2\ \mathrm{ms}$.

## Electrical dynamics

### EQ-MOT-01 — Flux-linkage law (stationary frame) {/* #eq-mot-01 */}

$$
\frac{d}{dt}\begin{bmatrix}\psi_\alpha\\\psi_\beta\end{bmatrix}
= \begin{bmatrix}v_\alpha\\v_\beta\end{bmatrix} - R\begin{bmatrix}i_\alpha\\i_\beta\end{bmatrix},
\qquad
\begin{bmatrix}v_\alpha\\v_\beta\end{bmatrix} = \text{Clarke}(v_{aT}, v_{bT}, v_{cT})
$$

The common-mode part of the terminal voltages drops out of the Clarke transform (EQ-CONV-01), so the unknown neutral voltage is not needed to compute currents. This is exactly why an isolated-neutral machine has only **two** independent electrical states.

### EQ-MOT-02 — Flux ↔ current relation {/* #eq-mot-02 */}

The stator flux is the rotor-frame flux $\psi_{dq}(i_d,i_q)$ rotated into the stationary frame, plus the magnet flux **harmonics** (everything except the fundamental):

$$
\psi_{\alpha\beta} = \mathbf R(\theta_e)\begin{bmatrix}\psi_d(i_d,i_q)\\\psi_q(i_d,i_q)\end{bmatrix} + \lambda_m\,\Phi_{h,\alpha\beta}(\theta_e),
\qquad
\mathbf R(\theta) = \begin{bmatrix}\cos\theta & -\sin\theta\\ \sin\theta & \cos\theta\end{bmatrix}
$$

$\Phi_{h,\alpha\beta}$ is the Clarke transform of the per-phase harmonic flux $\Phi(\theta_e - x\tfrac{2\pi}{3}) - \cos(\theta_e - x\tfrac{2\pi}{3})$ for $x = 0,1,2$.

**Linear magnetics** (default; Ideal and Standard tiers):

$$
\psi_d = L_d i_d + \lambda_m,\qquad \psi_q = L_q i_q
$$

To get currents from the states:

1. subtract the harmonic flux;
2. rotate by $-\theta_e$ (Park);
3. invert $\psi_{dq}(i)$: closed form when linear, $i_d = (\psi_d - \lambda_m)/L_d$ and $i_q = \psi_q/L_q$; Newton's method when saturating (EQ-MOT-10);
4. rotate back by $+\theta_e$.

### EQ-MOT-03 — Back-EMF shape {/* #eq-mot-03 */}

The magnet flux in phase $x \in \{a,b,c\} = \{0,1,2\}$ is $\lambda_m\,\Phi(\theta_e - x\tfrac{2\pi}{3})$. Its time derivative is the back-EMF:

$$
e_x = \omega_e\,\lambda_m\,k\!\left(\theta_e - x\tfrac{2\pi}{3}\right),\qquad k(\theta) = \frac{d\Phi}{d\theta}
$$

**$k$ is always normalised so its fundamental is $-\sin\theta$.** That keeps $\lambda_m$, $K_t$, $K_e$ and $K_v$ referring to the fundamental, so they mean the same thing for every shape. Available shapes (`motor.magnetic.emf_shape`):

| Shape | $k(\theta)$ | Notes |
|---|---|---|
| `sinusoidal` | $-\sin\theta$ | PMSM. Default for gimbal motors. |
| `trapezoidal(w)` | $-\mathrm{trap}_w(\theta)\,/\,b_1(w)$ | BLDC. $\mathrm{trap}_w$ is a unit-amplitude odd trapezoid with flat-top width $w$ per half-period. Its fundamental is $b_1(w) = \dfrac{4}{\pi}\dfrac{\sin r}{r}$ with $r = \dfrac{\pi - w}{2}$. Classic $w = 120°$: $b_1 = 12/\pi^2 = 1.2159$, so the normalised flat top is $0.8225$. |
| `harmonics` | $-\left(\sin\theta + \sum_n b_n\sin(n\theta + \varphi_n)\right)$ | From measured back-EMF. Odd $n$. Triplen harmonics ($n = 3, 9, \ldots$) change the neutral voltage but produce **no** current or torque with an isolated neutral. |

$\Phi$ is the integral of $k$ with zero mean. For `sinusoidal`, $\Phi = \cos\theta$, which matches EQ-CONV-06.

### EQ-MOT-04 — Neutral and phase voltages {/* #eq-mot-04 */}

These are needed only for display and for the floating-terminal logic. They are not needed for the currents (EQ-MOT-01). For linear, non-salient magnetics:

$$
v_n = \frac13\Big(\sum_x v_{xT} - \sum_x e_x\Big),\qquad
v_x = v_{xT} - v_n = R\,i_x + L_s\frac{di_x}{dt} + e_x
$$

With saliency or saturation, the simulator uses the same expression with $L_s = (L_d + L_q)/2$. **Approximation:** this affects only the displayed neutral voltage and the floating-terminal estimate, never the currents.

### EQ-MOT-05 — Linear non-salient special case {/* #eq-mot-05 */}

With $L_d = L_q = L_s$ and no saturation, EQ-MOT-01/02 reduce to the textbook per-phase circuit:

$$
L_s\,\frac{d i_{\alpha\beta}}{dt} = v_{\alpha\beta} - R\,i_{\alpha\beta} - e_{\alpha\beta},
\qquad i_{\alpha\beta} = \frac{\psi_{\alpha\beta} - \lambda_m\Phi_{\alpha\beta}(\theta_e)}{L_s}
$$

### EQ-MOT-06 — Equivalence with the dq model {/* #eq-mot-06 */}

For a sinusoidal shape and linear magnetics, differentiate $\psi_{dq} = \mathbf R(-\theta_e)\,\psi_{\alpha\beta}$ using EQ-MOT-01 and $\dot\theta_e = \omega_e$. This gives exactly the classic rotor-frame model:

$$
L_d\frac{di_d}{dt} = v_d - R i_d + \omega_e L_q i_q,\qquad
L_q\frac{di_q}{dt} = v_q - R i_q - \omega_e L_d i_d - \omega_e\lambda_m
$$

This identity is validation case V-MOT-equivalence (P05.T03): the stationary-frame model and the dq reference must agree to integrator accuracy.

## Torque

### EQ-MOT-07 — Electromagnetic torque {/* #eq-mot-07 */}

$$
T_e = \underbrace{\tfrac32\,p\,\big(\psi_d\,i_q - \psi_q\,i_d\big)}_{\text{fundamental + reluctance + saturation}}
\;+\;\underbrace{p\,\lambda_m\sum_{x} k_h\!\left(\theta_e - x\tfrac{2\pi}{3}\right) i_x}_{\text{back-EMF harmonics}},
\qquad k_h(\theta) = k(\theta) + \sin\theta
$$

- **Linear case:** the first term is $\tfrac32 p(\lambda_m i_q + (L_d - L_q)i_d i_q)$ (EQ-CONV-08).
- **Linear, non-salient:** the whole expression equals the power-balance form $T_e = \sum_x e_x i_x/\omega_m = p\lambda_m\sum_x k(\theta_e - x\tfrac{2\pi}3)\,i_x$. That form is singular-free at $\omega_m = 0$ because $\omega_e/\omega_m = p$ cancels analytically.
- This sum is what gives **six-step torque ripple**. For a sinusoidal machine driven with 120° block currents the ripple is $1 - \cos 30° \approx 13.4\,\%$. For an ideal trapezoidal machine ($w = 120°$) it is ideally zero apart from commutation transients.

## Cogging

### EQ-MOT-08 — Cogging torque {/* #eq-mot-08 */}

Cogging is magnet attraction to the stator teeth. It exists with zero current, and it is **conservative**: it can be written as a potential energy.

$$
T_{cog} = \sum_{k=1}^{K} A_k \sin(k N_c \theta_m + \varphi_k),\qquad
U_{cog} = \sum_{k=1}^{K} \frac{A_k}{k N_c}\cos(k N_c\theta_m + \varphi_k),\qquad
T_{cog} = -\frac{dU_{cog}}{d\theta_m}
$$

- $N_c = \mathrm{LCM}(Q, 2p)$ cogging periods per revolution ($Q$ = slots). For example, 12N14P gives $N_c = 84$.
- Typical first-harmonic amplitude $A_1$: 0.5–5 % of rated torque for slotted gimbal motors `[HendershotMiller2010]`. The presets use estimates flagged as such.
- Cogging is applied to the rotor, together with the mechanical torques (see [Mechanical](./mechanical.md)). $U_{cog}$ is reported as stored energy (EQ-ENER).

## Iron losses

### EQ-MOT-09 — Iron loss as a drag torque {/* #eq-mot-09 */}

**Approximation** (Steinmetz-type, at nominal flux):

$$
P_{fe} = k_h\,|\omega_e| + k_e\,\omega_e^2,\qquad
T_{fe} = -\left(k_h\,p\,\tanh\!\frac{\omega_m}{\omega_\epsilon} + k_e\,p^2\,\omega_m\right)
$$

- Hysteresis loss grows ∝ $|\omega_e|$ and eddy-current loss ∝ $\omega_e^2$ (both at nominal flux). $\tanh(\omega_m/\omega_\epsilon)$ with $\omega_\epsilon \approx 0.01$ rad/s replaces $\mathrm{sign}(\omega_m)$, so the torque is smooth and still always dissipative ($T_{fe}\,\omega_m \le 0$).
- The loss power reported to the energy balance is exactly $-T_{fe}\,\omega_m$. Its heat goes to the stator node (see [Thermal](./thermal.md)).
- $k_h$ and $k_e$ are fitted to the no-load spin-down or no-load current; presets mark them as estimated.

## Magnetic saturation (Detailed tier)

### EQ-MOT-10 — Co-energy saturation model {/* #eq-mot-10 */}

Saturation is defined by one rotor-frame co-energy function, so the model is **energy-consistent by construction**:

$$
W'(i_d,i_q) = \lambda_m\, i_d\,\big(1 - h(i_q)\big) + \tfrac12 L_d i_d^2 + Q(i_q)
$$

$$
h(i_q) = c\,\frac{(i_q/i_k)^2}{1 + (i_q/i_k)^2},\qquad
Q(i_q) = \tfrac12 L_\infty i_q^2 + (L_{q} - L_\infty)\, i_k^2 \ln\cosh\frac{i_q}{i_k}
$$

$$
\psi_d = \frac{\partial W'}{\partial i_d} = \lambda_m\big(1 - h(i_q)\big) + L_d i_d,\qquad
\psi_q = \frac{\partial W'}{\partial i_q} = L_\infty i_q + (L_q - L_\infty)\,i_k\tanh\frac{i_q}{i_k} - \lambda_m\, i_d\, h'(i_q)
$$

| Parameter | Meaning | Default |
|---|---|---|
| $i_k$ | knee current | 1.5–2 × rated current |
| $L_\infty$ | incremental q-inductance deep in saturation | $0.3\,L_q$ |
| $c$ | cross-saturation: fraction of magnet flux lost at very high $i_q$ ($0 \le c < 1$) | 0.15 |

- **Linear limit:** $c = 0$ and $L_\infty = L_q$ give back the linear model exactly.
- **Torque:** with $i_d = 0$, $T_e = \tfrac32 p\lambda_m(1 - h(i_q))\,i_q$. Torque per amp falls as current rises, which is what datasheet torque–current curves show.
- **Inversion:** solve $\psi_{dq}(i_d,i_q) = \psi$ with 2-D Newton from the previous step's currents. The Jacobian is the Hessian of $W'$ (symmetric incremental-inductance matrix). Parameter validation (P04.T05) rejects sets where it is not positive definite over the reachable current range.
- **Stored magnetic energy** (amplitude-invariant scaling, EQ-CONV-07): $W_{mag} = \tfrac32\big(\psi_d i_d + \psi_q i_q - W'\big)$. The linear case gives $\tfrac12 L_s\sum_x i_x^2$.
- **Approximation:** d-axis self-saturation is neglected. Surface-mount magnets give a large effective d-axis air gap. Verified numerically: the energy residual of this model halves when the step size halves, so any residual comes from the integrator, not the model.

## A disconnected or floating phase

### EQ-MOT-11 — Single-path (open-phase) mode {/* #eq-mot-11 */}

When terminal $c$ is open (`fault.phase_open.c`, or an inverter leg off with its current at zero), $i_c = 0$, so $i_a = -i_b = i$ and $i_{\alpha\beta} = i\,[1,\ -1/\sqrt3]^\top$. The two-state law EQ-MOT-01 no longer applies because $v_{cT}$ is unknown. The model switches to one state, the **line flux** $\psi_{ab} = \psi_a - \psi_b = \tfrac32\psi_\alpha - \tfrac{\sqrt3}{2}\psi_\beta$:

$$
\frac{d\psi_{ab}}{dt} = v_{aT} - v_{bT} - 2R\,i
$$

$i$ follows from $\psi_{ab}(i,\theta_e)$, built with EQ-MOT-02 along that current direction (a 1-D Newton solve). In the linear non-salient case this is $\psi_{ab} = 2L_s i + \lambda_m(\Phi_a - \Phi_b)$. The other phase pairs follow by symmetry. With two terminals open the current is zero.

### EQ-MOT-12 — Floating-terminal voltage and reconnection {/* #eq-mot-12 */}

In open-phase mode (linear, non-salient):

$$
v_n = \tfrac12\big(v_{aT} + v_{bT} - e_a - e_b\big),\qquad v_{cT} = v_n + e_c
$$

- **Inverter leg off** (not a physical cut): if $v_{cT}$ would rise above $V_{bus} + V_f$ or fall below $-V_f$, a free-wheeling diode starts conducting. That is a **state event** (EQ-NUM): the model returns to the two-state mode with the same $\psi_{\alpha\beta}$ (continuous) and the inverter clamps the terminal (EQ-INV).
- **Physical disconnect fault:** the phase never reconnects.
- Entering open-phase mode is also a state event: the leg is off **and** $i_c$ crosses zero.

## Power and energy

### EQ-MOT-13 — Motor power terms {/* #eq-mot-13 */}

$$
P_{elec} = \sum_x v_{xT}\,i_x = \tfrac32\,(v_\alpha i_\alpha + v_\beta i_\beta),\qquad
P_{cu} = R\sum_x i_x^2 = \tfrac32 R\,(i_\alpha^2 + i_\beta^2),\qquad
P_{ag} = T_e\,\omega_m
$$

$$
P_{elec} = P_{cu} + \frac{dW_{mag}}{dt} + P_{ag}
$$

The identity holds exactly for the continuous model. The simulator checks it numerically through `energy.residual` (EQ-ENER). Cogging ($U_{cog}$) and iron loss ($P_{fe}$) act on the mechanical side and are accounted there.

## Dimensional check

| Equation | Left side | Right side |
|---|---|---|
| EQ-MOT-01 | Wb/s = V | V − Ω·A = V ✓ |
| EQ-MOT-02 | Wb | H·A + Wb = Wb ✓ |
| EQ-MOT-03 | V | rad/s · Wb = V ✓ |
| EQ-MOT-07 | N·m | Wb·A = (V·s)·A = J = N·m ✓ |
| EQ-MOT-08 | $U$: J | N·m / (rad⁻¹·rad) = J ✓ |
| EQ-MOT-09 | W | (W/(rad/s))·rad/s + (W/(rad/s)²)·(rad/s)² ✓ |
| EQ-MOT-10 | $W'$: J | Wb·A + H·A² ✓ |

## References

- `[Krishnan2010]` ch. 3–4 (dq model, torque, saliency) and ch. 9–10 (BLDC, trapezoidal EMF, six-step).
- `[Mohan2014]` ch. 4–5: space-vector / flux-linkage formulation.
- `[HendershotMiller2010]`: cogging (LCM rule), iron losses, saturation behaviour of PM machines.

See [References](./references.md).
