"""Pure equation functions, one per spec equation (CONVENTIONS section 6)."""

from __future__ import annotations

import math

SQRT3 = math.sqrt(3.0)
TWO_PI_3 = 2.0 * math.pi / 3.0
TWO_PI = 2.0 * math.pi


def clarke(a: float, b: float, c: float) -> tuple[float, float]:
    """EQ-CONV-01, amplitude-invariant Clarke (zero sequence dropped)."""
    return (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c), (b - c) / SQRT3


def inv_clarke(al: float, be: float) -> tuple[float, float, float]:
    """EQ-CONV-02 with i_0 = 0."""
    return al, -0.5 * al + 0.5 * SQRT3 * be, -0.5 * al - 0.5 * SQRT3 * be


def park(al: float, be: float, th: float) -> tuple[float, float]:
    """EQ-CONV-03."""
    c, s = math.cos(th), math.sin(th)
    return al * c + be * s, -al * s + be * c


def inv_park(d: float, q: float, th: float) -> tuple[float, float]:
    """EQ-CONV-04."""
    c, s = math.cos(th), math.sin(th)
    return d * c - q * s, d * s + q * c


def wrap_2pi(x: float) -> float:
    return x % TWO_PI


def wrap_pi(x: float) -> float:
    return (x + math.pi) % TWO_PI - math.pi


# ---------------------------------------------------------------- EMF shapes (EQ-MOT-03)


def trap_b1(w: float) -> float:
    """EQ-MOT-03: fundamental of the unit odd trapezoid, b1 = (4/pi) sin(r)/r, r = (pi-w)/2."""
    r = 0.5 * (math.pi - w)
    return 4.0 / math.pi * math.sin(r) / r


def _trap(th: float, w: float) -> float:
    """Unit-amplitude odd trapezoid with flat-top width w per half period."""
    r = 0.5 * (math.pi - w)
    x = th % TWO_PI
    sgn = 1.0
    if x >= math.pi:
        x -= math.pi
        sgn = -1.0
    if x < r:
        v = x / r
    elif x <= math.pi - r:
        v = 1.0
    else:
        v = (math.pi - x) / r
    return sgn * v


def _trap_int(th: float, w: float) -> float:
    """Zero-mean integral of the trapezoid (period 2 pi)."""
    r = 0.5 * (math.pi - w)
    a_half = math.pi - r  # integral over one half period
    x = th % TWO_PI
    neg = False
    if x >= math.pi:
        x -= math.pi
        neg = True
    if x < r:
        f = x * x / (2.0 * r)
    elif x <= math.pi - r:
        f = 0.5 * r + (x - r)
    else:
        f = a_half - (math.pi - x) ** 2 / (2.0 * r)
    if neg:
        f = a_half - f
    return f - 0.5 * a_half


class EmfShape:
    """Normalised back-EMF shape k(theta) and flux shape Phi(theta) of EQ-MOT-03.

    k is normalised so that its fundamental is -sin(theta); Phi = integral of k, zero mean.
    """

    def __init__(
        self, kind: str, trap_width: float, harmonics: tuple[tuple[int, float, float], ...]
    ):
        self.kind = kind
        self.w = trap_width
        self.harm = tuple(harmonics)
        if kind == "trapezoidal":
            self.b1 = trap_b1(trap_width)
        elif kind not in ("sinusoidal", "harmonics"):
            raise ValueError(f"unknown emf_shape {kind!r}")

    def k(self, th: float) -> float:
        if self.kind == "sinusoidal":
            return -math.sin(th)
        if self.kind == "trapezoidal":
            return -_trap(th, self.w) / self.b1
        return -(math.sin(th) + sum(b * math.sin(n * th + ph) for n, b, ph in self.harm))

    def phi(self, th: float) -> float:
        if self.kind == "sinusoidal":
            return math.cos(th)
        if self.kind == "trapezoidal":
            return -_trap_int(th, self.w) / self.b1
        return math.cos(th) + sum(b / n * math.cos(n * th + ph) for n, b, ph in self.harm)

    def phi_h_ab(self, th: float) -> tuple[float, float]:
        """EQ-MOT-02: Clarke of the per-phase harmonic flux Phi(th - x 2pi/3) - cos(...)."""
        if self.kind == "sinusoidal":
            return 0.0, 0.0
        h = [self.phi(th - x * TWO_PI_3) - math.cos(th - x * TWO_PI_3) for x in range(3)]
        return clarke(h[0], h[1], h[2])

    def k3(self, th: float) -> tuple[float, float, float]:
        return self.k(th), self.k(th - TWO_PI_3), self.k(th + TWO_PI_3)


def torque_em(
    p: int,
    lam: float,
    l_d: float,
    l_q: float,
    i_d: float,
    i_q: float,
    i_abc: tuple[float, float, float],
    k_abc: tuple[float, float, float],
    th_e: float,
    shape_sin: bool,
) -> float:
    """EQ-MOT-07 (linear magnetics): 1.5 p (psi_d i_q - psi_q i_d) + harmonic term."""
    psi_d = l_d * i_d + lam
    psi_q = l_q * i_q
    t = 1.5 * p * (psi_d * i_q - psi_q * i_d)
    if not shape_sin:
        kh = 0.0
        for x in range(3):
            kh += (k_abc[x] + math.sin(th_e - x * TWO_PI_3)) * i_abc[x]
        t += p * lam * kh
    return t


def r_of_t(r_ref: float, alpha_cu: float, t_w: float, t_ref: float) -> float:
    """EQ-THERM-02."""
    return r_ref * (1.0 + alpha_cu * (t_w - t_ref))


def lambda_of_t(lam_ref: float, alpha_br: float, t_mag: float, t_ref: float) -> float:
    """EQ-THERM-03."""
    return lam_ref * (1.0 + alpha_br * (t_mag - t_ref))


def cogging(nc: int, terms: tuple[tuple[float, float], ...], th_m: float) -> tuple[float, float]:
    """EQ-MOT-08: returns (T_cog, U_cog)."""
    t = 0.0
    u = 0.0
    for k, (a, ph) in enumerate(terms, start=1):
        arg = k * nc * th_m + ph
        t += a * math.sin(arg)
        u += a / (k * nc) * math.cos(arg)
    return t, u


def iron_loss_torque(k_hy: float, k_ed: float, p: int, om_m: float, om_eps: float) -> float:
    """EQ-MOT-09: T_fe = -(k_hy p tanh(w/w_eps) + k_ed p^2 w)."""
    return -(k_hy * p * math.tanh(om_m / om_eps) + k_ed * p * p * om_m)


# ---------------------------------------------------------------- modulation (EQ-INV-04..06)


def spwm(v_abc: tuple[float, float, float], v_bus: float) -> tuple[float, float, float]:
    """EQ-INV-04: d_x = 1/2 + v_x*/V_bus."""
    return tuple(0.5 + v / v_bus for v in v_abc)  # type: ignore[return-value]


def svpwm(v_abc: tuple[float, float, float], v_bus: float) -> tuple[float, float, float]:
    """EQ-INV-05: min-max injection."""
    vo = -0.5 * (max(v_abc) + min(v_abc))
    return tuple(0.5 + (v + vo) / v_bus for v in v_abc)  # type: ignore[return-value]


def overmod_clamp(v_al: float, v_be: float, v_lim: float) -> tuple[float, float, bool]:
    """EQ-INV-06: keep the angle, shorten to the linear limit."""
    mag = math.hypot(v_al, v_be)
    if mag > v_lim and mag > 0.0:
        s = v_lim / mag
        return v_al * s, v_be * s, True
    return v_al, v_be, False


# ---------------------------------------------------------------- sensors (EQ-SENS-02)


def hall_bits(th_e: float) -> tuple[int, int, int]:
    """EQ-SENS-02 with delta = h = 0 (ideal Halls)."""
    return tuple(1 if math.cos(th_e - (x + 1) * TWO_PI_3) > 0.0 else 0 for x in range(3))  # type: ignore[return-value]


# EQ-SENS-02 code table -> EQ-INV-07 (high leg, low leg), legs a=0, b=1, c=2.
HALL_TABLE: dict[int, tuple[int, int, int]] = {
    # code: (sector index 0..5 in EQ-INV-07 table order, high, low)
    1: (0, 1, 2),
    5: (1, 1, 0),
    4: (2, 2, 0),
    6: (3, 2, 1),
    2: (4, 0, 1),
    3: (5, 0, 2),
}


# ---------------------------------------------------------------- supply (EQ-SUP-04)


def ocv_interp(soc: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    """Piecewise-linear OCV(SoC), clamped at the table ends."""
    if soc <= xs[0]:
        return ys[0]
    if soc >= xs[-1]:
        return ys[-1]
    for j in range(len(xs) - 1):
        if soc <= xs[j + 1]:
            f = (soc - xs[j]) / (xs[j + 1] - xs[j])
            return ys[j] + f * (ys[j + 1] - ys[j])
    return ys[-1]


def ocv_integral(soc: float, xs: tuple[float, ...], ys: tuple[float, ...]) -> float:
    """Exact integral_0^SoC OCV(s) ds of the piecewise-linear table (EQ-SUP-04 E_chem)."""
    total = 0.0
    lo = 0.0
    sign = 1.0
    if soc < 0.0:
        lo, soc, sign = soc, 0.0, -1.0
    # integrate from lo to soc using trapezoids on the breakpoints
    pts = sorted({lo, soc, *[x for x in xs if lo < x < soc]})
    for a, b in zip(pts[:-1], pts[1:], strict=True):
        total += 0.5 * (b - a) * (ocv_interp(a, xs, ys) + ocv_interp(b, xs, ys))
    return sign * total
