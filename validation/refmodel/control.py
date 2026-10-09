"""Discrete controllers (zero-order hold between ticks, EQ-NUM-02). Sensors are ideal (bypass,
EQ-SENS-01): the controller reads the true state at the tick instant."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from . import physics as ph
from .params import RefParams


@dataclass
class PI:
    """EQ-CTRL-01: discrete PI with anti-windup (clamping or back-calculation)."""

    kp: float
    ki: float
    T: float
    lo: float = -math.inf
    hi: float = math.inf
    mode: str = "clamping"
    integ: float = 0.0

    def step(self, e: float, ff: float = 0.0) -> float:
        u_star = self.kp * e + self.integ + ff
        u = min(max(u_star, self.lo), self.hi)
        self.update(e, u_star, u)
        return u

    def update(self, e: float, u_star: float, u: float) -> None:
        if self.mode == "back_calculation":
            kb = self.ki / self.kp if self.kp != 0.0 else 0.0
            self.integ += self.T * (self.ki * e + kb * (u - u_star))
        else:
            saturated = u != u_star
            pushes = e * (u_star - u) > 0.0
            if not (saturated and pushes):
                self.integ += self.T * self.ki * e


def foc_gains(omega_c: float, ind: float, r: float) -> tuple[float, float]:
    """EQ-CTRL-03: K_p = omega_c L, K_i = omega_c R."""
    return omega_c * ind, omega_c * r


def velocity_gains(omega_v: float, j: float, kt: float) -> tuple[float, float]:
    """EQ-CTRL-04: K_pv = omega_v J / K_t, K_iv = K_pv omega_v / 4."""
    kp = omega_v * j / kt
    return kp, kp * omega_v / 4.0


def modulate(
    v_al: float, v_be: float, v_bus: float, scheme: str
) -> tuple[tuple[float, float, float], bool]:
    """EQ-INV-06 clamp, then EQ-INV-05 (SVPWM) or EQ-INV-04 (SPWM); duties clamped to [0,1]."""
    lim = v_bus / math.sqrt(3.0) if scheme == "svpwm" else v_bus / 2.0
    v_al, v_be, sat = ph.overmod_clamp(v_al, v_be, lim)
    vabc = ph.inv_clarke(v_al, v_be)
    dut = ph.svpwm(vabc, v_bus) if scheme == "svpwm" else ph.spwm(vabc, v_bus)
    return tuple(min(max(x, 0.0), 1.0) for x in dut), sat  # type: ignore[return-value]


class Controller:
    """Holds the state of the selected controller; ``tick`` returns the ZOH outputs."""

    def __init__(self, P: RefParams, j_eq: float):
        self.P = P
        c = P.ctrl
        self.T = round(1e9 / c.f_ctrl) * 1e-9 if c.f_ctrl > 0 else 0.0  # EQ-NUM-01
        m = P.motor
        self.kt = 1.5 * m.pole_pairs * m.lambda_m  # EQ-CONV-11 (nominal)
        f = c.foc
        wc = f.omega_c if f.omega_c is not None else 2.0 * math.pi * c.f_ctrl / 20.0
        self.omega_c = wc
        kpd, kid = foc_gains(wc, m.l_d, m.r_phase)
        kpq, kiq = foc_gains(wc, m.l_q, m.r_phase)
        self.pi_d = PI(kpd, kid, self.T, mode=f.anti_windup)
        self.pi_q = PI(kpq, kiq, self.T, mode=f.anti_windup)
        wv = f.omega_v if f.omega_v is not None else wc / 10.0
        self.omega_v = wv
        kpv, kiv = velocity_gains(wv, j_eq, self.kt)
        if f.kp_v is not None:
            kpv = f.kp_v
        if f.ki_v is not None:
            kiv = f.ki_v
        tv = self.T * f.vel_div
        self.pi_v = PI(kpv, kiv, tv, -c.i_max, c.i_max, f.anti_windup)
        wp = f.omega_p if f.omega_p is not None else wv / 5.0
        kpp = f.kp_p if f.kp_p is not None else wp
        self.pi_p = PI(kpp, f.ki_p, self.T * f.pos_div, -math.inf, math.inf, f.anti_windup)
        self.pref_alpha = 1.0 - math.exp(-tv / (4.0 / wv)) if wv > 0 else 1.0
        self.omega_ref_f: float | None = None
        self.iq_ref = 0.0
        self.omega_ref_act = 0.0
        self.k = 0
        s6 = c.six_step
        self.pi_6 = PI(s6.kp, s6.ki, self.T, 0.0, 1.0, "clamping")
        self.ol_theta = 0.0
        self.ol_omega = 0.0

    # ------------------------------------------------------------------
    def tick(self, meas: dict[str, float]) -> dict[str, Any]:
        kind = self.P.ctrl.kind
        out: dict[str, Any]
        if kind == "foc":
            out = self._foc(meas, None)
        elif kind == "mit":
            out = self._mit(meas)
        elif kind == "six_step":
            out = self._six_step(meas)
        elif kind == "open_loop":
            out = self._open_loop(meas)
        else:
            out = {"leg_cmd": ["O", "O", "O"], "duty": [0.0, 0.0, 0.0], "sig": {}}
        self.k += 1
        return out

    def _current_loop(
        self, meas: dict[str, float], id_ref: float, iq_ref: float
    ) -> tuple[list[float], dict[str, Any]]:
        """EQ-CTRL-03."""
        P = self.P
        m, f = P.motor, P.ctrl.foc
        th_e, om_e = meas["theta_e"], meas["omega_e"]
        i_al, i_be = ph.clarke(meas["i_a"], meas["i_b"], -meas["i_a"] - meas["i_b"])
        i_d, i_q = ph.park(i_al, i_be, th_e)
        e_d, e_q = id_ref - i_d, iq_ref - i_q
        vd_ff = -om_e * m.l_q * i_q
        vq_ff = om_e * (m.l_d * i_d + m.lambda_m)
        # SPEC-AMBIGUITY: Q-13 (saturation/anti-windup act on PI output + feed-forward)
        vd_star = self.pi_d.kp * e_d + self.pi_d.integ + vd_ff
        vq_star = self.pi_q.kp * e_q + self.pi_q.integ + vq_ff
        vb = meas["v_bus"]
        vmax = f.m_max * vb / math.sqrt(3.0)
        v_d = min(max(vd_star, -vmax), vmax)
        vq_lim = math.sqrt(max(vmax * vmax - v_d * v_d, 0.0))
        v_q = min(max(vq_star, -vq_lim), vq_lim)
        self.pi_d.update(e_d, vd_star, v_d)
        self.pi_q.update(e_q, vq_star, v_q)
        th_c = th_e + (1.5 * self.T * om_e if f.delay_comp else 0.0)
        v_al, v_be = ph.inv_park(v_d, v_q, th_c)
        duty, _ = modulate(v_al, v_be, vb, P.inverter.modulation)
        sig = {
            "ctrl.foc.id_ref": id_ref,
            "ctrl.foc.iq_ref": iq_ref,
            "ctrl.foc.v_d": v_d,
            "ctrl.foc.v_q": v_q,
            "ctrl.foc.saturated": bool(v_d != vd_star or v_q != vq_star),
        }
        return list(duty), sig

    def _foc(self, meas: dict[str, float], iq_override: float | None) -> dict[str, Any]:
        """EQ-CTRL-03/04: position -> velocity -> current cascade."""
        P = self.P
        c, f = P.ctrl, P.ctrl.foc
        sig: dict[str, Any] = {}
        # Q-14 (answered): all setpoints are load-side; motor-side speed = N · ω*.
        omega_ref = P.gearbox.ratio * c.omega_ref
        if f.mode == "position" and self.k % f.pos_div == 0:
            w_l = self.pi_p.step(c.theta_ref - meas["theta_l"])
            self.omega_ref_act = min(max(P.gearbox.ratio * w_l, -c.omega_max), c.omega_max)
        if f.mode == "position":
            omega_ref = self.omega_ref_act
            sig["ctrl.theta_ref"] = c.theta_ref
        if f.mode in ("velocity", "position") and self.k % f.vel_div == 0:
            r = omega_ref
            if f.prefilter:
                if self.omega_ref_f is None:
                    self.omega_ref_f = 0.0
                self.omega_ref_f += self.pref_alpha * (r - self.omega_ref_f)
                r = self.omega_ref_f
            self.iq_ref = self.pi_v.step(r - meas["omega_m"])
        if f.mode in ("velocity", "position"):
            sig["ctrl.omega_ref"] = omega_ref
            iq_ref = self.iq_ref
        else:
            iq_ref = min(max(f.iq_ref, -c.i_max), c.i_max)
        if iq_override is not None:
            iq_ref = iq_override
        duty, s2 = self._current_loop(meas, f.id_ref, iq_ref)
        sig.update(s2)
        return {"leg_cmd": ["pwm"] * 3, "duty": duty, "sig": sig}

    def _mit(self, meas: dict[str, float]) -> dict[str, Any]:
        """EQ-CTRL-07: tau* = Kp (theta* - theta_L) + Kd (omega* - omega_L) + tau_ff."""
        c = self.P.ctrl
        mp = c.mit
        # Q-14 (answered): ctrl.omega_ref is load-side in every mode.
        tau = mp.kp * (c.theta_ref - meas["theta_l"]) + mp.kd * (c.omega_ref - meas["omega_l"])
        tau += mp.tau_ff
        iq = min(max(tau / (self.P.gearbox.ratio * self.kt), -c.i_max), c.i_max)
        out = self._foc(meas, iq)
        out["sig"].update(
            {"ctrl.torque_ref": tau, "ctrl.theta_ref": c.theta_ref, "ctrl.omega_ref": c.omega_ref}
        )
        return out

    def _six_step(self, meas: dict[str, float]) -> dict[str, Any]:
        """EQ-CTRL-05 with ideal Halls (EQ-SENS-02) and the EQ-INV-07 table."""
        c = self.P.ctrl
        s6 = c.six_step
        # SPEC-AMBIGUITY: Q-15 (commutation only at controller ticks, ZOH per EQ-NUM-02)
        hb = ph.hall_bits(meas["theta_e"] + s6.advance)
        code = 4 * hb[0] + 2 * hb[1] + hb[2]
        # SPEC-AMBIGUITY: Q-16 (ctrl.six_step.sector numbering 0..5 in EQ-INV-07 table order)
        if code not in ph.HALL_TABLE:  # codes 0 / 7 are a fault: all legs off
            return {"leg_cmd": ["O"] * 3, "duty": [0.0] * 3, "sig": {"ctrl.six_step.sector": -1}}
        sector, hi, lo = ph.HALL_TABLE[code]
        if s6.direction < 0:
            hi, lo = lo, hi
        if s6.speed_pi:
            duty = self.pi_6.step(c.omega_ref - meas["omega_m"])
            sig = {"ctrl.omega_ref": c.omega_ref}
        else:
            duty = min(max(s6.duty, 0.0), 1.0)
            sig = {}
        legs = ["O", "O", "O"]
        duties = [0.0, 0.0, 0.0]
        legs[hi] = "ho"
        duties[hi] = duty
        legs[lo] = "L"
        sig["ctrl.six_step.sector"] = sector
        return {"leg_cmd": legs, "duty": duties, "sig": sig}

    def _open_loop(self, meas: dict[str, float]) -> dict[str, Any]:
        """EQ-CTRL-06: open-loop V/f."""
        P = self.P
        ol = P.ctrl.open_loop
        dw = ol.omega_e_ref - self.ol_omega
        step = ol.accel * self.T
        self.ol_omega += min(max(dw, -step), step)
        self.ol_theta += self.ol_omega * self.T
        vmag = ol.v0 + ol.k_vf * abs(self.ol_omega)
        v_al, v_be = -vmag * math.sin(self.ol_theta), vmag * math.cos(self.ol_theta)
        duty, _ = modulate(v_al, v_be, meas["v_bus"], P.inverter.modulation)
        return {
            "leg_cmd": ["pwm"] * 3,
            "duty": list(duty),
            "sig": {"ctrl.open_loop.theta_ref": self.ol_theta},
        }
