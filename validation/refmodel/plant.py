"""Continuous plant: motor, rigid drivetrain, loads, thermal, averaged inverter, bus, supply.

One function ``Plant.eval`` computes, in a fixed order (EQ-NUM-02), every algebraic output
from the full state and the held discrete state, then the derivatives. Energy terms are
integrated as extra states (EQ-ENER-01, EQ-NUM-06).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from . import physics as ph
from .params import RefParams

# ---------------------------------------------------------------- state layout
PSA, PSB, TH, OM, TW, TS, THS, VB, V1, SOC = range(10)
E_IN, E_EXT, L_CU, L_FE, L_FR, L_GB, L_LD, L_INV, L_CH, L_BAT, E_THR = range(10, 21)
N_STATES = 21
# SPEC-AMBIGUITY: Q-09 (names of energy.loss.<module>)
LOSS_IDX = {
    "copper": L_CU,
    "iron": L_FE,
    "friction": L_FR,
    "gearbox": L_GB,
    "load": L_LD,
    "inverter": L_INV,
    "chopper": L_CH,
    "battery": L_BAT,
}

STICK = 2  # friction mode code; +1 / -1 = slipping in that direction; 0 = no stick-slip


@dataclass
class Disc:
    """Discrete (held) state: ZOH controller outputs and hybrid modes (EQ-NUM-02/05)."""

    leg_cmd: list[str] = field(default_factory=lambda: ["O", "O", "O"])  # pwm | ho | L | O
    duty: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    off_sub: list[str | None] = field(default_factory=lambda: [None, None, None])  # dlo|dhi|float
    zero_current: bool = False
    fric: int = 0
    psu_mode: str = "CV"
    chopper_on: bool = False
    t_dist: float = 0.0
    ctrl_sig: dict[str, Any] = field(default_factory=dict)


class Plant:
    def __init__(self, P: RefParams):
        self.P = P
        self.refresh()

    def refresh(self) -> None:
        """Recompute derived constants (called after parameter actions)."""
        P = self.P
        m, gb, ld = P.motor, P.gearbox, P.load
        self.shape = ph.EmfShape(m.emf_shape, m.trap_width, m.harmonics)
        self.sin_shape = m.emf_shape == "sinusoidal"
        self.N = gb.ratio
        self.j_load = ld.arm_mass * ld.arm_length**2 / 3.0 + ld.mass * ld.distance**2  # EQ-MECH-06
        self.m_eff = 0.5 * ld.arm_mass * ld.arm_length + ld.mass * ld.distance  # EQ-MECH-06
        # EQ-MECH-02
        self.j_eq = m.j_rotor + gb.j_in + (gb.j_out + self.j_load) / self.N**2
        # mechanical.md assumption 4: lumped rigid friction.
        # SPEC-AMBIGUITY: Q-18 (gearbox.friction.* is not part of the rigid lumping)
        self.ts_eq = m.friction_static + ld.friction_static / self.N
        self.tc_eq = m.friction_coulomb + ld.friction_coulomb / self.N
        self.fric_enabled = self.ts_eq > 0.0 or self.tc_eq > 0.0
        s = P.supply
        self.ocv_scale = float(s.series)
        self.r0 = s.series / s.parallel * s.r0_cell
        self.r1 = s.series / s.parallel * s.r1_cell
        self.c1 = s.parallel / s.series * s.c1_cell
        self.q_ah = s.parallel * s.q_cell_ah
        self.fixture = P.ctrl.kind == "ideal_voltage"

    def j_tot_load(self) -> float:
        """EQ-MECH-02: J_tot,L = J_load + J_go + N^2 (J_m + J_gi)."""
        P = self.P
        return self.j_load + P.gearbox.j_out + self.N**2 * (P.motor.j_rotor + P.gearbox.j_in)

    # ------------------------------------------------------------ helpers
    def temps(self, y: list[float]) -> tuple[float, float]:
        """EQ-THERM-02/03: (R, lambda_m) at the current node temperatures."""
        P = self.P
        if not P.thermal.enabled:
            return P.motor.r_phase, P.motor.lambda_m
        th = P.thermal
        r = ph.r_of_t(P.motor.r_phase, th.alpha_cu, y[TW], th.t_ref)
        t_mag = y[THS] if P.motor.topology == "outrunner" else y[TS]
        lam = ph.lambda_of_t(P.motor.lambda_m, P.motor.alpha_br, t_mag, th.t_ref)
        return r, lam

    def ocv(self, soc: float) -> float:
        s = self.P.supply
        return self.ocv_scale * ph.ocv_interp(soc, s.ocv_soc, s.ocv_cell)

    def currents(self, psa: float, psb: float, th_e: float, lam: float) -> tuple[float, float]:
        """EQ-MOT-02 inversion (linear magnetics): psi_ab -> i_ab."""
        m = self.P.motor
        hal, hbe = self.shape.phi_h_ab(th_e)
        pd, pq = ph.park(psa - lam * hal, psb - lam * hbe, th_e)
        i_d = (pd - lam) / m.l_d
        i_q = pq / m.l_q
        return ph.inv_park(i_d, i_q, th_e)

    def flux(self, ial: float, ibe: float, th_e: float, lam: float) -> tuple[float, float]:
        """EQ-MOT-02 forward: i_ab -> psi_ab (linear magnetics)."""
        m = self.P.motor
        i_d, i_q = ph.park(ial, ibe, th_e)
        pa, pb = ph.inv_park(m.l_d * i_d + lam, m.l_q * i_q, th_e)
        hal, hbe = self.shape.phi_h_ab(th_e)
        return pa + lam * hal, pb + lam * hbe

    def v_bus(self, y: list[float]) -> float:
        return self.P.supply.v_set if self.P.supply.kind == "ideal" else y[VB]

    # ------------------------------------------------------------ main evaluation
    def eval(self, t: float, yv: Any, d: Disc, full: bool = False) -> tuple[list[float], dict]:
        P = self.P
        m, gb, ld, th, inv, bus, sup = (
            P.motor,
            P.gearbox,
            P.load,
            P.thermal,
            P.inverter,
            P.bus,
            P.supply,
        )
        y = yv.tolist() if hasattr(yv, "tolist") else list(yv)
        dy = [0.0] * N_STATES
        p = m.pole_pairs
        r, lam = self.temps(y)
        th_m, om_m = y[TH], y[OM]
        th_e, om_e = p * th_m, p * om_m  # EQ-CONV-05

        # --- motor currents (EQ-MOT-02)
        if d.zero_current:
            ial = ibe = 0.0
        else:
            ial, ibe = self.currents(y[PSA], y[PSB], th_e, lam)
        i_abc = ph.inv_clarke(ial, ibe)
        i_d, i_q = ph.park(ial, ibe, th_e)
        k_abc = self.shape.k3(th_e)
        e_abc = tuple(om_e * lam * k for k in k_abc)  # EQ-MOT-03
        vb = self.v_bus(y)

        # --- terminal voltages (EQ-INV-01/03/07) or ideal fixture
        v_t = [math.nan, math.nan, math.nan]
        i_dc = 0.0
        p_fixture = 0.0
        float_leg = -1
        if self.fixture:
            ctx = {
                "theta_e": th_e,
                "omega_e": om_e,
                "theta_m": th_m,
                "omega_m": om_m,
                "r": r,
                "lambda_m": lam,
                "i_alpha": ial,
                "i_beta": ibe,
            }
            fn = P.ctrl.ideal_voltage
            v_al, v_be = fn(t, ctx) if fn is not None else (0.0, 0.0)
            v_t = list(ph.inv_clarke(v_al, v_be))
            p_fixture = 1.5 * (v_al * ial + v_be * ibe)
        elif d.zero_current:
            v_al, v_be = ph.clarke(*e_abc)  # keeps i = 0 exactly (EQ-MOT-11, zero-current mode)
        else:
            kdt = inv.dead_time * inv.f_pwm
            for x in range(3):
                ix = i_abc[x]
                cmd = d.leg_cmd[x]
                if cmd == "pwm":  # EQ-INV-03 averaged with dead-time error
                    sg = math.tanh(ix / inv.i_eps)
                    dx = d.duty[x]
                    v_t[x] = (
                        dx * vb
                        - inv.r_on * ix
                        - sg * kdt * (vb + 2 * inv.v_f - 2 * inv.r_on * abs(ix))
                    )
                    i_dc += (dx - sg * kdt) * ix  # EQ-INV-08 averaged
                elif cmd == "ho":  # EQ-INV-07 averaged six-step High leg (H/O)
                    # SPEC-AMBIGUITY: Q-03 (sign of i blended with sigma(i) as in EQ-INV-03)
                    dx = d.duty[x]
                    wp = 0.5 * (1.0 + math.tanh(ix / inv.i_eps))
                    v_off = wp * (-inv.v_f) + (1.0 - wp) * (vb + inv.v_f)
                    v_t[x] = dx * (vb - inv.r_on * ix) + (1.0 - dx) * v_off
                    i_dc += (wp * dx + (1.0 - wp)) * ix
                elif cmd == "L":
                    v_t[x] = -inv.r_on * ix
                else:  # Off: EQ-INV-01 diode / floating rule
                    sub = d.off_sub[x]
                    if sub == "dlo":
                        v_t[x] = -inv.v_f
                    elif sub == "dhi":
                        v_t[x] = vb + inv.v_f
                        i_dc += ix  # EQ-INV-08: high-side diode carries bus current
                    else:
                        float_leg = x
            if float_leg >= 0:  # EQ-MOT-12 (linear, non-salient)
                o1, o2 = [j for j in range(3) if j != float_leg]
                vn = 0.5 * (v_t[o1] + v_t[o2] - e_abc[o1] - e_abc[o2])
                v_t[float_leg] = vn + e_abc[float_leg]
            v_al, v_be = ph.clarke(*v_t)  # EQ-MOT-01
        dy[PSA] = v_al - r * ial  # EQ-MOT-01
        dy[PSB] = v_be - r * ibe

        # --- mechanics (EQ-MECH-02/03/05..08, EQ-MOT-07..09)
        N = self.N
        th_l, om_l = th_m / N, om_m / N
        if d.zero_current:
            t_e = 0.0
        else:
            t_e = ph.torque_em(p, lam, m.l_d, m.l_q, i_d, i_q, i_abc, k_abc, th_e, self.sin_shape)
        t_cog, u_cog = ph.cogging(m.cogging_nc, m.cogging, th_m) if m.cogging_nc > 0 else (0.0, 0.0)
        t_fe = ph.iron_loss_torque(m.k_hy, m.k_ed, p, om_m, m.omega_eps_fe)
        t_g = -ld.g * self.m_eff * math.sin(th_l)  # EQ-MECH-06
        u_g = ld.g * self.m_eff * (1.0 - math.cos(th_l))
        if ld.kind == "constant":  # EQ-MECH-07
            t_ext = ld.torque
        elif ld.kind == "brake":
            t_ext = -ld.torque * math.tanh(om_l / ld.omega_eps)
        elif ld.kind == "viscous":
            t_ext = -ld.viscous * om_l
        else:
            t_ext = 0.0
        t_dist = d.t_dist
        t_l0 = t_g + t_ext + t_dist  # load-side torques without load friction
        t_motor = t_e + t_cog + t_fe
        jl = gb.j_out + self.j_load
        prescribed = P.mech.mode == "prescribed"
        if prescribed:
            s = 0.0 if om_m == 0.0 else math.copysign(1.0, om_m)
            fric_mode = 0
        else:
            fric_mode = d.fric
            s = float(fric_mode) if fric_mode in (1, -1) else 0.0
        if fric_mode == STICK:
            t_fm = t_fl = 0.0
            t_gl = 0.0  # tanh(0) = 0
            t_a = t_motor + t_l0 / N  # EQ-MECH-05 applied torque
            t_f_eq = -t_a
            om_dot = 0.0
            dy[TH] = 0.0
        else:
            t_fm = -s * m.friction_coulomb - m.friction_viscous * om_m
            t_fl = -s * ld.friction_coulomb - ld.friction_viscous * om_l
            t_l = t_l0 + t_fl
            if prescribed:
                om_dot0 = 0.0
            else:
                om_dot0 = (t_motor + t_fm + t_l / N) / self.j_eq
            # SPEC-AMBIGUITY: Q-02 (T_L includes the load-bearing share of the lumped friction)
            tau_in = (jl / N * om_dot0 - t_l) / N  # EQ-MECH-03
            t_gl = -(1.0 - gb.efficiency) * abs(tau_in) * math.tanh(om_m / gb.omega_eps)
            t_a = t_motor + t_gl + t_l0 / N
            t_f_eq = t_fm + t_fl / N
            om_dot = 0.0 if prescribed else om_dot0 + t_gl / self.j_eq
            dy[TH] = om_m
        dy[OM] = om_dot
        p_fric = -(t_fm * om_m + t_fl * om_l)  # SPEC-AMBIGUITY: Q-10 (viscous part included)

        # --- thermal (EQ-THERM-01/04)
        p_cu = 1.5 * r * (ial * ial + ibe * ibe)  # EQ-MOT-13
        p_fe = -t_fe * om_m  # SPEC-AMBIGUITY: Q-04
        if th.enabled:
            r_ha = th.r_ha / (1.0 + th.kappa * abs(om_m))
            q_ws = (y[TW] - y[TS]) / th.r_ws
            q_sh = (y[TS] - y[THS]) / th.r_sh
            q_ha = (y[THS] - th.t_amb) / r_ha
            dy[TW] = (p_cu + th.p_inject - q_ws) / th.c_w
            dy[TS] = (p_fe + q_ws - q_sh) / th.c_s
            dy[THS] = (p_fric + q_sh - q_ha) / th.c_h

        # --- bus and supply (EQ-SUP-01..05, EQ-INV-09)
        if self.fixture:
            i_sw = 0.0
            i_dc = 0.0
        else:
            # Q-05 (answered): only legs that switch (PWM legs; the H/O leg in six-step).
            switching = [abs(i) for i, c in zip(i_abc, d.leg_cmd) if c in ("pwm", "ho")]
            i_sw = inv.f_pwm * 0.5 * (inv.t_rise + inv.t_fall) * sum(switching)
        i_ch = vb / bus.r_brake if (bus.chopper_enabled and d.chopper_on) else 0.0
        i_load = i_dc + i_sw + i_ch
        p_bat = 0.0
        if sup.kind == "ideal":
            i_src = i_load  # EQ-SUP-02
        elif sup.kind == "psu":  # EQ-SUP-03 by located mode
            if d.psu_mode == "CV":
                i_src = (sup.v_set - vb) / sup.r_o
            elif d.psu_mode == "CC":
                i_src = sup.i_lim
            else:
                i_src = 0.0
            dy[VB] = (i_src - i_load) / bus.capacitance
        else:  # EQ-SUP-04
            i_src = (self.ocv(y[SOC]) - y[V1] - vb) / self.r0
            dy[V1] = i_src / self.c1 - y[V1] / (self.r1 * self.c1)
            dy[SOC] = -i_src / (3600.0 * self.q_ah)
            dy[VB] = (i_src - i_load) / bus.capacitance
            p_bat = self.r0 * i_src * i_src + y[V1] ** 2 / self.r1

        # --- energy power terms (EQ-ENER-01)
        if self.fixture:
            p_in = p_fixture
            p_inv = 0.0
        else:
            p_in = vb * i_src if sup.kind != "battery" else 0.0
            p_inv = vb * (i_dc + i_sw) - sum(v_t[x] * i_abc[x] for x in range(3))  # EQ-INV-09
            if d.zero_current:
                p_inv = 0.0
        p_ext = 0.0
        p_load = 0.0
        if ld.kind == "constant":
            p_ext += t_ext * om_l
        else:
            p_load = -t_ext * om_l
        p_ext += t_dist * om_l
        if prescribed:  # kinematic fixture supplies whatever torque holds omega constant
            p_ext += -om_m * (t_motor + t_gl + t_fm + (t_l0 + t_fl) / N)
        p_gb = -t_gl * om_m
        p_ch = vb * i_ch
        dy[E_IN] = p_in
        dy[E_EXT] = p_ext
        dy[L_CU] = p_cu
        dy[L_FE] = p_fe
        dy[L_FR] = p_fric
        dy[L_GB] = p_gb
        dy[L_LD] = p_load
        dy[L_INV] = p_inv
        dy[L_CH] = p_ch
        dy[L_BAT] = p_bat
        # SPEC-AMBIGUITY: Q-06 (throughput = sum of |P_k| over the EQ-ENER-01 table terms)
        dy[E_THR] = (
            abs(p_in) + abs(p_ext) + p_cu + abs(p_fe) + abs(p_fric) + abs(p_gb) + abs(p_load)
        ) + (abs(p_inv) + p_ch + p_bat)

        if not full:
            return dy, {}

        # ---------------------------------------------------- algebraic outputs
        w_mag = 0.75 * (m.l_d * i_d * i_d + m.l_q * i_q * i_q)  # EQ-MOT-10 linear limit
        e_st = w_mag + 0.5 * self.j_eq * om_m * om_m + u_cog + u_g
        if sup.kind != "ideal":
            e_st += 0.5 * bus.capacitance * vb * vb
        if sup.kind == "battery":
            e_st += 0.5 * self.c1 * y[V1] ** 2
            e_st += (
                3600.0
                * self.q_ah
                * self.ocv_scale
                * ph.ocv_integral(y[SOC], sup.ocv_soc, sup.ocv_cell)
            )
        if d.zero_current:
            v_n = math.nan
        else:
            v_n = (sum(v_t) - sum(e_abc)) / 3.0  # EQ-MOT-04
        hb = ph.hall_bits(th_e)
        t_mag = (y[THS] if m.topology == "outrunner" else y[TS]) if th.enabled else th.t_amb
        if d.zero_current:
            open_phase = "all"
        elif float_leg >= 0:
            open_phase = "abc"[float_leg]
        else:
            open_phase = "none"
        legs = []
        for x in range(3):
            c = d.leg_cmd[x]
            legs.append({"ho": "H", "L": "L", "O": "O"}.get(c))  # SPEC-AMBIGUITY: Q-07
        sig: dict[str, Any] = {
            "motor.theta": th_m,
            "motor.omega": om_m,
            "motor.theta_e": th_e,
            "motor.omega_e": om_e,
            "motor.psi_alpha": y[PSA],
            "motor.psi_beta": y[PSB],
            "motor.i_a": i_abc[0],
            "motor.i_b": i_abc[1],
            "motor.i_c": i_abc[2],
            "motor.i_alpha": ial,
            "motor.i_beta": ibe,
            "motor.i_d": i_d,
            "motor.i_q": i_q,
            "motor.e_a": e_abc[0],
            "motor.e_b": e_abc[1],
            "motor.e_c": e_abc[2],
            "motor.v_n": v_n,
            "motor.torque_em": t_e,
            "motor.torque_cog": t_cog,
            "motor.p_cu": p_cu,
            "motor.p_fe": p_fe,
            "motor.open_phase": open_phase,
            "gearbox.delta": 0.0,
            "gearbox.p_loss": p_gb,
            "load.theta": th_l,
            "load.omega": om_l,
            "load.torque_gravity": t_g,
            "load.torque_ext": t_ext,
            "load.disturbance": t_dist,
            "thermal.t_winding": y[TW],
            "thermal.t_stator": y[TS],
            "thermal.t_housing": y[THS],
            "thermal.t_magnet": t_mag,
            "inverter.leg_a": legs[0],
            "inverter.leg_b": legs[1],
            "inverter.leg_c": legs[2],
            "inverter.duty_a": d.duty[0],
            "inverter.duty_b": d.duty[1],
            "inverter.duty_c": d.duty[2],
            "inverter.v_a": v_t[0],
            "inverter.v_b": v_t[1],
            "inverter.v_c": v_t[2],
            "inverter.i_dc": i_dc,
            "inverter.i_sw": i_sw,
            "inverter.p_loss": p_inv,
            "bus.v": vb,
            "bus.i_chopper": i_ch,
            "bus.chopper_on": bool(bus.chopper_enabled and d.chopper_on),
            "bus.p_brake": p_ch,
            "supply.i": i_src,
            "supply.mode": d.psu_mode if sup.kind == "psu" else None,
            "supply.soc": y[SOC] if sup.kind == "battery" else None,
            "supply.v_terminal": vb,
            "supply.battery.v_rc": y[V1] if sup.kind == "battery" else None,
            "sensors.hall.a": hb[0],
            "sensors.hall.b": hb[1],
            "sensors.hall.c": hb[2],
            "sensors.hall.code": 4 * hb[0] + 2 * hb[1] + hb[2],
            # internals used by events / energy
            "_t_a": t_a,
            "_t_f_eq": t_f_eq,
            "_e_st": e_st,
            "_p_elec": 1.5 * (v_al * ial + v_be * ibe),
            "_w_mag": w_mag,
        }
        return dy, sig
