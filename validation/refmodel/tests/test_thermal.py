"""V-THERM-* cases (EQ-THERM)."""

from __future__ import annotations

import math

import numpy as np

from refmodel import physics as ph
from refmodel import simulate

from .helpers import close, m1


def _thermal(P):
    th = P.thermal
    th.enabled = True
    th.c_w, th.c_s, th.c_h = 2.0, 5.0, 10.0
    th.r_ws, th.r_sh, th.r_ha = 0.5, 0.5, 4.0
    th.t_amb = 298.15
    return P


def test_v_therm_001_linear_network_analytic():
    P = _thermal(m1())
    P.thermal.alpha_cu = 0.0
    P.thermal.p_inject = 5.0
    P.mech.mode = "prescribed"
    df = simulate(P, [], 200.0, ["thermal.t_winding", "thermal.t_stator", "thermal.t_housing"], 1.0)
    th = P.thermal
    a = np.array(
        [
            [-1 / (th.r_ws * th.c_w), 1 / (th.r_ws * th.c_w), 0.0],
            [1 / (th.r_ws * th.c_s), -(1 / th.r_ws + 1 / th.r_sh) / th.c_s, 1 / (th.r_sh * th.c_s)],
            [0.0, 1 / (th.r_sh * th.c_h), -(1 / th.r_sh + 1 / th.r_ha) / th.c_h],
        ]
    )
    b = np.array([5.0 / th.c_w, 0.0, 0.0])
    x_ss = -np.linalg.solve(a, b)
    lam, vec = np.linalg.eig(a)
    c0 = np.linalg.solve(vec, -x_ss)
    t = df["t"].to_numpy()
    x = x_ss[:, None] + (vec @ (c0[:, None] * np.exp(np.outer(lam, t)))).real
    sim = np.vstack(
        [df[c].to_numpy() for c in ("thermal.t_winding", "thermal.t_stator", "thermal.t_housing")]
    )
    assert close(sim - th.t_amb, x, rtol=1e-6, atol=1e-9)


def test_v_therm_002_continuous_current_reaches_t_max():
    P = _thermal(m1())
    th = P.thermal
    t_max = 373.15
    r_th = th.r_ws + th.r_sh + th.r_ha
    # EQ-THERM-05 with P_fe = P_fric = 0
    i_cont = math.sqrt(
        (t_max - th.t_amb) / (1.5 * P.motor.r_phase * (1 + th.alpha_cu * (t_max - th.t_ref)) * r_th)
    )
    P.mech.mode = "prescribed"
    P.ctrl.kind = "ideal_voltage"
    # current regulator fixture: v = R(T_w) * I along alpha (locked rotor, no EMF)
    P.ctrl.ideal_voltage = lambda t, ctx: (ctx["r"] * i_cont, 0.0)
    P.sim.method = "Radau"
    df = simulate(P, [], 1500.0, ["thermal.t_winding", "motor.i_alpha"], 50.0)
    assert close(df["motor.i_alpha"][-1], i_cont, rtol=1e-6)
    assert abs(df["thermal.t_winding"][-1] - t_max) <= 1.0


def test_v_therm_003_resistance_ratio():
    r100 = ph.r_of_t(1.0, 0.00393, 373.15, 293.15)
    r20 = ph.r_of_t(1.0, 0.00393, 293.15, 293.15)
    assert close(r100 / r20, 1.3144, rtol=1e-9)
    P = _thermal(m1())
    P.thermal.t_init = 373.15
    P.mech.mode = "prescribed"
    df = simulate(P, [], 1e-3, ["thermal.t_winding"], 1e-3)
    assert df["thermal.t_winding"][0] == 373.15


def test_magnet_weakening_eq_therm_03():
    """At 80 C the torque per amp is about 7 % lower than at 20 C."""
    lam80 = ph.lambda_of_t(1.0, -0.0012, 353.15, 293.15)
    assert close(lam80, 0.928, rtol=1e-12)
