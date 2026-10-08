"""V-SUP-* cases (EQ-SUP)."""

from __future__ import annotations

import math

import numpy as np
from scipy.integrate import simpson
from scipy.linalg import expm

from refmodel import Action, simulate

from .helpers import close, m1


def test_v_sup_001_psu_current_limit():
    P = m1()
    P.supply.kind = "psu"
    P.supply.i_lim, P.supply.r_o = 1.0, 0.05
    P.bus.chopper_enabled = True  # used as a resistive bus load
    P.bus.r_brake, P.bus.v_on, P.bus.v_off = 10.0, 1.0, 0.5
    df = simulate(P, [], 0.1, ["supply.i", "supply.mode", "bus.v"], 1e-3)
    assert df["supply.mode"][-1] == "CC"
    assert close(df["supply.i"][-1], 1.0, atol=1e-6)
    assert close(df["bus.v"][-1], 10.0, rtol=1e-6)
    assert df["supply.mode"][0] == "CV"


def test_v_sup_002_flywheel_brake_into_blocking_psu():
    P = m1()
    P.supply.kind = "psu"
    P.motor.omega0 = 30.0
    P.ctrl.kind = "foc"
    P.ctrl.foc.mode = "velocity"
    P.ctrl.omega_ref = 0.0
    P.ctrl.i_max = 3.0
    df = simulate(P, [], 0.1, ["bus.v", "motor.omega", "supply.mode", "energy.residual"], 1e-4)
    v2 = math.sqrt(24.0**2 + 2.5e-4 * 30.0**2 / 470e-6)
    assert close(v2, 32.48, atol=0.005)
    assert df["bus.v"].max() <= v2
    assert df["bus.v"].max() > 25.0
    assert "BLOCKING" in df["supply.mode"].to_list()
    assert abs(df["motor.omega"][-1]) < 0.5
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-3


def _battery(P):
    s = P.supply
    s.kind = "battery"
    s.series, s.parallel = 6, 2
    s.r0_cell, s.r1_cell, s.c1_cell, s.q_cell_ah = 0.02, 0.01, 200.0, 2.0
    s.soc0 = 0.8
    return P


def test_v_sup_004_battery_step_analytic():
    P = _battery(m1())
    P.supply.ocv_cell = (3.7,) * 7  # flat OCV -> linear system
    P.bus.capacitance = 470e-6
    P.bus.r_brake, P.bus.v_on, P.bus.v_off = 2.0, 1.0, 0.5
    t_on = 0.01
    df = simulate(
        P,
        [Action(t_on, "bus.chopper_enabled", True)],
        0.5,
        ["bus.v", "supply.battery.v_rc", "supply.i"],
        1e-3,
    )
    ocv = 6 * 3.7
    r0, r1, c1 = 6 / 2 * 0.02, 6 / 2 * 0.01, 2 / 6 * 200.0
    c, rb = 470e-6, 2.0
    # states [V, v1]; C V' = (ocv - v1 - V)/R0 - V/Rb ; C1 v1' = (ocv - v1 - V)/R0 - v1/R1
    a = np.array(
        [
            [-1 / (r0 * c) - 1 / (rb * c), -1 / (r0 * c)],
            [-1 / (r0 * c1), -1 / (r0 * c1) - 1 / (r1 * c1)],
        ]
    )
    b = np.array([ocv / (r0 * c), ocv / (r0 * c1)])
    x_ss = -np.linalg.solve(a, b)
    t = df["t"].to_numpy()
    x0 = np.array([ocv, 0.0])
    ref = np.array([x_ss + expm(a * (tt - t_on)) @ (x0 - x_ss) if tt >= t_on else x0 for tt in t]).T
    assert close(df["bus.v"].to_numpy(), ref[0], rtol=1e-6)
    assert close(df["supply.battery.v_rc"].to_numpy(), ref[1], rtol=1e-6, atol=1e-9)
    # instant sag (after the bus RC settles, << R1 C1) is I R0; relaxation constant R1 C1
    k = int(np.searchsorted(t, t_on + 0.005))
    i_k = df["supply.i"][k]
    sag = ocv - df["bus.v"][k]
    assert close(sag, i_k * r0 + df["supply.battery.v_rc"][k], rtol=1e-6)
    assert close(r1 * c1, 0.01 * 200.0, rtol=1e-12)


def test_v_sup_005_battery_soc_coulomb_count():
    P = _battery(m1())
    P.bus.chopper_enabled = True
    P.bus.r_brake, P.bus.v_on, P.bus.v_off = 2.0, 1.0, 0.5
    ocv0 = 6 * (3.70 + (0.8 - 0.5) / 0.3 * 0.25)
    P.bus.v0 = ocv0 * 2.0 / (2.0 + 0.06)  # start at the resistive-divider equilibrium
    P.sim.rtol = 1e-12  # Delta SoC is ~2e-3 of the SoC state: tighten for rtol 1e-9 on it
    df = simulate(P, [], 1.0, ["supply.i", "supply.soc"], 2.5e-4)
    q = simpson(df["supply.i"].to_numpy(), x=df["t"].to_numpy())
    dsoc = df["supply.soc"][0] - df["supply.soc"][-1]
    assert close(dsoc, q / (3600 * 4.0), rtol=1e-9)
