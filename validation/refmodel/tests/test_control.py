"""V-CTRL-* cases (EQ-CTRL), ideal (bypass) sensors."""

from __future__ import annotations

import math

import numpy as np
import pytest

from refmodel import Action, simulate
from refmodel.control import PI

from .helpers import close, m1


def test_pi_anti_windup_modes():
    """EQ-CTRL-01: clamping freezes I while pushing into the limit; back-calculation bleeds it."""
    c = PI(1.0, 10.0, 1e-3, -1.0, 1.0, "clamping")
    for _ in range(100):
        assert c.step(5.0) == 1.0
    assert c.integ == 0.0
    b = PI(1.0, 10.0, 1e-3, -1.0, 1.0, "back_calculation")
    for _ in range(5000):
        b.step(5.0)
    # steady state of I += T (Ki e + Kb (u - u*)) with u* = e + I: Ki e = Kb (e + I - 1)
    assert close(b.integ, 1.0 - 5.0 + 10.0 * 5.0 / 10.0, rtol=1e-6)


def test_v_ctrl_001_current_loop_time_constant():
    P = m1()
    P.mech.mode = "prescribed"
    P.ctrl.kind = "foc"
    P.ctrl.f_ctrl = 20e3
    t_s = 1e-3
    df = simulate(P, [Action(t_s, "ctrl.foc.iq_ref", 1.0)], 4e-3, ["motor.i_q"], 2e-6)
    t = df["t"].to_numpy()
    iq = df["motor.i_q"].to_numpy()
    t_step = t_s  # commands at t_s are applied before the tick at t_s (EQ-NUM-02, Q-17)
    t63 = t[np.argmax(iq >= 1 - math.exp(-1))] - t_step
    wc = 2 * math.pi * 20e3 / 20
    assert close(t63, 1 / wc, rtol=0.15)


@pytest.mark.parametrize("prefilter,expected", [(False, 0.135), (True, 0.0)])
def test_v_ctrl_002_velocity_step_overshoot(prefilter, expected):
    P = m1()
    P.ctrl.kind = "foc"
    P.ctrl.foc.mode = "velocity"
    P.ctrl.foc.prefilter = prefilter
    step = 10.0
    df = simulate(P, [Action(0.002, "ctrl.omega_ref", step)], 0.1, ["motor.omega"], 1e-4)
    w = df["motor.omega"].to_numpy()
    overshoot = max(w.max() - step, 0.0) / step
    assert abs(overshoot - expected) <= 0.02
    assert abs(w[-1] - step) < 0.02 * step


def test_v_ctrl_003_mit_static_deflection():
    P = m1()
    P.ctrl.kind = "mit"
    P.ctrl.f_ctrl = 5e3
    # eta = 1: with eta < 1 the tanh-smoothed mesh loss (EQ-MECH-03) behaves like a large
    # viscous drag below omega_eps and the final creep to tau_L/K_p takes seconds.
    P.gearbox.ratio, P.gearbox.efficiency = 3.0, 1.0
    P.load.kind = "constant"
    P.load.torque = 0.05
    kp = 2.0
    j_l = 9 * 2.5e-4
    P.ctrl.mit.kp = kp
    P.ctrl.mit.kd = 2 * 0.7 * math.sqrt(kp * j_l)  # zeta = 0.7
    df = simulate(P, [], 0.6, ["load.theta", "ctrl.torque_ref"], 1e-3)
    assert close(df["load.theta"][-1], 0.05 / kp, rtol=0.02)
    assert close(df["ctrl.torque_ref"][-1], -0.05, rtol=0.02)


def test_v_ctrl_position_loop_p_only_error():
    """EQ-CTRL-04: P-position + PI velocity -> zero steady error under constant load."""
    P = m1()
    P.ctrl.kind = "foc"
    P.ctrl.foc.mode = "position"
    P.ctrl.f_ctrl = 5e3
    P.load.kind = "constant"
    P.load.torque = 0.02
    df = simulate(P, [Action(0.0, "ctrl.theta_ref", 0.5)], 0.6, ["load.theta"], 1e-3)
    assert abs(df["load.theta"][-1] - 0.5) < 1e-3
