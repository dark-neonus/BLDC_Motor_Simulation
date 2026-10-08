"""V-MECH-* cases (EQ-MECH, rigid drivetrain)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from refmodel import Action, simulate

from .helpers import close, crossings, m1

G = 9.80665


def _arm(P, theta0):
    P.motor.friction_viscous = 0.0
    P.load.arm_mass, P.load.arm_length = 0.1, 0.2
    P.load.mass, P.load.distance = 0.5, 0.15
    P.load.theta0 = theta0
    return P


def test_v_mech_001_pendulum_period():
    P = _arm(m1(), 0.01)
    df = simulate(P, [], 4.0, ["load.theta"], 1e-3)
    tc = crossings(df["t"].to_numpy(), df["load.theta"].to_numpy())
    period = np.mean(np.diff(tc))
    j_tot = 0.1 * 0.2**2 / 3 + 0.5 * 0.15**2 + 2.5e-4
    m_eff = 0.1 * 0.2 / 2 + 0.5 * 0.15
    t0 = 2 * math.pi * math.sqrt(j_tot / (G * m_eff))
    assert close(t0, 0.77961, rtol=1e-5)
    assert close(period, t0, rtol=5e-4)


def test_v_mech_002_large_swing_energy():
    P = _arm(m1(), 2.0)
    df = simulate(P, [], 10 * 1.0, ["load.theta", "energy.residual"], 1e-2)
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6
    assert df["load.theta"].max() > 1.99


def test_v_mech_003_torque_step_through_gearbox():
    P = m1()
    P.motor.friction_viscous = 0.0
    P.gearbox.ratio, P.gearbox.j_in, P.gearbox.j_out = 5.0, 1e-5, 4e-4
    P.load.mass, P.load.distance, P.load.g = 0.3, 0.1, 0.0
    P.load.kind = "constant"
    tq = 0.2
    df = simulate(P, [Action(0.01, "load.torque", tq)], 0.05, ["motor.omega", "load.omega"], 1e-3)
    j_eq = 2.5e-4 + 1e-5 + (4e-4 + 0.3 * 0.1**2) / 25
    t = df["t"].to_numpy()
    w = df["motor.omega"].to_numpy()
    sel = t >= 0.01
    assert close(w[sel], (tq / 5 / j_eq) * (t[sel] - 0.01), rtol=1e-9, atol=1e-12)
    assert close(df["load.omega"].to_numpy(), w / 5, rtol=1e-12)


def _stiction(P):
    P.motor.friction_static = 0.01
    P.motor.friction_coulomb = 0.008
    P.motor.friction_viscous = 1e-4
    P.load.kind = "constant"
    return P


def test_v_mech_006_holds_below_static():
    P = _stiction(m1())
    P.motor.theta0 = 0.123
    df = simulate(
        P, [Action(0.01, "load.torque", 0.98 * 0.01)], 0.2, ["motor.theta", "motor.omega"], 1e-3
    )
    assert np.all(df["motor.theta"].to_numpy() == 0.123)
    assert np.all(df["motor.omega"].to_numpy() == 0.0)


def test_v_mech_007_breakaway_and_restick():
    P = _stiction(m1())
    ts, tc, b, j = 0.01, 0.008, 1e-4, 2.5e-4
    acts = [Action(0.01, "load.disturbance.torque", 1.2 * ts, duration=0.05)]
    df = simulate(P, acts, 0.4, ["motor.theta", "motor.omega", "energy.residual"], 1e-4)
    t = df["t"].to_numpy()
    w = df["motor.omega"].to_numpy()
    assert np.all(w[t < 0.01] == 0.0)
    assert w[np.searchsorted(t, 0.06)] > 0.0
    # after release: J w' = -T_c - b w  -> analytic decay until the stop
    t_r = 0.06
    w_r = w[np.searchsorted(t, t_r)]
    tau = j / b
    ref = (w_r + tc / b) * np.exp(-(t - t_r) / tau) - tc / b
    sel = (t >= t_r) & (ref > 0.05 * w_r)
    assert close(w[sel], ref[sel], rtol=1e-3, atol=1e-9)
    # re-sticks: exactly still at the end
    th = df["motor.theta"].to_numpy()
    assert w[-1] == 0.0 and th[-1] == th[-200]
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-3


def test_v_mech_008_impulse():
    P = m1()
    P.motor.friction_viscous = 0.0
    P.gearbox.ratio, P.gearbox.j_out = 3.0, 1e-4
    P.load.mass, P.load.distance, P.load.g = 0.2, 0.1, 0.0
    h = 0.01
    df = simulate(
        P,
        [Action(0.005, "load.disturbance.impulse", h)],
        0.02,
        ["load.omega", "energy.residual"],
        1e-3,
    )
    j_tot_l = 0.2 * 0.01 + 1e-4 + 9 * 2.5e-4
    assert close(df["load.omega"][-1], h / j_tot_l, rtol=1e-6)
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6


@pytest.mark.parametrize("mode", ["keep_speed", "conserve_momentum"])
def test_v_mech_009_live_mass_change(mode):
    P = m1()
    P.motor.friction_viscous = 0.0
    P.load.mass, P.load.distance, P.load.g = 0.2, 0.1, 0.0
    P.motor.omega0 = 5.0
    acts = [Action(0.01, "load.mass", 0.5, mode=mode)]
    df = simulate(
        P, acts, 0.02, ["load.omega", "energy.external", "energy.stored", "energy.residual"], 1e-3
    )
    j_m = 0.2 * 0.01 + 2.5e-4
    j_p = 0.5 * 0.01 + 2.5e-4
    w_after = df["load.omega"][-1]
    expect = 5.0 if mode == "keep_speed" else 5.0 * j_m / j_p
    assert close(w_after, expect, rtol=1e-9)
    de = 0.5 * j_p * expect**2 - 0.5 * j_m * 25.0
    assert close(df["energy.external"][-1], de, rtol=1e-9)
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6


def test_gearbox_loss_both_directions():
    """V-MECH-004 analogue (rigid): loss = (1-eta)|tau_in w_m| when driving and back-driving."""
    for sign in (1.0, -1.0):
        P = m1()
        P.motor.friction_viscous = 0.0
        P.gearbox.ratio, P.gearbox.efficiency = 4.0, 0.8
        P.load.kind = "constant"
        P.load.torque = sign * 0.1
        P.motor.omega0 = -sign * 20.0  # load torque opposes motion first (driving) ...
        df = simulate(P, [], 0.01, ["motor.omega", "gearbox.p_loss", "energy.residual"], 1e-3)
        w = df["motor.omega"].to_numpy()
        tau_in = -(P.load.torque / 4.0)  # lossless, J_load = J_go = 0
        p_exp = 0.2 * np.abs(tau_in * w) * np.tanh(np.abs(w) / 1e-3)
        assert close(df["gearbox.p_loss"].to_numpy(), p_exp, rtol=1e-6)
        assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6
