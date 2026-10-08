"""V-INV-* cases (EQ-INV, averaged model)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from refmodel import physics as ph
from refmodel import simulate

from .helpers import close, m1

# switching vectors V1..V6 (a, b, c high?) at 0, 60, ..., 300 degrees
VECS = [(1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (1, 0, 1)]


def _sector_svpwm(v_al: float, v_be: float, vbus: float) -> tuple[float, ...]:
    """Classic sector-based SVPWM with the zero vectors split equally [HolmesLipo2003]."""
    mag = math.hypot(v_al, v_be)
    ang = math.atan2(v_be, v_al) % (2 * math.pi)
    n = int(ang // (math.pi / 3)) % 6
    th = ang - n * math.pi / 3
    t1 = math.sqrt(3) * mag / vbus * math.sin(math.pi / 3 - th)
    t2 = math.sqrt(3) * mag / vbus * math.sin(th)
    t0 = 1 - t1 - t2
    return tuple(t0 / 2 + t1 * VECS[n][x] + t2 * VECS[(n + 1) % 6][x] for x in range(3))


def test_v_inv_002_svpwm_vs_sector():
    vbus, amp = 24.0, 10.0
    for k in range(36):
        th = k * 2 * math.pi / 36 + 0.01
        v_al, v_be = amp * math.cos(th), amp * math.sin(th)
        d = ph.svpwm(ph.inv_clarke(v_al, v_be), vbus)
        assert close(d, _sector_svpwm(v_al, v_be, vbus), atol=1e-12)
    # worked example of EQ-INV-05
    d = ph.svpwm(ph.inv_clarke(10 * math.cos(0.3), 10 * math.sin(0.3)), 24.0)
    assert close(d, (0.8519, 0.3614, 0.1481), atol=5e-5)


@pytest.mark.parametrize("scheme,expected", [("svpwm", 24.0 / math.sqrt(3)), ("spwm", 12.0)])
def test_v_inv_003_max_linear_amplitude(scheme, expected):
    vbus = 24.0
    fn = ph.svpwm if scheme == "svpwm" else ph.spwm
    angles = np.concatenate([np.linspace(0, 2 * math.pi, 7201), np.arange(12) * math.pi / 6])
    worst = 0.0
    for th in angles:
        d = fn(ph.inv_clarke(math.cos(th), math.sin(th)), vbus)  # unit amplitude
        worst = max(worst, max(d) - 0.5, 0.5 - min(d))
    a_lin = 0.5 / worst
    assert close(a_lin, expected, rtol=1e-12)


def _inv_params(P):
    inv = P.inverter
    inv.dead_time, inv.r_on, inv.v_f = 0.5e-6, 0.02, 0.8
    inv.t_rise, inv.t_fall, inv.i_eps = 50e-9, 50e-9, 0.02
    return P


SIGS = [
    "bus.v",
    "inverter.i_dc",
    "inverter.i_sw",
    "inverter.p_loss",
    *[f"inverter.v_{x}" for x in "abc"],
    *[f"motor.i_{x}" for x in "abc"],
    *[f"inverter.duty_{x}" for x in "abc"],
    *[f"inverter.leg_{x}" for x in "abc"],
]


def _p_inv_formula(df, inv) -> np.ndarray:
    """EQ-INV-09 component formulas (conduction + diode + switching), evaluated per leg."""
    vb = df["bus.v"].to_numpy()
    tot = vb * df["inverter.i_sw"].to_numpy()
    kdt = inv.dead_time * inv.f_pwm
    for x in "abc":
        i = df[f"motor.i_{x}"].to_numpy()
        d = df[f"inverter.duty_{x}"].to_numpy()
        leg = df[f"inverter.leg_{x}"].to_list()
        sg = np.tanh(i / inv.i_eps)
        for k, lg in enumerate(leg):
            if lg is None:  # averaged PWM leg: R_on i^2 outside dead time, diode inside
                tot[k] += inv.r_on * i[k] ** 2 + sg[k] * i[k] * kdt * (
                    2 * inv.v_f - 2 * inv.r_on * abs(i[k])
                )
            elif lg == "H":  # six-step H/O: MOSFET for d, diode for 1-d
                tot[k] += d[k] * inv.r_on * i[k] ** 2 + (1 - d[k]) * inv.v_f * i[k] * sg[k]
            elif lg == "L":
                tot[k] += inv.r_on * i[k] ** 2
            else:  # Off leg: diode conducts or floating (i = 0)
                tot[k] += inv.v_f * abs(i[k])
    return tot


@pytest.mark.parametrize("ctrl", ["foc", "six_step"])
def test_v_inv_007_power_check(ctrl):
    P = _inv_params(m1())
    P.ctrl.kind = ctrl
    P.ctrl.f_ctrl = 10e3
    if ctrl == "foc":
        P.ctrl.foc.iq_ref = 1.5
        P.ctrl.foc.id_ref = -0.3
    else:
        P.ctrl.six_step.duty = 0.4
    df = simulate(P, [], 0.05, SIGS, 1e-4)
    vb = df["bus.v"].to_numpy()
    lhs = vb * (df["inverter.i_dc"].to_numpy() + df["inverter.i_sw"].to_numpy())
    p_motor = sum(df[f"inverter.v_{x}"].to_numpy() * df[f"motor.i_{x}"].to_numpy() for x in "abc")
    p_inv = df["inverter.p_loss"].to_numpy()
    scale = np.max(np.abs(lhs))
    assert close(lhs, p_motor + p_inv, rtol=1e-6, atol=1e-9 * scale)
    assert close(
        p_inv, _p_inv_formula(df, P.inverter), rtol=1e-6, atol=1e-6 * np.max(np.abs(p_inv))
    )


def test_dead_time_voltage_error_sign():
    """EQ-INV-03: average terminal-voltage error -sigma(i) t_d/T (V + 2V_f - 2 R_on |i|)."""
    P = _inv_params(m1())
    P.mech.mode = "prescribed"
    P.ctrl.kind = "foc"
    P.ctrl.foc.iq_ref = 2.0
    df = simulate(P, [], 0.02, SIGS, 1e-3)
    inv = P.inverter
    kdt = inv.dead_time * inv.f_pwm
    for x in "abc":
        i = df[f"motor.i_{x}"].to_numpy()
        err = df[f"inverter.v_{x}"].to_numpy() - (
            df[f"inverter.duty_{x}"].to_numpy() * 24.0 - inv.r_on * i
        )
        ref = -np.tanh(i / inv.i_eps) * kdt * (24.0 + 2 * inv.v_f - 2 * inv.r_on * np.abs(i))
        assert close(err, ref, atol=1e-12)


def test_six_step_runs_with_valid_halls():
    """EQ-CTRL-05 + EQ-SENS-02: six-step accelerates, Hall codes valid, open phase floats."""
    P = _inv_params(m1())
    P.ctrl.kind = "six_step"
    P.ctrl.six_step.duty = 0.3
    df = simulate(
        P, [], 0.1, ["motor.omega", "sensors.hall.code", "motor.open_phase", "motor.i_c"], 1e-4
    )
    assert df["motor.omega"][-1] > 5.0
    assert set(df["sensors.hall.code"].to_list()) <= {1, 2, 3, 4, 5, 6}
    op = df["motor.open_phase"].to_list()
    assert "c" in op and "a" in op
    ic = df["motor.i_c"].to_numpy()
    sel = np.array([o == "c" for o in op])
    assert np.max(np.abs(ic[sel])) < 1e-9
