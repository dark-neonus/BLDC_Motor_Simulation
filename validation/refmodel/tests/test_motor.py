"""V-MOT-* cases (EQ-MOT, EQ-CONV)."""

from __future__ import annotations

import math

import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve

from refmodel import Action, lambda_m_from_kv, simulate
from refmodel import physics as ph
from refmodel.params import kt_from_lambda

from .helpers import close, m1, vdq_fixture


def test_v_mot_001_locked_rotor_step():
    P = m1()
    P.mech.mode = "prescribed"
    P.motor.theta0 = 0.3
    P.ctrl.kind = "ideal_voltage"
    P.ctrl.ideal_voltage = vdq_fixture(0.0, 1.0)
    df = simulate(P, [], 0.02, ["motor.i_q", "motor.i_d"], 1e-4)
    t = df["t"].to_numpy()
    r, ls, v = 1.0, 2.5e-3, 1.0
    ref = v / r * (1 - np.exp(-t * r / ls))
    assert close(df["motor.i_q"].to_numpy(), ref, atol=0.005 * v / r, rtol=1e-6)
    assert np.max(np.abs(df["motor.i_d"].to_numpy())) < 1e-9


def test_v_mot_002_free_spin_steady_speed():
    P = m1()
    v = 5.0
    P.ctrl.kind = "ideal_voltage"
    P.ctrl.ideal_voltage = vdq_fixture(0.0, v)
    df = simulate(P, [], 0.3, ["motor.omega"], 1e-3)
    w_sim = df["motor.omega"][-1]
    p, lam, r, ls, b = 14, 0.03, 1.0, 2.5e-3, 1e-4

    def eqs(x):
        i_d, i_q, w = x
        we = p * w
        return [
            -r * i_d + we * ls * i_q,
            v - r * i_q - we * ls * i_d - we * lam,
            1.5 * p * lam * i_q - b * w,
        ]

    w_ss = fsolve(eqs, [0.0, 0.0, v / (p * lam)], xtol=1e-14)[2]
    assert close(w_sim, w_ss, rtol=5e-3)
    # B -> 0 limit
    assert abs(w_ss - v / (p * lam)) / (v / (p * lam)) < 0.01


def test_v_mot_003_back_emf_constant():
    P = m1()
    P.mech.mode = "prescribed"
    w = 50.0
    P.mech.omega_prescribed = w
    df = simulate(P, [], 0.01, ["motor.e_a", "motor.e_b", "motor.e_c", "motor.i_a"], 1e-4)
    ea, eb, ec = (df[f"motor.e_{x}"].to_numpy() for x in "abc")
    amp = np.sqrt(((ea - eb) ** 2 + (eb - ec) ** 2 + (ec - ea) ** 2) / 1.5)
    ke = math.sqrt(3) * 14 * 0.03
    assert close(amp, ke * w, rtol=1e-6)
    assert np.all(df["motor.i_a"].to_numpy() == 0.0)


def test_v_mot_004_torque_constant():
    P = m1()
    P.mech.mode = "prescribed"
    P.ctrl.kind = "foc"
    P.ctrl.foc.iq_ref = 2.0
    df = simulate(P, [], 0.02, ["motor.torque_em", "motor.i_q"], 1e-3)
    iq = df["motor.i_q"].to_numpy()[-5:]
    te = df["motor.torque_em"].to_numpy()[-5:]
    assert close(te, 1.5 * 14 * 0.03 * iq, rtol=1e-9)
    assert close(iq, 2.0, rtol=1e-3)


def test_v_mot_005_stationary_vs_dq_reference():
    """EQ-MOT-06: same sinusoidal alpha-beta voltages into both models (salient motor)."""
    P = m1()
    P.motor.l_d, P.motor.l_q = 2.0e-3, 3.0e-3
    p, lam, r, ld, lq, j, b = 14, 0.03, 1.0, 2.0e-3, 3.0e-3, 2.5e-4, 1e-4
    vm, ws = 3.0, 60.0

    def vab(t):
        return vm * math.cos(ws * t), vm * math.sin(ws * t)

    P.ctrl.kind = "ideal_voltage"
    P.ctrl.ideal_voltage = lambda t, ctx: vab(t)
    P.sim.rtol, P.sim.atol = 1e-11, 1e-13
    t_end = 0.2
    df = simulate(P, [], t_end, ["motor.i_d", "motor.i_q", "motor.omega", "motor.torque_em"], 1e-3)

    def rhs(t, x):
        i_d, i_q, w, th = x
        the = p * th
        va, vb = vab(t)
        v_d, v_q = ph.park(va, vb, the)
        we = p * w
        te = 1.5 * p * (lam * i_q + (ld - lq) * i_d * i_q)
        return [
            (v_d - r * i_d + we * lq * i_q) / ld,
            (v_q - r * i_q - we * ld * i_d - we * lam) / lq,
            (te - b * w) / j,
            w,
        ]

    t = df["t"].to_numpy()
    sol = solve_ivp(
        rhs, (0, t_end), [0, 0, 0, 0], method="DOP853", rtol=1e-12, atol=1e-14, t_eval=t
    )
    i_d, i_q, w = sol.y[0], sol.y[1], sol.y[2]
    te = 1.5 * p * (lam * i_q + (ld - lq) * i_d * i_q)
    for col, ref in (
        ("motor.i_d", i_d),
        ("motor.i_q", i_q),
        ("motor.omega", w),
        ("motor.torque_em", te),
    ):
        assert close(df[col].to_numpy(), ref, atol=1e-9, rtol=1e-6), col


def test_v_mot_006_kv_conversion():
    lam = lambda_m_from_kv(100.0, 14)
    assert close(lam, 3.9381e-3, rtol=1e-5)
    assert close(kt_from_lambda(lam, 14), 82.699e-3, rtol=1e-5)


def test_v_mot_007_cogging_period_and_energy():
    P = m1()
    P.motor.friction_viscous = 0.0
    P.motor.cogging_nc = 84
    P.motor.cogging = ((0.01, 0.2), (0.003, -0.5))
    P.motor.theta0 = 0.01
    P.sim.rtol = 1e-12
    df = simulate(P, [], 0.05, ["motor.torque_cog", "energy.residual", "energy.stored"], 1e-4)
    # period 2 pi / N_c and zero mean of T_cog (EQ-MOT-08)
    th = np.linspace(0.0, 2 * math.pi / 84, 4001)[:-1]
    tc = np.array([ph.cogging(84, P.motor.cogging, x)[0] for x in th])
    tc2 = np.array([ph.cogging(84, P.motor.cogging, x + 2 * math.pi / 84)[0] for x in th])
    assert close(tc, tc2, atol=1e-9)
    assert abs(tc.mean()) < 1e-9
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6


def test_v_mot_008_iron_loss_spin_down():
    P = m1()
    P.motor.friction_viscous = 0.0
    P.motor.k_hy, P.motor.k_ed = 2e-4, 1e-6
    P.motor.omega0 = 100.0
    df = simulate(P, [], 1.0, ["motor.omega", "energy.residual"], 1e-2)

    def rhs(t, x):
        return [ph.iron_loss_torque(2e-4, 1e-6, 14, x[0], 0.01) / 2.5e-4]

    t = df["t"].to_numpy()
    sol = solve_ivp(rhs, (0, 1.0), [100.0], method="DOP853", rtol=1e-12, atol=1e-14, t_eval=t)
    assert close(df["motor.omega"].to_numpy(), sol.y[0], rtol=1e-4, atol=1e-9)
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6


def _six_step_ripple(shape) -> float:
    """Torque over one electrical turn with ideal 120-degree block currents (EQ-MOT-07)."""
    p, lam, i0 = 14, 0.03, 1.0
    ts = []
    for th in np.linspace(0, 2 * math.pi, 3601)[:-1]:
        code = 4 * ph.hall_bits(th)[0] + 2 * ph.hall_bits(th)[1] + ph.hall_bits(th)[2]
        _, hi, lo = ph.HALL_TABLE[code]
        i = [0.0, 0.0, 0.0]
        i[hi], i[lo] = i0, -i0
        ial, ibe = ph.clarke(*i)
        i_d, i_q = ph.park(ial, ibe, th)
        ts.append(
            ph.torque_em(
                p,
                lam,
                2.5e-3,
                2.5e-3,
                i_d,
                i_q,
                tuple(i),
                shape.k3(th),
                th,
                shape.kind == "sinusoidal",
            )
        )
    ts = np.array(ts)
    return (ts.max() - ts.min()) / ts.max()


def test_v_mot_012_six_step_ripple_sinusoidal():
    rip = _six_step_ripple(ph.EmfShape("sinusoidal", 0.0, ()))
    assert abs(rip - (1 - math.cos(math.pi / 6))) <= 0.005


def test_v_mot_013_six_step_ripple_trapezoidal():
    rip = _six_step_ripple(ph.EmfShape("trapezoidal", 2 * math.pi / 3, ()))
    assert rip <= 0.005


def test_emf_shape_normalisation():
    """EQ-MOT-03: fundamental of k is -sin; Phi' = k; b1(120 deg) = 12/pi^2."""
    assert close(ph.trap_b1(2 * math.pi / 3), 12 / math.pi**2, rtol=1e-12)
    for shape in (
        ph.EmfShape("trapezoidal", 2 * math.pi / 3, ()),
        ph.EmfShape("harmonics", 0.0, ((5, 0.05, 0.3), (7, 0.02, 0.0))),
    ):
        th = np.linspace(0, 2 * math.pi, 20001)[:-1]
        k = np.array([shape.k(x) for x in th])
        assert close(2 * np.mean(k * np.sin(th)), -1.0, atol=1e-6)
        assert abs(np.mean(k * np.cos(th))) < 1e-9
        phi = np.array([shape.phi(x) for x in th])
        assert abs(phi.mean()) < 1e-9
        h = 1e-6
        for x in (0.1, 1.0, 2.5, 4.0):
            assert close((shape.phi(x + h) - shape.phi(x - h)) / (2 * h), shape.k(x), atol=1e-6)


def test_open_loop_vf_runs_synchronous():
    """EQ-CTRL-06 smoke: rotor follows the open-loop angle at the commanded speed."""
    P = m1()
    P.ctrl.kind = "open_loop"
    P.ctrl.f_ctrl = 5e3
    P.ctrl.open_loop.omega_e_ref = 100.0
    P.ctrl.open_loop.accel = 1000.0
    P.ctrl.open_loop.v0 = 1.0
    P.ctrl.open_loop.k_vf = 0.03
    df = simulate(
        P,
        [Action(0.0, "ctrl.open_loop.omega_e_ref", 100.0)],
        0.4,
        ["motor.omega_e", "energy.residual"],
        1e-3,
    )
    assert abs(df["motor.omega_e"][-1] - 100.0) < 1.0
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-3
