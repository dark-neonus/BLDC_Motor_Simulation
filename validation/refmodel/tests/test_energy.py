"""V-ENER-001 on the reference model's own scenarios (EQ-ENER-01..03), plus V-DET-001."""

from __future__ import annotations

import math

import numpy as np

from refmodel import Action, simulate

from .helpers import m1

ENERGY = ["energy.residual", "energy.in", "energy.stored", "energy.out", "energy.external"]


def _check(df, r_tol=1e-3):
    r = df["energy.residual"].to_numpy()
    assert np.all(np.isfinite(r))
    assert np.max(np.abs(r)) < r_tol, np.max(np.abs(r))


def _nonideal_inverter(P):
    inv = P.inverter
    inv.dead_time, inv.r_on, inv.v_f, inv.t_rise, inv.t_fall = 0.5e-6, 0.02, 0.8, 40e-9, 40e-9
    return P


def scenario_foc_psu_regen():
    P = _nonideal_inverter(m1())
    P.supply.kind = "psu"
    P.supply.i_lim = 3.0
    P.bus.chopper_enabled = True
    P.bus.r_brake, P.bus.v_on, P.bus.v_off = 20.0, 25.0, 24.6
    P.gearbox.ratio, P.gearbox.efficiency, P.gearbox.j_in, P.gearbox.j_out = 4.0, 0.85, 1e-5, 1e-4
    P.load.arm_mass, P.load.arm_length, P.load.mass, P.load.distance = 0.1, 0.2, 0.2, 0.15
    P.motor.cogging_nc, P.motor.cogging = 84, ((0.003, 0.1),)
    P.motor.k_hy, P.motor.k_ed = 1e-4, 1e-7
    P.ctrl.kind = "foc"
    P.ctrl.foc.mode = "velocity"
    P.ctrl.i_max = 4.0
    acts = [Action(0.0, "ctrl.omega_ref", 60.0), Action(0.08, "ctrl.omega_ref", -20.0)]
    return P, acts, 0.15


def scenario_six_step_battery():
    P = _nonideal_inverter(m1())
    P.supply.kind = "battery"
    P.motor.emf_shape = "trapezoidal"
    P.motor.friction_static, P.motor.friction_coulomb = 0.004, 0.003
    P.load.kind = "brake"
    P.load.torque = 0.01
    P.ctrl.kind = "six_step"
    P.ctrl.six_step.speed_pi = True
    P.ctrl.six_step.kp, P.ctrl.six_step.ki = 0.02, 0.5
    acts = [Action(0.0, "ctrl.omega_ref", 30.0), Action(0.09, "ctrl.omega_ref", 5.0)]
    return P, acts, 0.15


def scenario_mit_disturbance():
    P = m1()
    P.ctrl.kind = "mit"
    P.ctrl.f_ctrl = 5e3
    P.ctrl.mit.kp, P.ctrl.mit.kd = 1.0, 0.03
    P.load.kind = "viscous"
    P.load.viscous = 1e-3
    P.load.mass, P.load.distance = 0.2, 0.1
    P.motor.emf_shape = "harmonics"
    P.motor.harmonics = ((5, 0.04, 0.2), (3, 0.1, 0.0))
    acts = [
        Action(0.05, "load.disturbance.impulse", 0.002),
        Action(0.1, "load.mass", 0.4, mode="conserve_momentum"),
        Action(0.12, "ctrl.theta_ref", 0.3),
    ]
    return P, acts, 0.25


def scenario_stick_slip_open_loop():
    P = m1()
    P.motor.friction_static, P.motor.friction_coulomb = 0.01, 0.008
    P.ctrl.kind = "open_loop"
    P.ctrl.f_ctrl = 5e3
    P.ctrl.open_loop.accel = 2000.0
    P.ctrl.open_loop.v0, P.ctrl.open_loop.k_vf = 1.5, 0.03
    acts = [
        Action(0.0, "ctrl.open_loop.omega_e_ref", 150.0),
        Action(0.15, "ctrl.open_loop.omega_e_ref", 0.0),
    ]
    return P, acts, 0.3


def test_v_ener_001_foc_psu_regen():
    P, acts, t_end = scenario_foc_psu_regen()
    df = simulate(P, acts, t_end, [*ENERGY, "bus.chopper_on", "supply.mode", "motor.omega"], 1e-3)
    _check(df)
    assert "BLOCKING" in df["supply.mode"].to_list()


def test_v_ener_001_six_step_battery():
    P, acts, t_end = scenario_six_step_battery()
    df = simulate(P, acts, t_end, [*ENERGY, "motor.omega"], 1e-3)
    _check(df)
    assert df["motor.omega"].max() > 10.0


def test_v_ener_001_mit_disturbance():
    P, acts, t_end = scenario_mit_disturbance()
    df = simulate(P, acts, t_end, [*ENERGY, "load.theta"], 1e-3)
    _check(df)
    assert abs(df["load.theta"][-1] - 0.3) < 0.05


def test_v_ener_001_stick_slip_open_loop():
    P, acts, t_end = scenario_stick_slip_open_loop()
    df = simulate(P, acts, t_end, [*ENERGY, "motor.omega"], 1e-3)
    _check(df)
    assert df["motor.omega"].max() > 5.0
    assert np.any(df["motor.omega"].to_numpy()[:5] == 0.0)  # held by static friction at start


def test_v_det_001_bit_identical_reruns():
    P, acts, _ = scenario_mit_disturbance()
    a = simulate(P, acts, 0.06, ["motor.i_q", "load.theta", *ENERGY], 1e-3)
    b = simulate(P, acts, 0.06, ["motor.i_q", "load.theta", *ENERGY], 1e-3)
    assert a.equals(b)
    assert not math.isnan(a["energy.residual"][-1])
