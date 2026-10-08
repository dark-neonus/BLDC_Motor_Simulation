"""Skeleton validation (P01.T09): analytic checks of the CLI output.

Skeleton motor (P01.T01): p = 14, R = 1.0 Ω, L = 2.5 mH, λ = 0.03 Wb, J = 2.5e-4, B = 1e-4.
These constants are restated here independently of the Rust code on purpose.
"""

import numpy as np
from scipy.optimize import brentq

P, R, L, LAMBDA, B = 14.0, 1.0, 2.5e-3, 0.03, 1e-4
KT = 1.5 * P * LAMBDA


def test_v_skel_001_locked_rotor_rl(run_scenario) -> None:
    """Locked rotor, v_q = 1 V step: i_q(t) = (V/R)(1 − e^{−t R/L})."""
    df = run_scenario("skeleton_locked_rotor")
    t = df["t"].to_numpy()
    iq = df["motor.i_q"].to_numpy()
    expected = 1.0 / R * (1.0 - np.exp(-t * R / L))
    # Plant dt = τ/10000 → RK4 error ≪ 1e-9 A; atol = 0.5 % of the final 1 A current
    # (the V-SKEL criterion), rtol 1e-6 covers float round-off near the plateau.
    np.testing.assert_allclose(iq, expected, atol=0.005 * (1.0 / R), rtol=1e-6)
    assert np.all(df["motor.omega"].to_numpy() == 0.0)


def test_v_skel_002_no_load_speed(run_scenario) -> None:
    """Free spin, v_d = 0, v_q = 6 V: steady speed solves the model with derivatives = 0."""
    v = 6.0

    def residual(w: float) -> float:
        we = P * w
        iq = B * w / KT
        i_d = we * L * iq / R
        return R * iq + we * L * i_d + we * LAMBDA - v

    expected = brentq(residual, 0.0, 2 * v / (P * LAMBDA))
    df = run_scenario("skeleton_no_load")
    final = df["motor.omega"].to_numpy()[-1]
    # 0.5 s ≫ the ~1 ms electromechanical time constant: transient fully decayed.
    # rtol 0.5 % is the V-SKEL criterion; atol 1e-9 rad/s is a round-off floor.
    np.testing.assert_allclose(final, expected, atol=1e-9, rtol=0.005)
