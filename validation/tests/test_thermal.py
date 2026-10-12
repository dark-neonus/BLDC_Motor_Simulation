"""V-THERM-004 (P06.T08): the hold-heats-motor scenario.

Holding still (ω = 0), the copper loss is the only heat, so the winding trajectory is the
linear three-node network (EQ-THERM-01) driven by that loss. The 6020-class preset's
estimated network is restated here independently of the Rust code on purpose.
"""

import subprocess
from pathlib import Path

import numpy as np
import polars as pl
from scipy.linalg import expm

REPO = Path(__file__).resolve().parents[2]
SCENARIO = REPO / "presets" / "scenarios" / "hold-heats-motor.yaml"

# presets/motors/6020-outrunner.yaml (thermal block).
R_WS, R_SH, R_HA = 0.5, 0.3, 4.91583
C_W, C_S, C_H = 27.027, 75.348, 168.48
T_AMB = 298.15


def test_v_therm_004_hold_heats_motor(bldc_sim: Path, tmp_path: Path) -> None:
    run = subprocess.run(
        [str(bldc_sim), "run-scenario", str(SCENARIO), "--out", str(tmp_path)],
        capture_output=True,
        text=True,
    )
    assert run.returncode == 0, run.stdout + run.stderr
    df = pl.read_parquet(tmp_path / "signals.parquet")
    t = df["t"].to_numpy()
    tw = df["thermal.t_winding"].to_numpy()
    p_cu = df["motor.p_cu"].to_numpy()
    # The arm sags for a few ms while the current builds, then load friction holds it.
    held = t > 0.5
    assert np.all(df["load.omega"].to_numpy()[held] == 0.0), "the arm is held still"
    assert np.all(np.diff(tw[t > 0.5]) >= -1e-9), "the winding only warms while holding"

    # λ(T) changes the magnet flux at fixed ψ; energy.md neglects that work. Check it is
    # far below the balance tolerance over the heating run (P06.T10).
    assert np.max(np.abs(df["energy.residual"].to_numpy())) < 1e-6

    # Copper loss rises a few % with R(T); drive the linear network with its mean.
    p = float(np.mean(p_cu[t > 0.5]))
    a = np.array(
        [
            [-1 / (R_WS * C_W), 1 / (R_WS * C_W), 0.0],
            [1 / (R_WS * C_S), -(1 / R_WS + 1 / R_SH) / C_S, 1 / (R_SH * C_S)],
            [0.0, 1 / (R_SH * C_H), -(1 / R_SH + 1 / R_HA) / C_H],
        ]
    )
    b = np.array([p / C_W, 0.0, 0.0])
    ss = -np.linalg.solve(a, b)
    for tk in (5.0, 10.0, 20.0, 30.0):
        exact = ss - expm(a * tk) @ ss
        got = np.interp(tk, t, tw) - T_AMB
        # 5 % of the rise: the mean-loss approximation (R(T) changes ~3 % over the run)
        # and the controller's current ripple.
        assert abs(got - exact[0]) <= 0.05 * exact[0], f"t = {tk}: {got} vs {exact[0]}"
