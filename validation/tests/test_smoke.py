"""Smoke test: the scientific stack imports on the pinned Python (D-005)."""

import sys


def test_python_version_is_313() -> None:
    assert sys.version_info[:2] == (3, 13)


def test_scientific_stack_imports() -> None:
    import numpy as np
    import polars as pl
    import pyarrow  # noqa: F401
    import scipy.integrate

    # solve_ivp on dy/dt = -y gives y(1) = e^-1
    sol = scipy.integrate.solve_ivp(lambda t, y: -y, (0.0, 1.0), [1.0], rtol=1e-10, atol=1e-12)
    assert abs(sol.y[0, -1] - np.exp(-1.0)) < 1e-8
    assert pl.DataFrame({"a": [1, 2]}).height == 2
