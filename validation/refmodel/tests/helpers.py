"""Shared fixtures for the reference-model self-tests."""

from __future__ import annotations

import math

import numpy as np

from refmodel import RefParams


def m1() -> RefParams:
    """Default test motor M1 of the validation catalog."""
    return RefParams()


def close(a, b, atol: float = 0.0, rtol: float = 0.0) -> bool:
    """CONVENTIONS section 7 tolerance: |a - b| <= atol + rtol |b|."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return bool(np.all(np.abs(a - b) <= atol + rtol * np.abs(b)))


def crossings(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Upward zero crossings by linear interpolation."""
    idx = np.where((x[:-1] < 0.0) & (x[1:] >= 0.0))[0]
    return t[idx] - x[idx] * (t[idx + 1] - t[idx]) / (x[idx + 1] - x[idx])


def vdq_fixture(v_d: float, v_q: float):
    """Ideal continuous rotor-frame voltage source (test fixture, EQ-CONV-04)."""

    def f(t: float, ctx: dict) -> tuple[float, float]:
        th = ctx["theta_e"]
        c, s = math.cos(th), math.sin(th)
        return v_d * c - v_q * s, v_d * s + v_q * c

    return f
