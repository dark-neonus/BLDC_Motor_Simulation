"""Hybrid simulation loop (EQ-NUM-02/05): solve_ivp between discrete events, state events
located by solve_ivp ``events``, energy terms integrated as states (EQ-ENER-01..03)."""

from __future__ import annotations

import copy
import math
from collections.abc import Callable
from typing import Any

import numpy as np
import polars as pl
from scipy.integrate import solve_ivp

from . import physics as ph
from .control import Controller
from .params import Action, RefParams
from .plant import (
    E_EXT,
    E_IN,
    E_THR,
    L_FR,
    LOSS_IDX,
    N_STATES,
    OM,
    PSA,
    PSB,
    SOC,
    STICK,
    TH,
    THS,
    TS,
    TW,
    V1,
    VB,
    Disc,
    Plant,
)

EPS_T = 1e-12
TICKING = ("foc", "mit", "six_step", "open_loop")
CTRL_SIGNALS = (
    "ctrl.omega_ref",
    "ctrl.theta_ref",
    "ctrl.torque_ref",
    "ctrl.foc.id_ref",
    "ctrl.foc.iq_ref",
    "ctrl.foc.v_d",
    "ctrl.foc.v_q",
    "ctrl.foc.saturated",
    "ctrl.six_step.sector",
    "ctrl.open_loop.theta_ref",
)
ENERGY_SIGNALS = (
    "energy.in",
    "energy.out",
    "energy.stored",
    "energy.external",
    "energy.residual",
    "energy.ok",
    *(f"energy.loss.{k}" for k in LOSS_IDX),
)
LOAD_INERTIA_PATHS = ("load.mass", "load.distance", "load.arm_mass", "load.arm_length")
U_A = ((1.0, 0.0), (-0.5, 0.5 * math.sqrt(3.0)), (-0.5, -0.5 * math.sqrt(3.0)))


class ZenoError(RuntimeError):
    pass


class _Sim:
    def __init__(self, params: RefParams, actions: list[Action]):
        self.P = copy.deepcopy(params)
        self.P.ctrl.ideal_voltage = params.ctrl.ideal_voltage
        self.plant = Plant(self.P)
        self.d = Disc()
        self.actions = sorted(actions, key=lambda a: a.t)
        self.ai = 0
        self.dist: list[tuple[float, float]] = []  # (t_end, torque)
        P = self.P
        self.ctrl = Controller(P, self.plant.j_eq) if P.ctrl.kind in TICKING else None
        self.tick_ns = round(1e9 / P.ctrl.f_ctrl) if self.ctrl else 0
        self.k_tick = 0
        self.y = self._initial_state()
        self._cache_key: tuple | None = None
        self._cache_sig: dict = {}
        self.fired_at: tuple[float, set[str]] = (-1.0, set())

    # ------------------------------------------------------------ init
    def _initial_state(self) -> np.ndarray:
        P, pl_ = self.P, self.plant
        y = np.zeros(N_STATES)
        th0 = P.motor.theta0 if P.load.theta0 is None else pl_.N * P.load.theta0
        om0 = P.mech.omega_prescribed if P.mech.mode == "prescribed" else P.motor.omega0
        y[TH], y[OM] = th0, om0
        t0 = P.thermal.t_init if P.thermal.t_init is not None else P.thermal.t_amb
        y[TW] = y[TS] = y[THS] = t0
        sup = P.supply
        y[SOC] = sup.soc0
        y[V1] = sup.v_rc0 if sup.kind == "battery" else 0.0
        if P.bus.v0 is not None:
            y[VB] = P.bus.v0
        elif sup.kind == "battery":
            y[VB] = pl_.ocv(sup.soc0) - sup.v_rc0
        else:
            y[VB] = sup.v_set
        _, lam = pl_.temps(y.tolist())
        y[PSA], y[PSB] = pl_.flux(P.motor.i_alpha0, P.motor.i_beta0, P.motor.pole_pairs * th0, lam)
        return y

    # ------------------------------------------------------------ helpers
    def sig(self, t: float, y: np.ndarray) -> dict:
        key = (t, y.tobytes())
        if key != self._cache_key:
            self._cache_key = key
            self._cache_sig = self.plant.eval(t, y, self.d, full=True)[1]
        return self._cache_sig

    def _invalidate(self) -> None:
        self._cache_key = None

    def _project(self, y: np.ndarray, zero_leg: int | None) -> None:
        """Rebuild psi from currents with i_x = 0 (one floating leg) or i = 0 (all)."""
        pl_ = self.plant
        _, lam = pl_.temps(y.tolist())
        th_e = self.P.motor.pole_pairs * y[TH]
        if zero_leg is None:
            ia = ib = 0.0
        else:
            ia, ib = pl_.currents(y[PSA], y[PSB], th_e, lam)
            u = U_A[zero_leg]
            dot = u[0] * ia + u[1] * ib
            ia, ib = ia - dot * u[0], ib - dot * u[1]
        y[PSA], y[PSB] = pl_.flux(ia, ib, th_e, lam)
        self._invalidate()

    def init_leg_modes(self, y: np.ndarray, prev_cmd: list[str]) -> None:
        """EQ-INV-01 / EQ-MOT-11: choose diode / floating sub-modes of Off legs."""
        d = self.d
        if self.plant.fixture:
            return
        off = [x for x in range(3) if d.leg_cmd[x] == "O"]
        for x in range(3):
            if d.leg_cmd[x] != "O":
                d.off_sub[x] = None
        if d.zero_current:
            for x in off:
                d.off_sub[x] = "float"
            if len(off) < 2:
                d.zero_current = False
            self._invalidate()
            return
        pl_ = self.plant
        _, lam = pl_.temps(y.tolist())
        ia, ib = pl_.currents(y[PSA], y[PSB], self.P.motor.pole_pairs * y[TH], lam)
        i_abc = ph.inv_clarke(ia, ib)
        for x in off:
            if prev_cmd[x] == "O" and d.off_sub[x] is not None:
                continue
            ix = i_abc[x]
            d.off_sub[x] = "dlo" if ix > 1e-9 else ("dhi" if ix < -1e-9 else "float")
        fl = [x for x in off if d.off_sub[x] == "float"]
        if len(fl) >= 2:
            d.zero_current = True
            for x in off:
                d.off_sub[x] = "float"
            self._project(y, None)
        elif len(fl) == 1:
            self._project(y, fl[0])
        self._invalidate()

    def settle(self, t: float, y: np.ndarray) -> None:
        """Make discrete modes consistent with the state after discontinuous changes."""
        P, d, pl_ = self.P, self.d, self.plant
        self._invalidate()
        # floating terminal bounds (EQ-MOT-12)
        if not d.zero_current and not pl_.fixture:
            for x in range(3):
                if d.leg_cmd[x] == "O" and d.off_sub[x] == "float":
                    s = self.sig(t, y)
                    vt = s[f"inverter.v_{'abc'[x]}"]
                    tol = 1e-9 * max(1.0, abs(s["bus.v"]))
                    if vt > s["bus.v"] + P.inverter.v_f + tol:
                        d.off_sub[x] = "dhi"
                    elif vt < -P.inverter.v_f - tol:
                        d.off_sub[x] = "dlo"
                    self._invalidate()
        # friction (EQ-MECH-05)
        if P.mech.mode == "free" and pl_.fric_enabled:
            if y[OM] != 0.0:
                if d.fric not in (1, -1):
                    d.fric = 1 if y[OM] > 0 else -1
            elif d.fric in (0, STICK):
                d.fric = STICK
                self._invalidate()
                ta = self.sig(t, y)["_t_a"]
                if abs(ta) > pl_.ts_eq:
                    d.fric = 1 if ta > 0 else -1
        elif not pl_.fric_enabled:
            d.fric = 0
        # PSU mode (EQ-SUP-03)
        sup = P.supply
        if sup.kind == "psu":
            # modes are only re-assigned when clearly past a boundary; a located event
            # leaves the state within round-off of it and its handler has the last word
            v = y[VB]
            tol = 1e-9 * max(1.0, abs(sup.v_set))
            v_cc = sup.v_set - sup.r_o * sup.i_lim
            if v > sup.v_set + tol:
                d.psu_mode = "BLOCKING"
            elif v < v_cc - tol:
                d.psu_mode = "CC"
            elif v_cc + tol < v < sup.v_set - tol:
                d.psu_mode = "CV"
        # chopper hysteresis (EQ-SUP-05)
        if P.bus.chopper_enabled:
            v = pl_.v_bus(y.tolist())
            tol = 1e-9 * max(1.0, abs(P.bus.v_on))
            if not d.chopper_on and v > P.bus.v_on + tol:
                d.chopper_on = True
            elif d.chopper_on and v < P.bus.v_off - tol:
                d.chopper_on = False
        self._invalidate()

    # ------------------------------------------------------------ events
    def make_events(self) -> list[tuple[Callable, int, str]]:
        P, d, pl_ = self.P, self.d, self.plant
        evs: list[tuple[Callable, int, str]] = []
        if P.mech.mode == "free" and pl_.fric_enabled:
            if d.fric in (1, -1):
                s = d.fric
                evs.append((lambda t, y, s=s: s * y[OM], -1, "fric_zero"))
            elif d.fric == STICK:
                ts = pl_.ts_eq
                evs.append((lambda t, y: abs(self.sig(t, y)["_t_a"]) - ts, 1, "fric_break"))
        sup = P.supply
        if sup.kind == "psu":
            v_cc = sup.v_set - sup.r_o * sup.i_lim
            if d.psu_mode == "CV":
                evs.append((lambda t, y: y[VB] - sup.v_set, 1, "psu_BLOCKING"))
                evs.append((lambda t, y: y[VB] - v_cc, -1, "psu_CC"))
            elif d.psu_mode == "CC":
                evs.append((lambda t, y: y[VB] - v_cc, 1, "psu_CV"))
            else:
                evs.append((lambda t, y: y[VB] - sup.v_set, -1, "psu_CV"))
        if P.bus.chopper_enabled and sup.kind != "ideal":
            if d.chopper_on:
                evs.append((lambda t, y: y[VB] - P.bus.v_off, -1, "chop_off"))
            else:
                evs.append((lambda t, y: y[VB] - P.bus.v_on, 1, "chop_on"))
        if not pl_.fixture and not d.zero_current:
            vf = P.inverter.v_f
            for x in range(3):
                if d.leg_cmd[x] != "O":
                    continue
                sub = d.off_sub[x]
                ni = f"motor.i_{'abc'[x]}"
                nv = f"inverter.v_{'abc'[x]}"
                if sub == "dlo":
                    evs.append((lambda t, y, ni=ni: self.sig(t, y)[ni], -1, f"leg_zero_{x}"))
                elif sub == "dhi":
                    evs.append((lambda t, y, ni=ni: self.sig(t, y)[ni], 1, f"leg_zero_{x}"))
                else:
                    evs.append(
                        (
                            lambda t, y, nv=nv: self.sig(t, y)[nv] - self.sig(t, y)["bus.v"] - vf,
                            1,
                            f"leg_hi_{x}",
                        )
                    )
                    evs.append((lambda t, y, nv=nv: self.sig(t, y)[nv] + vf, -1, f"leg_lo_{x}"))
        return evs

    def handle(self, tag: str, t: float, y: np.ndarray) -> None:
        d, pl_ = self.d, self.plant
        self._invalidate()
        if tag == "fric_zero":
            # EQ-MECH-05 slip -> stick: book remaining kinetic energy as friction loss
            y[L_FR] += 0.5 * pl_.j_eq * y[OM] ** 2
            y[OM] = 0.0
            d.fric = STICK
            self._invalidate()
            ta = self.sig(t, y)["_t_a"]
            if abs(ta) > pl_.ts_eq:
                d.fric = 1 if ta > 0 else -1
        elif tag == "fric_break":
            ta = self.sig(t, y)["_t_a"]
            d.fric = 1 if ta > 0 else -1
        elif tag.startswith("psu_"):
            d.psu_mode = tag[4:]
        elif tag == "chop_on":
            d.chopper_on = True
        elif tag == "chop_off":
            d.chopper_on = False
        elif tag.startswith("leg_zero_"):
            x = int(tag[-1])
            others_float = [
                j for j in range(3) if j != x and d.leg_cmd[j] == "O" and d.off_sub[j] == "float"
            ]
            if others_float:
                d.zero_current = True
                for j in range(3):
                    if d.leg_cmd[j] == "O":
                        d.off_sub[j] = "float"
                self._project(y, None)
            else:
                d.off_sub[x] = "float"
                self._project(y, x)
        elif tag.startswith("leg_hi_"):
            d.off_sub[int(tag[-1])] = "dhi"
        elif tag.startswith("leg_lo_"):
            d.off_sub[int(tag[-1])] = "dlo"
        self._invalidate()

    # ------------------------------------------------------------ discrete updates
    def tick_time(self, k: int) -> float:
        return k * self.tick_ns * 1e-9

    def fire_discrete(self, t: float, y: np.ndarray) -> None:
        d = self.d
        # 1. controller tick (sensors bypass -> controller -> modulator -> inverter)
        if self.ctrl is not None and abs(self.tick_time(self.k_tick) - t) < EPS_T:
            s = self.sig(t, y)
            meas = {
                "theta_e": s["motor.theta_e"],
                "omega_e": s["motor.omega_e"],
                "theta_m": s["motor.theta"],
                "omega_m": s["motor.omega"],
                "theta_l": s["load.theta"],
                "omega_l": s["load.omega"],
                "i_a": s["motor.i_a"],
                "i_b": s["motor.i_b"],
                "v_bus": s["bus.v"],
            }
            out = self.ctrl.tick(meas)
            prev = list(d.leg_cmd)
            d.leg_cmd = list(out["leg_cmd"])
            d.duty = list(out["duty"])
            d.ctrl_sig = dict(out["sig"])
            self.init_leg_modes(y, prev)
            self.k_tick += 1
        # 2. queued commands  SPEC-AMBIGUITY: Q-17 (applied after the tick at the same instant)
        while self.ai < len(self.actions) and self.actions[self.ai].t <= t + EPS_T:
            self.apply_action(self.actions[self.ai], t, y)
            self.ai += 1
        ended = [e for e in self.dist if e[0] <= t + EPS_T]
        if ended:
            self.dist = [e for e in self.dist if e[0] > t + EPS_T]
            d.t_dist = sum(e[1] for e in self.dist)
        self._invalidate()

    def apply_action(self, a: Action, t: float, y: np.ndarray) -> None:
        P, pl_, d = self.P, self.plant, self.d
        if a.path in ("load.disturbance.impulse", "load.disturbance.torque"):
            if a.path.endswith("impulse"):  # EQ-MECH-08
                dt = max(1e-3, 5.0 * P.sim.dt_max)
                torque = float(a.value) / dt
            else:
                if a.duration is None:
                    raise ValueError("load.disturbance.torque needs a duration")
                dt, torque = a.duration, float(a.value)
            self.dist.append((t + dt, torque))
            d.t_dist = sum(e[1] for e in self.dist)
            return
        if a.path in LOAD_INERTIA_PATHS:  # EQ-MECH-09
            ld = P.load
            th_l = y[TH] / pl_.N
            om_l_m = y[OM] / pl_.N
            j_m = pl_.j_tot_load()
            e_m = 0.5 * pl_.j_eq * y[OM] ** 2 + ld.g * pl_.m_eff * (1.0 - math.cos(th_l))
            setattr(ld, a.path.split(".")[1], float(a.value))
            pl_.refresh()
            j_p = pl_.j_tot_load()
            om_l_p = om_l_m * j_m / j_p if a.mode == "conserve_momentum" else om_l_m
            if d.fric != STICK:
                y[OM] = om_l_p * pl_.N
            e_p = 0.5 * pl_.j_eq * y[OM] ** 2 + ld.g * pl_.m_eff * (1.0 - math.cos(th_l))
            y[E_EXT] += e_p - e_m
            self._invalidate()
            return
        obj: Any = P
        parts = a.path.split(".")
        for part in parts[:-1]:
            obj = getattr(obj, part)
        if not hasattr(obj, parts[-1]):
            raise KeyError(f"unknown parameter path {a.path!r}")
        setattr(obj, parts[-1], a.value)
        pl_.refresh()
        self._invalidate()

    # ------------------------------------------------------------ run
    def next_break(self, t: float, t_end: float) -> float:
        cands = [t_end]
        if self.ctrl is not None:
            cands.append(self.tick_time(self.k_tick))
        if self.ai < len(self.actions):
            cands.append(self.actions[self.ai].t)
        cands += [e[0] for e in self.dist]
        return (
            min(c for c in cands if c > t + EPS_T) if any(c > t + EPS_T for c in cands) else t_end
        )

    def run(self, t_end: float, record: list[str], record_dt: float) -> pl.DataFrame:
        P = self.P
        unknown = [
            s
            for s in record
            if s != "t"
            and s not in CTRL_SIGNALS
            and s not in ENERGY_SIGNALS
            and s not in self.plant.eval(0.0, self.y, self.d, full=True)[1]
        ]
        if unknown:
            raise KeyError(f"unsupported signals: {unknown}")
        n_rec = int(math.floor(t_end / record_dt + 1e-9)) + 1
        rec_t = [j * record_dt for j in range(n_rec)]
        self.ri = 0
        self.rows: dict[str, list] = {k: [] for k in ["t", *record]}
        self.record = record
        # dense-output interpolation over very long steps loses accuracy: cap h at record_dt
        t = 0.0
        y = self.y
        # initial discrete state
        self.d.leg_cmd = ["O", "O", "O"]
        self.init_leg_modes(y, ["", "", ""])
        self.settle(t, y)
        self.e_st0 = self.sig(t, y)["_e_st"]
        zeno = 0
        while True:
            if t < t_end - EPS_T:
                self.fire_discrete(t, y)
            self.settle(t, y)
            self._record_at(t, y, rec_t)
            if t >= t_end - EPS_T:
                break
            t_next = self.next_break(t, t_end)
            # integrate t -> t_next, stopping at state events
            while t < t_next - EPS_T:
                evs = self.make_events()
                if self.fired_at[0] == t:
                    evs = [e for e in evs if e[2] not in self.fired_at[1]]
                fns = []
                for fn, direction, _tag in evs:
                    fn.terminal = True  # type: ignore[attr-defined]
                    fn.direction = direction  # type: ignore[attr-defined]
                    fns.append(fn)
                d = self.d
                sol = solve_ivp(
                    lambda tt, yy, d=d: self.plant.eval(tt, yy, d)[0],
                    (t, t_next),
                    y,
                    method=P.sim.method,
                    rtol=P.sim.rtol,
                    atol=P.sim.atol,
                    events=fns or None,
                    dense_output=True,
                    max_step=min(P.sim.max_step, record_dt),
                )
                if sol.status < 0:
                    raise RuntimeError(f"solve_ivp failed at t={t}: {sol.message}")
                t_b = float(sol.t[-1])
                self._record_between(t, t_b, sol.sol, rec_t)
                y = sol.y[:, -1].copy()
                if sol.status == 1:
                    tags = [evs[i][2] for i, te in enumerate(sol.t_events) if len(te)]
                    if self.fired_at[0] == t_b:
                        self.fired_at[1].update(tags)
                        zeno += 1
                    else:
                        self.fired_at = (t_b, set(tags))
                        zeno = 0
                    if zeno > 100:
                        raise ZenoError(f"event chattering at t={t_b}: {tags}")
                    t = t_b
                    for tag in tags:
                        self.handle(tag, t, y)
                    self.settle(t, y)
                    self._record_at(t, y, rec_t)
                else:
                    t = t_next
            t = t_next
        return pl.DataFrame(self.rows, strict=False)

    # ------------------------------------------------------------ recording
    def _record_between(self, ta: float, tb: float, dense: Any, rec_t: list[float]) -> None:
        while self.ri < len(rec_t) and rec_t[self.ri] < tb - EPS_T:
            tr = rec_t[self.ri]
            if tr > ta + EPS_T:
                self._emit(tr, np.asarray(dense(tr)))
            self.ri += 1

    def _record_at(self, t: float, y: np.ndarray, rec_t: list[float]) -> None:
        while self.ri < len(rec_t) and abs(rec_t[self.ri] - t) <= EPS_T:
            self._emit(rec_t[self.ri], y)
            self.ri += 1

    def _emit(self, t: float, y: np.ndarray) -> None:
        self._invalidate()
        s = self.sig(t, y)
        self._invalidate()
        rows = self.rows
        rows["t"].append(t)
        e_loss = sum(float(y[i]) for i in LOSS_IDX.values())
        e_in, e_ext = float(y[E_IN]), float(y[E_EXT])
        resid = (e_in + e_ext - e_loss - (s["_e_st"] - self.e_st0)) / max(
            float(y[E_THR]), self.P.sim.e_floor
        )
        en = {
            "energy.in": e_in,
            "energy.out": e_loss,  # SPEC-AMBIGUITY: Q-08
            "energy.stored": s["_e_st"],
            "energy.external": e_ext,
            "energy.residual": resid,
            "energy.ok": bool(abs(resid) < self.P.sim.r_tol),
        }
        for k, i in LOSS_IDX.items():
            en[f"energy.loss.{k}"] = float(y[i])
        for name in self.record:
            if name == "t":
                continue
            if name in en:
                rows[name].append(en[name])
            elif name in CTRL_SIGNALS:
                rows[name].append(self.d.ctrl_sig.get(name))
            else:
                rows[name].append(s[name])


def simulate(
    params: RefParams,
    actions: list[Action],
    t_end: float,
    record: list[str],
    record_dt: float,
) -> pl.DataFrame:
    """Run the reference model and return the recorded signals (column ``t`` in seconds).

    SPEC-AMBIGUITY: Q-01 (time column is ``t``, not ``sim.t``).

    Signal names follow ``docs/docs/physics/signals.md``. Records falling exactly on a
    discrete event time show the values after the events at that instant (EQ-NUM-02).
    """
    if record_dt <= 0 or t_end <= 0:
        raise ValueError("t_end and record_dt must be positive")
    return _Sim(params, actions).run(t_end, list(record), record_dt)
