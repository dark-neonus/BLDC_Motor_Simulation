"""Parameter tree of the reference model (all SI, kelvin for temperatures).

Every field is documented in ``README.md`` with its unit and the spec equation it feeds.
Defaults are the validation catalog's default test motor "M1" (p = 14, R = 1 Ohm,
L_s = 2.5 mH, lambda_m = 0.03 Wb, J = 2.5e-4 kg m^2, B = 1e-4 N m s/rad, V_bus = 24 V).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

G_STD = 9.80665  # m/s^2, mechanical.md assumption 3
T_20C = 293.15  # K, T_ref of EQ-THERM-02/03


@dataclass
class SimParams:
    """Integrator settings (EQ-NUM-02, EQ-NUM-07)."""

    method: str = "DOP853"
    rtol: float = 1e-9
    atol: float = 1e-12
    max_step: float = math.inf
    dt_max: float = 1e-4  # only used for the disturbance pulse width, EQ-MECH-08
    r_tol: float = 1e-3  # EQ-ENER-03 tolerance for energy.ok
    e_floor: float = 1e-6  # EQ-ENER-03


@dataclass
class MotorParams:
    """EQ-MOT-01..09, EQ-MECH-05 (motor bearing), EQ-THERM-03."""

    pole_pairs: int = 14
    r_phase: float = 1.0
    l_d: float = 2.5e-3
    l_q: float = 2.5e-3
    lambda_m: float = 0.03
    emf_shape: str = "sinusoidal"  # "sinusoidal" | "trapezoidal" | "harmonics"
    trap_width: float = 2.0 * math.pi / 3.0  # rad, flat-top width w (EQ-MOT-03)
    harmonics: tuple[tuple[int, float, float], ...] = ()  # (n, b_n, phi_n), EQ-MOT-03
    j_rotor: float = 2.5e-4
    friction_static: float = 0.0
    friction_coulomb: float = 0.0
    friction_viscous: float = 1e-4
    cogging_nc: int = 0  # N_c, EQ-MOT-08
    cogging: tuple[tuple[float, float], ...] = ()  # (A_k, phi_k), k = 1..K
    k_hy: float = 0.0  # EQ-MOT-09, W/(rad/s)
    k_ed: float = 0.0  # EQ-MOT-09, W/(rad/s)^2
    omega_eps_fe: float = 0.01
    alpha_br: float = -0.0012  # EQ-THERM-03
    topology: str = "outrunner"  # "outrunner" | "inrunner"
    theta0: float = 0.0
    omega0: float = 0.0
    i_alpha0: float = 0.0
    i_beta0: float = 0.0


@dataclass
class MechParams:
    """Kinematic test fixtures: "free" (EQ-MECH-02) or "prescribed" constant speed."""

    mode: str = "free"  # "free" | "prescribed"
    omega_prescribed: float = 0.0


@dataclass
class GearboxParams:
    """EQ-MECH-02/03 (rigid only)."""

    ratio: float = 1.0
    efficiency: float = 1.0
    j_in: float = 0.0
    j_out: float = 0.0
    omega_eps: float = 1e-3


@dataclass
class LoadParams:
    """EQ-MECH-05 (load bearing), EQ-MECH-06/07/08/09."""

    arm_mass: float = 0.0
    arm_length: float = 0.0
    mass: float = 0.0
    distance: float = 0.0
    g: float = G_STD
    kind: str = "none"  # "none" | "constant" | "brake" | "viscous"
    torque: float = 0.0  # T_0 for constant / brake
    viscous: float = 0.0  # c for viscous
    omega_eps: float = 1e-3  # SPEC-AMBIGUITY: Q-11 (brake-load smoothing speed)
    friction_static: float = 0.0
    friction_coulomb: float = 0.0
    friction_viscous: float = 0.0
    theta0: float | None = None  # if set, motor.theta0 = N * theta0


@dataclass
class ThermalParams:
    """EQ-THERM-01..04."""

    enabled: bool = False
    c_w: float = 10.0
    c_s: float = 30.0
    c_h: float = 60.0
    r_ws: float = 0.5
    r_sh: float = 0.5
    r_ha: float = 4.0
    kappa: float = 0.0
    t_amb: float = T_20C
    t_ref: float = T_20C
    alpha_cu: float = 0.00393
    t_init: float | None = None
    p_inject: float = 0.0  # test fixture: extra heat into the winding node, W


@dataclass
class InverterParams:
    """EQ-INV-03..09 (averaged model only)."""

    f_pwm: float = 20e3
    dead_time: float = 0.0
    r_on: float = 0.0
    v_f: float = 0.8
    t_rise: float = 0.0
    t_fall: float = 0.0
    i_eps: float = 0.01  # A, sigma(i) smoothing, EQ-INV-03. SPEC-AMBIGUITY: Q-12
    modulation: str = "svpwm"  # "svpwm" | "spwm"


@dataclass
class BusParams:
    """EQ-SUP-01, EQ-SUP-05."""

    capacitance: float = 470e-6
    v0: float | None = None
    chopper_enabled: bool = False
    r_brake: float = 10.0
    v_on: float = 30.0
    v_off: float = 28.0


@dataclass
class SupplyParams:
    """EQ-SUP-02..04. Battery values are per cell (pack built from series/parallel)."""

    kind: str = "ideal"  # "ideal" | "psu" | "battery"
    v_set: float = 24.0
    i_lim: float = 10.0
    r_o: float = 0.05
    series: int = 6
    parallel: int = 1
    ocv_soc: tuple[float, ...] = (0.0, 0.1, 0.2, 0.5, 0.8, 0.9, 1.0)
    ocv_cell: tuple[float, ...] = (3.00, 3.40, 3.55, 3.70, 3.95, 4.05, 4.20)
    r0_cell: float = 0.02
    r1_cell: float = 0.01
    c1_cell: float = 2000.0
    q_cell_ah: float = 2.0
    soc0: float = 0.8
    v_rc0: float = 0.0


@dataclass
class FocParams:
    """EQ-CTRL-01/03/04."""

    mode: str = "current"  # "current" | "velocity" | "position"
    id_ref: float = 0.0
    iq_ref: float = 0.0
    omega_c: float | None = None  # default 2*pi*f_ctrl/20
    anti_windup: str = "clamping"  # "clamping" | "back_calculation"
    m_max: float = 0.95
    delay_comp: bool = True
    omega_v: float | None = None  # default omega_c/10 (EQ-CTRL-12)
    kp_v: float | None = None
    ki_v: float | None = None
    vel_div: int = 1  # velocity loop runs every vel_div current ticks
    prefilter: bool = False
    omega_p: float | None = None  # default omega_v/5
    kp_p: float | None = None
    ki_p: float = 0.0
    pos_div: int = 1


@dataclass
class SixStepParams:
    """EQ-CTRL-05, EQ-INV-07."""

    duty: float = 0.5
    direction: int = 1
    advance: float = 0.0  # rad electrical
    speed_pi: bool = False
    kp: float = 0.01
    ki: float = 0.1


@dataclass
class OpenLoopParams:
    """EQ-CTRL-06."""

    omega_e_ref: float = 0.0
    accel: float = math.inf  # rad/s^2 electrical
    v0: float = 0.0
    k_vf: float = 0.0


@dataclass
class MitParams:
    """EQ-CTRL-07."""

    kp: float = 1.0
    kd: float = 0.05
    tau_ff: float = 0.0


@dataclass
class CtrlParams:
    """Controller selection, setpoints and limits."""

    kind: str = "none"  # "none" | "foc" | "six_step" | "open_loop" | "mit" | "ideal_voltage"
    f_ctrl: float = 10e3
    omega_ref: float = 0.0  # rad/s motor side (velocity loop) / load side (MIT)
    theta_ref: float = 0.0  # rad, load side
    i_max: float = 10.0
    omega_max: float = 1e4
    foc: FocParams = field(default_factory=FocParams)
    six_step: SixStepParams = field(default_factory=SixStepParams)
    open_loop: OpenLoopParams = field(default_factory=OpenLoopParams)
    mit: MitParams = field(default_factory=MitParams)
    # Test fixture (not a discrete controller): continuous ideal voltage source,
    # f(t, ctx) -> (v_alpha, v_beta). ctx holds theta_e, omega_e, r, lambda_m, i_alpha, ...
    ideal_voltage: Callable[[float, dict[str, float]], tuple[float, float]] | None = None


@dataclass
class RefParams:
    sim: SimParams = field(default_factory=SimParams)
    motor: MotorParams = field(default_factory=MotorParams)
    mech: MechParams = field(default_factory=MechParams)
    gearbox: GearboxParams = field(default_factory=GearboxParams)
    load: LoadParams = field(default_factory=LoadParams)
    thermal: ThermalParams = field(default_factory=ThermalParams)
    inverter: InverterParams = field(default_factory=InverterParams)
    bus: BusParams = field(default_factory=BusParams)
    supply: SupplyParams = field(default_factory=SupplyParams)
    ctrl: CtrlParams = field(default_factory=CtrlParams)


@dataclass
class Action:
    """A queued command applied at time ``t`` after the discrete events at ``t`` (EQ-NUM-02).

    ``path`` is a dotted parameter path (e.g. ``"ctrl.omega_ref"``) or one of the special
    paths ``"load.disturbance.impulse"`` (value H in N m s, EQ-MECH-08) and
    ``"load.disturbance.torque"`` (value in N m, needs ``duration``). Changes of
    ``load.mass``, ``load.distance``, ``load.arm_mass``, ``load.arm_length`` follow EQ-MECH-09
    with ``mode`` = ``"keep_speed"`` (default) or ``"conserve_momentum"``.
    """

    t: float
    path: str
    value: Any
    mode: str | None = None
    duration: float | None = None


def lambda_m_from_kv(kv_rpm_per_v: float, pole_pairs: int) -> float:
    """EQ-CONV-13 (LL-peak Kv): lambda_m = 60 / (2 pi sqrt3 p Kv)."""
    return 60.0 / (2.0 * math.pi * math.sqrt(3.0) * pole_pairs * kv_rpm_per_v)


def kt_from_lambda(lambda_m: float, pole_pairs: int) -> float:
    """EQ-CONV-11: K_t = 1.5 p lambda_m."""
    return 1.5 * pole_pairs * lambda_m


def ke_from_lambda(lambda_m: float, pole_pairs: int) -> float:
    """EQ-CONV-09: K_e = sqrt3 p lambda_m (LL peak)."""
    return math.sqrt(3.0) * pole_pairs * lambda_m
