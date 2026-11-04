"""Detour Steer - a planner for a counter-clockwise-only steering robot.

Standalone file: configuration, planner, proofs-by-computer, simulator,
figures and benchmark all live here.

The robot is mor_luam (E:/GPS_Localize/old/mor_luam): ONE motor, the wheel
module steers COUNTER-CLOCKWISE ONLY, and it cannot steer and drive at the same time.
Its firmware today (the "current method") always steers straight at the goal,
so a goal 10 deg to the RIGHT costs a 350 deg turn.

The idea
--------
Two straight legs whose headings differ by MORE than 180 deg can reach any
direction. So instead of steering phi (> 180 deg), the robot may:

    1. drive a metres straight on (no steer),
    2. steer beta counter-clockwise, with 180 deg < beta < phi,
    3. drive b metres to the goal.

It steers less and drives a bit further. Which beta is best has a closed
form, so the planner costs O(1):

    k     = (d * w / v) * |sin phi|            (w steer speed, v drive speed)
    beta* = 2*pi - arccos(k - 1)               only if phi > pi and k < 2
    a     = d * sin(beta* - phi) / sin(beta*)
    b     = d * sin(phi) / sin(beta*)

and the detour is used only when it is really faster than steering straight.
When it is not, the planner does exactly what the current method does.

Proved in README.md, and checked by computer in verify() below:
  T1  phi <= 180 deg      -> steering straight at the goal is optimal.
  T2  phi >  180 deg      -> the best path has at most 2 legs, headings 0 and beta.
  T3  the best beta is the closed form above (the time is unimodal in beta).
  T4  after leg a, planning again gives "steer beta*, drive b" (consistent),
      so the planner can simply run again after every move (closed loop).
      A wheel that stops PAST its target just reads as a new goal 350 deg
      away, and the detour recovers it without a full extra turn.

Angles: every heading is measured COUNTER-CLOCKWISE from the +x axis, so steering
only ever ADDS to the heading. The unit vector of heading psi is
(cos psi, sin psi).

Usage:
    python detour_steer.py             # verify + examples + benchmark + PNG
    python detour_steer.py --no-bench  # skip the slow benchmark
"""
import argparse
import math
import os
import random
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

TAU = 2.0 * math.pi

# =====================================================================
# Configuration (dataclasses, course house style)
# =====================================================================


@dataclass(frozen=True)
class RobotConfig:
    """The robot. Set these to the robot's own values."""
    drive_speed_mps: float = 0.03   # the real robot mor_luam (measured)
    steer_speed_dps: float = 35.0   # the real robot's steering speed (measured)
    settle_s: float = 0.05          # STEER_SETTLE_MS = 50 (hold inside tolerance)
    stop_s: float = 0.20            # stop + clutch switch, drive -> steer
    tol_deg: float = 3.5            # Wheel_STEER_ERROR_TOLERANCE = 3.5 deg
    goal_radius_m: float = 0.0025   # the robot's goal radius: arrived when this close
    max_phases: int = 60            # safety cap on steer/drive phases per goal


@dataclass(frozen=True)
class NoiseConfig:
    """Where the wheel really stops after a steer: + = past the target, - = short.

    Nobody knows this in advance - not the planner, not the robot. The robot
    only reads the real wheel angle (AS5600) once the wheel has stopped, and
    plans again from there. The planner never uses these numbers.
    """
    bias_deg: float = 0.0   # on average neither past nor short
    sd_deg: float = 2.0     # a little past or a little short, at random


@dataclass(frozen=True)
class BenchConfig:
    """Random goals: the same list for both methods (same seed)."""
    n_goals: int = 5000
    dist_range_m: Tuple[float, float] = (0.3, 2.0)
    steer_speeds_dps: Tuple[float, ...] = (20.0, 35.0, 60.0, 90.0, 120.0)
    # (bias, sd) in deg: none, a little either way, mostly short, mostly past
    noise_cases: Tuple[Tuple[float, float], ...] = ((0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0),
                                                     (-2.0, 2.0), (2.0, 2.0), (2.0, 3.0))
    noise_case_speed_dps: float = 35.0
    # "other speeds": the same goals with a faster or slower robot
    sweep_drive_mps: Tuple[float, ...] = (0.03, 0.10, 0.25)
    sweep_steer_dps: Tuple[float, ...] = (20.0, 35.0, 60.0, 120.0)
    seed: int = 2026


@dataclass(frozen=True)
class VerifyConfig:
    """Checking the theorems by brute force."""
    n_cases: int = 20000          # random (phi, d, w) for the 1-D grid check
    grid_beta: int = 20000        # beta samples in the 1-D grid search
    n_cases_2d: int = 200         # random cases for the full 2-leg (alpha, beta) search
    grid_step_2d_deg: float = 2.0
    seed: int = 7


@dataclass(frozen=True)
class MapConfig:
    """The decision map: where the detour wins, for a few steering speeds."""
    steer_speeds_dps: Tuple[float, ...] = (20.0, 35.0, 60.0)
    phi_range_deg: Tuple[float, float] = (180.0, 360.0)
    dist_range_m: Tuple[float, float] = (0.05, 2.0)
    resolution: int = 240


@dataclass(frozen=True)
class HWConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    noise: NoiseConfig = field(default_factory=NoiseConfig)
    bench: BenchConfig = field(default_factory=BenchConfig)
    verify: VerifyConfig = field(default_factory=VerifyConfig)
    decision_map: MapConfig = field(default_factory=MapConfig)
    output_dir: str = "output"


def describe(cfg: HWConfig) -> str:
    r, n = cfg.robot, cfg.noise
    return "\n".join([
        "Detour Steer",
        f"Drive speed     : {r.drive_speed_mps} m/s",
        f"Steer speed     : {r.steer_speed_dps} deg/s",
        f"Settle / stop   : {r.settle_s} s / {r.stop_s} s",
        f"Steer tolerance : +-{r.tol_deg} deg",
        f"Stop error      : bias {n.bias_deg} deg, sd {n.sd_deg} deg",
    ])


EXAMPLE_D_M = 0.30   # the goal distance used in the robot test


def measured_robot() -> RobotConfig:
    """Current measured planning profile, separate from the theory benchmark."""
    return RobotConfig(drive_speed_mps=0.03, steer_speed_dps=35.0, goal_radius_m=0.0025)


def comparison_route(total_m: float = 2.0, segments: int = 7, right_deg: float = 6.0):
    """Seven short right-bending legs; their lengths SUM to two metres."""
    if not 1 <= segments <= 7 or total_m / segments < 0.1:
        raise ValueError('Use 1..7 segments, each at least 0.10 m')
    x = y = 0.0
    points = []
    for i in range(segments):
        angle = math.radians(-(i + 1) * right_deg)
        x += total_m / segments * math.cos(angle)
        y += total_m / segments * math.sin(angle)
        points.append((x, y))
    return points


def ideal_route(points, robot=None, use_detour=True):
    """Constant-speed model used by the browser; never a hardware measurement."""
    robot = robot or measured_robot()
    x = y = h = elapsed = turn_total = length = 0.0
    detours = 0
    path = [(0.0, 0.0)]

    def drive(distance):
        nonlocal x, y, elapsed, length
        x += distance * math.cos(h); y += distance * math.sin(h)
        elapsed += distance / robot.drive_speed_mps; length += distance
        path.append((x, y))

    def steer(angle):
        nonlocal h, elapsed, turn_total
        if angle > 1e-12:
            h += angle; turn_total += angle
            elapsed += math.degrees(angle) / robot.steer_speed_dps + robot.settle_s

    for gx, gy in points:
        d = math.hypot(gx - x, gy - y)
        if d <= 0.02:
            continue
        phi = steer_error(bearing(gx - x, gy - y), h)
        if in_tolerance(phi, math.radians(robot.tol_deg)):
            phi = 0.0
        plan = plan_leg(phi, d, robot) if use_detour else Plan('direct', 0, phi, d, direct_time(phi, d, robot))
        if plan.kind == 'detour':
            drive(plan.a); elapsed += robot.stop_s; detours += 1
        steer(plan.beta); drive(plan.b)
        remaining = math.hypot(gx - x, gy - y)
        if remaining > 0.02:
            steer(steer_error(bearing(gx - x, gy - y), h)); drive(remaining)
    return {'time_s': elapsed, 'length_m': length, 'turn_deg': math.degrees(turn_total),
            'detours': detours, 'path': path}


def with_speed(robot: RobotConfig, steer_dps: float) -> RobotConfig:
    return RobotConfig(**{**robot.__dict__, "steer_speed_dps": steer_dps})


# =====================================================================
# Algorithm 1.1: counter-clockwise geometry (as in the firmware, main.cpp:40-45)
# =====================================================================


def unit(psi: float) -> Tuple[float, float]:
    """Unit vector of a COUNTER-CLOCKWISE heading."""
    return math.cos(psi), math.sin(psi)


def bearing(dx: float, dy: float) -> float:
    """CounterClockwise heading that points along (dx, dy)."""
    return math.atan2(dy, dx) % TAU


def steer_error(target: float, current: float) -> float:
    """How far the wheel must turn COUNTER-CLOCKWISE to reach target: 0 .. 2*pi."""
    return (target - current) % TAU


# Compatibility alias for older notebooks; the physical direction is CCW.
cw_error = steer_error


def in_tolerance(e: float, tol: float) -> bool:
    """Close enough on either side. Just past the target reads as ~2*pi."""
    return e <= tol or e >= TAU - tol


# =====================================================================
# Algorithm 1.2: the best detour angle, in closed form
# =====================================================================


def optimal_detour_angle(phi: float, d: float, robot: RobotConfig) -> Optional[float]:
    """beta* in (pi, phi), or None when no detour can beat steering straight.

    phi: counter-clockwise angle from the wheel to the goal (0 .. 2*pi)
    d  : distance to the goal (m)
    """
    if phi <= math.pi:                       # T1: straight at the goal is optimal
        return None
    w = math.radians(robot.steer_speed_dps)
    k = (d * w / robot.drive_speed_mps) * abs(math.sin(phi))
    if k >= 2.0:                             # T3: time falls all the way to phi
        return None
    beta = TAU - math.acos(k - 1.0)          # T3: the one root of dT/dbeta = 0
    return beta if beta < phi else None


def detour_legs(phi: float, d: float, beta: float) -> Tuple[float, float]:
    """Lengths of leg 1 (heading 0) and leg 2 (heading beta) that end at the goal."""
    s = math.sin(beta)
    return d * math.sin(beta - phi) / s, d * math.sin(phi) / s


# =====================================================================
# Algorithm 1.3: plan one leg (direct or detour)
# =====================================================================


@dataclass(frozen=True)
class Plan:
    kind: str            # "direct" or "detour"
    a: float             # leg 1, driven first with NO steer (0 for direct)
    beta: float          # the one counter-clockwise steer
    b: float             # leg 2, to the goal
    time_s: float


def direct_time(phi: float, d: float, robot: RobotConfig) -> float:
    """The current method: steer phi counter-clockwise, then drive d."""
    w = math.radians(robot.steer_speed_dps)
    steer = phi / w + robot.settle_s if phi > 1e-12 else 0.0
    return steer + d / robot.drive_speed_mps


def detour_time(a: float, beta: float, b: float, robot: RobotConfig) -> float:
    w = math.radians(robot.steer_speed_dps)
    v = robot.drive_speed_mps
    return a / v + robot.stop_s + beta / w + robot.settle_s + b / v


def plan_leg(phi: float, d: float, robot: RobotConfig) -> Plan:
    """O(1). The faster of 'steer phi, drive d' and the best detour."""
    direct = Plan("direct", 0.0, phi, d, direct_time(phi, d, robot))
    beta = optimal_detour_angle(phi, d, robot)
    if beta is None:
        return direct
    a, b = detour_legs(phi, d, beta)
    t = detour_time(a, beta, b, robot)
    # The detour adds one stop (drive -> steer), so it must really be faster.
    return Plan("detour", a, beta, b, t) if t < direct.time_s else direct


# =====================================================================
# Algorithm 1.4: closed-loop execution (plan again after every move)
# =====================================================================

StopError = Callable[[], float]       # returns radians, + = past the target


def gaussian_stop_error(noise: NoiseConfig, rng: random.Random) -> StopError:
    mu, sd = math.radians(noise.bias_deg), math.radians(noise.sd_deg)
    return lambda: rng.gauss(mu, sd)


def fixed_stop_errors(errors_deg: List[float]) -> StopError:
    """Scripted stop errors for the example figures, then exact stops."""
    queue = [math.radians(e) for e in errors_deg]
    return lambda: queue.pop(0) if queue else 0.0


@dataclass
class Run:
    time_s: float = 0.0
    steer_turn_rad: float = 0.0
    n_steers: int = 0
    n_detours: int = 0
    events: list = field(default_factory=list)   # for the figures


def run_goal(goal: Tuple[float, float], robot: RobotConfig, stop_error: StopError,
             use_detour: bool = True) -> Run:
    """Drive from (0,0), heading 0, to goal.

    use_detour=False is the current method, kept here ONLY so the comparison runs
    both planners through the very same simulator.
    """
    w = math.radians(robot.steer_speed_dps)
    tol = math.radians(robot.tol_deg)
    x = y = h = 0.0
    run = Run()
    for _ in range(robot.max_phases):
        dx, dy = goal[0] - x, goal[1] - y
        dist = math.hypot(dx, dy)
        if dist <= robot.goal_radius_m:
            break
        e = steer_error(bearing(dx, dy), h)             # measured, every time
        if in_tolerance(e, tol):
            # DRIVE along the wheel's REAL heading, which may be up to tol off.
            # A miss bigger than goal_radius is simply planned again, as
            # drive_to_xy.py does on the robot; both methods get this.
            ux, uy = unit(h)
            nx, ny = x + dist * ux, y + dist * uy
            run.events.append(("drive", x, y, nx, ny))
            run.time_s += dist / robot.drive_speed_mps
            x, y = nx, ny
            continue
        plan = plan_leg(e, dist, robot) if use_detour else None
        if plan is not None and plan.kind == "detour":
            # Leg 1: drive straight on, NO steer. Then plan again (T4 says
            # the new plan is "steer beta*, drive b").
            ux, uy = unit(h)
            nx, ny = x + plan.a * ux, y + plan.a * uy
            run.events.append(("drive", x, y, nx, ny))
            run.time_s += plan.a / robot.drive_speed_mps + robot.stop_s
            run.n_detours += 1
            x, y = nx, ny
            continue
        # STEER counter-clockwise. Where the wheel stops is unknown until it stops:
        # a little short is fixed by a small extra steer, a little past
        # reads as a goal ~360 deg away, which the next PlanLeg detours.
        turn = max(0.0, e + stop_error())
        run.events.append(("steer", x, y, h, turn))
        run.time_s += turn / w + robot.settle_s
        run.steer_turn_rad += turn
        run.n_steers += 1
        h = (h + turn) % TAU
    return run


# =====================================================================
# Robot-like model: stop error + drift while driving, fitted to the robot
# =====================================================================


@dataclass(frozen=True)
class RealismConfig:
    """What the ideal model leaves out, MEASURED from the robot's own logs.

    Source: tools/test_out/*.csv (2026-10-08, every 10 ms): 15 big turns,
    5 small turns, 23 drives. Each turn and drive is drawn at random from
    these measured spreads, so no two runs are the same.
    - steering time (motor on) = fixed + angle / 37.8 deg/s, fixed 0.90 s
      for big turns and ~0.45 s for 3-4 deg turns, scatter sd 0.19 s
    - after a steer the robot waits 0.11 s (measured) plus the IMU settle
      of the current firmware (0 .. 0.4 s); a drive -> steer switch: ~0 s
    - where the wheel stops: the current firmware lands 2 deg short on
      purpose (the logs, before that change: 3.45 +- 0.15 deg short).
      overshoot_p: chance that it turns PAST instead (by tol .. 10 deg),
      which costs almost a full extra turn; none was in the logs, so it is
      a what-if setting (default 0)
    - drift while driving: the direction of travel = body heading (IMU) +
      wheel angle (AS5600). Body: 7.4 +- 7.1 deg/m to the right; wheel:
      7.0 +- 12.6 deg/m to the right; together 14.4 +- 14.5 deg/m
    """
    bias_deg: float = -2.0
    sd_deg: float = 0.15
    overshoot_p: float = 0.0
    overshoot_max_deg: float = 10.0
    drift_deg_per_m: float = 14.4
    drift_sd_deg_per_m: float = 14.5
    steer_dps: float = 37.8         # wheel speed while turning (the planner still uses its own 35 deg/s)
    steer_fixed_s: float = 0.90     # motor start + slow-down, big turns
    steer_fixed_small_s: float = 0.45
    steer_time_sd_s: float = 0.19
    settle_min_s: float = 0.11      # measured gap steer -> drive
    settle_max_s: float = 0.51      # + up to 0.4 s IMU settle (current firmware)
    switch_s: float = 0.0           # measured gap drive -> steer
    drive_bias_m: float = 0.0       # where a drive really stops: + past, - short (not in the logs)
    drive_sd_m: float = 0.0
    reaim_ratio: float = 3.0        # Algorithm 1.7: stop when the goal is this many times its offset ahead
    reaim_min_m: float = 0.02


def real_stop_error(real: RealismConfig, rng: random.Random, tol: float) -> float:
    """Where the wheel stops against the target (rad, + = past)."""
    if real.overshoot_p > 0 and rng.random() < real.overshoot_p:
        return rng.uniform(tol + 1e-3, math.radians(real.overshoot_max_deg))
    return rng.gauss(math.radians(real.bias_deg), math.radians(real.sd_deg))


def real_steer_time(real: RealismConfig, turn: float, rng: random.Random, driven: bool) -> float:
    """Measured steering time for one turn (rad), including the waits around it."""
    deg = math.degrees(turn)
    fixed = real.steer_fixed_small_s + (real.steer_fixed_s - real.steer_fixed_small_s) * min(1.0, deg / 40.0)
    motor = max(0.2, fixed + deg / real.steer_dps + rng.gauss(0.0, real.steer_time_sd_s))
    return motor + rng.uniform(real.settle_min_s, real.settle_max_s) + (real.switch_s if driven else 0.0)


def run_route_real(points, robot: RobotConfig, real: RealismConfig, rng: random.Random,
                   use_detour: bool = True) -> Tuple[float, list]:
    """Closed loop over a route with stop error and drive drift. Returns (time s, path)."""
    w = math.radians(robot.steer_speed_dps)
    tol = math.radians(robot.tol_deg)
    x = y = h = t = 0.0
    path = [(0.0, 0.0)]
    driven = False                     # a steer right after a drive pays the motor switch t_b

    def drive(dist: float) -> None:
        """Algorithm 1.7: drive towards the leg end q with drift; stop early to re-aim."""
        nonlocal x, y, h, t, driven
        rate = math.radians(rng.gauss(real.drift_deg_per_m, real.drift_sd_deg_per_m))
        qx, qy = x + dist * math.cos(h), y + dist * math.sin(h)
        step, moved = 0.002, 0.0
        while moved < dist + 0.05:
            ex, ey = qx - x, qy - y
            f = ex * math.cos(h) + ey * math.sin(h)          # ahead
            s = -ex * math.sin(h) + ey * math.cos(h)         # beside, + = steering side (left)
            if math.hypot(ex, ey) <= robot.goal_radius_m or f <= 0.0:
                break
            if s > robot.goal_radius_m and f <= max(real.reaim_min_m, real.reaim_ratio * s):
                break
            h -= rate * step / 2
            x += step * math.cos(h); y += step * math.sin(h)
            h -= rate * step / 2
            moved += step
        # the drive motor also coasts: the robot never stops at exactly the planned point
        coast = max(-moved, rng.gauss(real.drive_bias_m, real.drive_sd_m))
        x += coast * math.cos(h); y += coast * math.sin(h)
        moved += coast
        h %= TAU
        t += moved / robot.drive_speed_mps
        driven = True
        path.append((x, y))

    for gx, gy in points:
        for _ in range(robot.max_phases):
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist <= robot.goal_radius_m:
                break
            e = steer_error(bearing(dx, dy), h)
            if in_tolerance(e, tol):
                drive(dist)
                continue
            plan = plan_leg(e, dist, robot) if use_detour else None
            if plan is not None and plan.kind == "detour":
                drive(plan.a)
                continue
            turn = max(0.0, e + real_stop_error(real, rng, tol))
            t += real_steer_time(real, turn, rng, driven)
            driven = False
            h = (h + turn) % TAU
    return t, path


ROBOT_TEST = {"old": (24.17, 22.94, 23.36), "detour": (20.83, 20.77, 22.33)}   # s, 3 rounds each


def real_times(points, robot, real, n=400, seed=11):
    out = {}
    for name, det in (("old", False), ("detour", True)):
        ts = sorted(run_route_real(points, robot, real, random.Random(seed + i), det)[0] for i in range(n))
        out[name] = (sum(ts) / n, ts[int(0.05 * n)], ts[int(0.95 * n) - 1])
    return out


# =====================================================================
# Verification: the theorems, checked by brute force
# =====================================================================


def grid_best_time(phi: float, d: float, robot: RobotConfig, n: int) -> float:
    """Best time over direct + n evenly spaced beta in (pi, phi)."""
    best = direct_time(phi, d, robot)
    if phi > math.pi:
        for i in range(1, n):
            beta = math.pi + (phi - math.pi) * i / n
            a, b = detour_legs(phi, d, beta)
            best = min(best, detour_time(a, beta, b, robot))
    return best


def two_leg_time(phi: float, d: float, alpha: float, beta: float,
                 robot: RobotConfig) -> Optional[float]:
    """Any 2-leg path: steer alpha, drive, steer to beta, drive. None if impossible."""
    ua, ub = unit(alpha), unit(beta)
    det = ua[0] * ub[1] - ua[1] * ub[0]
    if abs(det) < 1e-9:
        return None
    gx, gy = d * math.cos(phi), d * math.sin(phi)
    a = (gx * ub[1] - gy * ub[0]) / det
    b = (ua[0] * gy - ua[1] * gx) / det
    if a < -1e-12 or b < -1e-12:
        return None
    w = math.radians(robot.steer_speed_dps)
    v = robot.drive_speed_mps
    t = a / v + b / v + robot.stop_s + (beta - alpha) / w + robot.settle_s
    if alpha > 0:
        t += alpha / w + robot.settle_s
    return t


def verify(cfg: HWConfig) -> None:
    vc = cfg.verify
    rng = random.Random(vc.seed)
    print("\n============================================================")
    print("1. Verification by brute force")
    print("============================================================")

    # T3: closed form vs a fine 1-D grid over beta
    worst = 0.0
    for _ in range(vc.n_cases // 20):
        robot = with_speed(cfg.robot, rng.uniform(15.0, 180.0))
        phi, d = rng.uniform(0.0, TAU), rng.uniform(0.05, 3.0)
        worst = max(worst, plan_leg(phi, d, robot).time_s - grid_best_time(phi, d, robot, vc.grid_beta))
    print(f"  T3 closed form vs {vc.grid_beta}-point grid, {vc.n_cases // 20} cases: "
          f"worst (closed - grid) = {worst:+.2e} s  -> {'PASS' if worst <= 1e-9 else 'FAIL'}")

    # T1 + T2: no 2-leg path with ANY first heading alpha beats the plan
    step = math.radians(vc.grid_step_2d_deg)
    n_ang = int(round(TAU / step))
    worst2 = 0.0
    for _ in range(vc.n_cases_2d):
        robot = with_speed(cfg.robot, rng.uniform(15.0, 180.0))
        phi, d = rng.uniform(0.0, TAU), rng.uniform(0.05, 3.0)
        best = plan_leg(phi, d, robot).time_s
        for i in range(n_ang):
            alpha = i * step
            for j in range(i, n_ang):
                t = two_leg_time(phi, d, alpha, j * step, robot)
                if t is not None:
                    worst2 = max(worst2, best - t)
    print(f"  T1+T2 plan vs every 2-leg path (alpha<=beta, {vc.grid_step_2d_deg:.0f} deg grid), "
          f"{vc.n_cases_2d} cases: worst (plan - search) = {worst2:+.2e} s  -> "
          f"{'PASS' if worst2 <= 1e-9 else 'FAIL'}")

    # T4: after leg a, planning again gives "steer beta*, drive b"
    checked = bad = 0
    while checked < 2000:
        robot = with_speed(cfg.robot, rng.uniform(15.0, 180.0))
        phi, d = rng.uniform(math.pi, TAU), rng.uniform(0.05, 3.0)
        p = plan_leg(phi, d, robot)
        if p.kind != "detour":
            continue
        checked += 1
        again = plan_leg(p.beta, p.b, robot)       # from the end of leg 1
        if again.kind != "direct" or abs(again.beta - p.beta) > 1e-9:
            bad += 1
    print(f"  T4 plan again after leg 1, {checked} detours: {bad} changed their mind"
          f"  -> {'PASS' if bad == 0 else 'FAIL'}")

    # O(1) vs searching: planning time per call
    robot = cfg.robot
    cases = [(rng.uniform(0.0, TAU), rng.uniform(0.3, 2.0)) for _ in range(2000)]
    t0 = time.perf_counter()
    for phi, d in cases * 10:
        plan_leg(phi, d, robot)
    closed_us = (time.perf_counter() - t0) / (len(cases) * 10) * 1e6
    t0 = time.perf_counter()
    for phi, d in cases[:200]:
        grid_best_time(phi, d, robot, 3600)
    grid_us = (time.perf_counter() - t0) / 200 * 1e6
    print(f"  planning time per call: closed form {closed_us:.2f} us (O(1)), "
          f"3600-point search {grid_us:.0f} us (O(K)), {grid_us / closed_us:.0f}x")


# =====================================================================
# Figures
# =====================================================================

INK = "#222222"
MUTED = "#6b6b66"
GRID = "#e4e4e0"
SURFACE_LOW = "#ecebe7"   # "direct is optimal" cells
C_OLD = "#eb6834"         # orange: the current method in every figure
C_NEW = "#2a78d6"         # blue:   Detour Steer
BLUE_RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]


def _style(ax) -> None:
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)


def _draw_steer(ax, x, y, h0, turn, color, radius) -> None:
    """A counter-clockwise arc around (x, y) showing how far the wheel turned."""
    n = max(8, int(math.degrees(turn) / 4))
    pts = [(x + radius * math.cos(h0 + turn * i / n),
            y + radius * math.sin(h0 + turn * i / n)) for i in range(n + 1)]
    ax.plot([p[0] for p in pts[:-1]], [p[1] for p in pts[:-1]], color=color, lw=1.6)
    ax.annotate("", xy=pts[-1], xytext=pts[-2],
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6, mutation_scale=11))


def _draw_run(ax, run: Run, color: str) -> None:
    for ev in run.events:
        if ev[0] == "drive":
            _, x0, y0, x1, y1 = ev
            ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=2, mutation_scale=13))
            length = math.hypot(x1 - x0, y1 - y0)
            if length > 0.4:                      # short legs speak for themselves
                ax.text((x0 + x1) / 2 + 0.07, (y0 + y1) / 2 - 0.03, f"drive {length:.2f} m",
                        fontsize=8.5, color=INK, ha="left", va="top")
    # one label block per place the wheel steered, numbered in time order
    labels = {}
    n = 0
    for ev in run.events:
        if ev[0] == "steer":
            _, x, y, h0, turn = ev
            n += 1
            key = (round(x, 3), round(y, 3))
            radius = 0.07 + 0.04 * len(labels.get(key, []))
            _draw_steer(ax, x, y, h0, turn, color, radius)
            labels.setdefault(key, []).append(f"{n}. steer {math.degrees(turn):.0f}\u00b0")
    for (x, y), lines in labels.items():
        ax.text(x - 0.13, y + 0.06, "\n".join(lines), fontsize=8.5, color=INK, ha="right", va="bottom")


def plot_examples(cfg: HWConfig, path: str) -> None:
    robot = cfg.robot
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.6))
    cases = [
        ("(a) Goal 0.30 m away, 6\u00b0 to the RIGHT (the robot test)", 354.0, []),
        ("(b) Goal 0.30 m at 60\u00b0, the wheel stops 8\u00b0 too far", 60.0, [8.0]),
    ]
    for ax, (title, phi_deg, errs) in zip(axes, cases):
        phi = math.radians(phi_deg)
        goal = (EXAMPLE_D_M * math.cos(phi), EXAMPLE_D_M * math.sin(phi))
        new = run_goal(goal, robot, fixed_stop_errors(errs), use_detour=True)
        old = run_goal(goal, robot, fixed_stop_errors(errs), use_detour=False)
        _draw_run(ax, new, C_NEW)
        ax.plot(0, 0, "o", color=INK, ms=6)
        ax.text(-0.02, -0.01, "start,\nheading 0\u00b0", fontsize=8.5, color=INK, ha="right", va="top")
        ax.plot(*goal, marker="*", color=INK, ms=13)
        ax.text(goal[0] + 0.01, goal[1] + 0.01, "goal", fontsize=9, color=INK)
        saved = 100.0 * (old.time_s - new.time_s) / old.time_s
        ax.set_title(f"{title}\nDetour Steer {new.time_s:.2f} s, steered {math.degrees(new.steer_turn_rad):.0f}\u00b0"
                     f" ({saved:.1f}% faster)\nold method {old.time_s:.2f} s, steered "
                     f"{math.degrees(old.steer_turn_rad):.0f}\u00b0", fontsize=9.5, color=INK, loc="left")
        ax.set_aspect("equal")
        ax.set_xlim(-0.16, 0.46)
        ax.set_ylim(-0.12, 0.40)
        ax.set_xlabel("x (m)", fontsize=9, color=MUTED)
        _style(ax)
    axes[0].set_ylabel("y (m)", fontsize=9, color=MUTED)
    fig.suptitle(f"Detour Steer (steer {robot.steer_speed_dps:.0f}\u00b0/s, "
                 f"drive {robot.drive_speed_mps} m/s): drive on, steer less, come back",
                 fontsize=11, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def plot_route_2m(cfg: HWConfig, path: str) -> None:
    """The 2 m / 7-leg route at the robot's speeds: old method vs Detour Steer."""
    pts = comparison_route()
    old = ideal_route(pts, cfg.robot, use_detour=False)
    new = ideal_route(pts, cfg.robot, use_detour=True)
    saved = 100.0 * (old["time_s"] - new["time_s"]) / old["time_s"]
    fig, ax = plt.subplots(figsize=(10.5, 4.4))
    for res, color, name in ((old, C_OLD, "Old method"), (new, C_NEW, "Detour Steer")):
        xs, ys = zip(*res["path"])
        ax.plot(xs, ys, "-", color=color, lw=2.2,
                label=f"{name}: {res['time_s']:.2f} s, {res['length_m']:.2f} m, "
                      f"steered {res['turn_deg']:.0f}°, {res['detours']} detours")
    ax.plot(0, 0, "o", color=INK, ms=6)
    ax.text(0, 0.03, "start", fontsize=8.5, color=INK, ha="center")
    for i, (x, y) in enumerate(pts, 1):
        ax.plot(x, y, marker="o", mfc="white", mec=INK, ms=7)
        ax.text(x, y + 0.03, str(i), fontsize=8.5, color=INK, ha="center")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", fontsize=9, color=MUTED)
    ax.set_ylabel("y (m)", fontsize=9, color=MUTED)
    ax.legend(frameon=False, fontsize=9, loc="lower left")
    ax.set_title(f"2 m in 7 legs, each 6° to the right (steer {cfg.robot.steer_speed_dps:.0f}°/s, "
                 f"drive {cfg.robot.drive_speed_mps} m/s): Detour Steer {saved:.1f}% faster",
                 fontsize=10.5, color=INK, loc="left")
    _style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"  2 m / 7 legs: old method {old['time_s']:.2f} s ({old['turn_deg']:.0f} deg steered), "
          f"Detour Steer {new['time_s']:.2f} s ({new['turn_deg']:.0f} deg, {new['detours']} detours), {saved:.1f}% faster")


def print_robot_like(cfg: HWConfig) -> None:
    """Robot-like model (stop bias, drift, re-aim, per-steer time) against the robot test, then 2 m."""
    real = RealismConfig()
    phi = math.radians(354.0)
    test = [(EXAMPLE_D_M * math.cos(phi), EXAMPLE_D_M * math.sin(phi))]
    print("\n  robot-like model, values measured from tools/test_out logs (stop %+.1f+-%.2f deg, "
          "drift %.1f+-%.1f deg/m, steer %.2f s + angle/%.1f deg/s), 1000 runs:"
          % (real.bias_deg, real.sd_deg, real.drift_deg_per_m, real.drift_sd_deg_per_m,
             real.steer_fixed_s, real.steer_dps))
    for name, pts in (("0.30 m test", test), ("2 m / 7 legs", comparison_route())):
        res = real_times(pts, cfg.robot, real, n=1000)
        o, n = res["old"], res["detour"]
        print(f"    {name:<13}: old {o[0]:.2f} s (5-95 %: {o[1]:.2f}-{o[2]:.2f}), "
              f"Detour {n[0]:.2f} s ({n[1]:.2f}-{n[2]:.2f}), saved {100 * (o[0] - n[0]) / o[0]:.1f}%")
    print("    robot test, mean of 3: old 23.49 s, Detour 21.31 s (saved 9.3%)")


def plot_decision_map(cfg: HWConfig, path: str) -> None:
    """Where the detour wins, and by how much, for a few steering speeds."""
    mc = cfg.decision_map
    n = mc.resolution
    phis = [mc.phi_range_deg[0] + (mc.phi_range_deg[1] - mc.phi_range_deg[0]) * (i + 0.5) / n for i in range(n)]
    ds = [mc.dist_range_m[0] + (mc.dist_range_m[1] - mc.dist_range_m[0]) * (j + 0.5) / n for j in range(n)]
    cmap = LinearSegmentedColormap.from_list("blue", BLUE_RAMP)
    cmap.set_bad(SURFACE_LOW)
    fig, axes = plt.subplots(1, len(mc.steer_speeds_dps), figsize=(12, 4.0), sharey=True)
    vmax = 50.0
    im = None
    for ax, w in zip(axes, mc.steer_speeds_dps):
        robot = with_speed(cfg.robot, w)
        grid = []
        for d in ds:
            row = []
            for p in phis:
                phi = math.radians(p)
                t_dir = direct_time(phi, d, robot)
                plan = plan_leg(phi, d, robot)
                s = 100.0 * (t_dir - plan.time_s) / t_dir
                row.append(s if plan.kind == "detour" else float("nan"))
            grid.append(row)
        im = ax.imshow(grid, origin="lower", aspect="auto", cmap=cmap, vmin=0.0, vmax=vmax,
                       extent=(mc.phi_range_deg[0], mc.phi_range_deg[1], mc.dist_range_m[0], mc.dist_range_m[1]),
                       interpolation="nearest")
        ax.set_title(f"steer {w:.0f}\u00b0/s", fontsize=10.5, color=INK, loc="left")
        ax.set_xlabel("goal angle \u03c6, counter-clockwise (\u00b0)", fontsize=9, color=MUTED)
        ax.set_xticks([180, 225, 270, 315, 360])
        ax.text(200, 0.3, "gray: steering\nstraight is best", fontsize=8.5, color=INK)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.tick_params(colors=MUTED, labelsize=9)
    axes[0].set_ylabel("distance to the goal d (m)", fontsize=9, color=MUTED)
    cb = fig.colorbar(im, ax=axes, shrink=0.9, pad=0.015)
    cb.set_label("time saved by the detour (%)", fontsize=9, color=MUTED)
    cb.ax.tick_params(colors=MUTED, labelsize=8.5)
    fig.suptitle("Detour Steer decision map: where the detour is faster (blue)",
                 fontsize=11, color=INK, x=0.01, ha="left")
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)


# =====================================================================
# Benchmark: both algorithms on the same random goals
# =====================================================================


def random_goals(bench: BenchConfig) -> List[Tuple[float, float, int]]:
    """(phi, d, noise seed) per goal. Both methods get the same list."""
    rng = random.Random(bench.seed)
    goals = []
    for _ in range(bench.n_goals):
        phi = rng.uniform(0.0, TAU)
        d = rng.uniform(*bench.dist_range_m)
        goals.append((phi, d, rng.randrange(2 ** 31)))
    return goals


def compare(goals, robot: RobotConfig, noise: Optional[NoiseConfig]) -> Tuple[float, float, float, float]:
    """Both methods on the same goals and the same stop errors.

    Returns (old mean s, Detour Steer mean s, share of goals that used a
    detour, share of goals where Detour Steer was slower).
    """
    old = new = 0.0
    used = slower = 0
    for phi, d, seed in goals:
        goal = (d * math.cos(phi), d * math.sin(phi))
        times = []
        for use_detour in (False, True):
            err = gaussian_stop_error(noise, random.Random(seed)) if noise else (lambda: 0.0)
            run = run_goal(goal, robot, err, use_detour)
            times.append(run.time_s)
            if use_detour:
                used += run.n_detours > 0
        old += times[0]
        new += times[1]
        slower += times[1] > times[0] + 1e-9
    n = len(goals)
    return old / n, new / n, used / n, slower / n


def run_benchmark(cfg: HWConfig) -> None:
    goals = random_goals(cfg.bench)
    nz = cfg.noise
    print("\n============================================================")
    print(f"3. Benchmark: {len(goals)} random goals, d in {cfg.bench.dist_range_m} m, same goals for both methods")
    print("============================================================")
    print(f"3a. steering speed (stop error: none | bias {nz.bias_deg:+.0f} deg, sd {nz.sd_deg:.0f} deg)")
    print(f"{'steer deg/s':>11} | {'old s':>7} {'new s':>7} {'saved':>6} {'detour':>7} | "
          f"{'old s':>7} {'new s':>7} {'saved':>6} {'detour':>7} {'slower':>7}")
    rows = []
    for w in cfg.bench.steer_speeds_dps:
        robot = with_speed(cfg.robot, w)
        e1, e2, eu, _ = compare(goals, robot, None)
        n1, n2, nu, ns = compare(goals, robot, nz)
        rows.append((w, e1, e2, n1, n2))
        print(f"{w:>11.0f} | {e1:>7.2f} {e2:>7.2f} {100 * (e1 - e2) / e1:>5.1f}% {100 * eu:>6.1f}% | "
              f"{n1:>7.2f} {n2:>7.2f} {100 * (n1 - n2) / n1:>5.1f}% {100 * nu:>6.1f}% {100 * ns:>6.1f}%")

    # Nobody knows the real stop error, so try many: none, a little either way,
    # mostly short, mostly past. The planner is the same in every case.
    robot = with_speed(cfg.robot, cfg.bench.noise_case_speed_dps)
    print(f"\n3b. unknown stop error (+ past, - short), steer {robot.steer_speed_dps:.0f} deg/s")
    print(f"{'bias deg':>8} {'sd deg':>7} | {'old s':>7} {'new s':>7} {'saved':>6} {'slower':>7}")
    cases = []
    for bias, sd in cfg.bench.noise_cases:
        o, n, _, sl = compare(goals, robot, NoiseConfig(bias, sd))
        cases.append((bias, sd, o, n, sl))
        print(f"{bias:>+8.0f} {sd:>7.0f} | {o:>7.2f} {n:>7.2f} {100 * (o - n) / o:>5.1f}% {100 * sl:>6.1f}%")

    fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.0), sharey=True)
    ws = [r[0] for r in rows]
    top = 1.1 * max(max(r[1:]) for r in rows)
    for ax, (title, i1, i2) in zip(axes, (("Exact stop", 1, 2),
                                           (f"Unknown stop error (bias {nz.bias_deg:+.0f}°, "
                                            f"sd {nz.sd_deg:.0f}°)", 3, 4))):
        ax.plot(ws, [r[i1] for r in rows], "-o", color=C_OLD, lw=2, ms=7, label="Current method (steer straight)")
        ax.plot(ws, [r[i2] for r in rows], "-o", color=C_NEW, lw=2, ms=7, label="Detour Steer")
        for r in rows:
            saved = 100 * (r[i1] - r[i2]) / r[i1]
            ax.text(r[0], r[i2] - 0.5, f"{saved:.1f}%", fontsize=8.5, color=INK, ha="center", va="top")
        ax.set_title(title, fontsize=10.5, color=INK, loc="left")
        ax.set_xlabel("steering speed (°/s)", fontsize=9, color=MUTED)
        ax.set_ylim(0, top)
        _style(ax)
    axes[0].set_ylabel("mean time per goal (s)", fontsize=9, color=MUTED)
    axes[0].legend(frameon=False, fontsize=9, loc="upper right")
    fig.suptitle(f"Current method vs Detour Steer on the same {len(goals)} random goals (labels: time saved)",
                 fontsize=11, color=INK, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(cfg.output_dir, "fig_compare.png"), dpi=160, bbox_inches="tight")
    plt.close(fig)

    # one bar per stop-error case: how much faster Detour Steer is
    fig, ax = plt.subplots(figsize=(8.6, 3.8))
    def case_label(bias: float, sd: float) -> str:
        if bias == 0 and sd == 0:
            return "exact\nstop"
        if bias == 0:
            return f"either way\nsd {sd:.0f}°"
        word = "mostly past" if bias > 0 else "mostly short"
        return f"{word}\n{bias:+.0f}°, sd {sd:.0f}°"

    labels = [case_label(bias, sd) for bias, sd, *_ in cases]
    saved = [100 * (o - n) / o for _, _, o, n, _ in cases]
    xs = list(range(len(cases)))
    ax.bar(xs, saved, color=C_NEW, width=0.6)
    for x, s_, c in zip(xs, saved, cases):
        ax.text(x, s_ + 0.3, f"{s_:.1f}%", ha="center", va="bottom", fontsize=9, color=INK)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, fontsize=8.5, color=MUTED)
    ax.set_ylabel("time saved by Detour Steer (%)", fontsize=9, color=MUTED)
    ax.set_ylim(0, max(saved) * 1.2 + 1)
    ax.set_title(f"Unknown stop error, steer {robot.steer_speed_dps:.0f}°/s: "
                 f"never slower on any goal in any case" if all(c[4] == 0 for c in cases) else
                 f"Unknown stop error, steer {robot.steer_speed_dps:.0f}°/s",
                 fontsize=10.5, color=INK, loc="left")
    _style(ax)
    ax.grid(False, axis="x")
    fig.tight_layout()
    fig.savefig(os.path.join(cfg.output_dir, "fig_noise.png"), dpi=160, bbox_inches="tight")
    plt.close(fig)
    run_speed_sweep(cfg, goals)


def run_speed_sweep(cfg: HWConfig, goals) -> None:
    """Other speeds: the same goals and stop errors with a faster or slower robot."""
    nz = cfg.noise
    b = cfg.bench
    print(f"\n3c. other speeds: time saved by Detour Steer (stop error bias {nz.bias_deg:+.0f} deg, sd {nz.sd_deg:.0f} deg)")
    print(f"{'drive m/s':>9} | " + " ".join(f"{w:>8.0f}" for w in b.sweep_steer_dps) + "   <- steer deg/s")
    table = []
    for v in b.sweep_drive_mps:
        row = []
        for w in b.sweep_steer_dps:
            robot = RobotConfig(**{**cfg.robot.__dict__, "drive_speed_mps": v, "steer_speed_dps": w})
            o, n, _, _ = compare(goals, robot, nz)
            row.append(100.0 * (o - n) / o)
        table.append(row)
        print(f"{v:>9.2f} | " + " ".join(f"{s:>7.1f}%" for s in row))

    fig, ax = plt.subplots(figsize=(8.6, 3.8))
    xs = list(range(len(b.sweep_steer_dps)))
    width = 0.8 / len(b.sweep_drive_mps)
    colors = [C_NEW, "#7fa8d8", "#c3d5ec"]
    for i, (v, row) in enumerate(zip(b.sweep_drive_mps, table)):
        pos = [x + (i - (len(b.sweep_drive_mps) - 1) / 2) * width for x in xs]
        label = f"drive {v:.2f} m/s" + (" (the robot)" if abs(v - cfg.robot.drive_speed_mps) < 1e-9 else "")
        ax.bar(pos, row, width=width * 0.95, color=colors[i % len(colors)], label=label)
        for p, s_ in zip(pos, row):
            ax.text(p, s_ + 0.2, f"{s_:.1f}", ha="center", va="bottom", fontsize=7.5, color=INK)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"steer {w:.0f}°/s" + ("\n(the robot)" if abs(w - cfg.robot.steer_speed_dps) < 1e-9 else "")
                        for w in b.sweep_steer_dps], fontsize=8.5, color=MUTED)
    ax.set_ylabel("time saved by Detour Steer (%)", fontsize=9, color=MUTED)
    ax.legend(frameon=False, fontsize=8.5, loc="upper right")
    ax.set_title(f"Other speeds: the same {len(goals)} goals, stop error sd {nz.sd_deg:.0f}°",
                 fontsize=10.5, color=INK, loc="left")
    _style(ax)
    ax.grid(False, axis="x")
    fig.tight_layout()
    fig.savefig(os.path.join(cfg.output_dir, "fig_speeds.png"), dpi=160, bbox_inches="tight")
    plt.close(fig)


# =====================================================================
# Entry point
# =====================================================================


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-bench", action="store_true", help="skip the random-goal benchmark")
    parser.add_argument("--comparison", action="store_true", help="show the measured-profile 2 m / 7-leg MODEL only")
    args = parser.parse_args()
    if args.comparison:
        print('IDEAL MODEL ONLY: CCW steering, 2 m total / 7 legs, 0.03 m/s, 35 deg/s')
        for name, detour in [('Direct', False), ('Detour', True)]:
            result = ideal_route(comparison_route(), use_detour=detour)
            print(f"{name}: {result['time_s']:.2f} s, {result['length_m']:.3f} m, turn {result['turn_deg']:.1f} deg, {result['detours']} detours")
        return

    cfg = HWConfig()
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(cfg.output_dir, exist_ok=True)
    print(describe(cfg))

    verify(cfg)

    print("\n============================================================")
    print(f"2. Examples (goal {EXAMPLE_D_M} m away, the robot test distance)")
    print("============================================================")
    for name, phi_deg, errs in (("6 deg to the right (robot test)", 354.0, []),
                                ("60 deg, stops 8 deg too far", 60.0, [8.0])):
        phi = math.radians(phi_deg)
        goal = (EXAMPLE_D_M * math.cos(phi), EXAMPLE_D_M * math.sin(phi))
        plan = plan_leg(phi, EXAMPLE_D_M, cfg.robot)
        new = run_goal(goal, cfg.robot, fixed_stop_errors(errs), True)
        old = run_goal(goal, cfg.robot, fixed_stop_errors(errs), False)
        print(f"  {name:<28}: first plan {plan.kind:<6} a={plan.a:.3f} m beta={math.degrees(plan.beta):6.1f} deg "
              f"b={plan.b:.3f} m")
        print(f"  {'':<28}  Detour Steer {new.time_s:6.2f} s (steered {math.degrees(new.steer_turn_rad):4.0f} deg), "
              f"current method {old.time_s:6.2f} s (steered {math.degrees(old.steer_turn_rad):4.0f} deg)")
    plot_examples(cfg, os.path.join(cfg.output_dir, "fig_path.png"))
    plot_decision_map(cfg, os.path.join(cfg.output_dir, "fig_decision_map.png"))
    plot_route_2m(cfg, os.path.join(cfg.output_dir, "fig_route_2m.png"))
    print_robot_like(cfg)

    if not args.no_bench:
        run_benchmark(cfg)
    print(f"\nFigures written to {cfg.output_dir}/")


if __name__ == "__main__":
    main()
