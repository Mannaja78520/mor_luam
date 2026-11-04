"""Detour Steer - simulation on the computer (a tkinter window).

Two copies of the same robot race to the same goals, side by side:
    left  : the old method (steer straight at the goal, as the firmware does)
    right : Detour Steer   (drive on, steer less, come back)
Both get the same goals and the same stop errors, so the only difference is
the planner. The planner is imported from detour_steer.py, so the window runs
exactly the algorithm that the proofs and the benchmark there check.

How to use the window:
    - click on either field to put a goal there (several clicks = a route)
    - or press a button: random goal, random route, example (a), example (b)
    - press Start; the sliders set the robot and the playback speed

Usage:
    python sim_detour_steer.py              # open the window
    python sim_detour_steer.py --selftest   # no window: check the animation
                                            # against detour_steer.run_goal()
"""
import argparse
import math
import os
import random
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from detour_steer import (TAU, NoiseConfig, RealismConfig, RobotConfig, bearing, steer_error,
                          fixed_stop_errors, gaussian_stop_error, in_tolerance,
                          plan_leg, real_steer_time, real_stop_error, real_times, run_goal, unit)

# =====================================================================
# Configuration (dataclasses, course house style)
# =====================================================================


@dataclass(frozen=True)
class ViewConfig:
    canvas_px: int = 410            # each field is a square canvas
    half_width_m: float = 2.5       # the field shows -2.5 .. 2.5 m around the start
    grid_m: float = 0.5
    frame_ms: int = 16              # about 60 frames per second


@dataclass(frozen=True)
class SliderConfig:
    """(label, min, max, start, step) for every slider."""
    steer_dps: Tuple = ("ความเร็วเลี้ยวล้อ ω (°/s)", 5.0, 180.0, 35.0, 1.0)
    drive_mps: Tuple = ("ความเร็ววิ่ง v (m/s)", 0.005, 0.035, 0.03, 0.001)
    stop_s: Tuple = ("เวลาสลับมอเตอร์ t_b (s)", 0.0, 1.0, 0.2, 0.05)
    bias_deg: Tuple = ("ล้อหยุดคลาด เฉลี่ย (°) +เกิน −ไม่ถึง", -3.0, 3.0, 0.0, 0.5)
    sd_deg: Tuple = ("ล้อหยุดคลาด สุ่ม sd (°)", 0.0, 6.0, 2.0, 0.5)
    speed_x: Tuple = ("ความเร็วการเล่น (เท่า)", 1.0, 20.0, 4.0, 1.0)


@dataclass(frozen=True)
class SimConfig:
    view: ViewConfig = field(default_factory=ViewConfig)
    sliders: SliderConfig = field(default_factory=SliderConfig)
    settle_s: float = 0.05          # STEER_SETTLE_MS = 50
    tol_deg: float = 3.5            # Wheel_STEER_ERROR_TOLERANCE
    seed: int = 2026


# =====================================================================
# The robot, moved in small time steps so it can be drawn
# =====================================================================


@dataclass
class Phase:
    kind: str            # "steer", "settle", "drive", "stop"
    left: float          # rad, s or m still to go
    dx: float = 0.0      # drive direction (unit vector)
    dy: float = 0.0
    label: str = ""
    real: bool = False   # robot-like drive: drift, early re-aim, coast at the end
    qx: float = 0.0      # where this leg should end
    qy: float = 0.0
    rate: float = 0.0    # heading drift, rad per metre (+ = turns right)
    moved: float = 0.0


PHASE_TEXT = {"steer": "เลี้ยวล้อ (ทวนเข็ม)", "settle": "รอล้อนิ่ง",
              "drive": "วิ่ง", "stop": "หยุด สลับมอเตอร์"}


class SimRobot:
    """One robot. step(dt) moves it dt seconds; the decisions are the same as
    detour_steer.run_goal(), so the finishing time is the same too."""

    def __init__(self, use_detour: bool):
        self.use_detour = use_detour
        self.real: Optional[RealismConfig] = None     # set = robot-like mode (drift, slip, coast)
        self.reset([], [], [])

    def reset(self, goals, seeds, scripted_deg) -> None:
        self.goals = list(goals)
        self.seeds = list(seeds)
        self.scripted = [math.radians(e) for e in scripted_deg]
        self.x = self.y = self.h = 0.0
        self.t = 0.0
        self.steered = 0.0
        self.n_steers = 0
        self.n_detours = 0
        self.queue: List[Phase] = []
        self.goal_i = 0
        self.decisions = 0
        self.trail = [(0.0, 0.0)]
        self.steer_start: Optional[float] = None
        self.plan_drawn = None          # (x, y, h, a, goal) of the last detour, for the dashed line
        self.plan_text = ""
        self.detour_goal = None         # index of the goal whose detour is in progress
        self.decision = None            # (phi, d, robot) of the last plan, for the info panel
        self.pending_stop = None        # where the wheel will stop vs the target: unknown to the robot
        self.last_stop = None           # ... until the wheel has stopped and the angle is read
        self.driven = False             # robot-like mode: a steer after a drive pays t_b
        self.done = not self.goals
        self._start_goal()

    def _start_goal(self) -> None:
        if self.goal_i < len(self.goals):
            self.rng = random.Random(self.seeds[self.goal_i])
            self.decisions = 0

    def _stop_error(self, noise: NoiseConfig) -> float:
        if self.scripted:
            return self.scripted.pop(0)
        return self.rng.gauss(math.radians(noise.bias_deg), math.radians(noise.sd_deg))

    def _real_drive(self, dist: float, label: str) -> Phase:
        real = self.real
        return Phase("drive", dist + 0.05, label=label, real=True,
                     qx=self.x + dist * math.cos(self.h), qy=self.y + dist * math.sin(self.h),
                     rate=math.radians(self.rng.gauss(real.drift_deg_per_m, real.drift_sd_deg_per_m)))

    def _decide_real(self, robot: RobotConfig) -> None:
        """Robot-like mode: the same decisions as detour_steer.run_route_real()."""
        real = self.real
        gx, gy = self.goals[self.goal_i]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        if dist <= robot.goal_radius_m or self.decisions >= robot.max_phases:
            self.goal_i += 1
            self.done = self.goal_i >= len(self.goals)
            self._start_goal()
            return
        self.decisions += 1
        e = steer_error(bearing(dx, dy), self.h)
        self.decision = (e, dist, robot)
        if in_tolerance(e, math.radians(robot.tol_deg)):
            self.queue = [self._real_drive(dist, "วิ่งเข้าเป้า")]
            return
        if self.use_detour:
            plan = plan_leg(e, dist, robot)
            if plan.kind == "detour":
                self.queue = [self._real_drive(plan.a, "วิ่งขาแรก (ทางอ้อม)")]
                self.n_detours += 1
                self.plan_drawn = (self.x, self.y, self.h, plan.a, (gx, gy))
                self.plan_text = (f"ทางอ้อม: วิ่ง a = {plan.a:.2f} m → เลี้ยว β* = {math.degrees(plan.beta):.0f}° "
                                  f"→ วิ่ง b = {plan.b:.2f} m")
                self.detour_goal = self.goal_i
                return
            self.plan_text = f"เลี้ยว {math.degrees(e):.0f}° แล้ววิ่ง {dist:.2f} m (วางแผนใหม่จากตำแหน่งจริง)"
        else:
            self.plan_text = f"เลี้ยวตรง: เลี้ยว {math.degrees(e):.0f}° แล้ววิ่ง {dist:.2f} m"
        turn = max(0.0, e + real_stop_error(real, self.rng, math.radians(robot.tol_deg)))
        self.pending_stop = turn - e
        # measured steering time: the wheel animates at real.steer_dps, the rest
        # (motor start, slow-down, the wait for the IMU) is shown as "settle"
        total = real_steer_time(real, turn, self.rng, self.driven)
        rest = max(0.0, total - math.degrees(turn) / real.steer_dps)
        self.queue = [Phase("steer", turn), Phase("settle", rest)]
        self.driven = False
        self.n_steers += 1

    def _step_real_drive(self, ph: Phase, dt: float, robot: RobotConfig) -> float:
        """Move a robot-like drive by up to dt seconds; returns the seconds used."""
        real, v = self.real, robot.drive_speed_mps
        used = 0.0
        while used < dt - 1e-12:
            ex, ey = ph.qx - self.x, ph.qy - self.y
            f = ex * math.cos(self.h) + ey * math.sin(self.h)
            s = -ex * math.sin(self.h) + ey * math.cos(self.h)
            end = (math.hypot(ex, ey) <= robot.goal_radius_m or f <= 0.0 or ph.moved >= ph.left
                   or (s > robot.goal_radius_m and f <= max(real.reaim_min_m, real.reaim_ratio * s)))
            if end:
                coast = max(-ph.moved, self.rng.gauss(real.drive_bias_m, real.drive_sd_m))
                self.x += coast * math.cos(self.h)
                self.y += coast * math.sin(self.h)
                self.t += coast / v
                self.h %= TAU
                ph.left = 0.0
                self.driven = True
                return used
            step = min(0.002, v * (dt - used))
            self.h -= ph.rate * step / 2
            self.x += step * math.cos(self.h)
            self.y += step * math.sin(self.h)
            self.h -= ph.rate * step / 2
            ph.moved += step
            if int(ph.moved / 0.01) != int((ph.moved - step) / 0.01):
                self.trail.append((self.x, self.y))       # draw the curve every centimetre
            self.t += step / v
            used += step / v
        return used

    def _decide(self, robot: RobotConfig, noise: NoiseConfig) -> None:
        if self.real is not None:
            self._decide_real(robot)
            return
        gx, gy = self.goals[self.goal_i]
        dx, dy = gx - self.x, gy - self.y
        dist = math.hypot(dx, dy)
        if dist <= robot.goal_radius_m or self.decisions >= robot.max_phases:
            self.goal_i += 1
            self.done = self.goal_i >= len(self.goals)
            self._start_goal()
            return
        self.decisions += 1
        e = steer_error(bearing(dx, dy), self.h)
        self.decision = (e, dist, robot)
        if in_tolerance(e, math.radians(robot.tol_deg)):
            # drive along the wheel's REAL heading (up to tol off); a miss
            # bigger than goal_radius is planned again, as on the robot
            ux, uy = unit(self.h)
            self.queue = [Phase("drive", dist, ux, uy, "วิ่งเข้าเป้า")]
            return
        if self.use_detour:
            plan = plan_leg(e, dist, robot)
            if plan.kind == "detour":
                ux, uy = unit(self.h)
                self.queue = [Phase("drive", plan.a, ux, uy, "วิ่งขาแรก (ทางอ้อม)"),
                              Phase("stop", robot.stop_s)]
                self.n_detours += 1
                self.plan_drawn = (self.x, self.y, self.h, plan.a, (gx, gy))
                self.plan_text = (f"ทางอ้อม: วิ่ง a = {plan.a:.2f} m → เลี้ยว β* = {math.degrees(plan.beta):.0f}° "
                                  f"→ วิ่ง b = {plan.b:.2f} m")
                self.detour_goal = self.goal_i
                return
            if getattr(self, "detour_goal", None) == self.goal_i:
                # leg 2 of the detour just planned (theorem 4: same plan as before)
                self.plan_text = (f"ทางอ้อม ขาที่ 2: เลี้ยว {math.degrees(e):.0f}° แล้ววิ่ง {dist:.2f} m "
                                  f"(วางแผนใหม่ ได้แผนเดิม)")
            else:
                self.plan_text = f"เลี้ยวตรงดีที่สุด: เลี้ยว {math.degrees(e):.0f}° แล้ววิ่ง {dist:.2f} m"
        turn = max(0.0, e + self._stop_error(noise))
        self.pending_stop = turn - e
        self.queue = [Phase("steer", turn), Phase("settle", robot.settle_s)]
        self.n_steers += 1

    def step(self, dt: float, robot: RobotConfig, noise: NoiseConfig) -> None:
        w = math.radians(self.real.steer_dps if self.real is not None and self.real.steer_dps > 0
                         else robot.steer_speed_dps)
        v = robot.drive_speed_mps
        while dt > 1e-12 and not self.done:
            if not self.queue:
                self._decide(robot, noise)
                continue
            ph = self.queue[0]
            if ph.kind == "drive" and ph.real:
                dt -= self._step_real_drive(ph, dt, robot)
                if ph.left <= 0.0:
                    self.queue.pop(0)
                    self.trail.append((self.x, self.y))
                continue
            if ph.kind == "steer":
                if self.steer_start is None:
                    self.steer_start = self.h
                use = min(dt, ph.left / w)
                turn = w * use
                self.h = (self.h + turn) % TAU
                self.steered += turn
            elif ph.kind == "drive":
                use = min(dt, ph.left / v)
                turn = v * use
                self.x += turn * ph.dx
                self.y += turn * ph.dy
            else:
                use = min(dt, ph.left)
                turn = use
            ph.left -= turn
            self.t += use
            dt -= use
            if ph.left <= 1e-9:
                self.queue.pop(0)
                if ph.kind == "steer":
                    self.steer_start = None
                    self.last_stop = self.pending_stop      # read from the AS5600 now
                elif ph.kind == "drive":
                    self.trail.append((self.x, self.y))

    @property
    def stop_note(self) -> str:
        """Where the wheel stopped last time, known only after it stopped."""
        if self.last_stop is None:
            return ""
        deg = math.degrees(self.last_stop)
        if abs(deg) < 0.05:
            return "ล้อหยุดตรงเป้า"
        return f"ล้อหยุด{'เกิน' if deg > 0 else 'ไม่ถึง'} {abs(deg):.1f}°"

    @property
    def status(self) -> str:
        if self.done:
            return "ถึงเป้าครบแล้ว" if self.goals else "ยังไม่มีเป้า"
        if not self.queue:
            return "วางแผน"
        ph = self.queue[0]
        return ph.label or PHASE_TEXT[ph.kind]


# =====================================================================
# Self-test: the animation must finish at the same time as run_goal()
# =====================================================================


def run_to_end(sim: SimRobot, robot: RobotConfig, noise: NoiseConfig, dt: float = 0.037) -> float:
    for _ in range(1_000_000):
        if sim.done:
            break
        sim.step(dt, robot, noise)
    return sim.t


def selftest(cfg: SimConfig) -> bool:
    robot = RobotConfig()
    exact = NoiseConfig(0.0, 0.0)
    print("Self-test: the animated robot vs detour_steer.run_goal()")
    for name, phi_deg, errs in (("(a) goal 0.30 m, 6 deg to the right", 354.0, []),
                                ("(b) goal 0.30 m at 60 deg, stops 8 deg too far", 60.0, [8.0])):
        phi = math.radians(phi_deg)
        goal = (0.3 * math.cos(phi), 0.3 * math.sin(phi))
        row = []
        for use_detour in (False, True):
            sim = SimRobot(use_detour)
            sim.reset([goal], [0], errs)
            t_sim = run_to_end(sim, robot, exact)
            t_ref = run_goal(goal, robot, fixed_stop_errors(errs), use_detour).time_s
            row.append((t_sim, t_ref))
        print(f"  {name:<40} old {row[0][0]:6.2f} s (ref {row[0][1]:6.2f})   "
              f"Detour Steer {row[1][0]:6.2f} s (ref {row[1][1]:6.2f})")
    noise = NoiseConfig(2.0, 3.0)
    rng = random.Random(cfg.seed)
    worst = 0.0
    for _ in range(300):
        phi, d, seed = rng.uniform(0, TAU), rng.uniform(0.3, 2.0), rng.randrange(2 ** 31)
        goal = (d * math.cos(phi), d * math.sin(phi))
        for use_detour in (False, True):
            sim = SimRobot(use_detour)
            sim.reset([goal], [seed], [])
            t_sim = run_to_end(sim, robot, noise)
            t_ref = run_goal(goal, robot, gaussian_stop_error(noise, random.Random(seed)), use_detour).time_s
            worst = max(worst, abs(t_sim - t_ref))
    ok = worst < 1e-6
    print(f"  300 random goals with stop error, both methods: worst |sim - ref| = {worst:.1e} s"
          f"  -> {'PASS' if ok else 'FAIL'}")
    return ok


# =====================================================================
# The window
# =====================================================================

BG = "#f7f7f5"
FIELD = "#fcfcfb"
GRID = "#e4e4e0"
INK = "#222222"
MUTED = "#6b6b66"
C_OLD = "#eb6834"         # orange: the old method, as in the figures
C_NEW = "#2a78d6"         # blue:   Detour Steer
C_NEW_LIGHT = "#9ec5f4"


class Field:
    """One side: title, canvas, two status lines."""

    def __init__(self, parent, app, title: str, color: str, sim: SimRobot, col: int):
        import tkinter as tk
        self.app, self.sim, self.color = app, sim, color
        px = app.px
        box = tk.Frame(parent, bg=BG)
        box.grid(row=0, column=col, padx=8, sticky="n")
        head = tk.Frame(box, bg=BG)
        head.pack(fill="x")
        tk.Frame(head, bg=color, width=16, height=4).pack(side="left", padx=(0, 8), pady=8)
        tk.Label(head, text=title, bg=BG, fg=INK, font=app.f_head).pack(side="left")
        self.canvas = tk.Canvas(box, width=px, height=px, bg=FIELD, highlightthickness=1,
                                highlightbackground=GRID, cursor="crosshair")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_click)
        self.status = tk.Label(box, text="", bg=BG, fg=INK, font=app.f_body, anchor="w", justify="left")
        self.status.pack(fill="x", pady=(4, 0))
        self.plan = tk.Label(box, text="", bg=BG, fg=MUTED, font=app.f_small, anchor="w", justify="left",
                             wraplength=px)
        self.plan.pack(fill="x")

    # world (m, y up) <-> canvas (px, y down)
    def to_px(self, x: float, y: float) -> Tuple[float, float]:
        px, s = self.app.px, self.app.px / (2 * self.app.cfg.view.half_width_m)
        return px / 2 + x * s, px / 2 - y * s

    def to_world(self, px: float, py: float) -> Tuple[float, float]:
        size, s = self.app.px, self.app.px / (2 * self.app.cfg.view.half_width_m)
        return (px - size / 2) / s, (size / 2 - py) / s

    def on_click(self, event) -> None:
        self.app.add_goal(*self.to_world(event.x, event.y))

    def draw(self) -> None:
        c, v, sim, size, k = self.canvas, self.app.cfg.view, self.sim, self.app.px, self.app.k
        c.delete("all")
        s = size / (2 * v.half_width_m)
        n = int(v.half_width_m / v.grid_m)
        for i in range(-n, n + 1):
            x0, _ = self.to_px(i * v.grid_m, 0)
            _, y0 = self.to_px(0, i * v.grid_m)
            c.create_line(x0, 0, x0, size, fill=GRID)
            c.create_line(0, y0, size, y0, fill=GRID)
            if i and i % 2 == 0:
                c.create_text(x0, size - 8 * k, text=f"{i * v.grid_m:g} m", fill=MUTED, font=self.app.f_small)
        # start
        sx, sy = self.to_px(0, 0)
        c.create_oval(sx - 4 * k, sy - 4 * k, sx + 4 * k, sy + 4 * k, outline=MUTED)
        # goals
        for n_goal, (gx, gy) in enumerate(self.app.goals, 1):
            px, py = self.to_px(gx, gy)
            reached = n_goal <= sim.goal_i
            c.create_text(px, py, text="★", fill=MUTED if reached else INK, font=self.app.f_star)
            c.create_text(px + 12 * k, py - 10 * k, text=str(n_goal), fill=MUTED, font=self.app.f_small)
        if not self.app.goals:
            c.create_text(size / 2, size / 2 - 40 * k, fill=MUTED, font=self.app.f_body, justify="center",
                          text="ยังไม่มีเป้า\nคลิกบนพื้นที่นี้เพื่อวางเป้า (วางได้หลายจุด)\n"
                               "หรือกด สุ่มเป้า / ตัวอย่าง ด้านล่าง")
        # the planned detour, dashed
        if sim.plan_drawn and not sim.done:
            x0, y0, h0, a, (gx, gy) = sim.plan_drawn
            ux, uy = unit(h0)
            pts = [self.to_px(x0, y0), self.to_px(x0 + a * ux, y0 + a * uy), self.to_px(gx, gy)]
            c.create_line(*[q for p in pts for q in p], fill=C_NEW_LIGHT, dash=(5, 4), width=max(1, round(2 * k)))
        # trail
        pts = [self.to_px(x, y) for x, y in sim.trail] + [self.to_px(sim.x, sim.y)]
        if len(pts) >= 2:
            c.create_line(*[q for p in pts for q in p], fill=self.color, width=max(1, round(3 * k)), capstyle="round")
        # robot body + the wheel heading arrow
        rx, ry = self.to_px(sim.x, sim.y)
        r = 0.09 * s
        c.create_oval(rx - r, ry - r, rx + r, ry + r, fill="white", outline=INK, width=max(1, round(2 * k)))
        ux, uy = unit(sim.h)
        hx, hy = self.to_px(sim.x + 0.26 * ux, sim.y + 0.26 * uy)
        c.create_line(rx, ry, hx, hy, fill=self.color, width=max(1, round(3 * k)), arrow="last",
                      arrowshape=(10 * k, 12 * k, 4 * k))
        # the counter-clockwise turn so far, while steering
        if sim.steer_start is not None:
            turned = (sim.h - sim.steer_start) % TAU
            rr = 0.17 * s
            extent = math.degrees(turned)
            if extent >= 359.0:
                c.create_oval(rx - rr, ry - rr, rx + rr, ry + rr, outline=self.color, width=max(1, round(2 * k)))
            elif extent > 0.5:
                c.create_arc(rx - rr, ry - rr, rx + rr, ry + rr, start=math.degrees(sim.steer_start),
                             extent=extent, style="arc", outline=self.color, width=max(1, round(2 * k)))
            c.create_text(rx, ry - rr - 10 * k, text=f"{math.degrees(turned):.0f}°", fill=INK, font=self.app.f_small)
        # status lines
        note = f"   ({sim.stop_note})" if sim.stop_note else ""
        self.status.config(text=f"เวลา {sim.t:6.2f} s   เลี้ยวรวม {math.degrees(sim.steered):5.0f}°   "
                                f"สถานะ: {sim.status}{note}")
        self.plan.config(text=sim.plan_text if sim.use_detour else "เลี้ยวตรงไปหาเป้าเสมอ (state machine เดิมใน main.cpp)")


class App:
    def __init__(self, cfg: SimConfig):
        import tkinter as tk
        import tkinter.font as tkfont
        self.cfg = cfg
        try:            # crisp text on a scaled Windows screen, and Thai marks drawn right
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
        self.root = tk.Tk()
        self.k = self.root.winfo_fpixels("1i") / 96.0          # 1.0 at 100 %, 1.75 at 175 %
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        want = cfg.view.canvas_px * self.k
        self.px = int(max(220, min(want, sh - 380 * self.k, (sw - 80 * self.k) / 2)))
        self.root.title("Detour Steer — ซิมบนคอม")
        self.root.configure(bg=BG)
        fams = set(tkfont.families())
        fam = next((f for f in ("Leelawadee UI", "Tahoma", "Segoe UI") if f in fams), "TkDefaultFont")
        self.f_small = (fam, 9)
        self.f_body = (fam, 10)
        self.f_head = (fam, 11, "bold")
        self.f_big = (fam, 13, "bold")
        self.f_star = (fam, 15)

        self.goals: List[Tuple[float, float]] = []
        self.seeds: List[int] = []
        self.scripted: List[float] = []
        self.rng = random.Random(cfg.seed)
        self.running = False
        self.old = SimRobot(use_detour=False)
        self.new = SimRobot(use_detour=True)

        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=12, pady=(10, 4))
        # two short lines: Tk on Windows can split one very long Thai line between a
        # consonant and its tone mark, which draws the mark on a dotted circle
        for line in ("หุ่นมอเตอร์ตัวเดียว: ล้อเลี้ยวได้ทวนเข็มอย่างเดียว ต้องหยุดก่อนแล้วค่อยเลี้ยว",
                     "หุ่น 2 ตัวไปเป้าเดียวกัน และล้อหยุดคลาดแบบเดียวกันทุกครั้ง",
                     "ล้อหยุดเกินหรือไม่ถึงไม่รู้ล่วงหน้า หุ่นรู้มุมจริงหลังล้อหยุดเท่านั้น"):
            tk.Label(top, text=line, bg=BG, fg=INK, font=self.f_body).pack(anchor="w")

        arena = tk.Frame(self.root, bg=BG)
        arena.pack(padx=4)
        self.fields = [Field(arena, self, "วิธีเดิม — เลี้ยวตรงไปหาเป้า", C_OLD, self.old, 0),
                       Field(arena, self, "Detour Steer — วิ่งเลย เลี้ยวน้อยลง วิ่งกลับ", C_NEW, self.new, 1)]

        self.result = tk.Label(self.root, text="", bg=BG, fg=INK, font=self.f_big)
        self.result.pack(pady=(6, 2))
        self.avg = tk.Label(self.root, text="", bg=BG, fg=MUTED, font=self.f_body)
        self.avg.pack(pady=(0, 2))
        self.avg_cache = {}

        ctl = tk.Frame(self.root, bg=BG)
        ctl.pack(padx=12, pady=(2, 4), fill="x")
        self.btn_run = tk.Button(ctl, text="▶ เริ่ม", width=12, command=self.toggle_run, font=self.f_body,
                                 bg=C_NEW, fg="white", activebackground="#256abf", activeforeground="white",
                                 relief="flat", padx=6, pady=3)
        self.btn_run.pack(side="left", padx=(0, 8))
        for text, cmd in (("↺ เริ่มใหม่", self.restart), ("ล้างเป้า", self.clear_goals),
                          ("สุ่มเป้า", self.random_goal), ("สุ่มเส้นทาง 5 จุด", self.random_route),
                          ("ตัวอย่าง (a): 0.3 m ขวา 6°", self.example_a),
                          ("ตัวอย่าง (b): ล้อเลยเป้า 8°", self.example_b),
                          ("2 m / 7 ช่วง", self.example_comparison),
                          ("สุ่มความคลาดใหม่", self.reseed)):
            tk.Button(ctl, text=text, command=cmd, font=self.f_body, relief="groove",
                      padx=6, pady=3).pack(side="left", padx=3)

        sl = tk.Frame(self.root, bg=BG)
        sl.pack(padx=12, pady=(4, 12), fill="x")
        self.vars = {}
        for i, key in enumerate(("steer_dps", "drive_mps", "stop_s", "bias_deg", "sd_deg", "speed_x")):
            label, lo, hi, start, step = getattr(cfg.sliders, key)
            var = tk.DoubleVar(value=start)
            self.vars[key] = var
            cell = tk.Frame(sl, bg=BG)
            cell.grid(row=i // 3, column=i % 3, padx=10, pady=2, sticky="w")
            tk.Label(cell, text=label, bg=BG, fg=INK, font=self.f_small, width=24, anchor="w").pack(anchor="w")
            tk.Scale(cell, from_=lo, to=hi, resolution=step, orient="horizontal", variable=var, length=int(250 * self.k),
                     bg=BG, fg=INK, highlightthickness=0, troughcolor=GRID, font=self.f_small,
                     command=lambda _v: self.redraw()).pack(anchor="w")
        self.real_on = tk.BooleanVar(value=False)
        tk.Checkbutton(sl, text="แบบใกล้หุ่นจริง: เบี่ยง ไถล ล้อไหล (สุ่มใหม่ทุกครั้งที่กดเริ่ม)",
                       variable=self.real_on, command=self.toggle_real, bg=BG, fg=INK,
                       activebackground=BG, font=self.f_small).grid(row=2, column=0, columnspan=3, sticky="w", padx=10)

        self.redraw()
        self.root.after(cfg.view.frame_ms, self.tick)

    # ---- robot-like mode -------------------------------------------------
    def toggle_real(self) -> None:
        real = RealismConfig() if self.real_on.get() else None
        for sim in self.all_sims():
            sim.real = real
        self._reset_robots()

    def all_sims(self):
        return (self.old, self.new)

    def show_average(self) -> None:
        """Mean and 5-95 % range of 1000 robot-like runs on the current goals."""
        if not (self.real_on.get() and self.goals):
            self.avg.config(text="")
            return
        robot = self.robot_cfg()
        key = (tuple(self.goals), robot)
        if key not in self.avg_cache:
            self.avg.config(text="กำลังคำนวณค่าเฉลี่ย 1000 รอบ…")
            self.root.update_idletasks()
            self.avg_cache[key] = real_times(self.goals, robot, RealismConfig(), n=1000)
        res = self.avg_cache[key]
        self.avg.config(text=self.average_text(res))

    def average_text(self, res) -> str:
        o, n = res["old"], res["detour"]
        return (f"เฉลี่ย 1000 รอบ (แบบใกล้หุ่นจริง): วิธีเดิม {o[0]:.1f} s ({o[1]:.1f}–{o[2]:.1f})  |  "
                f"Detour {n[0]:.1f} s ({n[1]:.1f}–{n[2]:.1f})  →  เร็วกว่า {100 * (o[0] - n[0]) / o[0]:.1f}%")

    # ---- settings from the sliders -------------------------------------
    def robot_cfg(self) -> RobotConfig:
        g = lambda k: float(self.vars[k].get())
        return RobotConfig(drive_speed_mps=g("drive_mps"), steer_speed_dps=g("steer_dps"),
                           settle_s=self.cfg.settle_s, stop_s=g("stop_s"), tol_deg=self.cfg.tol_deg, goal_radius_m=0.0025)

    def noise_cfg(self) -> NoiseConfig:
        return NoiseConfig(float(self.vars["bias_deg"].get()), float(self.vars["sd_deg"].get()))

    # ---- goals -----------------------------------------------------------
    def _reset_robots(self) -> None:
        self.running = False
        self.btn_run.config(text="▶ เริ่ม")
        for sim in (self.old, self.new):
            sim.reset(self.goals, self.seeds, self.scripted)
        self.result.config(text="พร้อม — กด ▶ เริ่ม" if self.goals else "")
        self.redraw()

    def add_goal(self, x: float, y: float) -> None:
        if self.running:
            self.result.config(text="หยุดก่อน แล้วค่อยวางเป้าเพิ่ม")
            return
        self.goals.append((x, y))
        self.seeds.append(self.rng.randrange(2 ** 31))
        self._reset_robots()

    def clear_goals(self) -> None:
        self.goals, self.seeds, self.scripted = [], [], []
        self._reset_robots()

    def random_goal(self) -> None:
        self.running = False
        phi, d = self.rng.uniform(0, TAU), self.rng.uniform(0.5, 2.0)
        self.scripted = []
        self.add_goal(d * math.cos(phi), d * math.sin(phi))

    def random_route(self) -> None:
        self.goals, self.seeds, self.scripted = [], [], []
        for _ in range(5):
            self.goals.append((self.rng.uniform(-2.0, 2.0), self.rng.uniform(-2.0, 2.0)))
            self.seeds.append(self.rng.randrange(2 ** 31))
        self._reset_robots()

    def _example(self, phi_deg: float, scripted: List[float]) -> None:
        phi = math.radians(phi_deg)
        d = 0.3   # the robot test distance
        self.goals, self.seeds, self.scripted = [(d * math.cos(phi), d * math.sin(phi))], [0], scripted
        # the examples in the README use exact stops and the default robot
        for key, val in (("steer_dps", 35.0), ("drive_mps", 0.03), ("stop_s", 0.2),
                         ("bias_deg", 0.0), ("sd_deg", 0.0)):
            self.vars[key].set(val)
        self._reset_robots()

    def example_a(self) -> None:
        self._example(354.0, [])

    def example_comparison(self) -> None:
        from detour_steer import comparison_route
        self.goals = comparison_route()
        self.seeds, self.scripted = [0] * len(self.goals), []
        for key, val in (("steer_dps", 35.0), ("drive_mps", 0.03), ("stop_s", 0.2),
                         ("bias_deg", 0.0), ("sd_deg", 0.0)):
            self.vars[key].set(val)
        self._reset_robots()

    def example_b(self) -> None:
        self._example(60.0, [8.0])

    def reseed(self) -> None:
        self.seeds = [self.rng.randrange(2 ** 31) for _ in self.goals]
        self._reset_robots()

    # ---- running ---------------------------------------------------------
    def toggle_run(self) -> None:
        if not self.goals:
            self.result.config(text="ยังไม่มีเป้า — คลิกบนพื้นที่ หรือกด สุ่มเป้า")
            return
        if self.old.done and self.new.done:
            if self.real_on.get():                     # robot-like: a new random run every press
                self.seeds = [self.rng.randrange(2 ** 31) for _ in self.goals]
            self._reset_robots()
        self.running = not self.running
        self.btn_run.config(text="⏸ หยุดชั่วคราว" if self.running else "▶ เล่นต่อ")
        if self.running:
            self.result.config(text="กำลังวิ่ง…")
            self.show_average()

    def restart(self) -> None:
        self._reset_robots()

    def tick(self) -> None:
        if self.running:
            dt = self.cfg.view.frame_ms / 1000.0 * float(self.vars["speed_x"].get())
            robot, noise = self.robot_cfg(), self.noise_cfg()
            for sim in (self.old, self.new):
                sim.step(dt, robot, noise)
            if self.old.done and self.new.done:
                self.running = False
                self.btn_run.config(text="▶ เริ่ม")
                self.show_result()
            self.redraw()
        self.root.after(self.cfg.view.frame_ms, self.tick)

    def show_result(self) -> None:
        t0, t1 = self.old.t, self.new.t
        if abs(t0 - t1) < 1e-6:
            text = (f"เท่ากัน {t1:.2f} s — เป้านี้เลี้ยวตรงดีที่สุด Detour Steer จึงเลือกวิธีเดิมเอง")
        else:
            pct = 100.0 * (t0 - t1) / t0
            word = "เร็วกว่า" if pct > 0 else "ช้ากว่า"
            text = (f"วิธีเดิม {t0:.2f} s  |  Detour Steer {t1:.2f} s  →  {word} {abs(pct):.1f}%   "
                    f"(เลี้ยวรวม {math.degrees(self.old.steered):.0f}° vs {math.degrees(self.new.steered):.0f}°)")
        self.result.config(text=text)

    def redraw(self) -> None:
        for f in self.fields:
            f.draw()

    def screenshot(self, path: str) -> None:
        """Run example (a) at speed, then save the window as a PNG."""
        from PIL import ImageGrab
        self.root.attributes("-topmost", True)
        self.example_a()
        self.vars["speed_x"].set(20.0)
        self.toggle_run()

        def grab():
            if self.running:
                self.root.after(200, grab)
                return
            self.root.update()
            x, y = self.root.winfo_rootx(), self.root.winfo_rooty()
            w, h = self.root.winfo_width(), self.root.winfo_height()
            ImageGrab.grab(bbox=(x, y, x + w, y + h)).save(path)
            print(f"saved {path}")
            self.root.destroy()

        self.root.after(500, grab)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--selftest", action="store_true", help="no window: check against run_goal()")
    parser.add_argument("--shot", metavar="PNG", help=argparse.SUPPRESS)   # README screenshot
    args = parser.parse_args()
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    cfg = SimConfig()
    if args.selftest:
        sys.exit(0 if selftest(cfg) else 1)
    app = App(cfg)
    if args.shot:
        app.screenshot(args.shot)
    app.run()


if __name__ == "__main__":
    main()
