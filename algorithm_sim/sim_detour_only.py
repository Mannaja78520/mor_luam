"""Detour Steer - simulation on the computer, the algorithm on its own.

One robot, one field, no second robot to race against. The panel on the right
shows what the algorithm is thinking at every decision: phi, d, k, beta*, a, b
and which rule picked the move (theorems 1-4 in README.md).

The robot and the planner are the same ones as in sim_detour_steer.py and
detour_steer.py, so this window runs exactly the algorithm the proofs check.

Usage:
    python sim_detour_only.py              # open the window
    python sim_detour_only.py --selftest   # no window: check the animation
                                           # against detour_steer.run_goal()
"""
import argparse
import math
import os
import random
import sys
from typing import List, Tuple

from detour_steer import (TAU, NoiseConfig, RobotConfig, detour_legs, fixed_stop_errors,
                          gaussian_stop_error, in_tolerance, plan_leg, run_goal)
from sim_detour_steer import (BG, C_NEW, GRID, INK, MUTED, Field, SimConfig, SimRobot,
                              run_to_end)


# =====================================================================
# What the algorithm is thinking, in words
# =====================================================================


def explain(sim: SimRobot) -> str:
    """The last decision of PlanLeg, step by step, with the rule that decided it."""
    if sim.decision is None:
        return "ยังไม่ได้วางแผน — วางเป้าแล้วกด ▶ เริ่ม"
    e, d, robot = sim.decision
    lines = [f"φ  มุมไปเป้า (ทวนเข็ม) = {math.degrees(e):.1f}°",
             f"d  ระยะถึงเป้า = {d:.2f} m"]
    if sim.stop_note:
        lines.insert(0, f"{sim.stop_note} (อ่านมุมจริงหลังล้อหยุด)")
    if in_tolerance(e, math.radians(robot.tol_deg)):
        lines.append(f"φ อยู่ใน ±{robot.tol_deg:g}° แล้ว → วิ่งเข้าเป้าได้เลย")
        return "\n".join(lines)
    if sim.plan_text.startswith("ทางอ้อม ขาที่ 2"):
        lines.append("ขาที่ 2 ของทางอ้อม: วางแผนใหม่แล้วได้แผนเดิม (ทฤษฎีบท 4)")
        lines.append(f"→ เลี้ยว {math.degrees(e):.0f}° แล้ววิ่ง {d:.2f} m เข้าเป้า")
        return "\n".join(lines)
    if e <= math.pi:
        lines.append("φ ≤ 180° → เลี้ยวตรง φ ดีที่สุด (ทฤษฎีบท 1)")
        return "\n".join(lines)
    w = math.radians(robot.steer_speed_dps)
    k = (d * w / robot.drive_speed_mps) * abs(math.sin(e))
    lines.append(f"k = (d·ω/v)·|sin φ| = {k:.2f}")
    if k >= 2.0:
        lines.append("k ≥ 2 → เลี้ยวตรง φ ดีที่สุด (ทฤษฎีบท 3)")
        return "\n".join(lines)
    beta = TAU - math.acos(k - 1.0)
    lines.append(f"β* = 360° − arccos(k − 1) = {math.degrees(beta):.1f}°")
    if beta >= e:
        lines.append("β* ≥ φ → เลี้ยวตรง φ ดีที่สุด (ทฤษฎีบท 3)")
        return "\n".join(lines)
    a, b = detour_legs(e, d, beta)
    lines.append(f"a = d·sin(β* − φ)/sin β* = {a:.2f} m")
    lines.append(f"b = d·sin φ/sin β* = {b:.2f} m")
    plan = plan_leg(e, d, robot)
    if plan.kind == "detour":
        lines.append(f"→ ทางอ้อม: วิ่ง {a:.2f} m, เลี้ยว {math.degrees(beta):.0f}°, วิ่ง {b:.2f} m")
        lines.append(f"    เวลาตามแผน {plan.time_s:.2f} s")
    else:
        lines.append("ทางอ้อมต้องหยุดสลับมอเตอร์เพิ่ม จึงไม่เร็วขึ้น → เลี้ยวตรง φ")
    return "\n".join(lines)


FORMULAS = ("k  = (d·ω / v)·|sin φ|\n"
            "β* = 360° − arccos(k − 1)\n"
            "     ใช้เมื่อ φ > 180°, k < 2 และ β* < φ\n"
            "a  = d·sin(β* − φ) / sin β*\n"
            "b  = d·sin φ / sin β*")


# =====================================================================
# Self-test: the animation must finish at the same time as run_goal()
# =====================================================================


def selftest(cfg: SimConfig) -> bool:
    robot = RobotConfig()
    print("Self-test: the animated Detour Steer robot vs detour_steer.run_goal()")
    for name, phi_deg, errs in (("(a) goal 1 m, 10 deg to the right", 350.0, []),
                                ("(b) goal at 60 deg, stops 8 deg too far", 60.0, [8.0])):
        phi = math.radians(phi_deg)
        goal = (math.cos(phi), math.sin(phi))
        sim = SimRobot(use_detour=True)
        sim.reset([goal], [0], errs)
        t_sim = run_to_end(sim, robot, NoiseConfig(0.0, 0.0))
        t_ref = run_goal(goal, robot, fixed_stop_errors(errs), True).time_s
        print(f"  {name:<40} {t_sim:6.2f} s (ref {t_ref:6.2f})")
    noise = NoiseConfig(2.0, 3.0)
    rng = random.Random(cfg.seed)
    worst = 0.0
    for _ in range(300):
        phi, d, seed = rng.uniform(0, TAU), rng.uniform(0.3, 2.0), rng.randrange(2 ** 31)
        goal = (d * math.cos(phi), d * math.sin(phi))
        sim = SimRobot(use_detour=True)
        sim.reset([goal], [seed], [])
        t_sim = run_to_end(sim, robot, noise)
        t_ref = run_goal(goal, robot, gaussian_stop_error(noise, random.Random(seed)), True).time_s
        worst = max(worst, abs(t_sim - t_ref))
    ok = worst < 1e-6
    print(f"  300 random goals with stop error: worst |sim - ref| = {worst:.1e} s  -> {'PASS' if ok else 'FAIL'}")
    return ok


# =====================================================================
# The window
# =====================================================================


class OnlyApp:
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
        self.root.title("Detour Steer — ซิมบนคอม")
        self.root.configure(bg=BG)
        self.k = self.root.winfo_fpixels("1i") / 96.0
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        want = 520 * self.k
        self.px = int(max(260, min(want, sh - 360 * self.k, sw - 480 * self.k)))
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
        self.sim = SimRobot(use_detour=True)

        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=12, pady=(10, 4))
        for line in ("หุ่นมอเตอร์ตัวเดียว: ล้อเลี้ยวได้ทวนเข็มอย่างเดียว ต้องหยุดก่อนแล้วค่อยเลี้ยว",
                     "Detour Steer: ถ้าเป้าอยู่ขวานิดเดียว วิ่งเลยไปก่อน เลี้ยวน้อยลง แล้ววิ่งกลับ",
                     "ล้อหยุดเกินหรือไม่ถึงไม่รู้ล่วงหน้า หุ่นรู้มุมจริงหลังล้อหยุดเท่านั้น"):
            tk.Label(top, text=line, bg=BG, fg=INK, font=self.f_body).pack(anchor="w")

        body = tk.Frame(self.root, bg=BG)
        body.pack(padx=4, fill="x")
        self.field = Field(body, self, "Detour Steer", C_NEW, self.sim, 0)

        side = tk.Frame(body, bg=BG)
        side.grid(row=0, column=1, padx=(8, 12), sticky="n")
        tk.Label(side, text="อัลกอริทึมกำลังคิดอะไร (PlanLeg)", bg=BG, fg=INK,
                 font=self.f_head).pack(anchor="w", pady=(8, 4))
        self.info = tk.Label(side, text="", bg="white", fg=INK, font=self.f_body, justify="left",
                             anchor="nw", width=40, height=10, padx=10, pady=8,
                             wraplength=int(330 * self.k),        # a long line wraps, never clips
                             highlightthickness=1, highlightbackground=GRID)
        self.info.pack(anchor="w", fill="x")
        tk.Label(side, text="สูตร", bg=BG, fg=INK, font=self.f_head).pack(anchor="w", pady=(12, 4))
        tk.Label(side, text=FORMULAS, bg="white", fg=INK, font=self.f_body, justify="left", anchor="nw",
                 width=40, padx=10, pady=8, highlightthickness=1,
                 highlightbackground=GRID).pack(anchor="w", fill="x")
        tk.Label(side, text="เส้นประ = แผนทางอ้อม  •  วงโค้งรอบหุ่น = มุมที่เลี้ยวไปแล้ว",
                 bg=BG, fg=MUTED, font=self.f_small).pack(anchor="w", pady=(8, 0))

        self.result = tk.Label(self.root, text="", bg=BG, fg=INK, font=self.f_big)
        self.result.pack(pady=(6, 2))

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
            tk.Scale(cell, from_=lo, to=hi, resolution=step, orient="horizontal", variable=var,
                     length=int(250 * self.k), bg=BG, fg=INK, highlightthickness=0, troughcolor=GRID,
                     font=self.f_small, command=lambda _v: self.redraw()).pack(anchor="w")

        self.redraw()
        self.root.after(cfg.view.frame_ms, self.tick)

    # ---- settings from the sliders -------------------------------------
    def robot_cfg(self) -> RobotConfig:
        g = lambda k: float(self.vars[k].get())
        return RobotConfig(drive_speed_mps=g("drive_mps"), steer_speed_dps=g("steer_dps"),
                           settle_s=self.cfg.settle_s, stop_s=g("stop_s"), tol_deg=self.cfg.tol_deg, goal_radius_m=0.0025)

    def noise_cfg(self) -> NoiseConfig:
        return NoiseConfig(float(self.vars["bias_deg"].get()), float(self.vars["sd_deg"].get()))

    # ---- goals -----------------------------------------------------------
    def _reset_robot(self) -> None:
        self.running = False
        self.btn_run.config(text="▶ เริ่ม")
        self.sim.reset(self.goals, self.seeds, self.scripted)
        self.result.config(text="พร้อม — กด ▶ เริ่ม" if self.goals else "")
        self.redraw()

    def add_goal(self, x: float, y: float) -> None:
        if self.running:
            self.result.config(text="หยุดก่อน แล้วค่อยวางเป้าเพิ่ม")
            return
        self.goals.append((x, y))
        self.seeds.append(self.rng.randrange(2 ** 31))
        self._reset_robot()

    def clear_goals(self) -> None:
        self.goals, self.seeds, self.scripted = [], [], []
        self._reset_robot()

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
        self._reset_robot()

    def _example(self, phi_deg: float, scripted: List[float]) -> None:
        phi = math.radians(phi_deg)
        d = 0.3 if phi_deg == 354.0 else 1.0
        self.goals, self.seeds, self.scripted = [(d * math.cos(phi), d * math.sin(phi))], [0], scripted
        for key, val in (("steer_dps", 35.0), ("drive_mps", 0.03), ("stop_s", 0.2),
                         ("bias_deg", 0.0), ("sd_deg", 0.0)):
            self.vars[key].set(val)
        self._reset_robot()

    def example_a(self) -> None:
        self._example(354.0, [])

    def example_comparison(self) -> None:
        from detour_steer import comparison_route
        self.goals = comparison_route()
        self.seeds, self.scripted = [0] * len(self.goals), []
        for key, val in (("steer_dps", 35.0), ("drive_mps", 0.03), ("stop_s", 0.2),
                         ("bias_deg", 0.0), ("sd_deg", 0.0)):
            self.vars[key].set(val)
        self._reset_robot()

    def example_b(self) -> None:
        self._example(60.0, [8.0])

    def reseed(self) -> None:
        self.seeds = [self.rng.randrange(2 ** 31) for _ in self.goals]
        self._reset_robot()

    # ---- running ---------------------------------------------------------
    def toggle_run(self) -> None:
        if not self.goals:
            self.result.config(text="ยังไม่มีเป้า — คลิกบนพื้นที่ หรือกด สุ่มเป้า")
            return
        if self.sim.done:
            self._reset_robot()
        self.running = not self.running
        self.btn_run.config(text="⏸ หยุดชั่วคราว" if self.running else "▶ เล่นต่อ")
        if self.running:
            self.result.config(text="กำลังวิ่ง…")

    def restart(self) -> None:
        self._reset_robot()

    def tick(self) -> None:
        if self.running:
            dt = self.cfg.view.frame_ms / 1000.0 * float(self.vars["speed_x"].get())
            self.sim.step(dt, self.robot_cfg(), self.noise_cfg())
            if self.sim.done:
                self.running = False
                self.btn_run.config(text="▶ เริ่ม")
                self.result.config(text=f"ถึงเป้าครบ {len(self.goals)} จุด: เวลา {self.sim.t:.2f} s, "
                                        f"เลี้ยวรวม {math.degrees(self.sim.steered):.0f}°, "
                                        f"ใช้ทางอ้อม {self.sim.n_detours} ครั้ง")
            self.redraw()
        self.root.after(self.cfg.view.frame_ms, self.tick)

    def redraw(self) -> None:
        self.field.draw()
        self.info.config(text=explain(self.sim))

    def screenshot(self, path: str, after_s: float) -> None:
        """Run example (b) in real time and save the window part-way through:
        the wheel has just stopped 8 deg past, and the detour is under way."""
        from PIL import ImageGrab
        self.root.attributes("-topmost", True)
        self.example_b()
        self.vars["speed_x"].set(1.0)
        self.toggle_run()

        def grab():
            self.running = False
            self.root.update()
            x, y = self.root.winfo_rootx(), self.root.winfo_rooty()
            w, h = self.root.winfo_width(), self.root.winfo_height()
            ImageGrab.grab(bbox=(x, y, x + w, y + h)).save(path)
            print(f"saved {path}")
            self.root.destroy()

        self.root.after(int(after_s * 1000), grab)

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
    app = OnlyApp(cfg)
    if args.shot:
        app.screenshot(args.shot, after_s=3.0)
    app.run()


if __name__ == "__main__":
    main()
