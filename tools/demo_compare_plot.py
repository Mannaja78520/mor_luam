"""Picture of the button demos 3 (Direct) and 4 (Detour), from the robot's own record.

Reads GET /api/demo/compare (start, goal, time and recorded path of the last run of
each planner) and draws both runs in their own start frame - start at (0,0), the
wheel's start heading along +x - so they share one start and one goal.
Dashed = the plan (same maths as the robot), solid = the path the robot measured.
Read only: sends no commands.

    python tools/demo_compare_plot.py                        # robot -> demo_compare.png
    python tools/demo_compare_plot.py --wait                 # wait for a NEW Direct and Detour run first
    python tools/demo_compare_plot.py --host mor-luam.local --out result.png
    python tools/demo_compare_plot.py --json saved.json      # draw a saved reply
"""
import argparse
import json
import math
import sys
import time
import urllib.request

COLORS = {"direct": "#eb6834", "detour": "#1baf7a"}   # validated pair (light surface), same as the web page
ACT_COLORS = {"turn": "#e87ba4", "drive": "#2a78d6", "still": "#d5dce8"}   # time line, validated pair + neutral
ACT_WORDS = {"turn": "หมุนล้อ", "drive": "วิ่ง", "still": "นิ่ง"}
NAMES = {"direct": "Direct (กด 3 ครั้ง)", "detour": "Detour (กด 4 ครั้ง)"}


def fetch(host):
    with urllib.request.urlopen(f"http://{host}/api/demo/compare", timeout=4) as r:
        return json.loads(r.read().decode())


def local(run):
    """Run in its own start frame: start (0,0), the wheel's start heading = +x."""
    h = math.radians(run["headingDeg"])
    c, s = math.cos(h), math.sin(h)

    def f(x, y):
        dx, dy = x - run["startX"], y - run["startY"]
        return dx * c + dy * s, -dx * s + dy * c
    return f(run["goalX"], run["goalY"]), [f(p["x"], p["y"]) for p in run["path"]]


def plan(goal, run, detour):
    """Plan from the start, as nav/WaypointRunner + algorithm/DetourSteer do it."""
    d = math.hypot(*goal)
    phi = math.degrees(math.atan2(goal[1], goal[0])) % 360
    if min(phi, 360 - phi) <= 3.5:
        phi = 0.0
    v, w, stop, settle = run["speedMps"], run["steerDps"], 0.2, 0.05
    direct = {"kind": "direct", "turn": phi, "at": (0.0, 0.0), "pts": [(0, 0), goal],
              "time": d / v + (phi / w + settle if phi else 0)}
    if not detour or phi <= 180:
        return direct
    k = d * math.radians(w) / v * abs(math.sin(math.radians(phi)))
    if k >= 2:
        return direct
    beta = 360 - math.degrees(math.acos(k - 1))
    if beta >= phi:
        return direct
    sb = math.sin(math.radians(beta))
    a = d * math.sin(math.radians(beta - phi)) / sb
    b = d * math.sin(math.radians(phi)) / sb
    t = a / v + stop + beta / w + settle + b / v
    if t >= direct["time"]:
        return direct
    return {"kind": "detour", "turn": beta, "at": (a, 0.0), "pts": [(0, 0), (a, 0), goal], "time": t, "a": a}


def fetch_retry(host, until):
    """fetch(), trying again through Wi-Fi drops and IP changes (use the .local name) until `until`."""
    while True:
        try:
            return fetch(host)
        except (OSError, ValueError) as e:
            if time.time() > until:
                raise
            print(f"  robot not answering ({e.__class__.__name__}), trying again...", flush=True)
            time.sleep(3)


def wait_for_new(host, limit_s):
    """Wait until both planners have a closed run newer than the ones present now."""
    t_end = time.time() + limit_s
    first = fetch_retry(host, t_end)
    old = {k: (first.get(k) or {}).get("id", 0) for k in COLORS}
    print(f"waiting for a new Direct (3 clicks) and Detour (4 clicks) run... (ids now {old})", flush=True)
    t0 = time.time()
    seen = {}
    while time.time() - t0 < limit_s:
        try:
            d = fetch(host)
        except (OSError, ValueError):
            time.sleep(2)
            continue
        if all(not d.get(k) for k in COLORS) and any(old.values()):
            old = {k: 0 for k in COLORS}           # the robot restarted: its record starts empty
        for k in COLORS:
            r = d.get(k)
            if r and r["id"] != old[k] and not r["open"] and seen.get(k) != r["id"]:
                seen[k] = r["id"]
                print(f"  {k}: run {r['id']} {'valid' if r['valid'] else 'NOT valid'} {r['elapsedMs'] / 1000:.2f} s", flush=True)
        if all(k in seen for k in COLORS):
            return d
        time.sleep(1)
    print("time limit: drawing what the robot has", flush=True)
    return fetch_retry(host, time.time() + 30)


def segments(run):
    """[(what, t0 s, t1 s)] from the robot's change points."""
    acts, end = run.get("acts") or [], run["elapsedMs"] / 1000
    out = []
    for i, a in enumerate(acts):
        t1 = acts[i + 1]["t"] / 1000 if i + 1 < len(acts) else end
        if t1 > a["t"] / 1000:
            out.append((a["a"], a["t"] / 1000, t1))
    return out


def timeline(ax, runs, ink, mut, grid):
    """One bar per run on a shared time axis: what the robot did, second by second."""
    from matplotlib.patches import Patch, Rectangle
    max_s = max(r["elapsedMs"] / 1000 for _, r, *_ in runs)
    ys = {k: i for i, (k, *_) in enumerate(reversed(runs))}
    for k, r, *_ in runs:
        y = ys[k]
        for what, t0, t1 in segments(r):
            ax.broken_barh([(t0, t1 - t0)], (y - 0.3, 0.6), facecolors=ACT_COLORS[what], edgecolor="white", lw=1.5,
                           hatch="///" if what == "turn" else None)
            if what != "still" and (t1 - t0) / max_s >= 0.12:
                ax.text((t0 + t1) / 2, y + 0.36, f"{ACT_WORDS[what]} {t1 - t0:.1f} s", ha="center", va="bottom",
                        color=ink, fontsize=10)
        end = r["elapsedMs"] / 1000
        ax.text(end + max_s * 0.012, y, f"{end:.2f} s" if r["valid"] else "ไม่ครบ", ha="left", va="center",
                color=ink, fontsize=10, fontweight="bold")
    done = [(k, r) for k, r, *_ in runs if r["valid"]]
    if len(done) == 2:
        (kf, rf), (_, rs) = sorted(done, key=lambda x: x[1]["elapsedMs"])
        t0, t1 = rf["elapsedMs"] / 1000, rs["elapsedMs"] / 1000
        ax.add_patch(Rectangle((t0, ys[kf] - 0.3), t1 - t0, 0.6, fill=False, ls="--", lw=1.5, ec=ink))
        if (t1 - t0) / max_s >= 0.06:
            ax.text((t0 + t1) / 2, ys[kf] + 0.36, f"ถึงก่อน {t1 - t0:.1f} s", ha="center", va="bottom", color=ink, fontsize=10)
    ax.set_yticks([ys[k] for k, *_ in runs], [NAMES[k].split(" (")[0] for k, *_ in runs], color=ink, fontsize=11,
                  fontweight="bold")
    ax.set_xlim(0, max_s * 1.1)
    ax.set_ylim(-0.6, len(runs) - 0.25)
    ax.set_xlabel("วินาทีนับจากเริ่มจับเวลา", color=mut)
    ax.grid(axis="x", color=grid, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=mut, labelsize=9)
    ax.tick_params(axis="y", length=0)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.legend(handles=[Patch(fc=ACT_COLORS["turn"], hatch="///", ec="white", label="หมุนล้ออยู่กับที่"),
                       Patch(fc=ACT_COLORS["drive"], label="วิ่ง"),
                       Patch(fc=ACT_COLORS["still"], label="นิ่ง (ตั้งล้อ / หยุดเปลี่ยนท่า)")],
              loc="lower left", bbox_to_anchor=(0, 1.0), ncol=3, frameon=False, fontsize=10, labelcolor=ink)


def draw(data, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for name in ("Leelawadee UI", "Leelawadee", "Tahoma", "Noto Sans Thai"):
        if any(name == f.name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break

    runs = []
    for k in ("direct", "detour"):
        r = data.get(k)
        if r:
            goal, path = local(r)
            runs.append((k, r, goal, path, plan(goal, r, k == "detour")))
    if not runs:
        print("no demo 3/4 run on the robot yet: press the button 3 times, then 4 times")
        return None

    ink, mut, grid = "#16202e", "#5b6878", "#e3e8f0"
    timed = [x for x in runs if x[1].get("acts")]
    if timed:
        fig, (axt, ax) = plt.subplots(2, 1, figsize=(10, 6.6), dpi=150, gridspec_kw={"height_ratios": [1.15, 1.6]})
        timeline(axt, timed, ink, mut, grid)
    else:
        fig, ax = plt.subplots(figsize=(10, 3.9), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    for k, r, goal, path, p in runs:
        xs, ys = zip(*p["pts"])
        ax.plot(xs, ys, color=COLORS[k], lw=1.6, ls=(0, (5, 4)), zorder=2)
    for k, r, goal, path, p in runs:
        t = f"{r['elapsedMs'] / 1000:.2f} s" if r["valid"] else "ไม่ครบ (หยุดก่อนถึง)"
        xs, ys = zip(*path)
        ax.plot(xs, ys, color=COLORS[k], lw=2.6, solid_capstyle="round", zorder=3, label=f"{NAMES[k]}  {t}")
        # the turn in place: a ring at the spot + a label in ink
        ax.add_patch(plt.Circle(p["at"], 0.009, fill=False, color=COLORS[k], lw=2, zorder=4))
        txt = f"{k.capitalize()}: หมุนล้อ {p['turn']:.0f}° " + ("ตรงนี้" if p["kind"] == "detour" else "ก่อนออกวิ่ง")
        if k == "direct":
            ax.annotate(txt, p["at"], xytext=(0, -26), textcoords="offset points", ha="left", va="top", color=ink, fontsize=10)
        else:
            ax.annotate(txt, p["at"], xytext=(0, 18), textcoords="offset points", ha="right", va="bottom", color=ink, fontsize=10)
    goal = runs[0][2]
    ax.plot(*goal, marker="o", ms=10, mfc="white", mec=ink, mew=2, zorder=5)
    ax.annotate("เป้า", goal, xytext=(0, -14), textcoords="offset points", ha="center", va="top", color=ink, fontsize=10)
    ax.plot(0, 0, marker=">", ms=12, color=ink, zorder=5)
    ax.annotate("เริ่ม", (0, 0), xytext=(-16, 0), textcoords="offset points", ha="right", va="center", color=ink, fontsize=10)

    ax.set_aspect("equal")
    ax.margins(x=0.1, y=0.45)
    ax.grid(color=grid, lw=0.8)
    ax.tick_params(colors=mut, labelsize=9)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_xlabel("x จากจุดเริ่ม (m)", color=mut)
    ax.set_ylabel("y (m)", color=mut)
    d, t = data.get("direct"), data.get("detour")
    if d and t and d["valid"] and t["valid"]:
        diff = (d["elapsedMs"] - t["elapsedMs"]) / 1000
        head = (f"Detour เร็วกว่า {diff:.2f} s ({100 * diff / (d['elapsedMs'] / 1000):.1f}%)" if diff > 0
                else f"รอบนี้ Direct เร็วกว่า {-diff:.2f} s")
    else:
        head = "ยังเทียบเวลาไม่ได้: ต้องมีรอบที่ถึงเป้าครบทั้ง 2 แบบ"
    if timed:
        axt.set_title(f"เดโม 3 / 4 ครั้ง: {head}", loc="left", color=ink, fontsize=13, fontweight="bold", pad=30)
        ax.set_title("ทางที่วิ่ง (มองจากด้านบน)", loc="left", color=ink, fontsize=11)
    else:
        ax.set_title(f"เดโม 3 / 4 ครั้ง: {head}", loc="left", color=ink, fontsize=13, fontweight="bold")
    leg = ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2, frameon=False, fontsize=10, labelcolor=ink)
    leg.set_zorder(6)
    fig.text(0.01, 0.01, "เส้นประ = ทางตามแผน · เส้นทึบ = ทางที่หุ่นวัดได้เอง (odometry) · วาดจากจุดเริ่มของแต่ละรอบ",
             color=mut, fontsize=9)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out)
    print(f"saved {out}: {head}")
    for k, r, goal, path, p in runs:
        end = path[-1]
        miss = 1000 * math.hypot(end[0] - goal[0], end[1] - goal[1])
        how = f"a {p['a'] * 100:.0f} cm, turn {p['turn']:.0f} deg" if p["kind"] == "detour" else f"turn {p['turn']:.0f} deg first"
        print(f"  {k}: {r['elapsedMs'] / 1000:.2f} s valid {r['valid']} | plan {p['kind']} ({how}, ~{p['time']:.1f} s)"
              f" | end {miss:.1f} mm from goal | {len(path)} path points")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="mor-luam.local")
    ap.add_argument("--out", default="demo_compare.png")
    ap.add_argument("--wait", action="store_true", help="wait for a new Direct and Detour run first")
    ap.add_argument("--wait-s", type=float, default=900)
    ap.add_argument("--json", help="draw a saved /api/demo/compare reply instead of asking the robot")
    a = ap.parse_args()
    if a.json:
        with open(a.json, encoding="utf-8") as f:
            data = json.load(f)
    else:
        data = wait_for_new(a.host, a.wait_s) if a.wait else fetch(a.host)
    return 0 if draw(data, a.out) else 1


if __name__ == "__main__":
    sys.exit(main())
