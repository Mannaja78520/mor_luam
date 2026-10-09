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
    python tools/demo_compare_plot.py --series [--wait]      # the "test 3 rounds" series (every run)
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


def fetch(host, path="/api/demo/compare"):
    with urllib.request.urlopen(f"http://{host}{path}", timeout=6) as r:
        return json.loads(r.read().decode())


def wait_series(host, limit_s):
    """Wait until the robot's series has ended (all runs, or stopped early)."""
    t_end = time.time() + limit_s
    last = None
    while time.time() < t_end:
        try:
            d = fetch(host, "/api/demo/series")
        except (OSError, ValueError):
            time.sleep(3)
            continue
        if (d["count"], d["active"]) != last:
            last = (d["count"], d["active"])
            state = "running" if d["active"] else "ended"
            print(f"  series {d['id']}: {d['count']}/{d['total']} runs {state} {d.get('why', '')}", flush=True)
        if d["id"] and not d["active"] and d["count"]:
            return d
        time.sleep(3)
    return fetch(host, "/api/demo/series")


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
    """One bar per run on a shared time axis: what the robot did, second by second.
    runs: [(key, run, goal, path, plan, label)], drawn top to bottom."""
    from matplotlib.patches import Patch, Rectangle
    max_s = max(r["elapsedMs"] / 1000 for _, r, *_ in runs)
    n = len(runs)
    for i, (k, r, *_rest) in enumerate(runs):
        y = n - 1 - i
        for what, t0, t1 in segments(r):
            ax.broken_barh([(t0, t1 - t0)], (y - 0.3, 0.6), facecolors=ACT_COLORS[what], edgecolor="white", lw=1.5,
                           hatch="///" if what == "turn" else None)
            if what == "still" or (t1 - t0) / max_s < 0.12:
                continue
            if n <= 2:
                ax.text((t0 + t1) / 2, y + 0.36, f"{ACT_WORDS[what]} {t1 - t0:.1f} s", ha="center", va="bottom",
                        color=ink, fontsize=10)
            else:                                     # many rows: seconds inside the bar
                ax.text((t0 + t1) / 2, y, f"{t1 - t0:.1f}", ha="center", va="center", color=ink, fontsize=9,
                        fontweight="bold", bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.85))
        end = r["elapsedMs"] / 1000
        ax.text(end + max_s * 0.012, y, f"{end:.2f} s" if r["valid"] else "ไม่ครบ", ha="left", va="center",
                color=ink, fontsize=10, fontweight="bold")
    done = [(i, r) for i, (k, r, *_rest) in enumerate(runs) if r["valid"]]
    if n == 2 and len(done) == 2:                     # a pair: box the time the faster one saved
        (i_f, rf), (_, rs) = sorted(done, key=lambda x: x[1]["elapsedMs"])
        t0, t1 = rf["elapsedMs"] / 1000, rs["elapsedMs"] / 1000
        yf = n - 1 - i_f
        ax.add_patch(Rectangle((t0, yf - 0.3), t1 - t0, 0.6, fill=False, ls="--", lw=1.5, ec=ink))
        if (t1 - t0) / max_s >= 0.06:
            ax.text((t0 + t1) / 2, yf + 0.36, f"ถึงก่อน {t1 - t0:.1f} s", ha="center", va="bottom", color=ink, fontsize=10)
    ax.set_yticks([n - 1 - i for i in range(n)], [x[5] for x in runs], fontsize=10 if n > 2 else 11, fontweight="bold")
    ax.set_xlim(0, max_s * 1.1)
    ax.set_ylim(-0.6, n - 0.25)
    ax.set_xlabel("วินาทีนับจากเริ่มจับเวลา" + (" (ตัวเลขในแถบ = วินาที)" if n > 2 else ""), color=mut)
    ax.grid(axis="x", color=grid, lw=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=mut, labelsize=9)
    ax.tick_params(axis="y", length=0, labelcolor=ink)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.legend(handles=[Patch(fc=ACT_COLORS["turn"], hatch="///", ec="white", label="หมุนล้ออยู่กับที่"),
                       Patch(fc=ACT_COLORS["drive"], label="วิ่ง"),
                       Patch(fc=ACT_COLORS["still"], label="นิ่ง (ตั้งล้อ / หยุดเปลี่ยนท่า)")],
              loc="lower left", bbox_to_anchor=(0, 1.0), ncol=3, frameon=False, fontsize=10, labelcolor=ink)


def unflat(run):
    """A series run keeps its path as flat [x0, y0, x1, y1, ...]."""
    if "path" in run or "xy" not in run:
        return run
    xy = run["xy"]
    return {**run, "path": [{"x": xy[i], "y": xy[i + 1]} for i in range(0, len(xy) - 1, 2)]}


def headline(data):
    """The one-line result: one pair, or the mean of a series' rounds. Also [(direct s, detour s)] per round."""
    if "runs" in data:
        rounds = []
        for i in range(0, len(data["runs"]) - 1, 2):
            pair = {r["planner"]: r for r in data["runs"][i:i + 2]}
            if pair.get("direct", {}).get("valid") and pair.get("detour", {}).get("valid"):
                rounds.append((pair["direct"]["elapsedMs"] / 1000, pair["detour"]["elapsedMs"] / 1000))
        if not rounds:
            return "ยังเทียบไม่ได้: ยังไม่มีรอบที่ครบทั้ง 2 แบบ", rounds
        d = sum(r[0] for r in rounds) / len(rounds)
        t = sum(r[1] for r in rounds) / len(rounds)
        wins = sum(1 for r in rounds if r[1] < r[0])
        if d > t:
            return f"Detour เร็วกว่าเฉลี่ย {d - t:.2f} s ({100 * (d - t) / d:.1f}%) · ชนะ {wins} จาก {len(rounds)} รอบ", rounds
        return f"เฉลี่ยแล้ว Direct เร็วกว่า {t - d:.2f} s · Detour ชนะ {wins} จาก {len(rounds)} รอบ", rounds
    d, t = data.get("direct"), data.get("detour")
    if d and t and d["valid"] and t["valid"]:
        diff = (d["elapsedMs"] - t["elapsedMs"]) / 1000
        return (f"Detour เร็วกว่า {diff:.2f} s ({100 * diff / (d['elapsedMs'] / 1000):.1f}%)" if diff > 0
                else f"รอบนี้ Direct เร็วกว่า {-diff:.2f} s"), []
    return "ยังเทียบเวลาไม่ได้: ต้องมีรอบที่ถึงเป้าครบทั้ง 2 แบบ", []


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
    if "runs" in data:                                # a series: every run, in the order it ran
        for i, raw in enumerate(data["runs"]):
            k, r = raw["planner"], unflat(raw)
            goal, path = local(r)
            runs.append((k, r, goal, path, plan(goal, r, k == "detour"), f"รอบ {i // 2 + 1} {k.capitalize()}"))
    else:
        for k in ("direct", "detour"):
            r = data.get(k)
            if r:
                goal, path = local(r)
                runs.append((k, r, goal, path, plan(goal, r, k == "detour"), k.capitalize()))
    if not runs:
        print("no demo 3/4 run on the robot yet: press the button 3 times, then 4 times")
        return None

    ink, mut, grid = "#16202e", "#5b6878", "#e3e8f0"
    many = len(runs) > 2
    timed = [x for x in runs if x[1].get("acts")]
    if timed:
        top = 1.15 if len(timed) <= 2 else 0.42 * len(timed)
        fig, (axt, ax) = plt.subplots(2, 1, figsize=(10, 5.0 + top * 1.4), dpi=150,
                                      gridspec_kw={"height_ratios": [top, 1.6]})
        timeline(axt, timed, ink, mut, grid)
    else:
        fig, ax = plt.subplots(figsize=(10, 3.9), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    firsts = {}
    for x in runs:
        firsts.setdefault(x[0], x)
    for k, r, goal, path, p, _ in firsts.values():    # the plan once per planner, under the real paths
        xs, ys = zip(*p["pts"])
        ax.plot(xs, ys, color=COLORS[k], lw=1.6, ls=(0, (5, 4)), zorder=2)
    for k, r, goal, path, p, _ in runs:
        xs, ys = zip(*path)
        ax.plot(xs, ys, color=COLORS[k], lw=2.0 if many else 2.6, alpha=0.8 if many else 1,
                solid_capstyle="round", zorder=3)
    for k, r, goal, path, p, _ in firsts.values():
        mine = [x[1] for x in runs if x[0] == k and x[1]["valid"]]
        if many and mine:
            t = f"เฉลี่ย {sum(m['elapsedMs'] for m in mine) / len(mine) / 1000:.2f} s ({len(mine)} ครั้ง)"
        else:
            t = f"{r['elapsedMs'] / 1000:.2f} s" if r["valid"] else "ไม่ครบ"
        ax.plot([], [], color=COLORS[k], lw=2.6, label=f"{NAMES[k]}  {t}")
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
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_xlabel("x จากจุดเริ่ม (m)", color=mut)
    ax.set_ylabel("y (m)", color=mut)
    head, rounds = headline(data)
    title = f"ทดสอบ {len(rounds)} รอบ: {head}" if "runs" in data else f"เดโม 3 / 4 ครั้ง: {head}"
    if timed:
        axt.set_title(title, loc="left", color=ink, fontsize=13, fontweight="bold", pad=30)
        ax.set_title("ทางที่วิ่ง (มองจากด้านบน)" + (" · ทุกครั้งซ้อนกัน" if many else ""), loc="left", color=ink, fontsize=11)
    else:
        ax.set_title(title, loc="left", color=ink, fontsize=13, fontweight="bold")
    leg = ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2, frameon=False, fontsize=10, labelcolor=ink)
    leg.set_zorder(6)
    foot = "เส้นประ = ทางตามแผน · เส้นทึบ = ทางที่หุ่นวัดได้เอง (odometry) · วาดจากจุดเริ่มของแต่ละครั้ง"
    if rounds:
        foot = " · ".join(f"รอบ {i + 1}: Direct {d:.2f} / Detour {t:.2f} s" for i, (d, t) in enumerate(rounds)) + "\n" + foot
    fig.text(0.01, 0.01, foot, color=mut, fontsize=9)
    fig.tight_layout(rect=(0, 0.06 if rounds else 0.05, 1, 1))
    fig.savefig(out)
    print(f"saved {out}: {head}")
    for k, r, goal, path, p, label in runs:
        end = path[-1]
        miss = 1000 * math.hypot(end[0] - goal[0], end[1] - goal[1])
        print(f"  {label}: {r['elapsedMs'] / 1000:.2f} s valid {r['valid']} | end {miss:.1f} mm from goal")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="mor-luam.local")
    ap.add_argument("--out", default="demo_compare.png")
    ap.add_argument("--wait", action="store_true", help="wait for a new Direct and Detour run first")
    ap.add_argument("--wait-s", type=float, default=900)
    ap.add_argument("--json", help="draw a saved /api/demo/compare or /api/demo/series reply instead of asking the robot")
    ap.add_argument("--series", action="store_true", help="the last 'test 3 rounds' series (--wait: until it ends)")
    a = ap.parse_args()
    if a.json:
        with open(a.json, encoding="utf-8") as f:
            data = json.load(f)
    elif a.series:
        data = wait_series(a.host, a.wait_s) if a.wait else fetch(a.host, "/api/demo/series")
    else:
        data = wait_for_new(a.host, a.wait_s) if a.wait else fetch(a.host)
    return 0 if draw(data, a.out) else 1


if __name__ == "__main__":
    sys.exit(main())
