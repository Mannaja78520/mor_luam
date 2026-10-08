"""Race Direct against Detour Steer on the real robot (same start, same goal).

    python tools/race_test.py --runs 2 --dist 0.3 --angle 6
    python tools/race_test.py --runs 2 --steer-dps 36      # plan with another steering speed

THE ROBOT MOVES: floor, ~2 m clear, someone next to it. Ctrl+C = E-STOP.

Uses the robot's own route test (POST /api/nav/test): the wheel is first turned
to the same start angle (not timed), then the robot's clock times the route
until the goal is reached. The goal is `dist` metres away, `angle` degrees on
the far side of the wheel: steering straight at it would take a turn of
360 - angle degrees (the wheel steers one way only), the case Detour Steer is for.
The order alternates (Direct, Detour, Direct, ...) and the robot drives itself
back to the start between runs.
"""
import argparse
import math
import socket
import sys
import threading
import time
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_test as t  # noqa: E402


def wait_test_end(timeout=90.0):
    end = time.time() + timeout
    plans = []
    while time.time() < end:
        try:
            s = t.status()
        except OSError:
            time.sleep(0.3)
            continue
        test, p = s["nav"].get("test", {}), s["nav"]["plan"]
        if test.get("phase") == "running" and p.get("distM", 0) > 0:   # this run's plans only
            key = (p["kind"], round(p["phiDeg"]), round(p["timeS"], 2))
            if not plans or plans[-1] != key:
                plans.append(key)
        if test.get("phase") not in ("aligning", "running") and time.time() > end - timeout + 1:
            return s, plans
        time.sleep(0.2)
    t.estop()
    raise RuntimeError("race run timed out; E-STOP sent")


def main():
    ap = argparse.ArgumentParser(description="Race Direct vs Detour Steer on the robot.")
    ap.add_argument("--host", default="mor-luam.local")
    ap.add_argument("--runs", type=int, default=2, help="runs per method")
    ap.add_argument("--dist", type=float, default=0.3, help="goal distance, m")
    ap.add_argument("--angle", type=float, default=6.0, help="goal angle past the wheel, deg")
    ap.add_argument("--steer-dps", type=float, default=0.0, help="plan with this steering speed (restored after)")
    a = ap.parse_args()
    t.HOST = socket.gethostbyname(a.host)
    threading.Thread(target=t._keepalive_loop, daemon=True).start()
    t.KEEPALIVE.set()
    settings = t.call("GET", "/api/settings")
    results = {"direct": [], "detour": []}
    try:
        if a.steer_dps:
            r = t.call("POST", "/api/settings", dict(settings, steerDps=a.steer_dps))
            if not r.get("ok"):
                raise RuntimeError(f"settings refused: {r.get('error')}")
        if not t.call("POST", "/api/pose/reset").get("ok"):
            raise RuntimeError("pose reset refused (is the robot moving?)")
        time.sleep(0.5)
        # Aim the wheel first and use where it REALLY stopped (it stops up to the
        # 3.5 deg tolerance short): the goal is then exactly `angle` past the wheel.
        h_cmd = t.status()["robot"]["wheelHeadingDeg"]
        t.move(0, h_cmd)
        t.wait_until(lambda s: s["robot"].get("steerAimed", abs(s["robot"]["steerErrDeg"]) <= 3.5) and abs(s["robot"]["steerRateDps"]) < 3, timeout=15)
        t.estop()                      # route tests start only from a halted robot
        time.sleep(0.8)
        h0 = t.status()["robot"]["wheelHeadingDeg"]
        b = math.radians(h0 - a.angle)
        goal = {"x": round(a.dist * math.cos(b), 3), "y": round(a.dist * math.sin(b), 3)}
        if not t.call("POST", "/api/waypoints", {"points": [goal]}).get("ok"):
            raise RuntimeError("waypoint refused")
        print(f"start (0,0), wheel {h0:.1f} deg, goal ({goal['x']}, {goal['y']}) = {a.dist} m, "
              f"{a.angle} deg past the wheel; steering speed used for planning: "
              f"{a.steer_dps or settings.get('steerDps')} deg/s, drive {settings.get('navSpeedMps')} m/s")
        order = [p for _ in range(a.runs) for p in ("direct", "detour")]
        for i, planner in enumerate(order, 1):
            t.countdown(f"run {i}/{len(order)}: {planner.upper()} to the goal, then back to the start")
            t.estop()                  # route tests start only from a halted, still robot
            t.wait_until(lambda s: s["robot"]["mode"] == "halt" and abs(s["robot"]["rpm"]) < 0.3
                         and abs(s["robot"]["steerRateDps"]) < 2.0, timeout=8)
            time.sleep(0.5)
            r = t.call("POST", "/api/nav/test", {"planner": planner, "startHeadingDeg": round(h0 % 360, 1), "ready": True})
            if not r.get("ok"):
                raise RuntimeError(f"{planner} refused: {r.get('error')}")
            s, plans = wait_test_end()
            test = s["nav"]["test"]
            ok = test.get("phase") == "done" and test.get("valid")
            sec = test.get("elapsedMs", 0) / 1000.0
            if ok:
                results[planner].append(sec)
            steps = " -> ".join(f"{k} phi {ph}" for k, ph, _ in plans) or "?"
            print(f"  {planner:6s} run {i}: {'%.2f s' % sec if ok else 'NOT VALID (' + str(test.get('phase')) + ': ' + s['nav'].get('message', '') + ' / ' + s['robot'].get('haltWhy', '') + ')'}"
                  f" | plans: {steps} (first predicted {plans[0][2] if plans else 0:.2f} s)"
                  f" | ends {1000 * math.hypot(s['robot']['x'] - goal['x'], s['robot']['y'] - goal['y']):.1f} mm from the goal"
                  f" (allowed {1000 * settings.get('navTolM', 0):.1f} mm)")
            t.go_home(f"race_home_{i}")
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        print("E-STOP:", "sent" if t.estop() else "NOT CONFIRMED - switch the robot off!")
        if a.steer_dps:
            t.call("POST", "/api/settings", settings)
    print("\nResult (robot clock, wheel alignment not counted):")
    for k, v in results.items():
        if v:
            print(f"  {k:6s}: " + ", ".join(f"{x:.2f}" for x in v) + f"   mean {sum(v) / len(v):.2f} s")
    if results["direct"] and results["detour"]:
        d = sum(results["direct"]) / len(results["direct"])
        e = sum(results["detour"]) / len(results["detour"])
        print(f"  Detour - Direct = {e - d:+.2f} s ({(e - d) / d * 100:+.1f} %)  ->  "
              f"{'DETOUR FASTER' if e < d else 'DIRECT FASTER'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
