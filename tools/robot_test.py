"""Test the real mor_luam robot over Wi-Fi: settings, PIDF, steering, driving,
routes with Detour Steer, and the safety stops.

    python tools/robot_test.py --host mor-luam.local --steps 1,2,3,4
    docker\\mor_luam.bat robot-test --steps 2          (same, through the helper)

    step 1  no motion: settings / PID / points read-write, pose reset
    step 2  steering only (rpm 0): the wheel turns in place to +90, +180, +270, +345 deg
    step 3  short drives (<= 0.25 m at 7.5 rpm) and back, E-STOP and moving refusals
    step 4  routes: one Detour Steer case and back; a route with no heartbeat (must stop)

The robot MOVES in steps 2-4. Put it on the floor with ~2 m clear around it and
stand next to it. Ctrl+C (or any error) sends E-STOP. Every step keeps within
~0.7 m of where the robot starts. Raw 100 Hz traces go to tools/test_out/.
"""
import argparse
import csv
import io
import json
import math
import os
import sys
import time
import urllib.request

HOST = "mor-luam.local"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_out")
WHEEL_D = 0.0762                      # firmware/config/esp32_hardware.h WHEEL_DIAMETER
DRIVE_RPM = 7.5                      # measured full power is only ~9.7 rpm
DRIVE_DIST_M = 0.25
results = []                          # (step, name, ok, detail)


# ---- talking to the robot ----------------------------------------------------------

def call(method, path, body=None, timeout=4.0, raw=False):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://{HOST}{path}", data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            txt = r.read().decode("utf-8")
            return txt if raw else json.loads(txt or "{}")
    except urllib.error.HTTPError as e:
        txt = e.read().decode("utf-8", "replace")
        try:
            return json.loads(txt)
        except ValueError:
            return {"ok": False, "error": f"HTTP {e.code}"}


def status():
    return call("GET", "/api/status")


def estop():
    for _ in range(3):
        try:
            if call("POST", "/api/estop", timeout=2).get("ok"):
                return True
        except OSError:
            pass
    return False


def move(rpm, heading, dist=0.0):
    r = call("POST", "/api/test/move", {"rpm": rpm, "headingDeg": heading % 360, "distM": dist, "tolM": 0.02})
    if not r.get("ok"):
        raise RuntimeError(f"move refused: {r.get('error')}")


def trace(seconds):
    txt = call("GET", f"/api/trace?s={seconds:.1f}", timeout=15, raw=True)
    rows = list(csv.DictReader(io.StringIO(txt)))
    return [{k: float(v) for k, v in r.items()} for r in rows]


def save_trace(name, rows):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".csv")
    with open(path, "w", newline="") as f:
        if rows:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    return path


def wait_until(pred, timeout, period=0.1):
    end = time.time() + timeout
    while time.time() < end:
        s = status()
        if pred(s):
            return s
        time.sleep(period)
    return None


def report(step, name, ok, detail):
    results.append((step, name, ok, detail))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}: {detail}")


def countdown(what):
    print(f"\n  >>> the robot will MOVE: {what} - starting in 3 s (Ctrl+C = stop)")
    time.sleep(3)


def cw(a, b):
    """how far the wheel turns from b to reach a, 0..360. It steers clockwise
    seen from above, which DEcreases the angle (firmware angles::cwErrorDeg)."""
    return (b - a) % 360.0


def unwrap_rotation(angles):
    """total signed turn along a list of angles (deg)"""
    tot = 0.0
    for i in range(1, len(angles)):
        d = (angles[i] - angles[i - 1] + 180.0) % 360.0 - 180.0
        tot += d
    return tot


# ---- step 1: no motion ---------------------------------------------------------------

def step1():
    print("\nStep 1 - no motion")
    s = call("GET", "/api/settings")
    r = call("POST", "/api/settings", s)
    report(1, "settings read + write back", r.get("ok", False), r.get("error", "same values accepted"))
    bad = dict(s, navSpeedMps=5.0)
    r = call("POST", "/api/settings", bad)
    report(1, "settings: bad value refused", not r.get("ok", True), r.get("error", "was accepted!"))
    pid = call("GET", "/api/pid")
    ok = True
    for loop in ("spin", "steer"):
        ok &= call("POST", "/api/pid", {"loop": loop, "values": pid[loop]}).get("ok", False)
    back = call("GET", "/api/pid")
    report(1, "PID read + write back", ok and back == pid, f"spin {pid['spin']} steer {pid['steer']}")
    invalid = [
        [-1.0] + pid["spin"][1:],
        ["invalid"] + pid["spin"][1:],
        pid["spin"] + [10, -10],              # reversed integral limits
        pid["spin"] + [-1023, 1023, 0, 2000], # exceeds hardware power
    ]
    refused = all(not call("POST", "/api/pid", {"loop": "spin", "values": v}).get("ok", True) for v in invalid)
    report(1, "invalid PID values refused without changes", refused and call("GET", "/api/pid") == pid,
           "negative/non-number gains and invalid integral/output limits")
    old = call("GET", "/api/waypoints")["points"]
    test = [{"x": 0.3, "y": 0.0}, {"x": 0.3, "y": 0.3}]
    call("POST", "/api/waypoints", {"points": test})
    got = call("GET", "/api/waypoints")["points"]
    call("POST", "/api/waypoints", {"points": old})
    same = len(got) == 2 and all(abs(a["x"] - b["x"]) < 1e-3 and abs(a["y"] - b["y"]) < 1e-3 for a, b in zip(got, test))
    report(1, "points save + read back", same, f"{got}")
    r = call("POST", "/api/pose/reset")
    st = status()["robot"]
    report(1, "pose reset", r.get("ok", False) and abs(st["x"]) < 1e-6 and abs(st["y"]) < 1e-6,
           f"x {st['x']:.3f} y {st['y']:.3f}")


# ---- step 2: steering only ---------------------------------------------------------------

def steer_once(delta, tag):
    s0 = status()["robot"]
    start_steer = s0["steerDeg"]
    move(0, s0["wheelHeadingDeg"] - delta)            # delta = clockwise turn
    t0 = time.time()
    # aimed = inside tolerance, not coasting, wheel still
    s = wait_until(lambda s: abs(s["robot"]["steerErrDeg"]) <= 3.5 and not s["robot"]["coasting"]
                   and abs(s["robot"]["steerRateDps"]) < 3, timeout=12)
    took = time.time() - t0
    time.sleep(0.6)
    rows = trace(min(15.0, took + 1.5))
    save_trace(f"steer_{tag}", rows)
    estop()                                   # hold: motor off until the next command
    if not rows:
        report(2, f"steer +{delta}", False, "no trace")
        return
    # from the first sample of the command (flags != 0)
    i0 = next((i for i, r in enumerate(rows) if int(r["flags"]) & 3), 0)
    seg = rows[i0:]
    target = seg[-1]["target_deg"]
    turned = -unwrap_rotation([r["steer_deg"] for r in seg])   # + = in the steering direction
    wanted = cw(target, seg[0]["steer_deg"])
    past = turned - wanted                    # + = went past the target
    t_in = next((r["t_ms"] - seg[0]["t_ms"] for r in seg
                 if min(cw(target, r["steer_deg"]), cw(r["steer_deg"], target)) <= 3.5), None)
    final = (seg[-1]["steer_deg"] - target + 180) % 360 - 180   # + = stopped short, - = went past
    peak_rate = max(r["rate_dps"] for r in seg)
    st = status()["robot"]
    ok = s is not None and abs(final) <= 3.5 and past < 180
    report(2, f"steer {delta:>3} deg clockwise",
           ok, f"wanted {wanted:5.1f}, turned {turned:6.1f} -> past target {past:+5.1f} deg, "
               f"in tolerance after {t_in/1000 if t_in is not None else float('nan'):.2f} s, "
               f"stopped {final:+.1f} deg before the target, peak {peak_rate:.0f} deg/s, coast {st['coastS']:.3f} s "
               f"({st['coastSamples']} samples){'' if s else ', TIMEOUT'}")
    if not ok:
        raise RuntimeError("steering check failed; further motion cancelled")


def step2():
    print("\nStep 2 - steering only (wheel turns in place, rpm 0)")
    countdown("steering only, the wheel turns in place")
    for i, d in enumerate([90, 180, 270, 345, 90, 90, 180]):
        steer_once(d, f"{i + 1}_{d}")
        time.sleep(0.5)


# ---- step 3: drives, E-STOP, refusals ---------------------------------------------------

def drive_once(rpm, heading, dist, tag):
    if not (0 < abs(rpm) <= DRIVE_RPM and 0 < dist <= DRIVE_DIST_M):
        raise ValueError("drive test exceeds 7.5 rpm or 0.25 m")
    s0 = status()["robot"]
    x0, y0 = s0["x"], s0["y"]
    move(rpm, heading, dist)
    t0 = time.time()
    s = wait_until(lambda s: s["robot"]["mode"] == "halt" or
                   (s["robot"]["mode"] == "steer" and not s["robot"]["goalActive"] and time.time() - t0 > 1),
                   timeout=dist / (rpm / 60 * math.pi * WHEEL_D) + 12)
    took = time.time() - t0
    if s is None:
        estop()  # Stop before any slow diagnostics after a timeout.
    time.sleep(0.5)
    rows = trace(min(15.0, took + 1.5))
    save_trace(f"drive_{tag}", rows)
    estop()
    st = status()["robot"]
    moved = math.hypot(st["x"] - x0, st["y"] - y0)
    drv = [r for r in rows if int(r["flags"]) & 2]
    if not drv:
        report(3, f"drive {dist} m at {rpm} rpm", False, "never entered DRIVE")
        raise RuntimeError("drive trace has no DRIVE samples; further motion cancelled")
    tgt = abs(drv[0]["target_rpm"])
    t_d0 = drv[0]["t_ms"]
    rise = next((r["t_ms"] - t_d0 for r in drv if abs(r["rpm"]) >= 0.9 * tgt), None)
    # A steering re-lock restarts the drive ramp. Exclude startup from each
    # DRIVE interval, rather than calling later startup samples "steady".
    steady, drive_start, starts = [], None, 0
    for row in rows:
        if int(row["flags"]) & 2:
            if drive_start is None:
                drive_start = row["t_ms"]
                starts += 1
            if row["t_ms"] - drive_start > 1000:
                steady.append(abs(row["rpm"]))
        else:
            drive_start = None
    mean = sum(steady) / len(steady) if steady else float("nan")
    sd = (sum((v - mean) ** 2 for v in steady) / len(steady)) ** 0.5 if steady else float("nan")
    peak = max(abs(r["rpm"]) for r in drv)
    ok = s is not None and abs(moved - dist) <= 0.05 and (not steady or abs(mean - tgt) <= 0.1 * tgt)
    report(3, f"drive {dist} m at {rpm} rpm", ok,
           f"odometry {moved:.3f} m, rpm rise to 90% {rise/1000 if rise is not None else float('nan'):.2f} s, "
           f"peak {peak:.1f}, steady {mean:.1f} +/- {sd:.1f} (target {tgt:.1f})"
           f", re-aims {max(0, starts - 1)}"
           f", learned gain {st.get('driveGain', 1.0):.3f} ({st.get('driveLearnedS', 0.0):.1f} s)"
           f"{'' if s else ', TIMEOUT'}")
    if not ok:
        raise RuntimeError("drive check failed; stopping before the next motion test")


def step3():
    print("\nStep 3 - short drives (<= 0.25 m at 7.5 rpm), E-STOP, moving refusals")
    if not call("POST", "/api/pose/reset").get("ok"):
        raise RuntimeError("pose reset refused before drive test")
    countdown("drive 0.25 m straight ahead along the wheel, then back")
    h = status()["robot"]["wheelHeadingDeg"]
    drive_once(DRIVE_RPM, h, DRIVE_DIST_M, "1_out_7p5rpm")
    drive_once(DRIVE_RPM, h + 180, DRIVE_DIST_M, "2_back_7p5rpm")

    countdown("drive towards 0.25 m, check moving refusals, then E-STOP")
    h = status()["robot"]["wheelHeadingDeg"]
    move(DRIVE_RPM, h, DRIVE_DIST_M)
    moving = wait_until(lambda s: s["robot"]["mode"] == "drive" and abs(s["robot"]["rpm"]) > 1, timeout=8)
    if not moving:
        raise RuntimeError("robot did not start driving; moving-refusal checks cancelled")
    # while it moves: OTA and pose reset must be refused
    r = call("POST", "/api/pose/reset")
    report(3, "pose reset refused while moving", not r.get("ok", True), r.get("error", "accepted!"))
    def ota_probe(password):
        boundary = "x"
        body = (f"--{boundary}\r\nContent-Disposition: form-data; name=\"firmware\"; filename=\"t.bin\"\r\n"
                f"Content-Type: application/octet-stream\r\n\r\nXX\r\n--{boundary}--\r\n").encode()
        req = urllib.request.Request(f"http://{HOST}/api/ota", data=body, method="POST")
        req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
        req.add_header("X-OTA-Pass", password)
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                return json.loads(resp.read().decode())
        except urllib.error.HTTPError as e:
            return json.loads(e.read().decode() or "{}")
    ota = ota_probe("not-the-password")
    report(3, "wrong OTA password refused", not ota.get("ok", True) and "รหัส" in ota.get("error", ""),
           ota.get("error", "accepted!"))
    password = os.environ.get("MORLUAM_OTA_PASS")
    if password:
        # Invalid image bytes cannot install firmware if the moving guard breaks.
        ota = ota_probe(password)
        report(3, "authenticated OTA refused while moving",
               not ota.get("ok", True) and "กำลังวิ่ง" in ota.get("error", ""),
               ota.get("error", "accepted!"))
    else:
        print("  SKIP  authenticated OTA motion guard: MORLUAM_OTA_PASS is not set")
    time.sleep(max(0.0, 1.2 - 0.3))
    t_stop = time.time()
    estop()
    s = wait_until(lambda s: s["robot"]["mode"] == "halt" and abs(s["robot"]["rpm"]) < 1, timeout=3)
    rows = trace(4)
    save_trace("drive_3_estop", rows)
    stopped = next((r for r in reversed(rows) if int(r["flags"]) & 2), None)
    after = [r for r in rows if stopped and r["t_ms"] > stopped["t_ms"]]
    roll = math.hypot(after[-1]["x_m"] - after[0]["x_m"], after[-1]["y_m"] - after[0]["y_m"]) if len(after) > 1 else 0
    pwm_zero = next((r["t_ms"] - stopped["t_ms"] for r in after if int(r["pwm"]) == 0), None) if stopped else None
    report(3, "E-STOP while driving", s is not None,
           f"halted, wheel stopped within {time.time() - t_stop:.2f} s of the request (incl. Wi-Fi), "
           f"pwm 0 at the next tick ({pwm_zero} ms), odometry after the stop {roll * 1000:.0f} mm")
    go_home("4_return")


def go_home(tag):
    """drive straight back to (0, 0) of the last pose reset"""
    for attempt in range(6):
        st = status()["robot"]
        dist = math.hypot(st["x"], st["y"])
        if dist <= 0.08:
            return
        if dist > 0.7:
            raise RuntimeError("robot is over 0.7 m from the start; automatic return cancelled")
        leg = min(dist, DRIVE_DIST_M)
        countdown(f"drive {leg:.2f} m towards the start at {DRIVE_RPM} rpm")
        drive_once(DRIVE_RPM, math.degrees(math.atan2(-st["y"], -st["x"])), leg, f"{tag}_{attempt + 1}")
    raise RuntimeError("robot did not return within six short drives")


# ---- step 4: routes ------------------------------------------------------------------------

def run_route(points, heartbeat=True, timeout=60.0, tag="route"):
    if not call("POST", "/api/waypoints", {"points": points}).get("ok"):
        raise RuntimeError("route points refused")
    r = call("POST", "/api/nav/start")
    if not r.get("ok"):
        raise RuntimeError(f"route refused: {r.get('error')}")
    t0 = time.time()
    plans, last_hb, log = [], 0.0, []
    while time.time() - t0 < timeout:
        if heartbeat and time.time() - last_hb >= 1.0:
            call("POST", "/api/nav/heartbeat", timeout=2)
            last_hb = time.time()
        s = status()
        nav, rb = s["nav"], s["robot"]
        log.append((round(time.time() - t0, 2), nav["status"], nav["index"], round(rb["x"], 3), round(rb["y"], 3),
                    rb["mode"], nav["plan"]["kind"], nav.get("heartbeatAgeMs", -1)))
        p = nav["plan"]
        key = (nav["index"], p["kind"], round(p["phiDeg"]), round(p["a"], 2), round(p["betaDeg"]), round(p["b"], 2))
        if nav["status"] == "running" and p["distM"] > 0 and (not plans or plans[-1] != key):
            plans.append(key)
        if nav["status"] != "running":
            break
        time.sleep(0.2)
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{tag}_status.csv"), "w") as f:
        f.write("t_s,status,index,x,y,mode,plan,heartbeat_age_ms\n")
        f.writelines(",".join(map(str, row)) + "\n" for row in log)
    end = status()
    if end["nav"]["status"] == "running":
        estop()
        raise RuntimeError("route timed out; E-STOP sent")
    return end, plans, time.time() - t0


def step4():
    print("\nStep 4 - routes (Detour Steer), heartbeat stop")
    if not call("POST", "/api/pose/reset").get("ok"):
        raise RuntimeError("pose reset refused before route test")
    s = call("GET", "/api/settings")
    keys = ("planner", "navLoop", "navSpeedMps", "steerDps", "navTolM")
    saved = {k: s[k] for k in keys}
    try:
        if not call("POST", "/api/settings", dict(planner="detour", navLoop=False, navSpeedMps=0.03,
                                                   steerDps=60.0, navTolM=0.02)).get("ok"):
            raise RuntimeError("route test settings refused")
        wheel = status()["robot"]["wheelHeadingDeg"]
        # At 0.03 m/s, d=0.3 m and phi=354 give k=1.095 and beat a direct turn.
        # d=0.4 m loses after including the 0.20 s stop cost (host regression).
        b = math.radians(wheel + 6)
        goal = {"x": round(0.3 * math.cos(b), 3), "y": round(0.3 * math.sin(b), 3)}
        countdown(f"route: to ({goal['x']}, {goal['y']}) - a Detour Steer case - then back to (0, 0)")
        end, plans, took = run_route([goal, {"x": 0.0, "y": 0.0}], tag="route_detour")
        nav, rb = end["nav"], end["robot"]
        det = [p for p in plans if p[1] == "detour"]
        report(4, "route with a Detour Steer leg", nav["status"] == "done",
               f"{nav['status']} in {took:.1f} s, ends at ({rb['x']:.2f}, {rb['y']:.2f}) "
               f"[goal (0,0), tolerance 0.02 m], overshoots {nav['overshoots']}")
        report(4, "planner chose a detour for the 354 deg case", bool(det),
               "plans: " + "; ".join(f"pt{p[0] + 1} {p[1]} phi {p[2]} a {p[3]} beta {p[4]} b {p[5]}" for p in plans))
        if nav["status"] != "done" or not det:
            raise RuntimeError("detour route check failed; stopping before the next motion test")

        countdown("route without heartbeat: must stop by itself after ~3 s")
        wheel = status()["robot"]["wheelHeadingDeg"]
        b = math.radians(wheel)
        far = {"x": round(rb["x"] + 0.25 * math.cos(b), 3), "y": round(rb["y"] + 0.25 * math.sin(b), 3)}
        end, _, took = run_route([far], heartbeat=False, timeout=10, tag="route_no_heartbeat")
        report(4, "no heartbeat -> robot stops", end["nav"]["status"] == "stopped" and 2.5 <= took <= 5.0,
               f"{end['nav']['status']} after {took:.1f} s: {end['nav']['message']}")
        estop()
        go_home("route_return")
    finally:
        estop()
        if not call("POST", "/api/settings", saved).get("ok"):
            raise RuntimeError("could not restore the original route settings")


def main():
    global HOST
    ap = argparse.ArgumentParser(description="Test the real mor_luam robot.")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--steps", default="1,2,3,4")
    ap.add_argument("--imu", action="store_true", help="require fresh gyro/acceleration and capture a stationary baseline before motion")
    a = ap.parse_args()
    HOST = a.host
    steps = [int(x) for x in a.steps.split(",") if x.strip()]
    try:
        if not estop():
            raise RuntimeError("initial E-STOP was not confirmed")
        st = status()
        print(f"robot {st['sys']['name']} fw {st['sys']['fw']} ({st['sys']['build']}), "
              f"IMU {'ok' if st['robot']['imuOk'] else 'NOT ok'}, steering sensor {'ok' if st['robot']['steerOk'] else 'NOT ok'}")
        if not (st["robot"]["imuOk"] and st["robot"]["steerOk"]) and any(s > 1 for s in steps):
            raise RuntimeError("sensors not ready; motion tests cancelled")
        if a.imu:
            if not st["robot"].get("imuMotionFresh"):
                raise RuntimeError("IMU motion reports are stale or unavailable; motion cancelled")
            print("Recording 4 s stationary IMU baseline; keep the robot still.")
            time.sleep(4)
            baseline = trace(3.5)
            save_trace("imu_baseline", baseline)
            if not baseline or any(int(r.get("imu_flags", 0)) != 3 or int(r["pwm"]) != 0 for r in baseline):
                raise RuntimeError("stationary baseline has stale IMU or powered samples; motion cancelled")
            gaps = [b["t_ms"] - a["t_ms"] for a, b in zip(baseline, baseline[1:])]
            if gaps and max(gaps) > 30:
                raise RuntimeError(f"control trace gap {max(gaps):.0f} ms exceeds diagnostic limit; motion cancelled")
            print(f"IMU baseline saved: {len(baseline)} samples, largest control gap {max(gaps, default=0):.0f} ms")
        for n, fn in ((1, step1), (2, step2), (3, step3), (4, step4)):
            if n in steps:
                fn()
                if any(not r[2] for r in results):
                    break
    except KeyboardInterrupt:
        report(0, "test run cancelled", False, "Ctrl+C")
    except Exception as e:  # noqa: BLE001 - any failure must still stop the robot
        print(f"\nERROR: {e}")
        report(0, "test run", False, str(e))
    finally:
        stopped = estop()
        print("E-STOP:", "sent" if stopped else "NOT CONFIRMED - switch the robot off!")
        if not stopped:
            report(0, "final E-STOP", False, "not confirmed")
    fails = [r for r in results if not r[2]]
    print(f"\n{len(results) - len(fails)} passed, {len(fails)} failed. Traces: {OUT}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
