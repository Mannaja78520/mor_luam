"""A fake mor_luam on the PC, to work on the web page without the robot.

    python web/mock_robot.py            then open http://localhost:8000
    python web/mock_robot.py --port 9000

Serves web/index.html, web/app.css and web/js/*.js straight from disk (edit,
then reload the browser: no build), and answers the same /api/... as
src/web/WebApp.cpp with a simple simulated robot: stop-to-steer, the wheel
steers clockwise only, Detour Steer or Direct planning, 3 s web heartbeat.

Test the page's bad days:
    curl -X POST "http://localhost:8000/mock/offline?s=8"   API answers 503 for 8 s
    curl -X POST "http://localhost:8000/mock/slow?ms=1800"  every API answer waits 1.8 s
    curl -X POST "http://localhost:8000/mock/sensors?ok=0"  IMU and steering sensor 'not ready'

All values here are made up - no real Wi-Fi password is in this file.
"""
import argparse
import json
import math
import os
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

WEB = os.path.dirname(os.path.abspath(__file__))
LOCK = threading.Lock()
T0 = time.time()


def now_ms():
    return int((time.time() - T0) * 1000)


class Robot:
    def __init__(self):
        self.x = self.y = 0.0
        self.theta = 0.0          # body yaw, deg, CCW
        self.wheel = 0.0          # wheel heading in the world, deg, CCW
        self.mode = "halt"
        self.source = "none"
        self.halt_why = "เพิ่งเปิดเครื่อง"
        self.rpm = 0.0
        self.goal = None          # end of the current drive leg
        self.target_heading = 0.0
        self.sensors_ok = True
        self.coast_s, self.coast_n = 0.10, 0
        self.points = [{"x": 1.0, "y": 0.0}, {"x": 1.0, "y": 1.0}, {"x": -0.5, "y": 1.0}]
        self.nav = {"status": "idle", "message": "", "index": 0, "tries": 0, "overshoots": 0}
        self.plan = {"kind": "direct", "phiDeg": 0, "distM": 0, "k": 0, "a": 0, "betaDeg": 0, "b": 0, "timeS": 0}
        self.heartbeat_ms = 0
        self.settings = {
            "robotName": "mor_luam", "hostname": "mor-luam", "agentHost": "", "agentPort": 8888,
            "navSpeedMps": 0.03, "navTolM": 0.08, "navLoop": False, "planner": "detour",
            "planners": ["detour", "direct"], "steerDps": 60.0,
            "otaPass": "mock-ota", "apPass": "mock-ap-pass",
        }
        self.pid = {"spin": [40.0, 30.0, 0.0, 85.0, 0.3], "steer": [27.8, 0.26, 7.4, 0.0, 3.5]}
        self.wifi = [{"ssid": "manny", "pass": "mock-pass-123"}, {"ssid": "Lab-IoT", "pass": ""}]
        self.scan = {"scanning": False, "at": 0, "nets": []}
        self.peers = {"searching": False, "at": 0, "peers": []}
        self.offline_until = 0.0
        self.slow_ms = 0
        self.ota_pct = -1
        self.route_planner = self.settings["planner"]
        self.route_loop = self.settings["navLoop"]
        self.test_id = 0
        self.pid_revision = 0
        self.test_started_ms = 0
        self.test = {"id": 0, "active": False, "phase": "idle", "planner": "direct", "startHeadingDeg": 0,
                     "actualStartHeadingDeg": None, "startX": None, "startY": None, "startThetaDeg": None,
                     "speedMps": 0.03, "tolM": 0.08, "steerDps": 60.0, "elapsedMs": 0,
                     "valid": False, "observationMaxGapMs": 20, "pidRevision": 0}

    def motion_settings(self):
        s = self.settings.copy()
        if self.test["active"]:
            s.update(navSpeedMps=self.test["speedMps"], navTolM=self.test["tolM"],
                     steerDps=self.test["steerDps"], navLoop=False)
        return s

    def start_test(self, planner, heading):
        self.test_id += 1
        self.route_planner = planner
        self.route_loop = False
        self.test.update(id=self.test_id, active=True, phase="aligning", planner=planner,
                         startHeadingDeg=heading % 360, actualStartHeadingDeg=None,
                         pidRevision=self.pid_revision,
                         startX=None, startY=None, startThetaDeg=None, elapsedMs=0, valid=False,
                         speedMps=self.settings["navSpeedMps"], tolM=self.settings["navTolM"],
                         steerDps=self.settings["steerDps"])
        self.nav.update(status="running", message="กำลังเตรียมมุมล้อ", index=0, tries=0, overshoots=0)
        self.heartbeat_ms = now_ms()
        self.target_heading = heading % 360
        self.mode, self.rpm, self.goal, self.source = "steer", 0.0, None, "web"

    def finish_test(self, phase):
        if self.test["active"]:
            if self.test["phase"] == "running":
                self.test["elapsedMs"] = now_ms() - self.test_started_ms
            self.test.update(active=False, phase=phase, valid=phase == "done")

    # ---- planning (same maths as src/algorithm/DetourSteer.cpp) ------------
    def make_plan(self, tx, ty):
        s = self.motion_settings()
        v, w = s["navSpeedMps"], s["steerDps"]
        dx, dy = tx - self.x, ty - self.y
        d = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx)) % 360
        phi = (self.wheel - bearing) % 360                  # how far the wheel must still turn (clockwise only)
        direct = phi / w + 0.05 + d / v
        plan = {"kind": "direct", "phiDeg": phi, "distM": d, "k": 0, "a": 0, "betaDeg": phi, "b": d, "timeS": direct}
        if self.route_planner == "detour" and phi > 180:
            k = d * math.radians(w) / v * abs(math.sin(math.radians(phi)))
            plan["k"] = k
            if k < 2:
                beta = 360 - math.degrees(math.acos(k - 1))
                sb = math.sin(math.radians(beta))
                a = d * math.sin(math.radians(beta - phi)) / sb
                b = d * math.sin(math.radians(phi)) / sb
                t = a / v + 0.2 + beta / w + 0.05 + b / v
                if a > 0 and b > 0 and t < direct:
                    plan.update(kind="detour", betaDeg=beta, a=a, b=b, timeS=t)
        return plan, bearing

    def start_leg(self):
        tgt = self.points[self.nav["index"]]
        self.plan, bearing = self.make_plan(tgt["x"], tgt["y"])
        if self.plan["kind"] == "detour":   # drive a along the wheel as it is, no steering
            h, dist = self.wheel, self.plan["a"]
        else:
            h, dist = bearing, self.plan["distM"]
        self.target_heading = h
        self.goal = (self.x + dist * math.cos(math.radians(h)), self.y + dist * math.sin(math.radians(h)))
        self.leg_left = dist                # like the firmware: drive a distance along the heading
        self.mode = "steer"
        self.source = "web"

    def stop(self, why, status="stopped"):
        self.mode, self.rpm, self.goal = "halt", 0.0, None
        self.halt_why = why
        if self.nav["status"] == "running":
            self.nav.update(status=status, message=why)
        self.finish_test("failed" if status == "failed" else "stopped")

    # ---- 50 Hz simulation ---------------------------------------------------
    def step(self, dt):
        s = self.motion_settings()
        if self.nav["status"] == "running":
            if now_ms() - self.heartbeat_ms > 3000:
                self.stop("หน้าเว็บเงียบเกิน 3 วินาที")
                return
            if self.test["active"] and self.test["phase"] == "aligning":
                err = (self.wheel - self.target_heading) % 360
                if err <= 3.5 or err >= 356.5:
                    self.test_started_ms = now_ms()
                    self.test.update(phase="running", actualStartHeadingDeg=self.wheel,
                                     startX=self.x, startY=self.y, startThetaDeg=self.theta)
                    self.mode = "halt"
                    self.nav["message"] = ""
                else:
                    self.wheel = (self.wheel - min(err, s["steerDps"] * dt)) % 360
                    return
            if self.mode == "halt":
                tgt = self.points[self.nav["index"]]
                if math.hypot(tgt["x"] - self.x, tgt["y"] - self.y) <= s["navTolM"]:
                    self.nav["index"] += 1
                    if self.nav["index"] >= len(self.points):
                        if s["navLoop"]:
                            self.nav["index"] = 0
                        else:
                            self.nav.update(status="done", message="ถึงจุดสุดท้ายแล้ว", index=len(self.points) - 1)
                            self.halt_why = "ถึงจุดสุดท้ายแล้ว"
                            self.finish_test("done")
                            return
                    self.nav["message"] = ""
                self.start_leg()
        if self.mode == "steer":
            err = (self.wheel - self.target_heading) % 360
            if err <= 3.5 or err >= 356.5:
                self.mode = "drive"
            else:
                self.wheel = (self.wheel - min(err, s["steerDps"] * random.uniform(0.9, 1.1) * dt)) % 360   # clockwise only, like the robot
        elif self.mode == "drive":
            stepm = min(self.leg_left, s["navSpeedMps"] * dt)
            self.leg_left -= stepm
            h = math.radians(self.wheel)
            self.x += stepm * math.cos(h) + random.gauss(0, 0.0005)
            self.y += stepm * math.sin(h) + random.gauss(0, 0.0005)
            self.rpm = s["navSpeedMps"] / (math.pi * 0.1) * 60
            if self.leg_left <= 1e-6:
                self.mode, self.rpm, self.goal = "halt", 0.0, None
                self.halt_why = "ถึงปลายขาแล้ว"

    def status(self):
        s = self.settings
        steer = (self.wheel - self.theta) % 360
        err = (self.wheel - self.target_heading) % 360
        n = len(self.points)
        test = self.test.copy()
        if test["active"] and test["phase"] == "running":
            test["elapsedMs"] = now_ms() - self.test_started_ms
        return {
            "robot": {
                "x": self.x, "y": self.y, "thetaDeg": self.theta, "headingDeg": self.theta,
                "wheelHeadingDeg": self.wheel, "steerDeg": steer, "steerTargetDeg": (self.target_heading - self.theta) % 360,
                "steerErrDeg": err if self.mode != "halt" else 0, "steerOk": self.sensors_ok, "imuOk": self.sensors_ok,
                "imuHeadingFresh": self.sensors_ok,
                "rpm": self.rpm, "targetRpm": self.rpm, "targetHeadingDeg": self.target_heading, "vx": 0, "pwm": 0,
                "mode": self.mode, "source": self.source if self.mode != "halt" else "none",
                "overshot": False, "overshootDeg": 0, "steerRateDps": s["steerDps"] if self.mode == "steer" else 0,
                "coasting": False, "coastS": self.coast_s, "coastSamples": self.coast_n,
                "driveGain": 1.0, "driveLearnedS": 0.0,
                "steerPowerLimit": 500, "imuMotionFresh": False, "imuGyroDps": 0.0, "imuAccelMps2": 0.0,
                "motionFault": "", "hardwareEstop": "not-wired", "groundContact": "unknown",
                "goalActive": self.goal is not None, "goalX": self.goal[0] if self.goal else 0,
                "goalY": self.goal[1] if self.goal else 0, "haltWhy": self.halt_why,
            },
            "nav": {**self.nav, "count": n, "loop": self.route_loop,
                    "planner": self.route_planner, "plan": self.plan, "test": test,
                    "heartbeatAgeMs": now_ms() - self.heartbeat_ms if self.nav["status"] == "running" else 0},
            "net": self.net(),
            "ros": {"state": "connected", "agent": "192.168.137.1:8888", "from": "gateway", "domain": 10,
                    "connects": 1, "upMs": now_ms()},
            "sys": {"fw": "mock", "build": "PC mock", "uptimeS": now_ms() // 1000, "heap": 182000, "tickUs": 840,
                    "otaPct": self.ota_pct, "name": s["robotName"]},
        }

    def net(self):
        return {"connected": True, "ssid": "manny", "rssi": -52 + random.randint(-3, 3), "ip": "192.168.137.50",
                "gateway": "192.168.137.1", "host": self.settings["hostname"] + ".local", "connecting": "",
                "priority": next((i + 1 for i, w in enumerate(self.wifi) if w["ssid"] == "manny"), 0),
                "ap": {"active": False, "ssid": "mor-luam-3A5F", "ip": "192.168.4.1", "clients": 0}}


R = Robot()


def sim_loop():
    while True:
        with LOCK:
            R.step(0.02)
            if R.scan["scanning"] and time.time() - R.scan["started"] > 2.5:
                R.scan.update(scanning=False, at=now_ms(), nets=[
                    {"ssid": "manny", "rssi": -50, "secure": True}, {"ssid": "Lab-IoT", "rssi": -67, "secure": False},
                    {"ssid": "Neighbour 5G", "rssi": -81, "secure": True}])
            if R.peers["searching"] and time.time() - R.peers["started"] > 2.0:
                R.peers.update(searching=False, at=now_ms(), peers=[
                    {"name": "mor_luam_2", "host": "mor-luam-2.local", "ip": "192.168.137.51", "type": "morluam", "id": "a1b2"},
                    {"name": "nong-module-1", "host": "nong-1.local", "ip": "192.168.137.60", "type": "module", "id": "c3d4"}])
        time.sleep(0.02)


def bundle_js():
    names = sorted(n for n in os.listdir(os.path.join(WEB, "js")) if n.endswith(".js"))
    return b"\n".join(b"// ---- " + n.encode() + b"\n" + open(os.path.join(WEB, "js", n), "rb").read() for n in names)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def ok(self):
        self.send(200, {"ok": True})

    def fail(self, err, code=400):
        self.send(code, {"ok": False, "error": err})

    def body(self):
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b""
        if self.headers.get("Content-Type", "").startswith("application/json") and raw:
            return json.loads(raw)
        return raw

    def gate(self, path):
        if not path.startswith("/api/"):
            return True
        if R.slow_ms:
            time.sleep(R.slow_ms / 1000)
        if time.time() < R.offline_until:
            self.send(503, b"offline", "text/plain")
            return False
        return True

    def do_GET(self):
        u = urlparse(self.path)
        p = u.path
        if p in ("/", "/index.html"):
            return self.send(200, open(os.path.join(WEB, "index.html"), "rb").read(), "text/html; charset=utf-8")
        if p == "/app.css":
            return self.send(200, open(os.path.join(WEB, "app.css"), "rb").read(), "text/css; charset=utf-8")
        if p == "/app.js":
            return self.send(200, bundle_js(), "application/javascript; charset=utf-8")
        if not self.gate(p):
            return
        with LOCK:
            if p == "/api/status":
                return self.send(200, R.status())
            if p == "/api/whoami":
                return self.send(200, {"type": "morluam", "id": "3a5f", "name": R.settings["robotName"],
                                       "host": R.settings["hostname"] + ".local", "ip": "127.0.0.1", "fw": "mock"})
            if p == "/api/waypoints":
                return self.send(200, {"points": R.points})
            if p == "/api/wifi":
                return self.send(200, {"saved": R.wifi, "max": 6, "net": R.net()})
            if p == "/api/wifi/scan":
                return self.send(200, {"scanning": R.scan["scanning"], "ageMs": now_ms() - R.scan["at"] if R.scan["at"] else 0,
                                       "nets": R.scan["nets"]})
            if p == "/api/settings":
                return self.send(200, R.settings)
            if p == "/api/pid":
                return self.send(200, R.pid)
            if p == "/api/peers":
                return self.send(200, {"searching": R.peers["searching"], "ageMs": now_ms() - R.peers["at"] if R.peers["at"] else 0,
                                       "peers": R.peers["peers"]})
        self.send(404, b"not found", "text/plain")

    def do_POST(self):
        u = urlparse(self.path)
        p, q = u.path, parse_qs(u.query)
        if p.startswith("/mock/"):
            with LOCK:
                if p == "/mock/offline":
                    R.offline_until = time.time() + float(q.get("s", ["8"])[0])
                elif p == "/mock/slow":
                    R.slow_ms = int(q.get("ms", ["0"])[0])
                elif p == "/mock/sensors":
                    R.sensors_ok = q.get("ok", ["1"])[0] == "1"
            return self.ok()
        if not self.gate(p):
            return
        if p == "/api/ota":
            return self.ota()
        b = self.body()
        with LOCK:
            running = R.nav["status"] == "running"
            if p == "/api/estop":
                R.stop("หยุดฉุกเฉินจากหน้าเว็บ")
                return self.ok()
            if p == "/api/nav/stop":
                R.stop("หยุดจากหน้าเว็บ")
                return self.ok()
            if p == "/api/nav/heartbeat":
                R.heartbeat_ms = now_ms()
                return self.ok()
            if p == "/api/nav/start":
                if running or R.mode != "halt":
                    return self.fail("หยุดหุ่นก่อน แล้วค่อยเริ่มเส้นทาง")
                if not R.points:
                    return self.fail("ยังไม่มีจุด: คลิกบนระนาบเพื่อวางจุด")
                R.test.update(active=False, phase="idle", valid=False)
                R.route_planner = R.settings["planner"]
                R.route_loop = R.settings["navLoop"]
                R.nav.update(status="running", message="", index=0, tries=0, overshoots=0)
                R.heartbeat_ms = now_ms()
                R.mode = "halt"
                return self.ok()
            if p == "/api/nav/test":
                if running or R.mode != "halt":
                    return self.fail("หยุดหุ่นก่อน แล้วค่อยทดสอบ")
                if not R.points or not R.sensors_ok:
                    return self.fail("ต้องมีจุดและเซนเซอร์พร้อมก่อนทดสอบ")
                heading = b.get("startHeadingDeg")
                if b.get("ready") is not True or b.get("planner") not in ("direct", "detour") or \
                        isinstance(heading, bool) or not isinstance(heading, (int, float)) or \
                        not math.isfinite(heading) or not 0 <= heading <= 360:
                    return self.fail("ยืนยันความพร้อม เลือกแบบ และระบุมุมเริ่ม 0–360°")
                R.start_test(b["planner"], heading)
                return self.send(200, {"ok": True, "nav": R.status()["nav"]})
            if p == "/api/pose/reset":
                if running or R.mode != "halt":
                    return self.fail("หยุดหุ่นก่อน แล้วค่อยตั้งจุดเริ่มต้นใหม่")
                R.x = R.y = 0.0
                R.wheel = (R.wheel - R.theta) % 360
                R.theta = 0.0
                return self.ok()
            if p == "/api/waypoints":
                pts = b.get("points", [])
                if running:
                    return self.fail("หยุดเส้นทางก่อน แล้วค่อยแก้จุด")
                if len(pts) > 32:
                    return self.fail("จุดได้ไม่เกิน 32 จุด")
                for i, pt in enumerate(pts):
                    if abs(pt["x"]) > 50 or abs(pt["y"]) > 50:
                        return self.fail(f"จุดที่ {i + 1} อยู่นอกช่วง ±50 m")
                R.points = [{"x": float(pt["x"]), "y": float(pt["y"])} for pt in pts]
                return self.ok()
            if p == "/api/wifi/save":
                ssid, pw, orig = b.get("ssid", ""), b.get("pass", ""), b.get("original", "")
                if not ssid or len(ssid.encode()) > 32:
                    return self.fail("ชื่อ WiFi ต้องยาว 1-32 ตัว")
                if pw and not 8 <= len(pw) <= 63:
                    return self.fail("รหัส WiFi ต้องยาว 8-63 ตัว (หรือเว้นว่างถ้าไม่มีรหัส)")
                hit = next((w for w in R.wifi if w["ssid"] == ssid), None)
                if hit:
                    hit["pass"] = pw
                elif len(R.wifi) >= 6:
                    return self.fail("บันทึกไม่ได้ (เก็บได้สูงสุด 6 วง)")
                else:
                    R.wifi.append({"ssid": ssid, "pass": pw})
                if orig and orig != ssid:
                    R.wifi = [w for w in R.wifi if w["ssid"] != orig]
                return self.ok()
            if p == "/api/wifi/delete":
                before = len(R.wifi)
                R.wifi = [w for w in R.wifi if w["ssid"] != b.get("ssid")]
                return self.ok() if len(R.wifi) < before else self.fail("ไม่พบ WiFi นี้")
            if p == "/api/wifi/move":
                i = next((k for k, w in enumerate(R.wifi) if w["ssid"] == b.get("ssid")), -1)
                to = int(b.get("to", 0))
                if i < 0 or not 0 <= to < len(R.wifi):
                    return self.fail("ย้ายไม่ได้")
                R.wifi.insert(to, R.wifi.pop(i))
                return self.ok()
            if p == "/api/wifi/reconnect":
                return self.ok()
            if p == "/api/wifi/scan":
                R.scan.update(scanning=True, started=time.time())
                return self.ok()
            if p == "/api/peers":
                R.peers.update(searching=True, started=time.time())
                return self.ok()
            if p == "/api/settings":
                n = {**R.settings, **{k: v for k, v in b.items() if k in R.settings and k != "planners"}}
                if not 1 <= len(n["robotName"]) <= 31:
                    return self.fail("ชื่อหุ่นต้องยาว 1-31 ตัวอักษร")
                if not n["hostname"] or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789-" for c in n["hostname"]):
                    return self.fail("hostname ใช้ได้แค่ a-z 0-9 และ - (ไม่เกิน 31 ตัว)")
                if len(n["otaPass"]) < 4:
                    return self.fail("รหัส OTA ต้องยาวอย่างน้อย 4 ตัว")
                if not 8 <= len(n["apPass"]) <= 63:
                    return self.fail("รหัส hotspot ต้องยาว 8-63 ตัว")
                if not 0.05 <= n["navSpeedMps"] <= 1.0:
                    return self.fail("ความเร็วต้องอยู่ระหว่าง 0.05-1.0 m/s")
                R.settings = n
                return self.ok()
            if p == "/api/pid":
                if b.get("loop") not in ("spin", "steer") or len(b.get("values", [])) < 5:
                    return self.fail("ต้องมี loop = spin/steer และค่า Kp Ki Kd Kf tol")
                R.pid[b["loop"]] = b["values"][:5]
                R.pid_revision += 1
                if R.test["active"]:
                    R.stop("PID เปลี่ยนระหว่างทดสอบ", "failed")
                return self.ok()
            if p == "/api/reboot":
                if R.mode != "halt":
                    return self.fail("หุ่นกำลังวิ่ง - หยุดก่อน")
                R.offline_until = time.time() + 6
                return self.ok()
        self.send(404, b"not found", "text/plain")

    def ota(self):
        n = int(self.headers.get("Content-Length") or 0)
        with LOCK:
            moving = R.mode != "halt" or R.nav["status"] == "running"
            good = self.headers.get("X-OTA-Pass", "") == R.settings["otaPass"]
        got = 0
        while got < n:                       # read slowly so the progress bar moves
            chunk = self.rfile.read(min(65536, n - got))
            if not chunk:
                break
            got += len(chunk)
            with LOCK:
                R.ota_pct = int(100 * got / n) if good and not moving else -1
            time.sleep(0.05)
        with LOCK:
            R.ota_pct = -1
            if moving:
                return self.fail("หุ่นกำลังเคลื่อนที่ - หยุดก่อนอัปเดต")
            if not good:
                return self.fail("รหัส OTA ไม่ถูกต้อง", 403)
            R.offline_until = time.time() + 8
        self.send(200, {"ok": True, "reboot": True})


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true", help="do not open the browser")
    a = ap.parse_args()
    threading.Thread(target=sim_loop, daemon=True).start()
    print(f"mock mor_luam on http://localhost:{a.port}  (Ctrl+C to stop)")
    server = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = f"http://localhost:{a.port}/"
    print(f"Open the web app:  {url}")
    if not a.no_browser:
        import webbrowser
        threading.Timer(0.5, webbrowser.open, [url]).start()   # the server is listening by then
    server.serve_forever()


if __name__ == "__main__":
    main()
