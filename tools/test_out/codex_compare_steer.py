"""Session-specific attended A/B test after a fresh default boot.

Known original advanced limits: integral -1000..1000, PID output 0..664.
The controller adds base 210 after PID, so PID max290 produces final max500.
Do not reuse after custom advanced PID tuning without recording its limits.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_test as r

try:
    if not r.estop():
        raise RuntimeError("initial E-STOP not confirmed")
    initial = r.status()
    if initial["sys"]["build"] != "Oct  8 2026 08:58:54":
        raise RuntimeError("historical A/B script requires its original 08:58:54 build; use normal acceptance tests on later builds")
    if not initial["robot"].get("imuMotionFresh"):
        raise RuntimeError("fresh IMU motion reports required")
    original = r.call("GET", "/api/pid")["steer"]
    if original != [27.8, 0.26, 7.4, 0.0, 3.5]:
        raise RuntimeError("factory steering gains required for this session-specific test")
    print("Build:", initial["sys"]["build"])
    print("Recording stationary baseline, then four attended 90-degree turns: 664 / 500 / 500 / 664 PWM")
    time.sleep(4)
    baseline = r.trace(3.5)
    r.save_trace("codex_ab_baseline", baseline)
    if not baseline or any(x["imu_flags"] != 3 or x["pwm"] != 0 for x in baseline):
        raise RuntimeError("invalid stationary baseline")
    r.countdown("four 90-degree steering turns at normal/lower power")
    for i, cap in enumerate([664, 500, 500, 664], 1):
        pid_max = 664 if cap == 664 else cap - 210
        if not r.call("POST", "/api/pid", {"loop": "steer", "values": original + [-1000, 1000, 0, pid_max]}).get("ok"):
            raise RuntimeError("PID limit refused")
        r.steer_once(90, f"ab_{i}_cap{cap}")
        time.sleep(0.5)
finally:
    stopped = r.estop()
    if "original" in globals():
        restored = r.call("POST", "/api/pid", {"loop": "steer", "values": original + [-1000, 1000, 0, 664]}).get("ok")
        print("Original default PID limits restored:", restored)
        if not restored:
            raise RuntimeError("PID limit restore failed")
    print("E-STOP confirmed:", stopped)
    if not stopped:
        raise RuntimeError("E-STOP not confirmed")
