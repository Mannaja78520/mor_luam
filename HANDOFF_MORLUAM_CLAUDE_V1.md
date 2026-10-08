# HANDOFF — mor_luam firmware v2 (Claude Opus 5.5 → next agent)

## LATEST UPDATE (read this first - newer than the sections below)
**Finished since the first version**
- Steering direction fixed (angles::cwErrorDeg = (current - target) mod 360, rate sign flipped) + AS5600 spike
  filter (util/AngleSpikeFilter.h). On the robot: steering test 7/7 PASS, stops ~3.4 deg short (safe side), real speed ~100 deg/s.
- Drive test: E-STOP OK (pwm 0 next tick), pose reset refused while moving. OTA refusal only proven for a wrong password.
- Measured: drive wheel full power = ~9.7 rpm = 0.039 m/s (user confirmed real distance matches odometry).
  Open-loop curve PWM 300/450/600/800/1000 -> rpm 1.41/3.34/5.46/7.66/9.62 (~ PWM = 160 + 85*rpm, battery 12.2 V).
- New drive PID flashed (build Oct 8 2026 07:42:34): Kp 40, Ki 30, Kd 0, Kf 85, KS 160 (new start-power term), deadband 0.3 rpm.
  Route speed default 0.03 m/s (7.5 rpm), max 0.035 (settings + web). Planner: goal inside steering tolerance = aimed (no detour).
- Robot runs on battery now; OTA over WiFi works on battery. PC tests ALL PASS.

**Still in progress**
- Auto PID for battery drop (user request): src/algorithm/DriveGainLearner.h is WRITTEN BUT NOT WIRED IN.
  Plan: in SteerDriveController drive branch use output = spin_.compute(...) with PIDF Kf set to 0 (or compute F outside)
  + gainLearner_.feedforward(rpmSet, KS, KF); call gainLearner_.observe(...) every drive tick (pwm magnitude, dt 0.01,
  seconds since entering Drive); expose gain in RobotState/status/web PID card; persist in NVS like coastS (App::saveLearnedCoast).
  Add a PC test in test_host/tests.cpp (gain follows a weaker motor model). No battery ADC exists on this robot.
- New drive PID NOT yet verified on the robot: run a 7.5 rpm drive and check rise time / steady rpm from the trace.

**Next**
1. Finish DriveGainLearner wiring + test + flash, then `python tools/robot_test.py --steps 3` (script still uses 40/63 rpm:
   change drive_once calls to 7.5 rpm and <= 0.25 m first).
2. `--steps 4` routes: with v = 0.03 m/s Detour Steer only pays for goals just past the wheel (k = d*w/v*|sin phi| < 2);
   update step 4 goal (e.g. d 0.4 m, 6 deg counter-clockwise of the wheel) and check the plan.
3. Rest of the old "Next" list below (WiFi fallback live test, web OTA form, passwords, commit when asked).


Date 2026-10-08. Any agent (Codex/ChatGPT, Gemini/Antigravity, Claude) can continue from here.
Read in this order: this file → `CLAUDE.md` (source map + commands) → only the files your task needs.

## สรุปสั้น (ภาษาไทย)
- เฟิร์มแวร์ใหม่ (OOP, เว็บแอป, mDNS, WiFi หลายวงตามลำดับ, OTA, Detour Steer, PIDF ใหม่) **อยู่ในหุ่นจริงแล้ว** อัปเดตผ่าน WiFi ได้
- **ยังไม่ commit / push** ทุกอย่างอยู่ใน working tree บน commit `91130c0` (= GitHub `main`)
- **ปัญหาค้างข้อ 1 (สำคัญสุด):** ล้อเลี้ยวหมุน "ทางไกล" ทุกครั้ง เพราะทิศมุมของเซนเซอร์กับสูตร error สวนกัน
  (มีมาตั้งแต่ `main` เดิม ไม่ได้เกิดจากการ refactor) ต้องถามผู้ใช้ว่ามองจากด้านบน ล้อเลี้ยว **ตามเข็ม** หรือ **ทวนเข็ม** แล้วแก้ตามข้อ "Open issue 1"
- ทดสอบบนหุ่นจริงแล้ว: ขั้น 1 (ไม่ขยับ) ผ่าน 5/5, ขั้น 2 (เลี้ยวอยู่กับที่) ผ่าน 4/7 → เจอปัญหาข้อ 1 ยังไม่ได้ทดสอบขั้น 3–4 (วิ่งจริง)

## Status at a glance
**Finished**
- Firmware v2 (OOP), web app, mDNS, WiFi priority + move-up, OTA, Detour Steer on the robot, PIDF rework,
  ROS reconnect + rate fixes, test tools (trace, test move, `robot_test.py`), docs, Docker helper.
- On the real robot: flash (COM9), OTA, boot sensors, WiFi/mDNS/finder, web page, ROS, no-motion tests 5/5.

**Still in progress (stopped here)**
- Open issue 1 FIX APPLIED + flashed (build "Oct 8 2026 07:33:13", robot now on battery, OTA works on battery):
  `angles::cwErrorDeg` = (current − target) mod 360 (steering is clockwise = angle goes down; evidence: original
  code says STEER_MOTOR_DIR "ทางขวา" + homework "clockwise-only"); steering rate sign flipped; no hardware config change.
- AS5600 bad readings while the motor runs (30–55 % of samples jump 8–53°): new `util/AngleSpikeFilter.h`
  (ignore jumps > 6°/tick unless 3 readings agree), count in status `robot.steerGlitches`. PC tests ALL PASS.
- Next action: re-run `python tools/robot_test.py --steps 2` to confirm the fix on the robot, then steps 3–4.

**Next**
1. Get the steering direction answer → apply open issue 1 fix → `--steps 2` until 7/7.
2. `--steps 3`, `--steps 4` with the user watching; tune PID from the traces.
3. Live WiFi fallback test, web OTA form, settings/PID from the page; change default passwords; commit when the user asks.

## Repo state
- Repo `E:\GPS_Localize\old\mor_luam` (remote `https://github.com/Mannaja78520/mor_luam.git`, branch main).
- HEAD `91130c0` == `origin/main`. All work below is **uncommitted**. Do not commit/push unless the user asks.
- Hardware config is unchanged from main (user confirmed: "not change anything in hardware").
- Secrets: `firmware/config/network_secrets.h` (gitignored). Never print it. `GET /api/wifi` returns
  WiFi passwords in clear text (user asked for that) — never paste its output into chat or files.

## What was built (all verified to compile; `fw-test` ALL PASS)
| Area | Where |
|---|---|
| OOP firmware: App, Settings, ControlLoop (100 Hz FreeRTOS task), SteerDriveController, Odometry, hw wrappers | `firmware/src/{app,control,hw}` |
| Steering algorithms, plain C++: Detour Steer (HW04), Direct, factory, SteerStopPredictor | `firmware/src/algorithm/` (+ README) |
| PIDF rework (D on measurement, seeded first sample, anti-windup, I-zone, ramp) | `firmware/lib/PIDF/` |
| WiFi: 6 saved networks in NVS, **priority = list order** (rejoin last → 1,2,3…; move up when higher one is back and robot is still), setup hotspot `mor-luam-XXXX`, mDNS `mor-luam.local` + `_module._tcp` | `firmware/src/net/` (`WifiPolicy.h` = rules) |
| OTA: web upload (`X-OTA-Pass` header) + ArduinoOTA; refused while moving | `firmware/src/net/OtaService.cpp` |
| micro-ROS bridge, domain 10, agent discovery (settings → gateway → AGENT_IP), reconnect fixed | `firmware/src/ros/MicroRosBridge.cpp` |
| Web app (Thai UI, 2D plane route editor, WiFi/settings/PID/OTA/fleet), source in `firmware/web/`, packed to `src/web/WebPage.h` by `web/embed.py` at build | `firmware/web/`, `firmware/src/web/WebApp.cpp` |
| Test tools on the robot: `POST /api/test/move` (≤150 rpm, ≤2 m), `GET /api/trace?s=` (last 15 s at 100 Hz, CSV) | `WebApp.cpp` `routesTest()`, `control/TraceRecorder.h` |
| PC tools: robot finder, real-robot test runner, mock robot for web work | `tools/find_robots.py`, `tools/robot_test.py`, `firmware/web/mock_robot.py` |
| Windows helper: fw-build / fw-flash -Port COM9 / fw-ota / fw-test / fw-clean-ros / find / web-mock / robot-test | `docker/mor_luam.ps1` via `docker\mor_luam.bat` |
| Docs: source map `CLAUDE.md`, Thai guide `docker/README.md`, algorithm guide | |

## Verified on the real robot
- First flash over USB **COM9** (CP210x), then ~10 OTA updates over WiFi (`fw-ota -Host 192.168.137.50`).
- Boot: IMU BNO08x OK, AS5600 OK; joins `manny` (PC hotspot) as **192.168.137.50**, `mor-luam.local`, finder works.
- Web page loads, no JS errors; WiFi scan endpoint (route-order bug found + fixed); priority move-up check does not switch by mistake.
- ROS: connects; 4/4 agent restarts reconnect in 6–10 s (fixed init-options leak); odom ~19 Hz (was ~2 Hz), small topics 2.5–4 Hz.
- `tools/robot_test.py --steps 1` (no motion): 5/5 PASS.

## Open issue 1 — steering turns the long way (BLOCKING, exists in `main` too)
Evidence (`tools/robot_test.py --steps 2`, traces in `tools/test_out/steer_*.csv`):
asked +90° → wheel turned −268° (measured angle DEcreases while steering); +345° → −15° in 0.23 s.
The controller assumes steering INcreases the angle (`angles::cwErrorDeg(t,c) = (t − c) mod 360`), so it
drives the long way, sees a large error right next to the target, arrives at full power and overshoots
(3/7 runs failed). `SteerStopPredictor` never engages (it needs rate > 0). The route overshoot check
(`prevECw<90 && eCw>270`) is also mirrored.

**Ask the user first:** looking down from above, does the wheel module steer clockwise or counter-clockwise?
- **Clockwise (homework says "clockwise-only")** → sensor frame (STEER_SENSE −1) is CCW-positive like the IMU;
  fix the math, not the hardware config:
  1. `src/util/Angles.h`: `cwErrorDeg(target, current) = wrap360(current − target)` (distance in the steering direction).
  2. `SteerDriveController::step()`: steering rate must be positive while steering:
     `rateDps_ = 0.5*rateDps_ + 0.5*errDeg(prevSteerDeg_, steerDeg_)/CTRL_PERIOD_S` (swap the args).
  3. Check every `cwErrorDeg` caller keeps its meaning ("how far the wheel must still turn"):
     controller eCw, `learnIfStopped`, `WaypointRunner` phi. Detour Steer magnitudes are mirror-symmetric (no change).
  4. Fix comments that say "steering increases the angle" (`algorithm/LegPlanner.h`, `CLAUDE.md`) and the mock
     (`firmware/web/mock_robot.py`: steering must DEcrease the wheel heading again).
- **Counter-clockwise** → set `STEER_SENSE (+1)` and `STEER_ZERO_OFFSET_DEG (224.0f)` (keeps the same zero) in
  `config/esp32_hardware.h`; no math change. (User said not to change hardware config — only if they agree.)

Then: `docker\mor_luam.bat fw-test`, `fw-build`, `fw-ota -Host 192.168.137.50`, and re-run
`python tools/robot_test.py --host 192.168.137.50 --steps 2` → expect every steer to turn ≈ the asked angle,
"past target" a few degrees, coast samples > 0.

Also check in the traces: peak steering rate reads 2000–5800 °/s during motion — likely AS5600 glitches
(I2C noise from the motor?) rather than real speed. If so, filter `rateDps_` / reject jumps before
trusting the predictor.

## Next steps (in order)
1. Open issue 1 (above), then `--steps 2` until 7/7.
2. `--steps 3` (drives ≤ 0.5 m and back at 40/63 rpm, E-STOP while driving, OTA + pose reset refused while moving).
3. `--steps 4` (route with a Detour Steer leg and back; route without heartbeat must stop after ~3 s).
   Ask the user to watch whether the robot goes where the web plane shows (+x = facing at pose reset, +y = left).
4. PID tuning from the traces (`steer_*.csv`, `drive_*.csv`): rpm rise / steady error; steer overshoot.
5. Not yet tested live: WiFi fallback to network 2 and move-up (needs the hotspot off — user's action),
   setup hotspot after 20 s, OTA from the web page form, settings/PID edit from the page on the robot.
6. Known limits: ROS link carries ~45 msg/s over hotspot+Docker; control tick uses ~6 ms of 10 ms
   (I2C at 100 kHz — faster clock possible but wiring unknown, ask first).
7. Security: default OTA / hotspot passwords still set — tell the user to change them (Settings tab).

## Safety rules for motion tests (user's robot)
- Only with the user present: robot on the floor, ~2 m clear (user confirmed this setup on 2026-10-08).
- `tools/robot_test.py` sends E-STOP on exit, error or Ctrl+C; manual stop: `curl -X POST http://192.168.137.50/api/estop`.
- Each step stays within ~0.7 m of the start. Never raise `TEST_MAX_RPM` / `TEST_MAX_DIST_M` without asking.

## Robot state at handoff
Halted (E-STOP sent). Firmware build "Oct 8 2026 07:19:13" (has trace + test endpoints). micro-ROS agent container stopped
(`docker\mor_luam.bat wifi` starts it). Mock server stopped.
