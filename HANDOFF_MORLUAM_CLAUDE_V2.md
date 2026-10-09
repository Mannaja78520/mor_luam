# HANDOFF — mor_luam, Claude V2 (after Codex V1)

Date 2026-10-08. Newest handoff. Read this, then `HANDOFF_MORLUAM_CODEX_V1.md` (details + evidence),
then `CLAUDE.md` (source map). `HANDOFF_MORLUAM_CLAUDE_V1.md` is history.

## Status at a glance
**Finished (whole project request)**
- Firmware v2 (OOP), web app (route plane, WiFi, settings, PID, OTA, simulation), mDNS, WiFi priority + move-up,
  OTA, Detour Steer on the robot, PIDF rework + auto drive gain (battery), steering fixes, IMU diagnostics, safety guards.
- Codex V1: all real-robot checks passed (22/22, steps 1-4 incl. Detour route, heartbeat stop, E-STOP, OTA refusal).
- Claude V2 re-verified today (independent of Codex):
  - PC: `fw-test` ALL PASS, `node firmware/web/test_simulation.cjs` PASS, `tools/imu_shake.py --self-test` PASS,
    py_compile OK, `fw-build` OK (flash 59.1 %), `git diff -- firmware/config/esp32_hardware.h` empty.
  - Robot: build "Oct 8 2026 09:04:33" (Codex final), halted, PWM 0, IMU + AS5600 OK, WiFi priority 1,
    learned drive gain 1.0155 restored after a reboot.
  - Live WiFi drop + rejoin (`POST /api/wifi/reconnect`, robot halted): back on `manny` priority 1 after ~15 s, no reboot.
- Fixed: the robot's IP CHANGES (hotspot DHCP gave 192.168.137.212 instead of .50). Tools/docs now use
  `mor-luam.local` (resolves in 0.04 s on this PC): `tools/robot_test.py` default host, `docker\mor_luam.bat robot-test`,
  `docs/*.md`, `docker/README.md`, `PROMPT_FOR_NEXT_AI.md` (default OTA/AP passwords now live in gitignored `config/network_secrets.h` as MORLUAM_DEFAULT_OTA_PASS / MORLUAM_DEFAULT_AP_PASS; conf_network.h has placeholders).

- Later the same day: uploaded the newest web (Simulation with random error/shake + "real run mode 1 Direct /
  mode 2 Detour" timing panel, POST /api/nav/test) - it was in the source (16:36-16:39) but not on the robot.
  Robot build "Oct 8 2026 09:51:02" (UTC); verified on the robot page: both modes simulate, no JS errors,
  real-run buttons stay disabled until the operator checkbox. Not run for real (needs the owner at the robot).
- Thai web guide: `docs/web_guide_th.md` (how to open the web 3 ways + every tab/button), linked from docs/README.md and docker/README.md.

- GIT: everything committed as 233c2c8 on NEW branch `firmware-v2-web-ota`, pushed to origin (main untouched).
  Secrets: real values only in gitignored `firmware/config/network_secrets.h`; scanned all 195 files before commit.
  WARNING: an older commit already on GitHub (91130c0 and before, `conf_network.h` WIFI_PASS) contains a real WiFi
  password still in use - owner should change that WiFi password (history rewrite only if the owner asks).

- Ubuntu support + guides: `docker/mor_luam.sh` (same commands as the .bat; USB port via --port / MORLUAM_PORT,
  compose maps it to /dev/ttyUSB0), `docs/install_and_run.md` (install + run for Windows / Ubuntu+Docker / Ubuntu native).
  Tested from Git Bash: fw-test, fw-build, find, fw-ota to the real robot (OK). Not run on a real Ubuntu PC yet.
- `web` command (bat + sh): finds the robot, prints where to connect (PC URL, phone IP, Wi-Fi to join, hotspot fallback)
  and opens the browser. `web-mock` opens the browser too (`--no-browser` to skip; E:/.claude/launch.json uses it).

- STEERING CALIBRATION (owner present, 2026-10-08): the robot drove RIGHT when told forward. Measured with drive
  tests: zero was 90 deg off AND left/right were mirrored (STEER_SENSE -1 was a guess in the original code).
  Fixed in esp32_hardware.h with the owner: STEER_SENSE +1, STEER_ZERO_OFFSET_DEG 138.6 (wheel set to the front by hand).
  So the wheel steers COUNTER-clockwise from above and the angle increases: angles::cwErrorDeg back to (target - current),
  steering rate sign, web simulation, mock, tests, robot_test.py, docs all updated. Verified on the robot:
  forward 10 cm -> forward, left 10 cm -> left (owner watched), steering test 7/7. PC tests ALL PASS.
  Steps 3-4 re-run after the calibration (owner present): step 3 all PASS (drives 7.7 rpm steady, rise 0.29 s,
  E-STOP pwm 0 next tick, pose reset + authenticated OTA refused while moving); step 4 Detour route PASS
  (detour chosen at phi 354, done 39.9 s, back at (0.01, 0.02), 0 overshoots); no-heartbeat route PASS
  (stopped after 3.4 s, 8.5 cm) once ALL robot web pages were closed - an open page sends heartbeats.
  robot_test.py fix: heartbeat now in its own thread + mDNS name resolved once (a ~3 s PC-side stall had
  starved the heartbeat and stopped a route), one lost status answer no longer aborts the test.

- SIGNAL-LOSS STOP hardened (owner asked "make sure it never runs past the signal-loss time"):
  the 3 s heartbeat deadline was only checked in loop(), which Wi-Fi scans / mDNS searches / micro-ROS can block
  for seconds, and /api/test/move had no deadline at all. Now: ControlLoop watchdog (checked every 10 ms in the
  control task) armed by web routes, route tests and test moves; fed by /api/nav/heartbeat AND every GET /api/status
  (one lost HTTP request - Windows retries a lost connection after ~3 s - had caused a false stop); scans/searches
  wait while the robot moves; a ROS command disarms it. Measured on the robot: silent route 2.97 s, test move 3.01 s,
  scan+search during the drive still < 3 s, 0 mm after the cut; steps 2-3 13/13 PASS with 0 false trips.
  robot_test.py: one KEEPALIVE heartbeat thread for all test moves; robot name resolved once.

- DETOUR vs DIRECT RACE on the robot (tools/race_test.py, owner present): goal 0.3 m, 6 deg past the wheel,
  goal radius 2.5 mm (new setting minimum, owner asked for +-2.5 mm; navTolM is now 0.0025 on the robot).
  All 6 runs reached <= 2.5 mm (odometry). Direct 27.56/23.26/26.69 s (mean 25.84), Detour 19.63/20.00/20.34 s
  (mean 19.99) -> Detour 5.8 s faster (-22.6 %) and steadier. Planner predicted only ~0.2-1.5 s because it
  plans with steerDps 60 deg/s while the real wheel averages ~35 deg/s -> consider setting steerDps ~35.
  Fixes found on the way: steering speed now over 50 ms (one-count AS5600 flicker read as +-4 deg/s and
  blocked "wheel still" checks); steering tolerance hysteresis 1 deg (edge flicker held the wheel 8 s);
  route-test alignment uses the controller's own steerAimed flag. Earlier 5 cm-radius races were invalid
  (Detour "arrived" without its final turn); one run was touched by hand and discarded.

- DEMO BUTTON (2026-10-09, owner request): button on GPIO19 to GND with pull-up. 1 click = demo 1 (pose reset,
  (1,0) -> (1,1) -> (0,0), web route kept), 2 clicks = web route Direct, 3 clicks = web route Detour (both via the
  timed route test, wheel to +x first), any press while moving = stop, hold > 1.5 s = nothing. Button demos skip
  the web heartbeat (operator at the robot) with a 5 min cap. Files: app/DemoButton.*, util/ClickCounter.h,
  WaypointRunner startDemoSquare/startTest(byButton)/noteButton, web live button state. PC tests: click counter 7/7,
  route runner 18 scenarios. On the robot (build Oct 9 06:39): live pressed state works, long hold ignored.
  Owner tried demos 2/3: demo 3 could not start after demo 2 ("all points already reached" - the robot stayed at
  the goal). Fix: button demos 2/3 record the timed result at the goal, then drive back to where they started
  (untimed, Direct); demo start retries ~2 s if the wheel is still settling. PC: route runner 20 scenarios.
  Robot build Oct 9 08:49 (UTC). Two OTA attempts failed because the robot rebooted mid-upload (owner was
  power-cycling between demos); third attempt with the robot idle succeeded.
  Owner's runs on the OLD firmware (not comparable - different start points): Detour 21.2 s, Direct 19.5 s.
- BUTTON ROUTES (2026-10-09, owner request): owner saw demos 2/3 behave the same (web point 6 deg off = inside
  the steering tolerance after alignment, so both drove straight). New mapping:
  - 1 / 2 clicks = button route 1 / 2, points set on the web page (route tab -> "แก้เส้นทาง" -> ปุ่ม 1/2 ครั้ง ->
    บันทึกลงหุ่น). Saved in NVS "nav" (r1n/r1p, r2n/r2p); default square 1 m / triangle 0.5 m. Pose reset at the
    press: (0,0) = where the robot stands, +x = its front. Drives back to the start after the last point.
  - 3 / 4 clicks = Direct / Detour, timed, to one goal DEMO_COMPARE_DIST_M 0.20 m, DEMO_COMPARE_RIGHT_DEG 10 deg
    clockwise of the wheel AFTER alignment (phi = 350 exactly, planner picks the detour with steerDps 35..60).
    Then back to the start, untimed.
  - API: GET /api/waypoints?slot=1|2, POST {slot, points}. Slot routes can be edited while a demo runs.
    GET /api/waypoints (web route) now returns the web route even while a button demo borrows pts_.
  - Fixed: after resetPose the first leg waited for a fresh control snapshot (old pose could plan a wrong leg).
  - Code: WaypointRunner startButtonRoute/startCompare (startDemoSquare and startTest's byButton removed),
    DemoButton 1-4, web RouteModel slots + RouteSlotBar, mock_robot.py slots.
  - Checks: fw-test ALL PASS + route runner 24 scenarios; node test_route_test.cjs (slot load/save/edits),
    test_simulation(.cjs/_noise) PASS, test_mock_route_test.py OK; page checked at 360 px with the mock.
    fw-build OK (flash 59.6 %). OTA OK -> robot build "Oct 9 2026 09:21:34", idle; on the robot
    GET slot 1/2 = defaults, POST slot 2 ok, slot 3/5 refused. Button presses NOT yet tried by the owner.

- PER-POINT WAIT + DEMO 3/4 HOLD (2026-10-09, owner request):
  - Each point has `waitS` (0-60 s, NAV_MAX_WAIT_S): once reached, the robot stops there that long, then goes on.
    Web: "รอ (s)" column in the point table, "3s" label on the plane, countdown "อีก x วินาที" (nav.waitLeftMs).
    Works for the web route and button routes 1/2; the route test time includes waits; the simulator counts them.
  - Demos 3/4 hold DEMO_COMPARE_HOLD_S 5 s at the goal, then drive home. Time is recorded on arrival (hold excluded).
  - Storage: Waypoint is now x, y, waitS (12 bytes). readPoints() also reads the old 8-byte x/y blobs, so routes
    saved by older firmware are kept. Checked on the robot: web route + route 2 (old format) loaded unchanged.
  - Checks: fw-test ALL PASS + route runner 25 scenarios (wait at a point; 5 s hold before home, time unchanged),
    node route/simulation tests PASS, mock: countdown at point 1 then on to point 2; table fits at 360 px.
    OTA OK -> robot build "Oct 9 2026 09:36:59"; robot refused waitS 61, saved 2.5, route 2 restored to no waits.
    Button demos with the hold NOT yet tried by the owner.

- DYNAMIC BUTTON-DEMO LIMIT (2026-10-09, owner request "make it dynamic"):
  - The limit counts MOVING time only: localStartMs_ moves forward by every stop (point waits, demo 3/4 hold).
  - Button routes 1/2: limit = 2 x estimate (each leg at the set speed + 360 deg at 20 deg/s, the drive back
    included), clamped to 5-30 min (DEMO_MAX_MS, DEMO_LIMIT_CEIL_MS, DEMO_TIME_FACTOR, DEMO_SLOW_STEER_DPS).
    Demos 3/4 keep 5 min. Status: nav.demoLimitS / nav.demoMovingS; web hint "วิ่งแล้ว m:ss จาก m:ss นาที".
  - Checks: fw-test ALL PASS + route runner 27 scenarios (limit 790 s for a 10 m route, moving time frozen
    during a 30 s stop, still running after 5 min, stops past its own limit; 5 min floor; 30 min ceiling).
    fw-build OK. OTA OK -> robot build "Oct 9 2026 09:55:19", idle; status has demoLimitS/demoMovingS
    (0 while idle), saved points intact. A button demo with the new limit NOT yet run by the owner.

- DEMO 3/4 PICTURE + CLEARER FLOOR DIFFERENCE (2026-10-09, owner: "make it easier to see", all 3 options + "does it help?"):
  - Goal moved to the race geometry measured 2026-10-08: DEMO_COMPARE_DIST_M 0.30 m, DEMO_COMPARE_RIGHT_DEG 6 deg
    (was 0.20 / 10). Robot setting steerDps 60 -> 35 (measured wheel average; owner approved). Planner now:
    Direct turns 354 deg in place, then 0.30 m; Detour drives 0.31 m, turns 249 deg, last leg 3.4 cm.
  - Robot records each demo 3/4 run (nav/RunTrace.h, a point per 5 mm, thins itself when full): start, wheel heading,
    goal, time, valid, path. `GET /api/demo/compare` {distM, rightDeg, direct, detour}. Lost on reboot.
    status nav.test.demo / goalX / goalY.
  - Web: card "เดโม 3 / 4 ครั้ง" (js/47_demo_compare.js): both runs in their start frame, plan dashed, real solid,
    turn-in-place rings with the angle, times + "Detour เร็วกว่า x s (y%)". Colours validated with the dataviz
    validator (light #eb6834/#1baf7a, dark #d95926/#199e70). Under the plane on wide screens, last on phones.
  - `tools/demo_compare_plot.py [--wait]`: the same picture as a PNG (matplotlib, Thai font Leelawadee/Tahoma).
  - Checks: fw-test ALL PASS + route runner 28 scenarios (record of a finished and a stopped run) + RunTrace 5/5;
    node tests PASS (start frame, plan 354 / 31 cm + 249 deg); mock page checked at 1440 and 360 px, both themes'
    colours resolve. OTA OK -> robot build "Oct 9 2026 12:11:03"; /api/demo/compare empty until a run.
  - Does Detour help? Measured 2026-10-08 (race_test.py, 3+3 runs, goal 0.3 m / 6 deg): Detour 19.99 s vs
    Direct 25.84 s = 22.6 % faster, and steadier. Only for goals just clockwise of the wheel (Direct must spin
    almost a full turn); elsewhere the planner picks the same straight path, so both are equal.
  - OWNER RAN DEMO 3 + 4 on the new build (robot on `manny` but new subnet, IP 10.139.24.49; use mor-luam.local):
    Direct 24.55 s, Detour 21.66 s -> Detour 2.89 s faster (11.8 %), 1 run each, both valid, both ended
    2.5 / 2.4 mm from the goal (odometry). Direct drifted ~1 cm right of its line, then corrected at the end.
    Picture from tools/demo_compare_plot.py matched the plan (Direct 354 deg first; Detour 31 cm, 249 deg).
    Only 1 run each: repeat 3+ times before quoting a percentage (the 2026-10-08 race: 22.6 % over 3+3).

- DEMO 3/4 TIME LINE (2026-10-09, owner: "the paths look the same, make the difference easy to see"):
  - The robot records what it does during each timed run (nav/ActivityLog.h, change points debounced 150 ms):
    turn (steer mode, wheel > 3 deg/s), drive (drive mode, > 0.5 rpm), still. Returned as `acts` [{t, a}].
    FIRST VERSION WAS WRONG on the robot: it used rpm only, and the drive encoder also moves while the wheel
    steers, so a turn in place read as "drive". Now uses RobotState.driving. Test added (30 scenarios).
  - Web card: time line on top (pink hatched = turning, blue = driving, grey = still, dashed box = time saved),
    totals per run under it. PNG tool: time line + path. Colours validated (light #e87ba4/#2a78d6, dark #d55181/#3987e5).
  - `POST /api/demo/compare/start {planner, ready:true}`: demo 3/4 from the web/scripts, with the heartbeat rule.
  - Real runs today (owner present, robot on `manny` 10.139.24.49, build stamp "Oct 9 2026 12:37:47" + fix):
    | pair | how | Direct | Detour | Detour faster |
    | 1 | button | 24.55 s | 21.66 s | 2.89 s |
    | 2 | API (time line not yet right) | 22.70 s | 20.79 s | 1.91 s |
    | 3 | API | 24.59 s | 20.19 s | 4.40 s (17.9 %) |
    Mean Direct 23.95 s, Detour 20.88 s -> Detour 3.07 s (12.8 %) faster; Detour won all 3 pairs.
    Pair 3 time line: Direct turn 9.4 s -> drive 10.1 s -> corrections 4.7 s; Detour drive 10.4 s -> turn 7.2 s
    -> short drive/adjust 2.4 s. Both drift ~1.3 cm right while driving, then correct; both end 2.5 mm from goal.

- "TEST 3 ROUNDS" BUTTON (2026-10-09, owner: one press = 3 rounds + this comparison):
  - Robot runs the series itself: WaypointRunner::startSeries(rounds 1..3) -> Direct + Detour per round, order
    D T / T D / D T (battery/drift fairness), each run aligns, is timed, holds 5 s, drives home; 2 s pause
    (DEMO_SERIES_GAP_MS); next run from update(). Each closed run is copied into seriesRuns_[6] (path + time line).
    Any stop / failure ends the series (why kept). A web series does not start the next run without a heartbeat
    in the last 3 s. While a series runs, start / route test / button routes / demo 3/4 are refused; a press of
    the robot's button stops it.
  - API: POST /api/demo/series/start {rounds, ready}, GET /api/demo/series (streamed; paths as flat xy),
    status nav.series {id, active, count, total, why}.
  - Web card: ready tick + "ทดสอบ 3 รอบ" + "หยุดชุดทดสอบ" + progress; results: table per round + mean + wins,
    time line for every run, all paths overlaid. PNG: `tools/demo_compare_plot.py --series [--wait]`.
  - Checks: fw-test ALL PASS + route runner 33 scenarios (order, refusals, stop between runs, lost page);
    node tests PASS; mock page at 1440 / 360 px. RAM 33.1 %, flash 60.7 %. OTA OK (robot on 10.139.24.49,
    free heap 72.6 KB, GET /api/demo/series answered).
  - NOT RUN ON THE FLOOR YET: the robot went offline (probably powered off) right before the first series.

**Still in progress**
- Nothing running. Robot halted on battery.

**Next — all need the owner**
0. Run "ทดสอบ 3 รอบ" from the web card once on the floor (owner present, ~6 min), then
   `python tools/demo_compare_plot.py --series` for the PNG.
   Try the button on the floor: 3 then 4 clicks (Direct vs Detour; watch the picture card; then
   `python tools/demo_compare_plot.py` for the PNG). Also 1 click (square 1 m), 2 clicks (triangle 0.5 m).
1. WiFi fallback to network 2 / move up / setup hotspot (`mor-luam-XXXX`, 192.168.4.1): needs a second saved network
   in range and the owner switching the `manny` hotspot off and on. Logic is unit-tested (`fw-test` part 4).
2. OTA from the web page form with the real password (helper/direct OTA already passed many times).
3. Change the default OTA and setup-hotspot passwords (Settings tab).
4. Shaking: reduced (steering power cap 500), mechanical cause not confirmed — check mounting; use `tools/imu_shake.py`.
5. Battery: the drive gain learns automatically; a long run down to a low battery has not been measured.
6. Merge `firmware-v2-web-ota` into main when the owner says so (PR or fast-forward).

## Commands
```
docker\mor_luam.bat find                       robot IP (it can change)
docker\mor_luam.bat fw-build
docker\mor_luam.bat fw-ota -Host mor-luam.local
docker\mor_luam.bat fw-test
python -X utf8 -u tools\robot_test.py --steps 1        (no motion; steps 2-4 move: owner present)
curl.exe -X POST http://mor-luam.local/api/estop
```
Never print `network_secrets.h`, `GET /api/wifi` or `GET /api/settings` output (passwords).
