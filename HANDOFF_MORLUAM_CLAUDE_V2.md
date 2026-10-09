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

**Still in progress**
- Nothing running. Robot halted on battery.

**Next — all need the owner**
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
