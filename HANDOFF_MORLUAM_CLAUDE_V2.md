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
  NOT re-run after this: steps 3-4 (drive/route) - run them next with the owner present.

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
