# HANDOFF — mor_luam (one file, always current)

Last update: 2026-11-04 (Claude). **This is the only live handoff.** Every agent (Claude, Codex,
Gemini, ...) edits THIS file when it stops: update "State", "Open items" and add one line to "Log".
Do not create new `HANDOFF_*_V<n>.md` files. Old handoffs (evidence, measurements, root-cause
notes) are kept unchanged in [`docs/handoff_history/`](docs/handoff_history/README.md).

Source map: `CLAUDE.md`. Install: `docs/INSTALL_TH.md`. Safety: `docs/safety.md`.

## State

**Firmware / robot**
- Firmware v2 (ESP32, PlatformIO, micro-ROS over Wi-Fi, own web app, OTA, mDNS `mor-luam.local`).
  The robot IP changes; use `mor-luam.local` or `docker\mor_luam.bat find`.
- Wheel steers counter-clockwise only (STEER_SENSE +1, zero 138.6 deg, checked with the owner 2026-10-08).
- Detour Steer runs on the robot. Demo button GPIO19: 1/2 clicks = routes set on the web,
  3/4 = Direct/Detour comparison, press = stop. Web "ทดสอบ 3 รอบ" runs a 3-round series.
- Straighter Direct legs (2026-10-10): STEER_LAND_DEG 2.0, IMU settle, early re-aim.
  Floor result, 3 rounds: Direct mean 23.49 s, Detour mean 21.31 s (9.3 % faster, 3/3 wins).
- Pose from the web (`/api/pose/reset`, `/api/pose/set`); demo press = new zero (`demoReset`).
- HTTP replies must stay small (a few KB): big replies reset the ESP32. Series data is paged.
- Newest compiled build (Codex V9, 2026-10-27: firmware.bin 1213664 bytes) was **not confirmed
  installed** — the robot was not reachable then. Read `sys.build` from status before any claim.

**PC side**
- `docker\mor_luam.bat web` = `tools/pc_web.py` on localhost:8000: offline editor + simulator,
  connects to the real robot when found (keeps the robot address through short loss, 9 tests pass).
- Multi-machine setup and automatic subnet discovery (Codex V8). Ubuntu: `docker/mor_luam.sh`.

**Algorithm / homework (271401)**
- `algorithm_sim/`: Python planner + simulators (no Docker), `docs/algorithm.pdf` (last page =
  Git link only), `docs/report.pdf` (report + Appendix A: Git link page, then code listings),
  `docs/660610840_DetourSteer.pdf` (Thai report, `xelatex` twice, font Leelawadee UI),
  `docs/explain_th.md` (Thai talk guide). Video link (Google Drive) is on every Git page. Tests: `python -m unittest test_current_robot`,
  `python detour_steer.py --no-bench` (T1-T4 PASS).
- Rebuild PDFs after code changes: `cd algorithm_sim/docs && pdflatex report.tex` (twice).
  The Python listing uses fixed lines 241-299 of `detour_steer.py`; re-check after editing it.
- Homework copy `E:\271401\271401_pee_aut_homework\algorithm\DETOUR_STEERING\` is a separate repo,
  not committed.

## Open items (all need the owner at the robot unless marked PC)

1. Find the robot again and read `sys.build`, halt state and PWM 0. Last seen identity 8FD570.
   `esp32-1C5A74` on the hotspot is a different device: do not flash it.
2. Measure Wi-Fi status reliability over a long read-only session; then decide on OTA of the newest build.
3. Real 2 m / 7-leg comparison (3 paired rounds) — not run yet (only model predictions exist).
4. Wi-Fi fallback to a second network / setup hotspot: logic unit-tested, not tried live.
5. OTA from the web form with the real password.
6. Change the default OTA and setup-hotspot passwords (Settings tab).
7. Shaking: mechanical cause not confirmed (`tools/imu_shake.py`).
8. Long run down to low battery not measured.
9. Old commit 91130c0 on GitHub holds a real Wi-Fi password still in use: the owner should change
   that password (history rewrite only if the owner asks).

## Rules that stay in force
- Never print `firmware/config/network_secrets.h`, `GET /api/wifi` or `GET /api/settings` output.
- OTA password only through env `MORLUAM_OTA_PASS`.
- Motion tests only with the owner present. Ask before changing `firmware/config/esp32_hardware.h`.
- Compiling is not proof of an installed build; check `sys.build` on the robot.

## Log (newest first, one line per session)
- 2026-11-04 Claude: Thai report 660610840_DetourSteer.pdf; video link added to all PDFs.
- 2026-11-04 Claude: added `algorithm_sim/` + report PDF with Git page; merged all handoffs into this file.
- 2026-10-27 Codex V2-V9: PC web/simulator, 2 m multi-leg comparison, Wi-Fi/status fixes, portable setup.
- 2026-10-08..10 Claude V1-V2, Codex V1: firmware v2, real-robot checks, demos, series, steering fixes.
