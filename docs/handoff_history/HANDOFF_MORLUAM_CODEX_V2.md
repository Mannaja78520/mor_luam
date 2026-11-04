# mor_luam handoff — Codex V2

Date: 2026-10-26, Asia/Bangkok. Scope: locate the installed checkout, explain
Docker usage, refresh all README files and publish to GitHub at the owner's request.
Earlier agents' handoffs were preserved.

## Repository and publication

- Actual checkout: `E:\GPS_Localize\old\mor_luam`.
- Remote: `https://github.com/Mannaja78520/mor_luam.git`.
- Started clean on `main` at `f6c8e3c`; `git fetch origin` confirmed no divergence
  from `origin/main`. Firmware/web changes were already on GitHub. The root
  README still described the earlier ROS workflow and had not followed those changes.
- The owner explicitly requested updating all README files and uploading to Git.
  This documentation commit is to be published to `origin/main`; the final chat
  records its commit ID and the result of checking the remote ref after pushing.

## Changes

- Updated all eight tracked README files: root, Docker, docs index, algorithm,
  include, library overview, hardware-test overview, and bundled Encoder README.
  Encoder's upstream description/attribution/examples were preserved; added a
  project integration note identifying the active ESP32Encoder wrapper.
- Added Thai quick start with this PC's path, Windows/Ubuntu entry points,
  actual Docker requirements, web/mock, build/USB/OTA, ROS, current demo/route
  features, pose conventions, stopping and links to detailed guides.
- Removed obsolete high-speed examples from the root/Docker README and Windows
  help. Help now uses 0.03 m/s or 7.5 RPM and labels movement explicitly.
- Clarified on both helpers and in guides: Docker `stop` closes containers and
  is not robot E-STOP. Opening the ESP32 web page uses local Python/browser and
  does not require Docker or ROS. Firmware builds/tests and ROS services do.
- Updated install/source-map pointers and the handoff index in AGENTS.md.
  No firmware control logic or hardware settings changed.

## Verification

- Windows helper help, `web --help` and `web-mock --help`: passed without Docker.
- PowerShell parser: no errors. Git Bash `bash -n` and Ubuntu helper help: passed.
- Docker Compose configuration validation: passed without starting services.
- Markdown relative links and code fences checked; UTF-8 Thai text confirmed.
- `git diff --check`: passed. Private network header remains ignored and
  `firmware/config/esp32_hardware.h` is unchanged.
- Docker engine was not running when checked. No firmware build, OTA, robot
  connection/status polling or physical motion test was performed this session.
  Historical firmware/test evidence remains in Claude V2 and Codex V1; it does
  not confirm the current robot's installed build or physical condition.

## Next work

- For normal use: PowerShell in the checkout, `docker\mor_luam.bat web`.
- Start Docker Desktop for `fw-test`, `fw-build`, or ROS commands when needed.
- Read Claude V2 for outstanding firmware/hardware work; its old request to merge
  the feature branch is historical because this checkout is now already on main.
- Physical motion requires the owner beside the robot. Button demos and web
  routes have different watchdog rules. Keep credentials out of chat and logs;
  never print network_secrets.h or GET /api/wifi / GET /api/settings responses.
