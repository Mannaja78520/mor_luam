# Documentation map

## Start on this Windows PC

The working checkout is `E:\GPS_Localize\old\mor_luam` (located 26 October 2026).
From PowerShell:

```powershell
Set-Location E:\GPS_Localize\old\mor_luam
.\docker\mor_luam.bat web
```

`web` and `find` use local Python; the real web app runs on the ESP32. Docker is
needed for firmware builds/host tests and ROS containers, not for opening the robot
page. See the [main README](../README.md) for the quick start and the
[Docker command guide](../docker/README.md) for the full command list.
For offline page work use `.\docker\mor_luam.bat web-mock` (localhost:8000).

## Guides

Start with [Safety](safety.md) before powering the motors. Use [Testing](testing.md) for the checked commands, test order and current evidence.

| Need | Guide |
|---|---|
| Stop the robot, prepare a safe test, understand sensor limits | [Safety](safety.md) |
| Measure body shaking with the BNO085 | [IMU measurements](imu.md) |
| Build, update firmware, run PC tests and attended robot tests | [Testing](testing.md) |
| Compare Direct and Detour Steer in the browser, then run a real route | [Web UI](web_ui.md) |
| Install and run everything: Windows, Ubuntu + Docker, Ubuntu native | [install_and_run.md](install_and_run.md) |
| คู่มือเว็บภาษาไทย: วิธีเปิดเว็บ และทุกแท็บ | [web_guide_th.md](web_guide_th.md) |
| เดินรวม 2 m แบ่ง 7 ช่วง: เปรียบเทียบ Direct / Detour, ตั้งค่า, ระยะกลับ และผลจำลอง | [compare_2m_th.md](compare_2m_th.md) |
| Steering power ramp, trace checks and missing-response diagnosis | [Tuning](tuning.md) |
| Windows setup, Docker, Wi-Fi, browser tabs and ROS tools | [Windows guide](../docker/README.md) |
| Latest state, 27 October 2026: docs published first; new code remains local | [Codex V3](../HANDOFF_MORLUAM_CODEX_V3.md) |
| Previous documentation verification, 26 October 2026 | [Codex V2](../HANDOFF_MORLUAM_CODEX_V2.md) |
| Latest firmware history and attended tests, 8–10 October 2026 | [Claude V2](../HANDOFF_MORLUAM_CLAUDE_V2.md) |
| Earlier IMU, steering and drive evidence | [Codex V1](../HANDOFF_MORLUAM_CODEX_V1.md), [Claude V1](../HANDOFF_MORLUAM_CLAUDE_V1.md) |
| Source files and commands for development | [Source map](../CLAUDE.md), [agent rules](../AGENTS.md) |
| Direct steering, Detour Steer and coast prediction | [Algorithm guide](../firmware/src/algorithm/README.md) |

## What runs where

The ESP32 closes the steering and drive loops locally. The browser edits routes and settings. ROS is an optional command and telemetry link; a web route does not need a ROS agent.

| Part | Job | Source |
|---|---|---|
| Control task | Reads feedback and updates motor power every 10 ms | [ControlLoop.cpp](../firmware/src/control/ControlLoop.cpp) |
| Steering and drive controller | Aims the wheel, then drives; handles distance goals and software halt | [SteerDriveController.cpp](../firmware/src/control/SteerDriveController.cpp) |
| Feedback | AS5600 steering angle, BNO08x heading and drive encoder | [Sensor wrappers](../firmware/src/hw/), [encoder wrapper](../firmware/lib/esp32_Encoder/esp32_Encoder.h) |
| Odometry | Estimates position from encoder travel and heading | [Odometry.cpp](../firmware/src/control/Odometry.cpp) |
| Route runner | Chooses Direct or Detour Steer legs and checks the web heartbeat | [WaypointRunner.cpp](../firmware/src/nav/WaypointRunner.cpp) |
| Drive power learning | Adjusts feedforward from steady speed feedback; stores a bounded correction | [DriveGainLearner.h](../firmware/src/algorithm/DriveGainLearner.h) |
| Steering power ramp | Limits powered steering changes; zero power still cuts immediately | [SteerPowerRamp.h](../firmware/src/algorithm/SteerPowerRamp.h) |
| Motor-response guard | Stops an existing command when expected encoder or steering feedback is missing | [MotorResponseWatch.h](../firmware/src/algorithm/MotorResponseWatch.h) |
| Web app | Route editor, status, PID settings, Wi-Fi settings and OTA upload | [Web sources](../firmware/web/), [HTTP API](../firmware/src/web/WebApp.h) |
| Browser simulator | Compares ideal Direct and HW04 Detour Steer routes without motor commands | [45_simulation.js](../firmware/web/js/45_simulation.js) |
| Button routes, per-point waits and Direct/Detour series | Runs saved button routes 1/2; demos 3/4 and the three-round comparison record paths and activity | [DemoButton.cpp](../firmware/src/app/DemoButton.cpp), [WaypointRunner.cpp](../firmware/src/nav/WaypointRunner.cpp), [47_demo_compare.js](../firmware/web/js/47_demo_compare.js) |
| Network and ROS | Wi-Fi selection, mDNS, OTA and micro-ROS | [Network services](../firmware/src/net/), [ROS bridge](../firmware/src/ros/MicroRosBridge.cpp) |

The wheel steers one way only: counter-clockwise seen from above (measured 2026-10-08). Its measured angle increases while steering. The world frame uses metres: +x is forward at pose reset, +y is left, and heading increases counter-clockwise.

## Open the robot page

Connect to the robot's Wi-Fi network, then open [mor-luam.local](http://mor-luam.local/) or the current robot IP. DHCP addresses change; find the current address from the repository root:

```powershell
docker\mor_luam.bat find
```

The measured drive speed is much lower than some older ROS examples suggest: full power was about 9.7 rpm, or 0.039 m/s. The checked short-drive tests use 7.5 rpm. The checked route uses 0.03 m/s. Use those limits when following [Testing](testing.md).

## Simulation and real runs

The existing browser mock runs on the PC:

```powershell
docker\mor_luam.bat web-mock
```

Open [localhost:8000](http://localhost:8000/). This is a separate mock robot for browser work. It does not establish that a real motor, stop switch or floor contact works.

The robot page has a **Simulation** tab. It animates HW04 Detour Steer and Direct side by side, including optional synthetic error/shaking. It sends no motor commands. Use the edited route or the local 0.3 m / 6-degree example. Real route and comparison buttons are separate and require operator confirmation. Saved button routes 1/2 support per-point waits; demos 3/4 and the three-round series show timings, paths and turn/drive activity. See the [Thai web guide](web_guide_th.md) for the current controls and [Web UI](web_ui.md) for the earlier walkthrough.

## Development rules

Build web changes from `firmware/web/`. The firmware build generates `firmware/src/web/WebPage.h`; do not edit that generated file directly.

Keep Wi-Fi and OTA passwords out of logs, screenshots and handoffs. Never print `firmware/config/network_secrets.h` or responses from `GET /api/wifi` and `GET /api/settings`. The robot's Wi-Fi page displays saved passwords, so use a trusted network.

Do not commit or push without the owner's instruction. Do not change `firmware/config/esp32_hardware.h` without asking the owner. Read [Codex V2](../HANDOFF_MORLUAM_CODEX_V2.md) for documentation verification and [Claude V2](../HANDOFF_MORLUAM_CLAUDE_V2.md) for firmware history. Historical installed-build records do not confirm what is running today.
