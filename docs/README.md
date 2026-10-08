# Documentation map

Start with [Safety](safety.md) before powering the motors. Use [Testing](testing.md) for the checked commands, test order and current evidence.

| Need | Guide |
|---|---|
| Stop the robot, prepare a safe test, understand sensor limits | [Safety](safety.md) |
| Measure body shaking with the BNO085 | [IMU measurements](imu.md) |
| Build, update firmware, run PC tests and attended robot tests | [Testing](testing.md) |
| Compare Direct and Detour Steer in the browser, then run a real route | [Web UI](web_ui.md) |
| Install and run everything: Windows, Ubuntu + Docker, Ubuntu native | [install_and_run.md](install_and_run.md) |
| คู่มือเว็บภาษาไทย: วิธีเปิดเว็บ และทุกแท็บ | [web_guide_th.md](web_guide_th.md) |
| Steering power ramp, trace checks and missing-response diagnosis | [Tuning](tuning.md) |
| Windows setup, Docker, Wi-Fi, browser tabs and ROS tools | [Windows guide](../docker/README.md) |
| Current work, remaining problems and next actions | [Codex handoff](../HANDOFF_MORLUAM_CODEX_V1.md) |
| Previous work and measured motor curve | [Claude handoff](../HANDOFF_MORLUAM_CLAUDE_V1.md) |
| Source files and commands for development | [Source map](../CLAUDE.md), [agent rules](../AGENTS.md) |
| Direct steering, Detour Steer and coast prediction | [Algorithm guide](../firmware/src/algorithm/README.md) |

## What runs where

The ESP32 closes the steering and drive loops locally. The browser edits routes and settings. ROS is an optional command and telemetry link; a web route does not need a ROS agent.

| Part | Job | Source |
|---|---|---|
| Control task | Reads feedback and updates motor power every 10 ms | [ControlLoop.cpp](../firmware/src/control/ControlLoop.cpp) |
| Steering and drive controller | Aims the wheel, then drives; handles distance goals and software halt | [SteerDriveController.cpp](../firmware/src/control/SteerDriveController.cpp) |
| Feedback | AS5600 steering angle, BNO08x heading and drive encoder | [Hardware wrappers](../firmware/src/hw/) |
| Odometry | Estimates position from encoder travel and heading | [Odometry.cpp](../firmware/src/control/Odometry.cpp) |
| Route runner | Chooses Direct or Detour Steer legs and checks the web heartbeat | [WaypointRunner.cpp](../firmware/src/nav/WaypointRunner.cpp) |
| Drive power learning | Adjusts feedforward from steady speed feedback; stores a bounded correction | [DriveGainLearner.h](../firmware/src/algorithm/DriveGainLearner.h) |
| Steering power ramp | Limits powered steering changes; zero power still cuts immediately | [SteerPowerRamp.h](../firmware/src/algorithm/SteerPowerRamp.h) |
| Motor-response guard | Stops an existing command when expected encoder or steering feedback is missing | [MotorResponseWatch.h](../firmware/src/algorithm/MotorResponseWatch.h) |
| Web app | Route editor, status, PID settings, Wi-Fi settings and OTA upload | [Web sources](../firmware/web/), [HTTP API](../firmware/src/web/WebApp.h) |
| Browser simulator | Compares ideal Direct and HW04 Detour Steer routes without motor commands | [45_simulation.js](../firmware/web/js/45_simulation.js) |
| Network and ROS | Wi-Fi selection, mDNS, OTA and micro-ROS | [Network services](../firmware/src/net/), [ROS bridge](../firmware/src/ros/MicroRosBridge.cpp) |

The wheel steers one way only: counter-clockwise seen from above (measured 2026-10-08). Its measured angle increases while steering. The world frame uses metres: +x is forward at pose reset, +y is left, and heading increases counter-clockwise.

## Open the robot page

Connect to the robot's Wi-Fi network, then open [mor-luam.local](http://mor-luam.local/) or the current robot IP. This session used [192.168.137.50](http://192.168.137.50/). Find the current address from the repository root:

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

The robot page now has a **Simulation** tab. It animates the HW04 Detour Steer and the original Direct method side by side. It sends no motor commands. Use the route you edited, or the local 0.3 m / 6-degree example. Then return to Route, confirm floor placement and operator presence, and press **Run real robot** separately. See [Web UI](web_ui.md).

## Development rules

Build web changes from `firmware/web/`. The firmware build generates `firmware/src/web/WebPage.h`; do not edit that generated file directly.

Keep Wi-Fi and OTA passwords out of logs, screenshots and handoffs. Never print `firmware/config/network_secrets.h` or the response from `GET /api/wifi`. The robot's Wi-Fi page displays saved passwords, so use a trusted network.

Do not commit or push without the owner's instruction. Do not change `firmware/config/esp32_hardware.h` without asking the owner. The [handoff](../HANDOFF_MORLUAM_CODEX_V1.md) records the current checkout and installed build.
