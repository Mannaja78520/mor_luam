# mor_luam handoff - Codex V4

Date: 2026-10-27, Asia/Bangkok. Read V3 for the docs-first checkpoint and earlier OTA evidence.

## Current owner request and delivered behavior

The owner resumed implementation after the docs-first commit, requested an editable Simulation tab, corresponding homework algorithms/docs, and a PC-hosted page usable without a robot. They asked whether mor-luam.local can be used. Keep that name for the ESP32; use localhost:8000 for the independent PC UI. No hosts-file or mDNS override was made.

- `docker\mor_luam.bat web` / `./docker/mor_luam.sh web` starts tools/pc_web.py. It serves local assets, searches for a real robot, forwards API requests serially and reconnects automatically. No fake telemetry, background heartbeat, automatic movement or replay of offline commands. Keep its terminal open. `web-robot` retains direct ESP32 discovery/opening. `web-mock` remains a separate development fake on the same default port.
- SimulationWaypoints (32_simulation_route.js) is isolated from real RouteModel. Add/drag/delete, x/y entry, clear, fit and explicit copy are available offline. The simulator starts at (0,0), heading +x. The 0.3 m example now bends right (negative y in the CCW frame). The 2 m / 7-leg example agrees with Python.
- Firmware comparison max 2 m, 1..7 equal legs, minimum 0.10 m each. Existing default remains 0.30 m / one leg. Current owner profile was saved earlier as 2 m / 7 / 6 degrees with drive 0.03 m/s and steering model 35 deg/s. The two planners share the same short right-bending waypoint chain. Return is additional and untimed. Button routes 1/2 are preserved.
- Comparison/series summaries omit large paths. `?run=i` returns one run's flat xy, recorded route and activities. UI/Python plot download details sequentially. ActivityLog now holds 64 changes. Moving-time limits scale with route length/legs; web heartbeat rule is unchanged.
- Docs/README/command help updated. User also requested E:\271401\271401_pee_aut_homework: Detour Python/GUI/formal HTML/TeX/PDF use CCW, measured GUI defaults and a 2 m / 7-leg model. Historical real plots retained. Unrelated pre-existing homework changes remain unstaged. Root homework README publishes only the Detour section; other local additions are preserved.

## Verification

- Docker fw-test: ALL PASS, 0 failures; WaypointRunner 40 scenarios PASS.
- Final fw-build PASS: RAM 110636 bytes (33.8%); flash 1206153 bytes (61.3%); firmware.bin 1212736 bytes. Generated WebPage.h rebuilt from source. This final build is NOT installed by OTA.
- PC server HTTP contracts: 3/3 PASS (offline assets, reconnect without replay, cross-origin rejection). Browser waypoint isolation/edit/example test, route-test UI checks and 1,000 planner geometries PASS.
- Homework tests: 3/3 PASS, 300 random cases in each GUI self-test PASS; Python model matches browser Direct 137.82 s / 2.00 m, Detour 110.52 s / 2.174 m, four detours. These are model results.
- PDF compiled successfully, six pages; no Overfull/Undefined errors. Pages rendered and inspected. Current-profile section begins on its own final page.
- Actual IAB PC page opened without robot, clicked to add a waypoint, loaded seven-point example and ran it. PC service currently running on port 8000; user-facing tab retained. Screenshot in this chat's outputs/pc-simulation.png.

## Hardware status / unfinished physical work

- No movement command or real three-round trial was started. The owner asked about the model instead of confirming floor/clearance readiness. Do not infer motion authorization from power cycling or Wi-Fi association.
- Earlier OTA was verified as build Oct 27 2026 03:09:06 (see V3); it contains multi-leg firmware but predates the latest local simulation editor. PC page serves current local assets independently of the installed robot UI.
- Windows hotspot manny is on and reports hostname mor-luam, last IP 192.168.137.102. HTTP/ping later stopped responding. Owner confirmed power cycling; still no API response. A pending clarification asks whether this powered off ESP32 or only the motor. No USB serial board was found. Root cause remains unresolved; do not report that the real robot connection is fixed.
- PC app reports robotHost:null while offline and remains usable for simulation. Opening the PC page or a reconnect never starts motors. To resume a real test, first recover live whoami/status, check halted state, then obtain owner's attended readiness per AGENTS.md. Do not print settings/Wi-Fi credentials.
- No hardware pin/PWM/wheel config changes. No recursive deletion of user work. Temporary LaTeX first-attempt files and original homework backups remain in this chat's work directory.

Publication: implementation/docs are being committed to main under owner authorization to upload all requested work. Inspect git log for final commit IDs; V3's statement that code is unpublished applies only to that earlier checkpoint. The two-metre physical trial remains outstanding.
