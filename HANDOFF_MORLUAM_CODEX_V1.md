# mor_luam handoff — Codex V1

Date: 2026-10-08, Asia/Bangkok. Claude's handoff was not edited.

## Status at a glance

### Finished

- Continued Claude's drive learner. Learned `gain * (KS + KF * rpm)` replaces the PID F term inside output limits, anti-windup and ramp. Gain is bounded 0.6–1.8. Learning waits 1 s, near target speed and away from saturation. Invalid saved gain resets to 1.0. No battery ADC added.
- App saves learned gain while stopped, every 10 s after a change >=0.01; saved KS/KF identify the model, failed saves retry. Temporary nondefault Kf tuning is not saved. Gain/learned seconds appear in status, PID card and trace.
- PID validation rejects invalid numbers/gains/limit pairs. Drive ramp starts at zero after reset and rises by at most 40 PWM per 10 ms. Synthetic weaker-motor next-run first-3-s error improved 1.424 ->1.065 rpm.
- Final steering power ramps +15/-40 PWM per 10 ms; zero cuts immediately. Earlier starts jumped to 627–664 PWM.
- Added BNO085 calibrated gyro and linear acceleration at 50 Hz, retaining the original orientation report. A custom SH2 callback captures every report in batched packets (Adafruit getSensorEvent exposes only the last decoded report). Trace has six axes, yaw, freshness flags and local report counters; freshness expires after 100 ms.
- Added tools/imu_shake.py: fresh unique reports, centered 0.3 s rolling-mean subtraction, RMS/peak, sampling coverage and rate. Synthetic verification covers bias, vibration, duplicates, stale data and counter wrap. Analyzer never connects to the robot.
- IMU showed strongest steering vibration during sustained rotation, including constant PWM. Four attended 90-degree turns compared final caps 664/500/500/664. All met tolerance. Powered-only acceleration RMS fell ~40%, gyro RMS ~24%; time to tolerance rose from 2.06/2.28 s to 3.03/2.93 s. Two trials per cap, not a universal guarantee. Evidence: codex_imu_ab.log/json and steer_ab_*.csv under tools/test_out.
- Applied software steering cap 500 in app_config.h. Hardware file unchanged. PID maximum 290 + existing base 210, plus final cap 500. Advanced steering PID output limits above 290 are rejected. Drive power unchanged. Measured shaking is reduced, not eliminated.
- Added MotorResponseWatch: only watches existing commands; never starts probes. At >=300 PWM, expects 2 drive encoder ticks or 1 degree steering change within 1.5 s. Missing response halts and latches a fault until a new explicit command. Physical E-STOP is shown as not-wired; ground contact unknown.
- Added Simulation tab using HW04 Detour geometry and Direct comparison, animation, model-speed controls and local example. It sends no motor commands. Real route start is separate and requires a manual floor/operator checkbox; HTTP/ROS bypass this UI-only check.
- Inspected reference homework E:\271401\271401_pee_aut_homework\aut_HW_04_detour_steering and GPS docs D:\vm\share\GPS_Localize read-only. Added docs map, safety/testing/web/tuning/IMU guides.
- Hardened robot_test.py: 7.5 rpm, <=0.25 m manual drives, explicit 0.02 m tolerance, stop on failures/timeouts, E-STOP finally, correct-password moving OTA probe, restore route settings, heartbeat-age logs, steady stats exclude drive restarts. --imu captures a stationary baseline and checks freshness/control gaps.
- Fixed Windows OTA helper temporary-file creation. Fixed web OTA polling erasing result messages. Fixed CSV header/row bounds for short HTTP chunks.
- PC suite ALL PASS; browser planner checks homework numbers and 1,000 geometries; IMU analyzer self-test, Python/JS syntax and whitespace checks pass.
- Latest installed build: Oct 8 2026 09:04:33. Binary 1,154,480 bytes, flash 58.4%, static RAM 98,732 bytes (30.1%). Trace ring ~66 KB; observed free heap 79–85 KB.
- FINAL live steps 1/2/3 with --imu: 19/19 PASS (tools/test_out/codex_final_power500.log). All seven steering turns stopped 2.4–3.5 degrees short, no overshoot, peak 57–72 deg/s; 345-degree case took 9.07 s. Both drives reached 0.230 m for 0.25 m command with 0.02 m tolerance, rise 0.29 s, steady 7.7±0.1 and 7.6±0.2 rpm, zero re-aims. Correct-password OTA/pose reset refused moving. E-STOP PWM0 next tick 8 ms; request-to-reported-wheel-stop 1.20 s including Wi-Fi, not a worst-case guarantee.
- Final IMU capture: fresh 100%, ~50 Hz, baseline control max gap 13 ms. Representative steering acceleration RMS 0.246/0.409 m/s²; drive 0.968/1.095, stationary 0.0351. These measure body activity at the sensor, including normal motion. Evidence: codex_final_imu.json and final CSV files.

- Final step 4 PASSED 3/3: Detour route done in 41.3 s, end odometry (0.02,-0.01) m, zero overshoots. No-heartbeat route stopped after 3.6 s. Evidence: tools/test_out/codex_final_route.log and route_no_heartbeat_status.csv. Earlier heartbeat failure is resolved.
- Installed browser checks passed: simulation/example/pause, real-start confirmation gate, settings save/restore and both PID forms. Wrong-password OTA was refused and its error survived status polling. Robot remained halted.

### Still in progress

- Physical shaking is reduced but its mechanical cause remains unconfirmed. No claim of complete removal. Real battery/load/floor variation needs further checks. Final state verified: halted, PWM 0, no motion fault, IMU diagnostics fresh; build 09:04:33, software steering limit 500. All final real checks passed: 22/22.

### Next (in order)

1. Live Wi-Fi fallback/move-up/setup hotspot needs operator network changes and a reachable second saved network. The final route and heartbeat checks are complete. Keep all other robot pages closed when repeating a heartbeat check.
2. Full browser OTA success still needs the owner to enter the real password. Helper/direct multipart OTA passed. Browser automation returned a redacted credential on the first attempt; wrong-password refusal and persistent error display were later verified with a tiny invalid image. Real settings/PID forms passed: temporarily saved speed 0.029, restored 0.03, and applied identical spin/steer values. Installed Simulation example/pause passed without motor commands.
3. Further shaking/battery checks should use the saved IMU tools under the same floor, load and mounting conditions. Never print GET /api/wifi output.
4. Owner should change default OTA/setup-hotspot credentials. Codex changed no credentials.
5. Commit/push only when explicitly requested.

## Repo state

- E:\GPS_Localize\old\mor_luam, main, HEAD 91130c0 unchanged. Remote https://github.com/Mannaja78520/mor_luam.git.
- Large preexisting Claude refactor is uncommitted; many source directories are untracked. Preserve the entire working tree. Tracked-only diff is incomplete.
- No commits/pushes. git diff -- firmware/config/esp32_hardware.h is empty.
- Claude handoff SHA256 unchanged: 0E8840B3F22B50DAE81727AB40431AA215D469F12075AAC8632AAE5FA65E0C5C.
- Secrets were not printed or put in handoffs. Never dump network_secrets.h, /api/wifi or /api/settings responses. Settings contain OTA/AP passwords too.

## Open issues and evidence

- First drive stopped near 0.190 m because general goal tolerance is 0.06 m. Test now explicitly requests 0.02 m. Original codex_step3.log and codex_first_drive_default_tol.csv retained.
- Earlier 07:53 build: step 1 6/6, step 3 7/7 (codex_step1.log, codex_step3_retry.log). Detour completed 31.8 s, odometry (0.01,0.02), no overshoot (codex_step4.log). Odometry does not independently prove floor position.
- Gain persistence verified: saved 1.015524 restored after reboot; before-reboot 1.009856 differed by <0.01. Reboot halted.
- Build 08:41:39: steps 1+2 13/13 and step 3 6/6; logs codex_smooth_steering.log, codex_response_guard_drive.log. Saved traces in codex_0841/.
- First IMU test had a network timeout and stopped safely (codex_imu_steer_drive.log); no reboot or motion fault. Retry seven turns 7/7, fresh 100% (codex_imu_steer_retry.log, codex_imu_run1/).
- At phi354, v 0.03 m/s, w 60 deg/s:0.3m chooses Detour;0.4m has k<2 but extra 0.20 s stop makes Direct faster. Both host regressions. Simulation uses ideal speed and underpredicts real approach/settle time.
- Response guard cannot identify physical E-STOP cause. No physical missing-response fault injection yet. Lifted/slipping wheel can satisfy encoder checks; BNO085 cannot prove static floor contact.
- IMU freshness is ESP32 arrival age; local counters do not reveal sensor-internal dropped reports. At 50 Hz the measured band is below ~25 Hz; faster vibration can alias. Gyro trace resolution 0.1 deg/s; quiet can round to zero, so zero-baseline ratios are omitted.
- Existing imuOk means heading arrived since boot; it does not expire. Use imuMotionFresh/trace flags for current diagnostics. No new general IMU-loss motion interlock.
- Substantial real battery-drop compensation not measured. Learner follows required power, not voltage.
- Historical A/B script tools/test_out/codex_compare_steer.py is locked to build 08:58:54. Do not restore its old 664 output limit on the new 500 build.
- Password location correction: DEFAULT_OTA_PASS is in firmware/config/conf_network.h, not app_config.h in this checkout. Helper prompts privately or accepts process env MORLUAM_OTA_PASS. Never print the value.

## Exact commands

From repository root in PowerShell:

```powershell
Set-Location E:\GPS_Localize\old\mor_luam
docker\mor_luam.bat fw-test
docker\mor_luam.bat fw-build
docker\mor_luam.bat fw-ota -Host 192.168.137.50
node firmware\web\test_simulation.cjs
python tools\imu_shake.py --self-test
python -m py_compile tools\robot_test.py tools\imu_shake.py firmware\web\mock_robot.py
# Motion only while owner is beside the robot:
python -X utf8 -u tools\robot_test.py --host 192.168.137.50 --steps 1,2,3 --imu
python -X utf8 -u tools\robot_test.py --host 192.168.137.50 --steps 4
# Privately set MORLUAM_OTA_PASS for the authenticated moving-OTA probe.
python tools\imu_shake.py --baseline tools\test_out\codex_final_baseline.csv --input tools\test_out\steer_1_90.csv tools\test_out\drive_1_out_7p5rpm.csv tools\test_out\drive_2_back_7p5rpm.csv --output tools\test_out\imu_shake.json
curl.exe -X POST http://192.168.137.50/api/estop
docker\mor_luam.bat web-mock
git diff --check
git diff -- firmware/config/esp32_hardware.h
```

Read docs/README.md for the guide map. Chat outputs/ contains copies of guides, handoff and measured IMU charts.
