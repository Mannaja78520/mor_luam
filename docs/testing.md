# Build, test and verification record

[Documentation map](README.md) · [Safety](safety.md)

Run the Windows commands below from the repository root with Docker Desktop running. Read [Safety](safety.md) before testing the real robot.

```powershell
Set-Location E:\GPS_Localize\old\mor_luam
```

## PC tests and firmware build

These commands do not move the robot:

```powershell
docker\mor_luam.bat fw-test
docker\mor_luam.bat fw-build
```

`fw-test` compiles and runs the plain C++ host suite. It checks Detour Steer against the homework, theorem cases, PID behavior, steering-angle math, angle-spike filtering, Wi-Fi priority, learned drive feedforward, steering power ramp and missing-response guard.

The drive-learning test uses a synthetic weaker motor that needs 1.18 times the nominal PWM. It checks learning gates, bounded gain, invalid saved values, improvement on a later drive, external feedforward, anti-windup and startup ramp. This is a model test; it does not measure a real battery drop.

`fw-build` also regenerates the packed web page from `firmware/web/`. Firmware output is written to `firmware/firmware_out/`.

For Python changes, check syntax as well:

```powershell
python -m py_compile tools\robot_test.py firmware\web\mock_robot.py
node firmware\web\test_simulation.cjs
git diff --check
```

The Node test checks the browser planner against the homework numbers, the 0.3 m Detour and 0.4 m Direct cases at the measured robot speed, and 1,000 generated route geometries. It does not command the robot.

## Firmware update

Keep the robot stopped and the operator nearby during an update:

```powershell
docker\mor_luam.bat fw-ota -Host mor-luam.local
```

The helper prompts for the OTA password. It can also use a privately set `MORLUAM_OTA_PASS` process environment variable. Do not paste its value into chat, shell history, logs or documentation. The default OTA password is `MORLUAM_DEFAULT_OTA_PASS` in the gitignored `firmware/config/network_secrets.h` (`conf_network.h` only has a placeholder).

For a first USB flash, use the correct port; this machine previously used COM9:

```powershell
docker\mor_luam.bat fw-flash -Port COM9
```

Confirm the running build from status after updating. A successful PC build does not prove the new firmware is installed.

## Attended acceptance tests

Run step 1 first. Run steps 2, 3 or 4 only after the owner confirms being beside the robot, with about 2 m clear. These commands refer to the actual robot, not a simulator.

| Step | Purpose | Motion |
|---|---|---|
| 1 | Settings, PID validation, waypoints and pose reset | None |
| 2 | Clockwise steering, tolerance and coast prediction | Wheel steers in place; drive RPM is zero |
| 3 | Out-and-back drive, moving refusals and E-STOP | 7.5 rpm; each manual drive is at most 0.25 m |
| 4 | Detour route, return and lost-heartbeat stop | 0.03 m/s; first waypoint is about 0.3 m away |

```powershell
python -X utf8 -u tools\robot_test.py --host mor-luam.local --steps 1
# Motion: only with the owner beside the robot.
python -X utf8 -u tools\robot_test.py --host mor-luam.local --steps 2
python -X utf8 -u tools\robot_test.py --host mor-luam.local --steps 3
python -X utf8 -u tools\robot_test.py --host mor-luam.local --steps 4
```

Step 3 uses an explicit goal tolerance of 0.02 m. The firmware's general default is 0.06 m, which previously stopped a 0.25 m command near 0.19 m. Do not mistake that tolerance difference for a motor-speed problem.

To check the authenticated OTA motion guard in step 3, provide the correct password privately through `MORLUAM_OTA_PASS`. The script sends invalid image bytes while the robot moves and expects the moving refusal. With no password in the environment, this probe prints SKIP. A wrong-password refusal proves only authentication.

Before step 4, close all other robot pages on computers and phones. The route runner accepts heartbeat from any page; another page can prevent the no-heartbeat test from stopping. Step 4 restores its original route settings in `finally`, and the runner attempts E-STOP on exit.

The step 4 waypoint is 6 degrees counter-clockwise from the current wheel heading. For the clockwise-only wheel, that means a direct turn of 354 degrees. At 0.3 m and 0.03 m/s, Detour Steer wins. At 0.4 m with the same angle and a 0.20 s extra stop, Direct is slightly faster even though `k < 2`. Both cases have host regressions.

## Read the results

Raw traces and route logs are saved under `tools/test_out/`. Drive trace columns include time, steering angle and target, steering rate, PWM, measured and target RPM, odometry, state flags and learned drive gain. Up to 15 s of control history is available from `GET /api/trace?s=15`.

Trace flags use bit 1 for steering, bit 2 for driving, bit 4 for coasting and bit 8 for steering overshoot. When judging steady drive speed, discard the first second after every return to DRIVE. A re-aim resets the drive PID and ramp; that restart is not steady running.

## Verified baseline: 8 October 2026

The following historical record applies to the drive-learning update, running build `Oct 8 2026 07:53:28`. The next section records the later steering and browser checks. The [handoff](../HANDOFF_MORLUAM_CODEX_V1.md) is the current state record.

| Check | Result | Evidence or limit |
|---|---|---|
| Host suite | ALL PASS, 0 failures | `docker\mor_luam.bat fw-test` |
| Firmware build and OTA helper | Passed | Build 1,147,312 bytes; update confirmed on robot |
| Live step 1 | 6/6 passed | `tools/test_out/codex_step1.log` |
| Live step 3 | 7/7 passed | `tools/test_out/codex_step3_retry.log` |
| Drive rise | 90% target in 0.28 s | Live short-drive traces |
| E-STOP | PWM zero on next tick, 9 ms | Live step 3 trace; this is not a guaranteed worst-case stopping time |
| Moving pose reset and authenticated OTA | Refused | Live step 3 |
| Detour route | Done in 31.8 s; zero overshoots | End odometry (0.01, 0.02) m; physical floor position still needs operator confirmation |
| Saved gain | Restored after reboot | Saved 1.015524; pre-reboot 1.009856 differed by less than the 0.01 save threshold |

In the synthetic weaker-motor test, gain learned 1.217 for a plant gain of 1.18. Final speed was 7.80 rpm; the 0.3 rpm deadband permits this bias. A later fresh-PID run had mean first-3-s speed error 1.065 rpm versus 1.424 rpm without learning.

The earlier live back-drive trace had two steering re-aims near the 7-degree threshold, at 8.458 s and 11.778 s. Including later startup ramps gave 7.311 ± 1.637 rpm. Excluding the first second of each drive interval gave 7.760 ± 0.095 rpm over 572 samples. Later tests reuse `drive_2_back_7p5rpm.csv`; use the saved log `codex_step3_retry.log` for this historical result.

## Steering smoothing and response guard: 8 October 2026

These robot checks used build `Oct 8 2026 08:41:39`. The steering ramp limits the final powered command to +15/-40 PWM per 10 ms tick. Zero-power stops remain immediate. The response guard watches an existing command of at least 300 PWM and stops after 1.5 s without 2 drive encoder ticks or 1 degree of steering change.

| Check | Result | Evidence or limit |
|---|---|---|
| Host suite | ALL PASS | Includes response timeout, valid feedback, fault reset, steering power ramp and immediate zero power |
| Firmware build and OTA helper | Passed | Build 1,152,400 bytes; running build confirmed on robot |
| Live steps 1 and 2 | 13/13 passed | `tools/test_out/codex_smooth_steering.log` |
| Steering accuracy | All seven turns stopped 3.3–3.5 degrees before the target; no overshoot | Same log; peak rates 86–118 degrees/s |
| Steering power changes | First powered command 15 PWM; maximum powered rise 15 PWM per tick | Latest steering traces; previous starts were 627–664 PWM |
| Live step 3 | 6/6 passed | `tools/test_out/codex_response_guard_drive.log` |
| Short drives | Both reached 0.230 m with a 0.02 m goal tolerance; 90% speed in 0.29 s | Same log; steady 7.7 ± 0.1 rpm, no steering re-aims in either leg |
| Moving pose reset and authenticated OTA | Refused | Same log |
| E-STOP | PWM zero on next tick, 8 ms; no further encoder travel recorded | Same log; request-to-reported-wheel-stop was 0.85 s including Wi-Fi, not a worst-case guarantee |
| Browser simulation | Planner tests and mock browser checks passed | Local example: Detour 15.74 s versus Direct 15.95 s; simulator sent no motor commands |
| Real-run checkbox | Disabled start until manual floor/operator confirmation; confirmation clears after start | Mock browser check; API and ROS commands bypass this UI check |

This was the earlier ramp-only build. Later BNO085 measurements showed that sustained steering still vibrated. The software power limit below addresses that measured behaviour.

The guard does not identify an emergency switch state. Status reports `hardwareEstop: not-wired` and `groundContact: unknown`. A lifted wheel can satisfy the encoder check. Physical missing-response fault injection has not been tested on the robot.

## Pending work

- Further physical shaking reduction: the IMU-guided limit helped, but shaking is not eliminated and the mechanical cause is unconfirmed.
- Wi-Fi fallback, return to a higher-priority network and setup hotspot: live checks still need the operator to change network availability.
- Full successful OTA upload through the browser form: needs the owner to enter its real password. Helper/direct HTTP updates passed. Browser wrong-password refusal and persistent error message passed.
- Drive learning over a substantial real battery change: not measured yet.

Never raise the test speed or distance limits without the owner's approval. Do not print Wi-Fi password responses while collecting evidence.

## Final power limit and IMU verification

Installed build `Oct 8 2026 09:04:33` uses a 500-PWM software steering cap, with the hardware file unchanged. The PID contribution is limited to 290 before adding the existing 210 base. See [Tuning](tuning.md) for the four-turn comparison and [IMU measurements](imu.md) for the method.

| Check | Result | Evidence |
|---|---|---|
| Final PC suite and build | ALL PASS; binary 1,154,480 bytes | Firmware build and test commands |
| Final real steps 1/2/3 | 19/19 passed | `tools/test_out/codex_final_power500.log` |
| Seven steering turns | Within 2.4–3.5 degrees, no overshoot | Peak 57–72 degrees/s; 345-degree case took 9.07 s |
| Short drives | 90% speed in 0.29 s; steady 7.7 ± 0.1 and 7.6 ± 0.2 rpm | No re-aims; 0.230 m travel for 0.25 m request with 0.02 m tolerance |
| Moving refusals and E-STOP | Correct-password OTA/pose reset refused; PWM zero next tick, 8 ms | Stop request to reported stopped wheel 1.20 s including Wi-Fi; no worst-case guarantee |
| Final Detour route | Done in 41.3 s, end odometry (0.02,-0.01) m, no overshoot | `tools/test_out/codex_final_route.log` |
| Lost heartbeat | Stopped after 3.6 s | Same log and `route_no_heartbeat_status.csv`; earlier failed attempt resolved |
| IMU capture | Fresh 100%, about 50 Hz, stationary control max gap 13 ms | `codex_final_imu.json`, `codex_final_baseline.csv` |
| Live browser simulation | Local example, pause and Direct/Detour comparison passed | Installed robot page; no motor commands |
| Live settings and PID forms | Temporary speed 0.029 saved, restored to 0.03; same spin/steer values applied | Success messages and API readback; robot stayed halted |
| Browser OTA failure handling | Wrong password refused; message persists during polling | Tiny invalid image used; firmware build unchanged |

The four-turn A/B comparison used two 90-degree trials per power limit. Powered-only acceleration vibration fell about 40% and rotation-rate vibration about 24%; turns took about 40% longer. This is measured improvement under those test conditions, not proof of complete shake removal. Final drive IMU acceleration RMS was 0.968/1.095 m/s² compared with 0.035 at rest; this includes body activity during motion.

The first IMU attempt had a network timeout and sent E-STOP. A retry passed. Raw traces are overwritten by later tests; historical sets are preserved in `codex_0841/` and `codex_imu_run1/`.
