# mor_luam handoff — Codex V3

Date: 2026-10-27, Asia/Bangkok. Preserve earlier handoffs.

## Latest owner instruction

The owner asked to stop further checking for now and upload ALL documentation
and README updates first. Publish this documentation commit only. Preserve the
uncommitted firmware/web/test/plot changes in this checkout; do not discard them
or claim they are already available in Git. Continue implementation/robot checks
when the owner returns to that work.

## Request and final chosen profile

- The owner first asked for three trials, then a longer route, and finally chose
  a route totalling approximately 2 m split into short legs to make Direct/Detour
  behaviour visibly different. A single 2 m goal at the existing speed/angle
  causes the Detour planner to choose Direct, so it would not demonstrate this.
- Local feature uses `demoDistM=2.0`, `demoSegments=7`, `demoRightDeg=6`, drive
  0.03 m/s and steering model 35 degrees/s. Seven equal outbound waypoint legs
  total 2 m; actual Detour drive can be longer. Both methods use the same points
  in their own aligned start frame. The return to start is untimed.
- Model only: Direct 137.82 s / 2.00 m / 2478 degrees of steering; Detour 110.52 s /
  2.17 m / about 1291 degrees, four detours. Three paired rounds = SIX runs;
  including returns/alignment/holds/gaps the model budget is about 22 minutes.
  These are not physical measurements.

## Work preserved locally, NOT included in the docs-only publication

- Firmware Settings: max comparison length 2 m; `demoSegments` 1..7, NVS `dlegs`,
  at least 0.10 m per leg. Legacy/default profile remains 0.30 m / one leg.
- Runner creates successive short right-bending legs after alignment, records
  the actual waypoint list per run, returns home, and scales button moving-time
  limits with distance/segments. Existing button routes 1/2 are not changed.
- ActivityLog expanded to 64 changes. Compare and series summaries omit path
  and activity arrays; `compare?run=0|1` and `series?run=i` return bounded details
  with flat `xy`, waypoint `route`, and `acts`. Web and Python PNG client updated.
- UI shows current length, segment count, clearance and time budget; changing
  comparison parameters clears readiness. Settings changes affecting a live
  series are refused. A multi-leg recorded plan is drawn through all its points;
  mixed route/settings records are not compared as equal trials.
- Files: firmware/config/app_config.h; src/app/Settings.{h,cpp};
  src/nav/WaypointRunner.{h,cpp}; src/web/WebApp.{h,cpp}, generated WebPage.h;
  firmware/web/index.html, js/47_demo_compare.js, js/60_settings.js,
  mock_robot.py and test_route_test.cjs; test_host/nav_runner_tests.cpp and its
  Settings stub; tools/demo_compare_plot.py. No hardware config change.

## Verification actually completed

- Docker host suite ALL PASS, 0 failures; route runner 40 scenarios PASS.
- Browser planner/noise/UI regression tests PASS; mock Python tests 2/2 PASS;
  Python syntax and multi-leg PNG planner agree with browser model.
- Firmware build PASS: RAM 110636 bytes (33.8%), flash 1204553 bytes (61.3%);
  exported firmware.bin 1211136 bytes. Build installed by HTTP OTA, acknowledged
  ok/reboot; running build confirmed `Oct 27 2026 03:09:06` (firmware build stamp).
- Live settings probes rejected 2.01 m, 8 segments and seven legs within 0.30 m.
  Saved 2 m / 7 / 6 degrees, read back from non-secret `nav.demo`. All three
  existing saved waypoint slots were unchanged. Robot was halted, PWM 0.
- Hotspot `manny` is on. Last confirmed robot IP `192.168.137.102`; it previously
  used .134. Use discovery/mDNS; do not assume a fixed address.
- Mock browser boots and displays controls/results. On the actual robot, API
  requests work but browser page loading stalled in both IAB and Edge. Some
  concurrent asset requests returned BadStatusLine; after closing agent test
  tabs, whoami/status/app.js requests worked and uptime/heap were healthy.
  Root cause is unresolved. Do not claim live browser visual verification passed.
- A later HTML-only change removes stale fixed "30 cm" wording and disables
  initial series controls until status is loaded. A second fw-build was already
  running when the owner requested docs first; it has NOT been uploaded by a
  second OTA in this session. Preserve its output if present.
- No movement command, comparison run, or three-round real test was started.
  The owner asked to view a model instead of confirming readiness. Do not infer
  motion permission from "the robot is here" or from having an API connection.

## Documentation published now

New `docs/compare_2m_th.md`; root/Docker/docs READMEs, Thai web guide, web UI guide,
testing guide, source map and agent handoff pointers updated. Docs explicitly
mark the new code as local/unpublished and distinguish models from real results.
Button explanation: 1/2 run their separately saved routes (last read: 1 m and
0.5 m forward/left/return respectively); 3/4 record Direct/Detour comparisons.
Buttons 1/2 show live position/trail on the main plane, without adding compare
results. 3/4 results remain on the robot until reboot or a newer run replaces them.

## Resume

1. Resolve live browser loading, then rebuild/OTA the final page if requested.
2. Confirm owner beside the robot and all forward/right/return space clear
   before any real motion. The last chosen profile needs about 2.5 m along the
   route plus lateral space; that preparation margin is not a safety guarantee.
3. Run attended checks and record physical evidence before reporting a measured
   speedup. The original request for a real three-round test remains unfulfilled.
4. Publish the preserved implementation separately when appropriate; this docs
   commit intentionally does not publish source changes.
5. Never print network_secrets.h or GET /api/wifi or /api/settings responses.

Intermediate non-secret pre-OTA route/settings snapshot is in this chat's work/
directory. The inline model is separate conversation content, not robot evidence.
