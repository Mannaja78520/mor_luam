# src/algorithm — steering algorithms

Plain C++ (no Arduino headers), so every file here also compiles on a PC and is
checked by `test_host/tests.cpp` (`docker\mor_luam.bat fw-test`).

| File | What |
|---|---|
| `LegPlanner.h` | the interface: `plan(phiDeg, d, RobotParams) -> LegPlan` |
| `DetourSteer.{h,cpp}` | Detour Steer, HW04 (271401): drive `a` on the current wheel heading, steer `beta*`, drive `b` |
| `DirectPlanner.h` | steer straight at the goal, then drive (the baseline) |
| `PlannerFactory.h` | the list of algorithms; the web page's "Algorithm" choice comes from here |
| `SteerStopPredictor.h` | when to cut the steering motor so the wheel coasts onto its angle (learns the coast time) |
| `SteerApproach.h` | keep pushing through the tolerance band to the landing point (STEER_LAND_DEG) only while the wheel still moves |
| `FinalApproach.h` | stop a drive early and re-aim when the goal would be passed beside it (one motor: no steering while driving) |
| `SteerPowerRamp.h` | smooth final powered steering PWM; zero cuts immediately |
| `MotorResponseWatch.h` | stop when a powered command has no sufficient angle/encoder response |
| `DriveGainLearner.h` | bounded drive feedforward correction learned at steady speed |

## Mapping to the homework

`E:\271401\271401_pee_aut_homework\aut_HW_04_detour_steering\`

| Homework | Here |
|---|---|
| Algorithm 1.2 — `k = (d·ω/v)·|sin φ|`, `β* = 360° − acos(k − 1)` | `DetourSteer::optimalDetourAngleDeg` |
| Algorithm 1.3 — legs `a = d·sin(β*−φ)/sin β*`, `b = d·sin φ/sin β*`, compare with direct | `DetourSteer::plan` |
| Algorithm 1.4 — closed loop: plan again after every leg | `nav/WaypointRunner.cpp` (replans after each move and after an overshoot) |
| Theorems T1, T3, T4 | brute-force checks in `test_host/tests.cpp` |

`tests.cpp` checks the C++ gives the homework's numbers (goal 1 m at 350°:
a 1.034 m, β* 254.2°, b 0.180 m, 9.34 s vs 9.88 s steering straight).

## Add an algorithm

1. New file here, e.g. `MyPlanner.h`, with `class MyPlanner : public LegPlanner`
   (`name()` and `plan()`). Use only `<math.h>`-level C++.
2. Add it to `PlannerFactory.h`: one line in `get()`, one name in `names()`.
3. Optional: a name for the page in `PLANNER_NAMES` (`firmware/web/js/40_nav.js`).
4. Add a check to `test_host/tests.cpp`, run `fw-test`, then `fw-build` + `fw-ota`.

Nothing else changes: `WaypointRunner` asks the factory for the planner chosen
in the settings, and the settings page lists every name the factory knows.

## Conventions

- `phiDeg` = how far the wheel must still turn to face the goal, 0..360. The
  wheel turns one way only, so 350° means "almost a full turn", not "-10°".
- `LegPlan.kind == Detour`: the robot drives `a` metres WITHOUT steering, then
  the runner plans again (T4: the next plan is "steer β*, drive b").
- `RobotParams.steerDps` (ω) comes from the settings page; set it close to the
  real steering speed or the choice between detour and direct is off.
