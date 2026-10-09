# Web page: simulate, then run

[Documentation map](README.md) · [Safety](safety.md) · [Testing](testing.md)

Open `http://mor-luam.local/` or `http://<IP>/` (IP from `docker\mor_luam.bat find`; the hotspot can give a new one) while connected to the robot's network. The page is served by the ESP32; no ROS agent is needed for a web route.

## Place a route

1. Keep the robot stopped. Reset the pose if this is a new starting point.
2. Open Route. Place points on the plane, or edit the x/y table. Values are metres.
   The "รอ (s)" column is a stop at that point once reached (0-60 s, 0 = drive on); the page counts it down.
3. The frame is +x forward at pose reset and +y left. Wheel heading increases counter-clockwise, and the real wheel also steers counter-clockwise only (its heading only increases).
4. Edited points stay in the browser until saved or a real route is started.

## Try the simulator

1. Open Simulation. Set drive speed and steering speed. Defaults are 0.03 m/s and 60 degrees/s. These are model settings only; they do not change the real robot's settings.
2. Press Simulate placed points. The simulator copies the current edited route and the latest robot position/wheel heading at that moment.
3. Green shows Detour Steer. Orange shows Direct, the original steer-at-goal method. The display compares time, path length and motion phases.
4. Pause/resume, reset or choose playback at 1x, 4x or 10x. The 0.3 m / 6-degree example changes only the local simulation. It does not save or start a real route.
5. If you edit the route, the old simulation stops. Simulate again for the new points.

The planner matches the homework at `E:\271401\271401_pee_aut_homework\aut_HW_04_detour_steering`. It uses the same closed-form detour and compares its full time, including the extra stop, with Direct. Detour wins only in some cases. At 0.03 m/s and 60 degrees/s, the 0.3 m / 6-degree example is about 15.74 s versus 15.95 s; a 0.4 m case chooses Direct instead.

The model assumes constant speeds and accurate feedback. It does not model shaking, wheel slip, battery changes or physical stop distance. It cannot check emergency-switch state or ground contact. The displayed predicted time can differ from real motion.

## Start the real robot

1. Return to Route and inspect the real route and settings. Simulator speed settings do not change the real route speed.
2. Confirm the robot is on the floor, you are beside it and the area is clear. Tick the floor/operator checkbox. This is a manual confirmation, not a sensor reading.
3. Press Run real robot. Unsaved route points are saved before starting. The confirmation clears after each start attempt.
4. Watch the real movement. Use the red E-STOP button to stop. Keep the motor-power disconnect in reach.
5. Keep the page awake while a route runs. Every open robot page sends a heartbeat. If all pages stop sending, the route stops after about 3 s.

The Settings/PID card shows learned drive gain and motor-response faults. Hardware E-STOP is shown as not wired. Ground contact is unknown. A no-response fault means feedback was missing; it does not diagnose which physical component failed.

## Offline browser work

```powershell
docker\mor_luam.bat web-mock
node firmware\web\test_simulation.cjs
```

The mock runs at `http://localhost:8000/`. It is separate from the real robot. The Node test checks the homework numbers, measured-speed cases and 1,000 geometries without a robot or browser.
