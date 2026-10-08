# Safety and attended operation

[Documentation map](README.md) · [Testing](testing.md)

This is a real steer-drive robot. Before any motion, the operator must be beside it and confirm that it is on the floor, supported correctly and clear of people, cables and obstacles. Keep about 2 m of clear space for the short acceptance tests.

## Stop the robot

Use the E-STOP button at the top of the robot page. The direct command from PowerShell is:

```powershell
curl.exe -X POST http://mor-luam.local/api/estop
```

Use the current IP if it has changed. Software E-STOP requires a working network and responsive firmware. Keep the physical motor-power disconnect within reach. If software stop is not confirmed, remove motor power before approaching the wheel.

`tools/robot_test.py` attempts E-STOP on normal exit, Ctrl+C and errors. It also stops before collecting a trace after a drive timeout. Its final message reports whether the request was confirmed. A killed process, lost network or power failure can prevent that request from arriving.

## Implemented protections

| Protection | Trigger | Result | Limit |
|---|---|---|---|
| Startup halt | Firmware starts | Motor power command is zero until a command is accepted | A new command can enable motion |
| Missing motor response | At least 300 PWM is commanded for 1.5 s without sufficient encoder/steering-angle change | Controller halts and reports `drive-no-response` or `steer-no-response` | Does not identify the cause or prove physical switch status; noisy sensors can hide missing motion |
| Browser floor confirmation | Before each real route start from the page | Operator must tick floor/presence confirmation | Manual confirmation only; direct HTTP/ROS callers do not use this browser control |
| Software E-STOP | Web button or `POST /api/estop` | Cancels the web route, clears the drive goal and commands zero PWM | This is not a wired hardware emergency-stop circuit |
| Web route heartbeat | No heartbeat for more than 3 s during a web route | Route stops and the controller halts | Another open page can keep sending heartbeat; this does not cover every raw motor command |
| ROS link loss | Wi-Fi or agent loss while the active command source is ROS | ROS-controlled motion halts | This is not proof of a general command-expiry watchdog |
| Motion refusal | Pose reset or OTA is requested while moving | Request is refused | A wrong OTA password alone does not test the moving guard |
| Test runner limits | Manual drive in the acceptance script | At most 7.5 rpm and 0.25 m per drive | HTTP endpoint limits are larger; use the acceptance script for these checks |

The software halt is not a permanent safety latch. A later accepted movement command can restart the robot. Stop all command sources before inspecting it. A live ROS publisher can send another command after a web stop.

Close every other robot page before a heartbeat test. A second browser or phone can keep the route alive. Merely closing one page does not prove that no heartbeat remains.

## Hardware facts and sensor limits

The current firmware has **no wired hardware E-STOP input**. If an external switch cuts motor power, the firmware does not read the switch position directly. Do not label a software status value as confirmation that the physical switch is pressed or released.

The current robot also has **no ground-contact sensor**. Encoder movement cannot prove floor contact: a lifted drive wheel can spin in the air. A supported wheel, a slipping wheel and a wheel driving across the floor may all produce encoder counts. Floor placement and contact require the operator's direct confirmation.

No encoder response while power is requested has several possible causes: an external E-STOP power cut, a jam, motor wiring failure, motor failure or encoder failure. Software can report missing response, but it cannot identify which of those happened from the encoder alone. Do not bypass a stop by assuming the cause is harmless. Remove power and inspect the robot.

There is no battery-voltage ADC in this setup. The learned drive gain follows the power needed to hold speed. It is not a voltage reading or a battery cutoff.

## Position and simulation limits

Position on the web plane is wheel odometry. It can drift or be wrong when the wheel slips, spins above the floor or has bad encoder feedback. Reaching a plotted point does not independently prove the real floor position. Observe the real movement and measure distance when checking accuracy.

A simulation is useful for checking a route and the controls. It cannot verify physical E-STOP operation, motor wiring, traction, floor contact, sensor freshness or stopping distance. A simulator pass does not authorize a real run. The operator must confirm the physical setup before the real movement starts.

## Before and after a motion test

1. Confirm that the operator is beside the robot and the floor area is clear.
2. Confirm the physical power disconnect is reachable. Start halted; check that the wheel is still.
3. Check IMU and steering status. The acceptance script refuses motion if those readiness flags are false. Readiness flags do not prove every later reading is fresh or correct.
4. Run one short test at a time. Watch the real wheel, body direction and distance.
5. Stop if movement differs from the expected route, the steering shakes, a wheel is lifted or feedback looks wrong.
6. Confirm E-STOP at the end. Leave the robot halted before changing settings, updating firmware or inspecting hardware.
