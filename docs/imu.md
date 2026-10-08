# BNO085: measure body shaking

[Documentation map](README.md) · [Tuning](tuning.md) · [Testing](testing.md)

The BNO085 supplies more than heading. This firmware now records calibrated three-axis rotation rate and linear acceleration for shaking checks. These measurements describe movement at the IMU mounting point.

| Measurement | Unit | Used here |
|---|---|---|
| Fused orientation quaternion | Converted to yaw in degrees | Body heading and odometry; existing report kept |
| Calibrated gyro, x/y/z | Degrees per second | Body rotation and vibration; diagnostic reports requested at 50 Hz |
| Linear acceleration, x/y/z | Metres per second squared | Body vibration and acceleration, with gravity removed; 50 Hz |
| Raw acceleration, gravity, magnetic field, roll/pitch and other reports | Depends on report | Available from the sensor/library, but not enabled by this change |

The gyro units from SH2 are radians per second. Firmware converts them to degrees per second. See the [Adafruit report definitions](https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/sh2_SensorValue.h). The [library's sensor-event method](https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/Adafruit_BNO08x.cpp) exposes the last report decoded in a packet. Firmware therefore uses the SH2 callback to capture every enabled report in a packet.

## Capture a comparison

Only run these motion tests while the operator is beside the robot, on the floor with clear space.

```powershell
python -X utf8 -u tools\robot_test.py --host mor-luam.local --steps 2 --imu
python -X utf8 -u tools\robot_test.py --host mor-luam.local --steps 3 --imu
python tools\imu_shake.py --baseline tools\test_out\imu_baseline.csv --input tools\test_out\steer_1_90.csv tools\test_out\drive_1_out_7p5rpm.csv --output tools\test_out\imu_shake.json
python tools\imu_shake.py --self-test
```

`--imu` first records a stationary baseline. Keep the robot still for four seconds. The script requires fresh gyro and acceleration, zero motor power in the baseline and no control-trace gap above 30 ms before starting motion. It sends E-STOP on exit. The latest test overwrites `imu_baseline.csv`; preserve a copy if comparing separate runs.

The CSV contains `gx_dps`, `gy_dps`, `gz_dps`, `ax_mps2`, `ay_mps2`, `az_mps2` and `imu_yaw_deg`. `imu_flags` bit 1 means gyro fresh and bit 2 means acceleration fresh, within 100 ms. Each report stream has its own sequence number. The 100 Hz trace repeats the latest reading between sensor reports.

Freshness uses arrival time on the ESP32. The sequence numbers count accepted reports locally; they cannot prove that the sensor itself dropped no reports. The older `imuOk` flag only says heading data has arrived since boot. Use `imuMotionFresh` and the trace flags to check the current diagnostic streams.

The analyzer counts only fresh, unique reports. It subtracts a centered 0.3 s rolling mean from each axis and reports the remaining vector RMS and peak, separately for steering and driving. It also reports sampling rate, coverage and control gaps. Missing data receives no vibration score. The Settings/PID card shows current gyro and acceleration magnitude; those instantaneous values are not the filtered vibration score.

## Interpret the result

Compare the same motion, battery, load, floor and IMU mount before and after a tuning change. A larger RMS than the stationary baseline shows body activity in the measured band. Normal starts, stops, turns and impacts can also raise it. There is no automatic safe/unsafe shaking threshold.

At 50 reports per second, this check covers frequencies below about 25 Hz. Faster motor vibration can be missed or aliased. A rigid IMU mount matters. Trace gyro resolution is 0.1 degree/s; a quiet baseline can round to zero, so a ratio to that baseline would be misleading and is omitted.

The IMU cannot prove that a wheel touches the floor. A robot held still in the air can have the same orientation and acceleration as a robot resting on the floor. It also cannot read a physical emergency switch without a wired input. Motor feedback, IMU measurements and operator checks answer different questions.

The current measured results and installed build are in the [Codex handoff](../HANDOFF_MORLUAM_CODEX_V1.md).
