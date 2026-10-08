# Steering smoothing and diagnosis

[Documentation map](README.md) · [Safety](safety.md) · [Testing](testing.md)

The robot uses one motor for steering and driving. AS5600 supplies steering angle; the drive encoder supplies wheel travel. Software filtering does not repair loose hardware, power problems or electrical noise.

## Steering changes

Earlier filtered traces showed starts from zero to 627–664 PWM, changes of 75–209 PWM per 10 ms, and repeated near-target starts above 300 PWM. Those sudden power changes can contribute to shaking. The steering-angle filter had already removed the large angle spikes.

`SteerPowerRamp` limits the final powered steering command, including base power, to a rise of 1500 PWM/s and a fall of 4000 PWM/s. At 100 Hz that is +15/-40 PWM per tick. A zero-power request cuts immediately. E-STOP, coasting and goal stops are not delayed by the ramp.

PID gains, hardware pins and physical wiring were not changed for this smoothing step. The existing coast predictor remains active. The hardware configuration file must not be edited without the owner's approval.

## IMU-guided power limit

Seven turns with BNO085 measurements showed the strongest vibration during continuous rotation. Even constant 664 PWM showed shaking. A four-turn 90-degree comparison then used final limits 664, 500, 500 and 664 PWM. All turns met the 3.5-degree tolerance.

Counting only powered, fresh, unique IMU samples, the lower limit reduced mean acceleration vibration by about 40% and rotation-rate vibration by about 24%. Time to tolerance rose from 2.06/2.28 s to 3.03/2.93 s. These are two trials per limit, not a guarantee for every floor, load or battery.

`app_config.h` now sets a software steering limit of 500 PWM. The physical hardware maximum stays unchanged. The steering PID maximum is 290, plus the existing 210 base power. This places the limit inside anti-windup; the final command is also clamped to 500. Advanced PID requests cannot exceed that 290 PID maximum. Drive power is unchanged.

The lower speed reduces measured body vibration; it does not eliminate it. The route and browser simulator use an ideal steering speed. Their predicted time still differs from the real turn, especially during the slow approach to its target. See [IMU measurements](imu.md) for the method and raw trace commands.

## Read a trace

Use the attended step 2 test, then inspect `tools/test_out/steer_*.csv`. Compare powered-to-powered PWM changes separately from immediate stops. Check clockwise rotation, final angular error, coast samples and repeated starts near the target.

Changing battery/load can change coast distance. Do not increase derivative gain to hide noise. Confirm the sensor and mechanical behaviour first. If shaking remains, record when it happens: initial acceleration, continuous rotation, or the last few degrees. Physical observation is needed to confirm that the improvement is visible.

## Missing-response guard

`MotorResponseWatch` observes existing operator commands. It never starts a motor test by itself. With at least 300 PWM commanded, it expects at least 2 encoder ticks during drive, or 1 degree of steering-angle change, within 1.5 s. Missing feedback stops the controller. A later explicit movement command clears the fault.

A power-cut emergency switch, jam, wiring fault, failed motor or failed feedback can all look similar. Small sensor noise can also hide missing motion. The guard is a useful diagnostic and stop, not a safety-rated emergency circuit. A freely spinning lifted wheel passes the encoder check; floor contact remains unknown.
