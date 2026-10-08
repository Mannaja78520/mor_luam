#ifndef PIDF_CONFIG_H
#define PIDF_CONFIG_H

// Drive wheel, measured on the robot 2026-10-08 (battery, open loop, tools/test_out/sweep_pwm*.csv):
//   PWM  300  450  600  800  1000   ->  rpm 1.41 3.34 5.46 7.66 9.62   ~ PWM = 160 + 85 * rpm
// Full power is only ~9.7 rpm (0.039 m/s), so the old gains (Kf 16.5, 4 rpm deadband,
// made for 60+ rpm) left the wheel standing still below ~8 rpm: the I term had to do it all.
// Old values: KP 8.3, KI 0.53, KD 0.001, KF 16.5, tolerance 4.0.
#define Wheel_SPIN_KP  40.0f
#define Wheel_SPIN_KI  30.0f
#define Wheel_SPIN_KD  0.0f
#define Wheel_SPIN_KF  85.0f                // PWM per rpm (slope of the curve above)
#define Wheel_SPIN_KS  160.0f               // PWM where the wheel starts to turn (added when driving)
#define Wheel_SPIN_I_Max 1023
#define Wheel_SPIN_I_Min -Wheel_SPIN_I_Max
#define Wheel_SPIN_ERROR_TOLERANCE  0.3f   // RPM

#define Wheel_STEER_KP  27.8f
#define Wheel_STEER_KI  0.26f
#define Wheel_STEER_KD  7.4f
#define Wheel_STEER_KF  0.0f
#define Wheel_STEER_I_Max  1000
#define Wheel_STEER_I_Min -Wheel_STEER_I_Max
#define Wheel_STEER_ERROR_TOLERANCE  3.5f  // deg
#define Wheel_STEER_BASE_SPEED  210

// Added with the PIDF rework (lib/PIDF/PIDF.h explains each one).
#define Wheel_SPIN_D_FILTER_HZ   12.0f   // derivative low-pass, Hz
#define Wheel_SPIN_RAMP          4000.0f // PWM per second: 0 -> full in 0.26 s (softer start, less current spike)
#define Wheel_STEER_D_FILTER_HZ  8.0f
#define Wheel_STEER_I_ZONE       30.0f   // deg: integrate only this close to the target (no wind-up on long turns)

// Cut the steering motor early by the angle the wheel will still coast
// (src/algorithm/SteerStopPredictor.h). The coast time is learned on the robot
// and kept in NVS; this is only the value a new board starts from.
#define STEER_STOP_PREDICT       1       // 0 = steer like the original firmware
#define STEER_COAST_INIT_S       0.10f   // s


#endif
