#ifndef PIDF_CONFIG_H
#define PIDF_CONFIG_H

#define Wheel_SPIN_KP  2.5f
#define Wheel_SPIN_KI  0.5f
#define Wheel_SPIN_KD  0.0f
#define Wheel_SPIN_KF  4.0f
#define Wheel_SPIN_I_Max 400
#define Wheel_SPIN_I_Min -Wheel_SPIN_I_Max
#define Wheel_SPIN_ERROR_TOLERANCE  2.0f   // RPM

#define Wheel_STEER_KP  1.0f
#define Wheel_STEER_KI  0.1f
#define Wheel_STEER_KD  0.0f
#define Wheel_STEER_KF  0.0f
#define Wheel_STEER_I_Max  1500
#define Wheel_STEER_I_Min -Wheel_STEER_I_Max
#define Wheel_STEER_ERROR_TOLERANCE  5.5f  // deg
#define Wheel_STEER_BASE_SPEED  550


#endif