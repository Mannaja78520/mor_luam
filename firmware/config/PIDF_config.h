#ifndef PIDF_CONFIG_H
#define PIDF_CONFIG_H

#define Wheel_SPIN_KP  8.3f
#define Wheel_SPIN_KI  0.53f
#define Wheel_SPIN_KD  0.001f
#define Wheel_SPIN_KF  16.5f
#define Wheel_SPIN_I_Max 1023
#define Wheel_SPIN_I_Min -Wheel_SPIN_I_Max
#define Wheel_SPIN_ERROR_TOLERANCE  4.0f   // RPM

#define Wheel_STEER_KP  27.8f
#define Wheel_STEER_KI  0.26f
#define Wheel_STEER_KD  7.4f
#define Wheel_STEER_KF  0.0f
#define Wheel_STEER_I_Max  1000
#define Wheel_STEER_I_Min -Wheel_STEER_I_Max
#define Wheel_STEER_ERROR_TOLERANCE  3.5f  // deg
#define Wheel_STEER_BASE_SPEED  210


#endif
