#ifndef ESP32_HARDWARE_H
#define ESP32_HARDWARE_H

    //define your robot' specs here
    #define MOTOR_MAX_RPM 500                                               // motor's max RPM          
    #define MAX_RPM_RATIO 0.85                                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
    #define MOTOR_OPERATING_VOLTAGE 12                                      // motor's operating voltage (used to calculate max RPM)
    #define MOTOR_POWER_MAX_VOLTAGE 12                                      // max voltage of the motor's power source (used to calculate max RPM)
    #define MOTOR_POWER_MEASURED_VOLTAGE 12                                 // current voltage reading of the power connected to the motor (used for calibration)
    #define ENCODER1_PULSES_PER_REVOLUTION 990                               // encoder 1 pulse

    #define ENCODER_TICKS 20                                              // encoder ticks
    #define COUNTS_PER_REV1 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel1 encoder's no of ticks per rev
    #define WHEEL_DIAMETER 0.040                                           // wheel's diameter in meters
    // #define LR_WHEELS_DISTANCE 0.335                                        // distance between left and right wheels
    #define PWM_BITS 10                                                     // PWM Resolution of the microcontroller
    #define PWM_FREQUENCY 20000                                             // PWM Frequency
    #define PWM_Max 1023
    #define PWM_Min PWM_Max * -1
    #define GEAR_Ratio 1.575                                               


    // INVERT MOTOR DIRECTIONS
    #define MOTOR_INV false

    // INVERT ENCODER DIRECTIONS
    #define MOTOR_ENCODER_INV true

    //  Motor Brake
    #define MOTOR_BRAKE true

    // Motor 1 Parameters
    #define MOTOR_PWM  -1
    #define MOTOR_IN_A 25
    #define MOTOR_IN_B 26

    // Encoder 1 Parameter
    #define MOTOR_ENCODER_INCRIMENT -1
    #define MOTOR_ENCODER_PIN_A 32
    #define MOTOR_ENCODER_PIN_B 33
    #define MOTOR_ENCODER_RATIO 7


    // I2C communication
    #define SCL_PIN 22
    #define SDA_PIN 21

#endif