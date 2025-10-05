#include <Arduino.h>
#include <config.h>

#include <esp32_Encoder.h>
#include <PIDF.h>
#include <motor.h>
#include <config.h>

float max_val = 1023.0;
float min_val = -max_val;
float Kp = 5.5;
float Ki = 0.3;
float i_min = -max_val;
float i_max = max_val;
float Kd = 0.5;
float Kf = 0.0;
float error_tolerance = 1; // 0.75 degrees

int target_position = 0;


PIDF motor_PIDF(min_val, max_val, Kp, Ki, i_min, i_max , Kd, Kf, error_tolerance);
Controller motor(Controller::Drive2pin, PWM_FREQUENCY, PWM_BITS, MOTOR_INV, MOTOR_BRAKE, MOTOR_PWM, MOTOR_IN_A, MOTOR_IN_B);
esp32_Encoder encoder(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B,
					  ENCODER_PULSES_PER_REVOLUTION, MOTOR_ENCODER_INV, MOTOR_ENCODER_RATIO, WHEEL_DIAMETER);
void setup(){
	Serial.begin(115200);

	// set starting count value after attaching
	encoder.reset();
	
	Serial.println("Encoder Start = " + String((int32_t)encoder.read()));
}

void loop(){
	// Loop and read the count
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        target_position = input.toFloat();
    }
    
    motor.spin(motor_PIDF.compute(target_position, encoder.read()));
	Serial.print("Encoder count = " + String((int32_t)encoder.5()));
	Serial.println(", Motor RPM = " + String((int32_t)encoder.getRPM()));
	// motor.spin(-1023);
}