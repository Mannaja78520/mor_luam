#include <Arduino.h>
#include <esp32_Encoder.h>
#include <config.h>

esp32_Encoder encoder(MOTOR_ENCODER_PIN_A, MOTOR_ENCODER_PIN_B,
                      ENCODER_PULSES_PER_REVOLUTION, MOTOR_ENCODER_INV, MOTOR_ENCODER_RATIO);

void setup() {
  Serial.begin(115200);
  delay(50);
  
  encoder.reset(); 
}

void loop() {
  // อ่านค่าตำแหน่ง encoder
  int64_t ticks = encoder.read();
  Serial.print("Encoder Ticks: ");
  Serial.println(ticks);

  // อ่านค่า RPM
  // float rpm = encoder.getRPM();
  // Serial.print("RPM: ");
  // Serial.println(rpm);

  // รอ 1 วินาที
  // delay(1000);
}