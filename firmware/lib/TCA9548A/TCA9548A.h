#ifndef TCA9548A_H
#define TCA9548A_H

#include <Arduino.h>
#include <Wire.h>

class TCA9548A {
public:
    TCA9548A(uint8_t address = 0x70) : _address(address) {}

    void begin() {
        Wire.begin(SDA_PIN , SCL_PIN); 
        // Wire.setClock(100000); 
    }

    bool selectChannel(uint8_t channel) {
        if (channel > 7) return false;
        Wire.beginTransmission(_address);
        Wire.write(1 << channel);
        uint8_t result = Wire.endTransmission();
        // Serial.println(channel);
        if (result !=  0) {
            Serial.print("TCA9548A Error: Failed to select channel ");
            Serial.print(channel);
            Serial.print(" (status ");
            Serial.print(result);
            Serial.println(")");
        }
        // delay(10); // allow some time for switching
        // delayMicroseconds(500);  // or even delay(1)


        return result == 0;
    }

    void disableAll() {
        Wire.beginTransmission(_address);
        Wire.write(0x00);  // Disable all channels
        Wire.endTransmission();
    }

private:
    uint8_t _address;
};

#endif
