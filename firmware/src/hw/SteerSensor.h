#pragma once
// The steering angle sensor (AS5600 on I2C). Gives the wheel angle vs the body.
#include <Adafruit_AS5600.h>
#include "app_config.h"
#include "util/AngleSpikeFilter.h"

class SteerSensor {
public:
    bool begin();
    // Mechanical wheel angle 0..360, in the direction the wheel steers
    // (STEER_SENSE and STEER_ZERO_OFFSET_DEG in esp32_hardware.h).
    float readDeg();
    bool ok() const { return ok_; }
    uint32_t glitches() const { return filter_.rejected(); }   // readings ignored so far

private:
    Adafruit_AS5600 as_;
    bool ok_ = false;
    AngleSpikeFilter filter_{STEER_GLITCH_DEG, STEER_GLITCH_CONFIRM};
};
