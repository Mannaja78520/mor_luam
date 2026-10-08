#include "hw/SteerSensor.h"
#include <config.h>
#include "util/Angles.h"
#include "app_config.h"

bool SteerSensor::begin() {
    ok_ = as_.begin();
    if (!ok_) {
        Serial.println("[AS5600] NOT FOUND!");
        return false;
    }
    Serial.println("[AS5600] OK");
    as_.setSlowFilter(AS5600_SLOW_FILTER_16X);
    as_.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
    as_.setPowerMode(AS5600_POWER_MODE_NOM);
    return true;
}

// Raw angle through AngleSpikeFilter: the AS5600 gives bad single readings
// while the motor runs (see util/AngleSpikeFilter.h).
float SteerSensor::readDeg() {
    const uint16_t raw = as_.getAngle();
    const float deg = (raw * 360.0f) / 4096.0f;                 // 0..360 from the chip
    return filter_.update(angles::wrap360(STEER_SENSE * deg + STEER_ZERO_OFFSET_DEG));
}
