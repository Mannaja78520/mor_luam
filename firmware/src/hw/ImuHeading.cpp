#include "hw/ImuHeading.h"
#include <math.h>
#include "util/Angles.h"

#ifdef FAST_MODE
static const sh2_SensorId_t REPORT = SH2_GYRO_INTEGRATED_RV;   // ~1000 Hz, noisier
static const long REPORT_US = 2000;
#else
static const sh2_SensorId_t REPORT = SH2_ARVR_STABILIZED_RV;   // ~250 Hz, more accurate
static const long REPORT_US = 5000;
#endif

static float yawFromQuat(float qr, float qi, float qj, float qk) {
    const float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;
    return atan2f(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
}

bool ImuHeading::begin() {
    found_ = bno_.begin_I2C(0x4A);
    // Adafruit getSensorEvent exposes only the last report in a packet.
    // Capture each SH2 report so batched heading/gyro/acceleration are retained.
    if (found_) sh2_setSensorCallback(sensorEvent, this);
    enableReports();
    Serial.println(found_ ? "[IMU] BNO08x OK" : "[IMU] BNO08x NOT FOUND");
    return found_;
}

void ImuHeading::enableReports() {
    if (!found_) return;
    if (!bno_.enableReport(REPORT, REPORT_US)) Serial.println("[IMU] could not enable the rotation report");
    if (!bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)) Serial.println("[IMU] could not enable gyro diagnostics");
    if (!bno_.enableReport(SH2_LINEAR_ACCELERATION, 20000)) Serial.println("[IMU] could not enable acceleration diagnostics");
}

bool ImuHeading::gyroFresh() const { return haveGyro_ && millis() - gyroMs_ <= 100; }
bool ImuHeading::accelFresh() const { return haveAccel_ && millis() - accelMs_ <= 100; }
bool ImuHeading::headingFresh() const { return haveHeading_ && millis() - headingMs_ <= 100; }

bool ImuHeading::readReports(float& yawOut) {
    if (!found_) return false;
    if (bno_.wasReset()) {
        Serial.println("[IMU] sensor was reset");
        haveGyro_ = haveAccel_ = haveHeading_ = false;
        enableReports();
    }
    newYaw_ = false;
    const uint32_t started = micros();
    // Bound additional reads. One I2C transaction can itself exceed this budget;
    // tickUs and trace timestamps must be checked on the actual board.
    for (unsigned n = 0; n < 4 && micros() - started < 3000; ++n) {
        const uint32_t before = reportCount_;
        sh2_service();
        if (reportCount_ == before) break;
    }
    yawOut = reportYaw_;
    return newYaw_;
}

void ImuHeading::sensorEvent(void* cookie, sh2_SensorEvent_t* event) {
    auto* self = static_cast<ImuHeading*>(cookie);
    if (sh2_decodeSensorEvent(&self->value_, event) == SH2_OK) self->consumeReport(self->value_);
}

void ImuHeading::consumeReport(const sh2_SensorValue_t& report) {
    ++reportCount_;
    switch (report.sensorId) {
        case SH2_ARVR_STABILIZED_RV: {
            const auto& q = report.un.arvrStabilizedRV;
            const float yaw = yawFromQuat(q.real, q.i, q.j, q.k);
            if (isfinite(yaw)) { reportYaw_ = yaw; newYaw_ = true; headingMs_ = millis(); haveHeading_ = true; }
            break;
        }
        case SH2_GYRO_INTEGRATED_RV: {
            const auto& q = report.un.gyroIntegratedRV;
            const float yaw = yawFromQuat(q.real, q.i, q.j, q.k);
            if (isfinite(yaw)) { reportYaw_ = yaw; newYaw_ = true; headingMs_ = millis(); haveHeading_ = true; }
            break;
        }
        case SH2_GYROSCOPE_CALIBRATED: {
            const auto& g = report.un.gyroscope;
            if (!isfinite(g.x) || !isfinite(g.y) || !isfinite(g.z)) break;
            gyroDps_[0] = g.x * RAD_TO_DEG; gyroDps_[1] = g.y * RAD_TO_DEG; gyroDps_[2] = g.z * RAD_TO_DEG;
            gyroMs_ = millis(); ++gyroSeq_; haveGyro_ = true;
            break;
        }
        case SH2_LINEAR_ACCELERATION: {
            const auto& a = report.un.linearAcceleration;
            if (!isfinite(a.x) || !isfinite(a.y) || !isfinite(a.z)) break;
            accelMps2_[0] = a.x; accelMps2_[1] = a.y; accelMps2_[2] = a.z;
            accelMs_ = millis(); ++accelSeq_; haveAccel_ = true;
            break;
        }
        default:
            break;
    }
}

void ImuHeading::requestReference() {
    if (!refReady_) refPending_ = true;
}

void ImuHeading::resetReference() {
    refReady_ = false;
    refPending_ = true;
}

void ImuHeading::update() {
    float yaw;
    const bool rawOk = readReports(yaw);
    if (rawOk) {
        lastRaw_ = angles::wrap360(yaw);
        haveLast_ = true;
    }
    if (refPending_ && haveLast_) {
        base_ = angles::wrap360(lastRaw_);
        refReady_ = true;
        refPending_ = false;
    }
    if (refReady_ && haveLast_) {
        const float raw = rawOk ? angles::wrap360(yaw) : lastRaw_;
        yawDeg_ = angles::wrap360(raw - base_);
        bodyDeg_ = yawDeg_;
    } else if (!refReady_ && !haveLast_) {
        bodyDeg_ = 0.0f;
    }
    available_ = refReady_ && haveLast_;
}
