#pragma once
// Body heading from the BNO08x.
//
// The yaw is RELATIVE: zero is wherever the robot pointed when the first
// command arrived (or when resetReference() is called from the web page).
// Until then the heading stays 0, exactly as the firmware always did.
#include <Adafruit_BNO08x.h>

class ImuHeading {
public:
    bool begin();
    void update();                 // call every control tick
    void requestReference();       // take the zero at the next reading (if none yet)
    void resetReference(float headingDeg = 0.0f);   // new zero now: the robot's front reads headingDeg
    float yawDeg() const { return yawDeg_; }
    float bodyHeadingDeg() const { return bodyDeg_; }
    bool available() const { return available_; }     // readings AND a zero
    bool receiving() const { return haveLast_; }      // readings have arrived
    bool found() const { return found_; }
    // Diagnostic reports: 50 Hz, in sensor axes; never used to infer floor contact.
    float gyroDps(unsigned axis) const { return gyroDps_[axis]; }
    float accelMps2(unsigned axis) const { return accelMps2_[axis]; }
    uint16_t gyroSeq() const { return gyroSeq_; }
    uint16_t accelSeq() const { return accelSeq_; }
    bool gyroFresh() const;
    bool accelFresh() const;
    bool headingFresh() const;

private:
    bool readReports(float& yawOut);
    void enableReports();
    static void sensorEvent(void* cookie, sh2_SensorEvent_t* event);
    void consumeReport(const sh2_SensorValue_t& report);

    Adafruit_BNO08x bno_{-1};
    sh2_SensorValue_t value_{};
    bool found_ = false;
    bool available_ = false;
    bool haveLast_ = false;
    bool refReady_ = false;
    bool refPending_ = false;
    float refOffsetDeg_ = 0.0f;    // what the front reads after the pending zero
    float lastRaw_ = 0.0f;
    float base_ = 0.0f;
    float yawDeg_ = 0.0f;
    float bodyDeg_ = 0.0f;
    float gyroDps_[3] = {}, accelMps2_[3] = {};
    uint32_t gyroMs_ = 0, accelMs_ = 0, headingMs_ = 0;
    uint16_t gyroSeq_ = 0, accelSeq_ = 0;
    bool haveGyro_ = false, haveAccel_ = false, haveHeading_ = false;
    bool newYaw_ = false;
    uint32_t reportCount_ = 0;
    float reportYaw_ = 0;
};
