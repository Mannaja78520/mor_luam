#pragma once
// The wheel controller: steer to the commanded world heading (one way only: counter-clockwise from above),
// then drive at the commanded rpm, optionally for a distance.
//
//   STEER_TO_HEADING --(inside tolerance for STEER_SETTLE_MS)--> DRIVE
//   DRIVE --(wheel drifts > 2 x tolerance, goal reached, or rpm 0)--> STEER_TO_HEADING
//
// This is the logic of the old main.cpp controlCallback()/cmd_move_cb(),
// moved into a class unchanged. NOT thread-safe: ControlLoop owns the lock.
#include <motor.h>
#include <PIDF.h>
#include <esp32_Encoder.h>
#include "algorithm/SteerStopPredictor.h"
#include "algorithm/DriveGainLearner.h"
#include "algorithm/MotorResponseWatch.h"
#include "algorithm/SteerPowerRamp.h"
#include "control/DriveCommand.h"
#include "control/Odometry.h"
#include "control/RobotState.h"
#include "hw/ImuHeading.h"
#include "hw/SteerSensor.h"

class SteerDriveController {
public:
    SteerDriveController(Controller& motor, esp32_Encoder& encoder, SteerSensor& steer, ImuHeading& imu);

    void begin();
    void step();                                        // one 10 ms tick
    void apply(const DriveCommand& cmd, CommandSource src);
    void halt();                                        // motor 0 and hold until the next command
    // [Kp, Ki, Kd, Kf, tol, iMin, iMax, outMin, outMax], trailing values optional
    bool setPid(bool steerLoop, const float* v, size_t n);
    void getPid(bool steerLoop, float out[5]) const;
    void resetPose();                                   // position (0,0) and a new IMU zero
    void setCoastS(float s) { predictor_ = SteerStopPredictor(s); }   // learned value from NVS
    void setDriveGain(float gain) { gainLearner_.setGain(gain); }    // before the control task starts

    const RobotState& state() const { return state_; }
    CommandSource source() const { return source_; }
    bool moving() const { return !halted_ && (mode_ == Mode::Drive || pwm_ != 0); }

private:
    enum class Mode { Steer, Drive };
    void stopGoal();
    void fillState();

    Controller& motor_;
    esp32_Encoder& encoder_;
    SteerSensor& steerSensor_;
    ImuHeading& imu_;
    PIDF spin_;
    PIDF steer_;
    Odometry odom_;

    Mode mode_ = Mode::Steer;
    CommandSource source_ = CommandSource::None;
    bool halted_ = true;

    // command
    float targetRpm_ = 0.0f;
    float targetHeadingDeg_ = 0.0f;
    // feedback of this tick
    long ticks_ = 0;
    float rpm_ = 0.0f;
    float steerDeg_ = 0.0f;
    float headingMeasDeg_ = 0.0f;
    float steerTargetDeg_ = 0.0f;
    float steerErrDeg_ = 0.0f;
    int pwm_ = 0;
    uint32_t steerOkSinceMs_ = 0;
    float spinTol_, steerTol_;
    float pidSpin_[5], pidSteer_[5];
    DriveGainLearner gainLearner_;
    uint32_t driveSinceMs_ = 0;
    MotorResponseWatch responseWatch_;
    SteerPowerRamp steerPower_;

    // goal along the commanded heading
    float targetDistM_ = 0.0f;
    float targetTolM_;
    float progressM_ = 0.0f;
    float lastDeltaM_ = 0.0f;
    bool hasTarget_ = false;
    bool goalActive_ = false;
    float startX_ = 0.0f, startY_ = 0.0f;
    float goalX_ = 0.0f, goalY_ = 0.0f;
    float goalVecX_ = 0.0f, goalVecY_ = 0.0f, goalLenSq_ = 0.0f;

    // early cut: let the wheel coast into its angle (SteerStopPredictor)
    SteerStopPredictor predictor_;
    bool coasting_ = false;
    float cutAngleDeg_ = 0.0f;
    float rateDps_ = 0.0f;          // steering speed, + = in the steering direction
    float prevSteerDeg_ = 0.0f;
    bool havePrevSteer_ = false;
    bool steerAimed_ = false;       // inside the steering tolerance (with hysteresis)
    static constexpr uint8_t RATE_WINDOW = 5;   // ticks (50 ms) for the steering speed
    float steerHist_[RATE_WINDOW] = {};
    uint8_t rateIdx_ = 0, rateFill_ = 0;
    void learnIfStopped();

    // overshoot watch (DriveCommand::stopOnOvershoot)
    bool stopOnOvershoot_ = false;
    bool overshot_ = false;
    float overshootDeg_ = 0.0f;
    float prevECw_ = 0.0f;
    bool havePrevE_ = false;

    RobotState state_;
};
