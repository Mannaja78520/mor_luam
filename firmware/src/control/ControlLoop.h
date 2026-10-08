#pragma once
// Runs SteerDriveController every CTRL_PERIOD_MS in its own FreeRTOS task, so
// the wheel keeps being controlled whatever the network is doing (a micro-ROS
// ping can block loop() for hundreds of ms; the web server has its own task).
//
// Every public method is thread-safe: the web handlers, the ROS bridge and the
// waypoint runner all go through here, never to the controller directly.
#include <Arduino.h>
#include "control/SteerDriveController.h"
#include "control/TraceRecorder.h"

class ControlLoop {
public:
    void begin(SteerDriveController* ctrl);

    void command(const DriveCommand& cmd, CommandSource src);
    void halt(const char* why);               // stop the motor now, hold until a new command
    bool setPid(bool steerLoop, const float* v, size_t n);
    void getPid(bool steerLoop, float out[5]);
    uint32_t pidRevision();                  // successful PID writes, including advanced output limits
    void resetPose();
    RobotState snapshot();
    CommandSource source();
    bool moving();
    const char* lastHaltReason() const { return haltWhy_; }
    uint32_t tickUs() const { return tickUs_; }      // how long the last tick took

    // PIDF tuning on the robot: the last seconds at 100 Hz (TraceRecorder)
    uint16_t traceFreeze(uint32_t lastMs);
    const TraceRecorder::Sample& traceAt(uint16_t i) const { return trace_.at(i); }
    void traceUnfreeze();

private:
    static void taskEntry(void* arg);
    void run();
    struct Lock {
        explicit Lock(SemaphoreHandle_t m) : m_(m) { xSemaphoreTake(m_, portMAX_DELAY); }
        ~Lock() { xSemaphoreGive(m_); }
        SemaphoreHandle_t m_;
    };

    SteerDriveController* ctrl_ = nullptr;
    SemaphoreHandle_t mtx_ = nullptr;
    const char* haltWhy_ = "เพิ่งเปิดเครื่อง";
    volatile uint32_t tickUs_ = 0;
    TraceRecorder trace_;
    uint32_t pidRevision_ = 0;
};
