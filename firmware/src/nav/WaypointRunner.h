#pragma once
// Drives through the points set on the web page's 2D plane, on the robot
// itself (no ROS needed). Coordinates are the odom frame in metres: (0,0) is
// where the pose was last reset, +x is where the robot faced at that moment.
//
// Each step it plans ONE leg to the current point with the chosen algorithm
// (src/algorithm/), sends it to the controller, waits for it to finish, and
// plans again from where the robot really is - Algorithm 1.4 of the homework.
//
// Safety: a route keeps going only while the page sends heartbeat() at least
// every WEB_HEARTBEAT_TIMEOUT_MS. Close the tab or lose Wi-Fi and it stops.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "algorithm/LegPlanner.h"
#include "app/Settings.h"
#include "control/ControlLoop.h"

struct Waypoint {
    float x;
    float y;
};

class WaypointRunner {
public:
    enum class Status : uint8_t { Idle, Running, Done, Stopped, Failed };

    void begin(ControlLoop* ctrl, Settings* settings);
    void update();                                   // from loop(), any rate

    // from the web page (thread-safe)
    bool setPoints(const Waypoint* pts, size_t n, String& err);
    void pointsJson(JsonArray out);
    bool start(String& err);
    // One attended comparison trial. Align first, then time the current route on
    // the ESP32 with a temporary planner; saved settings/pose are not changed.
    bool startTest(const String& planner, float startHeadingDeg, bool ready, String& err);
    void stop(const char* why);                      // also halts the wheel
    void heartbeat();
    void cancelForRos();                             // ROS took over: stop the route, keep its command
    void statusJson(JsonObject out);
    bool running();

private:
    enum class TestPhase : uint8_t { Idle, Aligning, Running, Done, Stopped, Failed };
    void prepare(const SettingsData& settings);
    void alignTest(const RobotState& state, uint32_t now);
    bool testSensorsOk(const RobotState& state, uint32_t now) const;
    void sendCommand(const DriveCommand& cmd, const RobotState& before);
    void planNext(const RobotState& s);
    void finish(Status st, const char* why, bool haltWheel);
    void load();
    void save();
    void lock() { xSemaphoreTake(mtx_, portMAX_DELAY); }
    void unlock() { xSemaphoreGive(mtx_); }

    ControlLoop* ctrl_ = nullptr;
    Settings* settings_ = nullptr;
    Preferences prefs_;
    SemaphoreHandle_t mtx_ = nullptr;

    Waypoint pts_[32];
    uint8_t count_ = 0;
    Status status_ = Status::Idle;
    String message_ = "พร้อม";
    uint8_t idx_ = 0;
    uint8_t tries_ = 0;
    bool legActive_ = false;
    bool loop_ = false;
    float speedMps_ = 0.25f, tolM_ = 0.05f;
    String plannerName_ = "detour";
    RobotParams params_;
    LegPlan lastPlan_;
    float lastPhiDeg_ = 0.0f, lastDistM_ = 0.0f;
    uint8_t overshoots_ = 0;
    uint32_t heartbeatMs_ = 0;
    uint32_t lastUpdateMs_ = 0;
    // A command changes the controller before its published RobotState changes.
    // Wait for a later control tick before interpreting halted/goalActive.
    bool commandPending_ = false;
    uint32_t commandStampMs_ = 0;
    uint32_t testId_ = 0;
    uint32_t testPidRevision_ = 0;
    TestPhase testPhase_ = TestPhase::Idle;
    float testHeadingDeg_ = 0.0f, testActualHeadingDeg_ = 0.0f;
    float testStartX_ = 0.0f, testStartY_ = 0.0f, testStartThetaDeg_ = 0.0f;
    float testSteerTolDeg_ = 0.0f;
    uint32_t testPrepMs_ = 0, testStableMs_ = 0, testStableSampleMs_ = 0, testStartMs_ = 0, testElapsedMs_ = 0;
    uint32_t testMaxUpdateGapMs_ = 0;
    bool testStable_ = false, testTimed_ = false, testValid_ = false;
};
