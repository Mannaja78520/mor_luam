#pragma once
// Drives through the points set on the web page's 2D plane, on the robot
// itself (no ROS needed). Coordinates are the odom frame in metres: (0,0) is
// where the pose was last reset, +x is where the robot faced at that moment.
//
// Each step it plans ONE leg to the current point with the chosen algorithm
// (src/algorithm/), sends it to the controller, waits for it to finish, and
// plans again from where the robot really is - Algorithm 1.4 of the homework.
//
// Safety: a route keeps going only while a page sends heartbeat() (or polls
// /api/status) at least every WEB_HEARTBEAT_TIMEOUT_MS. Close the tab or lose
// Wi-Fi and it stops - enforced in the control task too (ControlLoop watchdog).
//
// The demo button (app/DemoButton.h) starts its own runs here: routes 1/2 (points
// saved per click count, relative to where the robot stands) and the Direct /
// Detour comparison (demos 3/4). They need no heartbeat and drive back to their start.
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "algorithm/LegPlanner.h"
#include "app/Settings.h"
#include "app_config.h"
#include "control/ControlLoop.h"
#include "nav/ActivityLog.h"
#include "nav/RunTrace.h"

struct Waypoint {
    float x;
    float y;
    float waitS;   // once reached, stop here this long (0 = drive on); 0..NAV_MAX_WAIT_S
};

class WaypointRunner {
public:
    enum class Status : uint8_t { Idle, Running, Done, Stopped, Failed };

    void begin(ControlLoop* ctrl, Settings* settings);
    void update();                                   // from loop(), any rate

    // from the web page (thread-safe). slot 0 = the web route; slot 1..DEMO_ROUTES =
    // the route the demo button drives on 1 / 2 clicks ((0,0) = where the robot
    // stands when the button is pressed, +x = where it faces).
    bool setPoints(const Waypoint* pts, size_t n, String& err, uint8_t slot = 0);
    void pointsJson(JsonArray out, uint8_t slot = 0);
    bool start(String& err);
    // One attended comparison trial. Align first, then time the current route on
    // the ESP32 with a temporary planner; saved settings/pose are not changed.
    bool startTest(const String& planner, float startHeadingDeg, bool ready, String& err);

    // From the robot's demo button: no web heartbeat needed (the operator is
    // there; a press stops it), a moving-time limit instead (see DEMO_MAX_MS). Both use their own
    // points and give the web route back when they end.
    // Button routes (1 / 2 clicks): here becomes (0,0); drive route `slot`, then back here.
    bool startButtonRoute(uint8_t slot, String& err);
    // Demos 3 / 4: a timed "direct" / "detour" trial to the DEMO_COMPARE_* goal,
    // placed from the wheel heading after alignment; then back to the start (untimed).
    // byButton false (POST /api/demo/compare/start): needs ready and the web heartbeat.
    bool startCompare(const String& planner, String& err, bool byButton = true, bool ready = true);
    // A series of `rounds` Direct + Detour pairs, run one after another by the robot
    // (GET /api/demo/series for the results). Any stop ends the series.
    bool startSeries(uint8_t rounds, String& err, bool byButton, bool ready);
    // run < 0: progress + every run's time and time line (no paths, ~2 KB);
    // run = i: run i with its path. Kept small on purpose: one big reply (~6 KB+)
    // never reached the page on the robot (2026-10-10). false = no such run.
    bool seriesJson(JsonObject out, int run = -1);
    bool seriesActive();
    void noteButton(const String& text, uint8_t clicks);   // last button event, shown on the web page
    void setButtonPressed(bool p) { buttonPressed_ = p; }   // live state for the web page
    void stop(const char* why);                      // also halts the wheel
    void heartbeat();
    void cancelForRos();                             // ROS took over: stop the route, keep its command
    void statusJson(JsonObject out);
    // GET /api/demo/compare: the last demo 3 (Direct) and demo 4 (Detour) runs with
    // their start, goal, time and recorded path, so the web page can draw both.
    void compareJson(JsonObject out);
    bool running();

private:
    enum class TestPhase : uint8_t { Idle, Aligning, Running, Done, Stopped, Failed };
    bool beginTest(const String& planner, float startHeadingDeg, bool ready, String& err, bool compare, bool local);
    void prepare(const SettingsData& settings);
    void alignTest(const RobotState& state, uint32_t now);
    bool testSensorsOk(const RobotState& state, uint32_t now) const;
    void sendCommand(const DriveCommand& cmd, const RobotState& before);
    void planNext(const RobotState& s);
    bool advance();                        // next point; false = nothing to plan now (finished or holding)
    bool readPoints(const char* key, uint8_t n, Waypoint* out);
    uint32_t demoLimitMs() const;          // button route: moving-time limit for pts_ from (0,0) and back
    void closeRun();                       // demo 3/4: the timed part ended (goal, stop or fault)
    void startNextInSeries();              // from update() while idle
    bool seriesBusy(String& err);          // refuse other starts while a series runs
    void finish(Status st, const char* why, bool haltWheel);
    void beginReturnHome();                // button demo: route done, drive back to its start
    void borrowPts();                      // a button demo uses pts_; finish() gives the web route back
    void load();
    void save();
    void saveRoute(uint8_t slot);
    void lock() { xSemaphoreTake(mtx_, portMAX_DELAY); }
    void unlock() { xSemaphoreGive(mtx_); }

    ControlLoop* ctrl_ = nullptr;
    Settings* settings_ = nullptr;
    Preferences prefs_;
    SemaphoreHandle_t mtx_ = nullptr;

    Waypoint pts_[NAV_MAX_POINTS];
    uint8_t count_ = 0;
    Waypoint routes_[DEMO_ROUTES][NAV_MAX_POINTS];   // the button routes (slot 1..)
    uint8_t routeCount_[DEMO_ROUTES] = {};
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
    bool waiting_ = false;                 // stopped at a reached point until waitUntilMs_
    bool waitAdvances_ = true;             // after the wait: next point (false: plan the current one)
    uint32_t waitUntilMs_ = 0, waitStartMs_ = 0;
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
    // started from the demo button: no heartbeat, time cap; borrows pts_
    bool local_ = false;
    bool compare_ = false;                       // demo 3/4: the goal is placed once the wheel is aligned
    bool returnHome_ = false, homing_ = false;   // button demos end where they started
    float homeX_ = 0.0f, homeY_ = 0.0f;
    uint32_t localStartMs_ = 0;            // moved forward by every stop, so now - it = moving time
    uint32_t localLimitMs_ = DEMO_MAX_MS;
    bool restorePts_ = false;
    Waypoint savedPts_[NAV_MAX_POINTS];
    uint8_t savedCount_ = 0;
    String buttonText_;
    uint8_t buttonClicks_ = 0;
    uint32_t buttonMs_ = 0;
    volatile bool buttonPressed_ = false;

    struct CompareRun {                    // one demo 3/4 run, kept until the next run of its planner
        uint32_t id = 0;                   // testId_; 0 = none since boot
        bool open = false;                 // still timed (driving to the goal)
        bool valid = false;                // reached the goal, timed, PID unchanged
        uint32_t elapsedMs = 0;
        float startX = 0, startY = 0, headingDeg = 0, goalX = 0, goalY = 0;
        float speedMps = 0, steerDps = 0, tolM = 0;
        RunTrace<100> path;
        ActivityLog<32> acts;              // turning / driving / still over time, for the time line
        uint8_t planner = 0;               // 0 direct, 1 detour
    };
    enum class PathJson : uint8_t { None, Flat, Objects };
    static void runJson(const CompareRun& r, JsonObject j, uint32_t elapsedMs, PathJson path);
    CompareRun seriesRuns_[DEMO_SERIES_MAX_ROUNDS * 2];
    uint8_t seriesTotal_ = 0, seriesCount_ = 0, seriesTries_ = 0;   // runs planned / recorded
    bool seriesActive_ = false, seriesLocal_ = false;
    uint32_t seriesId_ = 0, seriesNextMs_ = 0;
    String seriesWhy_;                     // why the last series ended early ("" = it did not)
    CompareRun runs_[2];                   // [0] direct (3 clicks), [1] detour (4 clicks)
    uint8_t runIdx_ = 0;
    bool testDemo_ = false;                // the current/last test came from the button (demo 3/4)
    float trackX_ = 0, trackY_ = 0;        // last position seen while a run is recorded
};
