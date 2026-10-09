#pragma once
// The demo button on the robot (DEMO_BUTTON_PIN, to GND, pull-up).
//
//   1 click   demo 1: route 1 (set on the web page; until then forward 1 m, left 1 m, back)
//   2 clicks  demo 2: route 2 (set on the web page; until then the same with 0.5 m)
//             routes 1/2: here becomes (0,0), +x = where the robot faces
//   3 clicks  demo 3: Direct (no shortcut) to a fixed goal 0.30 m away, 6 deg right of the wheel, timed
//   4 clicks  demo 4: Detour Steer (the shortcut) to the same goal, timed
//   every demo drives back to where it started, so the next one can run at once
//   any press while the robot moves: STOP (like E-STOP on the web page)
//
// A demo starts 1 s after the last click. Demos started here do not need the
// web page's heartbeat (the operator is at the robot; the button stops it) and
// stop by themselves when their moving time passes a limit set from the route
// (5-30 min; stops at points do not count; see DEMO_MAX_MS in app_config.h).
//
// The button is read every 5 ms in its own small task, so a busy loop()
// (Wi-Fi, micro-ROS) can neither miss a click nor delay a STOP.
#include <Arduino.h>
#include "util/ClickCounter.h"

class WaypointRunner;
class ControlLoop;

class DemoButton {
public:
    void begin(uint8_t pin, WaypointRunner* runner, ControlLoop* ctrl);

private:
    static void taskEntry(void* arg);
    void run();
    void startDemo(uint8_t clicks);
    void note(const String& text, uint8_t clicks);

    uint8_t pin_ = 0;
    WaypointRunner* runner_ = nullptr;
    ControlLoop* ctrl_ = nullptr;
    ClickCounter counter_;
    bool lastPressed_ = false;
};
