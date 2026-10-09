#pragma once
// The demo button on the robot (DEMO_BUTTON_PIN, to GND, pull-up).
//
//   1 click   demo 1: here becomes (0,0); forward 1 m, left 1 m, back to the start
//   2 clicks  demo 2: the route set on the web page, Direct (no shortcut), timed
//   3 clicks  demo 3: the same route with Detour Steer (the shortcut), timed
//   any press while the robot moves: STOP (like E-STOP on the web page)
//
// A demo starts 1 s after the last click. Demos started here do not need the
// web page's heartbeat (the operator is at the robot; the button stops it) and
// stop by themselves after DEMO_MAX_MS.
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
