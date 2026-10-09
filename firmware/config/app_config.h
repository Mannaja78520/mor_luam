/**
 * @file app_config.h
 * @brief Behaviour constants of the firmware, in one place.
 *
 * Hardware facts are in esp32_hardware.h, PID gains in PIDF_config.h and
 * network defaults in conf_network.h. Values that people change in the field
 * (Wi-Fi, names, agent address, nav speed) are NOT here: they live in NVS and
 * are edited from the web page (src/app/Settings.h).
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define FW_VERSION "2.0.0"

//---- control loop (src/control/SteerDriveController.cpp) ----
static const uint32_t CTRL_PERIOD_MS          = 10;     // 100 Hz
static const float    CTRL_PERIOD_S           = CTRL_PERIOD_MS / 1000.0f;
static const int      STEER_POWER_LIMIT_PWM   = 500;    // comfort limit from attended BNO085 A/B test; hardware maximum unchanged
static const int      STEER_MOTOR_DIR         = +1;     // motor direction that steers (one way only: counter-clockwise from above)
static const int      DRIVE_MOTOR_DIR         = -1;     // motor direction that drives forward
static const uint32_t STEER_SETTLE_MS         = 50;     // inside tolerance this long before driving
static const float    STEER_CMD_ZERO_DEG      = 0.0f;   // offset added to the commanded wheel angle
static const float    STEER_TOL_HYST_DEG      = 1.0f;   // once aimed, stay aimed until this much past the tolerance
static const float    STEER_GLITCH_DEG        = 6.0f;   // AS5600: a bigger jump in one tick is a bad reading
static const uint8_t  STEER_GLITCH_CONFIRM    = 3;      // ...unless this many readings in a row agree
static const float    CMD_SMALL_HEADING_EPS   = 2.5f;   // deg: smaller re-commands are ignored
static const float    CMD_SMALL_DIST_EPS      = 0.02f;  // m
static const float    CMD_SMALL_RPM_EPS       = 1.0f;   // rpm
static const float    DEFAULT_GOAL_TOL_M      = 0.06f;

//---- micro-ROS (src/ros/MicroRosBridge.cpp) ----
static const uint32_t ROS_PUBLISH_PERIOD_MS   = 20;     // each call: odom + one of the 6 small topics (MicroRosBridge publish())
static const uint32_t ROS_PING_WAITING_MS     = 1500;   // look for the agent this often
static const uint32_t ROS_PING_CONNECTED_MS   = 700;    // check the agent is still there
static const uint8_t  ROS_PING_MISSES_LOST     = 4;      // that many failed checks in a row = agent lost (~3 s)

//---- web driving safety (src/nav/WaypointRunner.cpp) ----
// A route started from the web keeps going only while the page keeps saying
// it is still open. Close the tab or lose Wi-Fi and the robot stops.
static const uint32_t WEB_HEARTBEAT_TIMEOUT_MS = 3000;
static const uint8_t  NAV_MAX_RETRIES          = 6;     // re-aims per waypoint before giving up
static const uint8_t  NAV_MAX_POINTS           = 32;
static const float    NAV_MAX_WAIT_S           = 60.0f;  // longest stop at one point (set per point on the web page)
static const float    TEST_MAX_RPM             = 150.0f; // POST /api/test/move limits
// Demo button (app/DemoButton.h, pin DEMO_BUTTON_PIN in esp32_hardware.h)
static const uint32_t DEMO_DEBOUNCE_MS         = 30;
static const uint32_t DEMO_CLICK_GAP_MS        = 1000;   // a demo starts this long after the last click
static const uint32_t DEMO_MAX_PRESS_MS        = 1500;   // held longer = not a click
static const uint8_t  DEMO_ROUTES              = 2;      // 1 and 2 clicks: routes set on the web page
static const float    DEMO_SQUARE_M            = 1.0f;   // route 1 until set: forward, left, back (m)
static const float    DEMO_TRIANGLE_M          = 0.5f;   // route 2 until set: forward, left, back (m)
static const float    DEMO_START_HEADING_DEG   = 0.0f;   // demos 3/4: wheel turned to +x first, then timed
// Demos 3/4 (Direct / Detour): one goal this far away, this far clockwise (to the
// right) of the wheel once it is aligned. Direct must first turn the wheel ~350 deg;
// Detour drives first, turns less, then a short leg sideways. Picked so the planner
// chooses the detour at 0.03 m/s with steerDps 35..60 (2026-10-09).
static const float    DEMO_COMPARE_DIST_M      = 0.20f;
static const float    DEMO_COMPARE_RIGHT_DEG   = 10.0f;
static const float    DEMO_COMPARE_HOLD_S      = 5.0f;   // demos 3/4: stay at the goal, then drive back (not timed)
static const uint32_t DEMO_MAX_MS              = 300000; // a button demo stops by itself after 5 min
static const float    NAV_DEFAULT_SPEED_MPS    = 0.03f;  // 7.5 rpm: full power is only ~0.039 m/s (PIDF_config.h)
static const float    NAV_MAX_SPEED_MPS        = 0.035f; // leave the PID some power to spare
static const float    TEST_MAX_DIST_M          = 2.0f;

//---- Detour Steer (src/nav/DetourPlanner.h) ----
static const float    DETOUR_SETTLE_S          = STEER_SETTLE_MS / 1000.0f;
static const float    DETOUR_STOP_S            = 0.20f; // stop + clutch switch, drive -> steer

//---- network (src/net/NetworkManager.cpp) ----
static const uint32_t WIFI_CONNECT_TIMEOUT_MS  = 12000;
static const uint32_t WIFI_RETRY_PERIOD_MS     = 15000;
static const uint32_t WIFI_MOVE_UP_CHECK_MS    = 30000; // on priority 2, 3 ...: look for a higher one this often (robot still)
static const int      WIFI_MOVE_UP_MIN_RSSI    = -75;   // dBm: weaker than this is not worth moving to
static const uint32_t WIFI_SCAN_MS_PER_CHANNEL = 120;   // ~1.5 s per scan
static const uint32_t AP_START_AFTER_MS        = 20000; // no Wi-Fi this long -> open the setup hotspot
static const uint32_t AP_KEEP_AFTER_JOIN_MS    = 60000; // keep it this long after joining, then close

#endif
