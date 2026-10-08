#pragma once
// micro-ROS link to the PC (agent on UDP 8888, ROS_DOMAIN_ID 10).
//
// Same topics, types and domain as before the refactor, so the ROS 2 programs
// (drive_to_xy.py, fallback_odom.py, ...) need no change:
//   publishes  /mor_luam/feedback/drive_ticks (Int32), steer_deg, imu_yaw_deg (Float32),
//              imu_ok (Bool), /mor_luam/odom/esp (Odometry), /mor_luam/debug/esp_state (String),
//              /mor_luam/debug/wheel/cmd_vel (Twist)
//   subscribes /mor_luam/cmd_vel/move (Twist), /mor_luam/config/spin_pid, steer_pid (Float32MultiArray)
//
// What is new: the Wi-Fi is NetworkManager's (many networks), and the agent is
// FOUND rather than fixed - candidates() in the .cpp. Losing the agent stops a
// command that came from ROS; a route started from the web carries on.
//
// All micro-ROS objects live in the .cpp so its headers never meet the web server's.
#include <Arduino.h>
#include <ArduinoJson.h>

class ControlLoop;
class WaypointRunner;
class Settings;
class NetworkManager;

class MicroRosBridge {
public:
    enum class State : uint8_t { NoWifi, Waiting, Connected };
    void begin(ControlLoop* ctrl, WaypointRunner* runner, Settings* settings, NetworkManager* net);
    void loop();
    void statusJson(JsonObject out);
    State state() const { return state_; }

private:
    State state_ = State::NoWifi;
};
