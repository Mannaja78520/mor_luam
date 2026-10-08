#pragma once
// One command for the wheel: steer to a world heading, then drive.
// Same meaning as the ROS topic /mor_luam/cmd_vel/move (geometry_msgs/Twist):
//   linear.x = rpm (signed), angular.z = heading deg, linear.y = distance m, linear.z = tolerance m
#include <stdint.h>

struct DriveCommand {
    float rpm = 0.0f;          // wheel rpm, signed; 0 = steer only
    float headingDeg = 0.0f;   // world heading of travel, 0..360
    float distM = 0.0f;        // 0 = drive until told otherwise
    float tolM = 0.0f;         // 0 = keep the current tolerance
    // If the wheel turns PAST its angle, stop there instead of turning almost a
    // full circle again, and let the planner aim from where it stopped
    // (Detour Steer, Algorithm 1.4). Off for ROS: same behaviour as before.
    bool stopOnOvershoot = false;
};

// Who gave the command now running. Losing that source stops the robot.
enum class CommandSource : uint8_t { None, Ros, Web };

inline const char* sourceName(CommandSource s) {
    switch (s) {
        case CommandSource::Ros: return "ros";
        case CommandSource::Web: return "web";
        default: return "none";
    }
}
