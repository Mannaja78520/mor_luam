#!/usr/bin/env python3
"""Host-side helper for mor_luam.

All closed-loop wheel control now runs directly on the firmware. This node keeps
listening to target waypoints so we can log and verify what is being sent to the
robot, but it no longer publishes velocity commands back to the ESP32.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class GoalMonitor(Node):
    """Log XY goals as they are issued to the firmware."""

    def __init__(self) -> None:
        super().__init__('robot_movement_monitor')
        self.create_subscription(Point, '/mor_luam/cmd_move_goal_xy', self._on_goal, 10)
        self.get_logger().info('Wheel control handled on firmware; monitoring goals only.')

    def _on_goal(self, msg: Point) -> None:
        dist = math.hypot(msg.x, msg.y)
        self.get_logger().info(
            'Waypoint: x=%.3f y=%.3f speed=%.3f m/s (|goal|=%.3f m)',
            msg.x, msg.y, msg.z, dist,
        )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GoalMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
