#!/usr/bin/env python3
"""Monitor goals while wheel control runs on firmware.

Firmware handles steering/drive transitions and odometry locally on the ESP32.
This node keeps listening to XY goals so operators can confirm what is being
issued, but it no longer publishes commands.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node


class GoalMonitor(Node):
    """Subscribe to XY goals and log them for visibility."""

    def __init__(self) -> None:
        super().__init__('robot_movement_monitor')
        self.create_subscription(Point, '/mor_luam/cmd_move_goal_xy', self._on_goal, 10)
        self.get_logger().info('Wheel control lives on firmware; monitoring goals only.')

    def _on_goal(self, msg: Point) -> None:
        dist = math.hypot(msg.x, msg.y)
        self.get_logger().info('Waypoint: x=%.3f y=%.3f speed=%.3f m/s (|goal|=%.3f m)',
                               msg.x, msg.y, msg.z, dist)


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
