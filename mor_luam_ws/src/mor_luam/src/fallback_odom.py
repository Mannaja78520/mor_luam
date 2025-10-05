#!/usr/bin/env python3
"""Pure-ROS odometry integrator to survive firmware resets."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Int32


def wrap_pi(angle: float) -> float:
    wrapped = (angle + math.pi) % (2.0 * math.pi)
    return wrapped - math.pi


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class FallbackOdom(Node):
    def __init__(self) -> None:
        super().__init__('mor_luam_fallback_odom')

        self.declare_parameter('ticks_topic', '/mor_luam/feedback/drive_ticks')
        self.declare_parameter('imu_topic', '/mor_luam/feedback/imu_yaw_deg')
        self.declare_parameter('odom_topic', '/mor_luam/odom/fallback')
        self.declare_parameter('steer_topic', '/mor_luam/feedback/steer_deg')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('wheel_diameter', 0.0762)
        self.declare_parameter('counts_per_rev', 64.0)
        self.declare_parameter('gear_ratio', 11.55555555)
        self.declare_parameter('max_jump_revs', 20.0)
        self.declare_parameter('default_dt', 0.01)

        p = self.get_parameter
        self._ticks_topic = p('ticks_topic').value
        self._imu_topic = p('imu_topic').value
        self._odom_topic = p('odom_topic').value
        self._steer_topic = p('steer_topic').value
        self._frame_id = p('frame_id').value
        self._child_frame_id = p('child_frame_id').value

        wheel_diameter = float(p('wheel_diameter').value)
        counts_per_rev = float(p('counts_per_rev').value)
        gear_ratio = float(p('gear_ratio').value)
        circumference = math.pi * wheel_diameter
        self._ticks_to_m = circumference / (counts_per_rev * gear_ratio)
        self._jump_threshold = int(counts_per_rev * gear_ratio * float(p('max_jump_revs').value))
        self._default_dt = float(p('default_dt').value)

        self._pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._sub_ticks = self.create_subscription(Int32, self._ticks_topic, self._on_ticks, 50)
        self._sub_yaw = self.create_subscription(Float32, self._imu_topic, self._on_yaw, 50)
        self._sub_steer = self.create_subscription(Float32, self._steer_topic, self._on_steer, 50)

        self._last_ticks: Optional[int] = None
        self._last_time: Optional[Time] = None
        self._yaw_rad: float = 0.0
        self._have_yaw: bool = False
        self._x_m: float = 0.0
        self._y_m: float = 0.0
        self._prev_yaw_for_twist: float = 0.0
        self._steer_rad: float = 0.0
        self._have_steer: bool = False

    def _on_yaw(self, msg: Float32) -> None:
        self._yaw_rad = wrap_pi(math.radians(msg.data))
        self._have_yaw = True

    def _on_steer(self, msg: Float32) -> None:
        self._steer_rad = wrap_pi(math.radians(msg.data))
        self._have_steer = True

    def _on_ticks(self, msg: Int32) -> None:
        now = self.get_clock().now()
        if self._last_ticks is None:
            self._last_ticks = msg.data
            self._last_time = now
            if self._have_yaw:
                self._prev_yaw_for_twist = self._yaw_rad
            elif self._have_steer:
                self._prev_yaw_for_twist = self._steer_rad
            else:
                self._prev_yaw_for_twist = 0.0
            return

        delta_ticks = msg.data - self._last_ticks
        if abs(delta_ticks) > self._jump_threshold:
            self._last_ticks = msg.data
            self._last_time = now
            self._prev_yaw_for_twist = self._yaw_rad
            self.get_logger().warn(
                f'Encoder jump detected (Δticks={delta_ticks}); resyncing without motion update')
            return

        self._last_ticks = msg.data

        distance = delta_ticks * self._ticks_to_m
        if self._have_yaw:
            heading = self._yaw_rad
        elif self._have_steer:
            heading = self._steer_rad
        else:
            heading = self._prev_yaw_for_twist
        self._x_m += distance * math.cos(heading)
        self._y_m += distance * math.sin(heading)

        if self._last_time is None:
            self._last_time = now
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        if dt <= 0.0:
            dt = self._default_dt

        vx = distance / dt
        current_heading = self._yaw_rad if self._have_yaw else heading
        yaw_delta = wrap_pi(current_heading - self._prev_yaw_for_twist)
        wz = yaw_delta / dt
        self._prev_yaw_for_twist = current_heading

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose.position.x = self._x_m
        odom.pose.pose.position.y = self._y_m
        odom.pose.pose.orientation = yaw_to_quaternion(heading)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self._pub.publish(odom)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FallbackOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
