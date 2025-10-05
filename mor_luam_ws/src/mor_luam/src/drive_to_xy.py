#!/usr/bin/env python3
"""Drive the robot to an XY goal using the firmware-controlled wheel."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

CMD_TOPIC = '/mor_luam/cmd_vel/move'
DEFAULT_ODOM_TOPIC = '/mor_luam/odom/esp'


def mps_to_rpm(speed_mps: float, wheel_diameter_m: float) -> float:
    if wheel_diameter_m <= 0.0:
        raise ValueError('wheel diameter must be > 0')
    circumference = math.pi * wheel_diameter_m
    if circumference <= 0.0:
        raise ValueError('wheel circumference must be > 0')
    return (speed_mps / circumference) * 60.0


def wrap_deg(angle_deg: float) -> float:
    return angle_deg % 360.0


def radians_to_degrees(angle_rad: float) -> float:
    return math.degrees(angle_rad)


class DriveToXY(Node):
    def __init__(self, *, target_x: float, target_y: float, target_rpm: float,
                 tolerance_m: float, odom_topic: str, odom_timeout: float) -> None:
        super().__init__('drive_to_xy_helper')

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self._cmd_pub = self.create_publisher(Twist, CMD_TOPIC, qos)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._target_x = target_x
        self._target_y = target_y
        self._target_rpm = target_rpm
        self._tolerance = max(tolerance_m, 0.0)
        self._odom_timeout = odom_timeout
        self._odom_topic = odom_topic

        self._cmd_sent = False
        self._have_pose = False
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._heading_deg: Optional[float] = None
        self._last_progress_distance: Optional[float] = None
        self._last_progress_time = 0.0
        self._stall_timeout = 1.5  # seconds to wait for progress before reissuing
        self._progress_eps = 0.01  # metres of improvement considered meaningful

    def _on_odom(self, msg: Odometry) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        self._have_pose = True

    def _send_command(self, rpm: float, heading_deg: float, distance_m: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(rpm)
        msg.linear.y = float(distance_m)
        msg.linear.z = float(self._tolerance)
        msg.angular.z = float(heading_deg)
        self._cmd_pub.publish(msg)

    def drive(self) -> bool:
        start_time = self.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self._have_pose:
                if (self.get_clock().now() - start_time) > Duration(seconds=self._odom_timeout):
                    self.get_logger().error(f'Timed out waiting for odometry on topic {self._odom_topic}')
                    return False
                time.sleep(0.05)
                continue

            dx = self._target_x - self._pose_x
            dy = self._target_y - self._pose_y
            distance = math.hypot(dx, dy)

            heading_rad = math.atan2(dy, dx) if (dx or dy) else 0.0
            heading_deg = wrap_deg(radians_to_degrees(heading_rad))

            now = time.monotonic()

            if (self._last_progress_distance is None or
                    distance < self._last_progress_distance - self._progress_eps):
                self._last_progress_distance = distance
                self._last_progress_time = now

            if not self._cmd_sent:
                self._heading_deg = heading_deg
                self._send_command(self._target_rpm, heading_deg, distance)
                self.get_logger().info(
                    f'Heading {heading_deg:.1f}° at {self._target_rpm:.2f} RPM (distance {distance:.3f} m)')
                self._cmd_sent = True
                self._last_progress_distance = distance
                self._last_progress_time = now
            elif distance > self._tolerance:
                if now - self._last_progress_time >= self._stall_timeout:
                    self._heading_deg = heading_deg
                    self._send_command(self._target_rpm, heading_deg, distance)
                    self.get_logger().info(
                        f'Reissuing command after stall: heading {heading_deg:.1f}° distance {distance:.3f} m')
                    self._last_progress_distance = distance
                    self._last_progress_time = now

            if distance <= self._tolerance:
                heading_deg = self._heading_deg if self._heading_deg is not None else 0.0
                self._send_command(0.0, heading_deg, 0.0)
                self.get_logger().info(f'Reached goal (|Δ|={distance:.3f} m); commanding stop.')
                time.sleep(0.2)
                return True

            time.sleep(0.05)

        return False


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Drive the robot to an XY position using firmware odometry')
    parser.add_argument('x', type=float, help='Target X position in metres (odom frame)')
    parser.add_argument('y', type=float, help='Target Y position in metres (odom frame)')
    parser.add_argument('--speed', type=float, default=0.25,
                        help='Speed value (unit controlled by --unit). Default: 0.25')
    parser.add_argument('--unit', choices=('rpm', 'mps'), default='rpm',
                        help='Interpret --speed as wheel RPM or metres/second (default: rpm)')
    parser.add_argument('--wheel-diameter', type=float, default=0.0762,
                        help='Wheel diameter in metres when --unit mps (default: 0.0762)')
    parser.add_argument('--tolerance', type=float, default=0.05,
                        help='Goal radius in metres (default: 0.05)')
    parser.add_argument('--odom-topic', type=str, default=DEFAULT_ODOM_TOPIC,
                        help=f'Odometry topic to follow (default: {DEFAULT_ODOM_TOPIC})')
    parser.add_argument('--odom-timeout', type=float, default=5.0,
                        help='Seconds to wait for first odom message (default: 5.0)')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    try:
        if args.unit == 'mps':
            target_rpm = mps_to_rpm(args.speed, args.wheel_diameter)
        else:
            target_rpm = float(args.speed)
    except ValueError as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 2

    if target_rpm <= 0.0:
        print('Speed must be positive to drive to a goal.', file=sys.stderr)
        return 2

    rclpy.init()
    node = DriveToXY(target_x=args.x,
                     target_y=args.y,
                     target_rpm=target_rpm,
                     tolerance_m=args.tolerance,
                     odom_topic=args.odom_topic,
                     odom_timeout=args.odom_timeout)
    try:
        success = node.drive()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
