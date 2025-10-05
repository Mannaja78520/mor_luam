#!/usr/bin/env python3
"""Publish a direct heading/speed command to the firmware controller."""

from __future__ import annotations

import argparse
import math
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

TOPIC = '/mor_luam/cmd_vel/move'


def mps_to_rpm(speed_mps: float, wheel_diameter_m: float) -> float:
    if wheel_diameter_m <= 0.0:
        raise ValueError('wheel diameter must be > 0')
    circumference = math.pi * wheel_diameter_m
    if circumference <= 0.0:
        raise ValueError('wheel circumference must be > 0')
    return (speed_mps / circumference) * 60.0


class HeadingSpeedSender(Node):
    def __init__(self) -> None:
        super().__init__('heading_speed_sender')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Twist, TOPIC, qos)

    def _wait_for_match(self, timeout_sec: float = 2.0) -> bool:
        deadline = self.get_clock().now() + Duration(seconds=timeout_sec)
        while self.get_clock().now() < deadline:
            if self._pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def send(self, rpm: float, heading_deg: float, wait_timeout: float = 2.0) -> None:
        matched = self._wait_for_match(wait_timeout)
        if not matched:
            self.get_logger().warn('No subscriber matched; sending command anyway')
        msg = Twist()
        msg.linear.x = float(rpm)
        msg.angular.z = float(heading_deg)
        self._pub.publish(msg)
        self.get_logger().info(f'Sent heading {heading_deg:.1f}° @ {rpm:.2f} RPM')
        rclpy.spin_once(self, timeout_sec=0.1)


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Send heading/speed command to firmware')
    parser.add_argument('heading', type=float, help='Heading in degrees (0..360)')
    parser.add_argument('speed', type=float, help='Wheel speed value (unit controlled by --unit)')
    parser.add_argument('--unit', choices=('rpm', 'mps'), default='rpm', help='Interpret speed as RPM or m/s (default: rpm)')
    parser.add_argument('--wheel-diameter', type=float, default=0.0762, help='Wheel diameter in metres when --unit mps (default: 0.0762)')
    parser.add_argument('--wait-timeout', type=float, default=2.0, help='Seconds to wait for subscriber discovery before sending')
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    try:
        if args.unit == 'mps':
            rpm = mps_to_rpm(args.speed, args.wheel_diameter)
        else:
            rpm = float(args.speed)
    except ValueError as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 2

    heading = float(args.heading) % 360.0

    rclpy.init()
    node = HeadingSpeedSender()
    try:
        node.send(rpm=rpm, heading_deg=heading, wait_timeout=args.wait_timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
