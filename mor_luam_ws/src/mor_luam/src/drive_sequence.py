#!/usr/bin/env python3
"""Drive the robot through a sequence of XY waypoints."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

try:
    import yaml
except ImportError:  # pragma: no cover - optional dependency
    yaml = None

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


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    mode: str = 'goto'  # 'goto' (stop) or 'through'
    speed_override: Optional[float] = None  # m/s or rpm depending on unit

    def require_stop(self) -> bool:
        return self.mode == 'goto'

    @classmethod
    def from_sequence(cls, values: Sequence[str | float]) -> Waypoint:
        if len(values) not in (2, 3, 4):
            raise ValueError('waypoint requires x y [mode] [speed]')

        x = float(values[0])
        y = float(values[1])
        mode = str(values[2]).lower() if len(values) >= 3 else 'goto'
        if mode not in ('goto', 'through'):
            raise ValueError(f'mode must be goto/through, got {mode}')
        speed_override: Optional[float] = None
        if len(values) == 4:
            speed_override = float(values[3])
        return cls(x=x, y=y, mode=mode, speed_override=speed_override)

    @classmethod
    def from_dict(cls, data: dict) -> Waypoint:
        return cls(
            x=float(data['x']),
            y=float(data['y']),
            mode=str(data.get('mode', 'goto')).lower(),
            speed_override=float(data['speed']) if 'speed' in data else None,
        )


class DriveSequence(Node):
    def __init__(
        self,
        *,
        waypoints: Iterable[Waypoint],
        target_rpm: float,
        goto_tolerance_m: float,
        through_tolerance_m: float,
        odom_topic: str,
        odom_timeout: float,
        stall_timeout: float,
        progress_eps: float,
    ) -> None:
        super().__init__('drive_sequence_helper')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._cmd_pub = self.create_publisher(Twist, CMD_TOPIC, qos)
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self._waypoints = list(waypoints)
        self._target_rpm_default = target_rpm
        self._goto_tol = max(goto_tolerance_m, 0.0)
        self._through_tol = max(through_tolerance_m, 0.0)
        self._odom_timeout = odom_timeout
        self._stall_timeout = stall_timeout
        self._progress_eps = progress_eps

        self._have_pose = False
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._last_heading_cmd_deg: Optional[float] = None

    def _on_odom(self, msg: Odometry) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        self._have_pose = True

    def _send_command(self, rpm: float, heading_deg: float, distance_m: float = 0.0, tolerance_m: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(rpm)
        msg.linear.y = float(distance_m)
        msg.linear.z = float(tolerance_m)
        msg.angular.z = float(heading_deg)
        self._cmd_pub.publish(msg)

    def drive_all(self) -> bool:
        for idx, wp in enumerate(self._waypoints):
            label = f'[{idx + 1}/{len(self._waypoints)}] ({wp.x:.3f}, {wp.y:.3f}) mode={wp.mode}'
            self.get_logger().info(f'Starting waypoint {label}')
            success = self._drive_waypoint(wp)
            if not success:
                self.get_logger().error(f'Aborting sequence at waypoint {label}')
                return False
        self.get_logger().info('Sequence complete; commanding stop')
        self._send_command(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.2)
        return True

    def _drive_waypoint(self, wp: Waypoint) -> bool:
        start_time = self.get_clock().now()
        odom_deadline = start_time + Duration(seconds=self._odom_timeout)

        self._have_pose = False if not self._have_pose else self._have_pose
        cmd_sent = False
        last_progress_distance: Optional[float] = None
        last_progress_time = time.monotonic()
        start_x: Optional[float] = None
        start_y: Optional[float] = None
        goal_vec_x = 0.0
        goal_vec_y = 0.0
        goal_len_sq = 0.0
        self._last_heading_cmd_deg = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if not self._have_pose:
                if self.get_clock().now() > odom_deadline:
                    self.get_logger().error('Timed out waiting for odometry')
                    return False
                time.sleep(0.05)
                continue

            dx = wp.x - self._pose_x
            dy = wp.y - self._pose_y
            distance = math.hypot(dx, dy)
            heading_rad = math.atan2(dy, dx) if (dx or dy) else 0.0
            heading_deg = wrap_deg(radians_to_degrees(heading_rad))
            steer_command = heading_deg

            now = time.monotonic()

            if start_x is None:
                start_x = self._pose_x
                start_y = self._pose_y
                goal_vec_x = wp.x - start_x
                goal_vec_y = wp.y - start_y
                goal_len_sq = goal_vec_x * goal_vec_x + goal_vec_y * goal_vec_y

            if (last_progress_distance is None) or (distance < last_progress_distance - self._progress_eps):
                last_progress_distance = distance
                last_progress_time = now

            tolerance = self._goto_tol if wp.require_stop() else self._through_tol
            speed_rpm = self._target_rpm_default
            if wp.speed_override is not None:
                speed_rpm = wp.speed_override

            if (not cmd_sent) or (distance > tolerance and now - last_progress_time >= self._stall_timeout):
                self._last_heading_cmd_deg = steer_command
                self._send_command(speed_rpm, steer_command, distance, tolerance)
                self.get_logger().info(
                    f'Commanding heading {heading_deg:.1f}° distance {distance:.3f} m '
                    f'@ {speed_rpm:.2f} RPM (tol={tolerance:.3f} m)'
                )
                cmd_sent = True
                last_progress_distance = distance
                last_progress_time = now

            if wp.require_stop():
                if distance <= tolerance:
                    stop_heading = self._last_heading_cmd_deg if self._last_heading_cmd_deg is not None else steer_command
                    self._send_command(0.0, stop_heading, 0.0, 0.0)
                    self.get_logger().info('Reached waypoint; stopping')
                    time.sleep(0.2)
                    return True
            else:
                progress_x = self._pose_x - (start_x or self._pose_x)
                progress_y = self._pose_y - (start_y or self._pose_y)
                along_dot = progress_x * goal_vec_x + progress_y * goal_vec_y
                if distance <= tolerance or (goal_len_sq > 1e-6 and along_dot >= goal_len_sq):
                    self.get_logger().info('Passed through waypoint; continuing')
                    self._last_heading_cmd_deg = steer_command
                    return True

            time.sleep(0.05)

        return False


def load_waypoints_from_json(path: Path) -> list[Waypoint]:
    data = json.loads(path.read_text())
    if not isinstance(data, list):
        raise ValueError('JSON must contain a list of waypoints')
    waypoints: list[Waypoint] = []
    for entry in data:
        if not isinstance(entry, dict):
            raise ValueError('Each waypoint must be an object with x/y[/mode][/speed]')
        waypoints.append(Waypoint.from_dict(entry))
    return waypoints


def load_waypoints_from_yaml(path: Path) -> list[Waypoint]:
    raw = yaml.safe_load(path.read_text())
    if isinstance(raw, dict):
        if 'waypoints' not in raw:
            raise ValueError('YAML dict must contain "waypoints" list')
        raw_list = raw['waypoints']
    elif isinstance(raw, list):
        raw_list = raw
    else:
        raise ValueError('YAML must be a list or a dict with key "waypoints"')

    waypoints: list[Waypoint] = []
    for entry in raw_list:
        if not isinstance(entry, dict):
            raise ValueError('Each waypoint must be a mapping with x/y[/mode][/speed]')
        waypoints.append(Waypoint.from_dict(entry))
    return waypoints


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Drive through a sequence of XY waypoints using firmware odometry'
    )
    parser.add_argument(
        '--waypoint',
        metavar='ARG',
        type=str,
        nargs='+',
        action='append',
        help='Add a waypoint: X Y [MODE] [SPEED]. MODE defaults to goto; SPEED overrides default speed (unit depends on --unit).'
    )
    parser.add_argument(
        '--waypoints-json',
        type=Path,
        help='Load waypoints from a JSON file containing a list of {"x":..,"y":..,"mode":"goto|through","speed":optional}'
    )
    parser.add_argument(
        '--waypoints-yaml',
        type=Path,
        help='Load waypoints from a YAML file. Supports either a list of waypoints or a dict with key "waypoints".'
    )
    parser.add_argument('--speed', type=float, default=0.25,
                        help='Speed value (unit controlled by --unit). Default: 0.25')
    parser.add_argument('--unit', choices=('rpm', 'mps'), default='rpm',
                        help='Interpret --speed and waypoint overrides as RPM or metres/second (default: rpm)')
    parser.add_argument('--wheel-diameter', type=float, default=0.0762,
                        help='Wheel diameter in metres when --unit mps (default: 0.0762)')
    parser.add_argument('--goto-tolerance', type=float, default=0.05,
                        help='Tolerance in metres for goto waypoints (default: 0.05)')
    parser.add_argument('--through-tolerance', type=float, default=0.15,
                        help='Tolerance in metres for through waypoints (default: 0.15)')
    parser.add_argument('--odom-topic', type=str, default=DEFAULT_ODOM_TOPIC,
                        help=f'Odometry topic to follow (default: {DEFAULT_ODOM_TOPIC})')
    parser.add_argument('--odom-timeout', type=float, default=5.0,
                        help='Seconds to wait for first odom message (default: 5.0)')
    parser.add_argument('--stall-timeout', type=float, default=1.5,
                        help='Seconds without progress before reissuing command (default: 1.5)')
    parser.add_argument('--progress-eps', type=float, default=0.01,
                        help='Metres of improvement counted as progress (default: 0.01)')
    return parser.parse_args(argv)


def collect_waypoints(args: argparse.Namespace, *, speed_unit: str) -> list[Waypoint]:
    waypoints: list[Waypoint] = []

    if args.waypoints_json:
        waypoints.extend(load_waypoints_from_json(args.waypoints_json))

    if args.waypoints_yaml:
        if yaml is None:
            raise ValueError('PyYAML not installed; cannot parse --waypoints-yaml')
        waypoints.extend(load_waypoints_from_yaml(args.waypoints_yaml))

    if args.waypoint:
        for raw in args.waypoint:
            waypoints.append(Waypoint.from_sequence(raw))

    if not waypoints:
        raise SystemExit('No waypoints provided. Use --waypoint or --waypoints-json.')

    if speed_unit == 'mps':
        for idx, wp in enumerate(waypoints):
            if wp.speed_override is not None:
                waypoints[idx] = Waypoint(
                    x=wp.x,
                    y=wp.y,
                    mode=wp.mode,
                    speed_override=mps_to_rpm(wp.speed_override, args.wheel_diameter),
                )
    return waypoints


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)

    try:
        if args.unit == 'mps':
            default_rpm = mps_to_rpm(args.speed, args.wheel_diameter)
        else:
            default_rpm = float(args.speed)
    except ValueError as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 2

    try:
        wp_list = collect_waypoints(args, speed_unit=args.unit)
    except (ValueError, SystemExit) as exc:
        print(exc, file=sys.stderr)
        return 2

    if args.unit == 'rpm':
        for idx, wp in enumerate(wp_list):
            if wp.speed_override is not None:
                wp_list[idx] = Waypoint(
                    x=wp.x,
                    y=wp.y,
                    mode=wp.mode,
                    speed_override=float(wp.speed_override),
                )

    rclpy.init()
    node = DriveSequence(
        waypoints=wp_list,
        target_rpm=default_rpm,
        goto_tolerance_m=args.goto_tolerance,
        through_tolerance_m=args.through_tolerance,
        odom_topic=args.odom_topic,
        odom_timeout=args.odom_timeout,
        stall_timeout=args.stall_timeout,
        progress_eps=args.progress_eps,
    )
    try:
        success = node.drive_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
