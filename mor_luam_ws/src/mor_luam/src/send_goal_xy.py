#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import argparse

TOPIC = '/mor_luam/cmd_move_goal_xy'

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,   # ผู้รับรันอยู่แล้วก็พอ
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub = self.create_publisher(Point, TOPIC, qos)

    def wait_for_match(self, timeout_sec=2.0):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            if self.pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def send(self, x, y, speed_mps=0.25):
        matched = self.wait_for_match()
        if not matched:
            self.get_logger().warn("No subscriber matched; sending anyway (message may be lost)")
        msg = Point()
        msg.x, msg.y, msg.z = float(x), float(y), float(speed_mps)
        self.pub.publish(msg)
        self.get_logger().info(f'Sent goal: ({msg.x:.2f}, {msg.y:.2f}) @ {msg.z:.2f} m/s')
        rclpy.spin_once(self, timeout_sec=0.1)

def main():
    parser = argparse.ArgumentParser(description="Send goal to robot")
    parser.add_argument("x", type=float, help="X coordinate (m)")
    parser.add_argument("y", type=float, help="Y coordinate (m)")
    parser.add_argument("-s", "--speed", type=float, default=0.25, help="Speed in m/s (default: 0.25)")
    args = parser.parse_args()

    rclpy.init()
    node = GoalSender()
    node.send(args.x, args.y, args.speed)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
