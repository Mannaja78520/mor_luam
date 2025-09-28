#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32, Float32

def wrap360(a): return a % 360.0 if a >= 0.0 else (a % 360.0 + 360.0)
def ang_err(target, current):
    return (wrap360(target) - wrap360(current) + 540.0) % 360.0 - 180.0

class Phase(Enum):
    IDLE = 0
    STEER_TO_HEADING = 1
    DRIVE_TO_GOAL = 2
    DONE = 3

class RobotMovement(Node):
    def __init__(self):
        super().__init__('robot_movement_control_node')

        # ---------- Parameters ----------
        p = self.declare_parameter
        p('cmd_goal_xy_topic', '/mor_luam/cmd_move_goal_xy')
        p('cmd_out_topic',     '/mor_luam/cmd_vel/move')
        p('ticks_topic',       '/mor_luam/feedback/drive_ticks')
        p('steer_deg_topic',   '/mor_luam/feedback/steer_deg')
        p('imu_yaw_deg_topic', '/mor_luam/feedback/imu_yaw_deg')

        p('timer_period', 0.03)              # s
        p('pos_epsilon_m', 0.02)             # goal radius
        p('steer_tolerance_deg', 2.0)        # เข้ากรอบสเตียร์
        p('steer_settle_time', 0.10)         # ต้องอยู่ในกรอบต่อเนื่องก่อนปล่อยวิ่ง
        p('steer_leak_ticks_thr', 5)         # ตัด ticks เล็กรั่วตอนเลี้ยว
        p('keep_going_tol_deg', 5.0)         # yaw error ที่ยอมให้วิ่งต่อ
        p('wheel_radius_m', 0.05)
        p('ticks_per_rev', 64.0)
        p('gear_ratio', 7.0)
        p('max_rpm', 120.0)
        p('rpm_ramp_per_sec', 300.0)

        # สำคัญ: ทิศหมุนของ "มอเตอร์เดียว"
        p('steer_motor_dir', +1)   # +1 = RPM > 0 คือโหมดสเตียร์
        p('drive_motor_dir', -1)   # -1 = RPM < 0 คือโหมดวิ่งหน้า

        g = self.get_parameter
        self.cmd_goal_xy_topic = g('cmd_goal_xy_topic').value
        self.cmd_out_topic     = g('cmd_out_topic').value
        self.ticks_topic       = g('ticks_topic').value
        self.steer_deg_topic   = g('steer_deg_topic').value
        self.imu_yaw_deg_topic = g('imu_yaw_deg_topic').value

        self.dt                   = float(g('timer_period').value)
        self.pos_eps              = float(g('pos_epsilon_m').value)
        self.steer_tol            = float(g('steer_tolerance_deg').value)
        self.steer_settle_time    = float(g('steer_settle_time').value)
        self.steer_leak_ticks_thr = int(g('steer_leak_ticks_thr').value)
        self.keep_going_tol       = float(g('keep_going_tol_deg').value)
        self.wheel_radius_m       = float(g('wheel_radius_m').value)
        self.ticks_per_rev        = float(g('ticks_per_rev').value) * float(g('gear_ratio').value)
        self.max_rpm              = float(g('max_rpm').value)
        self.rpm_ramp_rate        = float(g('rpm_ramp_per_sec').value)
        self.steer_motor_dir      = int(g('steer_motor_dir').value)
        self.drive_motor_dir      = int(g('drive_motor_dir').value)

        self.wheel_circum_m = 2.0 * math.pi * self.wheel_radius_m

        # ---------- Pub/Sub ----------
        self.pub_cmd = self.create_publisher(Twist, self.cmd_out_topic, qos.qos_profile_system_default)
        self.sub_goal = self.create_subscription(Point, self.cmd_goal_xy_topic, self.on_goal, qos.qos_profile_system_default)
        self.sub_ticks = self.create_subscription(Int32, self.ticks_topic, self.on_ticks, qos.qos_profile_sensor_data)
        self.sub_steer = self.create_subscription(Float32, self.steer_deg_topic, self.on_steer, qos.qos_profile_sensor_data)
        self.sub_imu   = self.create_subscription(Float32, self.imu_yaw_deg_topic, self.on_imu, qos.qos_profile_sensor_data)

        # ---------- State ----------
        self.phase = Phase.IDLE
        self.goal_x = 0.0; self.goal_y = 0.0; self.base_speed = 0.0  # m/s in Point.z

        self.curr_ticks = 0
        self.have_tick0 = False
        self.prev_ticks = 0
        self.x = 0.0; self.y = 0.0

        self.yaw = 0.0         # IMU 0..360
        self.steer_deg = 0.0   # AS5600 0..360

        self.target_heading = 0.0
        self.steer_settle_acc = 0.0
        self.steer_start_ticks = 0

        self.desired_rpm = 0.0
        self.actual_rpm  = 0.0
        self.desired_steer = 0.0

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info("robot_movement_control_node (single-motor steer/drive) ready.")

    # ---------- Callbacks ----------
    def on_goal(self, msg: Point):
        self.goal_x = float(msg.x)
        self.goal_y = float(msg.y)
        self.base_speed = max(0.0, float(msg.z))  # m/s

        # reset odom from start
        self.x = 0.0; self.y = 0.0
        self.have_tick0 = False
        self.desired_rpm = 0.0; self.actual_rpm = 0.0

        # ตั้ง phase แรก = หันล้อไปยัง heading เป้าหมาย
        self.target_heading = wrap360(math.degrees(math.atan2(self.goal_y, self.goal_x)))
        self.desired_steer  = self.target_heading
        self.phase = Phase.STEER_TO_HEADING
        self.steer_settle_acc = 0.0
        self.steer_start_ticks = self.curr_ticks
        self.get_logger().info(f"[GOAL] XY=({self.goal_x:.2f},{self.goal_y:.2f})m, v={self.base_speed:.2f} m/s, heading={self.target_heading:.1f}°")

    def on_ticks(self, msg: Int32):  self.curr_ticks = int(msg.data)
    def on_steer(self, msg: Float32): self.steer_deg = wrap360(float(msg.data))
    def on_imu(self, msg: Float32):   self.yaw = wrap360(float(msg.data))

    # ---------- Helpers ----------
    def rpm_from_mps(self, v):
        return max(-self.max_rpm, min(self.max_rpm, (v / self.wheel_circum_m) * 60.0))

    def ramp(self, curr, target):
        step = self.rpm_ramp_rate * self.dt
        if target > curr: return min(target, curr + step)
        else:             return max(target, curr - step)

    def integrate_odom_if_driving(self, driving: bool):
        if not self.have_tick0:
            self.prev_ticks = self.curr_ticks
            self.have_tick0 = True
            return
        dticks = self.curr_ticks - self.prev_ticks
        self.prev_ticks = self.curr_ticks
        if not driving:
            # โหมดเลี้ยว: ตัดการเคลื่อนที่ (ละทิ้ง ticks เล็ก ๆ ที่ไหล)
            if abs(dticks) > self.steer_leak_ticks_thr:
                # รีเซ็ต baseline ไม่ให้เล็ดลอดไปสะสม
                self.prev_ticks = self.curr_ticks
            return
        # โหมดวิ่ง: อินทิเกรตระยะ
        ds = (dticks / self.ticks_per_rev) * self.wheel_circum_m
        self.x += ds * math.cos(math.radians(self.yaw))
        self.y += ds * math.sin(math.radians(self.yaw))

    # ---------- Main Loop ----------
    def loop(self):
        # default
        out_rpm = 0.0
        out_steer = self.desired_steer

        if self.phase == Phase.IDLE:
            self.integrate_odom_if_driving(False)

        elif self.phase == Phase.STEER_TO_HEADING:
            # คุมเฉพาะสเตียร์: สั่งทิศ "สเตียร์" เท่านั้น
            self.desired_steer = self.target_heading
            out_steer = self.desired_steer

            err = ang_err(self.desired_steer, self.steer_deg)
            if abs(err) > self.steer_tol:
                # หมุนมอเตอร์ทิศสเตียร์ (เช่น RPM > 0 = ขวา)
                out_rpm = self.steer_motor_dir * min(self.max_rpm, 0.5 * self.max_rpm)  # สปีดเลี้ยวครึ่งหนึ่งของ max
                self.steer_settle_acc = 0.0
                # ไม่อินทิเกรตระยะ
                self.integrate_odom_if_driving(False)
            else:
                # เข้ากรอบแล้ว → รอ settle time
                self.steer_settle_acc += self.dt
                out_rpm = 0.0
                self.integrate_odom_if_driving(False)
                if self.steer_settle_acc >= self.steer_settle_time:
                    # เริ่มวิ่งหน้า
                    self.phase = Phase.DRIVE_TO_GOAL
                    self.get_logger().info("[STEER→DRIVE] steering settled")

        elif self.phase == Phase.DRIVE_TO_GOAL:
            # อัปเดต heading เป้าหมายไปยังเวกเตอร์จากตำแหน่งปัจจุบัน → เป้าหมาย
            dx = self.goal_x - self.x
            dy = self.goal_y - self.y
            dist = math.hypot(dx, dy)

            if dist <= self.pos_eps:
                self.phase = Phase.DONE
                out_rpm = 0.0
                self.integrate_odom_if_driving(False)
            else:
                self.target_heading = wrap360(math.degrees(math.atan2(dy, dx)))
                self.desired_steer = self.target_heading
                out_steer = self.desired_steer

                # ถ้ามุมล้อยังไม่เข้า → กลับไปเลี้ยว
                steer_error = ang_err(self.desired_steer, self.steer_deg)
                if abs(steer_error) > self.steer_tol:
                    self.phase = Phase.STEER_TO_HEADING
                    self.steer_settle_acc = 0.0
                    self.steer_start_ticks = self.curr_ticks
                    out_rpm = 0.0
                    self.integrate_odom_if_driving(False)
                else:
                    # วิ่งหน้า: ทิศหมุนที่ใช้ "เดินหน้า" เท่านั้น
                    # scale ความเร็วตาม yaw error (หัวเบี่ยงมาก → ช้าลง)
                    yaw_err = ang_err(self.target_heading, self.yaw)
                    if abs(yaw_err) > self.keep_going_tol:
                        speed = 0.0
                    else:
                        scale = math.cos(abs(yaw_err) / self.keep_going_tol * (math.pi / 2.0))
                        speed = self.base_speed * max(0.0, min(1.0, scale))
                    # เบรกก่อนถึง
                    speed *= min(1.0, dist / (5.0 * self.pos_eps))

                    out_rpm = self.drive_motor_dir * self.rpm_from_mps(speed)
                    self.integrate_odom_if_driving(True)

        elif self.phase == Phase.DONE:
            out_rpm = 0.0
            self.integrate_odom_if_driving(False)
            self.phase = Phase.IDLE

        # ramp และส่งคำสั่ง
        self.desired_rpm = out_rpm
        self.actual_rpm = self.ramp(self.actual_rpm, self.desired_rpm)

        cmd = Twist()
        cmd.linear.x  = float(self.actual_rpm)           # RPM (signed)
        cmd.angular.z = float(wrap360(self.desired_steer))  # deg
        self.pub_cmd.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(RobotMovement())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
