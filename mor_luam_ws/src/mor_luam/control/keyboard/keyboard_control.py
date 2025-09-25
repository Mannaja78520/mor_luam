#!/usr/bin/env python3

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos
from std_msgs.msg import String


class keyboard_control(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.keyboard_pub = self.create_publisher(
            String, '/keyboard_input', qos_profile=qos.qos_profile_system_default
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', qos_profile=qos.qos_profile_system_default
        )   
        
        self.move_mode_pub = self.create_publisher(
            String, '/movement_mode', qos_profile=qos.qos_profile_system_default
        )   
        
        self.maxlinear_speed = 10.0  # m/s max
        self.maxpwm_speed = 1023.0  # PWM max speed
        
        # Declare and get parameters
        self.declare_parameter('max_speed', self.maxlinear_speed) # Maximum speed in PWM
        self.declare_parameter('plus_move_speed', 0.25)
        self.declare_parameter('plus_slide_speed', 0.25)
        self.declare_parameter('plus_turn_speed', 0.5)
        self.declare_parameter('plus_speed_size', 0.01)

        self.maxSpeed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.plusMoveSpeed = self.get_parameter('plus_move_speed').get_parameter_value().double_value
        self.plusSlideSpeed = self.get_parameter('plus_slide_speed').get_parameter_value().double_value
        self.plusturnSpeed = self.get_parameter('plus_turn_speed').get_parameter_value().double_value
        self.plusSpeedSize = self.get_parameter('plus_speed_size').get_parameter_value().double_value

        self.moveSpeed: float = 0.0
        self.slideSpeed: float = 0.0
        self.turnSpeed: float = 0.0
        
        
        self.show_log()
        self.get_logger().info("Keyboard publisher started. Press keys to send messages. Press 'p' to quit.")


    def clip(self, value : float, min_val : float, max_val : float) -> float:
        return max(min(value, max_val), min_val)

    def show_log(self):
        log_message = (
            "Use the following keys to control the robot: \n"
            "'W' : Increase move speed, \n"
            "'S' : Decrease move speed, \n"
            "'a' : Slide left, \n"
            "'d' : Slide right, \n"
            "'q' : Turn left, \n"
            "'e' : Turn right, \n"
            "'y' : Increase move speed increment. \n"
            "'h' : Decrease move speed increment. \n"
            "'u' : Increase slide speed increment. \n"
            "'j' : Decrease slide speed increment. \n"
            "'i' : Increase turn speed increment. \n"
            "'k' : Decrease turn speed increment. \n"
            f"'m' : Set movement mode to manual (max speed {self.maxpwm_speed}), \n"
            f"'r' : Set movement mode to mps (max speed {self.maxlinear_speed}), \n"
            "' ' : Brake. \n"
            "Keyboard publisher started. Press keys to send messages. Press 'p' to quit.\n"
            f"Current mode: {'manual' if self.maxSpeed == self.maxpwm_speed else 'mps'}\n"
            f"Current Speeds: Move={self.moveSpeed:.2f}, Slide={self.slideSpeed:.2f}, Turn={self.turnSpeed:.2f}\n"
            f"Current Speed increment: Move={self.plusMoveSpeed:.2f}, Slide={self.plusSlideSpeed:.2f}, Turn={self.plusturnSpeed:.2f}"
        )

        # Log the combined message
        self.get_logger().info(log_message)
        
    def update_speeds(self, key):
        # Adjust movement, sliding, and turning speeds
        self.moveSpeed = self.moveSpeed + self.plusMoveSpeed if key == 'w' else self.moveSpeed - self.plusMoveSpeed if key == 's' else self.moveSpeed
        self.slideSpeed = self.slideSpeed + self.plusSlideSpeed if key == 'd' else self.slideSpeed - self.plusSlideSpeed if key == 'a' else self.slideSpeed
        self.turnSpeed = self.turnSpeed + self.plusturnSpeed if key == 'q' else self.turnSpeed - self.plusturnSpeed if key == 'e' else self.turnSpeed

        self.plusMoveSpeed = self.plusMoveSpeed + self.plusSpeedSize if key == 'W' else self.plusMoveSpeed - self.plusSpeedSize if key == 'S' else self.plusMoveSpeed
        self.plusSlideSpeed = self.plusSlideSpeed + self.plusSpeedSize if key == 'D' else self.plusSlideSpeed - self.plusSpeedSize if key == 'A' else self.plusSlideSpeed
        self.plusturnSpeed = self.plusturnSpeed + self.plusSpeedSize if key == 'Q' else self.plusturnSpeed - self.plusSpeedSize if key == 'E' else self.plusturnSpeed

        if key == ' ':
            self.moveSpeed = 0.0
            self.slideSpeed = 0.0
            self.turnSpeed = 0.0

        # Clip values
        self.moveSpeed = self.clip(self.moveSpeed, -self.maxSpeed, self.maxSpeed)
        self.slideSpeed = self.clip(self.slideSpeed, -self.maxSpeed, self.maxSpeed)
        self.turnSpeed = self.clip(self.turnSpeed, -self.maxSpeed, self.maxSpeed)
        self.plusMoveSpeed = self.clip(self.plusMoveSpeed, 0, self.maxSpeed)
        self.plusSlideSpeed = self.clip(self.plusSlideSpeed, 0, self.maxSpeed)
        self.plusturnSpeed = self.clip(self.plusturnSpeed, 0, self.maxSpeed)
        
    def update_mode(self, key):
        """Updates the movement mode based on the key pressed."""
        if key == 'm':
            mode_msg = String()
            mode_msg.data = "manual"
            self.maxSpeed = self.maxpwm_speed
            self.move_mode_pub.publish(mode_msg)
            self.get_logger().info(f"Movement mode set to manual max speed is {self.maxpwm_speed}.")
        elif key == 'r':
            mode_msg = String()
            mode_msg.data = "mps"
            self.maxSpeed = self.maxlinear_speed
            self.move_mode_pub.publish(mode_msg)
            self.get_logger().info(f"Movement mode set to autonomous max speed is {self.maxlinear_speed}.")
        
    def get_key(self):
        """Reads a single key without waiting for Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            self.show_log()
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        """Continuously listens for keypresses and publishes them."""
        while rclpy.ok():
            key = self.get_key()
            if key == 'p':  # Exit condition
                break
            self.update_speeds(key)
            self.publish_message(key)
            self.update_mode(key)

    def publish_message(self, key):
        keyboard_msg = String()
        keyboard_msg.data = key
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x    = float(self.moveSpeed)
        cmd_vel_msg.linear.y    = float(self.slideSpeed)
        cmd_vel_msg.angular.z   = float(self.turnSpeed)
        
        self.keyboard_pub.publish(keyboard_msg)
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main():
    rclpy.init()
    node = keyboard_control()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()