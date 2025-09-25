#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray , Float32MultiArray
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy
from rclpy import qos
import time

class Gamepad:
    def __init__(self):
        #Axes:--------------------------------------------------------
        
        self.lx : float = 0.0                   # 0: Left X-Axis
        self.ly : float = 0.0                   # 1: Left Y-Axis
        self.l2 : float = 0.0                   # 2: L2
        self.rx : float = 0.0                   # 3: Right X-Axis
        self.ry : float = 0.0                   # 4: Right Y-Axis
        self.r2 : float = 0.0                   # 5: R2
        self.dpadLeftRight : float = 0.0        # 6: Dpad Left and Right
        self.dpadUpDown : float = 0.0           # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------
        
        self.button_cross : float = 0.0         # 0: 
        self.button_circle : float = 0.0        # 1:
        self.button_triangle : float = 0.0      # 2:
        self.button_square : float = 0.0        # 3:
        self.l1 : float = 0.0                   # 4:
        self.r1 : float = 0.0                   # 5:
        #self.l2_logic : float = 0.0                   # 6:
        #self.r2_logic : float = 0.0                   # 7:
        self.button_share : float = 0.0         # 8:
        self.button_option : float = 0.0        # 9:
        self.button_logo : float = 0.0          # 10:
        self.PressedLeftAnalog : float = 0.0    # 11:
        self.PressedRightAnalog : float = 0.0   # 12:

        #----------------------------------------------------------------
        
        # self.auto_aim_bool: bool =False
        # self.dribble: bool = False 
        # self.toggle_shoot_bool : bool = False 
        # self.toggle_pass_bool : bool = False
        # self.toggle_pass_motor_bool : bool = False
        # self.toggle_shoot_2_bool : bool = False
        # self.toggle_encoder_bool : bool = False

        self.previous_triangle_state = False
        self.previous_circle_state = False
        self.previous_cross_state = False
        self.previous_square_state = False
        self.previous_l1_state = False
        self.previous_r1_state = False
        self.previous_PressedRightAnalog_state = False



        self.last_macro_button = None  # Stores 'shoot', 'pass', 'dribble', 'auto_aim', 'pass_motor', 'shoot2'



        

        


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick_control_node")

        # === Declare and get parameters (### ADD) ===
        self.declare_parameter('cmd_vel_topic', '/mor_luam/cmd_vel')
        self.declare_parameter('mode_topic', '/mor_luam/move_mode')
        self.declare_parameter('timer_period', 0.05)
        self.declare_parameter('maxlinear_speed', 0.25)
        self.declare_parameter('maxpwm_speed', 1023.0)
        self.declare_parameter('default_mode', 'mps')   # 'mps' or 'manual'
        self.declare_parameter('invert_lx', True)
        self.declare_parameter('invert_ly', False)
        self.declare_parameter('invert_rx', True)
        self.declare_parameter('invert_ry', False)
        self.declare_parameter('scale_lx', 1.0)
        self.declare_parameter('scale_ly', 1.0)
        self.declare_parameter('scale_rx', 1.0)
        self.declare_parameter('scale_ry', 1.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        mode_topic    = self.get_parameter('mode_topic').get_parameter_value().string_value
        timer_period  = self.get_parameter('timer_period').get_parameter_value().double_value
        self.maxlinear_speed = float(self.get_parameter('maxlinear_speed').value)
        self.maxpwm_speed    = float(self.get_parameter('maxpwm_speed').value)

        self.invert_lx = bool(self.get_parameter('invert_lx').value)
        self.invert_ly = bool(self.get_parameter('invert_ly').value)
        self.invert_rx = bool(self.get_parameter('invert_rx').value)
        self.invert_ry = bool(self.get_parameter('invert_ry').value)
        self.scale_lx  = float(self.get_parameter('scale_lx').value)
        self.scale_ly  = float(self.get_parameter('scale_ly').value)
        self.scale_rx  = float(self.get_parameter('scale_rx').value)
        self.scale_ry  = float(self.get_parameter('scale_ry').value)

        default_mode  = self.get_parameter('default_mode').get_parameter_value().string_value

        # === Publishers/Subscribers ===
        self.pub_move = self.create_publisher(Twist, cmd_vel_topic, qos_profile=qos.qos_profile_system_default)
        self.move_mode_pub = self.create_publisher(String, mode_topic, qos_profile=qos.qos_profile_system_default)

        self.create_subscription(Joy, '/joy', self.joy, qos_profile=qos.qos_profile_sensor_data)

        self.gamepad = Gamepad()

        # use one canonical name: self.maxspeed
        if default_mode == 'manual':
            self.maxspeed = self.maxpwm_speed
        else:
            self.maxspeed = self.maxlinear_speed

        # init speeds
        self.moveSpeed = 0.0
        self.slideSpeed = 0.0
        self.turnSpeed = 0.0

        self.sent_data_timer = self.create_timer(timer_period, self.sendData)

    def update_mode(self):
        """Updates the movement mode based on the joy pressed."""
        if self.gamepad.button_circle == 1.0:
            mode_msg = String()
            mode_msg.data = "manual"
            self.maxspeed = self.maxpwm_speed
            self.move_mode_pub.publish(mode_msg)
            self.get_logger().info(f"Movement mode set to manual max speed is {self.maxpwm_speed}.")
        elif self.gamepad.button_square == 1.0:
            mode_msg = String()
            mode_msg.data = "mps"
            self.maxspeed = self.maxlinear_speed
            self.move_mode_pub.publish(mode_msg)
            self.get_logger().info(f"Movement mode set to autonomous max speed is {self.maxlinear_speed}.")

    def update_speeds(self):
        # ใช้ parameter invert/scale และใช้ชื่อ self.maxspeed ให้ตรงกัน
        lx = (-1.0 if self.invert_lx else 1.0) * self.gamepad.lx * self.scale_lx
        ly = (-1.0 if self.invert_ly else 1.0) * self.gamepad.ly * self.scale_ly
        rx = (-1.0 if self.invert_rx else 1.0) * self.gamepad.rx * self.scale_rx

        self.moveSpeed  = ly * self.maxspeed
        self.slideSpeed = lx * self.maxspeed
        self.turnSpeed  = rx * self.maxspeed

        self.moveSpeed  = self.clip(self.moveSpeed,  -self.maxspeed, self.maxspeed)
        self.slideSpeed = self.clip(self.slideSpeed, -self.maxspeed, self.maxspeed)
        self.turnSpeed  = self.clip(self.turnSpeed,  -self.maxspeed, self.maxspeed)

        
        

    def joy(self, msg):
        
        #Axes:--------------------------------------------------------
        
        self.gamepad.lx = float(msg.axes[0] * -1)                   # 0: Left X-Axis
        self.gamepad.ly = float(msg.axes[1])                        # 1: Left Y-Axis
        self.gamepad.l2 = float((msg.axes[2] + 1)/ 2)               # 2: L2
        self.gamepad.rx = float(msg.axes[3] * -1)                   # 3: Right X-Axis
        self.gamepad.ry = float(msg.axes[4])                        # 4: Right Y-Axis
        self.gamepad.r2 = float((msg.axes[5] + 1)/ 2)               # 5: R2
        self.gamepad.dpadLeftRight  = float(msg.axes[6])            # 6: Dpad Left and Right
        self.gamepad.dpadUpDown     = float(msg.axes[7])            # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------

        self.gamepad.button_cross    = float(msg.buttons[0])        # 0: 
        self.gamepad.button_circle   = float(msg.buttons[1])        # 1:
        self.gamepad.button_triangle = float(msg.buttons[2])        # 2:
        self.gamepad.button_square   = float(msg.buttons[3])        # 3:
        self.gamepad.l1              = float(msg.buttons[4])        # 4:
        self.gamepad.r1              = float(msg.buttons[5])        # 5:
        #self.gamepad.l2              = float(msg.buttons[6])        # 6:
        #self.gamepad.r2              = float(msg.buttons[7])        # 7:
        self.gamepad.button_share    = float(msg.buttons[8])        # 8:
        self.gamepad.button_option   = float(msg.buttons[9])        # 9:
        self.gamepad.button_logo     = float(msg.buttons[10])       # 10:
        self.gamepad.PressedLeftAnalog  = float(msg.buttons[11])    # 11:
        self.gamepad.PressedRightAnalog = float(msg.buttons[12])    # 12:
        
        self.update_mode()
        

    def sendData(self):
        cmd_vel_move = Twist()
    
        self.update_speeds()
        
        cmd_vel_move.linear.x = float(self.moveSpeed)
        cmd_vel_move.linear.y = float(self.slideSpeed)
        cmd_vel_move.angular.z = float(self.turnSpeed)

        self.pub_move.publish(cmd_vel_move)



def main():
    rclpy.init()

    sub = Joystick()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()