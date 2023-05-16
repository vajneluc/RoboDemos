#!/usr/bin/env python
import rclpy
import time
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from pymoveit2 import MoveIt2Servo
from pymoveit2.robots import panda
from std_srvs.srv import Trigger
from keyboard_msgs.msg import KeyboardState

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class ServoClientNode(Node):

    def __init__(self):
        
        # Create teleop twist keyboard
        super().__init__('teleop_pymoveit2')
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 Servo interface
        self.moveit2_servo = MoveIt2Servo(
            node=self,
            frame_id=panda.base_link_name(),
            callback_group=callback_group,
        )
        print("moveit2 servo initialized")

        # Create joystick subscriber
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create keyboard subscriber
        self.subscription = self.create_subscription(
            KeyboardState,
            '/keyboard_msgs',
            self.keyboard_listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.last_keyboard = KeyboardState()
        self.last_joystick = Joy()
        
        # Default speeds
        self.linear_speed = 1
        self.angular_speed = 2

    def joy_listener_callback(self, msg):
        self.last_joystick = msg
        self.update_command()
        #self.get_logger().info(f'Joystick: {msg}')
    
    def keyboard_listener_callback(self, msg):
        self.last_keyboard = msg
        #self.get_logger().info(f'Keyboard: {msg}')
    
    def send_twist(self, Lx, Ly, Lz, Ax, Ay, Az):
        twist_msg = TwistStamped()
        twist = twist_msg.twist
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "panda_link0"
        twist.linear.x = float(Lx)
        twist.linear.y = float(Ly)
        twist.linear.z = float(Lz)
        twist.angular.x = float(Ax)
        twist.angular.y = float(Ay)
        twist.angular.z = float(Az)
        self.moveit2_servo(linear=(twist.linear.x, twist.linear.y, twist.linear.z), 
                           angular=(twist.angular.x, twist.angular.y, twist.angular.z))
    
    def update_command(self):
        j = self.last_joystick
        k = self.last_keyboard
        self.get_logger().info(f"\nLinear speed: {round(self.linear_speed, 3)} \nAngular speed: {round(self.angular_speed, 3)}")

        # Joystick speed settings
        coefficients = [0.9, 1.1]
        
        # Linear speed
        if j.buttons[0]:
            self.linear_speed *= coefficients[0]
        elif j.buttons[3]:
            self.linear_speed *= coefficients[1]
        
        # Angular speed
        if j.buttons[2]:
            self.angular_speed *= coefficients[0]
        elif j.buttons[1]:
            self.angular_speed *= coefficients[1]

        # Initialize twist linear and angular values
        Lx = 0
        Ly = 0
        Lz = 0
        Ax = 0
        Ay = 0
        Az = 0
        
        # Joystick bindings
        # twist.linear
        joy_x = j.axes[4]
        joy_y = j.axes[3]
        joy_z = j.axes[1]
            
        # twist.angular
        joy_ax = j.axes[6]
        joy_ay = j.axes[7]
        joy_az = j.axes[0]

        # Tolerance settings for joystick bindings
        tolerance = 0.2
        # X Axis
        if abs(joy_x) >= tolerance:
            Lx = joy_x * self.linear_speed
        # Y Axis
        if abs(joy_y) >= tolerance:
            Ly = joy_y * self.linear_speed
        # Z Axis
        if abs(joy_z) >= tolerance:
            Lz = joy_z * self.linear_speed
        # Z Axis turn
        if abs(joy_az) >= tolerance:
            Az = joy_az * self.angular_speed

        # Controler button settings
        # Z Axis turn
        Ax = - joy_ax * self.angular_speed
        # Y Axis turn
        Ay = joy_ay * self.angular_speed
        
        # If no input from joystick, use keyboard
        if Lx == Ly == Lz == Ax == Ay == Az == 0:
            print("No input from Joystick, using Keyboard!")
            
            # Keyboard speed settings
            # Linear speed
            if k.key_k:
                self.linear_speed *= coefficients[0]
            elif k.key_i:
                self.linear_speed *= coefficients[1]
            
            # Angular speed
            if k.key_l:
                self.angular_speed *= coefficients[0]
            elif k.key_o:
                self.angular_speed *= coefficients[1]

            # Keyboard bindings
            # twist.linear
            key_x = 0
            key_y = 0
            key_z = 0

            # twist angular
            key_ax = 0
            key_ay = 0
            key_az = 0

            # X Axis
            if k.key_w:
                key_x = 1
            elif k.key_s:
                key_x = -1
            
            # Y Axis
            if k.key_a:
                key_y = 1
            elif k.key_d:
                key_y = -1
            
            # Z Axis
            if k.key_shift:
                key_z = 1
            elif k.key_ctrl:
                key_z = -1
            
            # X Axis turn
            if k.key_right:
                key_ax = 1
            elif k.key_left:
                key_ax = -1

            # Y Axis turn
            if k.key_up:
                key_ay = 1
            elif k.key_down:
                key_ay = -1

            # Z Axis turn
            if k.key_q:
                key_az = 1
            elif k.key_e:
                key_az = -1

            Lx = key_x * self.linear_speed
            Ly = key_y * self.linear_speed
            Lz = key_z * self.linear_speed
            Ax = key_ax * self.angular_speed
            Ay = key_ay * self.angular_speed
            Az = key_az * self.angular_speed

        self.send_twist(Lx, Ly, Lz, Ax, Ay, Az)







msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

def main():	
    rclpy.init()
    node = ServoClientNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


    