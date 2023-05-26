#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from pymoveit2 import MoveIt2Servo
from pymoveit2.robots import panda
from keyboard_msgs.msg import KeyboardState


class TeleopNode(Node):
    def __init__(self):
        # Create Teleop Node
        super().__init__("teleop_node")

        # Declare launch parameter to start joystick controller (works with generic controller in Linux)
        self.declare_parameter("start_joy", True)
        self.start_joy = self.get_parameter("start_joy").get_parameter_value().bool_value

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Default speed settings
        self.linear_speed = 2
        self.angular_speed = 4

        # Create MoveIt2 Servo
        self.moveit2_servo = MoveIt2Servo(
            node=self,
            frame_id=panda.base_link_name(),
            callback_group=callback_group,
        )
        if self.start_joy:
            # Create /joy subsriber
            self.subscription = self.create_subscription(
                Joy, "/joy", self.joy_listener_callback, 10
            )
            self.subscription  # prevent unused variable warning

        # Create /keyboard_msgs subsriber
        self.subscription = self.create_subscription(
            KeyboardState, "/keyboard_msgs", self.keyboard_listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        
        
        self.last_keyboard = KeyboardState()

        if self.start_joy:
            self.last_joystick = Joy()

        

    def joy_listener_callback(self, msg):
        if self.start_joy:
            self.last_joystick = msg
            self.update_command()

    def keyboard_listener_callback(self, msg):
        if not self.start_joy:
            self.last_keyboard = msg         
            self.update_command()
            
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
        self.moveit2_servo(
            linear=(twist.linear.x, twist.linear.y, twist.linear.z),
            angular=(twist.angular.x, twist.angular.y, twist.angular.z),
        )

    def update_command(self):
        if self.start_joy:
            j = self.last_joystick
        
        k = self.last_keyboard
        self.get_logger().info(
            f"\nLinear speed: {round(self.linear_speed, 3)} \nAngular speed: {round(self.angular_speed, 3)}"
        )

        # Joystick speed settings
        coefficients = [0.9, 1.1]

        if self.start_joy:
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
            Ax = -joy_ax * self.angular_speed
            # Y Axis turn
            Ay = joy_ay * self.angular_speed

            # If no input from joystick, use keyboard
            if Lx == Ly == Lz == Ax == Ay == Az == 0:
                print("No input from Joystick, using Keyboard!")

                # Keyboard speed settings
                # Linear speed
                if k.key_r:
                    self.linear_speed *= coefficients[1]
                elif k.key_f:
                    self.linear_speed *= coefficients[0]

                # Angular speed
                if k.key_y:
                    self.angular_speed *= coefficients[1]
                elif k.key_h:
                    self.angular_speed *= coefficients[0]

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
                if k.key_q:
                    key_z = 1
                elif k.key_e:
                    key_z = -1

                # X Axis turn
                if k.key_l:
                    key_ax = 1
                elif k.key_j:
                    key_ax = -1

                # Y Axis turn
                if k.key_i:
                    key_ay = 1
                elif k.key_k:
                    key_ay = -1

                # Z Axis turn
                if k.key_u:
                    key_az = 1
                elif k.key_o:
                    key_az = -1

                Lx = key_x * self.linear_speed
                Ly = key_y * self.linear_speed
                Lz = key_z * self.linear_speed
                Ax = key_ax * self.angular_speed
                Ay = key_ay * self.angular_speed
                Az = key_az * self.angular_speed

            self.send_twist(Lx, Ly, Lz, Ax, Ay, Az)
        
        elif not self.start_joy:
            # Keyboard speed settings
            # Linear speed
            if k.key_r:
                self.linear_speed *= coefficients[1]
            elif k.key_f:
                self.linear_speed *= coefficients[0]

            # Angular speed
            if k.key_y:
                self.angular_speed *= coefficients[1]
            elif k.key_h:
                self.angular_speed *= coefficients[0]

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
            if k.key_q:
                key_z = 1
            elif k.key_e:
                key_z = -1

            # X Axis turn
            if k.key_l:
                key_ax = 1
            elif k.key_j:
                key_ax = -1

            # Y Axis turn
            if k.key_i:
                key_ay = 1
            elif k.key_k:
                key_ay = -1

            # Z Axis turn
            if k.key_u:
                key_az = 1
            elif k.key_o:
                key_az = -1

            Lx = key_x * self.linear_speed
            Ly = key_y * self.linear_speed
            Lz = key_z * self.linear_speed
            Ax = key_ax * self.angular_speed
            Ay = key_ay * self.angular_speed
            Az = key_az * self.angular_speed

            self.send_twist(Lx, Ly, Lz, Ax, Ay, Az)

def main():
    rclpy.init()
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
