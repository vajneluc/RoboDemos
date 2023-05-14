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
from pynput import keyboard
from pynput.keyboard import Key

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class KeyboardControlNode(Node):

    def __init__(self):
        
        # Create teleop twist keyboard
        super().__init__('teleop_twist_keyboard')

        self.pressed_keys = {
            Key.esc : False,
            Key.enter : False,
            Key.shift : False,
            Key.ctrl : False,
            "q" : False,
            "w" : False,
            "e" : False,
            "a" : False,
            "s" : False,
            "d" : False,
            "z" : False,
            "x" : False,
            "c" : False,
            Key.up : False,
            Key.down : False,
            Key.right : False,
            Key.left : False,
        }

        self.publisher = self.create_publisher(
            KeyboardState,
            "keyboard_msgs",
            10
        )

        print("Listening to keyboartd events...")
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        try:
            # alphanumerical characters
            lookup = key.char.lower()
        except AttributeError:
            # special characters
            lookup = key

        if lookup in self.pressed_keys:
            self.pressed_keys[lookup]=True
            self.send_keyboard_state()

    def on_release(self, key):
        try:
            # alphanumerical characters
            lookup = key.char.lower()
        except AttributeError:
            # special characters
            lookup = key
            
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup]=False
            self.send_keyboard_state()

    def send_keyboard_state(self):
        msg = KeyboardState()
        msg.key_esc = self.pressed_keys[Key.esc]
        msg.key_enter = self.pressed_keys[Key.enter]
        msg.key_shift = self.pressed_keys[Key.shift]
        msg.key_ctrl = self.pressed_keys[Key.ctrl]
        msg.key_q = self.pressed_keys["q"]
        msg.key_w = self.pressed_keys["w"]
        msg.key_e = self.pressed_keys["e"]
        msg.key_a = self.pressed_keys["a"]
        msg.key_s = self.pressed_keys["s"]
        msg.key_d = self.pressed_keys["d"]
        msg.key_z = self.pressed_keys["z"]
        msg.key_x = self.pressed_keys["x"]
        msg.key_c = self.pressed_keys["c"]
        msg.key_up = self.pressed_keys[Key.up]
        msg.key_down = self.pressed_keys[Key.down]
        msg.key_right = self.pressed_keys[Key.right]
        msg.key_left = self.pressed_keys[Key.left]

        self.publisher.publish(msg) 

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    

def main():	
    rclpy.init()
    node = KeyboardControlNode()

    print("Starting keyboard state publisher")
    rclpy.spin(node)
    
    rclpy.shutdown()


    