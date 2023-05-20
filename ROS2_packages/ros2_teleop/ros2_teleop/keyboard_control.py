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
from keyboard_msgs.msg import KeyboardState
#from pynput import keyboard
#from pynput.keyboard import Key
from sshkeyboard import listen_keyboard

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

class KeyboardControlNode(Node):

    def __init__(self):
        
        # Create teleop twist keyboard
        super().__init__('keyboard_control')

        self.pressed_keys = {
            "q" : False,
            "w" : False,
            "e" : False,
            "a" : False,
            "s" : False,
            "d" : False,
            "r" : False,
            "f" : False,
            "u" : False,
            "i" : False,
            "o" : False,
            "j" : False,
            "k" : False,
            "l" : False,
            "y" : False,
            "h" : False,
        }

        self.publisher = self.create_publisher(
            KeyboardState,
            "keyboard_msgs",
            10
        )

    def on_press(key, self):
        print("on press:", key)
        lookup = key.lower()
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup]=True
            self.send_keyboard_state()
    
    def on_release(key, self):
        print("on realease:", key)
        lookup = key.lower()
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup]=False
            self.send_keyboard_state()

    print("Listening to keyboard events...")
    listen_keyboard(on_press=on_press, on_release=on_release)
    

    def send_keyboard_state(self):
        msg = KeyboardState()
        msg.key_q = self.pressed_keys["q"]
        msg.key_w = self.pressed_keys["w"]
        msg.key_e = self.pressed_keys["e"]
        msg.key_a = self.pressed_keys["a"]
        msg.key_s = self.pressed_keys["s"]
        msg.key_d = self.pressed_keys["d"]
        msg.key_r = self.pressed_keys["r"]
        msg.key_f = self.pressed_keys["f"]
        msg.key_u = self.pressed_keys["u"]
        msg.key_i = self.pressed_keys["i"]
        msg.key_o = self.pressed_keys["o"]
        msg.key_j = self.pressed_keys["j"]
        msg.key_k = self.pressed_keys["k"]
        msg.key_l = self.pressed_keys["l"]
        msg.key_y = self.pressed_keys["y"]
        msg.key_h = self.pressed_keys["h"]

        self.publisher.publish(msg) 

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    

def main():	
    print("stage 1")
    rclpy.init()
    print("stage 2")
    node = KeyboardControlNode()
    print("Starting keyboard state publisher")
    rclpy.spin(node)
    rclpy.shutdown()


    