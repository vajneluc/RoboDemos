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
from pynput import keyboard
from pynput.keyboard import Key
from sshkeyboard import listen_keyboard



class KeyboardControlNode(Node):

    def __init__(self):
        
        
        # Create teleop twist keyboard
        super().__init__('keyboard_control')

        self.declare_parameter('use_pynput', True)
        use_pynput = self.get_parameter('use_pynput').get_parameter_value().bool_value
        print(use_pynput)
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
            
        if use_pynput:
            print("Using pynput keyboard.")
            print("Listening to keyboard events...")
            with keyboard.Listener(on_press=self.on_press_pyn,
                                   on_release=self.on_release_pyn) as listener:
                                   listener.join()
        else:
            print("Using sshkeyboard.")
            print("Listening to keyboard events...")
            listen_keyboard(on_press=lambda key: self.on_press(key), 
                            on_release=lambda key: self.on_release(key))

    # Callback functions for sshkeyboard:
    def on_press(self, key):
        print("on press:", key)
        lookup = key.lower()
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup]=True
            self.send_keyboard_state()
    
    def on_release(self, key):
        print("on release:", key)
        lookup = key.lower()
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup]=False
            self.send_keyboard_state()
    
    # Callback functions for pynput keyboard
    def on_press_pyn(self, key):
        try:
            print("on press pyn:", key)
            lookup = key.char.lower()
        except AttributeError:    
             lookup = key
        if lookup in self.pressed_keys:
            print("lookup found:", lookup)
            self.pressed_keys[lookup]=True
            self.send_keyboard_state()
    
    def on_release_pyn(self, key):
        try:
            print("on release pyn:", key)  
            lookup = key.char.lower()
        except AttributeError:
             lookup = key    
        if lookup in self.pressed_keys:
            print("lookup found:", lookup)
            self.pressed_keys[lookup]=False
            self.send_keyboard_state()
    
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
    rclpy.init()
    node = KeyboardControlNode()
    print("Starting keyboard state publisher")
    rclpy.spin(node)
    rclpy.shutdown()


    