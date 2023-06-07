#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from keyboard_msgs.msg import KeyboardState
from pynput import keyboard
from pynput.keyboard import Key
from sshkeyboard import listen_keyboard


class KeyboardControlNode(Node):
    def __init__(self):
        # Create Keyboard Control Node
        super().__init__("keyboard_control")

        # Declare launch parameter to use pynput(Linux) or sshkeyboard(works with WSL2)
        self.declare_parameter("use_pynput", False)
        use_pynput = self.get_parameter("use_pynput").get_parameter_value().bool_value

        # Registered keys for both keyboards
        self.pressed_keys = {
            "q": False,
            "w": False,
            "e": False,
            "a": False,
            "s": False,
            "d": False,
            "r": False,
            "f": False,
            "u": False,
            "i": False,
            "o": False,
            "j": False,
            "k": False,
            "l": False,
            "y": False,
            "h": False,
            "n": False,
            "c": False,
        }

        # Create keyboard_msgs publisher
        self.publisher = self.create_publisher(KeyboardState, "keyboard_msgs", 10)
        self.timer = self.create_timer(0.05, self.send_keyboard_state)

        if use_pynput:
            self.get_logger().info("Using Pynput keyboard.")

            with keyboard.Listener(
                on_press=self.on_press_pyn, on_release=self.on_release_pyn
            ) as listener:
                listener.join()
        else:
            self.get_logger().info("Using Sshkeyboard")
            listen_keyboard(
                on_press=lambda key: self.on_press(key),
                on_release=lambda key: self.on_release(key),
            )

    # Callback functions for sshkeyboard on press
    def on_press(self, key):
        lookup = key.lower()
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup] = True
            self.send_keyboard_state()

    # Callback functions for sshkeyboard on release
    def on_release(self, key):
        lookup = key.lower()
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup] = False
            self.send_keyboard_state()

    # Callback functions for pynput keyboard on press
    def on_press_pyn(self, key):
        try:
            lookup = key.char.lower()
        except AttributeError:
            lookup = key
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup] = True
            self.send_keyboard_state()

    # Callback functions for pynput keyboard on release
    def on_release_pyn(self, key):
        try:
            lookup = key.char.lower()
        except AttributeError:
            lookup = key
        if lookup in self.pressed_keys:
            self.pressed_keys[lookup] = False
            self.send_keyboard_state()

    # Function publishing keyboard_msg
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
        msg.key_n = self.pressed_keys["n"]
        msg.key_c = self.pressed_keys["c"]
        # publish the keyboard_msg
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = KeyboardControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
