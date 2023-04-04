from math import sin, cos, pi
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_msgs.msg import TFMessage
from launch.actions import DeclareLaunchArgument
from rosidl_runtime_py import message_to_ordereddict

from commandr import Commandr


class R2d2PublisherNode(Node):

    def __init__(self):
        rclpy.init()
        super().__init__(f'r2d2_publisher_node')
        #Parameters
        self.declare_parameter("fields", "")
        self.declare_parameter("filterby", "")

        self.fields = self.get_parameter("fields").value.split(",")
        self.filterby = tuple(self.get_parameter("filterby").value.split("="))

        self.get_logger().info(f'Fields: {self.fields}  Filterby: {self.filterby}')
        
        self.publisher_ = self.create_publisher(Vector3, 'tf_computed', 10)

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        """
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        """
        self.nodeName = self.get_name()
        self.get_logger().info(f"{self.nodeName} started!")

    def tf_callback(self, msg):
        # Process the TFMessage data here
        for tf_msg in msg.transforms:
            val = json.loads(json.dumps(message_to_ordereddict(tf_msg)))
            if self.filterby is not None:
                if self.get_field(val, self.filterby[0]) == self.filterby[1]:
                    # field accepted!
                    values = [self.get_field(val, f) for f in self.fields]
                    self.get_logger().info(f'TF message extracted: {values}')

                    msg_to_send = Vector3()
                    msg_to_send.x = values[0]["x"]
                    msg_to_send.y = values[0]["y"]
                    msg_to_send.z = values[0]["z"]

                    self.publisher_.publish(msg_to_send)

                    
    def joint_state_callback(self, msg):
        # Process the JointState data here
        self.get_logger().info(f'Joint state message: {msg}')


    @staticmethod
    def get_field(data, key):
        if key is not None and key != "":
            if key[0] == ".":
                key = key[1:]
            if key[0].isalpha() or key[0]=="_": 
                prefix = ""
                rest = key
                while len(rest ) > 0:
                    c = rest[0]
                    if not c.isalpha() and c != "_":
                        break
                    else:
                        prefix += c
                        rest = rest[1:]
                return R2d2PublisherNode.get_field(data[prefix], rest)
            elif key[0] == "[":
                prefix, rest = key[1:].split("]")
                index = int(prefix)
                return R2d2PublisherNode.get_field(data[index], rest)
            else:
                print("# Error: unrecognized prefix")
        return data


def main():
    my_subscriber = R2d2PublisherNode()
    rclpy.spin(my_subscriber)

if __name__ == '__main__':
    main()