import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformStamped
from urdf_parser_py.urdf import URDF
from robot_state_publisher.robot_state_publisher import RobotStatePublisher
 
package_name = "urdf_tutorial_r2d2"
urdf_name = "r2d2.urdf.xml"

class DummyRobotPublisherNode(Node):
    
    def __init__(self):
        super().__init__('dummy_robot_publisher_node')
 
        # Get the package path where the URDF file is located
        package_path = get_package_share_directory(package_name)
 
        # Load the URDF file
        with open(os.path.join(package_path, 'urdf', urdf_name), 'r') as file:
            urdf_file = file.read()
        self.urdf = URDF.from_xml_string(urdf_file)
 
        # Initialize the robot state publisher
        self.robot_state_publisher = RobotStatePublisher(self.urdf)
 
        # Initialize the joint states subscriber
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
 
        # Initialize the transform broadcaster
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)
 
    def joint_state_callback(self, msg):
        # Update the robot state publisher with the current joint states
        self.robot_state_publisher.publish_joint_state(msg)
 
        # Publish the transforms for each link in the URDF
        for link_name in self.urdf.link_map.keys():
            if link_name != self.urdf.get_root():
                tf = self.robot_state_publisher.get_link_transform(link_name)
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = self.urdf.get_root()
                transform_stamped.child_frame_id = link_name
                transform_stamped.transform.translation.x = tf.translation.x
                transform_stamped.transform.translation.y = tf.translation.y
                transform_stamped.transform.translation.z = tf.translation.z
                transform_stamped.transform.rotation.x = tf.rotation.x
                transform_stamped.transform.rotation.y = tf.rotation.y
                transform_stamped.transform.rotation.z = tf.rotation.z
                transform_stamped.transform.rotation.w = tf.rotation.w
                self.transform_broadcaster.sendTransform(transform_stamped)
 
        # Publish the static transforms in the URDF
        for joint_name, joint in self.urdf.joint_map.items():
            if joint.type != 'fixed':
                continue
            parent_link = self.urdf.get_joint(joint_name).parent
            child_link = self.urdf.get_joint(joint_name).child
            parent_frame_id = parent_link
            child_frame_id = child_link
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = parent_frame_id
            transform_stamped.child_frame_id = child_frame_id
            transform_stamped.transform.translation.x = joint.origin.xyz[0]
            transform_stamped.transform.translation.y = joint.origin.xyz[1]
            transform_stamped.transform.translation.z = joint.origin.xyz[2]
            transform_stamped.transform.rotation.x = joint.origin.rpy[0]
            transform_stamped.transform.rotation.y = joint.origin.rpy[1]
            transform_stamped.transform.rotation.z = joint.origin.rpy[2]
            transform_stamped.transform.rotation.w = 1.0
            self.static_transform_broadcaster.sendTransform(transform_stamped)
 
def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()