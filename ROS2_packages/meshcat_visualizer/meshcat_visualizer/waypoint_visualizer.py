#!/usr/bin/env python
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from dummy_control_msgs.msg import DummyControlDebug
from sensor_msgs.msg import JointState
import yaml 

# pinocchio API:
# https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_display_b-meshcat-viewer.html
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Meshcat
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf



class MeshcatVisualizerNode(Node):
    def __init__(self):
        super().__init__("meshcat_visualizer")

        # Declare launch parameter to use data source. (works with /mytopic and /joint_states)
        self.declare_parameter("topic_source", "/joint_states")
        topic_source = (
            self.get_parameter("topic_source").get_parameter_value().string_value
        )
        self.declare_parameter("config_path", "")
        config_path = (
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        
        try:
            self.get_logger().info(f"Loading config from: {config_path}")
            with open(config_path, "r", encoding="utf-8") as infile:
                self.config = yaml.safe_load(infile)
            self.get_logger().info("Config loaded successfuly!")
        except Exception as err:
            self.get_logger().error("Config loading failed!")
            sys.exit(0)


        if topic_source == "/my_topic":
            # Create /my_topic subscriber
            self.subscription = self.create_subscription(
                DummyControlDebug, "/mytopic", self.dummy_listener_callback, 10
            )
            self.subscription  # prevent unused variable warning
            # Call subsriber
            self.last_dummy = DummyControlDebug()
        elif topic_source == "waypoints.csv":
            self.csv = topic_source
        elif topic_source == "traces.csv":
            self.csv = topic_source
        else:   
            # Create /joint_states subscriber
            self.subscription = self.create_subscription(
                JointState, "/joint_states", self.joint_listener_callback, 10
            )
            self.subscription  # prevent unused variable warning
            # Call subsriber
            self.last_joint = JointState()

        # Load URDF model
        self.mesh_dir = self.config["mesh_dir"]
        self.urdf_model_path = self.config["urdf_model_path"]

        # Build from URDF
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            self.urdf_model_path, self.mesh_dir, pin.JointModelFreeFlyer()
        )

        # Start MeshCat server and client
        self.viz = MeshcatVisualizer(
            self.model, self.collision_model, self.visual_model
        )
        try:
            self.viz.initViewer(open=False)
        except ImportError as err:
            print(
                "Error while initializing the viewer. It seems you should install Python meshcat"
            )
            print(err)
            sys.exit(0)

        # Load the built robot in the viewer
        self.viz.loadViewerModel()

        # Init the robot configuration.
        self.q = pin.neutral(self.model)
        self.viz.display(self.q)

        # Display Visuals or Collision
        DISPLAY_VISUALS = True
        DISPLAY_COLLISIONS = False
        self.viz.displayCollisions(DISPLAY_COLLISIONS)
        self.viz.displayVisuals(DISPLAY_VISUALS)

        # Init Axes X Y Z for all joints
        for joint_name in self.model.names:
            if "panda_joint" in joint_name:
                self.create_axes(joint_name)
        self.create_axes("ee")

    # Callback function for /mytopic
    def dummy_listener_callback(self, msg):
        self.last_dummy = msg
        self.update(msg.position[:7])

    # Callback function for /joint_states
    def joint_listener_callback(self, msg):
        self.last_joint = msg
        names = msg.name
        positions = msg.position
        out = []
        for name, pos in zip(names, positions):
            if "panda_joint" in name:
                out.append((name, pos))
        out.sort(key=lambda x: x[0])
        out_positions = [np[1] for np in out]
        self.update(out_positions)

    # Function for creating Axes for panda joints
    def create_axes(self, joint_name):
        # Gemotery constants
        AXIS_HEIGHT = 0.2
        AXIS_RADIUS = 0.004
        CENTER_POINT_RADIUS = 0.01
        # Centering Axes
        x_axis, y_axis, z_axis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
        Rx = tf.rotation_matrix(-np.pi / 2, x_axis)
        Ry = tf.rotation_matrix(np.pi, x_axis)
        Rz = tf.rotation_matrix(np.pi / 2, z_axis)
        axes_translation = tf.translation_matrix([0, -AXIS_HEIGHT / 2, 0])

        # Center point
        self.viz.viewer["AXES"][joint_name]["EE"].set_object(
            g.Sphere(CENTER_POINT_RADIUS),
            g.MeshLambertMaterial(color=0x000000, reflectivity=0.8),
        )

        # X Axis
        self.viz.viewer["AXES"][joint_name]["X"].set_object(
            g.Cylinder(AXIS_HEIGHT, AXIS_RADIUS),
            g.MeshLambertMaterial(color=0xFF0000, reflectivity=0.8),
        )
        self.viz.viewer["AXES"][joint_name]["X"].set_transform(Rz @ axes_translation)

        # Y Axis
        self.viz.viewer["AXES"][joint_name]["Y"].set_object(
            g.Cylinder(AXIS_HEIGHT, AXIS_RADIUS),
            g.MeshLambertMaterial(color=0x008000, reflectivity=0.8),
        )
        self.viz.viewer["AXES"][joint_name]["Y"].set_transform(Ry @ axes_translation)

        # Z Axis
        self.viz.viewer["AXES"][joint_name]["Z"].set_object(
            g.Cylinder(AXIS_HEIGHT, AXIS_RADIUS),
            g.MeshLambertMaterial(color=0x0000FF, reflectivity=0.8),
        )
        self.viz.viewer["AXES"][joint_name]["Z"].set_transform(Rx @ axes_translation)

    # Function for displaying panda joint axes
    def display_joint_axes(self):
        # Pair joit name with oMi data
        for joint_name, oMi in zip(self.model.names, self.viz.data.oMi):
            if "panda_joint" in joint_name:
                placement = oMi
                # Calculate matrixes
                t_matrix = tf.translation_matrix(placement.translation)
                r_matrix = placement.rotation
                t_matrix[0:3, 0:3] = r_matrix
                # Update Axes position
                self.viz.viewer["AXES"][joint_name].set_transform(t_matrix)
        # Display ee axis
        placement = self.viz.data.oMi[9]  # < ---(9)
        # Calculate matrixes
        t_matrix = tf.translation_matrix(placement.translation)
        r_matrix = placement.rotation
        t_matrix[0:3, 0:3] = r_matrix
        # Update ee position
        self.viz.viewer["AXES"]["ee"].set_transform(t_matrix)

    def link_axes(self):
        # Not implemented
        None

    # Function for updating
    def update(self, panda_position):
        # Updates position of panda
        self.q[7:14] = panda_position
        self.viz.display(self.q)
        pin.forwardKinematics(self.model, self.viz.data, self.q)
        pin.updateFramePlacements(self.model, self.viz.data)
        # Updates position of axes
        self.display_joint_axes()


def main():
    rclpy.init()
    node = MeshcatVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
