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
import meshcat_shapes



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
        

        self.create_axes("reference")
        self.create_axes("ee")
        self.viz.viewer["AXES"]["reference"].set_transform(tf.translation_matrix(np.array([0.5, - 0.5, 0])))
        # Time for updating frequency
        self.frame_count = 0
        

    # Callback function for /mytopic
    def dummy_listener_callback(self, msg):
        self.last_dummy = msg
        self.update(msg.position[:7])

    # Callback function for /joint_states
    def joint_listener_callback(self, msg):
        self.frame_count += 1
        if self.frame_count == 10:
            self.last_joint
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
            self.frame_count = 0

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
        self.viz.viewer["AXES"][joint_name]["CENTER"].set_object(
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
        
        # Pair frame name with oMf data
        for frame, oMf in zip(self.model.frames, self.viz.data.oMf):
            if "panda_joint" in frame.name:
                joint_indx = self.model.getFrameId(frame.name)
                # Calculate matrixes
                t_matrix = tf.translation_matrix(oMf.translation)
                r_matrix = oMf.rotation
                t_matrix[0:3, 0:3] = r_matrix
                
                # Update Axes position
                self.viz.viewer["AXES"][frame.name].set_transform(t_matrix)
        
        # Find ee index in oMf
        ee_indx = self.model.getFrameId("panda_hand_tcp")
        
        # Display ee axis
        oMf = self.viz.data.oMf[ee_indx]  # < ---(9) this index works for oMi
        t_matrix = tf.translation_matrix(oMf.translation)
        r_matrix = oMf.rotation
        t_matrix[0:3, 0:3] = r_matrix
        self.viz.viewer["AXES"]["ee"].set_transform(t_matrix)

        # Calculate matrixes
        t_matrix_x = tf.translation_matrix(np.array([0, -0.2, 0]))
        t_matrix_y = tf.translation_matrix(np.array([0, -0.25, 0]))
        t_matrix_z = tf.translation_matrix(np.array([0, -0.3, 0]))
        t_matrix_quat = tf.translation_matrix(np.array([0, -0.05, 0]))
        t_matrix_angle = tf.translation_matrix(np.array([0, -0.1, 0]))
        t_matrix_axis = tf.translation_matrix(np.array([0, -0.15, 0]))
        #  Update ee position
        ee_x_coord = round(oMf.translation[0], 3)
        ee_y_coord = round(oMf.translation[1], 3)
        ee_z_coord = round(oMf.translation[2], 3)

        quat = pin.Quaternion(r_matrix)
        angle = pin.AngleAxis(r_matrix).angle
        axis =  pin.AngleAxis(r_matrix).axis

        # Display ee coords
        meshcat_shapes.textarea(self.viz.viewer["world"]["ee_coord_x"], f"X = {ee_x_coord}", font_size=10)
        meshcat_shapes.textarea(self.viz.viewer["world"]["ee_coord_y"], f"Y = {ee_y_coord}", font_size=10)
        meshcat_shapes.textarea(self.viz.viewer["world"]["ee_coord_z"], f"Z = {ee_z_coord}", font_size=10)
        meshcat_shapes.textarea(self.viz.viewer["AXES"]["reference"]["quat"], f"X = {round(quat.x, 3)} Y = {round(quat.y, 3)} Z = {round(quat.z, 3)} W = {round(quat.w,3)}", font_size=10)
        meshcat_shapes.textarea(self.viz.viewer["AXES"]["reference"]["angle"], f"angle = {round(angle, 3)}", font_size=10)
        meshcat_shapes.textarea(self.viz.viewer["AXES"]["reference"]["axis"], f"axis = [{round(axis[0],3)}, {round(axis[1],3)}, {round(axis[2],3)}]", font_size=10)

        # Update coordinate text position
        self.viz.viewer["world"]["ee_coord_x"].set_transform(t_matrix_x)
        self.viz.viewer["world"]["ee_coord_y"].set_transform(t_matrix_y)
        self.viz.viewer["world"]["ee_coord_z"].set_transform(t_matrix_z)
        self.viz.viewer["AXES"]["reference"]["quat"].set_transform(t_matrix_quat)
        self.viz.viewer["AXES"]["reference"]["angle"].set_transform(t_matrix_angle)
        self.viz.viewer["AXES"]["reference"]["axis"].set_transform(t_matrix_axis)

        self.get_logger().info(f"\nangle = {angle}\naxis =  {axis}\nquat = {quat}")
        
        


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
