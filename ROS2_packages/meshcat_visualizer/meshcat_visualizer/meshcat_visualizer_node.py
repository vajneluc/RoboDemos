#!/usr/bin/env python
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from dummy_control_msgs.msg import DummyControlDebug


# pinocchio API:
# https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_display_b-meshcat-viewer.html
import pinocchio as pin 
from pinocchio.visualize import MeshcatVisualizer

# Meshcat
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

axes_displayed = input("Display panda joint axes?\nEnter up to seven numbers (1 - 7)\n:").split()
allowed = [str(x) for x in range(1,8)]
for a in axes_displayed:
    if a not in allowed:
        print("Input Error")
        quit()
    else:
        continue

class MeshcatVisualizerNode(Node):
    
    def __init__(self):

        super().__init__('meshcat_visualizer')

        # Create dummy subscriber
        self.subscription = self.create_subscription(
            DummyControlDebug,
            '/mytopic',
            self.dummy_listener_callback,
            10)
        self.subscription  # prevent unused variable warning     
        
        # Call subsriber
        self.last_dummy = DummyControlDebug()

        # Load URDF model
        self.mesh_dir = "/home/julius/devel/RoboDemos/ROS2_packages/panda2_description/panda/meshes"
        self.urdf_model_path = "/home/julius/devel/RoboDemos/ROS2_packages/panda2_description/urdf/panda.urdf"
        
        # Build from URDF
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            self.urdf_model_path, self.mesh_dir, pin.JointModelFreeFlyer())
    
        # Start MeshCat server and client
        self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        try:
            self.viz.initViewer(open=True)
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

        DISPLAY_VISUALS = True
        DISPLAY_COLLISIONS = False
        self.viz.displayCollisions(DISPLAY_COLLISIONS)
        self.viz.displayVisuals(DISPLAY_VISUALS)
              
        # Init Geometry
        AXIS_HEIGHT = 0.2
        AXIS_RADIUS = 0.004
        x_axis, y_axis, z_axis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
        Rx = tf.rotation_matrix(-np.pi/2, x_axis)
        Ry = tf.rotation_matrix(np.pi, x_axis)
        Rz = tf.rotation_matrix(np.pi/2, z_axis)
        axes_translation = tf.translation_matrix([0, -AXIS_HEIGHT/2, 0])
        
        axes_displayed.sort()
        for axe_index in axes_displayed:

            self.viz.viewer["AXES"][axe_index]["EE"].set_object(g.Sphere(0.02), 
                g.MeshLambertMaterial(
                    color=0x000000,
                    reflectivity=0.8)) 
            # X Axis
            self.viz.viewer["AXES"][axe_index]["X"].set_object(g.Cylinder(height=AXIS_HEIGHT, radius=AXIS_RADIUS), 
                g.MeshLambertMaterial(
                    color=0xFF0000,
                    reflectivity=0.8))
            self.viz.viewer["AXES"][axe_index]["X"].set_transform(Rz @ axes_translation)
            
            # Y Axis
            self.viz.viewer["AXES"][axe_index]["Y"].set_object(g.Cylinder(AXIS_HEIGHT, radius=AXIS_RADIUS), 
                g.MeshLambertMaterial(
                    color=0x008000,
                    reflectivity=0.8))
            self.viz.viewer["AXES"][axe_index]["Y"].set_transform(Ry @ axes_translation)

            # Z Axis
            self.viz.viewer["AXES"][axe_index]["Z"].set_object(g.Cylinder(AXIS_HEIGHT, radius=AXIS_RADIUS), 
                g.MeshLambertMaterial(
                    color=0x0000FF,
                    reflectivity=0.8))
            self.viz.viewer["AXES"][axe_index]["Z"].set_transform(Rx @ axes_translation)
        
    def dummy_listener_callback(self, msg):
        self.last_dummy = msg
        self.update()
        # self.get_logger().info(f'Dummy msgs: {msg}')
 
    def update(self):

        # Updates position of panda
        q_index = 7
        self.q[q_index:q_index+7] = self.last_dummy.position[:7]
        self.viz.display(self.q)

        # Updates position of axes
        for axe_index in axes_displayed:
            pin.forwardKinematics(self.model, self.viz.data, self.q)
            placement = self.viz.data.oMi[int(axe_index) + 2]
            t_matrix = tf.translation_matrix(placement.translation)   
            r_matrix = placement.rotation
            t_matrix[0:3,0:3] = r_matrix
           #print(f"placement.rotation: \n {placement.rotation} \n placement.translation {placement.translation}")
            self.viz.viewer["AXES"][axe_index].set_transform(t_matrix)



def main():	
    rclpy.init()
    node = MeshcatVisualizerNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
