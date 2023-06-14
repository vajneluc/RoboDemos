import sys
import numpy as np
import pandas as pd
import time
# Pinocchio
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
# Meshcat
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import meshcat_shapes


# Path to Waypoint CSV
waypoints_csv_path = "/home/ros/dumps/waypoints.csv"

# Joint name list
names_list = ["panda_joint1", 
              "panda_joint2",
              "panda_joint3",
              "panda_joint4", 
              "panda_joint5", 
              "panda_joint6", 
              "panda_joint7"]

# Pandas dataframe
df = pd.read_csv(waypoints_csv_path)

if len(sys.argv) > 1:
    waypoint_list = [int(sys.argv[1])]
else:
    waypoint_list = [int(x) for x in range(len(df))]


configurations_list = []

for index, row in df.iterrows():
    if index in waypoint_list:
        waypoint_configuration = [row[x] for x in names_list]
        configurations_list.append(waypoint_configuration)

# Paths to urdf
default_urdf_path = "/home/ros/devel/RoboDemos/ROS2_packages/panda2_description/urdf/panda2_inertias.urdf"
mesh_path = "/home/ros/devel/RoboDemos/ROS2_packages/panda2_description/panda/meshes"

# Build from URDF
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    default_urdf_path, mesh_path, pin.JointModelFreeFlyer()
)
 
viz = MeshcatVisualizer(model, collision_model, visual_model)

try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

viz.loadViewerModel()

data = viz.data

# create dictionary: joint name -> q index
joint_indices_dict = {}
for name, jnt in zip(model.names, model.joints):
    joint_indices_dict[name] = jnt.idx_q

q = pin.neutral(model)

q2 = [0.0,-0.7853981633974483,0.0,-2.356194490192345,0.0,1.5707963267948966,0.7853981633974483]

for fname, fvalue in zip(names_list, q2):
    q[joint_indices_dict[fname]] = fvalue

pin.forwardKinematics(model, viz.data, q)
pin.updateFramePlacements(model, viz.data)

# Display Visuals or Collision
DISPLAY_VISUALS = True
DISPLAY_COLLISIONS = False
viz.displayCollisions(DISPLAY_COLLISIONS)
viz.displayVisuals(DISPLAY_VISUALS)




duration = 3

def create_text():
    # Create reference frame
        meshcat_shapes.frame(viz.viewer["world"]["reference"], axis_length=0.2, axis_thickness=0.01, opacity=0.8, origin_radius=0.02)
        viz.viewer["world"]["reference"].set_transform(tf.translation_matrix(np.array([0.5, - 0.5, 0])))

        # Find ee index in oMf
        ee_indx = model.getFrameId("panda_hand_tcp")
        
        # Display ee axis
        oMf = viz.data.oMf[ee_indx]  # < ---(9) this index works for oMi
        t_matrix = tf.translation_matrix(oMf.translation)
        r_matrix = oMf.rotation
        t_matrix[0:3, 0:3] = r_matrix

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
        meshcat_shapes.textarea(viz.viewer["world"]["reference"]["ee_coord_x"], f"X = {ee_x_coord}", font_size=10)
        meshcat_shapes.textarea(viz.viewer["world"]["reference"]["ee_coord_y"], f"Y = {ee_y_coord}", font_size=10)
        meshcat_shapes.textarea(viz.viewer["world"]["reference"]["ee_coord_z"], f"Z = {ee_z_coord}", font_size=10)
        meshcat_shapes.textarea(viz.viewer["world"]["reference"]["quat"], f"X = {round(quat.x, 3)} Y = {round(quat.y, 3)} Z = {round(quat.z, 3)} W = {round(quat.w,3)}", font_size=10)
        meshcat_shapes.textarea(viz.viewer["world"]["reference"]["angle"], f"angle = {round(angle, 3)}", font_size=10)
        meshcat_shapes.textarea(viz.viewer["world"]["reference"]["axis"], f"axis = [{round(axis[0],3)}, {round(axis[1],3)}, {round(axis[2],3)}]", font_size=10)

        # Update coordinate text position
        viz.viewer["world"]["reference"]["ee_coord_x"].set_transform(t_matrix_x)
        viz.viewer["world"]["reference"]["ee_coord_y"].set_transform(t_matrix_y)
        viz.viewer["world"]["reference"]["ee_coord_z"].set_transform(t_matrix_z)
        viz.viewer["world"]["reference"]["quat"].set_transform(t_matrix_quat)
        viz.viewer["world"]["reference"]["angle"].set_transform(t_matrix_angle)
        viz.viewer["world"]["reference"]["axis"].set_transform(t_matrix_axis)

while True:
    for waypoint in configurations_list:
        waypoint_indx = configurations_list.index(waypoint)
        print(f"Displaying Waypoint {waypoint_indx}\n{waypoint}")
        q2 = waypoint

        for fname, fvalue in zip(names_list, q2):
            q[joint_indices_dict[fname]] = fvalue

        pin.forwardKinematics(model, viz.data, q)
        pin.updateFramePlacements(model, viz.data)
        
        # Update Waypoint Label
        meshcat_shapes.textarea(viz.viewer["world"]["label"], f"Waypoint{waypoint_indx}", font_size=20)
        t_matrix = tf.translation_matrix(np.array([0, -0.66, 0]))
        viz.viewer["world"]["label"].set_transform(t_matrix)
        
        # Update position of Joint labels and Joint frames
        for frame, oMf in zip(model.frames, data.oMf):
            # Joints
            if str(frame.type) == "JOINT":
                meshcat_shapes.textarea(viz.viewer[frame.name]["name"], f"{frame.name}", font_size=10)
                meshcat_shapes.frame(viz.viewer[frame.name]["frame"],axis_length=0.2, axis_thickness=0.01, opacity=0.8, origin_radius=0.02)
                t_matrix_name = tf.translation_matrix(oMf.translation + np.array([0, -0.25, 0]))
                t_matrix_frame = tf.translation_matrix(oMf.translation)
                r_matrix_frame = oMf.rotation
                t_matrix_frame[0:3, 0:3] = r_matrix_frame
                viz.viewer[frame.name]["name"].set_transform(t_matrix_name)
                viz.viewer[frame.name]["frame"].set_transform(t_matrix_frame)
            # EE
            if str(frame.name) == "panda_hand_tcp":
                meshcat_shapes.textarea(viz.viewer[frame.name]["name"], f"{frame.name}", font_size=10)
                meshcat_shapes.frame(viz.viewer[frame.name]["frame"],axis_length=0.2, axis_thickness=0.01, opacity=0.8, origin_radius=0.02)
                t_matrix_name = tf.translation_matrix(oMf.translation + np.array([0, -0.25, 0]))
                t_matrix_frame = tf.translation_matrix(oMf.translation)
                r_matrix_frame = oMf.rotation
                t_matrix_frame[0:3, 0:3] = r_matrix_frame
                viz.viewer[frame.name]["name"].set_transform(t_matrix_name)
                viz.viewer[frame.name]["frame"].set_transform(t_matrix_frame)
        
        create_text()
        start_time = time.time()
        viz.display(q)
        time.sleep(duration)
            
