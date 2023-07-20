import sys
import numpy as np
import time
import math
# Pinocchio
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
# Meshcat
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import meshcat_shapes
import matplotlib.pyplot as plt
from panda_simple_kinematics import PandaSimpleKinematics, rotate_vec_by_angle, rotate_xy_by_angle



 
# creating initial data values
# of x and y
x = np.array([0,1])
y = np.array([0,1])
 
plt.gca().set_aspect('equal')

# to run GUI event loop
plt.ion()
 
# here we are creating sub plots
figure, ax = plt.subplots(figsize=(10, 10))
lineA, = ax.plot(x, y)
lineB, = ax.plot(x, y)
lineAB, = ax.plot(x, y)
lineC, = ax.plot(x, y)
lineBC, = ax.plot(x, y)
lineD, = ax.plot(x, y)
lineCD, = ax.plot(x, y)
pointP, =  ax.plot(x, y, marker="o")
pointEE, =  ax.plot(x, y, marker="o")
  
# setting x-axis label and y-axis label
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
 


############################

# Paths to urdf
default_urdf_path = "/home/ros/devel/RoboDemos/ROS2_packages/panda2_description/urdf/panda2_inertias_poleholder.urdf"
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

tbl_header = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
q2 = [0.0,-0.7853981633974483,0.0,-2.356194490192345,0.0,1.5707963267948966,0.7853981633974483]

def apply_joint_config(q2):
    for fname, fvalue in zip(tbl_header, q2):
        q[joint_indices_dict[fname]] = fvalue

apply_joint_config(q2)

viz.display(q)
pin.forwardKinematics(model, viz.data, q)
pin.updateFramePlacements(model, viz.data)

# Display Visuals or Collision
DISPLAY_VISUALS = True
DISPLAY_COLLISIONS = False
# viz.displayCollisions(DISPLAY_COLLISIONS)
viz.displayVisuals(DISPLAY_VISUALS)


transforms = {}

for frame, oMf in zip(model.frames, data.oMf):
    print(frame.name)
    t_matrix_frame = tf.translation_matrix(oMf.translation)
    # print("  Transform:", t_matrix_frame)
    transforms[frame.name] = oMf
    if "link" in frame.name and "_sc" not in frame.name:
        meshcat_shapes.textarea(viz.viewer[frame.name]["name"], f"{frame.name}", font_size=10)
        meshcat_shapes.frame(viz.viewer[frame.name]["frame"],axis_length=0.2, axis_thickness=0.01, opacity=0.8, origin_radius=0.02)
        t_matrix_name = tf.translation_matrix(oMf.translation + np.array([0, -0.25, 0]))
        r_matrix_frame = oMf.rotation
        t_matrix_frame[0:3, 0:3] = r_matrix_frame
        viz.viewer[frame.name]["name"].set_transform(t_matrix_name)
        viz.viewer[frame.name]["frame"].set_transform(t_matrix_frame)

frame = 0
current_time = 0
alpha = 0 #0.345
A = 0.6 # 0 # -0.7853981633974483
beta = 0
B = -math.pi/2 # -2.356194490192345
gamma = 0
C = 0 # 0.3 # 0 # 1.5707963267948966
delta = 0.7853981633974483

pc = PandaSimpleKinematics()



while True:
    print("Rendering frame:", frame, "J2:", transforms["panda_joint2"].translation, "J4:", transforms["panda_joint4"].translation, "J6:", transforms["panda_joint6"].translation, "J8:", transforms["panda_joint8"].translation)

    q2 = pc.make_qvec(alpha, A, B)
    apply_joint_config(q2)

    C = q2[5]

    pin.forwardKinematics(model, viz.data, q)
    pin.updateFramePlacements(model, viz.data)

    # compute coordinates r, h based on angles A,B
    origin = np.array([0,0])
    pos_A = origin + pc.link_a
    pos_B = pos_A + rotate_vec_by_angle(pc.link_b, A)
    pos_AB = pos_A + rotate_vec_by_angle([pc.link_b[0], 0], A)
    pos_C = pos_B + rotate_vec_by_angle(pc.link_c, A-B)
    pos_BC = pos_B + rotate_vec_by_angle([0, pc.link_c[1]], A-B)
    pos_D = pos_C + rotate_vec_by_angle(pc.link_d, A-B-C)
    pos_CD = pos_C + rotate_vec_by_angle([0, pc.link_d[1]], A-B-C)

    # express formulas for computing h, r
    # so we can get inverse formula for computing A,B from (h,r)
    pos_hr = pc.link_a + rotate_vec_by_angle(pc.link_b, A) + rotate_vec_by_angle(pc.link_c, A-B) + pc.link_d

    h, r = pc.compute_hr_from_AB(A, B)

    x, y, z = pc.forward_kinematics(alpha, A, B)
    alpha_, A_, B_ = pc.inverse_kinematics(x, y, z)

    print(f"Forward kinematics: x={x} y={y} z={z}")
    print(f"Inverse kinematics: alpha={alpha} A={A} B={B}")
    print(f"A={A} B={B} D={A-B}")

    Ahat, Bhat, px, py = pc.compute_AB_from_hr(h, r)

    print("Ahat:", Ahat, "Bhat:", Bhat) # equal to Joint4 position...


    print("  posA:", pos_A, "posB:", pos_B, "posC:", pos_C, "posHR:", pos_hr, "h=", h, "r=", r)

    viz.display(q)
    delay = 0.05
    time.sleep(delay)
    current_time += delay
    frame += 1

    # animate...
    alpha += delay * 0.2
     
    # updating data values
    lineA.set_ydata([origin[0], pos_A[0]])
    lineA.set_xdata([origin[1], pos_A[1]])
    lineB.set_ydata([pos_A[0], pos_B[0]])
    lineB.set_xdata([pos_A[1], pos_B[1]])
    lineAB.set_ydata([pos_A[0], pos_AB[0], pos_B[0]])
    lineAB.set_xdata([pos_A[1], pos_AB[1], pos_B[1]])
    lineC.set_ydata([pos_B[0], pos_C[0]])
    lineC.set_xdata([pos_B[1], pos_C[1]])
    lineBC.set_ydata([pos_B[0], pos_BC[0], pos_C[0]])
    lineBC.set_xdata([pos_B[1], pos_BC[1], pos_C[1]])
    lineD.set_ydata([pos_C[0], pos_D[0]])
    lineD.set_xdata([pos_C[1], pos_D[1]])
    lineCD.set_ydata([pos_C[0], pos_CD[0], pos_D[0]])
    lineCD.set_xdata([pos_C[1], pos_CD[1], pos_D[1]])
    pointP.set_xdata([px])
    pointP.set_ydata([py])
    pointEE.set_xdata([px])
    pointEE.set_ydata([py])
 
    # drawing updated values
    figure.canvas.draw()
 
    # This will run the GUI event
    # loop until all UI events
    # currently waiting have been processed
    figure.canvas.flush_events()
