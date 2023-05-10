# This examples shows how to load and move a robot in meshcat.
# Note: this feature requires Meshcat to be installed, this can be done using
# pip install --user meshcat
import time
import numpy as np
import sys
import os
from os.path import dirname, join, abspath


# rosbag2 API:
# https://ternaris.gitlab.io/rosbags/topics/rosbag2.html
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# pinocchio API:
# https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_display_b-meshcat-viewer.html
import pinocchio as pin 
from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
# Conversion with str seems to be necessary when executing this file with ipython

mesh_dir = "/home/julius/devel/franka_panda/meshes"
# urdf_filename = "talos_reduced.urdf"
# urdf_model_path = join(join(model_path,"talos_data/robots"),urdf_filename)
urdf_model_path = "/home/julius/devel/franka_panda/urdf/panda.urdf"
 
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir, pin.JointModelFreeFlyer()
)
 
viz = MeshcatVisualizer(model, collision_model, visual_model)
 
# Start a new MeshCat server and client.
# Note: the server can also be started separately using the "meshcat-server" command in a terminal:
# this enables the server to remain active after the current script ends.
#
# Option open=True pens the visualizer.
# Note: the visualizer can also be opened seperately by visiting the provided URL.
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)
 
# Load the robot in the viewer.
viz.loadViewerModel()
 
# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)

DISPLAY_VISUALS = False

viz.displayCollisions(not DISPLAY_VISUALS)
viz.displayVisuals(DISPLAY_VISUALS)

"""
mesh = visual_model.geometryObjects[0].geometry
mesh.buildConvexRepresentation(True)
convex = mesh.convex
 
if convex is not None:
    placement = pin.SE3.Identity()
    placement.translation[0] = 2.0
    geometry = pin.GeometryObject("convex", 0, convex, placement)
    geometry.meshColor = np.ones((4))
    visual_model.addGeometryObject(geometry)
""" 
# standing config
q1 = np.array(
    [0.0, 0.0, 0.235, 0.0, 0.0, 0.0, 1.0, 0.8, -1.6, 0.8, -1.6, -0.8, 1.6, -0.8, 1.6, 0]
)

"""
for i in range(10000):
    print("Frame:", i)
    time.sleep(0.01)
    angle = np.sin(i/100)
    q1[-4] = angle
    viz.display(q1)
"""
time.sleep(3)
# create reader instance and open for reading

# iterate over messages
while True:
    with Reader('/home/julius/devel/RoboDemos/bagfiles/panda_tests/x_axis_minus_2') as reader:
        # topic and msgtype information is available on .connections list
        print("Topics read:")
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        i = 0
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/joint_states':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                print(i, msg.position)
                q_ndx = 7
                q1[q_ndx:q_ndx+7] = msg.position[:7]
                time.sleep(0.02)
                viz.display(q1)
                i += 1
    """
    # messages() accepts connection filters
    connections = [x for x in reader.connections if x.topic == '/imu_raw/Imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = deserialize_cdr(rawdata, connection.msgtype)
        print(msg.header.frame_id)   
    """
        
"""
v0 = np.random.randn(model.nv) * 2
data = viz.data
pin.forwardKinematics(model, data, q1, v0)
frame_id = model.getFrameId("HR_FOOT")
viz.display()
viz.drawFrameVelocities(frame_id=frame_id)
 
model.gravity.linear[:] = 0.0
dt = 0.01
 
 
def sim_loop():
    tau0 = np.zeros(model.nv)
    qs = [q1]
    vs = [v0]
    nsteps = 100
    for i in range(nsteps):
        q = qs[i]
        v = vs[i]
        a1 = pin.aba(model, data, q, v, tau0)
        vnext = v + dt * a1
        qnext = pin.integrate(model, q, dt * vnext)
        qs.append(qnext)
        vs.append(vnext)
        viz.display(qnext)
        viz.drawFrameVelocities(frame_id=frame_id)
    return qs, vs
 
 
qs, vs = sim_loop()
 
fid2 = model.getFrameId("FL_FOOT")
 
 
def my_callback(i, *args):
    viz.drawFrameVelocities(frame_id)
    viz.drawFrameVelocities(fid2)
 
 
with viz.create_video_ctx("../leap.mp4"):
    viz.play(qs, dt, callback=my_callback)
"""