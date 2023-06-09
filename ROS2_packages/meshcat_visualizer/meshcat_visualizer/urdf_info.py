import sys
# pinocchio API:
# https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_display_b-meshcat-viewer.html
import pinocchio as pin


default_urdf_path = "/home/ros/devel/RoboDemos/ROS2_packages/panda2_description/urdf/panda2_inertias.urdf"


if len(sys.argv) > 1:
    urdf_path =  sys.argv[1]
else:
    urdf_path = default_urdf_path

        
model = pin.buildModelFromUrdf(urdf_path)

data = model.createData()
print("No. of frames:", len(model.frames))

print("ALL FRAMES:")
for i, frame in enumerate(model.frames):
    print(f"{i:4} NAME = {frame.name:20}\t PARENT = {frame.parent} PREVIOUS = {frame.previousFrame}\t TYPE = {frame.type}")

print()
print("JOINTS:")
for i, frame in enumerate(model.frames):
    if str(frame.type) == "JOINT":
        print(f"{i:4} NAME = {frame.name:20}\t PARENT = {frame.parent} PREVIOUS = {frame.previousFrame}\t TYPE = {frame.type}")

