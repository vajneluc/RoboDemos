# FOXGLOVE BRIDGE
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# FRANKA_MOVEIT
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# Using URDF with robot_state_publisher
ros2 launch urdf_tutorial_r2d2 demo.launch.py
rviz2 -d ~/ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz

# ros2 pkg create
cd /home/julius/ros2_ws/src/
ros2 pkg create --build-type ament_python