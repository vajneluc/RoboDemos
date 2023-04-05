#!/bin/bash

if [[ $1 == "rviz2" ]]; then
	ros2 launch urdf_tutorial_r2d2 demo.launch.py | rviz2 -d ~/ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
elif [[ $1 == "play" ]]; then
	ros2 bag play ~/devel/RoboDemos/bagfiles/rosbag2_2023_03_31-14_49_38
else
	ros2 launch urdf_tutorial_r2d2 demo.launch.py
fi
