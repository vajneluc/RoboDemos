#!/bin/bash

if [[ $1 == "rviz2" ]]; then
	ros2 launch urdf_tutorial_r2d2 demo.launch.py | rviz2 -d ~/ros2_ws/install/urdf_tutorial_r2d2/share/urdf_tutorial_r2d2/r2d2.rviz
else
	ros2 launch urdf_tutorial_r2d2 demo.launch.py
fi
