#!/bin/bash

if [[ $1 == "1" ]]; then
	BAGPATH=/home/ros/devel/RoboDemos/bagfiles/rosbag2_2023_03-1
elif [[ $1 == "2" ]]; then
	BAGPATH=/home/ros/devel/RoboDemos/bagfiles/rosbag2_2023_03-2
elif [[ $1 == "3" ]]; then
        BAGPATH=/home/ros/devel/RoboDemos/bagfiles/rosbag2_2023_03-3
fi

while true
do
	ros2 bag play $BAGPATH
	sleep 1
done
