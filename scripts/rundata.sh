#!/bin/bash

if [[ $1 == "start" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/move_to_start_drift
elif [[ $1 == "x+" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/x_axis_plus
elif [[ $1 == "x+2" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/x_axis_plus_2
elif [[ $1 == "x-" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/x_axis_minus
elif [[ $1 == "x-2" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/x_axis_minus_2
elif [[ $1 == "y+" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/y_axis_plus
elif [[ $1 == "y-" ]]; then
	BAGPATH=~/devel/RoboDemos/bagfiles/panda_tests/y_axis_minus
fi

while true
do
	ros2 bag play $BAGPATH
	sleep 1
done
