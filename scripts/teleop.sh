#!/bin/bash

if [[ $1 == "start" ]]; then
	ros2 service call /servo_node/start_servo std_srvs/srv/Trigger
elif [[ $1 == "stop" ]]; then
	ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger
else
	echo "USAGE: teleop.sh start|stop"
fi
