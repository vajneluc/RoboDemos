#!/bin/bash
# if no arguments are passed
if [ "$#" -eq 0 ]; then

	echo "Launching ROS2 Teleop in default configuration..."
	echo "No arguments were passed."
	echo "Launch Arguments:"
	echo "-b {true}	                        # Builds the package"
	echo "-p {true(default)/false}        	# Use pynput or ssh keyboard"
	echo "-j {true(default)/false}	        # Starts ROS2 Joystick Node"
	echo "-m {true/false(default)}	        # Starts Meshcat Visualizer"
	echo "-t {my_topic/joint_states(default)}	# Select topic for Meshcat"
fi

# Launch arguments
# Default Config
build="false"
pynput="true"
joy="true"
meshcat="false"
topic="joint_states"
# Read Launch Config
while getopts "b:p:j:m:t:" option
do 
	case "$option"
		in
		b) build="$OPTARG" ;;
		p) pynput="$OPTARG" ;;
		j) joy="$OPTARG" ;;
		m) meshcat="$OPTARG" ;;
		t) topic="$OPTARG" ;;
	esac
done
# Print the current launch configuration
echo "LAUNCHING IN CONFIGURATION:"
echo "Build package : $build"
echo "Start Joystick : $joy"
echo "Using pynput: $pynput"
echo "Start Meshcat : $meshcat"
echo "Selected topic : /$topic"

sleep 2

# Launch Configurations
cd ~/ros2_ws
if [ $build = "true" ]; then
	colcon build --packages-select ros2_teleop meshcat_visualizer

source setenv.sh
source pinokio.sh
echo "launching in 3"
sleep 1
echo "launching in 2"
sleep 1
echo "launching in 1"
sleep 1
ros2 launch ros2_teleop ros2_teleop_launch.py start_joy:=$joy use_meshcat:=$meshcat topic_source:=/$topic use_pynut:=$pynput

