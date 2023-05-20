#!/bin/bash
# Build & Run
if [[ $1 == br  ]]; then
	cd ~/ros2_ws
	source ~/bin/pinokio.sh
	colcon build --packages-select  meshcat_visualizer
	source ~/bin/setenv.sh
	ros2 run meshcat_visualizer meshcat_visualizer_node
# Run
elif [[ $1 == r ]]; then
	cd ~/ros2_ws
	source ~/bin/pinokio.sh
	source ~/bin/setenv.sh
	ros2 run meshcat_visualizer meshcat_visualizer_node
# Build
elif [[ $1 == b ]]; then
	cd ~/ros2_ws
	colcon build --packages-select meshcat_visualizer meshcat_visualizer_node
	source ~/bin/setenv.sh
else
	echo "Usage meshcat.sh [br/b/r] (build and run/build/run)"
fi
