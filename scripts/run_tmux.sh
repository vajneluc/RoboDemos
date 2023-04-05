#!/bin/bash

tmux new-session \; \
     send-keys 'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash' C-m \; \
     split-window -h \; \
     send-keys 'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash' C-m \; \
     split-window -h \; \
     send-keys 'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash' C-m \; \
     split-window -h \; \
     send-keys  'mc' C-m \; \
     set-option -g mouse on \; \
     select-layout tiled
     
