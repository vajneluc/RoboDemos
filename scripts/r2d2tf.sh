#!/bin/bash

ros2 launch r2d2_publisher r2d2_publisher_node.launch.py fields:="transform.translation,transform.rotation" filterby:="header.frame_id=odom"
