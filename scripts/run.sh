#!/bin/bash

if [[ $1 == "moveit2" ]]; then
docker run \
  --rm \
  --network=host \
  -e DISPLAY \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v ~/.Xauthority:/tmp/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v `pwd`/share:/root/share \
  -p 9090:9090 \
  -p 8765:8765 \
  -p 5000:5000 \
  --expose=5000 \
  --expose=9090 \
  -it moveit2_tutorials_humble
elif [[ $1 == "ros2" ]]; then
docker run \
  --rm \
  --network=host \
  -e DISPLAY \
  -e XAUTHORITY=/tmp/.Xauthority \
  -v ~/.Xauthority:/tmp/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v `pwd`/share:/root/share \
  -p 9090:9090 \
  -p 8765:8765 \
  -p 5000:5000 \
  --expose=5000 \
  --expose=9090 \
  -it ros2-humble-lite
fi 
