#!/usr/bin/env bash
set -e

# Ako si u WSL2 + WSLg, provjeri DISPLAY (i instaliraj xhost jednom: sudo apt install -y x11-xserver-utils x11-apps)
xhost +local:root

docker run -it --rm \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e TURTLEBOT3_MODEL=waffle \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/ros_ws":/root/ros_ws \
  --name tag_game \
  ros:tag_game bash
