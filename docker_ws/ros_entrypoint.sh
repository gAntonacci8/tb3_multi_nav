#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f "/root/ros_ws/install/setup.bash" ]; then
  source /root/ros_ws/install/setup.bash
fi
export TURTLEBOT3_MODEL=burger_cam
exec "$@"
