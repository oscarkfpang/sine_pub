#!/bin/bash
#set -e

# setup ros environment
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "ros2" "run" "sine_pub" "sine_pub"
