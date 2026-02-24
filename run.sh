#!/usr/bin/env zsh

pkill -f ros2
pkill -f gz

source /opt/ros/jazzy/setup.zsh
colcon build || exit 1
source install/setup.zsh