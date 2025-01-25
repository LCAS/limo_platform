#!/bin/bash

# source additional packages
source /opt/ros/lcas/install/setup.bash

# ensure students always have the latest index of everything
sudo apt update
rosdep --rosdistro=${ROS_DISTRO} update
rosdep install --from-paths src --ignore-src -r -y
rm -rf build/ install/ log/
colcon build

