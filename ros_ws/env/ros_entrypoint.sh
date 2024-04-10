#!/bin/bash

# setup ros environment
source "/opt/ros/noetic/setup.sh"
# log_info "base ROS environment is ready"

# setup catkin workspace
source "${HOME}/ros_ws/devel/setup.sh"
log_info "catkin workspace is ready"

log_info_green "ROS environment is ready"