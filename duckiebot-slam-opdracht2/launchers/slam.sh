#!/bin/bash
source /environment.sh
source /code/devel/setup.bash
source /code/catkin_ws/devel/setup.bash
export VEHICLE_NAME=${VEHICLE_NAME:-$(hostname)}
export ROS_MASTER_URI=http://${VEHICLE_NAME}.local:11311
dt-launchfile-init
rosrun slam_pkg slam_node.py
dt-launchfile-join
