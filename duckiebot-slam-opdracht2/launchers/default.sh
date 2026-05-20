#!/bin/bash
source /environment.sh
export VEHICLE_NAME=${VEHICLE_NAME:-$(hostname)}

echo "============================================================"
echo "  Duckiebot Opdracht 2 — Odometry + SLAM + EKF Fusion"
echo "  Vehicle: ${VEHICLE_NAME}"
echo "============================================================"

mkdir -p /code/catkin_ws/src
cd /code/catkin_ws

/bin/bash -c "
    source /opt/ros/noetic/setup.bash && \
    source /code/devel/setup.bash && \
    catkin_make -j1 && \
    echo 'catkin_make SUCCESS'
"

if [ $? -ne 0 ]; then
    echo "ERROR: catkin_make failed"
    exit 1
fi

source /code/devel/setup.bash
source /code/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://${VEHICLE_NAME}.local:11311

dt-launchfile-init

echo ""
echo "Starting nodes..."

# Taak 1 — Odometry
rosrun slam_pkg odometry_node.py &
sleep 1
# Taak 3 — Fusion EKF
rosrun slam_pkg fusion_node.py &
sleep 1
# Taak 2 — SLAM
rosrun slam_pkg slam_node.py &
sleep 1
# Path publisher voor RViz
rosrun slam_pkg path_publisher_node.py &

echo ""
echo "Topics published:"
echo "  • /${VEHICLE_NAME}/odometry/pose"
echo "  • /${VEHICLE_NAME}/odometry/path"
echo "  • /${VEHICLE_NAME}/fusion_node/pose"
echo "  • /${VEHICLE_NAME}/fusion_node/path"
echo "  • /${VEHICLE_NAME}/slam_node/features/compressed"
echo "  • /${VEHICLE_NAME}/slam_node/map/compressed"
echo "  • /${VEHICLE_NAME}/slam_node/landmarks"
echo ""

dt-launchfile-join
