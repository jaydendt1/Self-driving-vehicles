cmake_minimum_required(VERSION 3.0.2)
project(slam_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  tf
  tf2_ros
  duckietown_msgs
)

catkin_package()

install(PROGRAMS
  src/odometry_node.py
  src/slam_node.py
  src/fusion_node.py
  src/path_publisher_node.py
  src/camera_reader_node.py
  src/wheel_encoder_reader_node.py
  src/wheel_publisher_node.py
  src/twist_publisher_node.py
  src/diagnostic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
