cmake_minimum_required(VERSION 3.0.2)
project(metrics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/metrics_node.py
  scripts/compute_accuracy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
