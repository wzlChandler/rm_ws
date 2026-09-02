cmake_minimum_required(VERSION 3.0.2)
project(rm75_roh_apple_grasp_sim)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/snapshot_pose.py scripts/apple_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS scripts/generate_urdf.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config urdf models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
