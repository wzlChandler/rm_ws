cmake_minimum_required(VERSION 3.0.2)
project(rm75_roh_description)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/a001_hand_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
