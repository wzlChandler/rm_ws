cmake_minimum_required(VERSION 3.0.2)
project(rm75_roh_gazebo_sim)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config models urdf scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
