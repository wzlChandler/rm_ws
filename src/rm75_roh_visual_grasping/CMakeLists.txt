cmake_minimum_required(VERSION 3.0.2)
project(rm75_roh_visual_grasping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  rm_msgs
  rospy
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/hand_teleop_node.py
  scripts/visual_grasp_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_core.py)
  catkin_add_nosetests(test/test_hand_teleop_core.py)
endif()
