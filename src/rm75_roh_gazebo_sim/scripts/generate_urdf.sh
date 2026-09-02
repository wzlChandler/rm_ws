#!/bin/bash
# 生成仿真 URDF:xacro 展开 rm75_roh_sim.xacro,然后给手爪 joint 加 _j 后缀
# (规避 ROH URDF 中 link/joint 同名导致 gazebo 的 name collision)
set -e
source /opt/ros/noetic/setup.bash
source "$HOME/rm_ws/devel/setup.bash"
PKG="$(rospack find rm75_roh_gazebo_sim)"
xacro --inorder "$PKG/urdf/rm75_roh_sim.xacro" "$@" \
  | python3 "$PKG/scripts/rename_rohand_joints.py"
