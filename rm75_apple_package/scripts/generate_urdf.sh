#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source "$HOME/rm_ws/devel/setup.bash"
PKG="$(rospack find rm75_roh_apple_grasp_sim)"
xacro --inorder "$PKG/urdf/rm75_roh_apple_sim.xacro" "$@" | sed -E 's/(joint name=")(if|mf|rf|lf|th)_(slider|root)_link"/\1\2_\3_link_j"/g'
