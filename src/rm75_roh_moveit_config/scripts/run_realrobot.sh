#!/usr/bin/env bash
set -o pipefail

workspace=/home/test/rm_ws
log_dir="$workspace/logs"
timestamp=$(date +%Y%m%d_%H%M%S)
log_file="$log_dir/realrobot_${timestamp}.log"

mkdir -p "$log_dir"
ln -sfn "$(basename "$log_file")" "$log_dir/realrobot_latest.log"

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
printf 'Started: %s\nLog file: %s\n' "$(date -Is)" "$log_file" | tee "$log_file"
roslaunch rm75_roh_moveit_config realrobot.launch "$@" 2>&1 | tee -a "$log_file"
