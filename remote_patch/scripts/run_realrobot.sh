#!/usr/bin/env bash
set -o pipefail

workspace=/home/test/rm_ws
log_dir="$workspace/logs"
timestamp=$(date +%Y%m%d_%H%M%S)
log_file="$log_dir/realrobot_${timestamp}.log"

mkdir -p "$log_dir"

exec 9>"$workspace/.realrobot.lock"
if ! flock -n 9; then
  printf 'realrobot is already starting or running.\n' >&2
  exit 1
fi

existing_launch=""
while read -r launch_pid; do
  launch_state=$(ps -o stat= -p "$launch_pid")
  if [[ "$launch_state" != Z* ]]; then
    existing_launch+="$launch_pid "
  fi
done < <(pgrep -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm75_roh_moveit_config realrobot.launch( |$)' || true)
if [[ -n "$existing_launch" ]]; then
  printf 'Existing realrobot roslaunch PID(s): %s\nStop the old launch before starting another one.\n' "$existing_launch" >&2
  exit 1
fi

ln -sfn "$(basename "$log_file")" "$log_dir/realrobot_latest.log"

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
printf 'Started: %s\nLog file: %s\n' "$(date -Is)" "$log_file" | tee "$log_file"
roslaunch rm75_roh_moveit_config realrobot.launch "$@" 2>&1 | tee -a "$log_file"
