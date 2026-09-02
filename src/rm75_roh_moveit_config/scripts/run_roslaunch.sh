#!/usr/bin/env bash
set -o pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <package> <launch-file> [launch arguments...]" >&2
  exit 2
fi

workspace=/home/test/rm_ws
package=$1
launch_file=$2
shift 2
launch_name=${launch_file%.launch}
log_dir="$workspace/logs"
timestamp=$(date +%Y%m%d_%H%M%S)
log_file="$log_dir/${launch_name}_${timestamp}.log"

mkdir -p "$log_dir"
ln -sfn "$(basename "$log_file")" "$log_dir/${launch_name}_latest.log"

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
printf 'Started: %s\nLog file: %s\n' "$(date -Is)" "$log_file" | tee "$log_file"
roslaunch "$package" "$launch_file" "$@" 2>&1 | tee -a "$log_file"
