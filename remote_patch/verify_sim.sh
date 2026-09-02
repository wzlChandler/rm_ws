#!/bin/bash
# 远程验证流程:重启仿真 -> 检查关节 -> 跑抓取脚本 -> 汇总
set -e
source /opt/ros/noetic/setup.bash
source ~/rm_ws/devel/setup.bash

# 杀掉残留进程
pkill -f "python3 /opt/ros/noetic/bin/roslaunch rm75_roh_gazebo_sim" 2>/dev/null || true
pkill -f "bin/gzserver" 2>/dev/null || true
pkill -f "bin/rosmaster" 2>/dev/null || true
pkill -f "bin/gzclient" 2>/dev/null || true
sleep 3

# 启动 headless 仿真
setsid roslaunch rm75_roh_gazebo_sim gazebo.launch headless:=true gui:=false \
  > /tmp/sim_test.log 2>&1 < /dev/null &
echo "launched"

# 等待就绪
for i in $(seq 1 40); do
  if timeout 3 rostopic echo -n1 /joint_states > /tmp/js0.txt 2>/dev/null; then
    echo "joint_states ready at $i"
    break
  fi
  sleep 3
done

echo "=== joint_states (初始) ==="
sed -n '/position:/,/velocity:/p' /tmp/js0.txt | head -4

echo "=== 错误统计 ==="
grep -cE "\[ERROR\]" /tmp/sim_test.log || true
