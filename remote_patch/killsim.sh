#!/bin/bash
# 彻底清理所有仿真相关进程(文件名不含匹配模式,避免误杀自身)
pkill -9 -f "roslaunch rm75_roh_gazebo_sim" 2>/dev/null
pkill -9 -f "controller_manager/spawne[r]" 2>/dev/null
pkill -9 -f "hold_init_pos[e]" 2>/dev/null
pkill -9 -f "robot_state_publishe[r]" 2>/dev/null
pkill -9 -f "gzserve[r]" 2>/dev/null
pkill -9 -f "gzclien[t]" 2>/dev/null
pkill -9 -f "rosmaste[r]" 2>/dev/null
pkill -9 -f "rosco[r]e" 2>/dev/null
sleep 4
echo "=== remaining (should be none) ==="
ps -eo pid,cmd | grep -E "roslaunch|gzserve[r]|rosmaste[r]|spawne[r]|hold_init|robot_state_pub|rosco[r]e" | grep -v grep || echo "CLEAN - no processes left"
