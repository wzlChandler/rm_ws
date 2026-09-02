#!/usr/bin/env python3
# 增强 proximal PID 并检查 5 个 ERROR
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
txt = txt.replace("pid: {p: 8.0, i: 0.0, d: 0.5}", "pid: {p: 30.0, i: 0.0, d: 2.0}")
open(path, "w").write(txt)
print("proximal PID strengthened:", txt.count("p: 30.0, i: 0.0, d: 2.0"))
