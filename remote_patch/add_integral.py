#!/usr/bin/env python3
# 给手臂控制器增益加积分项,消除重力稳态误差
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
txt = txt.replace("{p: 1000.0, i: 0.0, d: 0.5, i_clamp: 0.0}",
                  "{p: 1000.0, i: 0.01, d: 0.5, i_clamp: 1.0}")
txt = txt.replace("{p: 2500.0, i: 0.0, d: 3.0, i_clamp: 0.0}",
                  "{p: 2500.0, i: 0.01, d: 3.0, i_clamp: 1.0}")
txt = txt.replace("{p: 1000.0, i: 0.0, d: 0.5}",
                  "{p: 1000.0, i: 0.01, d: 0.5}")
txt = txt.replace("{p: 2500.0, i: 0.0, d: 3.0}",
                  "{p: 2500.0, i: 0.01, d: 3.0}")
open(path, "w").write(txt)
print("i-gain count:", txt.count("i: 0.01"))
