#!/usr/bin/env python3
# 回到稳定配置:质量x10、无积分、提高 P
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
txt = txt.replace("{p: 1000.0, i: 0.01, d: 0.5, i_clamp: 1.0}",
                  "{p: 2000.0, i: 0.0, d: 1.0, i_clamp: 0.0}")
txt = txt.replace("{p: 2500.0, i: 0.01, d: 3.0, i_clamp: 1.0}",
                  "{p: 2500.0, i: 0.0, d: 3.0, i_clamp: 0.0}")
txt = txt.replace("{p: 1000.0, i: 0.01, d: 0.5}",
                  "{p: 2000.0, i: 0.0, d: 1.0}")
txt = txt.replace("{p: 2500.0, i: 0.01, d: 3.0}",
                  "{p: 2500.0, i: 0.0, d: 3.0}")
open(path, "w").write(txt)
print("yaml updated")

path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/scripts/rename_rohand_joints.py"
txt = open(path).read()
txt = txt.replace("MASS_SCALE = 1.0", "MASS_SCALE = 10.0")
open(path, "w").write(txt)
print("MASS_SCALE 10")
