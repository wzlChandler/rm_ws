#!/usr/bin/env python3
# 修复 sim xacro:proximal transmission 的 joint name 去掉 _j(由 rename 脚本统一加)
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
for finger in ["if", "mf", "rf", "lf", "th"]:
    txt = txt.replace("<joint name=\"%s_proximal_link_j\">" % finger,
                      "<joint name=\"%s_proximal_link\">" % finger)
open(path, "w").write(txt)
print("proximal trans names fixed:", txt.count("proximal_link\">") >= 5)
