#!/usr/bin/env python3
# 降低 base 高度:world_joint origin z 1.0 -> 0.72
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
old = '<origin xyz="0 0 1.0" rpy="3.14159 0 0"/>'
new = '<origin xyz="0 0 0.72" rpy="3.14159 0 0"/>'
if old in txt:
    txt = txt.replace(old, new)
    open(path, "w").write(txt)
    print("base height lowered to 0.72")
else:
    print("pattern not found!")
