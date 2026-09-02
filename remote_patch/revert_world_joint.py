#!/usr/bin/env python3
# 去掉 world_joint,恢复 spawn 固定
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
import re
txt = re.sub(r"\n  <!-- 固定基座到世界.*?</joint>\n", "\n", txt, flags=re.S)
open(path, "w").write(txt)
print("world_joint removed:", "world_joint" not in txt)

path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/launch/gazebo.launch"
txt = open(path).read()
old = 'args="-urdf -model rm75_roh -param robot_description"'
new = 'args="-urdf -model rm75_roh -param robot_description -z $(arg base_z) -R 3.14159 -P 0 -Y 0"'
assert old in txt
txt = txt.replace(old, new)
open(path, "w").write(txt)
print("spawn args restored")
