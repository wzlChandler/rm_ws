#!/usr/bin/env python3
# 添加 world 固定关节,并简化 spawn 参数
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
add = """
  <!-- 固定基座到世界:base_link 位于 (0,0,1.0),绕 X 翻转 180 度(顶装倒挂) -->
  <link name="world"/>
  <joint name="world_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 1.0" rpy="3.14159 0 0"/>
  </joint>
"""
txt = txt.replace("</robot>", add + "</robot>")
open(path, "w").write(txt)
print("world joint added:", "world_joint" in txt)

path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/launch/gazebo.launch"
txt = open(path).read()
old = 'args="-urdf -model rm75_roh -param robot_description -z $(arg base_z) -R 3.14159 -P 0 -Y 0"'
new = 'args="-urdf -model rm75_roh -param robot_description"'
assert old in txt, "spawn args not found"
txt = txt.replace(old, new)
open(path, "w").write(txt)
print("spawn args simplified")
