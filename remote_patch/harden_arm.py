#!/usr/bin/env python3
# 给手臂关节加物理阻尼(implicitSpringDamper),提高 joint7 控制增益
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
add = ""
for j in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
    add += '  <gazebo reference="%s"><implicitSpringDamper>true</implicitSpringDamper><damping>1.0</damping></gazebo>\n' % j
add += '  <gazebo reference="joint7"><implicitSpringDamper>true</implicitSpringDamper><damping>3.0</damping></gazebo>\n'
# 插到 </robot> 之前
txt = txt.replace("</robot>", add + "</robot>")
open(path, "w").write(txt)
print("arm damping added:", txt.count("implicitSpringDamper"))

# yaml:joint7 gains 增强
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
txt = txt.replace("joint7: {p: 1000.0, i: 0.0, d: 0.5, i_clamp: 0.0}",
                  "joint7: {p: 2500.0, i: 0.0, d: 3.0, i_clamp: 0.0}")
txt = txt.replace("joint7: {p: 1000.0, i: 0.0, d: 0.1}",
                  "joint7: {p: 2500.0, i: 0.0, d: 3.0}")
open(path, "w").write(txt)
print("joint7 gains updated")
