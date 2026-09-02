#!/usr/bin/env python3
# 加固2:手指被动关节用弹簧固定(implicitSpringDamper stiffness+damping),
# 手臂物理阻尼提高,th_root PID 加强
import re

path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()

# 1) 全部 19 个手爪被动关节:stiffness 30 + damping 5
for tag in re.findall(r'<gazebo reference="(?:if|mf|rf|lf|th)_[a-z_]+">'
                      r'<implicitSpringDamper>true</implicitSpringDamper>'
                      r'<damping>[0-9.]+</damping></gazebo>', txt):
    new = tag.replace("</damping>", "</damping><stiffness>30</stiffness>")
    new = re.sub(r"<damping>[0-9.]+</damping>", "<damping>5</damping>", new)
    txt = txt.replace(tag, new)

# 2) 手臂物理阻尼 1.0->2.0,joint7 3.0->6.0
for j in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
    old = '<gazebo reference="%s"><implicitSpringDamper>true</implicitSpringDamper><damping>1.0</damping></gazebo>' % j
    new = '<gazebo reference="%s"><implicitSpringDamper>true</implicitSpringDamper><damping>2.0</damping></gazebo>' % j
    txt = txt.replace(old, new)
old = '<gazebo reference="joint7"><implicitSpringDamper>true</implicitSpringDamper><damping>3.0</damping></gazebo>'
new = '<gazebo reference="joint7"><implicitSpringDamper>true</implicitSpringDamper><damping>6.0</damping></gazebo>'
txt = txt.replace(old, new)

open(path, "w").write(txt)
print("passive joints spring-fixed:", txt.count("<stiffness>30</stiffness>"))
print("arm damping 2.0:", txt.count("<damping>2.0</damping>"))

# 3) yaml:th_root d 8->20
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
txt = txt.replace("pid: {p: 40.0, i: 0.0, d: 8.0}", "pid: {p: 40.0, i: 0.0, d: 20.0}")
open(path, "w").write(txt)
print("th_root PID updated")

# 4) 手爪质量 20 -> 10
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/scripts/rename_rohand_joints.py"
txt = open(path).read()
txt = txt.replace("MASS_SCALE = 20.0", "MASS_SCALE = 10.0")
open(path, "w").write(txt)
print("MASS_SCALE 10")
