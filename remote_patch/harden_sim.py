#!/usr/bin/env python3
# 加固:提高手臂/手爪 PID 阻尼,拇指被动关节阻尼
# 1) yaml
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
txt = txt.replace("d: 0.1, i_clamp: 0.0", "d: 0.5, i_clamp: 0.0")
txt = txt.replace("joint: th_root_link_j\n  pid: {p: 30.0, i: 0.0, d: 1.0}",
                  "joint: th_root_link_j\n  pid: {p: 40.0, i: 0.0, d: 8.0}")
txt = txt.replace("th_root_link_j: {p: 30.0, i: 0.0, d: 1.0}",
                  "th_root_link_j: {p: 40.0, i: 0.0, d: 8.0}")
open(path, "w").write(txt)
print("yaml PID updated")

# 2) 拇指被动关节 damping 1.0 -> 3.0
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
for j in ["th_proximal_link", "th_connecting_link", "th_distal_link"]:
    old = '<gazebo reference="%s"><implicitSpringDamper>true</implicitSpringDamper><damping>1.0</damping></gazebo>' % j
    new = '<gazebo reference="%s"><implicitSpringDamper>true</implicitSpringDamper><damping>3.0</damping></gazebo>' % j
    if old in txt:
        txt = txt.replace(old, new)
open(path, "w").write(txt)
print("thumb damping updated:", txt.count("<damping>3.0</damping>"))

# 3) 手爪质量放大 50 -> 20
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/scripts/rename_rohand_joints.py"
txt = open(path).read()
txt = txt.replace("MASS_SCALE = 50.0", "MASS_SCALE = 20.0")
open(path, "w").write(txt)
print("MASS_SCALE updated")
