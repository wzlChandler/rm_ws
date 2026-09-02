#!/usr/bin/env python3
# 手爪固化为刚体后,清理手爪 transmission/控制器
import re

# 1) xacro:删除所有手爪 transmission(slider/th_root/proximal)
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
txt = re.sub(r"  <!-- 手爪驱动关节传动.*?</transmission>\n", "", txt, flags=re.S)
txt = re.sub(r"  <!-- 手指弯曲关节\(proximal\)传动.*?</transmission>\n", "", txt, flags=re.S)
# 也删除残留的单个 transmission 块(按 joint 名)
for j in ["if_slider_link", "mf_slider_link", "rf_slider_link", "lf_slider_link",
          "th_slider_link", "th_root_link", "if_proximal_link",
          "mf_proximal_link", "rf_proximal_link", "lf_proximal_link",
          "th_proximal_link"]:
    txt = re.sub(r"<transmission name=\"" + j + r"_[a-z]+_trans\">.*?</transmission>\n",
                 "", txt, flags=re.S)
# 也删掉手爪的 maxEffort/maxVelocity 与被动关节阻尼标签(fixed 后无效,且引用名已 _j)
open(path, "w").write(txt)
n_trans = txt.count("<transmission")
print("remaining transmissions in xacro:", n_trans)

# 2) yaml:只保留 joint_state_controller + arm_joint_controller + gazebo_ros_control
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
# 保留第一段(arm 控制器)到 "arm_joint_controller:" 之后的所有 joint 配置,删掉手爪部分
# 简单方案:按控制器块切分
lines = txt.splitlines()
out = []
keep = True
for line in lines:
    if line.startswith("if_slider_controller"):
        keep = False
    if line.startswith("joint_state_controller") or line.startswith("arm_joint_controller"):
        keep = True
    if keep:
        out.append(line)
txt = "\n".join(out)
# 清理 gazebo_ros_control pid_gains 中的手爪项
txt = re.sub(r"    (?:if|mf|rf|lf|th)_[a-z_]+_j: \{p: [0-9.]+, i: 0.0, d: [0-9.]+\}\n", "", txt)
# th_root 的控制器段(在 hand 段里,已经被切掉),pid_gains 的 th_root 保留?th_root 也 fixed,删
txt = re.sub(r"    th_root_link_j: \{p: [0-9.]+, i: 0.0, d: [0-9.]+\}\n", "", txt)
txt = txt.rstrip() + "\n"
open(path, "w").write(txt)
print("yaml controllers left:")
for line in txt.splitlines():
    if line.startswith(("joint_state_controller", "arm_joint_controller", "gazebo_ros_control", "  pid_gains", "    joint")):
        print("  ", line)

# 3) launch:spawner 只加载 arm 控制器
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/launch/gazebo.launch"
txt = open(path).read()
txt = re.sub(r'args="joint_state_controller arm_joint_controller.*?"',
             'args="joint_state_controller arm_joint_controller"', txt, flags=re.S)
open(path, "w").write(txt)
print("spawner simplified")
