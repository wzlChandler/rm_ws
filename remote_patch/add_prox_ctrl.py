#!/usr/bin/env python3
# 在控制 yaml 中追加 5 个 proximal 弯曲关节位置控制器
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/config/rm75_roh_control.yaml"
txt = open(path).read()
add = """
# 手指弯曲关节(proximal)位置控制器
if_proximal_controller:
  type: position_controllers/JointPositionController
  joint: if_proximal_link_j
  pid: {p: 8.0, i: 0.0, d: 0.5}
mf_proximal_controller:
  type: position_controllers/JointPositionController
  joint: mf_proximal_link_j
  pid: {p: 8.0, i: 0.0, d: 0.5}
rf_proximal_controller:
  type: position_controllers/JointPositionController
  joint: rf_proximal_link_j
  pid: {p: 8.0, i: 0.0, d: 0.5}
lf_proximal_controller:
  type: position_controllers/JointPositionController
  joint: lf_proximal_link_j
  pid: {p: 8.0, i: 0.0, d: 0.5}
th_proximal_controller:
  type: position_controllers/JointPositionController
  joint: th_proximal_link_j
  pid: {p: 8.0, i: 0.0, d: 0.5}
"""
txt = txt.rstrip() + "\n" + add
# pid_gains 也要加(在 gazebo_ros_control 段)
txt = txt.replace("    th_root_link_j: {p: 40.0, i: 0.0, d: 20.0}",
                  "    th_root_link_j: {p: 40.0, i: 0.0, d: 20.0}\n"
                  "    if_proximal_link_j: {p: 8.0, i: 0.0, d: 0.5}\n"
                  "    mf_proximal_link_j: {p: 8.0, i: 0.0, d: 0.5}\n"
                  "    rf_proximal_link_j: {p: 8.0, i: 0.0, d: 0.5}\n"
                  "    lf_proximal_link_j: {p: 8.0, i: 0.0, d: 0.5}\n"
                  "    th_proximal_link_j: {p: 8.0, i: 0.0, d: 0.5}")
open(path, "w").write(txt)
print("proximal controllers added")

# launch 里 spawner 也要加 5 个控制器
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/launch/gazebo.launch"
txt = open(path).read()
old = "args=\"joint_state_controller arm_joint_controller\n              if_slider_controller mf_slider_controller rf_slider_controller lf_slider_controller\n              th_slider_controller th_root_controller\""
new = ("args=\"joint_state_controller arm_joint_controller\n"
       "              if_slider_controller mf_slider_controller rf_slider_controller lf_slider_controller\n"
       "              th_slider_controller th_root_controller\n"
       "              if_proximal_controller mf_proximal_controller rf_proximal_controller\n"
       "              lf_proximal_controller th_proximal_controller\"")
if old not in txt:
    print("WARN: spawner arg block not found, checking...")
    for line in txt.splitlines():
        if "args=" in line:
            print("  ", line.strip())
else:
    txt = txt.replace(old, new)
    open(path, "w").write(txt)
    print("spawner updated")
