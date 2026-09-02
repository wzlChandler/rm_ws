#!/usr/bin/env python3
# 降低桌子与苹果,避免初始与手爪干涉
# 1) 桌子腿缩短
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/models/table.sdf"
txt = open(path).read()
txt = txt.replace("<size>0.03 0.03 0.16</size>", "<size>0.03 0.03 0.10</size>")
txt = txt.replace("pose>-0.13 -0.13 -0.09", "pose>-0.13 -0.13 -0.05")
txt = txt.replace("pose>0.13 -0.13 -0.09", "pose>0.13 -0.13 -0.05")
txt = txt.replace("pose>-0.13 0.13 -0.09", "pose>-0.13 0.13 -0.05")
txt = txt.replace("pose>0.13 0.13 -0.09", "pose>0.13 0.13 -0.05")
open(path, "w").write(txt)
print("table legs shortened:", txt.count("<size>0.03 0.03 0.10</size>"))

# 2) launch:桌子 z 0.16->0.10,苹果 z 0.215->0.155
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/launch/gazebo.launch"
txt = open(path).read()
txt = txt.replace('<arg name="apple_z" default="0.215"/>', '<arg name="apple_z" default="0.155"/>')
old = 'args="-sdf -file $(find rm75_roh_gazebo_sim)/models/table.sdf -model table -x 0 -y 0 -z 0.16"'
new = 'args="-sdf -file $(find rm75_roh_gazebo_sim)/models/table.sdf -model table -x 0 -y 0 -z 0.10"'
assert old in txt
txt = txt.replace(old, new)
open(path, "w").write(txt)
print("launch updated")
