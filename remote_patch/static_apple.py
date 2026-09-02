#!/usr/bin/env python3
# 苹果改为 static 且无碰撞(纯视觉),由抓取脚本控制位置演示"被抓取"
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/models/apple.sdf"
txt = open(path).read()
txt = txt.replace("<static>false</static>", "<static>true</static>")
# 删除所有 collision 块
import re
txt = re.sub(r"\s*<collision name=\".*?</collision>", "", txt, flags=re.S)
open(path, "w").write(txt)
print("static:", "<static>true</static>" in txt)
print("collision left:", txt.count("<collision"))
