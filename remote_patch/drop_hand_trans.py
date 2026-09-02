#!/usr/bin/env python3
# 删除 xacro 中所有手爪 transmission(手臂 7 个保留)
import re
path = "/home/test/rm_ws/src/my_work/rm75_roh_gazebo_sim/urdf/rm75_roh_sim.xacro"
txt = open(path).read()
before = txt.count("<transmission")
txt = re.sub(r"<transmission name=\"(?:if|mf|rf|lf|th)_[a-z_]+_trans\">.*?</transmission>\n?",
             "", txt, flags=re.S)
open(path, "w").write(txt)
print("transmissions before:", before, "after:", txt.count("<transmission"))
