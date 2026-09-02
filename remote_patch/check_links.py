#!/usr/bin/env python3
# 检查 gazebo link_states,找出乱飞的 link(物理爆炸诊断)
import re
import sys

txt = open("/tmp/ls.txt").read().split("---")[0]
blocks = re.split(r"- name: ", txt)
count = 0
for b in blocks[1:]:
    nm = b.split("\n")[0].strip().strip("'")
    pos = re.search(r"x: ([-\d.e]+)\s+y: ([-\d.e]+)\s+z: ([-\d.e]+)", b)
    if pos:
        count += 1
        x, y, z = [float(pos.group(i)) for i in (1, 2, 3)]
        if abs(z) > 3 or abs(x) > 3 or abs(y) > 3:
            print("FLYING:", nm, round(x, 2), round(y, 2), round(z, 2))
print("total links:", count)
