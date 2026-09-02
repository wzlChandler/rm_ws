#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 stdin 的 URDF 中 ROH-A001 手爪改造为刚性"握持"手:
- joint 名加 _j 后缀
- 所有手爪关节转 fixed,手指固化为弯曲姿态(proximal origin 加弯曲角)
- 去掉 STL 网格碰撞,改为手掌一个大"碗"形碰撞盒(可托住苹果)
- link 名与 parent/child 引用保持原样
"""
import re
import sys

PREFIX = re.compile(r"(if|mf|rf|lf|th)_[a-z_]+")
MASS_SCALE = 2.0
BEND_ANGLE = 0.9

# 手掌碗形碰撞盒(挂在 hand_base_link 上):[尺寸x,y,z, 偏移x,y,z]
PALM_BOX = (0.15, 0.15, 0.10, 0.0, 0.0, -0.03)


def _rename(value):
    return PREFIX.sub(lambda m: m.group(0) + "_j", value)


def _remove_collisions(link_body):
    while True:
        m = re.search(r"<collision>.*?</collision>", link_body, flags=re.S)
        if not m:
            break
        link_body = link_body[:m.start()] + link_body[m.end():]
    return link_body


def _add_palm_box(link_body):
    sx, sy, sz, ox, oy, oz = PALM_BOX
    box = ('<collision><origin xyz="%g %g %g" rpy="0 0 0"/>'
           '<geometry><box size="%g %g %g"/></geometry></collision>'
           % (ox, oy, oz, sx, sy, sz))
    idx = link_body.find("</inertial>")
    if idx >= 0:
        link_body = (link_body[:idx + len("</inertial>")] + box
                     + link_body[idx + len("</inertial>"):])
    return link_body


def _bend_origin(body):
    m = re.search(r"<origin[^>]*rpy=\"([^\"]+)\"[^>]*>", body)
    if not m:
        return body
    rpy = [float(x) for x in m.group(1).split()]
    rpy[1] += BEND_ANGLE
    return body[:m.start(1)] + " ".join("%g" % v for v in rpy) + body[m.end(1):]


def main():
    text = sys.stdin.read()

    # 1) joint 名加 _j
    text = re.sub(r'(<joint name=")([^"]+)(")',
                  lambda m: m.group(1) + _rename(m.group(2)) + m.group(3), text)
    text = re.sub(r'(<gazebo reference=")([^"]+)(")',
                  lambda m: m.group(1) + _rename(m.group(2)) + m.group(3), text)

    # 2) 手爪关节全部 fixed,proximal 加弯曲角
    def _joint_proc(m):
        name, body = m.group(2), m.group(3)
        body = re.sub(r"\s*<axis[^>]*/>", "", body)
        body = re.sub(r"\s*<limit[^>]*/>", "", body)
        if name.endswith("_proximal_link_j"):
            body = _bend_origin(body)
        return '<joint name="%s" type="fixed">%s</joint>' % (name, body)

    text = re.sub(r'(<joint name="((?:if|mf|rf|lf|th)_[a-z_]+_j)"\s+type="(?:continuous|prismatic|revolute)">)(.*?)(</joint>)',
                  _joint_proc, text, flags=re.S)

    # 3) 手爪 link:质量放大 + 去 STL 碰撞;hand_base_link 加碗形碰撞盒
    def _link_proc(m):
        name, body = m.group(2), m.group(3)
        body = re.sub(r'(<mass\s+value=")([^"]+)(")',
                      lambda mm: mm.group(1)
                      + str(float(mm.group(2)) * MASS_SCALE) + mm.group(3), body)
        body = _remove_collisions(body)
        if name == "hand_base_link":
            body = _add_palm_box(body)
        return '<link name="%s">%s</link>' % (name, body)

    text = re.sub(
        r'(<link name="((?:if|mf|rf|lf|th)_[a-z_]+|hand_base_link)">)(.*?)(</link>)',
        _link_proc, text, flags=re.S)

    sys.stdout.write(text)


if __name__ == "__main__":
    main()
