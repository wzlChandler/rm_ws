#!/usr/bin/env python3
# 查询手爪关键 link 位置与苹果位置,判断抓取几何
import re
import rospy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import GetModelState

rospy.init_node("geom_check", anonymous=True)

links_of_interest = ["rm75_roh::hand_base_link", "rm75_roh::base_link",
                     "rm75_roh::if_slider_link", "rm75_roh::mf_slider_link",
                     "rm75_roh::rf_slider_link", "rm75_roh::lf_slider_link",
                     "rm75_roh::th_slider_link", "rm75_roh::Link7"]


def get_link_states():
    msg = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=8)
    out = {}
    for name, pose in zip(msg.name, msg.pose):
        if name in links_of_interest:
            p = pose.position
            out[name] = (p.x, p.y, p.z)
    return out


def get_apple():
    srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    r = srv("apple", "world")
    p = r.pose.position
    return (p.x, p.y, p.z)


states = get_link_states()
for k in links_of_interest:
    if k in states:
        print("%-32s %s" % (k, [round(v, 3) for v in states[k]]))
print("%-32s %s" % ("apple", [round(v, 3) for v in get_apple()]))
