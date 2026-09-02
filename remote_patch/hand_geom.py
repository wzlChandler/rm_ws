#!/usr/bin/env python3
# 查手爪所有关键 link 的世界位置,计算包围几何
import rospy
from gazebo_msgs.msg import LinkStates

rospy.init_node("hand_geom", anonymous=True)
msg = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=8)

keys = ["hand_base_link", "if_distal_link", "mf_distal_link", "rf_distal_link",
        "lf_distal_link", "th_distal_link", "if_proximal_link",
        "th_proximal_link", "Link7"]
for name, pose in zip(msg.name, msg.pose):
    for k in keys:
        if name == "rm75_roh::" + k:
            p = pose.position
            print("%-20s (%7.3f, %7.3f, %7.3f)" % (k, p.x, p.y, p.z))
