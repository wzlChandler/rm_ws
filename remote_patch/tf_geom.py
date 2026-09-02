#!/usr/bin/env python3
# 用 tf 重建手爪几何:查各指尖相对 Link7/world 的位置
import rospy
import tf

rospy.init_node("tf_geom", anonymous=True)
l = tf.TransformListener()
rospy.sleep(2.0)

links = ["hand_base_link", "if_distal_link", "mf_distal_link",
         "rf_distal_link", "lf_distal_link", "th_distal_link"]
print("frame:      x        y        z   (world)")
for name in ["Link7"] + links:
    try:
        (p, _) = l.lookupTransform("world", name, rospy.Time(0))
        print("%-14s %7.3f %7.3f %7.3f" % (name, p[0], p[1], p[2]))
    except Exception as e:
        print("%-14s ERR %s" % (name, str(e)[:50]))
