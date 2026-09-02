#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取实验:把苹果放回手爪正下方,闭合手爪,手臂微收,验证苹果被抓起"""
import math
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
SLIDERS = {"if_slider_link_j": 0.019, "mf_slider_link_j": 0.019,
           "rf_slider_link_j": 0.019, "lf_slider_link_j": 0.019,
           "th_slider_link_j": 0.010}


def get_apple(srv):
    r = srv("apple", "world")
    if r.success:
        p = r.pose.position
        return [p.x, p.y, p.z]
    return None


def set_apple(srv, x, y, z):
    ms = ModelState()
    ms.model_name = "apple"
    ms.pose.position.x = x
    ms.pose.position.y = y
    ms.pose.position.z = z
    ms.pose.orientation.w = 1.0
    ms.reference_frame = "world"
    srv(ms)


def main():
    rospy.init_node("grasp_experiment")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    pubs = {}
    for n in SLIDERS:
        pubs[n] = rospy.Publisher("/%s_controller/command" % n, Float64, queue_size=1)
    th_pub = rospy.Publisher("/th_root_controller/command", Float64, queue_size=1)
    rospy.sleep(1.0)

    rospy.loginfo("初始苹果位置: %s", get_apple(get))

    # 1. 苹果放回手爪正下方(桌面之上)
    set_apple(set_srv, 0.0, 0.0, 0.155)
    rospy.sleep(2.0)
    rospy.loginfo("重置后苹果位置: %s", get_apple(get))

    # 2. 手臂保持零位
    msg = JointTrajectory()
    msg.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * 7
    pt.time_from_start = rospy.Duration(3.0)
    msg.points = [pt]
    arm_pub.publish(msg)
    rospy.sleep(3.5)

    # 3. 张开手爪
    for n, pub in pubs.items():
        pub.publish(Float64(0.0))
    th_pub.publish(Float64(0.0))
    rospy.sleep(1.5)

    # 4. 闭合手爪(夹苹果)
    rospy.loginfo("闭合手爪...")
    for n, v in SLIDERS.items():
        pubs[n].publish(Float64(v))
    th_pub.publish(Float64(0.6))
    rospy.sleep(4.0)
    rospy.loginfo("闭合后苹果位置: %s", get_apple(get))

    # 5. 手臂微收提起
    msg = JointTrajectory()
    msg.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [0.0, 0.45, -0.35, 0.0, 0.0, 0.0, 0.0]
    pt.time_from_start = rospy.Duration(4.0)
    msg.points = [pt]
    arm_pub.publish(msg)
    rospy.sleep(5.0)
    apple2 = get_apple(get)
    rospy.loginfo("提起后苹果位置: %s", apple2)
    if apple2:
        dz = apple2[2] - 0.155
        rospy.loginfo("苹果 z 变化: %.3f m %s", dz,
                      "-> 抓取成功!" if dz > 0.02 else "-> 未抓起")


if __name__ == "__main__":
    main()
