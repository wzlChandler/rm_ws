#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""刚体手爪抓取实验:苹果放到手爪正下方,手臂上收,验证苹果被托起"""
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


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


def arm_move(pub, pos, dur=4.0):
    msg = JointTrajectory()
    msg.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in pos]
    pt.time_from_start = rospy.Duration(dur)
    msg.points = [pt]
    pub.publish(msg)


def main():
    rospy.init_node("rigid_grasp_test")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    rospy.sleep(1.0)

    rospy.loginfo("初始苹果: %s", get_apple(get))

    # 苹果放到手爪正下方(Link7 下方 ~0.17m)
    set_apple(set_srv, 0.15, 0.0, 0.150)
    rospy.sleep(2.0)
    rospy.loginfo("重置后苹果: %s", get_apple(get))

    # 手臂先下探一点(让手爪更贴近苹果)
    arm_move(arm_pub, [0.0, 0.0, 0.0, 0.0, 0.0, 1.9, 0.0], 3.0)
    rospy.sleep(4.0)
    rospy.loginfo("下探后苹果: %s", get_apple(get))

    # 手臂上收,带着苹果提起
    arm_move(arm_pub, [0.0, 0.6, -0.5, 0.0, 0.0, 0.3, 0.0], 5.0)
    rospy.sleep(6.0)
    a2 = get_apple(get)
    rospy.loginfo("提起后苹果: %s", a2)
    if a2:
        dz = a2[2] - 0.150
        rospy.loginfo("苹果 z 变化 %.3f m %s", dz,
                      "-> 抓取成功!" if dz > 0.03 else "-> 未抓起")


if __name__ == "__main__":
    main()
