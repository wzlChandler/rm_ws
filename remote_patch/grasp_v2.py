#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""刚体手爪抓取实验v2:苹果放到指尖钳口,手臂上收提起"""
import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def get_apple(srv):
    r = srv("apple", "world")
    p = r.pose.position
    return [p.x, p.y, p.z]


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


def finger_center(listener):
    """4 指与拇指指尖的中心(钳口)"""
    pts = []
    for f in ["if_distal_link", "mf_distal_link", "rf_distal_link",
              "lf_distal_link", "th_distal_link"]:
        try:
            (p, _) = listener.lookupTransform("world", f, rospy.Time(0))
            pts.append(p)
        except Exception:
            pass
    if not pts:
        return None
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    cz = min(p[2] for p in pts)
    return [cx, cy, cz]


def main():
    rospy.init_node("grasp_v2")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    listener = tf.TransformListener()
    rospy.sleep(2.0)

    rospy.loginfo("初始苹果: %s", get_apple(get))

    # 苹果放到指尖钳口中心
    c = finger_center(listener)
    if c is None:
        rospy.logerr("无法获取指尖位置")
        return
    rospy.loginfo("指尖钳口中心: %s", [round(v, 3) for v in c])
    set_apple(set_srv, c[0], c[1], 0.155)
    rospy.sleep(2.0)
    rospy.loginfo("重置后苹果: %s", get_apple(get))

    # 手臂先微下探(joint6 微调),让手爪贴近苹果
    arm_move(arm_pub, [0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0], 3.0)
    rospy.sleep(4.0)
    rospy.loginfo("下探后苹果: %s", get_apple(get))

    # 手臂上收,提起
    arm_move(arm_pub, [0.0, 0.7, -0.5, 0.0, 0.0, 0.5, 0.0], 5.0)
    rospy.sleep(6.0)
    a2 = get_apple(get)
    rospy.loginfo("提起后苹果: %s", a2)
    if a2:
        dz = a2[2] - 0.155
        rospy.loginfo("苹果 z 变化 %.3f m %s", dz,
                      "-> 抓取成功!" if dz > 0.03 else "-> 未抓起")


if __name__ == "__main__":
    main()
