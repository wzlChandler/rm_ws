#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探索手臂关节角 -> 手爪位置 的映射,找对准苹果(0,0,0.07)的姿态"""
import rospy
import tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def move(pub, pos, dur=4.0):
    msg = JointTrajectory()
    msg.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in pos]
    pt.time_from_start = rospy.Duration(dur)
    msg.points = [pt]
    pub.publish(msg)


def hand_pos(listener):
    try:
        (p, _) = listener.lookupTransform("world", "hand_base_link",
                                          rospy.Time(0))
        return p
    except Exception:
        return None


def main():
    rospy.init_node("explore")
    pub = rospy.Publisher("/arm_joint_controller/command",
                          JointTrajectory, queue_size=1)
    listener = tf.TransformListener()
    rospy.sleep(1.0)

    # 从当前姿态出发,测试 joint2/joint3 组合
    tests = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.3, -0.3, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.6, -0.6, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.9, -0.9, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.2, -1.2, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.5, -1.5, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.6, -0.6, 0.0, 0.0, 0.5, 0.0],
        [0.0, 0.9, -0.9, 0.0, 0.0, 1.0, 0.0],
    ]
    for t in tests:
        move(pub, t, 3.0)
        rospy.sleep(4.0)
        h = hand_pos(listener)
        if h:
            print("j2=%.1f j3=%.1f j6=%.1f -> hand (%7.3f, %7.3f, %7.3f)"
                  % (t[1], t[2], t[5], h[0], h[1], h[2]))


if __name__ == "__main__":
    main()
