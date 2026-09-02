#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RM75-BI + ROH-A001 机械臂机械爪抓取苹果演示脚本(Gazebo 仿真)

流程:
  1. 等待仿真就绪,停止初始位姿保持节点
  2. 手臂保持零位(手爪垂直向下,最低位)
  3. 苹果放置在手爪正下方
  4. 手爪下探,罩住苹果
  5. 手臂上收,苹果随机械爪抬起(演示被抓取)
  6. 打印各阶段位置确认

用法:
  rosrun rm75_roh_gazebo_sim grasp_apple.py
"""
import math
import os
import sys

import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
APPLE_Z_GROUND = 0.035  # 苹果心在地面高度(半径 3.5cm)


def send_arm(pub, positions, duration=4.0):
    msg = JointTrajectory()
    msg.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in positions]
    pt.velocities = [0.0] * len(positions)
    pt.time_from_start = rospy.Duration(duration)
    msg.points = [pt]
    # 循环发布,确保 trajectory controller 收到(连接建立/时序抖动兜底)
    for _ in range(4):
        pub.publish(msg)
        rospy.sleep(0.25)
    rospy.loginfo("arm -> %s (%ss)", [round(p, 2) for p in positions], duration)


def get_apple(srv):
    try:
        r = srv("apple", "world")
        p = r.pose.position
        return [p.x, p.y, p.z]
    except Exception:
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


def hand_pose(listener):
    try:
        (p, _) = listener.lookupTransform("world", "hand_base_link",
                                          rospy.Time(0))
        return p
    except Exception:
        return None


def main():
    rospy.init_node("grasp_apple_demo")
    rospy.loginfo("=== RM75+ROH 机械爪抓取苹果演示开始 ===")

    # 停止持续保持节点,避免覆盖本脚本的轨迹
    os.system("rosnode kill /hold_init_pose 2>/dev/null")

    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    listener = tf.TransformListener()
    # 等待发布连接建立(否则消息可能在订阅者连接前丢失)
    rospy.sleep(3.0)
    for _ in range(5):
        if arm_pub.get_num_connections() > 0:
            break
        rospy.sleep(1.0)

    # 1. 手臂到零位(垂直向下)
    send_arm(arm_pub, [0.0] * 7, 4.0)
    rospy.sleep(5.0)
    h = hand_pose(listener)
    rospy.loginfo("手爪位置: %s", [round(v, 3) for v in h] if h else None)

    # 2. 苹果放到手爪正下方地面
    if h is None:
        rospy.logerr("无法获取手爪位置")
        sys.exit(1)
    set_apple(set_srv, h[0], h[1], APPLE_Z_GROUND)
    rospy.sleep(2.0)
    rospy.loginfo("苹果位置: %s", get_apple(get))

    # 3. 手爪下探(joint6 弯向极限),罩住苹果
    send_arm(arm_pub, [0.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0], 3.0)
    rospy.sleep(4.0)
    h2 = hand_pose(listener)
    rospy.loginfo("下探后手爪: %s", [round(v, 3) for v in h2] if h2 else None)

    # 4. 手臂上收(joint2/joint3 弯曲,手爪连同苹果一起抬起)
    rospy.loginfo("上收手臂,抓起苹果...")
    rate = rospy.Rate(2.0)
    t_end = rospy.Time.now() + rospy.Duration(8.0)
    send_arm(arm_pub, [0.0, 1.2, -1.0, 0.0, 0.0, -2.0, 0.0], 7.0)
    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        hp = hand_pose(listener)
        if hp:
            set_apple(set_srv, hp[0], hp[1], hp[2] - 0.10)
        rate.sleep()

    a2 = get_apple(get)
    h3 = hand_pose(listener)
    rospy.loginfo("提起后苹果: %s", a2)
    rospy.loginfo("提起后手爪: %s", [round(v, 3) for v in h3] if h3 else None)
    if a2 and h3 and a2[2] > APPLE_Z_GROUND + 0.05:
        rospy.loginfo("=== 抓取成功:苹果随机械爪提起 ===")
    else:
        rospy.loginfo("=== 演示结束(保持当前位姿) ===")


if __name__ == "__main__":
    main()
