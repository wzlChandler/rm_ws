#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RM75-BI + ROH-A001 机械臂机械爪抓取苹果演示脚本(Gazebo 仿真)

流程:
  1. 等待仿真与控制器就绪
  2. 手臂运动到预抓取位姿(倒挂时零位即垂直向下,位于苹果正上方)
  3. 手爪张开(slider = 0)
  4. 手爪闭合(slider = 上限),环抱夹住苹果
  5. 手臂微收(弯曲 joint2/joint3),演示"抓起苹果"
  6. 打印苹果与手爪位置,确认抓取成功

用法:
  rosrun rm75_roh_gazebo_sim grasp_apple.py
"""
import math
import sys
import time

import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4",
              "joint5", "joint6", "joint7"]
# 手爪 slider 关节:0 = 张开,上限 = 闭合(与 ROH-A001 真机一致)
# 注:仿真中手爪关节名带 _j 后缀(规避 gazebo link/joint 同名冲突)
SLIDERS = {"if_slider_link_j": 0.019, "mf_slider_link_j": 0.019,
           "rf_slider_link_j": 0.019, "lf_slider_link_j": 0.019,
           "th_slider_link_j": 0.010}
TH_ROOT = "th_root_link_j"

# 预抓取/抓取/抬起的手臂关节目标(倒挂:零位即垂直向下)
ARM_PREGrasp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
ARM_LIFT = [0.0, 0.45, -0.35, 0.0, 0.0, 0.0, 0.0]  # 微收,手爪上移


def send_arm_traj(pub, positions, duration):
    msg = JointTrajectory()
    msg.joint_names = ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = [float(p) for p in positions]
    pt.velocities = [0.0] * len(positions)
    pt.time_from_start = rospy.Duration(duration)
    msg.points = [pt]
    pub.publish(msg)
    rospy.loginfo("arm trajectory -> %s (%ss)", positions, duration)


def set_slider(pub, value):
    pub.publish(Float64(value))


def wait_for_joints(timeout=30.0):
    """等待 /joint_states 就绪且无 NaN"""
    got = {}

    def cb(msg):
        for n, p in zip(msg.name, msg.position):
            if not math.isnan(p):
                got[n] = p

    rospy.Subscriber("/joint_states", JointState, cb)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if all(n in got for n in ARM_JOINTS + list(SLIDERS) + [TH_ROOT]):
            return got
        rospy.sleep(0.2)
    rospy.logwarn("joint states not fully ready after %ss", timeout)
    return got


def get_apple_pose():
    """通过 gazebo 模型状态服务获取苹果位置"""
    from gazebo_msgs.srv import GetModelState
    try:
        rospy.wait_for_service("/gazebo/get_model_state", timeout=5)
        srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        resp = srv("apple", "world")
        if resp.success:
            p = resp.pose.position
            return [p.x, p.y, p.z]
    except Exception as e:
        rospy.logwarn("cannot get apple pose: %s", e)
    return None


def get_hand_pose(listener):
    """读取 hand_base_link 在世界系的位置(倒挂时位于苹果上方)"""
    try:
        (pos, _) = listener.lookupTransform("world", "hand_base_link",
                                            rospy.Time(0))
        return pos
    except Exception:
        return None


def main():
    rospy.init_node("grasp_apple_demo")
    rospy.loginfo("=== RM75+ROH 抓取苹果仿真演示开始 ===")

    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    slider_pubs = {}
    for name in SLIDERS:
        slider_pubs[name] = rospy.Publisher(
            "/%s_controller/command" % name, Float64, queue_size=1)
    th_pub = rospy.Publisher("/th_root_controller/command", Float64,
                             queue_size=1)

    rospy.sleep(1.0)
    listener = tf.TransformListener()
    rospy.sleep(1.0)

    joints = wait_for_joints()
    if not joints:
        rospy.logerr("关节状态未就绪,退出")
        sys.exit(1)

    apple = get_apple_pose()
    hand = get_hand_pose(listener)
    rospy.loginfo("苹果位置: %s", apple)
    rospy.loginfo("手爪位置: %s", hand)

    # 1. 手臂到预抓取位姿(零位:垂直向下)
    send_arm_traj(arm_pub, ARM_PREGrasp, 3.0)
    rospy.sleep(4.0)

    # 2. 张开手爪
    rospy.loginfo("张开手爪...")
    for name, pub in slider_pubs.items():
        set_slider(pub, 0.0)
    set_slider(th_pub, 0.0)
    rospy.sleep(2.0)

    # 3. 闭合手爪,夹住苹果
    rospy.loginfo("闭合手爪,抓取苹果...")
    for name, pub in slider_pubs.items():
        set_slider(pub, SLIDERS[name])
    set_slider(th_pub, 0.5)
    rospy.sleep(3.0)

    # 4. 手臂微收,将苹果提起
    rospy.loginfo("手臂微收,提起苹果...")
    send_arm_traj(arm_pub, ARM_LIFT, 4.0)
    rospy.sleep(5.0)

    apple2 = get_apple_pose()
    hand2 = get_hand_pose(listener)
    rospy.loginfo("抓取后苹果位置: %s", apple2)
    rospy.loginfo("抓取后手爪位置: %s", hand2)
    if apple and apple2 and abs(apple2[2] - apple[2]) > 0.02:
        rospy.loginfo("=== 抓取成功:苹果随机械爪抬起 %.3f m ===",
                      apple2[2] - apple[2])
    else:
        rospy.logwarn("苹果位置变化不明显,请检查苹果/手爪相对位置")
    rospy.loginfo("=== 演示结束(保持当前位姿) ===")


if __name__ == "__main__":
    main()
