#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启动即保持手臂零位(JointTrajectoryController 无目标时不输出力,
本节点持续发布零位轨迹保证手臂稳定受控)。手爪为刚性体,无需控制。"""
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4",
       "joint5", "joint6", "joint7"]


def main():
    rospy.init_node("hold_init_pose")

    def on_joint_states(msg):
        pass

    rospy.Subscriber("/joint_states", JointState, on_joint_states)
    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)

    rate = rospy.Rate(2.0)
    rospy.loginfo("hold_init_pose: 持续保持 手臂零位")
    while not rospy.is_shutdown():
        msg = JointTrajectory()
        msg.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * len(ARM)
        pt.time_from_start = rospy.Duration(4.0)
        msg.points = [pt]
        arm_pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
