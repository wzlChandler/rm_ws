#!/usr/bin/env python3
"""Capture the current real-robot pose once; never command the robot."""
import time

import rospy
from sensor_msgs.msg import JointState

ARM_JOINTS = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"}


def main():
    rospy.init_node("snapshot_pose")
    positions = {}
    deadline = time.monotonic() + 30.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            message = rospy.wait_for_message("/joint_states", JointState,
                                             timeout=1.0)
        except rospy.ROSException:
            continue
        positions.update(dict(zip(message.name, message.position)))
        if ARM_JOINTS.issubset(positions) and len(positions) > len(ARM_JOINTS):
            break
    if not ARM_JOINTS.issubset(positions):
        raise rospy.ROSException(
            "Timed out waiting for arm and hand data on /joint_states")

    snapshot = JointState()
    snapshot.name = sorted(positions)
    snapshot.position = [positions[name] for name in snapshot.name]
    publisher = rospy.Publisher("/apple_snapshot/joint_states", JointState,
                                queue_size=1, latch=True)
    snapshot.header.stamp = rospy.Time.now()
    publisher.publish(snapshot)
    rospy.loginfo("Captured %d current arm/hand joints; pose is now static.", len(snapshot.name))
    rospy.spin()


if __name__ == "__main__":
    main()
