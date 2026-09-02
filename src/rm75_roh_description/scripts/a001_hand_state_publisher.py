#!/usr/bin/env python3
"""Publish ROH-A001 visual joint states from the commands sent to rm_hand_driver."""

import os
import sys

import rospkg
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray, String, UInt16MultiArray

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path("rohand_urdf_ros1"), "scripts"))
from FingerMathURDF import HAND_FingerPosToAngle


SLIDER_LIMITS = [0.01, 0.019, 0.019, 0.019, 0.019]
JOINT_NAMES = [
    ["th_proximal_link", "th_slider_link", "th_connecting_link", "th_distal_link"],
    ["if_slider_link", "if_slider_abpart_link", "if_proximal_link", "if_distal_link", "if_connecting_link"],
    ["mf_slider_link", "mf_slider_abpart_link", "mf_proximal_link", "mf_distal_link", "mf_connecting_link"],
    ["rf_slider_link", "rf_slider_abpart_link", "rf_proximal_link", "rf_distal_link", "rf_connecting_link"],
    ["lf_slider_link", "lf_slider_abpart_link", "lf_proximal_link", "lf_distal_link", "lf_connecting_link"],
]
GESTURES = {
    "open": [0, 0, 0, 0, 0, 0],
    "close": [1000, 1000, 1000, 1000, 1000, 0],
    "pinch_2": [900, 900, 0, 0, 0, 0],
    "pinch_3": [850, 850, 850, 0, 0, 0],
    "cylinder": [700, 900, 900, 900, 900, 0],
}


class A001HandStatePublisher:
    def __init__(self):
        self.publisher = rospy.Publisher("/joint_states", JointState, queue_size=1, latch=True)
        rospy.Subscriber("/hand/python_style", Int32MultiArray, self.python_style_callback, queue_size=1)
        rospy.Subscriber("/hand/target", UInt16MultiArray, self.target_callback, queue_size=1)
        rospy.Subscriber("/hand/gesture", String, self.gesture_callback, queue_size=1)
        # Supply a complete planning state before the first hand command arrives.
        self.publish_state(GESTURES["open"])
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, _event):
        self.publish_state(self.actuator_values)

    def python_style_callback(self, message):
        if len(message.data) != 6:
            rospy.logwarn("Ignoring /hand/python_style message with %d values", len(message.data))
            return
        self.publish_state(message.data)

    def target_callback(self, message):
        if len(message.data) != 6:
            rospy.logwarn("Ignoring /hand/target message with %d values", len(message.data))
            return
        self.publish_state(message.data)

    def gesture_callback(self, message):
        command = GESTURES.get(message.data.lower())
        if command is None:
            rospy.logwarn("Cannot visualize unknown A001 gesture: %s", message.data)
            return
        self.publish_state(command)

    def publish_state(self, actuator_values):
        self.actuator_values = list(actuator_values)
        result = JointState()
        result.header.stamp = rospy.Time.now()
        for finger_index, actuator_value in enumerate(actuator_values[:5]):
            slider_limit = SLIDER_LIMITS[finger_index]
            position = max(0.0, min(slider_limit, float(actuator_value) * slider_limit / 1000.0))
            angles = HAND_FingerPosToAngle(finger_index, position)
            if finger_index == 0:
                result.name.extend(JOINT_NAMES[finger_index])
                result.position.extend([angles[0], position, angles[1], angles[2]])
            else:
                result.name.extend(JOINT_NAMES[finger_index])
                result.position.extend([position, angles[0], angles[1], angles[2], angles[3]])
        result.name.append("th_root_link")
        result.position.append(float(actuator_values[5]) * 1.5708 / 1000.0)
        self.publisher.publish(result)


if __name__ == "__main__":
    rospy.init_node("a001_hand_state_publisher")
    A001HandStatePublisher()
    rospy.spin()
