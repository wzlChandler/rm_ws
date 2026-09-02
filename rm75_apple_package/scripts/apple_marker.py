#!/usr/bin/env python3
"""Publish an apple rigidly positioned in the ROH palm of the pose snapshot."""
import rospy
from visualization_msgs.msg import Marker


def main():
    print("Publishing apple marker in hand frame...")
    rospy.init_node("apple_marker")
    publisher = rospy.Publisher("/apple_snapshot/apple", Marker,
                                queue_size=1, latch=True)
    marker = Marker()
    marker.header.frame_id = "hand_base_link"
    marker.ns = "apple"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    # Palm-centred placement: no robot joint is changed.
    marker.pose.position.x = -0.100
    marker.pose.position.z = -0.055
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = 0.080
    marker.color.r = 0.92
    marker.color.g = 0.08
    marker.color.b = 0.03
    marker.color.a = 1.0
    publisher.publish(marker)
    rospy.loginfo("Apple is attached visually to the captured hand pose.")
    rospy.spin()


if __name__ == "__main__":
    main()
