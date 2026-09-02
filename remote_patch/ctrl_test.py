#!/usr/bin/env python3
# 控制测试:验证手臂轨迹控制器与手爪位置控制器是否真实驱动关节
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
HAND = {"if_slider_link_j": 0.019, "mf_slider_link_j": 0.019,
        "rf_slider_link_j": 0.019, "lf_slider_link_j": 0.019,
        "th_slider_link_j": 0.010}


def get_joint_pos(timeout=8):
    got = {}
    def cb(msg):
        for n, p in zip(msg.name, msg.position):
            got[n] = p
    rospy.Subscriber("/joint_states", JointState, cb)
    t0 = rospy.Time.now()
    while rospy.Time.now() - t0 < rospy.Duration(timeout) and not rospy.is_shutdown():
        rospy.sleep(0.2)
    return got


def main():
    rospy.init_node("ctrl_test")
    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    pubs = {}
    for n in HAND:
        pubs[n] = rospy.Publisher("/%s_controller/command" % n, Float64, queue_size=1)
    th = rospy.Publisher("/th_root_controller/command", Float64, queue_size=1)
    rospy.sleep(1)

    rospy.loginfo("--- 发送手臂全零轨迹 ---")
    msg = JointTrajectory()
    msg.joint_names = ARM
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * 7
    pt.time_from_start = rospy.Duration(5.0)
    msg.points = [pt]
    arm_pub.publish(msg)

    rospy.loginfo("--- 发送手爪目标(闭合) ---")
    for n, v in HAND.items():
        pubs[n].publish(Float64(v))
    th.publish(Float64(0.5))

    rospy.sleep(8)
    got = get_joint_pos()
    names = ARM + list(HAND) + ["th_root_link_j"]
    for n in names:
        v = got.get(n)
        print("%-18s %s" % (n, round(v, 3) if v is not None else "N/A"))
    rospy.loginfo("test done")


if __name__ == "__main__":
    main()
