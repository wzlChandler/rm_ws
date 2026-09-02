#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""刚体手爪抓取实验v3:苹果放进钳口内部,直接上收提起"""
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


def get_pts(listener, frames):
    out = []
    for f in frames:
        try:
            (p, _) = listener.lookupTransform("world", f, rospy.Time(0))
            out.append(p)
        except Exception:
            pass
    return out


def main():
    rospy.init_node("grasp_v3")
    rospy.wait_for_service("/gazebo/get_model_state", timeout=10)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10)
    get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    arm_pub = rospy.Publisher("/arm_joint_controller/command",
                              JointTrajectory, queue_size=1)
    listener = tf.TransformListener()
    rospy.sleep(2.0)

    # 1. 读取 4 指与拇指位置
    fingers = get_pts(listener, ["if_distal_link", "mf_distal_link",
                                 "rf_distal_link", "lf_distal_link"])
    thumb = get_pts(listener, ["th_distal_link"])
    if not fingers:
        rospy.logerr("无法获取指尖")
        return
    fx = sum(p[0] for p in fingers) / len(fingers)
    fy = sum(p[1] for p in fingers) / len(fingers)
    fz = sum(p[2] for p in fingers) / len(fingers)
    tx = thumb[0][0] if thumb else fx
    ty = thumb[0][1] if thumb else fy
    rospy.loginfo("4指尖中心: (%s), 拇指: (%s)", [round(v, 3) for v in (fx, fy, fz)], [round(v, 3) for v in (tx, ty)])

    # 2. 苹果放到 4 指与拇指之间的钳口(指尖上方一点)
    ax = (fx + tx) / 2.0
    ay = (fy + ty) / 2.0
    az = fz - 0.005  # 略高于指尖,在钳口内
    rospy.loginfo("苹果目标: (%s)", [round(v, 3) for v in (ax, ay, az)])
    set_apple(set_srv, ax, ay, az)
    rospy.sleep(2.0)
    rospy.loginfo("放置后苹果: %s", get_apple(get))

    # 3. 直接上收手臂(不动 hand 关节),看苹果是否被托起
    arm_move(arm_pub, [0.0, 0.7, -0.5, 0.0, 0.0, 0.0, 0.0], 5.0)
    rospy.sleep(6.0)
    a2 = get_apple(get)
    rospy.loginfo("提起后苹果: %s", a2)
    if a2:
        dz = a2[2] - az
        rospy.loginfo("苹果 z 变化 %.3f m %s", dz,
                      "-> 抓取成功!" if dz > 0.03 else "-> 未抓起")


if __name__ == "__main__":
    main()
