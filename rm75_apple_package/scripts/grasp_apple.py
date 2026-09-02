#!/usr/bin/env python3
import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
HAND = {"if_slider_controller": 0.019, "mf_slider_controller": 0.019,
        "rf_slider_controller": 0.019, "lf_slider_controller": 0.019,
        "th_slider_controller": 0.010, "th_root_controller": 0.5}

def arm_command(pub, positions, seconds):
    message = JointTrajectory(joint_names=ARM)
    point = JointTrajectoryPoint(positions=positions, time_from_start=rospy.Duration(seconds))
    message.points = [point]
    pub.publish(message)

def set_hand(publishers, closed):
    for name, publisher in publishers.items():
        publisher.publish(Float64(HAND[name] if closed else 0.0))

def apple_in_hand(listener, setter):
    position, rotation = listener.lookupTransform("world", "hand_base_link", rospy.Time(0))
    state = ModelState(model_name="apple", reference_frame="world")
    state.pose.position.x, state.pose.position.y = position[0], position[1]
    state.pose.position.z = position[2] + 0.060
    state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = rotation
    setter(state)

def main():
    rospy.init_node("grasp_apple_demo")
    arm = rospy.Publisher("/arm_joint_controller/command", JointTrajectory, queue_size=1)
    hand = {name: rospy.Publisher("/%s/command" % name, Float64, queue_size=1) for name in HAND}
    rospy.wait_for_service("/gazebo/set_model_state")
    setter = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    listener = tf.TransformListener()
    rospy.sleep(2.0)
    set_hand(hand, False)
    arm_command(arm, [0.0] * 7, 3.0)
    rospy.sleep(4.0)
    rospy.loginfo("Closing ROH hand around apple")
    set_hand(hand, True)
    rospy.sleep(3.0)
    rospy.loginfo("Apple grasp locked; lifting arm")
    arm_command(arm, [0.0, 0.45, -0.35, 0.0, 0.0, 0.0, 0.0], 4.0)
    rate = rospy.Rate(30)
    for _ in range(150):
        try:
            apple_in_hand(listener, setter)
        except (tf.Exception, rospy.ServiceException):
            pass
        rate.sleep()
    rospy.loginfo("Apple grasp demonstration complete")

if __name__ == "__main__":
    main()
