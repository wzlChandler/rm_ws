#!/usr/bin/env python3
"""Track one human hand and publish normalized ROH-A001 hand targets."""

import time

import cv2
import rospy
from std_msgs.msg import String, UInt16MultiArray
from std_srvs.srv import Trigger, TriggerResponse

from rm75_roh_visual_grasping.hand_teleop_core import (
    HandCalibration,
    LandmarkSmoother,
    OneEuroSmoother,
    apply_hysteresis,
    feature_median,
    hand_features,
)


class HandTeleopNode:
    def __init__(self):
        import mediapipe as mp

        self.camera_index = int(rospy.get_param("~camera_index", 0))
        self.camera_width = int(rospy.get_param("~camera_width", 640))
        self.camera_height = int(rospy.get_param("~camera_height", 480))
        self.camera_fps = int(rospy.get_param("~camera_fps", 30))
        self.confidence = float(rospy.get_param("~confidence", 0.7))
        self.model_complexity = int(rospy.get_param("~model_complexity", 1))
        self.calibration_duration = float(rospy.get_param("~calibration_duration", 2.0))
        self.preview = bool(rospy.get_param("~preview", False))
        self.enabled_default = bool(rospy.get_param("~enabled", False))
        self.deadband = int(rospy.get_param("~deadband", 6))
        self.smoother = OneEuroSmoother(
            min_cutoff=float(rospy.get_param("~filter_min_cutoff", 0.8)),
            beta=float(rospy.get_param("~filter_beta", 0.4)),
            derivative_cutoff=float(rospy.get_param("~filter_derivative_cutoff", 1.0)),
        )
        self.landmark_smoother = LandmarkSmoother(
            min_cutoff=float(rospy.get_param("~landmark_min_cutoff", 1.0)),
            beta=float(rospy.get_param("~landmark_beta", 0.3)),
            derivative_cutoff=float(rospy.get_param("~landmark_derivative_cutoff", 1.0)),
        )
        self.calibration = None
        self.open_samples = []
        self.closed_samples = []
        self.calibration_stage = None
        self.stage_started_at = None
        self.last_published = None
        self.current_status = None

        self.target_pub = rospy.Publisher("/hand/target", UInt16MultiArray, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.start_service = rospy.Service("~start_calibration", Trigger, self._start_calibration)
        self.reset_service = rospy.Service("~reset_calibration", Trigger, self._reset_calibration)

        self.capture = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.camera_fps)
        if not self.capture.isOpened():
            self._set_status("CAMERA_ERROR")
            raise RuntimeError("cannot open camera /dev/video{}".format(self.camera_index))

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=self.model_complexity,
            min_detection_confidence=self.confidence,
            min_tracking_confidence=self.confidence,
        )
        self._set_status("WAITING_FOR_CALIBRATION")

    def _set_status(self, status):
        if status != self.current_status:
            self.current_status = status
            self.status_pub.publish(String(data=status))
            rospy.loginfo("hand teleop status=%s", status)

    def _start_calibration(self, _request):
        self.calibration = None
        self.smoother.reset()
        self.landmark_smoother.reset()
        self.last_published = None
        self.open_samples = []
        self.closed_samples = []
        self.calibration_stage = "OPEN"
        self.stage_started_at = time.monotonic()
        self._set_status("CALIBRATING_HOLD_OPEN")
        return TriggerResponse(success=True, message="Hold an open hand in view for two seconds")

    def _reset_calibration(self, _request):
        self.calibration = None
        self.smoother.reset()
        self.landmark_smoother.reset()
        self.last_published = None
        self.calibration_stage = None
        self._set_status("WAITING_FOR_CALIBRATION")
        return TriggerResponse(success=True, message="Calibration cleared; no targets will be published")

    def _advance_calibration(self, features):
        samples = self.open_samples if self.calibration_stage == "OPEN" else self.closed_samples
        samples.append(features)
        if time.monotonic() - self.stage_started_at < self.calibration_duration:
            return
        if self.calibration_stage == "OPEN":
            self.calibration_stage = "CLOSED"
            self.stage_started_at = time.monotonic()
            self._set_status("CALIBRATING_HOLD_FIST")
            return
        try:
            self.calibration = HandCalibration(
                feature_median(self.open_samples), feature_median(self.closed_samples)
            )
        except ValueError as error:
            self.calibration_stage = None
            self._set_status("CALIBRATION_FAILED")
            rospy.logwarn("hand teleop calibration failed: %s", error)
            return
        self.calibration_stage = None
        self.smoother.reset()
        self._set_status("TRACKING")

    def _publish_target(self, features):
        target = self.smoother.update(self.calibration.command(features), time.monotonic())
        target = apply_hysteresis(target, self.last_published, self.deadband)
        self.target_pub.publish(UInt16MultiArray(data=target))
        self.last_published = target

    def _process_frame(self, frame):
        result = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        hands = result.multi_hand_landmarks or []
        if len(hands) != 1:
            self.landmark_smoother.reset()
            if self.calibration is None:
                self._set_status("WAITING_FOR_CALIBRATION")
            elif self.calibration_stage is None:
                self._set_status("TRACKING_LOST")
            return frame

        landmarks = hands[0].landmark
        filtered_landmarks = self.landmark_smoother.update(landmarks, time.monotonic())
        for landmark, filtered in zip(landmarks, filtered_landmarks):
            landmark.x, landmark.y, landmark.z = filtered
        if self.preview:
            self.mp_drawing.draw_landmarks(frame, hands[0], self.mp_hands.HAND_CONNECTIONS)
        try:
            features = hand_features(filtered_landmarks)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "invalid hand landmarks: %s", error)
            return frame
        if self.calibration_stage is not None:
            self._advance_calibration(features)
        elif self.calibration is None:
            self._set_status("WAITING_FOR_CALIBRATION")
        elif bool(rospy.get_param("~enabled", self.enabled_default)):
            self._set_status("TRACKING")
            self._publish_target(features)
        else:
            self._set_status("DISABLED")
        return frame

    def run(self):
        rate = rospy.Rate(self.camera_fps)
        while not rospy.is_shutdown():
            ok, frame = self.capture.read()
            if not ok:
                self._set_status("CAMERA_ERROR")
                rospy.logerr_throttle(2.0, "unable to read camera frame")
                rate.sleep()
                continue
            frame = self._process_frame(frame)
            if self.preview:
                cv2.putText(frame, self.current_status, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("hand_teleop", frame)
                cv2.waitKey(1)
            rate.sleep()
        self.hands.close()
        self.capture.release()
        if self.preview:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("hand_teleop")
    try:
        HandTeleopNode().run()
    except (ImportError, RuntimeError) as error:
        rospy.logfatal("hand teleop cannot start: %s", error)
        raise


if __name__ == "__main__":
    main()
