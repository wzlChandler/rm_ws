#!/usr/bin/env python3

import math
import unittest

from rm75_roh_visual_grasping.hand_teleop_core import (
    CommandSmoother,
    HandCalibration,
    LandmarkSmoother,
    OneEuroSmoother,
    apply_hysteresis,
    command_changed,
    hand_features,
    joint_angle,
)


class HandTeleopCoreTest(unittest.TestCase):
    def test_joint_angle(self):
        self.assertAlmostEqual(joint_angle((1, 0, 0), (0, 0, 0), (0, 1, 0)), math.pi / 2)

    def test_hand_features_returns_six_normalized_values(self):
        landmarks = [(0.0, 0.0, 0.0)] * 21
        landmarks[0] = (0.0, 0.0, 0.0)
        for base in (1, 5, 9, 13, 17):
            landmarks[base] = (0.0, 1.0, 0.0)
            landmarks[base + 1] = (0.0, 2.0, 0.0)
            landmarks[base + 2] = (0.0, 3.0, 0.0)
            landmarks[base + 3] = (0.0, 4.0, 0.0)
        values = hand_features(landmarks)
        self.assertEqual(len(values), 6)
        self.assertTrue(all(0.0 <= value <= 1.0 for value in values))

    def test_calibration_clamps_and_scales_values(self):
        calibration = HandCalibration([0.1] * 6, [0.9] * 6)
        self.assertEqual(calibration.command([0.1] * 6), [0] * 6)
        self.assertEqual(calibration.command([0.5] * 6), [500] * 6)
        self.assertEqual(calibration.command([1.0] * 6), [1000] * 6)

    def test_calibration_rejects_insufficient_movement(self):
        with self.assertRaises(ValueError):
            HandCalibration([0.1] * 6, [0.11] * 6)

    def test_smoothing_clamping_and_deadband(self):
        smoother = CommandSmoother(alpha=0.5)
        self.assertEqual(smoother.update([-1, 1, 2, 3, 4, 1001]), [0, 1, 2, 3, 4, 1000])
        self.assertEqual(smoother.update([100, 101, 102, 103, 104, 900]), [50, 51, 52, 53, 54, 950])
        self.assertFalse(command_changed([50] * 6, [55] * 6, 12))
        self.assertTrue(command_changed([50] * 6, [62] * 6, 12))

    def test_one_euro_filter_suppresses_stationary_jitter(self):
        smoother = OneEuroSmoother(min_cutoff=0.8, beta=0.4)
        outputs = []
        for frame in range(90):
            jittered = 500 + (18 if frame % 2 else -18)
            outputs.append(smoother.update([jittered] * 6, frame / 30.0)[0])
        self.assertLess(max(outputs[-30:]) - min(outputs[-30:]), 8)

    def test_one_euro_filter_responds_to_deliberate_motion(self):
        smoother = OneEuroSmoother(min_cutoff=0.8, beta=0.4)
        smoother.update([0] * 6, 0.0)
        output = None
        for frame in range(1, 7):
            output = smoother.update([1000] * 6, frame / 30.0)
        self.assertGreater(output[0], 650)

    def test_hysteresis_holds_only_small_channel_changes(self):
        self.assertEqual(
            apply_hysteresis([504, 506, 495, 520, 480, 500], [500] * 6, 6),
            [500, 506, 500, 520, 480, 500],
        )

    def test_landmark_filter_suppresses_preview_jitter(self):
        smoother = LandmarkSmoother(min_cutoff=1.0, beta=0.3)
        outputs = []
        for frame in range(90):
            offset = 0.01 if frame % 2 else -0.01
            landmarks = [(0.5 + offset, 0.5 - offset, 0.0)] * 21
            outputs.append(smoother.update(landmarks, frame / 30.0)[0][0])
        self.assertLess(max(outputs[-30:]) - min(outputs[-30:]), 0.004)


if __name__ == "__main__":
    unittest.main()
