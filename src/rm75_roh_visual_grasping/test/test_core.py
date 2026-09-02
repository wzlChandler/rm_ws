#!/usr/bin/env python3

import math
import unittest

from rm75_roh_visual_grasping.core import (
    StableTargetFilter,
    cartesian_fraction_ok,
    decode_register_values,
    hand_joint_positions,
    hand_state_allows_lift,
    link_position_for_tcp,
    point_before_target_along_tool_z,
    point_in_workspace,
    poses_are_close,
    surface_point_to_sphere_center,
)


class CoreTest(unittest.TestCase):
    def test_surface_point_moves_away_from_camera(self):
        center = surface_point_to_sphere_center((0.0, 0.0, 1.0), 0.02)
        self.assertEqual(center, (0.0, 0.0, 1.02))

    def test_surface_point_uses_full_sight_ray(self):
        center = surface_point_to_sphere_center((0.3, 0.4, 0.0), 0.02)
        self.assertAlmostEqual(center[0], 0.312)
        self.assertAlmostEqual(center[1], 0.416)

    def test_tcp_offset_is_rotated_before_subtraction(self):
        half = math.sqrt(0.5)
        position = link_position_for_tcp((1.0, 2.0, 3.0), (0.0, 0.0, half, half), (0.1, 0.0, 0.0))
        self.assertAlmostEqual(position[0], 1.0)
        self.assertAlmostEqual(position[1], 1.9)
        self.assertAlmostEqual(position[2], 3.0)

    def test_pregrasp_moves_opposite_tool_z(self):
        pregrasp = point_before_target_along_tool_z(
            (0.2, 0.0, 0.02), (1.0, 0.0, 0.0, 0.0), 0.15
        )
        self.assertAlmostEqual(pregrasp[0], 0.2)
        self.assertAlmostEqual(pregrasp[1], 0.0)
        self.assertAlmostEqual(pregrasp[2], 0.17)

    def test_duplicate_timestamps_do_not_fake_stability(self):
        target_filter = StableTargetFilter(3, 0.01, 0.5)
        self.assertTrue(target_filter.add(1.0, (0.2, 0.0, 0.03)))
        self.assertFalse(target_filter.add(1.0, (0.2, 0.0, 0.03)))
        self.assertFalse(target_filter.add(0.9, (0.2, 0.0, 0.03)))
        self.assertIsNone(target_filter.stable_target(1.1))

    def test_stable_target_rejects_spread_and_age(self):
        target_filter = StableTargetFilter(3, 0.01, 0.5)
        target_filter.add(1.0, (0.20, 0.00, 0.03))
        target_filter.add(1.1, (0.20, 0.00, 0.03))
        target_filter.add(1.2, (0.23, 0.00, 0.03))
        self.assertIsNone(target_filter.stable_target(1.2))
        target_filter.clear()
        for stamp in (2.0, 2.1, 2.2):
            target_filter.add(stamp, (0.20, 0.00, 0.03))
        self.assertIsNotNone(target_filter.stable_target(2.3))
        self.assertIsNone(target_filter.stable_target(2.8))

    def test_workspace_bounds(self):
        bounds = ((0.12, 0.42), (-0.18, 0.18), (-0.005, 0.10))
        self.assertTrue(point_in_workspace((0.2, 0.0, 0.03), bounds))
        self.assertTrue(point_in_workspace((0.25, -0.06, 0.003), bounds))
        self.assertFalse(point_in_workspace((0.29, -0.14, -0.027), bounds))
        self.assertFalse(point_in_workspace((0.5, 0.0, 0.03), bounds))

    def test_cartesian_fraction_gate(self):
        self.assertTrue(cartesian_fraction_ok(0.98, 0.98))
        self.assertFalse(cartesian_fraction_ok(0.97, 0.98))
        self.assertFalse(cartesian_fraction_ok(1.1, 0.98))

    def test_pose_proximity_accepts_equivalent_quaternion_sign(self):
        self.assertTrue(
            poses_are_close(
                (0.2, 0.0, 0.4),
                (0.0, 0.0, 0.0, 1.0),
                (0.203, 0.0, 0.4),
                (0.0, 0.0, 0.0, -1.0),
                0.005,
                0.02,
            )
        )
        self.assertFalse(
            poses_are_close(
                (0.2, 0.0, 0.4),
                (0.0, 0.0, 0.0, 1.0),
                (0.21, 0.0, 0.4),
                (0.0, 0.0, 0.0, 1.0),
                0.005,
                0.02,
            )
        )

    def test_lift_gate_requires_all_hand_checks(self):
        positions = [25000, 26000, 27000, 0, 0, 0]
        self.assertTrue(hand_state_allows_lift(True, 0, positions, 20000))
        self.assertFalse(hand_state_allows_lift(False, 0, positions, 20000))
        self.assertFalse(hand_state_allows_lift(True, 1, positions, 20000))
        self.assertFalse(hand_state_allows_lift(True, 0, [25000, 100, 27000, 0, 0, 0], 20000))

    def test_register_values_decode_controller_byte_arrays(self):
        self.assertEqual(decode_register_values([0, 0], 1), [0])
        self.assertEqual(
            decode_register_values([0, 0, 0xCC, 0xCC, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0], 6),
            [0, 0xCCCC, 0xFFFF, 0, 0, 0],
        )
        self.assertEqual(decode_register_values([1, 2, 3], 3), [1, 2, 3])
        with self.assertRaises(ValueError):
            decode_register_values([0, 0, 0], 2)

    def test_hand_joint_positions_maps_three_finger_command(self):
        def fake_converter(finger_index, slider):
            count = 3 if finger_index == 0 else 4
            return [finger_index + slider] * count

        names, positions = hand_joint_positions([800, 800, 800, 0, 0, 0], fake_converter)
        self.assertEqual(len(names), 25)
        self.assertEqual(len(positions), 25)
        self.assertAlmostEqual(positions[names.index("th_slider_link")], 0.008)
        self.assertAlmostEqual(positions[names.index("if_slider_link")], 0.0152)
        self.assertAlmostEqual(positions[names.index("mf_slider_link")], 0.0152)
        self.assertEqual(positions[names.index("rf_slider_link")], 0.0)
        self.assertEqual(positions[names.index("lf_slider_link")], 0.0)


if __name__ == "__main__":
    unittest.main()
