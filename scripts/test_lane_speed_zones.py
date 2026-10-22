#!/usr/bin/env python3
"""Arithmetic adversaries for the independent original-map speed reconstruction."""
import unittest
from check_lane_speed_zones import check_policy, reference_sections


class MapSpeedTests(unittest.TestCase):
    def test_arc_length_uses_interior_points_and_preserves_original_map(self):
        lanes = [{'id': 'a', 'centerline': [{'x': 0., 'y': 0.}, {'x': 3., 'y': 4.}, {'x': 6., 'y': 0.}],
                  'speed_limit_m_s': 20., 'successors': ['b']},
                 {'id': 'b', 'centerline': [{'x': 6., 'y': 0.}, {'x': 6., 'y': 2.}],
                  'speed_limit_m_s': 2., 'successors': []}]
        points, zones = reference_sections({'lanes': lanes}, ['a', 'b'])
        self.assertEqual(len(points), 4)
        self.assertEqual([(z['start_s_m'], z['end_s_m'], z['speed_m_s']) for z in zones], [(0., 10., 8.), (10., 12., 2.)])
        self.assertEqual(lanes[0]['speed_limit_m_s'], 20.)

    def test_policy_comparison_keeps_a_centimeter_limit_between_fast_sections(self):
        zone = lambda a, b, v: {'start_s_m': a, 'end_s_m': b, 'speed_m_s': v}
        expected = [zone(0., 10., 6.), zone(10., 10.01, 2.), zone(10.01, 20., 6.)]
        check_policy(expected, expected)
        with self.assertRaises(ValueError):
            check_policy([zone(0., 20., 6.)], expected)
        with self.assertRaises(ValueError):
            check_policy([zone(0., 9., 6.), zone(10.01, 20., 6.)], expected)
        check_policy([zone(0., 20., 6.)], [zone(0., 10., 6.), zone(10., 20., 6.)])

    def test_a_closed_or_disconnected_itinerary_is_not_reference_evidence(self):
        lanes = [{'id': name, 'centerline': [{'x': i, 'y': 0.}, {'x': i+1, 'y': 0.}],
                  'speed_limit_m_s': 2., 'successors': []} for i, name in enumerate(['a', 'b'])]
        with self.assertRaises(ValueError):
            reference_sections({'lanes': lanes}, ['a'], ['a'])
        with self.assertRaises(ValueError):
            reference_sections({'lanes': lanes}, ['a', 'b'])


if __name__ == '__main__':
    unittest.main()
