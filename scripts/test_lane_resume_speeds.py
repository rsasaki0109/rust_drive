#!/usr/bin/env python3
import math
import unittest

from check_lane_resume_speeds import interval_excess, resume_reference, speed_audit
from check_speed_zones import progress


def fixture():
    points = [{'x': x, 'y': y} for x, y in [(0., 0.), (25., 0.), (55., 3.5), (80., 3.5), (160., 3.5)]]
    limit = progress({'x': 100., 'y': 3.5}, points)
    zones = [{'start_s_m': 0., 'end_s_m': 55.3, 'speed_m_s': 2.},
             {'start_s_m': 55.3, 'end_s_m': limit, 'speed_m_s': 4.},
             {'start_s_m': limit, 'end_s_m': limit+.01, 'speed_m_s': 1.5},
             {'start_s_m': limit+.01, 'end_s_m': progress(points[-1], points), 'speed_m_s': 4.}]
    request = {'start': {'position': {'x': 42., 'y': 0.}, 'yaw': 0.}, 'distance_m': 30.,
               'target_start': {'x': 0., 'y': 3.5}, 'target_end': {'x': 80., 'y': 3.5}}
    return points, zones, request


class ResumeSpeeds(unittest.TestCase):
    def test_independent_connector_and_centimeter_tail_use_the_new_arc_origin(self):
        original, zones, request = fixture()
        points, mapped, join, connector = resume_reference(original, zones, request, 2.)
        self.assertGreater(connector, 30.)
        self.assertAlmostEqual(points[60]['y'], 1.75)
        self.assertAlmostEqual(points[120]['x'], 72.)
        thin = next(z for z in mapped if z['speed_m_s'] == 1.5)
        self.assertAlmostEqual(thin['end_s_m']-thin['start_s_m'], .01)
        self.assertAlmostEqual(thin['start_s_m'], connector+zones[2]['start_s_m']-join)
        self.assertEqual(points[-1], original[-1])

    def test_reusing_original_distances_misses_a_real_tail_speed_violation(self):
        original, zones, request = fixture()
        points, mapped, _, _ = resume_reference(original, zones, request, 2.)
        state = lambda x: {'pose': {'position': {'x': x, 'y': 3.5}}, 'speed': 3.}
        wrong, _ = interval_excess(points, zones, state(99.9), state(100.1), 1.25)
        correct, _ = interval_excess(points, mapped, state(99.9), state(100.1), 1.25)
        self.assertEqual(wrong, 0.)
        self.assertAlmostEqual(correct, 1.5)

    def test_rotating_the_reference_preserves_length_and_sparse_frames_are_rejected(self):
        original, zones, request = fixture()
        first = resume_reference(original, zones, request, 2.)
        for p in original: p['x'], p['y'] = -p['y'], p['x']
        for key in ['target_start', 'target_end']:
            p = request[key]; p['x'], p['y'] = -p['y'], p['x']
        p = request['start']['position']; p['x'], p['y'] = -p['y'], p['x']
        request['start']['yaw'] += math.pi/2
        second = resume_reference(original, zones, request, 2.)
        self.assertAlmostEqual(first[3], second[3])
        with self.assertRaises(ValueError): speed_audit({'frames': [], 'summary': {'steps': 1}}, original, zones, 2.)


if __name__ == '__main__': unittest.main()
