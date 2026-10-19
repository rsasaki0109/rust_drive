#!/usr/bin/env python3
"""Closed-form controls for the independent partial-route oracle."""
import copy
import math
import unittest
from check_mid_edge import compare, corpus_map, reference


def at(edge, s): return {'edge': edge, 's_m': s}


class MidEdgeOracleTests(unittest.TestCase):
    def nav(self, start, goal):
        return {'network': corpus_map(), 'start': start, 'goal': goal}

    def test_345_polyline_fractional_bounds_knots_and_caps(self):
        p = reference(self.nav(at('ab', 2.5), at('bc', 4.25)), [])
        self.assertEqual(p['points'], [{'x': 1.5, 'y': 2.}, {'x': 3., 'y': 4.}, {'x': 6., 'y': 8.}, {'x': 10.25, 'y': 8.}])
        self.assertEqual(p['length_m'], 11.75)
        self.assertEqual(p['node_ids'], ['b'])
        self.assertEqual(p['zones'], [{'start_s_m': 0., 'end_s_m': 7.5, 'speed_m_s': 3.},
                                      {'start_s_m': 7.5, 'end_s_m': 11.75, 'speed_m_s': 2.}])

    def test_incoming_no_only_and_partial_destination_admission(self):
        for kind in ['no', 'only']:
            nav = self.nav(at('ab', 7.), 'c')
            nav['network']['turn_restrictions'] = [{'id': 'r', 'from_edge': 'ab', 'via_node': 'b',
                                                  'to_edge': 'bc' if kind == 'no' else 'bd', 'kind': kind}]
            self.assertEqual(reference(nav, [])['edge_ids'], ['ab', 'bd', 'dc'])
            self.assertIsNone(reference(nav, ['bd']))
            nav['goal'] = at('bc', 4.)
            self.assertIsNone(reference(nav, []))

    def test_backwards_goal_requires_cycle_and_closed_endpoint_never_escapes(self):
        nav = self.nav(at('ab', 8.), at('ab', 2.))
        p = reference(nav, [])
        self.assertEqual(p['edge_ids'], ['ab', 'bc', 'ca', 'ab'])
        self.assertAlmostEqual(p['length_m'], 15.+math.hypot(17., 8.))
        self.assertIsNone(reference(nav, ['ca']))
        self.assertIsNone(reference(nav, ['ab']))
        nav['start'] = at('ab', 1.)
        self.assertEqual(reference(nav, [])['length_m'], 1.)

    def test_invalid_and_stationary_requests_are_not_clamped(self):
        for s in [0., -1., 10., math.inf, math.nan]:
            with self.assertRaises(ValueError): reference(self.nav(at('ab', s), 'c'), [])
        self.assertIsNone(reference(self.nav(at('ab', 4.), at('ab', 4.)), []))
        with self.assertRaises(ValueError): reference(self.nav(at('ab', 4.), 'c'), ['unknown'])

    def test_geometry_policy_and_traversal_corruption_are_detected(self):
        expected = reference(self.nav(at('ab', 2.5), at('bc', 4.25)), [])
        plan = {'edge_ids': expected['edge_ids'], 'node_ids': expected['node_ids'], 'distance_m': expected['length_m'],
                'route': {'points': expected['points'], 'half_width': expected['half_width']}, 'traversals': expected['traversals']}
        compare(plan, expected, expected['zones'])
        for key in ['distance', 'point', 'span', 'topology', 'cap']:
            bad, zones = copy.deepcopy(plan), copy.deepcopy(expected['zones'])
            if key == 'distance': bad['distance_m'] += 1.
            if key == 'point': bad['route']['points'][0]['x'] += 1.
            if key == 'span': bad['traversals'][0]['start_s_m'] = 0.
            if key == 'topology': bad['node_ids'].insert(0, 'a')
            if key == 'cap': zones[0]['end_s_m'] += 1.
            with self.assertRaises(ValueError): compare(bad, expected, zones)


if __name__ == '__main__': unittest.main()
