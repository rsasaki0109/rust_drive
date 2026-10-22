#!/usr/bin/env python3
"""Closed-form conditioning and corruption controls for initial-prior evidence."""
import copy
import json
import unittest
from check_gnss_latency import Filter, verify_pose_covariance
from check_initial_pose import BOUND_RAD, PRIOR, REPO, materialize, specifications, confirmation_specifications


class PriorEvidenceTests(unittest.TestCase):
    def test_correlated_prior_has_closed_form_conditional_yaw(self):
        p = [[4., 1., .2], [1., 2., .1], [.2, .1, .09]]
        f = Filter({'initial_pose': {'position': {'x': 0., 'y': 0.}, 'yaw': 0.}, 'initial_pose_covariance': p})
        f.correct({'stamp': 0., 'position': {'x': 1., 'y': -2.}, 'variance': 1.})
        self.assertAlmostEqual(f.x[0], 9./14.)
        self.assertAlmostEqual(f.x[1], -17./14.)
        self.assertAlmostEqual(f.x[2], -1./140.)
        self.assertAlmostEqual(f.p[2][2], .09-13./1400.)
        self.assertEqual(p, [[4., 1., .2], [1., 2., .1], [.2, .1, .09]])

    def test_off_diagonal_corruption_is_detected_even_with_unchanged_trace(self):
        config = {'initial_pose_covariance': PRIOR}
        output = {'pose_covariance': copy.deepcopy(PRIOR)}
        self.assertEqual(verify_pose_covariance(output, PRIOR, config), 0.)
        for kind in ['cross', 'missing', 'negative', 'asymmetric', 'nonfinite']:
            bad = copy.deepcopy(output)
            if kind == 'cross': bad['pose_covariance'][0][2] = bad['pose_covariance'][2][0] = .001
            if kind == 'missing': bad.pop('pose_covariance')
            if kind == 'negative': bad['pose_covariance'][2][2] = -1.
            if kind == 'asymmetric': bad['pose_covariance'][1][0] = .01
            if kind == 'nonfinite': bad['pose_covariance'][0][0] = float('nan')
            with self.assertRaises(ValueError): verify_pose_covariance(bad, PRIOR, config)

    def test_protocol_keeps_worlds_deadlines_and_perturbations_separate_from_calibration(self):
        specs = specifications()
        self.assertEqual(len(specs), 53)
        self.assertEqual(sum(s['expected_quality_pass'] for s in specs), 51)
        self.assertEqual(len({s['id'] for s in specs}), 53)
        confirmations = confirmation_specifications()
        self.assertEqual(len(confirmations), 42)
        self.assertEqual(len({s['id'] for s in specs+confirmations}), 95)
        self.assertAlmostEqual(PRIOR[2][2], BOUND_RAD**2/3.)
        for spec in specs+confirmations:
            scenario = materialize(spec)
            base = json.loads((REPO/spec['source']).read_text())
            self.assertEqual({k: v for k, v in scenario.items() if k not in ['initial_pose_covariance', 'initial_heading_offset_rad']}, base)
            self.assertEqual(scenario['initial_pose_covariance'], PRIOR)
            if spec['heading_offset_rad'] and spec['expected_quality_pass']:
                self.assertLess(abs(spec['heading_offset_rad']), BOUND_RAD)
                self.assertIn(spec['seed'], [101, 202, 303, 404, 505, 606, 707, 808, 909, 1001])
            if not spec['expected_quality_pass']: self.assertGreater(abs(spec['heading_offset_rad']), .1)

    def test_absent_or_null_prior_keeps_original_covariance(self):
        pose = {'position': {'x': 2., 'y': -1.}, 'yaw': .2}
        legacy = Filter({'initial_pose': pose})
        null = Filter({'initial_pose': pose, 'initial_pose_covariance': None})
        self.assertEqual(legacy.p, null.p)
        self.assertEqual(legacy.p, [[1., 0., 0.], [0., 1., 0.], [0., 0., .03]])


if __name__ == '__main__': unittest.main()
