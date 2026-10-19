#!/usr/bin/env python3
"""Closed-form antenna covariance and frozen physical-pair controls."""
import copy
import math
from pathlib import Path
import tempfile
import unittest

from check_gnss_antenna import freeze, materialize, specifications
from check_gnss_latency import Filter, verify_pose_covariance


class AntennaTests(unittest.TestCase):
    def test_joint_antenna_solution_matches_closed_form_and_exposes_yaw_cross_covariance(self):
        config = dict(initial_pose=dict(position=dict(x=0., y=0.), yaw=0.),
                      initial_pose_covariance=[[.25, 0., 0.], [0., .36, 0.], [0., 0., .04]],
                      gnss_antenna_offset_m=dict(x=1., y=0.))
        oracle = Filter(config)
        oracle.correct(dict(stamp=0., position=dict(x=1.1, y=.2), variance=.09))
        expected = [.25*.1/.34, .36*.2/.49, .04*.2/.49]
        for actual, value in zip(oracle.x, expected):
            self.assertAlmostEqual(actual, value, places=14)
        self.assertAlmostEqual(oracle.p[1][2], -.36*.04/.49, places=14)
        self.assertAlmostEqual(oracle.p[2][2], .04-.04**2/.49, places=14)
        out = dict(pose_covariance=copy.deepcopy(oracle.p))
        verify_pose_covariance(out, oracle.p, config)
        out['pose_covariance'][1][2] = out['pose_covariance'][2][1] = 0.
        with self.assertRaises(ValueError):
            verify_pose_covariance(out, oracle.p, config)
        # Heading crosses pi while the covariance uses the original linearization.
        config['initial_pose']['yaw'] = math.pi-.001
        turned = Filter(config)
        turned.correct(dict(stamp=0., position=dict(x=-1., y=-.2), variance=.09))
        self.assertLess(turned.x[2], 0.)
        self.assertEqual(turned.decision, 'Accepted')

    def test_all_88_cases_are_fixed_and_controls_change_only_filter_calibration(self):
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 88)
        self.assertEqual(sum(s['mode'] == 'control' for s in specs), 8)
        self.assertEqual(sum(s['mode'] == 'confirm' for s in specs), 8)
        self.assertEqual(len(specifications(['reference-2d'], True)), 16)
        for control in (s for s in specs if s['mode'] == 'control'):
            calibrated = dict(control, mode='calibrated')
            _, a = materialize(calibrated)
            _, b = materialize(control)
            self.assertEqual(a.pop('gnss_antenna_offset_m'), a['gnss_antenna_mount_m'])
            self.assertEqual(a, b)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            frozen = freeze(root, specs)
            self.assertEqual(freeze(root, specs), frozen)
            (root/frozen['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):
                freeze(root, specs)


if __name__ == '__main__':
    unittest.main()
