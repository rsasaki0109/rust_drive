#!/usr/bin/env python3
import json
import tempfile
import unittest
from pathlib import Path

from check_gnss_covariance import contains_declared, freeze, scenario
from check_gnss_latency import Filter, verify_pose_covariance


def config():
    return {'initial_pose': {'position': {'x': 0., 'y': 0.}, 'yaw': 0.},
            'initial_pose_covariance': [[.01, 0., 0.], [0., .01, 0.], [0., 0., .001]],
            'gnss_covariance_shape': [[1., .9], [.9, 1.]]}


class GnssCovariance(unittest.TestCase):
    def test_joint_oracle_matches_eigenbasis_closed_form_and_directional_gate(self):
        c = config()
        f = Filter(c)
        f.correct({'stamp': 0., 'variance': .02, 'position': {'x': .1, 'y': .1}})
        long_gain, narrow_gain = .01/.048, .01/.012
        long_p, narrow_p = .038*long_gain, .002*narrow_gain
        self.assertAlmostEqual(f.x[0], .1*long_gain)
        self.assertAlmostEqual(f.x[1], .1*long_gain)
        self.assertAlmostEqual(f.p[0][0], (long_p+narrow_p)/2.)
        self.assertAlmostEqual(f.p[0][1], (long_p-narrow_p)/2.)
        self.assertAlmostEqual(f.nis, .02/.048)
        for shaped, expected in [(True, 'RejectedInnovation'), (False, 'Accepted')]:
            c = config()
            if not shaped:
                del c['gnss_covariance_shape']
            f = Filter(c)
            f.correct({'stamp': 0., 'variance': .02, 'position': {'x': .6, 'y': -.6}})
            self.assertEqual(f.decision, expected)

    def test_full_posterior_checker_requires_shape_without_prior_and_rejects_forgeries(self):
        c = config()
        del c['initial_pose_covariance']
        p = [[.01, .004, 0.], [.004, .01, 0.], [0., 0., .001]]
        self.assertEqual(verify_pose_covariance({'pose_covariance': p}, p, c), 0.)
        for changed in [None, [[.01, 0., 0.], [0., .01, 0.], [0., 0., .001]],
                        [[.01, .02, 0.], [.02, .01, 0.], [0., 0., .001]],
                        [[float('nan'), .004, 0.], [.004, .01, 0.], [0., 0., .001]]]:
            with self.assertRaises(ValueError):
                verify_pose_covariance({'pose_covariance': changed}, p, c)

    def test_frozen_pairs_change_only_calibration_and_refuse_modified_sources(self):
        self.assertTrue(contains_declared({'objects': [{'s': 3., 'speed': 0.}]}, {'objects': [{'s': 3.}]}))
        self.assertFalse(contains_declared({'objects': [{'s': 4., 'speed': 0.}]}, {'objects': [{'s': 3.}]}))
        self.assertFalse(contains_declared({'objects': []}, {'objects': [{'s': 3.}]}))
        for world in ['north-bias', 'diagonal-bias']:
            _, positive = scenario(world)
            _, control = scenario(world, False)
            del positive['gnss_covariance_shape']
            positive['name'] = control['name']
            self.assertEqual(positive, control)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            result = freeze(root, ['reference-2d', 'rne-dynamic'], [1, 7, 42, 313])
            self.assertEqual(len(result['episodes']), 64)
            self.assertEqual(sum(r['calibrated'] for r in result['episodes']), 48)
            self.assertEqual(freeze(root, ['reference-2d', 'rne-dynamic'], [1, 7, 42, 313]), result)
            source = root/result['episodes'][0]['scenario']
            changed = json.loads(source.read_text())
            changed['cruise_speed'] = .5
            source.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                freeze(root, ['reference-2d', 'rne-dynamic'], [1, 7, 42, 313])


if __name__ == '__main__':
    unittest.main()
