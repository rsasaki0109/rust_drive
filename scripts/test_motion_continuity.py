#!/usr/bin/env python3
"""Independent continuity-auditor mutation controls and immutable cohort checks."""
import copy
from pathlib import Path
import tempfile
import unittest

from check_motion_continuity import freeze, materialize, source_contract, specifications, verify_continuity


class MotionContinuityTests(unittest.TestCase):
    def test_only_the_exact_invalid_epoch_accuracy_failure_can_explain_a_source_failure(self):
        summary = dict(passed=False, failures=['localization max error 3.135 m exceeds 1 m'])
        result = source_contract(summary, 'dropout', 3.1351, .1, {'index': 10})
        self.assertTrue(result['passed'])
        self.assertEqual(result['expected_run_exit_code'], 1)
        self.assertEqual(result['explained_invalid_pose_failures'], summary['failures'])
        for mode, maximum, valid, gap in [('healthy', 3.1351, .1, {'index': 10}),
                                          ('dropout', 3.1351, 1.1, {'index': 10}),
                                          ('dropout', 3.1351, .1, None),
                                          ('dropout', 2.5, .1, {'index': 10})]:
            self.assertFalse(source_contract(summary, mode, maximum, valid, gap)['passed'])
        corrupted = copy.deepcopy(summary)
        corrupted['failures'].append('1 colliding integration steps')
        self.assertFalse(source_contract(corrupted, 'dropout', 3.1351, .1, {'index': 10})['passed'])
        corrupted = dict(summary, passed=True)
        self.assertFalse(source_contract(corrupted, 'dropout', 3.1351, .1, {'index': 10})['passed'])
        self.assertTrue(source_contract(dict(passed=True, failures=[]), 'healthy', .1, .1, None)['passed'])

    def test_all_36_inputs_are_fixed_and_dropout_only_changes_transport_and_expectation(self):
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 36)
        self.assertEqual(sum(s['mode'] == 'healthy' for s in specs), 8)
        self.assertEqual(sum(s['mode'] == 'dropout' for s in specs), 24)
        self.assertEqual(sum(s['mode'] == 'delayed-motion' for s in specs), 4)
        self.assertEqual(len(specifications(['reference-2d'], True)), 10)
        for spec in (s for s in specs if s['mode'] == 'dropout'):
            _, fault = materialize(spec)
            _, healthy = materialize(dict(spec, mode='healthy'))
            fault.pop('name'); healthy.pop('name')
            fault['expected'] = 'goal'
            self.assertEqual(fault['sensor_timing'].pop('odometry_dropout_windows'), [{'from': 8., 'until': 11.}])
            self.assertEqual(fault, healthy)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            frozen = freeze(root, specs)
            self.assertEqual(freeze(root, specs), frozen)
            (root/frozen['episodes'][0]['scenario']).write_text('{}')
            with self.assertRaises(ValueError):
                freeze(root, specs)

    def test_oracle_rejects_invented_motion_covariance_freshness_and_premature_release(self):
        pose = dict(position=dict(x=0., y=0.), yaw=0.)
        config = dict(initial_pose=pose)
        state = dict(time=0., estimate=dict(pose=pose, speed=0.), tracks=[], predictions=[],
                     position_variance=2., emergency=True, command=dict(acceleration=-6., steering=0.),
                     health=['StaleGnss'], localization=dict(accepted_fixes=0, rejected_fixes=0,
                     last_observed_stamp=None, last_accepted_stamp=None, last_nis=None, last_decision=None))
        ticks = [dict(input=dict(time=0., odometry=dict(stamp=0., speed=0., yaw_rate=0.)), expected=state)]
        gap = dict(last_covered_time=0., detected_at=.3, last_accepted_odometry_stamp=0., cause='ClockGap')
        for time in [.3, .35]:
            output = copy.deepcopy(state)
            output.update(time=time, motion_discontinuity=gap, health=['MotionDiscontinuity'])
            ticks.append(dict(input=dict(time=time, odometry=dict(stamp=0. if time == .3 else time, speed=0., yaw_rate=0.)), expected=output))
        result = verify_continuity(config, ticks)
        self.assertTrue(result['passed'])
        self.assertEqual(result['held_ticks'], 2)
        for key, value in [('motion_discontinuity', None), ('emergency', False),
                           ('position_variance', 1.), ('estimate', dict(pose=pose, speed=1.)),
                           ('tracks', [{'id': 1}]), ('predictions', [{'id': 1}]),
                           ('gnss_latency', dict(history_predictions=1)),
                           ('map_localization', dict(gnss_outage_covered=True)),
                           ('localization', dict(last_accepted_stamp=.35))]:
            changed = copy.deepcopy(ticks)
            changed[-1]['expected'][key] = value
            with self.subTest(field=key), self.assertRaises(ValueError):
                verify_continuity(config, changed)
        changed = copy.deepcopy(ticks)
        changed[-1]['expected']['command']['steering'] = .01
        with self.assertRaises(ValueError):
            verify_continuity(config, changed)


if __name__ == '__main__':
    unittest.main()
