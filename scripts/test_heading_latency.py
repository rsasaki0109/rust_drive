#!/usr/bin/env python3
"""Closed-form and corruption checks for acquired-heading chronology."""
import copy
import unittest

from check_heading_latency import materialize, reconstruct, specifications, verify_scenario


def fixture():
    config = dict(absolute_heading_fusion=True, gnss_latency_compensation=True, heading_latency_compensation=True,
                  initial_pose=dict(position=dict(x=0., y=0.), yaw=0.))
    def tick(now, covariance, yaw, heading, diagnostics, health):
        return dict(input=dict(time=now, odometry=dict(stamp=now, speed=0., yaw_rate=0.), absolute_heading=heading),
                    expected=dict(estimate=dict(pose=dict(position=dict(x=0., y=0.), yaw=yaw), speed=0.),
                                  pose_covariance=covariance, position_variance=covariance[0][0]+covariance[1][1],
                                  absolute_heading=diagnostics, health=health, command=dict(acceleration=-6., steering=0.),
                                  localization=dict(last_observed_stamp=None, last_accepted_stamp=None, last_decision=None,
                                                    last_nis=None, accepted_fixes=0, rejected_fixes=0),
                                  gnss_latency=dict(compensated_fixes=0)))
    initial = dict(last_observed_stamp=None, last_accepted_stamp=None, last_decision=None, last_nis=None,
                   accepted_samples=0, rejected_samples=0)
    ticks = [tick(0., [[1., 0., 0.], [0., 1., 0.], [0., 0., .03]], 0., None, initial, ['StaleAbsoluteHeading', 'StaleGnss'])]
    before = .03+.0005*.025
    total = before+.01
    variance = before*.01/total+.0005*.025
    diagnostics = dict(last_observed_stamp=.025, last_accepted_stamp=.025, last_decision='Accepted',
                       last_nis=.02**2/total, accepted_samples=1, rejected_samples=0)
    ticks.append(tick(.05, [[1.+.015*.05, 0., 0.], [0., 1.+.015*.05, 0.], [0., 0., variance]],
                      before/total*.02, dict(stamp=.025, yaw_rad=.02, variance_rad2=.01), diagnostics, ['StaleGnss']))
    ticks[1]['expected']['gnss_latency']['history_heading_corrections'] = 1
    return config, ticks


class HeadingLatencyTests(unittest.TestCase):
    def test_fractional_acquisition_has_closed_form_covariance_and_own_clock(self):
        config, ticks = fixture()
        report = reconstruct(config, ticks)
        self.assertEqual(report['accepted_headings'], 1)
        self.assertLess(report['maximum_state_difference'], 1e-15)
        self.assertLess(report['maximum_covariance_difference'], 1e-15)

    def test_time_count_covariance_state_and_replay_counter_corruptions_fail(self):
        for mutation in ['stamp', 'count', 'covariance', 'state', 'replay', 'gnss-clock']:
            config, ticks = fixture(); out = ticks[-1]['expected']
            if mutation == 'stamp': out['absolute_heading']['last_accepted_stamp'] = .05
            elif mutation == 'count': out['absolute_heading']['accepted_samples'] = 2
            elif mutation == 'covariance': out['pose_covariance'][2][2] += .001
            elif mutation == 'state': out['estimate']['pose']['yaw'] += .001
            elif mutation == 'replay': out['gnss_latency']['replayed_gnss_corrections'] = 1
            else: out['localization']['last_accepted_stamp'] = 0.
            with self.assertRaises(ValueError): reconstruct(config, ticks)

    def test_lost_policy_changed_acquisition_and_missing_braking_fail(self):
        for mutation in ['policy', 'time', 'future', 'command', 'health', 'motion']:
            config, ticks = fixture()
            if mutation == 'policy': config.pop('heading_latency_compensation')
            elif mutation == 'time': ticks[-1]['input']['absolute_heading']['stamp'] = .02
            elif mutation == 'future': ticks[-1]['input']['absolute_heading']['stamp'] = .06
            elif mutation == 'command': ticks[0]['expected']['command']['acceleration'] = 0.
            elif mutation == 'health': ticks[0]['expected']['health'].remove('StaleAbsoluteHeading')
            else: ticks[-1]['input']['odometry'] = None
            with self.assertRaises(ValueError): reconstruct(config, ticks)

    def test_false_defaults_cannot_admit_lost_true_policies_or_changed_worlds(self):
        verify_scenario(dict(duration=45.), dict(duration=45., heading_latency_compensation=False))
        for declared in [dict(duration=45., heading_latency_compensation=True), dict(duration=44., heading_latency_compensation=False)]:
            with self.assertRaises(ValueError): verify_scenario(dict(duration=45.), declared)

    def test_nonfinite_state_or_motion_cannot_pass_nan_comparisons(self):
        for value in [float('nan'), float('inf')]:
            for field in ['position', 'yaw', 'speed', 'motion']:
                config, ticks = fixture()
                if field == 'position': ticks[-1]['expected']['estimate']['pose']['position']['x'] = value
                elif field == 'yaw': ticks[-1]['expected']['estimate']['pose']['yaw'] = value
                elif field == 'speed': ticks[-1]['expected']['estimate']['speed'] = value
                else: ticks[-1]['input']['odometry']['speed'] = value
                with self.assertRaises(ValueError): reconstruct(config, ticks)

    def test_fixed_comparisons_preserve_every_world_sensor_and_deadline(self):
        specs = specifications(['reference-2d', 'rne-dynamic'])
        self.assertEqual(len(specs), 26)
        for spec in specs:
            if spec['kind'] != 'base' or not spec['compensated'] or spec['mode'] == 'equal-time': continue
            control = next(s for s in specs if s['kind'] == 'base' and not s['compensated']
                           and all(s[k] == spec[k] for k in ['plant', 'seed', 'heading_delay_ticks', 'delay_ticks']))
            _, positive = materialize(spec); _, negative = materialize(control)
            positive.pop('name'); negative.pop('name')
            positive.pop('heading_latency_compensation'); negative.pop('heading_latency_compensation')
            self.assertEqual(positive, negative)
            self.assertEqual(positive['duration'], 45.)

    def test_additional_replay_composition_does_not_replace_original_conditions(self):
        original = specifications(['reference-2d', 'rne-dynamic'])
        followup = specifications(['reference-2d', 'rne-dynamic'], replay_composition=True)
        self.assertEqual(len(original), 26)
        self.assertEqual(len(followup), 2)
        for spec in followup:
            _, value = materialize(spec)
            self.assertEqual(value['heading_sensor']['period_ticks'], 2)
            self.assertEqual(value['heading_sensor']['delay_ticks'], 4)
            self.assertEqual(value['sensor_timing']['gnss_delay_ticks'], 1)
            self.assertEqual(value['duration'], 45.)


if __name__ == '__main__': unittest.main()
