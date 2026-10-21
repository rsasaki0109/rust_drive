#!/usr/bin/env python3
"""Corruption and analytic controls for the independent heading-history audit."""
import copy
import math
import unittest

from check_heading_history import chronological, reconstruct, verify_scenario


def fixture():
    config = dict(absolute_heading_fusion=True, gnss_latency_compensation=True,
                  initial_pose=dict(position=dict(x=0., y=0.), yaw=0.))
    total = .03+1e-4
    yaw, variance = .03/total*.02, .03*1e-4/total
    output = dict(time=0., estimate=dict(pose=dict(position=dict(x=0., y=0.), yaw=yaw), speed=0.),
                  pose_covariance=[[1., 0., 0.], [0., 1., 0.], [0., 0., variance]], position_variance=2.,
                  absolute_heading=dict(last_observed_stamp=0., last_accepted_stamp=0., last_decision='Accepted',
                                        last_nis=.02**2/total, accepted_samples=1, rejected_samples=0),
                  localization=dict(last_observed_stamp=None, last_accepted_stamp=None, last_decision=None,
                                    last_nis=None, accepted_fixes=0, rejected_fixes=0),
                  gnss_latency=dict(compensated_fixes=0, history_heading_corrections=1),
                  health=['StaleGnss'], command=dict(acceleration=-6., steering=0.))
    tick = dict(input=dict(time=0., absolute_heading=dict(stamp=0., yaw_rad=.02, variance_rad2=1e-4)), expected=output)
    return config, [tick]


class HeadingHistoryTests(unittest.TestCase):
    def test_omitted_false_policy_is_normalized_without_allowing_a_lost_true_policy(self):
        verify_scenario(dict(duration=45.), dict(duration=45., gnss_latency_compensation=False))
        for declared in [dict(duration=45., gnss_latency_compensation=True), dict(duration=40., gnss_latency_compensation=False)]:
            with self.assertRaises(ValueError): verify_scenario(dict(duration=45.), declared)

    def test_closed_form_single_heading_and_unrenewed_gnss(self):
        config, ticks = fixture()
        report = reconstruct(config, ticks)
        self.assertEqual(report['accepted_headings'], 1)
        self.assertLess(report['maximum_covariance_difference'], 1e-15)

    def test_replay_cannot_forge_a_heading_stamp_count_or_rewind_count(self):
        for key, value in [('last_accepted_stamp', .1), ('accepted_samples', 2), ('rejected_samples', 1)]:
            config, ticks = fixture()
            ticks[0]['expected']['absolute_heading'][key] = value
            with self.assertRaises(ValueError): reconstruct(config, ticks)
        config, ticks = fixture()
        ticks[0]['expected']['gnss_latency']['replayed_heading_corrections'] = 1
        with self.assertRaises(ValueError): reconstruct(config, ticks)

    def test_corrupted_state_and_full_covariance_are_rejected(self):
        for field in ['state', 'covariance']:
            config, ticks = fixture()
            if field == 'state': ticks[0]['expected']['estimate']['pose']['yaw'] += .001
            else: ticks[0]['expected']['pose_covariance'][2][2] += .001
            with self.assertRaises(ValueError): reconstruct(config, ticks)

    def test_heading_cannot_forge_gnss_acceptance_or_remove_required_braking(self):
        config, ticks = fixture()
        for mutation in ['clock', 'health', 'command']:
            changed = copy.deepcopy(ticks)
            if mutation == 'clock': changed[0]['expected']['localization']['last_accepted_stamp'] = 0.
            elif mutation == 'health': changed[0]['expected']['health'] = []
            else: changed[0]['expected']['command']['acceleration'] = 0.
            with self.assertRaises(ValueError): reconstruct(config, changed)

    def test_fractional_acquisition_conditions_the_pose_before_the_remaining_motion(self):
        config, _ = fixture()
        motions = [(0., .05, dict(stamp=.05, speed=4., yaw_rate=0.))]
        sample = dict(stamp=.025, yaw_rad=.02, variance_rad2=.01)
        result = chronological(config, motions, [], [sample], .05)
        total = .03+.0005*.025+.01
        yaw = (.03+.0005*.025)/total*.02
        self.assertAlmostEqual(result.x[0], .1+.1*math.cos(yaw), places=13)
        self.assertAlmostEqual(result.x[1], .003/total*.02+.1*math.sin(yaw), places=13)
        self.assertAlmostEqual(result.x[2], yaw, places=13)
        # Equal-time GNSS must see the prior before heading, irrespective of delivery.
        prior = chronological(config, motions, [], [sample], .025, priority=1)
        self.assertAlmostEqual(prior.x[2], 0., places=13)


if __name__ == '__main__': unittest.main()
