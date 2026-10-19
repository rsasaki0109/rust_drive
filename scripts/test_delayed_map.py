#!/usr/bin/env python3
import copy
import unittest

from check_delayed_map import Conflict, rebuild, verify_ticks
from test_gnss_latency import config, fix
from test_gnss_map_history import covariance, pose


def measured_ticks():
    c = config()
    c['localization_map'] = {'allow_delayed_scans': True, 'max_gnss_outage_s': 10.}
    # Original NIS at delivery .1 precedes the late map at stamp 0.
    # The map changes the historical posterior but must not rewrite that NIS.
    original_nis = 1./1.0015
    p = .2515*.5/.7515
    x = .2515/.7515
    final_x, final_p = x*.5/(p+.5), p*.5/(p+.5)+.0015
    ticks = []
    for now, mean, variance, accepted, stamp, measurement, scan, prior, replayed in [
            (0., 0., .5, 1, 0., fix(0., variance=1.), None, None, 0),
            (.1, x, p, 2, .1, fix(.1, 1., .5), 0., pose(), 1),
            (.2, final_x, final_p, 2, .1, None, .1, pose(x), 1)]:
        diagnostic = {'last_observed_stamp': scan, 'last_accepted_stamp': scan,
                      'decision': 'Missing' if scan is None else 'Accepted', 'gnss_outage_covered': False,
                      'matched_pose': None if scan is None else pose(), 'covariance': None if scan is None else covariance(.5)}
        if prior is not None:
            diagnostic.update(acquisition_prior=prior, acquisition_delay_s=.1)
        ticks.append({'input': {'time': now, 'odometry': {'stamp': now, 'speed': 0., 'yaw_rate': 0.},
                               'gnss': measurement, 'lidar': None if scan is None else {'stamp': scan, 'points': []},
                               'lidar_failed': False},
                      'expected': {'estimate': {'pose': pose(mean), 'speed': 0.}, 'position_variance': 2.*variance,
                                   'health': [], 'command': {'acceleration': 0.}, 'map_localization': diagnostic,
                                   'localization': {'accepted_fixes': accepted, 'rejected_fixes': 0, 'last_accepted_stamp': stamp,
                                                    'last_observed_stamp': stamp, 'last_decision': 'Accepted',
                                                    'last_nis': 0. if stamp == 0. else original_nis},
                                   'gnss_latency': {'compensated_fixes': 0, 'replayed_gnss_corrections': replayed,
                                                    'replayed_map_corrections': 0, 'history_predictions': 2,
                                                    'history_gnss_corrections': accepted, 'history_map_corrections': 2,
                                                    'history_unavailable_fixes': 0}}})
    return c, ticks


class DelayedMap(unittest.TestCase):
    def test_prior_excludes_a_gnss_acquired_later_and_fractional_motion_is_covered(self):
        c = config()
        motions = [(0., .1, {'speed': 2., 'yaw_rate': 0.}), (.1, .2, {'speed': 2., 'yaw_rate': 0.})]
        prior = rebuild(c, motions, [fix(.1, .3)], [], .075, 1)
        self.assertAlmostEqual(prior.x[0], .15)
        self.assertAlmostEqual(prior.p[0][0], 1.+.015*.075)
        self.assertEqual(prior.accepted, 0)

    def test_map_that_passes_its_own_gate_cannot_invalidate_later_gnss(self):
        c = config()
        motions = [(0., .1, {'speed': 0., 'yaw_rate': 0.})]
        self.assertAlmostEqual(rebuild(c, motions, [], [(0., pose(5.), covariance(.02))], .1).x[0], 5./1.02)
        with self.assertRaises(Conflict):
            rebuild(c, motions, [fix(.1, 0., .001)], [(0., pose(5.), covariance(.02))], .1)

    def test_closed_form_history_retains_original_nis_and_rejects_corrupted_evidence(self):
        c, ticks = measured_ticks()
        verified = verify_ticks(c, ticks)
        self.assertTrue(verified['passed'])
        self.assertEqual(verified['replayed_gnss_corrections'], 1)
        for corruption in ['posterior', 'acquisition_prior', 'rewritten_nis', 'map_stamp', 'replay_count']:
            bad = copy.deepcopy(ticks)
            out = bad[-1]['expected']
            if corruption == 'posterior':
                out['position_variance'] *= 1.1
            elif corruption == 'acquisition_prior':
                out['map_localization']['acquisition_prior']['position']['x'] += .1
            elif corruption == 'rewritten_nis':
                out['localization']['last_nis'] = 1./.7515
            elif corruption == 'map_stamp':
                out['map_localization']['last_accepted_stamp'] = .2
            else:
                out['gnss_latency']['replayed_gnss_corrections'] += 1
            with self.assertRaises(ValueError, msg=corruption):
                verify_ticks(c, bad)


if __name__ == '__main__':
    unittest.main()
