#!/usr/bin/env python3
import copy
import unittest

from check_gnss_map_history import MapFilter, chronological, verify_ticks
from test_gnss_latency import config, fix


def pose(x=0., yaw=0.):
    return {'position': {'x': x, 'y': 0.}, 'yaw': yaw}


def covariance(xy, yaw=.03):
    return [[xy, 0., 0.], [0., xy, 0.], [0., 0., yaw]]


class MapHistory(unittest.TestCase):
    def test_joint_map_update_matches_closed_form_without_inventing_gnss(self):
        f = MapFilter(config())
        self.assertTrue(f.map_correct(pose(1., .04), [[.25, 0., 0.], [0., .5, 0.], [0., 0., .01]]))
        self.assertAlmostEqual(f.x[0], .8)
        self.assertAlmostEqual(f.x[2], .03)
        self.assertAlmostEqual(f.p[0][0], .2)
        self.assertAlmostEqual(f.p[1][1], 1./3.)
        self.assertAlmostEqual(f.p[2][2], .0075)
        self.assertIsNone(f.accepted_stamp)
        self.assertEqual(f.accepted, 0)

    def test_same_stamp_gnss_precedes_map_and_conflicting_map_refuses_reconstruction(self):
        f = chronological(config(), [], [(0., fix(0., 1., .25))], [(0., pose(), covariance(.5))])
        self.assertAlmostEqual(f.x[0], .8*.5/.7)
        self.assertAlmostEqual(f.p[0][0], .2*.5/.7)
        with self.assertRaisesRegex(ValueError, 'conflicts'):
            chronological(config(), [], [(0., fix(0., 5., .001))], [(0., pose(), covariance(.02))])

    def test_full_tick_oracle_rejects_changed_map_covariance_freshness_and_replay_count(self):
        c = config()
        c['localization_map'] = {'max_gnss_outage_s': 1.}
        # Scalar XY closed forms. The second GNSS is inserted before the map
        # update at the identical acquisition stamp .1, after that map arrived.
        a = .50075*.5/1.00075
        b = a+.00075
        posterior1 = b*.5/(b+.5)
        d = b*.4/(b+.4)
        posterior2 = d*.5/(d+.5)+.0015
        ticks = []
        for now, variance, accepted, stamp, measurement, map_stamp, replayed in [
                (0., .5, 0, None, None, 0., 0),
                (.1, posterior1, 1, .05, fix(.05, variance=.5), .1, 0),
                (.2, posterior2, 2, .1, fix(.1, variance=.4), .1, 1)]:
            diagnostic = {'last_observed_stamp': map_stamp, 'last_accepted_stamp': map_stamp,
                          'decision': 'Accepted', 'matched_pose': pose(),
                          'covariance': covariance(1. if map_stamp == 0. else .5), 'gnss_outage_covered': False}
            ticks.append({'input': {'time': now, 'odometry': {'stamp': now, 'speed': 0., 'yaw_rate': 0.},
                                    'gnss': measurement, 'lidar': {'stamp': now, 'points': []} if now < .2 else None,
                                    'lidar_failed': False},
                          'expected': {'estimate': {'pose': pose(), 'speed': 0.}, 'position_variance': 2.*variance,
                                       'health': ['StaleGnss'] if stamp is None else [], 'command': {'acceleration': -6.},
                                       'map_localization': diagnostic,
                                       'localization': {'accepted_fixes': accepted, 'rejected_fixes': 0,
                                                        'last_accepted_stamp': stamp, 'last_observed_stamp': stamp,
                                                        'last_nis': None if stamp is None else 0.,
                                                        'last_decision': None if stamp is None else 'Accepted'},
                                       'gnss_latency': {'compensated_fixes': accepted, 'history_predictions': 2,
                                                        'history_unavailable_fixes': 0, 'history_map_corrections': 1,
                                                        'replayed_map_corrections': replayed}}})
        self.assertTrue(verify_ticks(c, ticks)['passed'])
        for mutation in ['map_covariance', 'map_stamp', 'posterior', 'replayed_maps', 'gnss_stamp']:
            corrupted = copy.deepcopy(ticks)
            out = corrupted[-1]['expected']
            if mutation == 'map_covariance':
                corrupted[1]['expected']['map_localization']['covariance'][0][0] *= 2.
            elif mutation == 'map_stamp':
                out['map_localization']['last_accepted_stamp'] = .2
            elif mutation == 'posterior':
                out['position_variance'] *= 1.1
            elif mutation == 'replayed_maps':
                out['gnss_latency']['replayed_map_corrections'] = 0
            else:
                out['localization']['last_accepted_stamp'] = .2
            with self.assertRaises(ValueError, msg=mutation):
                verify_ticks(c, corrupted)


if __name__ == '__main__':
    unittest.main()
