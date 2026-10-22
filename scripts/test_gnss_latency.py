#!/usr/bin/env python3
import copy
import math
import unittest

from check_gnss_latency import Filter, chronological, verify_ticks


def config():
    return {'initial_pose': {'position': {'x': 0., 'y': 0.}, 'yaw': 0.}, 'gnss_latency_compensation': True}


def fix(stamp, x=0., variance=.02):
    return {'stamp': stamp, 'position': {'x': x, 'y': 0.}, 'variance': variance}


class GnssLatency(unittest.TestCase):
    def test_joint_update_matches_closed_form_and_chassis_motion_uses_measured_yaw(self):
        f = Filter(config())
        f.correct(fix(0., 1., .25))
        self.assertAlmostEqual(f.x[0], .8)
        self.assertAlmostEqual(f.p[0][0], .2)
        self.assertAlmostEqual(f.p[1][1], .2)
        c = config()
        c['rear_axle_offset_m'] = 1.3
        f = Filter(c)
        f.predict({'speed': 2., 'yaw_rate': .2}, .1)
        self.assertAlmostEqual(f.x[0], (2.*math.cos(.01)-.26*math.sin(.01))*.1)
        self.assertAlmostEqual(f.x[1], (2.*math.sin(.01)+.26*math.cos(.01))*.1)

    def test_assimilating_the_old_position_at_delivery_introduces_a_real_motion_error(self):
        motion = {'speed': 2., 'yaw_rate': 0.}
        intervals = [(0., .5, motion), (.5, 1., motion)]
        delayed = fix(.5, 1.)
        aligned = chronological(config(), intervals, [(.5, delayed)])
        ordinary = chronological(config(), intervals, [(1., delayed)])
        self.assertAlmostEqual(aligned.x[0], 2.)
        self.assertGreater(2.-ordinary.x[0], .9)

    def test_raw_tick_audit_rejects_pose_covariance_and_freshness_corruption(self):
        def expected(accepted, stamp, variance, compensated, history):
            return {'estimate': {'pose': {'position': {'x': 0., 'y': 0.}, 'yaw': 0.}, 'speed': 0.},
                    'position_variance': variance,
                    'localization': {'accepted_fixes': accepted, 'rejected_fixes': 0, 'last_accepted_stamp': stamp,
                                     'last_observed_stamp': stamp, 'last_decision': 'Accepted', 'last_nis': 0.},
                    'gnss_latency': {'compensated_fixes': compensated, 'history_predictions': history, 'history_unavailable_fixes': 0},
                    'health': [], 'command': {'acceleration': 0.}}
        # Scalar closed form: initial P=1 -> .5 after R=1. At t=.05,
        # P=.50075 -> P*R/(P+R) for R=.5, then .00075 motion noise.
        posterior = .50075*.5/1.00075+.00075
        ticks = [{'input': {'time': 0., 'odometry': {'stamp': 0., 'speed': 0., 'yaw_rate': 0.}, 'gnss': fix(0., variance=1.)},
                  'expected': expected(1, 0., 1., 0, 0)},
                 {'input': {'time': .1, 'odometry': {'stamp': .1, 'speed': 0., 'yaw_rate': 0.}, 'gnss': fix(.05, variance=.5)},
                  'expected': expected(2, .05, 2.*posterior, 1, 1)}]
        self.assertTrue(verify_ticks(config(), ticks)['passed'])
        for field in ['pose', 'covariance', 'freshness']:
            corrupted = copy.deepcopy(ticks)
            out = corrupted[-1]['expected']
            if field == 'pose':
                out['estimate']['pose']['position']['x'] = .1
            elif field == 'covariance':
                out['position_variance'] *= 1.1
            else:
                out['localization']['last_accepted_stamp'] = .1
            with self.assertRaises(ValueError):
                verify_ticks(config(), corrupted)


if __name__ == '__main__':
    unittest.main()
