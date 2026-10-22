#!/usr/bin/env python3
"""Adversarial arithmetic witnesses for the independent return-policy scorer."""
import unittest

from check_lane_return_speeds import speed_audit


def fixture(states):
    frames = [{'truth': {'pose': {'position': {'x': x, 'y': 0.}}, 'speed': speed},
               'lane_change': {'phase': phase}} for x, speed, phase in states]
    return {'frames': frames, 'summary': {'steps': len(frames)}, 'vehicle': {'radius': 1.25},
            'route': {'points': [{'x': 0., 'y': 0.}, {'x': 100., 'y': 0.}]}}


class ReturnSpeeds(unittest.TestCase):
    def test_return_cap_cannot_disappear_after_leaving_the_original_low_speed_interval(self):
        run = fixture([(60., 2., 'Changing'), (61., 2., 'Aborting'), (62., 3., 'Aborted')])
        zones = [{'start_s_m': 0., 'end_s_m': 50., 'speed_m_s': 2.},
                 {'start_s_m': 50., 'end_s_m': 100., 'speed_m_s': 4.}]
        score = speed_audit(run, zones, 2.)
        self.assertFalse(score['passed'])
        self.assertEqual(score['violation_intervals'], 1)
        self.assertEqual(score['maximum_excess_m_s'], 1.)

    def test_switch_does_not_apply_a_new_cap_to_the_preceding_interval(self):
        run = fixture([(10., 3., 'Changing'), (11., 2., 'Aborting'), (12., 2., 'Aborted')])
        zones = [{'start_s_m': 0., 'end_s_m': 100., 'speed_m_s': 4.}]
        self.assertTrue(speed_audit(run, zones, 2.)['passed'])
        run['frames'][1]['truth']['speed'] = 2.2
        self.assertFalse(speed_audit(run, zones, 2.)['passed'])

    def test_unexercised_tail_is_allowed_but_a_swept_thin_limit_cannot_be_skipped(self):
        run = fixture([(10., 3., 'Changing'), (11., 3., 'Changing')])
        zones = [{'start_s_m': 10.4, 'end_s_m': 10.41, 'speed_m_s': 2.},
                 {'start_s_m': 80., 'end_s_m': 90., 'speed_m_s': 1.}]
        self.assertFalse(speed_audit(run, zones, 2.)['passed'])
        run['summary']['steps'] += 1
        with self.assertRaises(ValueError): speed_audit(run, zones, 2.)


if __name__ == '__main__': unittest.main()
