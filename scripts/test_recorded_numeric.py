"""Keep historical rounding tolerance separate from policy and acceptance."""
import math
import sys
import unittest

from recorded_numeric import compare_recorded


class HistoricalNumericTests(unittest.TestCase):
    def test_observed_backend_quaternion_roundoff_is_reported(self):
        before = {'evaluation_only_truth': {'quaternion_wxyz': [1., 0., 0., 1.3877787807814457e-17]}}
        after = {'evaluation_only_truth': {'quaternion_wxyz':
                  [1.0000000000000002, 0., 2.7755575615628914e-17, 2.7755575615628914e-17]}}
        result = compare_recorded(before, after)
        self.assertFalse(result['exact'])
        self.assertEqual(result['rounded_values'], 3)

    def test_pose_changes_larger_than_roundoff_are_rejected(self):
        with self.assertRaises(ValueError):
            compare_recorded({'translation_m': [1., 2., 3.]},
                             {'translation_m': [1.000001, 2., 3.]})

    def test_clocks_reject_even_one_ulp(self):
        for name in ['timestamp', 'time_s', 'acquisition_stamp', 'dt']:
            with self.subTest(clock=name), self.assertRaises(ValueError):
                compare_recorded({name: 1.e9}, {name: math.nextafter(1.e9, math.inf)})

    def test_configuration_rejects_even_one_ulp(self):
        for name in ['registration_config', 'effective_calibration', 'motion_policy']:
            with self.subTest(config=name), self.assertRaises(ValueError):
                compare_recorded({name: {'tolerance': .1}},
                                 {name: {'tolerance': math.nextafter(.1, math.inf)}})

    def test_acceptance_counts_and_source_identities_stay_exact(self):
        for before, after in [(True, False), (11, 10), ('sha256-original', 'sha256-other'),
                              (1, 1.), (True, 1)]:
            with self.subTest(before=before, after=after), self.assertRaises(ValueError):
                compare_recorded({'value': before}, {'value': after})

    def test_missing_fields_and_frames_are_rejected(self):
        for before, after in [({'frames': [1, 2]}, {'frames': [1]}),
                              ({'frame': 1, 'accepted': False}, {'frame': 1})]:
            with self.assertRaises(ValueError):
                compare_recorded(before, after)

    def test_nonfinite_values_and_excess_roundoff_are_rejected(self):
        for value in [math.nan, math.inf, -math.inf]:
            with self.assertRaises(ValueError):
                compare_recorded(value, value)
        with self.assertRaises(ValueError):
            compare_recorded(1., 1. + 16 * sys.float_info.epsilon)

    def test_exact_values_remain_identical(self):
        value = dict(timestamp=1.e9, accepted=False, count=11, pose=[0., 1., -2.])
        self.assertTrue(compare_recorded(value, value)['exact'])


if __name__ == '__main__':
    unittest.main()
