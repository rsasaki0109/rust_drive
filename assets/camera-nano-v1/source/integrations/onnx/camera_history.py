"""Narrow historical comparison for CPU-dependent detector confidences.

The existing tiled-detector historical check allows 1e-5 absolute f32 error.
Use that bound only for confidence measurements. Coordinates, clocks, policy,
identities, decisions, list order and every other value remain exact. Current
host repeats and independent guard/tracker replays must not use this helper.
"""
import math
import struct

CONFIDENCE_MAX_ABS = 1e-5
CONFIDENCE_FIELDS = frozenset(('confidence', 'last_observed_confidence'))


class Comparison:
    def __init__(self):
        self.differences = []

    def same(self, expected, actual, path='$', field=None):
        if isinstance(expected, dict):
            if not isinstance(actual, dict) or expected.keys() != actual.keys():
                self.fail(path, expected, actual)
            for key in expected:
                self.same(expected[key], actual[key], path + '.' + str(key), key)
        elif isinstance(expected, list):
            if not isinstance(actual, list) or len(expected) != len(actual):
                self.fail(path, expected, actual)
            for index, (left, right) in enumerate(zip(expected, actual)):
                self.same(left, right, path + f'[{index}]', field)
        elif field in CONFIDENCE_FIELDS:
            if (type(expected) not in (int, float) or type(actual) not in (int, float)
                    or not math.isfinite(expected) or not math.isfinite(actual)
                    or not 0 <= expected <= 1 or not 0 <= actual <= 1):
                self.fail(path, expected, actual)
            old_bytes = struct.pack('<f', expected)
            new_bytes = struct.pack('<f', actual)
            left = struct.unpack('<f', old_bytes)[0]
            right = struct.unpack('<f', new_bytes)[0]
            score_threshold = struct.unpack('<f', struct.pack('<f', .3))[0]
            if ((left >= score_threshold) != (right >= score_threshold)
                    or abs(left - right) > CONFIDENCE_MAX_ABS):
                self.fail(path, expected, actual)
            if expected != actual:
                self.differences.append(dict(path=path, expected=expected, actual=actual,
                    expected_f32le=old_bytes.hex(), actual_f32le=new_bytes.hex(),
                    absolute_f32_difference=abs(left - right)))
        elif type(expected) is not type(actual) or expected != actual:
            self.fail(path, expected, actual)

    @staticmethod
    def fail(path, expected, actual):
        raise ValueError(f'historical camera mismatch {path}: '
                         f'expected={repr(expected)[:240]}, actual={repr(actual)[:240]}')

    def report(self):
        return dict(confidence_max_absolute_f32_difference=CONFIDENCE_MAX_ABS,
                    score_threshold_crossing_rejected=True, other_fields_exact=True,
                    accepted_confidence_differences=self.differences,
                    same_host_bitwise_equivalence_claimed=False)
