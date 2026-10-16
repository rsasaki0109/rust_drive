#!/usr/bin/env python3
"""Independently exercise the historical-vs-current-host comparison boundary."""
import copy
import math
import gzip
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

import camera_history as history

ROOT = Path(__file__).resolve().parents[2]


def measured_fixture():
    return json.loads(gzip.decompress((ROOT / 'assets/camera-ci-v1/avx2-freshness.json.gz').read_bytes()))


class HistoricalConfidenceContract(unittest.TestCase):
    def setUp(self):
        self.original = dict(model_sha256='a' * 64, threshold=.3,
            events=[dict(now_s=1341845824.170751, acquired_s=1341845824.165751,
                decision='accepted', ready=False, tracks=[dict(track_id=1,
                bbox=[10., 20., 30., 40.], confidence=0.9397782683372498,
                last_observed_confidence=0.9397782683372498)])])

    def test_observed_avx2_confidence_keeps_metadata_and_all_nonconfidence_fields(self):
        actual = copy.deepcopy(self.original)
        track = actual['events'][0]['tracks'][0]
        track['confidence'] = 0.9397783279418945  # Measured fallback-kernel output.
        track['last_observed_confidence'] = track['confidence']
        audit = history.Comparison()
        audit.same(self.original, actual)
        rows = audit.report()['accepted_confidence_differences']
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]['absolute_f32_difference'], 2 ** -24)
        self.assertNotEqual(rows[0]['expected_f32le'], rows[0]['actual_f32le'])
        # This acceptance does not make the current-host exact check pass.
        self.assertNotEqual(self.original, actual)

    def test_same_float32_with_different_json_decimal_text_is_not_rounded_globally(self):
        audit = history.Comparison()
        audit.same({'confidence': .93977827}, {'confidence': .9397782683372498})
        row = audit.differences[0]
        self.assertEqual(row['expected_f32le'], row['actual_f32le'])
        with self.assertRaises(ValueError):
            audit.same({'timestamp_s': .93977827}, {'timestamp_s': .9397782683372498})

    def test_excess_confidence_error_is_rejected(self):
        # This independently chosen difference is substantially above the
        # pre-existing 1e-5 historical bound, irrespective of one-ULP rounding.
        with self.assertRaisesRegex(ValueError, r'\$\.confidence'):
            history.Comparison().same({'confidence': .75}, {'confidence': .7501})

    def test_the_original_score_threshold_cannot_be_crossed(self):
        bits = struct.unpack('<I', struct.pack('<f', .3))[0]
        below = struct.unpack('<f', struct.pack('<I', bits - 1))[0]
        with self.assertRaises(ValueError):
            history.Comparison().same({'confidence': .3}, {'confidence': below})

    def test_boundary_is_computed_from_represented_f32(self):
        bits = struct.unpack('<I', struct.pack('<f', .75))[0]
        inside = struct.unpack('<f', struct.pack('<I', bits + 167))[0]
        outside = struct.unpack('<f', struct.pack('<I', bits + 168))[0]
        self.assertLess(inside - .75, 1e-5)
        self.assertGreater(outside - .75, 1e-5)
        history.Comparison().same({'confidence': .75}, {'confidence': inside})
        with self.assertRaises(ValueError):
            history.Comparison().same({'confidence': .75}, {'confidence': outside})

    def test_every_coordinate_clock_and_other_numeric_leaf_remains_exact(self):
        for key, value in [('bbox', [10., 20., 30., math.nextafter(40., math.inf)]),
                           ('track_id', 2)]:
            actual = copy.deepcopy(self.original)
            actual['events'][0]['tracks'][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                history.Comparison().same(self.original, actual)
        for key in ('now_s', 'acquired_s'):
            actual = copy.deepcopy(self.original)
            actual['events'][0][key] = math.nextafter(actual['events'][0][key], math.inf)
            with self.subTest(key=key), self.assertRaises(ValueError):
                history.Comparison().same(self.original, actual)
        with self.assertRaises(ValueError):
            history.Comparison().same({'threshold': .3}, {'threshold': .30000001})

    def test_decisions_identities_order_counts_and_source_receipts_remain_exact(self):
        for modify in [lambda x: x.update(model_sha256='b' * 64),
                       lambda x: x['events'][0].update(decision='stale_sample'),
                       lambda x: x['events'][0].update(ready=True),
                       lambda x: x['events'].append(copy.deepcopy(x['events'][0])),
                       lambda x: x['events'][0]['tracks'].clear(),
                       lambda x: x.update(unexpected=True),
                       lambda x: x.pop('threshold')]:
            actual = copy.deepcopy(self.original)
            modify(actual)
            with self.assertRaises(ValueError):
                history.Comparison().same(self.original, actual)
        with self.assertRaises(ValueError):
            history.Comparison().same({'order': [1, 2]}, {'order': [2, 1]})

    def test_nonfinite_out_of_range_and_wrong_type_confidences_are_rejected(self):
        for value in (math.nan, math.inf, -math.inf, -.000001, 1.000001,
                      True, False, '.75', None, [], {}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                history.Comparison().same({'confidence': .75}, {'confidence': value})

    def test_unrelated_metadata_named_with_a_similar_suffix_is_exact(self):
        for key in ('confidence_threshold', 'mean_confidence', 'result_age_s'):
            with self.subTest(key=key), self.assertRaises(ValueError):
                history.Comparison().same({key: .75}, {key: math.nextafter(.75, math.inf)})

    def test_identical_input_records_no_accepted_changes(self):
        audit = history.Comparison()
        audit.same(self.original, copy.deepcopy(self.original))
        self.assertEqual(audit.differences, [])

    def test_complete_recorded_fallback_trace_preserves_the_published_historical_contract(self):
        fixture = measured_fixture()
        published = json.loads((ROOT / 'assets/camera-freshness-v1/verification/cases/recorded-healthy-first.json').read_text())
        self.assertEqual(fixture['baseline_freshness'], published['results'])
        self.assertNotEqual(fixture['baseline_freshness'], fixture['fallback_freshness'])
        audit = history.Comparison()
        audit.same(fixture['baseline_freshness'], fixture['fallback_freshness'])
        self.assertTrue(audit.differences)
        self.assertTrue(all(row['path'].endswith(('.confidence', '.last_observed_confidence'))
                            for row in audit.differences))

    def test_recorded_fallback_measurements_replay_exactly_in_the_original_native_guard(self):
        guard = ROOT / 'integrations/camera-tracking/target/release/rustdriving-camera-guard'
        if not guard.is_file():
            self.skipTest('build the independent camera-tracking workspace to run native replay')
        fixture = measured_fixture()
        with tempfile.TemporaryDirectory(prefix='camera-history-') as temporary:
            path = Path(temporary)
            source, result = path / 'input.json', path / 'result.json'
            source.write_text(json.dumps(fixture['guard_manifest'], allow_nan=False))
            process = subprocess.run([str(guard), '--sequence', str(source), '--output', str(result)],
                capture_output=True, text=True, timeout=30)
            self.assertEqual(process.returncode, 0, process.stderr)
            actual = json.loads(result.read_text())['results']
            self.assertEqual(actual, fixture['fallback_freshness'])
            # Historical tolerance never substitutes for current replay equality.
            self.assertNotEqual(actual, fixture['baseline_freshness'])


if __name__ == '__main__':
    unittest.main()
