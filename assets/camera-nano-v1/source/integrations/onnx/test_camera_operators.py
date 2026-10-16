"""Exercise the strict/current and bounded/historical boundary on real reports."""
import copy
import gzip
import importlib.util
import json
from pathlib import Path
import struct
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKET = ROOT / 'assets/camera-operators-v2'


def module(name):
    spec = importlib.util.spec_from_file_location(name, HERE / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


operator = module('camera_operator_check')
parallel = module('camera_repeat_check')


def read(name):
    path = PACKET / name
    if path.exists():
        return json.loads(path.read_text())
    return json.loads(gzip.decompress(Path(str(path) + '.gz').read_bytes()))


def adjacent(value):
    bits = struct.unpack('<I', struct.pack('<f', value))[0]
    return struct.unpack('<f', struct.pack('<I', bits + 1))[0]


class CurrentHostComparison(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = read('operator-validation/reports/tum-0-first.json')
        cls.lowered = read('parallel-validation/reports/tum-0-first.json')
        cls.current = read('parallel-validation/current-reference/tum-0.json')
        cls.saved = json.loads((ROOT / 'assets/camera-sequence-v1/recorded/exports/first-0.json').read_text())

    def audit(self, report, saved=None, historical=None):
        operator.audit(report, self.saved if saved is None else saved,
                       self.profile['compiled_sources'], True, False, historical)

    def history(self):
        return operator.latency.camera_history.Comparison()

    def test_real_profile_and_parallel_reference_pass(self):
        self.audit(self.profile, historical=self.history())
        parallel.check_analysis(self.lowered, self.current)

    def test_current_confidence_one_ulp_rejected_with_history_enabled(self):
        bad = copy.deepcopy(self.profile)
        detection = bad['views'][0]['detections'][0]
        detection['confidence'] = adjacent(detection['confidence'])
        with self.assertRaisesRegex(ValueError, 'current decoded'):
            self.audit(bad, historical=self.history())

    def test_current_merge_one_ulp_rejected(self):
        bad = copy.deepcopy(self.profile)
        detection = bad['merged']['detections'][0]
        detection['confidence'] = adjacent(detection['confidence'])
        with self.assertRaisesRegex(ValueError, 'current-host merge'):
            self.audit(bad, historical=self.history())

    def test_current_decision_change_rejected(self):
        bad = copy.deepcopy(self.profile)
        bad['result']['comparison_work'] += 1
        with self.assertRaisesRegex(ValueError, 'current-host final'):
            self.audit(bad, historical=self.history())

    def test_raw_output_hash_change_rejected(self):
        bad = copy.deepcopy(self.profile)
        bad['views'][0]['output_tensor_f32le_sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'raw output bit'):
            self.audit(bad, historical=self.history())

    def test_old_schema_cannot_claim_current_reference_parity(self):
        bad = copy.deepcopy(self.profile)
        bad['schema_version'] = 1
        with self.assertRaisesRegex(ValueError, 'diagnostic algorithm'):
            self.audit(bad, historical=self.history())

    def test_historical_confidence_drift_explicitly_recorded(self):
        old = copy.deepcopy(self.saved)
        detection = old['base_prediction']['views'][0]['prediction']['detections'][0]
        detection['confidence'] = adjacent(detection['confidence'])
        history = self.history()
        self.audit(self.profile, old, history)
        self.assertTrue(any(row['absolute_f32_difference'] > 0 for row in history.differences))
        with self.assertRaises(ValueError):
            self.audit(self.profile, old)

    def test_historical_coordinate_one_ulp_rejected(self):
        old = copy.deepcopy(self.saved)
        box = old['base_prediction']['views'][0]['prediction']['detections'][0]['bbox']
        box[0] = adjacent(box[0])
        with self.assertRaises(ValueError):
            self.audit(self.profile, old, self.history())

    def test_historical_image_identity_change_rejected(self):
        old = copy.deepcopy(self.saved)
        old['source_image_sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'encoded image'):
            self.audit(self.profile, old, self.history())

    def test_parallel_current_confidence_one_ulp_rejected_by_default(self):
        bad = copy.deepcopy(self.lowered)
        detection = bad['base_prediction']['views'][0]['prediction']['detections'][0]
        detection['confidence'] = adjacent(detection['confidence'])
        with self.assertRaisesRegex(ValueError, 'per-view measurements'):
            parallel.check_analysis(bad, self.current)
        history = self.history()
        parallel.check_analysis(bad, self.current, history)
        self.assertTrue(history.differences)

    def test_parallel_final_decision_change_rejected(self):
        bad = copy.deepcopy(self.lowered)
        bad['result']['comparison_work'] += 1
        with self.assertRaisesRegex(ValueError, 'final consistency'):
            parallel.check_analysis(bad, self.current, self.history())

    def test_parallel_view_order_change_rejected(self):
        bad = copy.deepcopy(self.lowered)
        bad['base_prediction']['views'].reverse()
        with self.assertRaisesRegex(ValueError, 'view pixels'):
            parallel.check_analysis(bad, self.current, self.history())


if __name__ == '__main__':
    unittest.main()
