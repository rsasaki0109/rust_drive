"""Check Nano provenance, exact f32 measurements and unchanged clock acceptance."""
import copy
import gzip
import importlib.util
import json
import math
from pathlib import Path
import struct
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKET = ROOT / 'assets/camera-nano-v1'
spec = importlib.util.spec_from_file_location('nano_tests', HERE / 'camera_nano_check.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def read(name):
    path = PACKET / name
    if path.exists():return json.loads(path.read_text())
    return json.loads(gzip.decompress(Path(str(path) + '.gz').read_bytes()))


def adjacent(value):
    bits = struct.unpack('<I', struct.pack('<f', value))[0]
    return struct.unpack('<f', struct.pack('<I', bits + 1))[0]


class NanoBoundary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.serial = read('validation-checked/serial/tum-0.json')
        cls.parallel = read('validation-checked/first/tum-0.json')
        cls.replay = read('validation-checked/nano-first.json')
        cls.inspection = read('validation-checked/nano-inspection.json')
        cls.source = json.loads((ROOT / 'assets/camera-sequence-v1/sequence.json').read_text())
        cls.saved = json.loads((ROOT / 'assets/camera-sequence-v1/recorded/exports/first-0.json').read_text())

    def image(self, report):
        checker.check_image(report, self.saved, self.serial)

    def clock(self, report):
        checker.clock.audit_measured(report, self.source, self.inspection, checker.freshness, checker.tracking)

    def test_real_current_serial_parallel_and_clock_pass(self):
        self.image(self.parallel)
        self.clock(self.replay)
        checker.check_clock_source()

    def test_timed_promoted_f32_matches_typed_current_reference(self):
        timed = self.replay['events'][0]['measured']['analysis']
        self.assertNotEqual(timed['result'], self.serial['result'])
        self.image(timed)

    def test_one_ulp_confidence_change_is_rejected(self):
        bad = copy.deepcopy(self.parallel)
        detection = bad['base_prediction']['views'][0]['prediction']['detections'][0]
        detection['confidence'] = adjacent(detection['confidence'])
        with self.assertRaises(ValueError):self.image(bad)

    def test_one_ulp_coordinate_change_is_rejected(self):
        bad = copy.deepcopy(self.parallel)
        box = bad['result']['detections'][0]['bbox']
        box[0] = adjacent(box[0])
        with self.assertRaises(ValueError):self.image(bad)

    def test_threshold_accepts_same_f32_and_rejects_next_f32(self):
        self.assertTrue(checker.exact_f32(.3, .30000001192092896))
        self.assertFalse(checker.exact_f32(adjacent(.3), .3))
        bad = copy.deepcopy(self.parallel)
        bad['base_prediction']['score_threshold'] = adjacent(.3)
        with self.assertRaises(ValueError):self.image(bad)

    def test_box_bits_distinguish_signed_zero(self):
        self.assertNotEqual(checker.result_bits({'bbox': [0., 1., 2., 3.]}),
                            checker.result_bits({'bbox': [-0., 1., 2., 3.]}))

    def test_work_count_types_remain_exact(self):
        self.assertNotEqual(checker.result_bits({'comparison_work': 1}),
                            checker.result_bits({'comparison_work': True}))
        bad = copy.deepcopy(self.parallel)
        bad['result']['comparison_work'] += 1
        with self.assertRaises(ValueError):self.image(bad)

    def test_wrong_model_and_source_receipts_rejected(self):
        for field, value in [('model_sha256', '0' * 64), ('compiled_sources', {})]:
            bad = copy.deepcopy(self.parallel); bad[field] = value
            with self.assertRaises(ValueError):self.image(bad)

    def test_wrong_tensor_geometry_and_view_order_rejected(self):
        bad = copy.deepcopy(self.parallel)
        bad['base_prediction']['views'][0]['prediction']['input_shape'] = [1, 3, 640, 640]
        with self.assertRaises(ValueError):self.image(bad)
        bad = copy.deepcopy(self.parallel); bad['base_prediction']['views'].reverse()
        with self.assertRaises(ValueError):self.image(bad)

    def test_control_permission_cannot_be_enabled(self):
        bad = copy.deepcopy(self.parallel); bad['control_permission'] = True
        with self.assertRaises(ValueError):self.image(bad)

    def test_clock_f64_ulp_and_policy_changes_rejected(self):
        bad = copy.deepcopy(self.replay)
        timing = bad['events'][0]['measured']['timing']
        timing['read_analysis_and_probe_wall_s'] = math.nextafter(timing['read_analysis_and_probe_wall_s'], math.inf)
        with self.assertRaises(ValueError):self.clock(bad)
        bad = copy.deepcopy(self.replay); bad['policy']['max_age_s'] = .21
        with self.assertRaises(ValueError):self.clock(bad)

    def test_acquisition_and_ready_flag_cannot_be_renewed(self):
        bad = copy.deepcopy(self.replay)
        bad['events'][0]['measured']['guard_events'][0]['event']['input']['acquired_s'] = .001
        with self.assertRaises(ValueError):self.clock(bad)
        bad = copy.deepcopy(self.replay)
        output = bad['events'][0]['measured']['guard_events'][1]['output']
        output['ready'] = not output['ready']
        with self.assertRaises(ValueError):self.clock(bad)


if __name__ == '__main__':unittest.main()
