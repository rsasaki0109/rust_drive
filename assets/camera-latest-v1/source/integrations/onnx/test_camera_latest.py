"""Reject future selection, hidden drops, renewed acquisitions and altered guards."""
import copy
import gzip
import importlib.util
import json
import math
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKET = ROOT / 'assets/camera-latest-v1'
spec = importlib.util.spec_from_file_location('latest_tests', HERE / 'camera_latest_check.py')
checker = importlib.util.module_from_spec(spec); spec.loader.exec_module(checker)


def read(name):
    path = PACKET / name
    if path.exists():return json.loads(path.read_text())
    return json.loads(gzip.decompress(Path(str(path)+'.gz').read_bytes()))


class LatestBoundary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report = read('validation/latest-first.json')
        cls.inspection = read('validation/latest-inspection.json')
        cls.source = json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text())
        cls.references = [read(f'validation/references/{i}.json') for i in range(8)]

    def audit(self, report):
        return checker.audit(report,self.source,self.inspection,self.references)

    def test_actual_selection_measurements_and_guards_pass(self):
        self.audit(self.report)
        checker.check_shared_controller()

    def test_future_image_cannot_be_selected(self):
        bad = copy.deepcopy(self.report)
        row = next(r for r in bad['events'] if r['superseded_sources'])
        row['selection_checked_s'] = row['source']['release_offset_s']-0.00001
        with self.assertRaises(ValueError):self.audit(bad)

    def test_superseded_source_cannot_be_hidden(self):
        bad = copy.deepcopy(self.report)
        row = next(r for r in bad['events'] if r['superseded_sources'])
        row['superseded_sources'].pop()
        with self.assertRaises(ValueError):self.audit(bad)

    def test_source_acquisition_cannot_be_replaced_with_dispatch_time(self):
        bad = copy.deepcopy(self.report); row = bad['events'][0]
        row['measured']['guard_events'][0]['event']['input']['acquired_s'] = row['selection_checked_s']
        with self.assertRaises(ValueError):self.audit(bad)

    def test_one_ulp_host_clock_change_is_rejected(self):
        bad = copy.deepcopy(self.report); timing = bad['events'][0]['measured']['timing']
        timing['sample_submitted_s'] = math.nextafter(timing['sample_submitted_s'],math.inf)
        with self.assertRaises(ValueError):self.audit(bad)

    def test_one_ulp_policy_relaxation_is_rejected(self):
        bad = copy.deepcopy(self.report)
        bad['policy']['max_age_s'] = math.nextafter(.2,math.inf)
        with self.assertRaises(ValueError):self.audit(bad)

    def test_dropped_images_never_count_as_available_or_inferred(self):
        for field in ('available_at_dispatched_checks','inferred_images'):
            bad = copy.deepcopy(self.report); bad['summary'][field] += bad['summary']['superseded_images']
            with self.assertRaises(ValueError):self.audit(bad)
        bad = copy.deepcopy(self.report); bad['summary']['all_source_results_available'] = True
        with self.assertRaises(ValueError):self.audit(bad)

    def test_false_ready_result_is_rejected_by_independent_guard(self):
        bad = copy.deepcopy(self.report)
        bad['events'][-1]['measured']['guard_events'][-1]['output']['ready'] = True
        bad['summary']['available_at_dispatched_checks'] += 1
        with self.assertRaises(ValueError):self.audit(bad)

    def test_measurement_change_is_rejected_against_current_serial(self):
        bad = copy.deepcopy(self.report)
        value = bad['events'][0]['measured']['analysis']['result']['detections'][0]
        value['confidence'] += .001
        with self.assertRaises(ValueError):self.audit(bad)

    def test_producer_and_control_permission_are_bound(self):
        for field,value in [('algorithm','yolox_nano_measured_replay_v1'),('compiled_sources',{}),('control_permission',True)]:
            bad = copy.deepcopy(self.report); bad[field] = value
            with self.assertRaises(ValueError):self.audit(bad)

    def test_missing_and_failed_events_are_preserved_without_a_model(self):
        report = read('controls/no-images-result.json')
        source = read('controls/no-images.json')
        inspection = read('controls/no-images-inspection.json')
        checker.audit(report,source,inspection)
        bad = copy.deepcopy(report); bad['events'].pop()
        with self.assertRaises(ValueError):checker.audit(bad,source,inspection)


if __name__ == '__main__':unittest.main()
