#!/usr/bin/env python3
"""Hand-computed metric witnesses and actual CLI controls on authored data only."""
import argparse
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT/'scripts'/filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


evaluator = load('depth_evaluator', 'evaluate-kitti-depth.py')


def png(array, dtype=np.uint16):
    stream = io.BytesIO()
    Image.fromarray(np.asarray(array, dtype=dtype)).save(stream, format='PNG')
    return stream.getvalue()


def report():
    return dict(schema_version=1, kind='kitti_sparse_axial_depth', camera_index=2,
                depth=dict(rectified_size=[2, 2], pixels=[
                    dict(pixel=[0, 0], depth_m=4., point_index=0, support_count=1),
                    dict(pixel=[1, 0], depth_m=2., point_index=1, support_count=1)],
                    summary=dict(points=2, inframe_points=2, occupied_pixels=2,
                                 occluded_points=0, outside=0, behind=0)))


class Metrics(unittest.TestCase):
    def test_hand_metric_units_and_missing_prediction_denominator(self):
        result = evaluator.summarize(evaluator.score(report(), png([[512, 1024], [0, 2048]]), 2))
        self.assertEqual(result['rmse_m'], 2.)
        self.assertEqual(result['mae_m'], 2.)
        self.assertEqual(result['irmse_1_per_km'], 250.)
        self.assertEqual(result['imae_1_per_km'], 250.)
        self.assertEqual(result['reference_coverage'], 2/3)
        self.assertEqual(result['missing_prediction_reference_pixels'], 1)
        self.assertEqual(result['prediction_density'], .5)
        self.assertFalse(result['full_reference_coverage'])

    def test_perfect_conditional_error_does_not_imply_full_coverage(self):
        result = evaluator.summarize(evaluator.score(report(), png([[1024, 512], [0, 2048]]), 2))
        self.assertEqual(result['rmse_m'], 0.)
        self.assertEqual(result['reference_coverage'], 2/3)
        self.assertFalse(result['full_reference_coverage'])

    def test_reference_zero_is_missing_and_no_overlap_is_not_zero_error(self):
        for reference in ([[0, 0], [0, 2048]], [[0, 0], [0, 0]]):
            result = evaluator.summarize(evaluator.score(report(), png(reference), 2))
            self.assertEqual(result['paired_pixels'], 0)
            for key in ('rmse_m', 'mae_m', 'irmse_1_per_km', 'imae_1_per_km'):
                self.assertIsNone(result[key])
            self.assertFalse(result['full_reference_coverage'])

    def test_no_crop_resize_color_or_eight_bit_conversion(self):
        for raw in (png([[1, 2], [3, 4]], np.uint8), png(np.zeros((2, 2, 3), np.uint8), np.uint8),
                    png([[512]]), b'not a PNG'):
            with self.assertRaises(ValueError):
                evaluator.score(report(), raw, 2)

    def test_prediction_corruptions_are_rejected(self):
        mutations = [
            lambda p: p.update(camera_index=3),
            lambda p: p['depth']['pixels'][0].update(depth_m=float('nan')),
            lambda p: p['depth']['pixels'][0].update(depth_m=float('inf')),
            lambda p: p['depth']['pixels'][0].update(depth_m=0),
            lambda p: p['depth']['pixels'][0].update(pixel=[2, 0]),
            lambda p: p['depth']['pixels'][0].update(pixel=[True, 0]),
            lambda p: p['depth']['pixels'][1].update(pixel=[0, 0]),
            lambda p: p['depth']['pixels'][1].update(point_index=0),
            lambda p: p['depth']['pixels'][0].update(support_count=0),
            lambda p: p['depth']['summary'].update(occupied_pixels=1),
            lambda p: p['depth']['summary'].update(points=3),
        ]
        for mutate in mutations:
            invalid = copy.deepcopy(report())
            mutate(invalid)
            with self.assertRaises(ValueError):
                evaluator.score(invalid, png([[512, 1024], [0, 0]]), 2)

    def test_pooled_errors_use_pixels_not_average_frame_rmse(self):
        totals = dict(canvas_pixels=10, reference_pixels=6, predicted_pixels=4, paired_pixels=4,
                      squared_error_sum_m2=12., absolute_error_sum_m=6.,
                      squared_inverse_error_sum=400., absolute_inverse_error_sum=40.)
        result = evaluator.summarize(totals)
        self.assertAlmostEqual(result['rmse_m'], math.sqrt(3.))
        self.assertEqual(result['mae_m'], 1.5)
        self.assertEqual(result['irmse_1_per_km'], 10.)
        self.assertEqual(result['imae_1_per_km'], 10.)
        self.assertEqual(result['reference_coverage'], 4/6)

    def test_fractional_metre_png_encoding_is_preserved(self):
        result = evaluator.summarize(evaluator.score(report(), png([[1, 0], [0, 0]]), 2))
        self.assertEqual(result['paired_pixels'], 1)
        self.assertAlmostEqual(result['mae_m'], 4.-1./256.)

    def test_duplicate_json_keys_and_nonfinite_constants_are_invalid(self):
        for raw in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}'):
            with self.assertRaises(ValueError):
                evaluator.strict_json(raw)


class ActualWorkflow(unittest.TestCase):
    """Execute the real frozen projection/depth binaries on an authored fixture."""

    def setUp(self):
        stdout = mock.patch('sys.stdout', new_callable=io.StringIO)
        stdout.start()
        self.addCleanup(stdout.stop)
        self.temporary = tempfile.TemporaryDirectory(prefix='rustdrive-depth-eval-')
        self.root = Path(self.temporary.name)
        oracle = load('original_projection_oracle', 'check-kitti-calibration.py')
        oracle.prepare_fixture(self.root/'raw')
        self.raw = self.root/'raw'
        (self.raw/'0000000005.bin').write_bytes((self.raw/'points.bin').read_bytes())
        independent = oracle.independently_project(self.raw, 2)
        depth_checker = load('original_depth_oracle', 'check-kitti-depth.py')
        expected = depth_checker.expected_depth(independent['rectified_size'], independent['rows'])
        width, height = independent['rectified_size']
        truth = np.zeros((height, width), dtype=np.uint16)
        for cell in expected['pixels']:
            x, y = cell['pixel']
            truth[y, x] = int(cell['depth_m']*256)+256
        self.assertEqual(int(truth[1, 1]), 0)
        truth[1, 1] = 2048  # One known reference pixel has no prediction.
        (self.raw/'0000000005.png').write_bytes(png(truth))
        self.expected_pairs = len(expected['pixels'])
        self.manifest = self.root/'manifest.json'
        self.document = dict(schema_version=1, reference_kind='kitti_annotated_depth',
            reference_encoding='uint16_png_div_256', split='val', recording_id='2011_09_26_drive_0001_sync',
            camera_index=2, provenance='AUTHORED ANALYTIC CONTROL; NOT REAL KITTI DATA',
            calibration=dict(camera='calib_cam_to_cam.txt', velo='calib_velo_to_cam.txt', imu='calib_imu_to_velo.txt'),
            frames=[dict(frame_id='0000000005', points='0000000005.bin', reference_depth='0000000005.png')])
        self.manifest.write_text(json.dumps(self.document))
        self.projection = ROOT/'integrations/rgbd/target/release/rustdriving-kitti-project'
        self.depth = ROOT/'integrations/kitti-depth/target/release/rustdriving-kitti-depth'
        self.assertTrue(self.projection.is_file() and self.depth.is_file(), 'build the projection/depth binaries first')

    def tearDown(self):
        self.temporary.cleanup()

    def args(self, output, prepare=False):
        return argparse.Namespace(output=self.root/output, root=self.raw, manifest=self.manifest,
            projection_binary=self.projection, depth_binary=self.depth,
            prepare_freeze=prepare, freeze=None if prepare else self.root/'prepared/freeze.json')

    def prepare(self):
        with mock.patch.object(evaluator, 'decode_reference', side_effect=AssertionError('early reference decode')):
            self.assertEqual(evaluator.execute(self.args('prepared', True)), 0)

    def test_actual_binaries_freeze_score_repeat_and_keep_missing_reference_predictions(self):
        self.prepare()
        for name in ('first', 'repeat'):
            self.assertEqual(evaluator.execute(self.args(name)), 0)
        first = json.loads((self.root/'first/results.json').read_text())
        repeat = json.loads((self.root/'repeat/results.json').read_text())
        self.assertEqual(first, repeat)
        self.assertEqual(first['comparable_frames'], 1)
        self.assertEqual(first['metrics_over_scored_frames']['paired_pixels'], self.expected_pairs)
        self.assertAlmostEqual(first['metrics_over_scored_frames']['reference_coverage'],
                               self.expected_pairs/(self.expected_pairs+1))
        self.assertIsNone(first['quality_gate_passed'])

    def test_changed_input_invalidates_freeze_before_any_process(self):
        self.prepare()
        with (self.raw/'0000000005.bin').open('ab') as stream:
            stream.write(b'corrupt')
        with mock.patch.object(evaluator.subprocess, 'run') as run:
            with self.assertRaisesRegex(ValueError, 'freeze changed'):
                evaluator.execute(self.args('changed'))
            run.assert_not_called()

    def test_failed_reference_frame_stays_in_denominator(self):
        (self.raw/'0000000005.png').write_bytes(png([[1]]))
        self.prepare()
        self.assertEqual(evaluator.execute(self.args('failed')), 1)
        result = json.loads((self.root/'failed/results.json').read_text())
        self.assertEqual(result['selected_frames'], 1)
        self.assertEqual(result['scored_frames'], 0)
        self.assertFalse(result['complete_evaluation'])
        self.assertIn('failure', result['frames'][0])
        self.assertIsNone(result['metrics_over_scored_frames']['rmse_m'])

    def test_zero_overlap_frame_is_retained_and_unscorable(self):
        (self.raw/'0000000005.png').write_bytes(png(np.zeros((480, 640), dtype=np.uint16)))
        self.prepare()
        self.assertEqual(evaluator.execute(self.args('unscorable')), 1)
        result = json.loads((self.root/'unscorable/results.json').read_text())
        self.assertEqual((result['selected_frames'], result['scored_frames'], result['comparable_frames']), (1, 1, 0))
        self.assertIsNone(result['metrics_over_scored_frames']['rmse_m'])

    def test_duplicate_frame_and_source_path_escape_are_rejected(self):
        for mutate in (lambda p: p['frames'].append(copy.deepcopy(p['frames'][0])),
                       lambda p: p['frames'][0].update(points='../0000000005.bin'),
                       lambda p: p['frames'][0].update(reference_depth='0000000006.png')):
            document = copy.deepcopy(self.document)
            mutate(document)
            with self.assertRaises(ValueError):
                evaluator.inventory(json.dumps(document).encode(), self.raw)


if __name__ == '__main__':
    unittest.main()
