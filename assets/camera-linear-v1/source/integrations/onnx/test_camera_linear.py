"""Model-free corruptions of receipts, graph parity and fixed quality gates."""
import copy
import json
import unittest

import camera_linear_check as check


class LinearChecks(unittest.TestCase):
    def setUp(self):
        self.row = dict(id='fixture', image_sha256='a' * 64)
        rectangles = check.prior.nano.tiles.rectangles(1280, 720)
        self.pixel = dict(views=[dict(rectangle=r, rgb_sha256='b' * 64,
                         tensor_f32le_sha256='c' * 64, resized=[416, 234] if i == 0 else [416, 416])
                         for i, r in enumerate(rectangles)])
        views = []
        for oracle in self.pixel['views']:
            rectangle = oracle['rectangle']
            p = dict(schema_version=1, algorithm='pinned_yolox_nano_controlled_preprocessing_v1',
                     model_profile='yolox-nano-controlled-preprocessing-v1', preprocessing='linear',
                     original_graph_reference=False, compiled_sources=check.expected(),
                     model_profile_sha256=check.digest(check.HERE / 'profile-linear.json'),
                     model_sha256=check.prior.MODEL_SHA, input_shape=[1, 3, 416, 416],
                     output_shape=[1, 3549, 85], width=rectangle['width'], height=rectangle['height'],
                     source_image_sha256=None, score_threshold=.3, nms_iou_threshold=.45,
                     model_load_seconds=.2, inference_seconds=.1,
                     input_tensor_f32le_sha256=oracle['tensor_f32le_sha256'],
                     output_tensor_f32le_sha256='d' * 64, resized=oracle['resized'], detections=[])
            views.append(dict(rectangle=rectangle, rgb_sha256=oracle['rgb_sha256'], prediction=p))
        fixture = dict(width=1280, height=720,
                       views=[dict(rectangle=r, detections=[]) for r in rectangles])
        base = dict(algorithm='nano_controlled_preprocessing_fixed_views_v1', compiled_sources=check.expected(),
                    source_image_sha256=self.row['image_sha256'], model_sha256=check.prior.MODEL_SHA,
                    width=1280, height=720, score_threshold=.3, nms_iou_threshold=.45, views=views,
                    **check.prior.nano.tiles.merge(fixture))
        self.report = dict(schema_version=1, algorithm='nano_controlled_preprocessing_analysis_v1',
                           preprocessing='linear', original_graph_reference=False, control_permission=False,
                           source_image_sha256=self.row['image_sha256'], compiled_sources=check.expected(),
                           profile_sha256=check.digest(check.HERE / 'profile-linear.json'), base_prediction=base,
                           execution=dict(model_read=True, images_read=1, model_forwards=4, inference_run=True,
                                          view_workers=4, shared_model_plans=1, analysis_wall_s=.2),
                           result=check.prior.nano.consistency.oracle(fixture, check.prior.nano.tiles))

    def audit(self):
        check.audit_image(self.report, self.row, self.pixel, 'linear')

    def rejected(self, mutation):
        mutation(self.report)
        with self.assertRaises((AssertionError, ValueError, KeyError)):
            self.audit()

    def test_complete_empty_typed_report_is_valid(self):
        self.audit()
        check.compare_pair(self.report, copy.deepcopy(self.report))

    def test_source_closure_is_required(self):
        self.rejected(lambda r: r['compiled_sources'].pop(next(iter(r['compiled_sources']))))

    def test_profile_is_required(self):
        self.rejected(lambda r: r.__setitem__('profile_sha256', '0' * 64))

    def test_mode_is_bound_to_report(self):
        self.rejected(lambda r: r['base_prediction']['views'][0]['prediction'].__setitem__('preprocessing', 'triangle'))

    def test_every_original_view_is_required(self):
        self.rejected(lambda r: r['base_prediction']['views'].pop())

    def test_real_input_tensor_must_match_cv2(self):
        self.rejected(lambda r: r['base_prediction']['views'][0]['prediction'].__setitem__('input_tensor_f32le_sha256', 'f' * 64))

    def test_current_reference_raw_output_must_match(self):
        changed = copy.deepcopy(self.report)
        changed['base_prediction']['views'][0]['prediction']['output_tensor_f32le_sha256'] = 'e' * 64
        with self.assertRaises((AssertionError, ValueError)):
            check.compare_pair(self.report, changed)

    def test_forged_inference_count_is_rejected(self):
        self.rejected(lambda r: r['execution'].__setitem__('model_forwards', 3))

    def test_nonfinite_timing_is_rejected(self):
        self.rejected(lambda r: r['base_prediction']['views'][0]['prediction'].__setitem__('inference_seconds', float('nan')))

    def test_wrong_reference_graph_is_rejected(self):
        self.rejected(lambda r: r.__setitem__('original_graph_reference', True))

    def test_forged_global_merge_is_rejected(self):
        self.rejected(lambda r: r['base_prediction']['merge'].__setitem__('comparisons', 1))

    def test_absolute_gates_keep_false_positives_and_all_misses(self):
        objects = [dict(id=i, category='car', attributes=dict(occluded=True, truncated=True),
                        box2d=dict(x1=10. * i, y1=0., x2=10. * i + 8., y2=8.)) for i in range(1, 4)]
        body = json.dumps(dict(name='fixture', frames=[dict(timestamp=10000, objects=objects)], attributes={})).encode()
        self.row.update(partition='fresh_sample', original_split='val', annotation_sha256=check.hashlib.sha256(body).hexdigest())
        self.report['result']['detections'] = [dict(class_index=2, confidence=.9, bbox=[10., 0., 18., 8.]),
                                                dict(class_index=2, confidence=.8, bbox=[50., 0., 58., 8.])]
        result = check.score([self.row], {'fixture': self.report}, lambda _: body)
        self.assertEqual([result['overall'][k] for k in ('true_positives', 'false_positives', 'false_negatives')], [1, 1, 2])
        self.assertFalse(result['quality_gates']['passed'])
        self.assertEqual(len(result['images'][0]['false_negatives']), 2)


if __name__ == '__main__':
    unittest.main()
