#!/usr/bin/env python3
"""Independent original-BDD-box fixed-IoU diagnostics, not BDD mAP."""
import argparse
import json
import math
from pathlib import Path
from bdd_fetch import checked_path, digest, protocol
from score import iou


def match_boxes(refs, predictions, threshold):
    used, matches, false_positives = set(), [], []
    for index, prediction in sorted(enumerate(predictions), key=lambda x: (-x[1]['confidence'], x[0])):
        candidates = [(iou(prediction['bbox'], ref['bbox']), k)
                      for k, ref in enumerate(refs)
                      if k not in used and ref['class_index'] == prediction['class_index']]
        overlap, k = max(candidates, default=(0., -1), key=lambda x: (x[0], -x[1]))
        if overlap >= threshold:
            used.add(k)
            matches.append({'prediction_index': index, 'reference_id': refs[k]['id'],
                            'reference_category': refs[k]['category'],
                            'class_index': prediction['class_index'], 'iou': overlap})
        else:
            false_positives.append({'prediction_index': index, **prediction,
                                   'best_unmatched_same_class_iou': overlap})
    return matches, false_positives, [ref for k, ref in enumerate(refs) if k not in used]


def valid_box(box, width, height):
    return (isinstance(box, list) and len(box) == 4
            and all(type(x) in (int, float) and math.isfinite(x) for x in box)
            and 0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height)


def aggregate(images, class_index=None):
    matches = [m for image in images for m in image['matches']
               if class_index is None or m['class_index'] == class_index]
    fp = sum(class_index is None or d['class_index'] == class_index
             for image in images for d in image['false_positives'])
    fn = sum(class_index is None or d['class_index'] == class_index
             for image in images for d in image['false_negatives'])
    tp = len(matches)
    return {'true_positives': tp, 'false_positives': fp, 'false_negatives': fn,
            'precision': tp/(tp+fp) if tp+fp else None,
            'recall': tp/(tp+fn) if tp+fn else None,
            'mean_matched_iou': sum(m['iou'] for m in matches)/tp if tp else None}


def score(data, predictions):
    p = protocol()
    checked_path(data/p['license']['file'], p['license'])
    rows = []
    for image in p['images']:
        checked_path(data/image['image']['file'], image['image'])
        labels = json.loads(checked_path(data/image['annotation']['file'], image['annotation']))
        if labels['name'] != image['id'] or len(labels['frames']) != 1 or labels['frames'][0]['timestamp'] != 10000 or labels['attributes'] != image['attributes']:
            raise ValueError('original annotation/image/frame metadata mismatch')
        refs, excluded, seen = [], [], set()
        for label in labels['frames'][0]['objects']:
            if label['id'] in seen:
                raise ValueError('duplicate original annotation identity')
            seen.add(label['id'])
            if 'box2d' not in label:
                continue
            b = label['box2d']
            box = [b[k] for k in ['x1', 'y1', 'x2', 'y2']]
            if not valid_box(box, image['width'], image['height']):
                raise ValueError('invalid original reference geometry')
            ref = {'id': label['id'], 'category': label['category'], 'bbox': box,
                   'attributes': label['attributes']}
            if label['category'] in p['class_mapping']:
                refs.append({**ref, 'class_index': p['class_mapping'][label['category']]})
            elif label['category'] in p['unscored_category_reason']:
                excluded.append(ref)
            else:
                raise ValueError('original reference category has no frozen evaluation policy')
        path = predictions/(image['id']+'.json')
        body = path.read_bytes()
        predicted = json.loads(body)
        if (predicted['model_sha256'] != p['model_sha256'] or predicted['source_image_sha256'] != image['image']['sha256'] or (predicted['width'], predicted['height']) != (image['width'], image['height']) or predicted['score_threshold'] != p['confidence_threshold'] or predicted['nms_iou_threshold'] != p['nms_iou_threshold']):
            raise ValueError('wrong image/model/dimensions/inference thresholds')
        all_detections = predicted['detections']
        if len(all_detections) > 300:
            raise ValueError('prediction budget exceeded')
        for d in all_detections:
            if type(d['class_index']) is not int or not 0 <= d['class_index'] < 80 or not valid_box(d['bbox'], image['width'], image['height']) or type(d['confidence']) not in (int, float) or not math.isfinite(d['confidence']) or not p['confidence_threshold'] <= d['confidence'] <= 1:
                raise ValueError('invalid prediction class/geometry/confidence')
        selected = [d for d in all_detections if d['class_index'] in p['class_mapping'].values()]
        matches, fp, fn = match_boxes(refs, selected, p['match_iou_threshold'])
        rows.append({'image_id': image['id'], 'partition': 'heldout',
                     'original_split': image['original_split'], 'attributes': image['attributes'],
                     'reference_count': len(refs), 'excluded_original_sign_boxes': excluded,
                     'all_class_prediction_count': len(all_detections),
                     'ignored_other_class_predictions': [d for d in all_detections if d['class_index'] not in p['class_mapping'].values()],
                     'matches': matches, 'false_positives': fp, 'false_negatives': fn,
                     'prediction_sha256': digest(body),
                     'original_image_sha256': image['image']['sha256'],
                     'original_annotation_sha256': image['annotation']['sha256']})
    names = {0: 'person including original rider', 1: 'bicycle', 2: 'car', 3: 'motorcycle',
             5: 'bus', 6: 'train', 7: 'truck', 9: 'traffic light'}
    return {'schema_version': 1, 'evaluation': 'Eight original BDD dashcam image diagnostics; all images and all scoped misses retained; not BDD mAP',
            'protocol_sha256': digest(Path(__file__).with_name('bdd_protocol.json').read_bytes()),
            'confidence_threshold': p['confidence_threshold'], 'nms_iou_threshold': p['nms_iou_threshold'],
            'match_iou_threshold': p['match_iou_threshold'], 'image_count': len(rows),
            'reference_count': sum(r['reference_count'] for r in rows),
            'overall': aggregate(rows), 'per_class': {name: aggregate(rows, k) for k, name in names.items()},
            'images': rows, 'limitations': p['limits'],
            'data_license': 'Regents/BAIR BDD data research/educational/not-for-profit terms; see accompanying LICENSE-data.txt. Evaluator code remains Apache-2.0.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--predictions', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = score(args.data, args.predictions)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['overall']))
