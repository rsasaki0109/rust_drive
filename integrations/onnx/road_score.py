#!/usr/bin/env python3
"""Independent class-aware, one-to-one fixed-IoU scoring; not COCO AP."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from road_fetch import checked, protocol
from score import iou


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def score(data, detections):
    p = protocol()
    if len(p['images']) != 6 or len({i['id'] for i in p['images']}) != 6:
        raise ValueError('duplicate or missing fixture identity')
    if sum(i['partition'] == 'calibration' for i in p['images']) != 2 or sum(i['partition'] == 'heldout' for i in p['images']) != 4:
        raise ValueError('partition changed')
    labels = json.loads(checked((data / 'instances_train2017.json').read_bytes(), p['annotation']))
    categories = {int(k): v for k, v in p['class_index_to_coco_category'].items()}
    names = {c['id']: c['name'] for c in labels['categories']}
    images = []
    for image in p['images']:
        checked((data / image['file_name']).read_bytes(), image)
        ref_image = next(i for i in labels['images'] if i['id'] == image['id'])
        if any(ref_image[k] != image[k] for k in ['width', 'height', 'license', 'file_name', 'flickr_url', 'coco_url']):
            raise ValueError('independent image metadata mismatch')
        path = detections / (image['file_name'] + '.json')
        prediction = json.loads(path.read_text())
        if prediction['model_sha256'] != p['model_sha256'] or prediction['source_image_sha256'] != image['sha256'] or (prediction['width'], prediction['height']) != (image['width'], image['height']):
            raise ValueError('prediction model/image identity mismatch')
        if prediction['score_threshold'] != p['score_threshold'] or prediction['nms_iou_threshold'] != p['nms_iou_threshold']:
            raise ValueError('prediction thresholds changed')
        all_predictions = prediction['detections']
        if len(all_predictions) > 300:
            raise ValueError('prediction bound exceeded')
        for d in all_predictions:
            b = d['bbox']
            if type(d['class_index']) is not int or not 0 <= d['class_index'] < 80 or len(b) != 4 or not all(math.isfinite(v) for v in b) or not (0 <= b[0] < b[2] <= image['width'] and 0 <= b[1] < b[3] <= image['height']) or not math.isfinite(d['confidence']) or not p['score_threshold'] <= d['confidence'] <= 1:
                raise ValueError('invalid prediction geometry/class/confidence')
        refs = []
        for a in labels['annotations']:
            if a['image_id'] == image['id'] and a['category_id'] in categories.values():
                if a['iscrowd'] != 0:
                    raise ValueError('crowd reference needs an explicit evaluation policy')
                x, y, w, h = a['bbox']
                if not all(math.isfinite(v) for v in [x, y, w, h]) or w <= 0 or h <= 0:
                    raise ValueError('invalid independent reference')
                refs.append({'annotation_id': a['id'], 'category_id': a['category_id'], 'bbox': [x, y, x+w, y+h]})
        predictions = [(n, d) for n, d in enumerate(all_predictions) if d['class_index'] in categories]
        matches, false_positives, used = [], [], set()
        for n, d in sorted(predictions, key=lambda nd: (-nd[1]['confidence'], nd[0])):
            candidates = [(iou(d['bbox'], r['bbox']), k) for k, r in enumerate(refs) if k not in used and r['category_id'] == categories[d['class_index']]]
            best, k = max(candidates, default=(0, -1), key=lambda pair: (pair[0], -pair[1]))
            if best >= p['match_iou_threshold']:
                used.add(k)
                matches.append({'prediction_index': n, 'annotation_id': refs[k]['annotation_id'], 'category_id': refs[k]['category_id'], 'iou': best})
            else:
                false_positives.append({'prediction_index': n, 'category_id': categories[d['class_index']], 'bbox': d['bbox'], 'confidence': d['confidence'], 'best_unmatched_same_class_iou': best})
        false_negatives = [r for k, r in enumerate(refs) if k not in used]
        images.append({'image_id': image['id'], 'partition': image['partition'], 'reference_count': len(refs), 'all_class_prediction_count': len(all_predictions), 'ignored_other_class_prediction_count': len(all_predictions)-len(predictions), 'matches': matches, 'false_positives': false_positives, 'false_negatives': false_negatives, 'prediction_sha256': digest(path)})
    def aggregate(rows, category=None):
        matches = [m for row in rows for m in row['matches'] if category is None or m['category_id'] == category]
        fp = sum(category is None or m['category_id'] == category for row in rows for m in row['false_positives'])
        fn = sum(category is None or m['category_id'] == category for row in rows for m in row['false_negatives'])
        tp = len(matches)
        return {'true_positives': tp, 'false_positives': fp, 'false_negatives': fn, 'precision': tp/(tp+fp) if tp+fp else None, 'recall': tp/(tp+fn) if tp+fn else None, 'mean_matched_iou': sum(m['iou'] for m in matches)/tp if tp else None}
    partitions = {}
    for partition in ['calibration', 'heldout']:
        rows = [r for r in images if r['partition'] == partition]
        partitions[partition] = {'image_count': len(rows), 'overall': aggregate(rows), 'per_class': {names[c]: aggregate(rows, c) for c in categories.values()}}
    return {'schema_version': 1, 'evaluation': 'six independently annotated urban COCO diagnostic images, fixed single-IoU threshold; not COCO AP', 'protocol_sha256': digest(Path(__file__).with_name('road-protocol.json')), 'annotation_sha256': p['annotation']['sha256'], 'score_threshold': p['score_threshold'], 'nms_iou_threshold': p['nms_iou_threshold'], 'match_iou_threshold': p['match_iou_threshold'], 'partitions': partitions, 'overall': aggregate(images), 'images': images, 'limitations': p['limitations']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--detections', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = score(args.data, args.detections)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['partitions'], indent=2))
