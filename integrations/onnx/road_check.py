#!/usr/bin/env python3
"""Execute frozen CPU inference twice and independently report every failure."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
from road_fetch import fetch, protocol, PROTOCOL
from road_score import score, iou
from fetch import checked

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCES = ['Cargo.toml', 'Cargo.lock', 'src/lib.rs', 'src/main.rs',
           'fetch.py', 'score.py', 'road_fetch.py', 'road_score.py',
           'road_check.py', 'road-protocol.json']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fingerprints():
    return {str((HERE / name).relative_to(ROOT)): digest(HERE / name) for name in SOURCES}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--data', type=Path, default=ROOT/'artifacts/road-camera')
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/road-camera/check')
    parser.add_argument('--summary', type=Path)
    parser.add_argument('--baseline', type=Path, help='Compare recorded diagnostic counts/identities; not an accuracy gate.')
    args = parser.parse_args()
    start_sources = fingerprints()
    binary = args.binary.resolve()
    binary_sha = digest(binary)
    checked('yolox_nano.onnx', args.model.read_bytes())
    p = protocol()
    fetch(args.data)
    args.output.mkdir(parents=True, exist_ok=True)
    for name in ['first', 'repeat']:
        output = args.output / name
        output.mkdir(exist_ok=True)
        for image in p['images']:
            subprocess.run([str(binary), '--model', str(args.model), '--image', str(args.data/image['file_name']), '--output', str(output/(image['file_name']+'.json'))], check=True)
    first = args.output/'first'
    for image in p['images']:
        filename = image['file_name']+'.json'
        a = json.loads((first/filename).read_text())
        b = json.loads((args.output/'repeat'/filename).read_text())
        if a['detections'] != b['detections']:
            raise ValueError('repeated actual CPU inference boxes differ')
    measured = score(args.data, first)
    if args.baseline:
        baseline = json.loads(args.baseline.read_text())
        if any(measured[k] != baseline[k] for k in ['protocol_sha256', 'annotation_sha256', 'score_threshold', 'nms_iou_threshold', 'match_iou_threshold']):
            raise ValueError('recorded diagnostic provenance changed')
        for row, old in zip(measured['images'], baseline['images'], strict=True):
            if any(row[k] != old[k] for k in ['image_id', 'partition', 'reference_count', 'all_class_prediction_count', 'ignored_other_class_prediction_count']):
                raise ValueError('recorded image/reference/prediction counts changed')
            for key in ['false_positives', 'false_negatives', 'matches']:
                if len(row[key]) != len(old[key]):
                    raise ValueError('recorded TP/FP/FN counts changed')
            for match, expected in zip(row['matches'], old['matches'], strict=True):
                if any(match[k] != expected[k] for k in ['prediction_index', 'annotation_id', 'category_id']) or abs(match['iou']-expected['iou']) > 1e-5:
                    raise ValueError('recorded match identity/IoU changed')
    # Pin identity, thresholds, finite geometry, class representation and label bytes.
    file = first/(p['images'][0]['file_name']+'.json')
    original = file.read_bytes()
    changes = [('model_sha256', '0'*64), ('source_image_sha256', '0'*64),
               ('width', 1), ('score_threshold', 0.01), ('nms_iou_threshold', 0.9)]
    for key, value in changes:
        altered = json.loads(original)
        altered[key] = value
        file.write_text(json.dumps(altered))
        try:
            score(args.data, first)
        except ValueError:
            pass
        else:
            raise ValueError(f'independent scorer accepted {key} mutation')
        finally:
            file.write_bytes(original)
    # A valid duplicated matching box must add an FP, never a second TP.
    matched_row = next(row for row in measured['images'] if row['matches'])
    matched_image = next(im for im in p['images'] if im['id'] == matched_row['image_id'])
    file = first/(matched_image['file_name']+'.json')
    original = file.read_bytes()
    altered = json.loads(original)
    altered['detections'].append(altered['detections'][matched_row['matches'][0]['prediction_index']])
    file.write_text(json.dumps(altered))
    try:
        duplicated = score(args.data, first)
        if duplicated['overall']['true_positives'] != measured['overall']['true_positives'] or duplicated['overall']['false_positives'] != measured['overall']['false_positives'] + 1:
            raise ValueError('one-to-one scorer accepted duplicate TP')
    finally:
        file.write_bytes(original)
    altered = json.loads(original)
    altered['detections'][0]['bbox'][2] = float('nan')
    file.write_text(json.dumps(altered))
    try:
        score(args.data, first)
    except ValueError:
        pass
    else:
        raise ValueError('independent scorer accepted nonfinite geometry')
    finally:
        file.write_bytes(original)
    annotation = args.data/'instances_train2017.json'
    original = annotation.read_bytes()
    annotation.write_bytes(original.replace(b'"bbox"', b'"boox"', 1))
    try:
        score(args.data, first)
    except ValueError:
        pass
    else:
        raise ValueError('independent scorer accepted changed label bytes')
    finally:
        annotation.write_bytes(original)
    # Independent hand-computable overlap checks, including exact match boundary.
    if iou([0, 0, 3, 1], [1, 0, 4, 1]) != 0.5 or iou([0, 0, 1, 1], [1, 0, 2, 1]) != 0 or iou([0, 0, 1, 1], [0, 0, 1, 1]) != 1:
        raise ValueError('independent IoU arithmetic changed')
    if fingerprints() != start_sources or digest(binary) != binary_sha:
        raise ValueError('source or binary changed during proof')
    reports = [json.loads((first/(i['file_name']+'.json')).read_text()) for i in p['images']]
    measured.update({'recorded_utc': datetime.now(timezone.utc).isoformat(), 'host': {'os': platform.system(), 'architecture': platform.machine(), 'gpu_used': False}, 'actual_inference_runs': 12, 'repeat_detection_identity': True, 'independent_mutations_rejected': 7, 'duplicate_prediction_adds_fp_only': True, 'iou_hand_calculation_checks': 3, 'source_sha256': start_sources, 'binary_sha256': binary_sha, 'model_sha256': p['model_sha256'], 'per_image_inference_seconds': [r['inference_seconds'] for r in reports], 'valid_execution_and_independent_scoring': True, 'recorded_baseline_compared': bool(args.baseline), 'automotive_accuracy_acceptance': False})
    target = args.summary or args.output/'results.json'
    target.write_text(json.dumps(measured, indent=2)+'\n')
    print(json.dumps(measured['partitions'], indent=2))
    print(f'Twelve actual CPU inference runs and independent scoring completed; {target}')


if __name__ == '__main__':
    main()
