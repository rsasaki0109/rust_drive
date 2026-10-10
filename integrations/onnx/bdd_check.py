#!/usr/bin/env python3
"""Frozen actual CPU BDD diagnostics; integrity success is not an accuracy pass."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
from bdd_fetch import digest, fetch, protocol
from bdd_score import score, match_boxes

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SOURCES = ['Cargo.toml', 'Cargo.lock', 'src/lib.rs', 'src/main.rs', 'fetch.py', 'score.py',
           'bdd_fetch.py', 'bdd_score.py', 'bdd_check.py', 'bdd_protocol.json']


def fingerprints():
    return {str((HERE/name).relative_to(ROOT)): digest((HERE/name).read_bytes()) for name in SOURCES}


def freeze(binary, model):
    p = protocol()
    if model.stat().st_size != p['model_size_bytes'] or digest(model.read_bytes()) != p['model_sha256']:
        raise ValueError('existing pinned model changed')
    prior = json.loads((HERE/'road-results.json').read_text())
    sources = fingerprints()
    for name in ['Cargo.toml', 'Cargo.lock', 'src/lib.rs', 'src/main.rs']:
        key = str((HERE/name).relative_to(ROOT))
        if prior['source_sha256'][key] != sources[key]:
            raise ValueError('existing compiled detector source/lock differs from checked baseline')
    binary_sha = digest(binary.read_bytes())
    return {'schema_version': 1, 'protocol_sha256': digest((HERE/'bdd_protocol.json').read_bytes()),
            'source_sha256': sources, 'binary_sha256': binary_sha, 'model_sha256': p['model_sha256'],
            'inferencer_reused_unchanged': True,
            'evaluation': 'All eight new BDD scenes, no BDD-specific calibration or model tuning; fixed pre-inference thresholds',
            'original_labels': {i['id']: i['annotation']['sha256'] for i in p['images']},
            'original_images': {i['id']: i['image']['sha256'] for i in p['images']},
            'data_license_sha256': p['license']['sha256']}


def hand_checks():
    # Hand-labelled disjoint boxes: exact classes, duplicate, wrong class, IoU boundary.
    refs = [{'id': 1, 'category': 'person', 'class_index': 0, 'bbox': [0, 0, 3, 1]},
            {'id': 2, 'category': 'car', 'class_index': 2, 'bbox': [10, 0, 13, 1]}]
    def pred(kind, box, confidence=.9):
        return {'class_index': kind, 'bbox': box, 'confidence': confidence}
    cases = [([pred(0, [0, 0, 3, 1]), pred(2, [10, 0, 13, 1])], (2, 0, 0)),
             ([pred(0, [0, 0, 3, 1]), pred(0, [0, 0, 3, 1], .8)], (1, 1, 1)),
             ([pred(2, [0, 0, 3, 1])], (0, 1, 2)),
             ([pred(0, [1, 0, 4, 1])], (1, 0, 1)),
             ([pred(0, [1.01, 0, 4.01, 1])], (0, 1, 2))]
    for detections, expected in cases:
        actual = tuple(len(x) for x in match_boxes(refs, detections, .5))
        if actual != expected:
            raise ValueError(f'independent hand-labelled contract changed: {actual}, expected {expected}')
    return len(cases)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--prepare-freeze', type=Path)
    parser.add_argument('--freeze', type=Path)
    parser.add_argument('--data', type=Path, default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/bdd-camera/check')
    parser.add_argument('--summary', type=Path)
    args = parser.parse_args()
    binary, model = args.binary.resolve(), args.model.resolve()
    expected = freeze(binary, model)
    if args.prepare_freeze:
        if args.freeze:
            raise ValueError('choose freeze preparation or evaluation')
        with args.prepare_freeze.open('x') as stream:
            json.dump(expected, stream, indent=2, allow_nan=False)
            stream.write('\n')
        print('Saved metadata/source/inferencer protocol before BDD image decoding or inference.')
        return
    if not args.freeze:
        raise ValueError('external --freeze is mandatory before first BDD inference')
    frozen_bytes = args.freeze.read_bytes()
    if json.loads(frozen_bytes) != expected:
        raise ValueError('external BDD freeze differs before image access/inference')
    hand_count = hand_checks()
    fetch(args.data)
    args.output.mkdir(parents=True, exist_ok=True)
    p = protocol()
    for name in ['first', 'repeat']:
        folder = args.output/name
        folder.mkdir(exist_ok=True)
        for image in p['images']:
            subprocess.run([str(binary), '--model', str(model), '--image', str(args.data/image['image']['file']),
                            '--output', str(folder/(image['id']+'.json'))], check=True)
    first = args.output/'first'
    for image in p['images']:
        name = image['id']+'.json'
        a = json.loads((first/name).read_text())
        b = json.loads((args.output/'repeat'/name).read_text())
        if a['detections'] != b['detections']:
            raise ValueError('repeated actual Rust CPU detections differ')
    measured = score(args.data, first)
    file = first/(p['images'][0]['id']+'.json')
    original = file.read_bytes()
    mutations = [('model_sha256', '0'*64), ('source_image_sha256', '0'*64), ('width', 1),
                 ('score_threshold', .01), ('nms_iou_threshold', .9)]
    rejected = []
    for key, value in mutations:
        changed = json.loads(original)
        changed[key] = value
        file.write_text(json.dumps(changed))
        try:
            score(args.data, first)
        except ValueError:
            rejected.append(key)
        else:
            raise ValueError('independent scorer accepted '+key+' mutation')
        finally:
            file.write_bytes(original)
    predicted_image = next(i for i in p['images'] if json.loads((first/(i['id']+'.json')).read_text())['detections'])
    file = first/(predicted_image['id']+'.json')
    original = file.read_bytes()
    for name in ['nonfinite_bbox', 'invalid_class_index']:
        changed = json.loads(original)
        if name == 'nonfinite_bbox':
            changed['detections'][0]['bbox'][2] = float('nan')
        else:
            changed['detections'][0]['class_index'] = True
        file.write_text(json.dumps(changed))
        try:
            score(args.data, first)
        except ValueError:
            rejected.append(name)
        else:
            raise ValueError('independent scorer accepted '+name)
        finally:
            file.write_bytes(original)
    label_path = args.data/p['images'][0]['annotation']['file']
    original_label = label_path.read_bytes()
    label_path.write_bytes(original_label.replace(b'"box2d"', b'"booxd"', 1))
    try:
        score(args.data, first)
    except ValueError:
        rejected.append('modified_original_annotation')
    else:
        raise ValueError('independent scorer accepted modified original label bytes')
    finally:
        label_path.write_bytes(original_label)
    if freeze(binary, model) != expected or args.freeze.read_bytes() != frozen_bytes:
        raise ValueError('source/binary/model/external freeze changed during proof')
    reports = [json.loads((first/(i['id']+'.json')).read_text()) for i in p['images']]
    measured.update({'recorded_utc': datetime.now(timezone.utc).isoformat(), 'freeze': expected,
                     'freeze_sha256': digest(frozen_bytes), 'actual_inference_runs': 16,
                     'repeat_detection_identity': True, 'independent_mutations_rejected': rejected,
                     'independent_hand_labelled_contracts_passed': hand_count,
                     'host': {'os': platform.system(), 'architecture': platform.machine(), 'gpu_used': False},
                     'per_image_inference_seconds': [r['inference_seconds'] for r in reports],
                     'integrity_and_actual_execution_passed': True,
                     'automotive_accuracy_protocol_passed': False,
                     'threshold_tuning_performed': False, 'raw_redistributed': False,
                     'source_model_binary_frozen_before_first_image_decode': True})
    target = args.summary or args.output/'results.json'
    target.write_text(json.dumps(measured, indent=2, allow_nan=False)+'\n')
    print(json.dumps(measured['overall']))
    print(f'Sixteen real Rust CPU runs and independent original-BDD scoring completed; {target}')


if __name__ == '__main__':
    main()
