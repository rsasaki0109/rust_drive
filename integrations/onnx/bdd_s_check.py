#!/usr/bin/env python3
"""Source-bound YOLOX-S viewed BDD diagnostics; preserve the nano first result."""
import argparse
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ASSETS = ROOT/'assets/yolox-s-bdd-v1'
DESIGN_SHA = '99a881e518819f0f07c2c1836d6b98ac2e37f38ef3e4a11490c9b68237e90565'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path, limit=8*1024*1024):
    with Path(path).open('rb') as stream:
        body = stream.read(limit+1)
    require(len(body) <= limit, 'bounded JSON input exceeded')
    return json.loads(body)


def new_directory(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(), 'output already exists')
    require(not any(parent.is_symlink() for parent in path.parents), 'symlink output parent')
    path.mkdir(parents=True, exist_ok=False)
    return path


def write(path, data):
    with Path(path).open('x') as stream:
        json.dump(data, stream, indent=2, allow_nan=False)
        stream.write('\n')


def context():
    require(digest(ASSETS/'design.json') == DESIGN_SHA, 'fixed S design changed')
    design = load(ASSETS/'design.json')
    old = design['historical_nano']
    require(digest(ROOT/old['result_path']) == old['result_sha256'], 'nano first result changed')
    require(digest(ROOT/old['freeze_path']) == old['freeze_sha256'], 'nano first freeze changed')
    pins = load(ROOT/old['freeze_path'])['source_sha256']
    for path, expected in pins.items():
        require(digest(ROOT/path) == expected, 'historical source changed: '+path)
    # Import only after verifying the immutable independent matching sources.
    sys.path.insert(0, str(HERE))
    spec = importlib.util.spec_from_file_location('s_immutable_bdd_math', HERE/'bdd_score.py')
    oracle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(oracle)
    return design, pins, oracle


def validate_prediction(report, image, design, compiled):
    model = design['model']
    require(report['model_sha256'] == model['sha256'], 'S model identity')
    require(report['source_image_sha256'] == image['image']['sha256'], 'original image identity')
    require((report['width'], report['height']) == (image['width'], image['height']), 'image dimensions')
    require(report['score_threshold'] == .3 and report['nms_iou_threshold'] == .45, 'fixed thresholds')
    require(report['model_profile'] == 'yolox-s-v1', 'model profile')
    require(report['model_profile_sha256'] == digest(HERE/'profile-s.json'), 'model profile bytes')
    require(report['input_shape'] == model['input_shape']
            and report['output_shape'] == model['output_shape']
            and all(type(x) is int for x in report['input_shape']+report['output_shape']), 'tensor domains')
    require(report['compiled_sources'] == compiled, 'compiled detector provenance')
    for name in ('model_load_seconds', 'inference_seconds'):
        require(type(report[name]) in (int, float) and math.isfinite(report[name])
                and report[name] >= 0, 'invalid timing measurement')
    detections = report['detections']
    require(isinstance(detections, list) and len(detections) <= 300, 'detection bound')
    for detection in detections:
        box, confidence, kind = detection['bbox'], detection['confidence'], detection['class_index']
        require(type(kind) is int and 0 <= kind < 80, 'class identity')
        require(type(confidence) in (int, float) and math.isfinite(confidence)
                and .3 <= confidence <= 1, 'confidence domain')
        require(isinstance(box, list) and len(box) == 4
                and all(type(v) in (int, float) and math.isfinite(v) for v in box)
                and 0 <= box[0] < box[2] <= image['width']
                and 0 <= box[1] < box[3] <= image['height'], 'box domain')


def score(data, reports, design, compiled, oracle):
    rows = []
    for image in design['images']:
        require(digest(data/image['annotation']['file']) == image['annotation']['sha256'], 'label identity')
        labels = load(data/image['annotation']['file'])
        require(labels['name'] == image['id'] and len(labels['frames']) == 1
                and labels['frames'][0]['timestamp'] == 10000
                and labels['attributes'] == image['attributes'], 'original label metadata')
        refs, excluded, identities = [], [], set()
        for label in labels['frames'][0]['objects']:
            require(label['id'] not in identities, 'duplicate reference identity')
            identities.add(label['id'])
            if 'box2d' not in label:
                continue
            box = [label['box2d'][key] for key in ('x1', 'y1', 'x2', 'y2')]
            require(oracle.valid_box(box, image['width'], image['height']), 'reference geometry')
            ref = dict(id=label['id'], category=label['category'], bbox=box, attributes=label['attributes'])
            if label['category'] in design['class_mapping']:
                refs.append(dict(ref, class_index=design['class_mapping'][label['category']]))
            else:
                require(label['category'] in design['unscored_category_reason'], 'unscoped reference category')
                excluded.append(ref)
        prediction = reports[image['id']]
        validate_prediction(prediction, image, design, compiled)
        scoped = [d for d in prediction['detections'] if d['class_index'] in design['class_mapping'].values()]
        matches, fp, fn = oracle.match_boxes(refs, scoped, .5)
        rows.append(dict(image_id=image['id'], partition='viewed_regression', reference_count=len(refs),
                         matches=matches, false_positives=fp, false_negatives=fn,
                         excluded_original_sign_boxes=excluded,
                         all_class_prediction_count=len(prediction['detections'])))
    names = {0:'person including original rider', 1:'bicycle', 2:'car', 3:'motorcycle',
             5:'bus', 6:'train', 7:'truck', 9:'traffic light'}
    overall = oracle.aggregate(rows)
    classes = {name:oracle.aggregate(rows, kind) for kind,name in names.items()}
    gates = design['evaluation']['quality_gates']
    passed = (overall['precision'] is not None and overall['recall'] is not None
              and overall['precision'] >= gates['minimum_overall_precision']
              and overall['recall'] >= gates['minimum_overall_recall']
              and all(c['recall'] >= gates['minimum_recall_each_class_with_reference']
                      for c in classes.values() if c['recall'] is not None))
    return dict(overall=overall, per_class=classes, images=rows,
                reference_count=sum(row['reference_count'] for row in rows),
                viewed_quality_gates_passed=passed, fresh_holdout_claim=False)


def corruption_checks(reports, design, compiled):
    image = design['images'][0]
    original = reports[image['id']]
    operations = {
        'model':lambda p:p.__setitem__('model_sha256', '0'*64),
        'image':lambda p:p.__setitem__('source_image_sha256', '0'*64),
        'dimensions':lambda p:p.__setitem__('width', 1),
        'score_threshold':lambda p:p.__setitem__('score_threshold', .01),
        'nms_threshold':lambda p:p.__setitem__('nms_iou_threshold', .9),
        'profile':lambda p:p.__setitem__('model_profile', 'yolox-nano'),
        'profile_bytes':lambda p:p.__setitem__('model_profile_sha256', '0'*64),
        'tensor_domain':lambda p:p.__setitem__('output_shape', [1,3549,85]),
        'tensor_type':lambda p:p.__setitem__('output_shape', [True,8400,85]),
        'compiled_source':lambda p:p.__setitem__('compiled_sources', {}),
        'timing':lambda p:p.__setitem__('inference_seconds', float('nan')),
    }
    rejected = []
    for name, change in operations.items():
        mutant = copy.deepcopy(original)
        change(mutant)
        require(json.dumps(mutant, sort_keys=True) != json.dumps(original, sort_keys=True), 'noop corruption')
        try:
            validate_prediction(mutant, image, design, compiled)
        except ValueError:
            rejected.append(name)
        else:
            raise ValueError('corruption accepted: '+name)
    return rejected


def run(args):
    output = new_directory(args.output)
    design, historical_sources, oracle = context()
    binary, model, data = args.binary.absolute(), args.model.absolute(), args.data.absolute()
    require(digest(model) == design['model']['sha256']
            and model.stat().st_size == design['model']['bytes'], 'model byte identity')
    for image in design['images']:
        require(digest(data/image['image']['file']) == image['image']['sha256'], 'image pin')
        require(digest(data/image['annotation']['file']) == image['annotation']['sha256'], 'label pin')
    require(digest(data/design['license']['file']) == design['license']['sha256'], 'data licence')
    subprocess.run([str(binary), '--executable-receipt', '--output', str(output/'receipt.json')], check=True)
    receipt = load(output/'receipt.json')
    compiled = receipt['compiled_sources']
    require(set(compiled) == {'integrations/onnx/src/bin/rustdriving-camera-detect-s.rs',
            'integrations/onnx/src/yolox_s.rs', 'integrations/onnx/src/lib.rs',
            'integrations/onnx/Cargo.toml', 'integrations/onnx/Cargo.lock',
            'integrations/onnx/profile-s.json', 'integrations/onnx/fetch_s.py',
            'rust-toolchain.toml'}, 'compiled source inventory')
    require(receipt['executable_sha256'] == digest(binary), 'executing binary receipt')
    for path, expected in compiled.items():
        require(digest(ROOT/path) == expected, 'compiled source bytes: '+path)
    sources = dict(historical_sources)
    sources.update(compiled)
    for path in ['integrations/onnx/bdd_s_check.py', 'integrations/onnx/fetch_s.py',
                 'integrations/onnx/profile-s.json', 'integrations/onnx/src/bin/rustdriving-camera-detect-s.rs',
                 'assets/yolox-s-bdd-v1/design.json']:
        sources[path] = digest(ROOT/path)
    freeze = dict(design_sha256=DESIGN_SHA, design=design, sources=sources,
                  compiled_sources=compiled, binary_sha256=digest(binary), model_sha256=digest(model),
                  prepared_before_any_s_image_decode_or_inference=True,
                  opaque_image_and_label_integrity_checked_only=True)
    write(output/'freeze.json', freeze)
    stages, reports = [], {}
    for stage in ('first', 'repeat'):
        folder = new_directory(output/stage)
        for image in design['images']:
            target = folder/(image['id']+'.json')
            command = [str(binary), '--model', str(model), '--image', str(data/image['image']['file']), '--output', str(target)]
            with (folder/(image['id']+'.log')).open('x') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=300)
            stages.append(dict(stage=stage, image_id=image['id'], exit_code=result.returncode))
            (output/'stage-journal.json').write_text(json.dumps(stages, indent=2)+'\n')
            require(result.returncode == 0, 'inference failed; preserve first output and journal')
            actual = load(target)
            validate_prediction(actual, image, design, compiled)
            if stage == 'first':
                reports[image['id']] = actual
            else:
                require(actual['detections'] == reports[image['id']]['detections'], 'same-host repeated detections differ')
    # Numerical reference boxes enter scoring only after every actual inference.
    measured = score(data, reports, design, compiled, oracle)
    require(measured['reference_count'] == 138, 'reference denominator changed')
    rejected = corruption_checks(reports, design, compiled)
    require(all(digest(ROOT/path) == sha for path,sha in sources.items()), 'sources changed during trial')
    measured.update(schema_version=1, design_sha256=DESIGN_SHA, freeze=freeze,
                    actual_inference_runs=16, repeat_detection_identity=True,
                    integrity_and_actual_execution_passed=True,
                    reference_boxes_scored_after_all_inference=True,
                    non_noop_corruptions_rejected=rejected, gpu_used=False,
                    recorded_utc=datetime.now(timezone.utc).isoformat(),
                    per_image_inference_seconds=[p['inference_seconds'] for p in reports.values()],
                    limitations=design['limits'], raw_redistributed=False)
    write(output/'results.json', measured)
    print(json.dumps(measured['overall']))
    return 0 if measured['viewed_quality_gates_passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--data', type=Path, default=ROOT/'artifacts/bdd-camera')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        sys.exit(run(args))
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print('YOLOX-S viewed diagnostic: '+str(error), file=sys.stderr)
        sys.exit(2)
