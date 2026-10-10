#!/usr/bin/env python3
"""Replay the immutable, viewed eight-image YOLOX-S diagnostic on actual CPU inference.

Integrity success preserves the first physical quality exit, including a failed gate.
Cross-host tolerance is absolute 1e-5; detection order, classes, matching identities,
counts, gates and source/model/data identities remain fixed. This is not a new holdout.
"""
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
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
ASSETS = ROOT / 'assets/yolox-s-bdd-v1'
DESIGN_SHA = '99a881e518819f0f07c2c1836d6b98ac2e37f38ef3e4a11490c9b68237e90565'
CHECKER_SHA = '35053e7a0e329df354cf694936832c05a7f502e4af7b97337cacf4f9807a25d9'
ABSOLUTE_TOLERANCE = 1e-5
# Captured from the published, preserved first evidence before any wrapper replay.
FIRST_SHA256 = {
    'first/00a395fe-d60c0b47.json': '0b79538cc4cabc356466f6ff2210a7d9a95d08b59099d6bbf3ecae9092019748',
    'first/026c7465-d54954fa.json': '18a652ccaa5ea0579f98b851af6fcdcd5db7f5df8ccf36dbb29049c56f96a076',
    'first/0798a3a8-c4501e0f.json': '3b3976fd0aa1f372bff19061a4d6c998cf63f95087bfd3306356bd3bbcdcfb32',
    'first/0a493f24-352f747e.json': '167488f6c68594e6576800d77952a9b5029208e6cfc302383045b847b5f7b5b6',
    'first/17d21997-6f076249.json': '5a795b303e733896b70ccec5889dfdf3d0693771dc8cffae23495aca774b054d',
    'first/4899be53-dcc6c017.json': '925878df18c96622091415074255960cd017fcbb201ec1833722356669f530a0',
    'first/4dab8d2a-be9667d4.json': '2ad9a4b72a932ffd10e8c305d3b72fab17f594ab3cd9f9bd3ccb396f3ea7cdc3',
    'first/ab2360f2-7704a1bf.json': 'bcf13a21ff3bff04eb966e331d35237be2049e244a874e65a5bd83b03fe04e04',
    'first-freeze.json': '108d0c4beb3550fc618e58fc01a255511a803636ad14acf623cad84ce99fc130',
    'first-results.json': '55839b5884f147e6c355ab1230245f0e0c9871cf087db6784f12ae072ce36a40',
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path, limit=8 * 1024 * 1024):
    path = Path(path)
    require(not path.is_symlink() and not any(p.is_symlink() for p in path.parents),
            'symlink evidence input: ' + str(path))
    with path.open('rb') as stream:
        body = stream.read(limit + 1)
    require(len(body) <= limit, 'bounded evidence input')
    return json.loads(body)


def output_admission(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(), 'fresh output directory required')
    require(not any(p.is_symlink() for p in path.parents), 'symlink output parent')
    return path


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def compare(expected, actual, path='$'):
    """Exact structure/identity, absolute tolerance only for finite float measurements."""
    if type(expected) is float:
        require(type(actual) in (int, float) and math.isfinite(expected)
                and math.isfinite(actual) and abs(expected - actual) <= ABSOLUTE_TOLERANCE,
                'floating measurement differs: ' + path)
    elif type(expected) is dict:
        require(type(actual) is dict and expected.keys() == actual.keys(), 'field inventory: ' + path)
        for key in expected:
            compare(expected[key], actual[key], path + '.' + key)
    elif type(expected) is list:
        require(type(actual) is list and len(expected) == len(actual), 'row inventory: ' + path)
        for index, (left, right) in enumerate(zip(expected, actual)):
            compare(left, right, path + '[' + str(index) + ']')
    else:
        require(type(expected) is type(actual) and expected == actual, 'identity/count/gate: ' + path)


def freeze_view(freeze):
    view = copy.deepcopy(freeze)
    require('binary_sha256' in view, 'binary receipt identity missing')
    del view['binary_sha256']
    return view


def result_view(result):
    view = copy.deepcopy(result)
    for key in ('recorded_utc', 'per_image_inference_seconds'):
        require(key in view, 'missing measured metadata: ' + key)
        del view[key]
    view['freeze'] = freeze_view(view['freeze'])
    return view


def prediction_view(prediction):
    view = copy.deepcopy(prediction)
    for key in ('model_load_seconds', 'inference_seconds'):
        require(key in view, 'missing measured timing: ' + key)
        del view[key]
    return view


def immutable_checker():
    require(digest(HERE / 'bdd_s_check.py') == CHECKER_SHA, 'immutable S checker changed')
    require(digest(ASSETS / 'design.json') == DESIGN_SHA, 'immutable S design changed')
    spec = importlib.util.spec_from_file_location('immutable_s_replay_checker', HERE / 'bdd_s_check.py')
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    design, historical, oracle = checker.context()
    return checker, design, historical, oracle


def validate_freeze(freeze, design):
    require(freeze['design_sha256'] == DESIGN_SHA and freeze['design'] == design, 'exact fixed design')
    require(freeze['prepared_before_any_s_image_decode_or_inference'] is True
            and freeze['opaque_image_and_label_integrity_checked_only'] is True, 'pre-inference freeze')
    require(freeze['model_sha256'] == design['model']['sha256'], 'frozen model identity')
    require(freeze['sources']['integrations/onnx/bdd_s_check.py'] == CHECKER_SHA,
            'frozen independent checker source')
    require(set(freeze['compiled_sources']) == {
        'integrations/onnx/src/bin/rustdriving-camera-detect-s.rs',
        'integrations/onnx/src/yolox_s.rs', 'integrations/onnx/src/lib.rs',
        'integrations/onnx/Cargo.toml', 'integrations/onnx/Cargo.lock',
        'integrations/onnx/profile-s.json', 'integrations/onnx/fetch_s.py',
        'rust-toolchain.toml'}, 'compiled source inventory')
    for relative, expected in freeze['sources'].items():
        require(not Path(relative).is_absolute() and '..' not in Path(relative).parts,
                'source path domain')
        require(digest(ROOT / relative) == expected, 'frozen source bytes: ' + relative)
    for relative, expected in freeze['compiled_sources'].items():
        require(freeze['sources'][relative] == expected, 'compiled binding absent from freeze')


def check_result(result, freeze, data, predictions, checker, design, oracle):
    require(result['freeze'] == freeze, 'result/external freeze mismatch')
    require(result['design_sha256'] == DESIGN_SHA and result['schema_version'] == 1, 'result protocol')
    for flag in ('repeat_detection_identity', 'integrity_and_actual_execution_passed',
                 'reference_boxes_scored_after_all_inference'):
        require(result[flag] is True, 'missing execution integrity: ' + flag)
    require(type(result['actual_inference_runs']) is int and result['actual_inference_runs'] == 16,
            'actual sixteen-run contract')
    require(result['gpu_used'] is False and result['fresh_holdout_claim'] is False
            and result['raw_redistributed'] is False, 'viewed CPU evidence scope')
    measured = checker.score(data, predictions, design, freeze['compiled_sources'], oracle)
    # Published/replayed boxes must independently reproduce every match, miss,
    # denominator and quality gate; float tolerance does not relax discrete gates.
    compare(measured, {key: result[key] for key in measured})
    require(result['non_noop_corruptions_rejected'] == checker.corruption_checks(
        predictions, design, freeze['compiled_sources']), 'checker corruption coverage')
    require(result['limitations'] == design['limits'], 'evidence limitations changed')
    timing = result['per_image_inference_seconds']
    require(type(timing) is list and len(timing) == 8 and all(type(v) in (int, float)
            and math.isfinite(v) and v >= 0 for v in timing), 'actual timing inventory')
    return 0 if measured['viewed_quality_gates_passed'] else 1


def public_first(data, checker, design, oracle):
    expected = {'first-freeze.json', 'first-results.json'} | {
        'first/' + image['id'] + '.json' for image in design['images']}
    require(set(FIRST_SHA256) == expected, 'published first byte pins not finalized')
    for relative, sha in FIRST_SHA256.items():
        require(digest(ASSETS / relative) == sha, 'published first evidence changed: ' + relative)
    freeze = load(ASSETS / 'first-freeze.json')
    result = load(ASSETS / 'first-results.json')
    validate_freeze(freeze, design)
    predictions = {image['id']: load(ASSETS / 'first' / (image['id'] + '.json'))
                   for image in design['images']}
    code = check_result(result, freeze, data, predictions, checker, design, oracle)
    return freeze, result, predictions, code


def self_tests():
    original = dict(images=[dict(image_id='actual-identity-domain', matches=[dict(
        reference_id=4, prediction_index=0, class_index=2, iou=.75)],
        detections=[dict(class_index=2, bbox=[10., 20., 30., 40.], confidence=.9)])],
        true_positives=1, false_negatives=1, viewed_quality_gates_passed=False,
        compiled_sources={'source.rs': 'a' * 64})
    # These are explicit analytic comparator controls, never claimed as inference.
    near = copy.deepcopy(original)
    near['images'][0]['detections'][0]['bbox'][0] += ABSOLUTE_TOLERANCE / 2
    compare(original, near)
    operations = {
        'detection_geometry': lambda d: d['images'][0]['detections'][0]['bbox'].__setitem__(0, 10.001),
        'nonfinite_detection': lambda d: d['images'][0]['detections'][0].__setitem__('confidence', float('nan')),
        'wrong_class': lambda d: d['images'][0]['detections'][0].__setitem__('class_index', 7),
        'wrong_match_identity': lambda d: d['images'][0]['matches'][0].__setitem__('reference_id', 5),
        'wrong_prediction_identity': lambda d: d['images'][0]['matches'][0].__setitem__('prediction_index', 1),
        'wrong_iou': lambda d: d['images'][0]['matches'][0].__setitem__('iou', .5),
        'omitted_image': lambda d: d['images'].clear(),
        'omitted_detection': lambda d: d['images'][0]['detections'].clear(),
        'fabricated_count': lambda d: d.__setitem__('true_positives', 2),
        'removed_miss': lambda d: d.__setitem__('false_negatives', 0),
        'flipped_failed_quality': lambda d: d.__setitem__('viewed_quality_gates_passed', True),
        'changed_source': lambda d: d['compiled_sources'].__setitem__('source.rs', 'b' * 64),
        'boolean_class': lambda d: d['images'][0]['detections'][0].__setitem__('class_index', True),
    }
    rejected = []
    for name, mutate in operations.items():
        changed = copy.deepcopy(original)
        mutate(changed)
        require(changed != original, 'no-op analytic corruption')
        try:
            compare(original, changed)
        except ValueError:
            rejected.append(name)
        else:
            raise ValueError('accepted analytic corruption: ' + name)
    guards = []
    with tempfile.TemporaryDirectory(prefix='bdd-s-replay-guards-') as temporary:
        parent = Path(temporary)
        old = parent / 'old'; old.write_bytes(b'preserved evidence')
        directory = parent / 'existing'; directory.mkdir()
        link = parent / 'link'; link.symlink_to(old)
        dangling = parent / 'dangling'; dangling.symlink_to(parent / 'missing')
        linked_parent = parent / 'linked-parent'; linked_parent.symlink_to(directory, target_is_directory=True)
        for name, target in [('existing_file', old), ('existing_directory', directory),
                             ('valid_symlink', link), ('dangling_symlink', dangling),
                             ('symlink_parent', linked_parent / 'fresh')]:
            try:
                output_admission(target)
            except ValueError:
                guards.append(name)
            else:
                raise ValueError('output guard accepted: ' + name)
        require(old.read_bytes() == b'preserved evidence', 'guard overwrote evidence')
    return dict(kind='analytic_comparison_and_output_guards', actual_inference_runs=0,
                tolerance_inside_bound_passed=True, corruptions_rejected=rejected,
                output_guards_rejected=guards, prior_evidence_preserved=True)


def run(args):
    output = output_admission(args.output)  # Before source/data reads or inference.
    if args.self_test:
        proof = self_tests()
        output.mkdir(parents=True, exist_ok=False)
        write(output / 'results.json', proof)
        return 0
    require(args.binary is not None and args.model is not None, 'actual replay needs --binary and --model')
    checker, design, historical, oracle = immutable_checker()
    binary, model, data = args.binary.absolute(), args.model.absolute(), args.data.absolute()
    require(digest(model) == design['model']['sha256']
            and model.stat().st_size == design['model']['bytes'], 'actual pinned model bytes')
    for image in design['images']:
        for role in ('image', 'annotation'):
            pin = image[role]
            require(digest(data / pin['file']) == pin['sha256']
                    and (data / pin['file']).stat().st_size == pin['bytes'], 'original data bytes')
    require(digest(data / design['license']['file']) == design['license']['sha256'], 'data licence')
    first_freeze, first_result, first_predictions, expected_code = public_first(
        data, checker, design, oracle)
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, str(HERE / 'bdd_s_check.py'), '--binary', str(binary),
               '--model', str(model), '--data', str(data), '--output', str(output / 'actual')]
    started = datetime.now(timezone.utc).isoformat()
    with (output / 'actual.log').open('x') as stream:
        completed = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
    write(output / 'actual-journal.json', dict(command=command, started_utc=started,
        finished_utc=datetime.now(timezone.utc).isoformat(), actual_exit_code=completed.returncode,
        expected_first_physical_exit_code=expected_code, checker_sha256=CHECKER_SHA,
        log_sha256=digest(output / 'actual.log')))
    require(completed.returncode == expected_code, 'actual physical exit differs; retained raw replay')
    actual = output / 'actual'
    freeze, result = load(actual / 'freeze.json'), load(actual / 'results.json')
    validate_freeze(freeze, design)
    require(freeze_view(first_freeze) == freeze_view(freeze), 'cross-host exact freeze/source/design')
    require(freeze['binary_sha256'] == digest(binary), 'actual executing binary binding')
    predictions = {}
    for image in design['images']:
        name = image['id']
        first = load(actual / 'first' / (name + '.json'))
        repeat = load(actual / 'repeat' / (name + '.json'))
        checker.validate_prediction(repeat, image, design, freeze['compiled_sources'])
        require(first['detections'] == repeat['detections'], 'same-host exact repeat detection identity')
        compare(prediction_view(first_predictions[name]), prediction_view(first), '$.raw.' + name)
        predictions[name] = first
    verified_code = check_result(result, freeze, data, predictions, checker, design, oracle)
    require(verified_code == expected_code, 'independently reconstructed physical gate differs')
    compare(result_view(first_result), result_view(result))
    stages = load(actual / 'stage-journal.json')
    require(stages == [dict(stage=stage, image_id=image['id'], exit_code=0)
                       for stage in ('first', 'repeat') for image in design['images']], 'actual sixteen-run journal')
    require(digest(HERE / 'bdd_s_check.py') == CHECKER_SHA, 'checker changed during replay')
    for relative, sha in FIRST_SHA256.items():
        require(digest(ASSETS / relative) == sha, 'first evidence changed during replay')
    proof = dict(schema='yolox-s-bdd-viewed-replay-v1', design_sha256=DESIGN_SHA,
        checker_sha256=CHECKER_SHA, wrapper_sha256=digest(Path(__file__)), first_evidence_sha256=FIRST_SHA256,
        integrity_and_reproduction_passed=True, actual_inference_runs=16,
        actual_checker_exit_code=completed.returncode, first_physical_exit_code=expected_code,
        viewed_quality_gates_passed=result['viewed_quality_gates_passed'],
        overall=result['overall'], per_class=result['per_class'], images=result['images'],
        cross_host_absolute_tolerance=ABSOLUTE_TOLERANCE, relative_tolerance=0,
        exact_identity_counts_gates_sources=True, same_host_repeat_detection_identity=True,
        exclusions=['result.recorded_utc', 'result.per_image_inference_seconds',
                    'freeze.binary_sha256', 'prediction.model_load_seconds', 'prediction.inference_seconds'],
        fresh_holdout_claim=False, threshold_or_model_tuning_performed=False,
        actual_binary_sha256=digest(binary), offline_controls=self_tests())
    write(output / 'results.json', proof)
    print(json.dumps(dict(integrity_and_reproduction_passed=True,
                         viewed_quality_gates_passed=proof['viewed_quality_gates_passed'],
                         actual_checker_exit_code=completed.returncode, overall=proof['overall'])))
    return completed.returncode


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--data', type=Path, default=ROOT / 'artifacts/bdd-camera')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    try:
        sys.exit(run(args))
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        print('YOLOX-S viewed reproduction: ' + str(error), file=sys.stderr)
        sys.exit(2)
