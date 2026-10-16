#!/usr/bin/env python3
"""Compare fixed Nano views, current serial references, BDD quality and clocks."""
import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import statistics
import struct
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


previous = module('nano_parallel_support', HERE / 'camera_parallel_check.py')
require, digest, load, write, fresh = previous.require, previous.digest, previous.load, previous.write, previous.fresh
require(digest(HERE / 'camera_parallel_check.py') == 'f9d4e081b4989f3ffe3bd40f7f030f817797188161370b636276c127ac90d109', 'pinned parallel support')
latency, freshness, tracking = previous.support()
clock = module('nano_clock', HERE / 'camera_nano_clock.py')
operator = module('nano_float_bits', HERE / 'camera_operator_check.py')
repeat = module('nano_s_reference', HERE / 'camera_repeat_check.py')
consistency = module('nano_consistency', HERE / 'bdd_vehicle_consistency_check.py')
tiles, _, design, matcher = consistency.context()
MODEL_SHA = 'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d'
NEW = ['integrations/onnx/src/camera_nano_detector.rs', 'integrations/onnx/src/camera_nano_analysis.rs',
       'integrations/onnx/src/bin/rustdriving-camera-nano.rs', 'integrations/onnx/profile-nano-parallel.json',
       'integrations/onnx/fetch.py']
ORDER = ['s-first', 'nano-first', 'nano-repeat', 's-repeat']


def exact_f32(actual, expected):
    return type(actual) in (int, float) and math.isfinite(actual) and struct.pack('<f', actual) == struct.pack('<f', expected)


def expected(role):
    paths = dict(runtime='assets/camera-parallel-v1/verification/executable.json',
                 image='assets/camera-parallel-v1/verification/parity/tum-0-first.json')
    base = load(ROOT / paths[role])['compiled_sources']
    return dict(base, **{name: digest(ROOT / name) for name in NEW})


def check_clock_source():
    text = (HERE / 'camera_parallel_check.py').read_text()
    node = next(n for n in ast.parse(text).body if isinstance(n, ast.FunctionDef) and n.name == 'audit_parallel')
    original = ast.get_source_segment(text, node)
    require(hashlib.sha256(original.encode()).hexdigest() == clock.ORIGINAL_FUNCTION_SHA256, 'original clock oracle')
    adapted = original.replace('def audit_parallel(', 'def audit_measured(').replace(
        "report['algorithm']=='yolox_s_measured_serial_replay_v1'", "report['algorithm']=='yolox_nano_measured_replay_v1'")
    text = (HERE / 'camera_nano_clock.py').read_text()
    node = next(n for n in ast.parse(text).body if isinstance(n, ast.FunctionDef) and n.name == 'audit_measured')
    require(ast.get_source_segment(text, node) == adapted, 'clock arithmetic and acceptance unchanged')


def check_image(report, saved, serial=None, parallel=True):
    require(report['schema_version'] == 1 and report['algorithm'] == (
        'yolox_nano_parallel_image_analysis_v1' if parallel else 'yolox_nano_serial_image_analysis_v1'), 'distinct Nano producer')
    pins = expected('image')
    require(report['compiled_sources'] == pins and report['model_sha256'] == MODEL_SHA, 'complete Nano producer receipt')
    require(report['control_permission'] is False, 'no camera control permission')
    require(report['source_image_sha256'] == saved['source_image_sha256'], 'original encoded image')
    base = report['base_prediction']; old = saved['base_prediction']
    require(base['compiled_sources'] == pins and base['algorithm'] == 'pinned_yolox_nano_fixed_views_v1', 'Nano base identity')
    require(base['model_sha256'] == MODEL_SHA and base['source_image_sha256'] == report['source_image_sha256'], 'original Nano weights/image')
    require((base['width'], base['height']) == (old['width'], old['height']), 'unchanged dimensions')
    require(exact_f32(base['score_threshold'], .3) and exact_f32(base['nms_iou_threshold'], .45), 'unchanged threshold f32 bits')
    require(len(base['views']) == len(old['views']) == 4, 'all original four views')
    detector_pins = load(ROOT / 'assets/camera-sequence-v1/recorded/exports/first-0.json')['base_prediction']['views'][0]['prediction']['compiled_sources']
    detector_pins = dict(detector_pins, **{name: digest(ROOT / name) for name in NEW if 'camera_nano_analysis.rs' not in name})
    for view, original in zip(base['views'], old['views']):
        require(view['rectangle'] == original['rectangle'] and view['rgb_sha256'] == original['rgb_sha256'], 'original crop pixels/order')
        prediction = view['prediction']
        require(prediction['algorithm'] == 'pinned_yolox_nano_cpu_image_triangle_argmax_classwise'
                and prediction['model_profile'] == 'yolox-nano-parallel-v1', 'Nano detector identity')
        require(prediction['compiled_sources'] == detector_pins and prediction['model_profile_sha256'] == digest(HERE / 'profile-nano-parallel.json'), 'complete Nano detector source/profile')
        require(prediction['model_sha256'] == MODEL_SHA and prediction['input_shape'] == [1, 3, 416, 416]
                and prediction['output_shape'] == [1, 3549, 85], 'actual Nano tensor geometry')
        require(prediction['width'] == view['rectangle']['width'] and prediction['height'] == view['rectangle']['height']
                and exact_f32(prediction['score_threshold'], .3) and exact_f32(prediction['nms_iou_threshold'], .45), 'local geometry and threshold f32 bits')
        require(all(math.isfinite(prediction[k]) and prediction[k] > 0 for k in ('model_load_seconds', 'inference_seconds')), 'actual bounded forward clocks')
    fixture = dict(width=base['width'], height=base['height'], views=[dict(rectangle=v['rectangle'], detections=v['prediction']['detections']) for v in base['views']])
    merged = tiles.merge(fixture)
    tiles.same(merged, {key: base[key] for key in ('detections', 'detection_origins', 'merge')})
    tiles.same(consistency.oracle(fixture, tiles), report['result'])
    execution = report['execution']
    require({key: value for key, value in execution.items() if key != 'analysis_wall_s'} == dict(
        model_read=True, images_read=1, model_forwards=4, inference_run=True, view_workers=4 if parallel else 1, shared_model_plans=1), 'actual Nano execution inventory')
    require(math.isfinite(execution['analysis_wall_s']) and execution['analysis_wall_s'] >= max(v['prediction']['inference_seconds'] for v in base['views']), 'wrapper clock covers every view')
    if serial is not None:
        require(previous.stripped(base) == previous.stripped(serial['base_prediction']), 'exact current serial/parallel per-view and merge parity')
        require(report['result'] == serial['result'], 'exact current serial/parallel decisions')


def controls(args):
    previous.controls(args)
    out = args.output; rows = []
    with tempfile.TemporaryDirectory(prefix='nano-controls-') as directory:
        temporary = Path(directory); fake = temporary / 'zero.onnx'
        with fake.open('xb') as stream:
            stream.truncate(3_659_407)
        link = temporary / 'model-link.onnx'; link.symlink_to(fake)
        for name, model, image, error in [
            ('wrong-S-model', args.s_model, out / 'inputs/0.png', 'model size differs'),
            ('wrong-nano-hash', fake, out / 'inputs/0.png', 'model SHA-256 differs'),
            ('model-symlink', link, out / 'inputs/0.png', 'regular non-symlink file')]:
            rows.append(dict(name=name, **latency.native(args.binary, ['--model', model, '--image', image], out / (name + '-unexpected.json'), 2, error)))
    rows.append(dict(name='serial-before-model', **latency.native(args.binary, ['--model', 'missing', '--serial-image', 'missing'], out / 'serial-unexpected.json', 2, 'read image:')))
    rows.append(dict(name='unknown-cli', **latency.native(args.binary, ['--unknown'], out / 'cli-unexpected.json', 2, 'expected --model')))
    check_clock_source()
    write(out / 'nano-summary.json', dict(pass_checks=True, shared_input_rejections=25, nano_rejections=rows,
        output_protections=2, checker_sha256=digest(__file__), executable_sha256=digest(args.binary), valid_model_forwards=0))


def quality(data, reports, projected):
    fresh(projected)
    p = matcher.protocol()
    for image in p['images']:
        report = reports[image['id']]; base = report['base_prediction']
        write(projected / (image['id'] + '.json'), dict(model_sha256=MODEL_SHA, source_image_sha256=report['source_image_sha256'],
            width=base['width'], height=base['height'], score_threshold=.3, nms_iou_threshold=.45, detections=report['result']['detections']))
    result = matcher.score(data, projected)
    for row in result['images']:
        row['original_protocol_partition'] = row['partition']; row['partition'] = 'viewed_development'
    gates = design['evaluation']['quality_gates']; overall = result['overall']
    passed = (overall['precision'] is not None and overall['recall'] is not None
              and overall['precision'] >= gates['minimum_overall_precision']
              and overall['recall'] >= gates['minimum_overall_recall']
              and all(row['recall'] >= gates['minimum_recall_each_class_with_reference'] for row in result['per_class'].values() if row['recall'] is not None))
    result.update(current_use='already viewed development images', new_holdout=False, quality_gates=dict(gates, **{'pass': passed}))
    return result


def run(args):
    out = fresh(args.output); check_clock_source()
    control = load(args.controls / 'nano-summary.json')
    require(control['pass_checks'] and control['checker_sha256'] == digest(__file__) and control['executable_sha256'] == digest(args.binary), 'matching Nano controls')
    source_path = ROOT / 'assets/camera-sequence-v1/sequence.json'; source = load(source_path)
    require(digest(source_path) == previous.SOURCE_SHA and digest(args.model) == MODEL_SHA and digest(args.s_model) == previous.MODEL_SHA, 'original schedule and both pinned models')
    binaries = dict(s=args.s_binary, nano=args.binary); receipts = {}
    for name, binary in dict(binaries, original=args.original).items():
        if name == 'original':continue
        receipts[name], _ = latency.native(binary, ['--executable-receipt'], out / (name + '-executable.json'))
        require(receipts[name]['executable_sha256'] == digest(binary), 'actual executable binding')
        if name == 'nano':require(receipts[name]['compiled_sources'] == expected('runtime'), 'complete Nano runtime sources')
        else:require(receipts[name]['compiled_sources'] == load(ROOT / 'assets/camera-operators-v2/parallel-validation/repeat2x-executable.json')['compiled_sources'], 'unchanged lowered S baseline')
    selection = [(f'tum-{i}', source_path.parent / event['input']['path'], ROOT / f'assets/camera-sequence-v1/recorded/exports/first-{i}.json') for i, event in enumerate(source['events'])]
    selection += [(p.stem, args.bdd / (p.stem + '.jpg'), p) for p in sorted((ROOT / 'assets/camera-analysis-v1/publication-repair/regression/first').glob('*.json'))]
    require(len(selection) == 16 and all(digest(image) == load(saved)['source_image_sha256'] for _, image, saved in selection), 'complete unchanged 16-image inventory')
    pins = dict(expected('runtime'))
    for name in ['camera_nano_check.py', 'camera_nano_clock.py', 'camera_parallel_check.py', 'camera_latency_check.py',
                 'camera_operator_check.py', 'camera_repeat_check.py', 'bdd_vehicle_consistency_check.py', 'bdd_vehicle_context_check.py',
                 'bdd_vehicle_vote_check.py', 'bdd_tiles_check.py', 'bdd_s_check.py', 'bdd_score.py', 'bdd_fetch.py', 'bdd_protocol.json', 'score.py']:
        pins['integrations/onnx/' + name] = digest(HERE / name)
    for name in ['integrations/camera-tracking/check.py', 'integrations/camera-tracking/freshness_check.py']:
        pins[name] = digest(ROOT / name)
    write(out / 'before-inference.json', dict(order=ORDER, source_sha256=pins, source_sequence_sha256=previous.SOURCE_SHA,
        model_sha256=MODEL_SHA, model_bytes=3_659_407, s_model_sha256=previous.MODEL_SHA, executable_receipts=receipts,
        original_executable_sha256=digest(args.original), policy=freshness.POLICY, new_holdout=False, inference_warmup=False,
        image_selection=[dict(name=name, image_sha256=digest(image), reference_report_sha256=digest(saved)) for name, image, saved in selection]))
    imported_forwards = 0
    if args.resume is not None:
        old = load(args.resume / 'before-inference.json'); current = load(out / 'before-inference.json')
        require({k: v for k, v in old.items() if k != 'source_sha256'} == {k: v for k, v in current.items() if k != 'source_sha256'}, 'resumed protocol, models, binaries and complete image selection unchanged')
        changes = {name for name in pins if old['source_sha256'].get(name) != pins[name]}
        require(changes == {'integrations/onnx/camera_nano_check.py'} and old['source_sha256'].keys() == pins.keys(), 'resume permits only an archived auditor correction')
        require(digest(args.resume / 'failed-checker.py') == old['source_sha256']['integrations/onnx/camera_nano_check.py'], 'complete failed auditor source binding')
        for label in ORDER:
            for suffix in ('.json', '-guard-input.json', '-guard-result.json'):
                shutil.copy2(args.resume / (label + suffix), out / (label + suffix))
            imported_forwards += load(out / (label + '.json'))['model_forwards']
        for directory in ('serial', 'original', 'first', 'repeat'):
            paths = list((args.resume / directory).glob('*.json'))
            if paths:
                fresh(out / directory)
                for path in paths:
                    shutil.copy2(path, out / directory / path.name)
                    imported_forwards += 1 if directory == 'original' else 4
        write(out / 'resumed-observations.json', dict(archived_checker_sha256=old['source_sha256']['integrations/onnx/camera_nano_check.py'],
            current_checker_sha256=digest(__file__), captured_prior_model_forwards=imported_forwards,
            original_observation_directory=str(args.resume.relative_to(ROOT)), raw_reports_rewritten=False))
    inspections = {}
    for name, binary in binaries.items():inspections[name], _ = latency.native(binary, ['--inspect-sequence', source_path], out / (name + '-inspection.json'))
    trials = []; timed_forwards = 0
    for label in ORDER:
        mode = label.split('-')[0]; print('Measured ' + label, flush=True)
        if args.resume is None:
            report, wall = latency.native(binaries[mode], ['--model', args.model if mode == 'nano' else args.s_model, '--sequence', source_path], out / (label + '.json'))
        else:
            report = load(out / (label + '.json'))
            wall = next(row['external_wall_s'] for row in load(args.resume / 'trials.json') if row['label'] == label)
        require(report['compiled_sources'] == receipts[mode]['compiled_sources'] and report['source_sequence_sha256'] == previous.SOURCE_SHA, 'actual runtime source/schedule')
        require(report['startup_before_schedule_wall_s'] + report['schedule_wall_s'] <= wall + .01, 'external timer covers all measured work')
        manifest, _, _ = (clock.audit_measured if mode == 'nano' else previous.audit_parallel)(report, source, inspections[mode], freshness, tracking)
        path = out / (label + '-guard-input.json')
        if args.resume is None:
            write(path, manifest)
            guard, _ = latency.native(args.guard, ['--sequence', path], out / (label + '-guard-result.json'))
        else:
            require(load(path) == manifest, 'unchanged independently reconstructed resumed guard events')
            guard = load(out / (label + '-guard-result.json'))
        require(guard['results'] == [entry['output'] for row in report['events'] for entry in row['measured']['guard_events']], 'exact independent guard replay')
        trials.append(dict(label=label, model_forwards=report['model_forwards'], external_wall_s=wall, summary=report['summary'],
            processing_s=[row['measured']['timing']['read_analysis_and_probe_wall_s'] for row in report['events'] if row['measured']['analysis'] is not None]))
        timed_forwards += report['model_forwards']
    write(out / 'trials.json', trials)
    for directory in ('serial', 'first', 'repeat', 'original'):
        if not (out / directory).exists():fresh(out / directory)
    def image_run(binary, flags, path):
        if path.exists():return load(path), None
        return latency.native(binary, flags, path)
    current = {}; parity_forwards = 0
    for name, image, saved in selection:
        print('Nano serial/parallel/original comparison ' + name, flush=True)
        serial, _ = image_run(args.binary, ['--model', args.model, '--serial-image', image], out / 'serial' / (name + '.json'))
        check_image(serial, load(saved), parallel=False); parity_forwards += 4; current[name] = serial
        original, _ = image_run(args.original, ['--model', args.model, '--image', image], out / 'original' / (name + '.json'))
        require(original['model_sha256'] == MODEL_SHA and original['source_image_sha256'] == serial['source_image_sha256'], 'original independent Nano execution')
        require(operator.measurement_bits(original['detections']) == operator.measurement_bits(serial['base_prediction']['views'][0]['prediction']['detections']), 'exact original full-view f32 measurement bits')
        parity_forwards += 1; results = []
        for repeat_name in ('first', 'repeat'):
            actual, _ = image_run(args.binary, ['--model', args.model, '--image', image], out / repeat_name / (name + '.json'))
            check_image(actual, load(saved), serial); parity_forwards += 4; results.append(previous.stripped(actual))
        require(results[0] == results[1], 'exact repeated Nano image outputs excluding durations')
    for label in ORDER:
        report = load(out / (label + '.json'))
        for row in report['events']:
            actual = row['measured']['analysis']
            if actual is None:continue
            name = 'tum-' + str(row['source']['event_index']); saved = load(ROOT / f"assets/camera-sequence-v1/recorded/exports/first-{row['source']['event_index']}.json")
            if label.startswith('nano'):check_image(actual, saved, current[name])
            else:repeat.check_analysis(actual, saved, latency.camera_history.Comparison())
    scored = quality(args.bdd, {name: load(out / 'first' / (name + '.json')) for name, _, _ in selection}, out / 'quality-predictions')
    write(out / 'quality.json', scored)
    baseline_quality = matcher.score(args.bdd, out / 'original'); write(out / 'original-quality.json', baseline_quality)
    manifest_path = out / 'legacy-input.json'; report_path = out / 'first/tum-0.json'
    write(manifest_path, dict(schema_version=1, camera_id='nano-candidate', width=640, height=480, events=[dict(now_s=0., input=dict(kind='analysis', acquired_s=0., report='first/tum-0.json', sha256=digest(report_path)))]))
    legacy = latency.native(args.guard, ['--sequence', manifest_path], out / 'legacy-unexpected.json', 2, 'invalid pinned camera analysis report')
    require(all(digest(ROOT / name) == pin for name, pin in pins.items()) and all(digest(binaries[name]) == receipt['executable_sha256'] for name, receipt in receipts.items()), 'sources and executables unchanged during inference')
    require(digest(args.original) == load(out / 'before-inference.json')['original_executable_sha256'], 'original Nano binary unchanged')
    nano_times = [row['processing_s'][0] for row in trials if row['label'].startswith('nano')]
    s_times = [row['processing_s'][0] for row in trials if row['label'].startswith('s')]
    summary = dict(pass_checks=True, original_images=16, repeats=2, nano_serial_and_parallel_exact=True, original_full_view_f32_bits_exact=True,
        timed_model_forwards=timed_forwards, parity_model_forwards=parity_forwards, actual_model_forwards=timed_forwards + parity_forwards,
        imported_model_forwards=imported_forwards, model_forwards_this_invocation=timed_forwards + parity_forwards - imported_forwards,
        first_image_nano_processing_s=nano_times, first_image_s_processing_s=s_times, ratio_of_first_image_means=statistics.mean(s_times) / statistics.mean(nano_times),
        deadline_gates={row['label']: row['summary']['deadline_gate_pass'] for row in trials}, actual_availability={row['label']: row['summary']['available_at_check_images'] for row in trials},
        quality=scored['overall'], original_single_view_quality=baseline_quality['overall'], inherited_s_quality=load(ROOT / 'assets/camera-analysis-v1/publication-repair/regression/results.json')['overall'],
        quality_gate_pass=scored['quality_gates']['pass'], timing_or_quality_success_required_for_execution=False, new_holdout=False, driving_integration=False,
        legacy_profile_rejection=legacy, checker_sha256=digest(__file__), executable_sha256=digest(args.binary))
    write(out / 'summary.json', summary)
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['controls', 'run'])
    parser.add_argument('--binary', type=Path, default=HERE / 'target/release/rustdriving-camera-nano')
    parser.add_argument('--original', type=Path, default=HERE / 'target/release/rustdriving-camera-detect')
    parser.add_argument('--s-binary', type=Path, default=HERE / 'target/release/rustdriving-camera-repeat2x')
    parser.add_argument('--guard', type=Path, default=ROOT / 'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--s-model', type=Path, default=ROOT / 'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--bdd', type=Path, default=ROOT / 'artifacts/bdd-camera')
    parser.add_argument('--controls', type=Path)
    parser.add_argument('--resume', type=Path, help='Re-audit saved observations after an archived checker-only correction')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    for name in ('binary', 'original', 's_binary', 'guard', 'model', 's_model', 'bdd', 'output'):
        setattr(args, name, getattr(args, name).absolute())
    if args.resume is not None:args.resume = args.resume.absolute()
    if args.mode == 'controls':controls(args)
    else:
        require(args.controls is not None, '--controls required')
        run(args)
