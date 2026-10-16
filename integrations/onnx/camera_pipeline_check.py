#!/usr/bin/env python3
"""Real image execution parity for the offline camera freshness pipeline.

Only standard-library Python is required. Delivery/fault controls are injected;
no identity-accuracy, actual latency or measured sensor-fault claim is made.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
history_spec = importlib.util.spec_from_file_location('pipeline_history', HERE / 'camera_history.py')
camera_history = importlib.util.module_from_spec(history_spec)
history_spec.loader.exec_module(camera_history)
FRESH_CHECK_SHA = '816e6c0a2f53abf8af38cb5b1875127cd86f25fe8294cf8b6db057f3929589b7'
SOURCE_SEQUENCE_SHA = '5fa15877e7d910fe102dae175af40b5ee473c6f614b0740f36127b58b77f864c'
MODEL_SHA = 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'
CASES = ['recorded-healthy', 'recorded-delayed-after-poll',
         'recorded-explicit-fault-recovery', 'recorded-deadline-still-held']


def require(condition, message):
    if not condition:
        raise ValueError(message)


def identical(expected, actual, message, path='$'):
    """Keep exact equality, but locate the first mismatch in remote CI logs."""
    if expected == actual:
        return
    if isinstance(expected, dict) and isinstance(actual, dict) and expected.keys() == actual.keys():
        for key in expected:
            identical(expected[key], actual[key], message, path + '.' + str(key))
    elif isinstance(expected, list) and isinstance(actual, list) and len(expected) == len(actual):
        for index, (left, right) in enumerate(zip(expected, actual)):
            identical(left, right, message, path + f'[{index}]')
    raise ValueError(f'{message}; first difference {path}: expected={repr(expected)[:240]}, actual={repr(actual)[:240]}')


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path):
    require(path.stat().st_size <= 16 * 1024 * 1024, 'bounded JSON report')
    return json.loads(path.read_text())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def fresh(path):
    require(not path.exists() and not path.is_symlink(), 'fresh evidence directory')
    path.mkdir(parents=True)
    return path


def without_timing(value):
    if isinstance(value, dict):
        return {k:without_timing(v) for k,v in value.items() if k not in ('inference_seconds', 'model_load_seconds')}
    if isinstance(value, list):
        return list(map(without_timing, value))
    return value


def native(binary, arguments, destination, expected=0, error=None):
    result = subprocess.run([str(binary), *map(str, arguments), '--output', str(destination)],
                            capture_output=True, text=True, timeout=240)
    require(result.returncode == expected, 'native exit: ' + result.stderr[-2000:])
    if error:
        require(error in result.stderr, 'failure stage: ' + result.stderr)
    require(not list(destination.parent.glob('.rustdriving-camera-*.partial')), 'pipeline staging leaked')
    require(not list(destination.parent.glob('.rustdriving-guard-*.partial')), 'guard staging leaked')
    if expected:
        require(not destination.exists(), 'failure published partial output')
        return dict(exit=result.returncode, stderr=result.stderr)
    return load(destination)


def oracle():
    path = ROOT / 'integrations/camera-tracking/freshness_check.py'
    require(digest(path) == FRESH_CHECK_SHA, 'unchanged freshness checker')
    spec = importlib.util.spec_from_file_location('pipeline_freshness_oracle', path)
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    return support, support.oracle()


def controls(args):
    out = fresh(args.output)
    inputs = fresh(out / 'inputs')
    for index in range(2):
        shutil.copyfile(ROOT / f'assets/camera-sequence-v1/controls/inputs/{index}.png', inputs / f'{index}.png')
    sequence = dict(schema_version=1, camera_id='pipeline-controls', width=8, height=6, events=[
        dict(now_s=str(index * .1 + .005), input=dict(kind='image', acquired_s=str(index * .1),
             path=f'inputs/{index}.png', sha256=digest(inputs / f'{index}.png'))) for index in range(2)])
    valid = out / 'valid.json'
    write(valid, sequence)
    inspected = native(args.binary, ['--inspect-sequence', valid], out / 'inspection.json')
    require(inspected['images_read'] == 2 and not inspected['model_read'] and not inspected['inference_run'], 'full model-free preflight')
    require([r['admission_without_model'] for r in inspected['events']] == ['accepted', 'accepted'], 'valid dry admission')
    require(all('tracks' not in r for r in inspected['events']), 'preflight does not emit synthetic tracks')
    invalid = inputs / 'invalid.png'
    invalid.write_bytes(b'not an image\n')
    link = inputs / 'link.png'
    link.symlink_to('0.png')
    fifo = inputs / 'fifo.png'
    os.mkfifo(fifo)
    operations = {
        'delivery-duplicate': lambda s: s['events'][1].__setitem__('now_s', s['events'][0]['now_s']),
        'delivery-reversal': lambda s: s['events'][1].__setitem__('now_s', '-1'),
        'delivery-too-close': lambda s: s['events'][1].__setitem__('now_s', '0.0050001'),
        'delivery-nonfinite': lambda s: s['events'][1].__setitem__('now_s', 'NaN'),
        'delivery-outside-range': lambda s: s['events'][1].__setitem__('now_s', '1000000000001'),
        'delivery-type': lambda s: s['events'][1].__setitem__('now_s', .1),
        'delivery-text-bound': lambda s: s['events'][1].__setitem__('now_s', '0' * 65),
        'source-nonfinite': lambda s: s['events'][1]['input'].__setitem__('acquired_s', 'NaN'),
        'source-type': lambda s: s['events'][1]['input'].__setitem__('acquired_s', .1),
        'source-text-bound': lambda s: s['events'][1]['input'].__setitem__('acquired_s', '0' * 65),
        'source-invalid-text': lambda s: s['events'][1]['input'].__setitem__('acquired_s', 'bad'),
        'bad-image-hash': lambda s: s['events'][1]['input'].__setitem__('sha256', '0' * 64),
        'invalid-image-hash': lambda s: s['events'][1]['input'].__setitem__('sha256', 'BAD'),
        'bad-dimensions': lambda s: s.__setitem__('width', 9),
        'missing-late-image': lambda s: s['events'][1]['input'].__setitem__('path', 'inputs/missing.png'),
        'corrupt-late-image': lambda s: s['events'][1]['input'].update(path='inputs/invalid.png', sha256=digest(invalid)),
        'image-symlink': lambda s: s['events'][1]['input'].__setitem__('path', 'inputs/link.png'),
        'image-fifo': lambda s: s['events'][1]['input'].__setitem__('path', 'inputs/fifo.png'),
        'empty-sequence': lambda s: s.__setitem__('events', []),
        'event-budget': lambda s: s.__setitem__('events', s['events'] * 17),
        'bad-camera-id': lambda s: s.__setitem__('camera_id', 'bad camera'),
        'bad-config': lambda s: s.__setitem__('config', dict(max_tracks=301)),
        'bad-policy': lambda s: s.__setitem__('policy', dict(recovery_samples=0)),
        'unknown-policy': lambda s: s.__setitem__('policy', dict(unexpected=True)),
        'unknown-input': lambda s: s['events'][1]['input'].__setitem__('unexpected', True),
        'unknown-root': lambda s: s.__setitem__('unexpected', True),
    }
    rejected = []
    for name, change in operations.items():
        value = copy.deepcopy(sequence)
        change(value)
        path = out / (name + '-input.json')
        write(path, value)
        failure = native(args.binary, ['--model', 'missing-model', '--sequence', path], out / (name + '-unexpected.json'), 2)
        require('load model:' not in failure['stderr'], 'reject whole preflight before model access')
        rejected.append(dict(name=name, **failure))
    oversized = out / 'oversized.json'
    with oversized.open('xb') as stream:
        stream.truncate(512 * 1024 + 1)
    rejected.append(dict(name='sequence-budget', **native(args.binary, ['--inspect-sequence', oversized], out / 'oversized-unexpected.json', 2, '512 KiB')))
    model = inputs / 'invalid.onnx'
    model.write_bytes(b'bad model')
    rejected.append(dict(name='model-size', **native(args.binary, ['--model', model, '--sequence', valid], out / 'model-unexpected.json', 2, 'load model:')))
    no_images = dict(schema_version=1, camera_id='no-images', width=8, height=6,
                     events=[dict(now_s='0', input=dict(kind='tick')), dict(now_s='.1', input=dict(kind='failed'))])
    path = out / 'no-images.json'
    write(path, no_images)
    result = native(args.binary, ['--model', 'missing-model', '--sequence', path], out / 'no-images-result.json')
    zero = dict(model_loads=0, preflight_image_reads=0, inference_image_reads=0, skipped_image_events=0, model_forwards=0)
    require(result['execution'] == zero, 'no-image sequence must not open model')
    rejected_images = copy.deepcopy(sequence)
    rejected_images['events'][0].update(now_s='1')
    rejected_images['events'][1].update(now_s='1.1')
    rejected_images['events'][1]['input']['acquired_s'] = '2'
    rejected_images['events'] += [dict(now_s='1.2', input=dict(kind='tick')), dict(now_s='1.3', input=dict(kind='failed'))]
    for now, source in [('1.4', '1'), ('1.5', '-1')]:
        image = copy.deepcopy(sequence['events'][0]['input'])
        image['acquired_s'] = source
        rejected_images['events'].append(dict(now_s=now, input=image))
    path = out / 'all-rejected-images.json'
    write(path, rejected_images)
    first = native(args.binary, ['--model', 'missing-model', '--sequence', path], out / 'all-rejected-first.json')
    repeat = native(args.binary, ['--model', 'missing-model', '--sequence', path], out / 'all-rejected-repeat.json')
    require(first == repeat, 'model-free repeat identical')
    require(first['execution'] == zero | dict(preflight_image_reads=4, skipped_image_events=4), 'all rejected images skip model and inference')
    expected = ['stale_sample', 'future_sample', 'polled', 'acquisition_failed', 'before_fault', 'invalid_acquisition_time']
    require([e['freshness']['decision'] for e in first['events']] == expected, 'semantic rejection is reported')
    require(all(e['analysis'] is None and not e['freshness']['ready'] for e in first['events']), 'no invented results')
    # Probe reused-image and acquisition-order admission without model execution.
    for name, change, decision in [
        ('reused-image', lambda s: s['events'][1]['input'].update(path='inputs/0.png', sha256=digest(inputs/'0.png')), 'invalid_payload'),
        ('out-of-order', lambda s: s['events'][1]['input'].__setitem__('acquired_s', '0'), 'out_of_order')]:
        value = copy.deepcopy(sequence)
        change(value)
        path = out / (name + '.json')
        write(path, value)
        inspected = native(args.binary, ['--inspect-sequence', path], out / (name + '-inspection.json'))
        require(inspected['events'][1]['admission_without_model'] == decision, 'dry rejected-image admission')
    existing = out / 'existing.txt'
    existing.write_bytes(b'preserve\n')
    dangling = out / 'existing-link.json'
    dangling.symlink_to('missing.json')
    output_rejections = []
    for destination in (existing, dangling):
        process = subprocess.run([str(args.binary), '--inspect-sequence', 'missing-source', '--output', str(destination)],
                                 capture_output=True, text=True, timeout=30)
        require(process.returncode == 2 and 'fresh output' in process.stderr, 'output reserved before source reads')
        output_rejections.append(dict(name=destination.name, exit=process.returncode, stderr=process.stderr))
    require(existing.read_bytes() == b'preserve\n' and dangling.is_symlink(), 'existing output changed')
    receipt = native(args.binary, ['--executable-receipt'], out / 'executable.json')
    require(receipt['executable_sha256'] == digest(args.binary), 'executable receipt')
    for path, pin in receipt['compiled_sources'].items():
        require(digest(ROOT / path) == pin, 'compiled sources match')
    special = dict(symlinks={str(p.relative_to(out)):os.readlink(p) for p in (link, dangling)},
                   fifo=str(fifo.relative_to(out)), sparse_files={oversized.name:oversized.stat().st_size})
    write(out / 'special-files.json', special)
    for path in (link, dangling, fifo, oversized):
        path.unlink()
    write(out / 'summary.json', dict(pass_checks=True, rejected_inputs=rejected, rejected_before_model_loading=27,
        model_size_rejection=True, output_rejections=output_rejections, existing_output_preserved=True,
        all_rejected_images_without_model=True, all_rejected_image_forwards=0, repeated_model_free_result_identical=True,
        reused_image_and_out_of_order_dry_admission=True, executable_sha256=digest(args.binary), checker_sha256=digest(__file__)))
    print(json.dumps(dict(pass_checks=True, rejected_inputs=len(rejected), no_model_all_rejected=True)), flush=True)


def run(args):
    out = fresh(args.output)
    control = load(args.controls / 'summary.json')
    require(control['pass_checks'] and control['executable_sha256'] == digest(args.binary)
            and control['checker_sha256'] == digest(__file__), 'matching source-bound controls')
    require(digest(args.model) == MODEL_SHA and args.model.stat().st_size == 35_858_002, 'pinned original model')
    source_path = ROOT / 'assets/camera-sequence-v1/sequence.json'
    require(digest(source_path) == SOURCE_SEQUENCE_SHA, 'unchanged original eight images')
    source = load(source_path)
    image_by_hash = {e['input']['sha256']:(e['timestamp_s'], source_path.parent/e['input']['path']) for e in source['events']}
    saved = ROOT / 'assets/camera-freshness-v1/verification/cases'
    freshness, support = oracle()
    receipt = native(args.binary, ['--executable-receipt'], out / 'executable.json')
    source_pins = dict(receipt['compiled_sources'])
    for path in [Path(__file__), HERE/'camera_history.py', HERE/'prepare_camera_pipeline.py', ROOT/'integrations/camera-tracking/freshness_check.py',
                 ROOT/'integrations/camera-tracking/check.py']:
        source_pins[str(path.relative_to(ROOT))] = digest(path)
    write(out / 'before-inference.json', dict(executable_sha256=digest(args.binary), source_sha256=source_pins,
        model_sha256=digest(args.model), source_sequence_sha256=SOURCE_SEQUENCE_SHA,
        guard_binary_sha256=digest(args.guard), tracker_binary_sha256=digest(args.tracker),
        real_object_identity_ground_truth=False, all_delivery_times_and_faults_injected=True))
    protocols = []
    # Freeze all four manifests and their selected reports before any inference.
    for name in CASES:
        original = load(saved / (name + '-input.json'))
        events = []
        for event in original['events']:
            data = event['input']
            if data['kind'] == 'analysis':
                report_path = saved/data['report']
                require(digest(report_path) == data['sha256'], 'saved detector report provenance')
                report = load(report_path)
                timestamp, image_path = image_by_hash[report['source_image_sha256']]
                require(float(timestamp) == data['acquired_s'], 'original source time preserved')
                converted = dict(kind='image', acquired_s=timestamp, path=os.path.relpath(image_path, out), sha256=report['source_image_sha256'])
            else:
                converted = copy.deepcopy(data)
            events.append(dict(now_s=repr(event['now_s']), input=converted))
        manifest = dict(schema_version=1, camera_id=original['camera_id'], width=640, height=480,
                        config=original['config'], policy=original['policy'], events=events)
        path = out / (name + '-input.json')
        write(path, manifest)
        protocols.append((name, path, manifest))
    write(out / 'design.json', dict(data_role='Previously viewed eight-frame TUM indoor development slice; original indices102..109',
        source_sequence_sha256=SOURCE_SEQUENCE_SHA, original_indices=list(range(102,110)),
        original_acquisition_text=[e['timestamp_s'] for e in source['events']],
        image_sha256=list(image_by_hash), protocols={name:digest(path) for name,path,_ in protocols},
        cases_selected_from_unchanged_freshness_protocol=True, all_delivery_times_and_faults_injected=True,
        independent_object_identity_ground_truth=False, planned_model_forwards=216))
    # Also exercise the user-facing decimal-lag conversion without model access.
    prepared = out / 'prepared-example.json'
    preparation = subprocess.run([sys.executable, str(HERE/'prepare_camera_pipeline.py'), '--sequence', str(source_path),
        '--replay-lag-s', '.005', '--output', str(prepared)], capture_output=True, text=True, timeout=30)
    require(preparation.returncode == 0, 'prepare example: '+preparation.stderr)
    prepared_value = load(prepared)
    require([e['input']['acquired_s'] for e in prepared_value['events']] == [e['timestamp_s'] for e in source['events']], 'preparer keeps acquisition text')
    native(args.binary, ['--inspect-sequence', prepared], out / 'prepared-inspection.json')
    write(out / 'preparation-receipt.json', json.loads(preparation.stdout))
    exports = fresh(out / 'exports')
    rows, forwards = [], 0
    historical = camera_history.Comparison()
    for name, path, manifest in protocols:
        print('Running '+name, flush=True)
        inspected = native(args.binary, ['--inspect-sequence', path], out / (name + '-inspection.json'))
        first = native(args.binary, ['--model', args.model, '--sequence', path], out / (name + '-first.json'))
        repeat = native(args.binary, ['--model', args.model, '--sequence', path], out / (name + '-repeat.json'))
        print(json.dumps(dict(pipeline_case=name, first_forwards=first['execution']['model_forwards'],
                              repeat_forwards=repeat['execution']['model_forwards'])), flush=True)
        require(without_timing(first) == without_timing(repeat), 'complete repeat excluding only measured inference/load time')
        require(first['compiled_sources'] == receipt['compiled_sources'] and first['source_sequence_sha256'] == digest(path), 'source/sequence receipts')
        require(first['algorithm'] == 'yolox_s_camera_pipeline_v1' and first['control_permission'] is False
                and first['freshness_clock'] == 'caller_supplied_replay', 'scope and clock identity')
        previous = load(saved / (name + '-first.json'))
        current_freshness = [e['freshness'] for e in first['events']]
        historical.same(previous['results'], current_freshness, name + '.freshness')
        exact_analyzer_parity = True
        require([e['admission_without_model'] for e in inspected['events']] == [e['freshness']['decision'] for e in first['events']], 'preflight/runtime decisions agree')
        guard_events, analysis_count, image_count = [], 0, 0
        for index,(event, actual) in enumerate(zip(manifest['events'], first['events'])):
            data = event['input']
            require(actual['event_index'] == index and actual['now_text'] == event['now_s'] and actual['now_s'] == float(event['now_s']), 'delivery-clock identity')
            if data['kind'] == 'image':
                image_count += 1
                require(actual['acquired_text'] == data['acquired_s'] and actual['acquired_s'] == float(data['acquired_s'])
                        and actual['source_image_sha256'] == data['sha256'], 'acquisition/image identity')
                if actual['analysis'] is not None:
                    analysis_count += 1
                    require(not actual['inference_skipped'] and actual['freshness']['decision'] == 'accepted', 'inference only for admitted images')
                    old_report = next(load(saved/old['input']['report']) for old in load(saved/(name+'-input.json'))['events']
                        if old['input']['kind']=='analysis' and load(saved/old['input']['report'])['source_image_sha256']==data['sha256'])
                    historical.same(without_timing(old_report), without_timing(actual['analysis']), name + f'.analysis[{index}]')
                    exact_analyzer_parity &= without_timing(old_report) == without_timing(actual['analysis'])
                    export = exports / (name + f'-{index}.json')
                    write(export, actual['analysis'])
                    guard_input = dict(kind='analysis', acquired_s=actual['acquired_s'], report=os.path.relpath(export,out), sha256=digest(export))
                else:
                    require(actual['inference_skipped'] and actual['freshness']['decision'] != 'accepted', 'rejected image skips inference')
                    guard_input = dict(kind='detections', acquired_s=actual['acquired_s'], image_sha256=data['sha256'], detections=[])
            else:
                require(actual['analysis'] is None and not actual['inference_skipped'], 'poll/failure has no analysis')
                guard_input = copy.deepcopy(data)
            guard_events.append(dict(now_s=actual['now_s'], input=guard_input))
        execution = dict(model_loads=1, preflight_image_reads=image_count, inference_image_reads=analysis_count,
                         skipped_image_events=image_count-analysis_count, model_forwards=analysis_count*4)
        require(first['execution'] == execution and repeat['execution'] == execution, 'one plan and exact forward count')
        forwards += execution['model_forwards'] * 2
        guard_manifest = dict(schema_version=1, camera_id=manifest['camera_id'], width=640, height=480,
            config=manifest['config'], policy=manifest['policy'], events=guard_events)
        guard_path = out / (name + '-guard-input.json')
        write(guard_path, guard_manifest)
        guard_result = native(args.guard, ['--sequence', guard_path], out / (name + '-guard-result.json'))
        require(guard_result['results'] == [e['freshness'] for e in first['events']], 'exact standalone guard replay')
        converted,_ = freshness.normalized(guard_manifest, out)
        backend_events,snapshots = freshness.check_contract(guard_manifest, converted, guard_result['results'], support)
        tracker_path = out / (name + '-tracker-input.json')
        write(tracker_path, dict(schema_version=1, camera_id=manifest['camera_id'], width=640, height=480,
            config=manifest['config'], events=backend_events))
        tracker_result = native(args.tracker, ['--sequence', tracker_path], out / (name + '-tracker-result.json'))
        require(tracker_result['snapshots'] == snapshots, 'exact standalone tracking replay')
        rows.append(dict(name=name, events=len(first['events']), accepted_images=analysis_count, skipped_images=image_count-analysis_count,
            model_forwards_across_two_runs=execution['model_forwards']*2, repeat_identical_except_timing=True,
            complete_analyzer_parity=exact_analyzer_parity, prior_freshness_parity=previous['results'] == current_freshness,
            historical_confidence_compatible=True, guard_replay_identical=True, tracker_replay_identical=True,
            final_ready=first['events'][-1]['freshness']['ready']))
        print('Verified '+name, flush=True)
    require(forwards == 216, 'fixed actual model forward count')
    for path,pin in source_pins.items():
        require(digest(ROOT/path) == pin, 'sources changed during inference')
    require(digest(args.binary) == receipt['executable_sha256'], 'executable changed during inference')
    write(out/'summary.json', dict(pass_checks=True, cases=rows, recorded_unique_images=8, actual_model_forwards=forwards,
        native_pipeline_runs=8, model_loads_per_sequence=1, image_events_skipped_per_four_cases=2,
        complete_analyzer_parity=all(row['complete_analyzer_parity'] for row in rows),
        prior_freshness_parity=all(row['prior_freshness_parity'] for row in rows),
        historical_comparison=historical.report(), standalone_guard_and_tracker_replay_identical=True,
        independent_freshness_motion_lifecycle_and_flow_verified=True, flow_exhaustive_selfchecks=96,
        all_delivery_times_and_faults_injected=True, actual_execution_latency_used_for_freshness=False,
        actual_sensor_failures_measured=False, real_object_identity_accuracy_measured=False,
        executable_sha256=digest(args.binary), checker_sha256=digest(__file__), source_sha256=source_pins))
    print(json.dumps(dict(pass_checks=True, cases=4, actual_model_forwards=forwards)), flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['controls','run'])
    parser.add_argument('--binary',type=Path,default=HERE/'target/release/rustdriving-camera-pipeline')
    parser.add_argument('--guard',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--tracker',type=Path,default=ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-track')
    parser.add_argument('--model',type=Path,default=ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--controls',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    for key in ('binary','guard','tracker','model','output'):
        setattr(args,key,getattr(args,key).absolute())
    if args.mode=='controls':
        controls(args)
    else:
        require(args.controls is not None,'--controls required for run')
        run(args)
