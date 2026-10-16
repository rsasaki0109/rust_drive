#!/usr/bin/env python3
"""Recorded-sequence execution equivalence; no identity accuracy or MOT score."""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess

from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TRACK_CHECK_SHA = '575669cb4e62943cf211138c432969c08da4cd3ec5419b4326d8e7ae8f02ee08'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(path):
    require(path.stat().st_size <= 4 * 1024 * 1024, 'bounded report')
    return json.loads(path.read_text())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def fresh(path):
    require(not path.exists() and not path.is_symlink(), 'fresh evidence directory')
    path.mkdir(parents=True)
    return path


def native(binary, arguments, destination, expected=0, error=None):
    result = subprocess.run([str(binary), *map(str, arguments), '--output', str(destination)],
                            capture_output=True, text=True, timeout=180)
    require(result.returncode == expected, 'native exit: ' + result.stderr[-2000:])
    if error:
        require(error in result.stderr, 'failure stage: ' + result.stderr)
    require(not list(destination.parent.glob('.rustdriving-camera-*.partial')), 'staging leaked')
    if expected:
        require(not destination.exists(), 'failure published partial output')
        return dict(exit=result.returncode, stderr=result.stderr)
    return load(destination)


def controls(args):
    out = fresh(args.output)
    inputs = fresh(out / 'inputs')
    for i in range(2):
        Image.new('RGB', (8, 6), (50 + i, 100, 150)).save(inputs / f'{i}.png')
    sequence = dict(schema_version=1, camera_id='controls', width=8, height=6, events=[
        dict(timestamp_s=f'1341845824.{165751 + i * 125000}',
             input=dict(kind='image', path=f'inputs/{i}.png', sha256=digest(inputs / f'{i}.png')))
        for i in range(2)])
    good = out / 'valid.json'
    write(good, sequence)
    inspect = native(args.binary, ['--inspect-sequence', good], out / 'inspect.json')
    require(not inspect['model_read'] and not inspect['inference_run'] and inspect['images_read'] == 2, 'model-free full preflight')
    invalid = inputs / 'invalid.png'
    invalid.write_bytes(b'not an image')
    link = inputs / 'link.png'
    link.symlink_to('0.png')
    fifo = inputs / 'fifo.png'
    os.mkfifo(fifo)
    rejected = []
    operations = {
        'clock-reversal': lambda s: s['events'][1].__setitem__('timestamp_s', '0'),
        'duplicate-clock': lambda s: s['events'][1].__setitem__('timestamp_s', s['events'][0]['timestamp_s']),
        'nonfinite-clock': lambda s: s['events'][1].__setitem__('timestamp_s', 'NaN'),
        'timestamp-type': lambda s: s['events'][1].__setitem__('timestamp_s', 1.0),
        'invalid-clock-text': lambda s: s['events'][1].__setitem__('timestamp_s', 'bad'),
        'clock-text-budget': lambda s: s['events'][1].__setitem__('timestamp_s', '0' * 65),
        'reused-image': lambda s: s['events'][1].__setitem__('input', copy.deepcopy(s['events'][0]['input'])),
        'bad-hash': lambda s: s['events'][1]['input'].__setitem__('sha256', '0' * 64),
        'bad-dimensions': lambda s: s.__setitem__('width', 9),
        'missing-late-image': lambda s: s['events'][1]['input'].__setitem__('path', 'inputs/missing.png'),
        'invalid-late-image': lambda s: s['events'][1].__setitem__('input', dict(kind='image', path='inputs/invalid.png', sha256=digest(invalid))),
        'image-symlink': lambda s: s['events'][1]['input'].__setitem__('path', 'inputs/link.png'),
        'image-fifo': lambda s: s['events'][1]['input'].__setitem__('path', 'inputs/fifo.png'),
        'too-many-events': lambda s: s.__setitem__('events', s['events'] * 17),
        'empty-sequence': lambda s: s.__setitem__('events', []),
        'unknown-input': lambda s: s['events'][1]['input'].__setitem__('unexpected', True),
        'bad-config': lambda s: s.__setitem__('config', dict(max_tracks=301)),
    }
    for name, change in operations.items():
        value = copy.deepcopy(sequence)
        change(value)
        path = out / (name + '-input.json')
        write(path, value)
        result = native(args.binary, ['--model', 'missing-model', '--sequence', path], out / (name + '-unexpected.json'), 2)
        require('load model:' not in result['stderr'], 'preflight must reject before model access')
        rejected.append(dict(name=name, **result))
    large = out / 'too-large.json'
    with large.open('xb') as stream:
        stream.truncate(512 * 1024 + 1)
    rejected.append(dict(name='manifest-budget', **native(args.binary, ['--inspect-sequence', large], out / 'large-unexpected.json', 2, '512 KiB')))
    model = inputs / 'invalid.onnx'
    model.write_bytes(b'bad model')
    rejected.append(dict(name='model-size', **native(args.binary, ['--model', model, '--sequence', good], out / 'model-unexpected.json', 2, 'load model:')))
    no_images = copy.deepcopy(sequence)
    for i, event in enumerate(no_images['events']):
        event['input'] = dict(kind='missing' if i == 0 else 'failed')
    path = out / 'no-images.json'
    write(path, no_images)
    result = native(args.binary, ['--model', 'missing-model', '--sequence', path], out / 'no-images-result.json')
    require(result['execution'] == dict(model_loads=0, preflight_image_reads=0, inference_image_reads=0, model_forwards=0), 'no-image events must not open model')
    existing = out / 'existing.txt'
    existing.write_bytes(b'preserve\n')
    dangling = out / 'existing-link.json'
    dangling.symlink_to('missing.json')
    for target in (existing, dangling):
        result = subprocess.run([str(args.binary), '--inspect-sequence', 'missing', '--output', str(target)],
                                capture_output=True, text=True, timeout=30)
        require(result.returncode == 2 and 'fresh output' in result.stderr, 'output admission before source reads')
    require(existing.read_bytes() == b'preserve\n' and dangling.is_symlink(), 'existing output changed')
    receipt = native(args.binary, ['--executable-receipt'], out / 'executable.json')
    require(receipt['executable_sha256'] == digest(args.binary), 'executable receipt')
    for path, pin in receipt['compiled_sources'].items():
        require(digest(ROOT / path) == pin, 'compiled source')
    write(out / 'summary.json', dict(pass_checks=True, rejected_inputs=rejected, rejected_before_model_loading=18,
          model_size_rejection=True, executable_sha256=digest(args.binary), checker_sha256=digest(__file__),
          positive_model_free_preflight=True, missing_and_failed_events_without_model=True, existing_output_preserved=True))


def flow_optimum(edges):
    """Min-cost maximum flow; structurally independent of native Hungarian."""
    if not edges:
        return 0, 0.
    n = len(edges)
    d = max((index for row in edges for index, _ in row), default=-1) + 1
    sink = 1 + n + d
    graph = [[] for _ in range(sink + 1)]

    def add(source, target, cost):
        graph[source].append([target, len(graph[target]), 1, cost])
        graph[target].append([source, len(graph[source]) - 1, 0, -cost])

    for i, row in enumerate(edges):
        add(0, 1 + i, 0.)
        for j, score in row:
            add(1 + i, 1 + n + j, -score)
    for j in range(d):
        add(1 + n + j, sink, 0.)
    count, cost = 0, 0.
    while True:
        distance = [math.inf] * len(graph)
        parent = [None] * len(graph)
        distance[0] = 0.
        for _ in range(len(graph) - 1):
            changed = False
            for source, row in enumerate(graph):
                for index, (target, _, capacity, price) in enumerate(row):
                    if capacity and distance[source] + price < distance[target] - 1e-14:
                        distance[target] = distance[source] + price
                        parent[target] = source, index
                        changed = True
            if not changed:
                break
        if parent[sink] is None:
            return count, -cost
        node = sink
        while node:
            source, index = parent[node]
            edge = graph[source][index]
            edge[2] -= 1
            graph[node][edge[1]][2] += 1
            node = source
        count += 1
        cost += distance[sink]


def oracle():
    path = ROOT / 'integrations/camera-tracking/check.py'
    require(digest(path) == TRACK_CHECK_SHA, 'unchanged standalone tracker checker')
    spec = importlib.util.spec_from_file_location('sequence_tracking_oracle', path)
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    rng = random.Random(20261016)
    for _ in range(96):
        edges = [[(j, rng.randrange(1, 17) / 16.) for j in range(4) if rng.random() > .35] for _ in range(4)]
        count, score = support.optimum(edges)
        other_count, other_score = flow_optimum(edges)
        require(count == other_count and abs(score - other_score) < 1e-10, 'flow oracle versus exhaustive enumeration')
    # Keep the pinned independent lifecycle/motion reconstruction and use the
    # self-checked residual-flow solver instead of exponential enumeration.
    support.optimum = flow_optimum
    return support


def without_timing(value):
    if isinstance(value, dict):
        return {k: without_timing(v) for k, v in value.items() if k not in ('model_load_seconds', 'inference_seconds')}
    if isinstance(value, list):
        return list(map(without_timing, value))
    return value


def run(args):
    out = fresh(args.output)
    control = load(args.controls / 'summary.json')
    require(control['pass_checks'] and control['executable_sha256'] == digest(args.binary)
            and control['checker_sha256'] == digest(__file__), 'matching source-bound controls')
    source = load(args.sequence)
    expected_images = sum(event['input']['kind'] == 'image' for event in source['events'])
    require(expected_images == 8, 'fixed eight-image development recording')
    support = oracle()
    receipt = native(args.binary, ['--executable-receipt'], out / 'executable.json')
    write(out / 'before-inference.json', dict(source_sequence_sha256=digest(args.sequence),
        checker_sha256=digest(__file__), executable_sha256=digest(args.binary),
        compiled_sources=receipt['compiled_sources'], model_sha256=digest(args.model),
        data_role='previously viewed indoor RGB recording; no object identity ground truth'))
    native(args.binary, ['--inspect-sequence', args.sequence], out / 'inspection.json')
    passes = []
    for mode in ('first', 'repeat'):
        passes.append(native(args.binary, ['--model', args.model, '--sequence', args.sequence], out / (mode + '.json')))
    require(without_timing(passes[0]) == without_timing(passes[1]), 'native sequence repeat differs')
    exports = fresh(out / 'exports')
    old = fresh(out / 'single-image')
    rows = []
    for i, (input_event, event) in enumerate(zip(source['events'], passes[0]['events'])):
        image = args.sequence.parent / input_event['input']['path']
        prediction = native(args.single_binary, ['--model', args.model, '--image', image], old / f'{i}.json')
        require(without_timing(prediction) == without_timing(event['analysis']), 'unchanged single-image detector parity')
    for mode, report in zip(('first', 'repeat'), passes):
        require(report['source_sequence_sha256'] == digest(args.sequence) and report['compiled_sources'] == receipt['compiled_sources'], 'sequence provenance')
        require(report['execution'] == dict(model_loads=1, preflight_image_reads=8, inference_image_reads=8, model_forwards=32), 'reused model and bounded views')
        loads = {view['prediction']['model_load_seconds'] for e in report['events'] for view in e['analysis']['base_prediction']['views']}
        require(len(loads) == 1, 'all views share one measured plan load')
        frames, shadow = [], []
        for i, (input_event, event) in enumerate(zip(source['events'], report['events'])):
            require(event['timestamp_text'] == input_event['timestamp_s']
                    and event['timestamp_s'] == float(input_event['timestamp_s']), 'original recorded timestamp')
            path = exports / f'{mode}-{i}.json'
            write(path, event['analysis'])
            shadow.append(dict(timestamp_s=event['timestamp_s'], input=dict(kind='analysis', report=str(path.relative_to(out)), sha256=digest(path))))
            boxes = copy.deepcopy(event['analysis']['result']['detections'])
            import struct
            canonical = lambda value: float(struct.unpack('f', struct.pack('f', value))[0])
            for detection in boxes:
                detection['confidence'] = canonical(detection['confidence'])
                detection['bbox'] = list(map(canonical, detection['bbox']))
            frames.append(dict(timestamp_s=event['timestamp_s'], input=dict(kind='detections', image_sha256=event['analysis']['source_image_sha256'], detections=boxes)))
        shadow_path = out / (mode + '-tracking-input.json')
        write(shadow_path, dict(schema_version=1, camera_id=source['camera_id'], width=source['width'], height=source['height'],
                               config=report['config'], events=shadow))
        replay = native(args.tracker_binary, ['--sequence', shadow_path], out / (mode + '-tracking-replay.json'))
        actual = [event['tracking'] for event in report['events']]
        require(replay['snapshots'] == actual, 'standalone tracker replay differs')
        support.check_trace(frames, actual, report['config'])
        rows.append(dict(mode=mode, image_count=8, detections=[len(e['analysis']['result']['detections']) for e in report['events']],
                         confirmed_counts=[sum(t['phase'] == 'confirmed' for t in e['tracking']['tracks']) for e in report['events']],
                         retired_tracks=[e['tracking']['retired'] for e in report['events']]))
    for path, pin in receipt['compiled_sources'].items():
        require(digest(ROOT / path) == pin, 'compiled source changed during run')
    summary = dict(schema_version=1, pass_checks=True, source_sequence_sha256=digest(args.sequence),
        original_recorded_image_count=8, sequence_repeats=2, model_loads_per_sequence=1,
        native_sequence_forward_passes=64, old_single_image_parity_forward_passes=32,
        actual_model_forward_passes_this_check=96, raw_detections_repeat_identical=True,
        unchanged_single_image_parity=True, standalone_tracking_replay_identical=True,
        independent_residual_flow_and_lifecycle_checked=True, flow_oracle_exhaustive_graph_checks=96,
        real_object_identity_accuracy_measured=False, record=rows, subjective_maturity_percent=20)
    write(out / 'summary.json', summary)
    print(summary)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('controls', 'run'))
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--controls', type=Path)
    parser.add_argument('--sequence', type=Path)
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--single-binary', type=Path, default=ROOT / 'integrations/onnx/target/release/rustdriving-camera-analyze')
    parser.add_argument('--tracker-binary', type=Path, default=ROOT / 'integrations/camera-tracking/target/release/rustdriving-camera-track')
    args = parser.parse_args()
    args.binary = args.binary.absolute()
    args.output = args.output.absolute()
    if args.command == 'controls':
        controls(args)
    else:
        require(args.sequence is not None and args.controls is not None, 'sequence and controls required')
        args.sequence = args.sequence.absolute()
        run(args)
