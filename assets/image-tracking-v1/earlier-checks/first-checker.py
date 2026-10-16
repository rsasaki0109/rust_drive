#!/usr/bin/env python3
"""Independent exhaustive-assignment and lifecycle checks; no MOT accuracy claim."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
DEFAULT = dict(min_iou=0.3, coast_seconds=0.5, confirmation_hits=2,
               velocity_gain=0.5, max_tracks=300)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path):
    require(path.stat().st_size <= 16 * 1024 * 1024, 'bounded result')
    return json.loads(path.read_text())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def same(expected, actual, location='$'):
    if isinstance(expected, dict):
        require(type(actual) is dict and expected.keys() == actual.keys(), 'keys: ' + location)
        for key, value in expected.items():
            same(value, actual[key], location + '.' + key)
    elif isinstance(expected, list):
        require(type(actual) is list and len(expected) == len(actual), 'length: ' + location)
        for index, (a, b) in enumerate(zip(expected, actual)):
            same(a, b, location + f'[{index}]')
    elif type(expected) is float:
        require(type(actual) in (int, float) and math.isfinite(actual)
                and math.isclose(expected, actual, abs_tol=1e-8, rel_tol=1e-12), 'measurement: ' + location)
    else:
        require(type(expected) is type(actual) and expected == actual, 'identity: ' + location)


def iou(a, b):
    area = lambda box: (box[2] - box[0]) * (box[3] - box[1])
    shared = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return shared / (area(a) + area(b) - shared)


def optimum(edges):
    # Exhaustive search, independent of the native Hungarian implementation.
    def search(row, used, count, total):
        if row == len(edges):
            return count, total
        best = search(row + 1, used, count, total)
        for index, score in edges[row]:
            if index not in used:
                best = max(best, search(row + 1, used | {index}, count + 1, total + score))
        return best
    return search(0, set(), 0, 0.)


def check_trace(events, snapshots, overrides):
    config = DEFAULT | overrides
    state, next_id, comparisons = {}, 1, 0
    require(len(events) == len(snapshots), 'all events retained')
    for sequence, (event, actual) in enumerate(zip(events, snapshots), 1):
        timestamp, acquisition = event['timestamp_s'], event['input']
        kind = acquisition['kind']
        retired = []
        image = acquisition.get('image_sha256')
        if kind == 'failed':
            retired = [dict(track_id=key, reason='acquisition_failed') for key in state]
            state = {}
        else:
            expired = [key for key, track in state.items() if timestamp - track['last_seen'] > config['coast_seconds']]
            for key in expired:
                del state[key]
                retired.append(dict(track_id=key, reason='expired'))
        detections = acquisition.get('detections', [])
        ids = list(state)
        edges = []
        for key in ids:
            track = state[key]
            age = timestamp - track['last_seen']
            box = [value + track['velocity'][axis % 2] * age for axis, value in enumerate(track['observed']['bbox'])]
            edges.append([(i, iou(box, d['bbox'])) for i, d in enumerate(detections)
                          if d['class_index'] == track['observed']['class_index'] and iou(box, d['bbox']) >= config['min_iou']])
        claimed = {track['track_id']: track['detection_index'] for track in actual['tracks']}
        matches = [claimed.get(key) for key in ids]
        used = [i for i in matches if i is not None]
        require(len(used) == len(set(used)), 'one-to-one assignment')
        total = 0.
        for eligible, index in zip(edges, matches):
            if index is not None:
                require(index in dict(eligible), 'class/IoU association gate')
                total += dict(eligible)[index]
        best_count, best_iou = optimum(edges)
        require(len(used) == best_count and abs(total - best_iou) <= 1e-10, 'exhaustive maximum-cardinality/IoU optimum')
        comparisons += 1
        indices = {}
        for key, index in zip(ids, matches):
            track = state[key]
            if index is None:
                if not track['confirmed']:
                    del state[key]
                    retired.append(dict(track_id=key, reason='tentative_miss'))
                    continue
            else:
                old, new = track['observed']['bbox'], detections[index]['bbox']
                dt = timestamp - track['last_seen']
                measured = [((new[a] + new[a + 2]) - (old[a] + old[a + 2])) / (2 * dt) for a in range(2)]
                gain = 1. if track['hits'] == 1 else config['velocity_gain']
                track['velocity'] = [(1 - gain) * a + gain * b for a, b in zip(track['velocity'], measured)]
                track['observed'] = copy.deepcopy(detections[index])
                track['last_seen'] = timestamp
                track['hits'] += 1
                track['confirmed'] |= track['hits'] >= config['confirmation_hits']
            indices[key] = index
        dropped = []
        for index in sorted(set(range(len(detections))) - set(used), key=lambda i: (-detections[i]['confidence'], i)):
            if len(state) == config['max_tracks']:
                dropped.append(index)
                continue
            state[next_id] = dict(observed=copy.deepcopy(detections[index]), last_seen=timestamp,
                                  velocity=[0., 0.], hits=1, confirmed=config['confirmation_hits'] == 1)
            indices[next_id] = index
            next_id += 1
        tracks = []
        for key, track in state.items():
            observed, velocity = track['observed'], track['velocity']
            age = timestamp - track['last_seen']
            phase = 'tentative' if not track['confirmed'] else 'confirmed' if indices[key] is not None else 'coasting'
            tracks.append(dict(track_id=key, class_index=observed['class_index'], phase=phase,
                bbox=[v + velocity[a % 2] * age for a, v in enumerate(observed['bbox'])], velocity_px_s=velocity,
                last_observed_bbox=observed['bbox'], last_observed_timestamp_s=track['last_seen'],
                last_observed_confidence=observed['confidence'], observation_count=track['hits'], detection_index=indices[key]))
        expected = dict(sequence_number=sequence, timestamp_s=timestamp,
                        health='acquisition_failed' if kind == 'failed' else 'observed' if kind == 'detections' else 'missing',
                        source_image_sha256=image, tracks=tracks, retired=sorted(retired, key=lambda v: v['track_id']),
                        untracked_detection_indices=sorted(dropped))
        same(expected, {key: value for key, value in actual.items() if key != 'assignment_work'})
        require(type(actual['assignment_work']) is int and 0 <= actual['assignment_work'] <= 60_000_000, 'work budget')
    return comparisons


def detected(name, time, boxes):
    return dict(timestamp_s=time, input=dict(kind='detections', image_sha256=hashlib.sha256(name.encode()).hexdigest(), detections=boxes))


def box(left, top=10., size=20., score=0.9, kind=2):
    return dict(class_index=kind, confidence=score, bbox=[left, top, left + size, top + size])


def cases():
    yield 'global-greedy-trap', {}, [detected('g0', 0., [box(10., size=10.), box(14., size=10., score=0.8)]),
                                    detected('g1', 0.125, [box(11., size=10.), box(7., size=10., score=0.8)])]
    crossing = [detected(f'c{i}', i * 0.125, [box(20. + i * 5.), box(70. - i * 5., score=0.8)]) for i in range(3)]
    crossing += [dict(timestamp_s=0.375, input=dict(kind='missing')),
                 detected('c4', 0.5, [box(50., score=0.8), box(40.)]),
                 dict(timestamp_s=0.625, input=dict(kind='missing')),
                 detected('c6', 0.75, [box(40., score=0.8), box(50.)])]
    yield 'crossing-with-occlusion', {}, crossing
    yield 'expiry-failure-recovery', {}, [detected('e0', 0., [box(10.)]), detected('e1', 0.25, [box(10.)]),
        dict(timestamp_s=0.75, input=dict(kind='missing')), dict(timestamp_s=0.875, input=dict(kind='missing')),
        detected('e2', 1., [box(10.)]), dict(timestamp_s=1.125, input=dict(kind='failed')),
        detected('e3', 1.25, [box(10.)])]
    yield 'capacity-and-tentative', dict(max_tracks=2, confirmation_hits=3), [
        detected('a0', 0., [box(0., score=0.8), box(30., score=0.7), box(60.)]),
        detected('a1', 0.125, []), detected('a2', 0.25, [box(10.)])]
    rng = random.Random(20261016)
    for trial in range(48):
        events = []
        for step in range(6):
            detections = [box(float(rng.randrange(0, 65, 4)), float(rng.randrange(0, 65, 4)),
                              float(rng.choice([16, 20, 24])), (6 + index) / 10., rng.choice([0, 2, 7]))
                          for index in range(rng.randrange(5))]
            event = detected(f'random-{trial}-{step}', step * 0.125, detections)
            if rng.random() < 0.15:
                event['input'] = dict(kind='missing' if step % 2 else 'failed')
            events.append(event)
        yield f'random-{trial:02}', dict(max_tracks=4, confirmation_hits=1, min_iou=0.1), events


def run(args):
    out = args.output.absolute()
    require(not out.exists() and not out.is_symlink(), 'fresh evidence directory')
    out.mkdir(parents=True)
    compiled = [HERE / 'Cargo.toml', HERE / 'Cargo.lock', ROOT / 'rust-toolchain.toml', HERE / 'producer-sources.json',
                *sorted((HERE / 'src').rglob('*.rs')), *sorted((HERE / 'tests').rglob('*.rs')), Path(__file__)]
    source_pins = {str(p.relative_to(ROOT)): digest(p) for p in compiled}
    write(out / 'before.json', dict(source_sha256=source_pins, binary_sha256=digest(args.binary),
                                   data_role='synthetic temporal controls plus isolated viewed image-report admission; no real MOT score'))
    native_runs = 0

    def invoke(manifest, destination, expected=0, error=None):
        nonlocal native_runs
        result = subprocess.run([str(args.binary), '--sequence', str(manifest), '--output', str(destination)],
                                capture_output=True, text=True, timeout=30)
        native_runs += 1
        require(result.returncode == expected, 'native exit: ' + result.stderr)
        if error:
            require(error in result.stderr, 'failure stage: ' + result.stderr)
        require(not list(out.glob('.rustdriving-tracks-*.partial')), 'staging leak')
        if expected:
            require(not destination.exists(), 'failure published output')
            return dict(exit=result.returncode, stderr=result.stderr)
        body = load(destination)
        require(body['source_sequence_sha256'] == digest(manifest), 'sequence byte identity')
        require(body['model_read'] is False and body['images_read'] is False and body['inference_run'] is False, 'report-only execution')
        for path, pin in body['compiled_sources'].items():
            require(digest(ROOT / path) == pin, 'compiled source')
        return body

    rows, frames = [], 0
    for name, config, events in cases():
        path = out / (name + '-input.json')
        write(path, dict(schema_version=1, camera_id='controls', width=100, height=100, config=config, events=events))
        first = invoke(path, out / (name + '-first.json'))
        repeat = invoke(path, out / (name + '-repeat.json'))
        require(first == repeat, 'repeat must be exactly deterministic')
        frames += check_trace(events, first['snapshots'], config)
        if name == 'global-greedy-trap':
            require([(t['track_id'], t['detection_index']) for t in first['snapshots'][-1]['tracks']] == [(1, 1), (2, 0)], 'greedy counterexample')
        if name == 'crossing-with-occlusion':
            require([(t['track_id'], t['detection_index']) for t in first['snapshots'][-1]['tracks']] == [(1, 1), (2, 0)], 'authored crossing IDs')
        rows.append(dict(name=name, frames=len(events), pass_checks=True))

    real = []
    parents = ROOT / 'assets/camera-analysis-v1/publication-repair/regression'
    for mode in ('first', 'repeat'):
        for index, source in enumerate(sorted((parents / mode).glob('*.json'))):
            parent = load(source)
            name = f'admit-{mode}-{index}'
            path = out / (name + '-input.json')
            event = dict(timestamp_s=0., input=dict(kind='analysis', report=str(source), sha256=digest(source)))
            write(path, dict(schema_version=1, camera_id='isolated-admission', width=1280, height=720, events=[event]))
            actual = invoke(path, out / (name + '-actual.json'))
            canonical = lambda v: float(struct.unpack('f', struct.pack('f', v))[0])
            boxes = copy.deepcopy(parent['result']['detections'])
            for detection in boxes:
                detection['confidence'] = canonical(detection['confidence'])
                detection['bbox'] = list(map(canonical, detection['bbox']))
            check_trace([detected('unused', 0., boxes) | {'input': dict(kind='detections', image_sha256=parent['source_image_sha256'], detections=boxes)}], actual['snapshots'], {})
            real.append(dict(source=str(source.relative_to(ROOT)), sha256=digest(source), admitted=len(boxes)))
    require(len(real) == 16, 'all eight first/repeat native image reports admitted separately')

    rejected = []
    base = dict(schema_version=1, camera_id='reject-controls', width=100, height=100,
                events=[detected('negative0', 0., [box(10.)]), detected('negative1', 0.125, [box(11.)])])
    operations = {
        'clock-reversal': lambda s: s['events'][1].__setitem__('timestamp_s', -0.1),
        'duplicate-clock': lambda s: s['events'][1].__setitem__('timestamp_s', 0.),
        'reused-image': lambda s: s['events'][1]['input'].__setitem__('image_sha256', s['events'][0]['input']['image_sha256']),
        'bad-box': lambda s: s['events'][1]['input']['detections'][0].__setitem__('bbox', [0, 0, 101, 10]),
        'bad-class': lambda s: s['events'][1]['input']['detections'][0].__setitem__('class_index', 80),
        'bad-score': lambda s: s['events'][1]['input']['detections'][0].__setitem__('confidence', 1.1),
        'too-many-detections': lambda s: s['events'][1]['input'].__setitem__('detections', [box(10.)] * 301),
        'too-many-events': lambda s: s.__setitem__('events', s['events'] * 65),
        'bad-config': lambda s: s.__setitem__('config', dict(max_tracks=301)),
        'unknown-field': lambda s: s['events'][1]['input'].__setitem__('surprise', True),
    }
    for name, change in operations.items():
        value = copy.deepcopy(base)
        change(value)
        path = out / (name + '-bad.json')
        write(path, value)
        rejected.append(dict(name=name, **invoke(path, out / (name + '-unexpected.json'), 2)))
    sample = parents / 'first/00a395fe-d60c0b47.json'
    producer = load(sample)
    mutations = {
        'producer-profile': lambda p: p.__setitem__('compiled_sources', {}),
        'producer-model': lambda p: p['base_prediction'].__setitem__('model_sha256', '0' * 64),
        'producer-dimensions': lambda p: p['base_prediction'].__setitem__('width', 640),
        'producer-image': lambda p: p.__setitem__('source_image_sha256', 'f' * 64),
        'producer-execution': lambda p: p['execution'].__setitem__('inference_run', False),
        'retained-rank': lambda p: p['result']['retained_base_ranks'].__setitem__(0, 999),
        'retained-box': lambda p: p['result']['detections'][0]['bbox'].__setitem__(0, 0.),
        'retained-origin': lambda p: p['result']['detection_origins'][0].__setitem__('rank', 999),
    }
    for name, change in mutations.items():
        mutant = copy.deepcopy(producer)
        change(mutant)
        report = out / (name + '-report.json')
        write(report, mutant)
        path = out / (name + '-input.json')
        write(path, dict(schema_version=1, camera_id='corruption', width=1280, height=720,
                         events=[dict(timestamp_s=0., input=dict(kind='analysis', report=str(report), sha256=digest(report)))]))
        rejected.append(dict(name=name, **invoke(path, out / (name + '-unexpected.json'), 2, 'invalid pinned')))
    wrong_hash = out / 'report-hash-input.json'
    write(wrong_hash, dict(schema_version=1, camera_id='checksum', width=1280, height=720,
        events=[dict(timestamp_s=0., input=dict(kind='analysis', report=str(sample), sha256='0' * 64))]))
    rejected.append(dict(name='report-hash', **invoke(wrong_hash, out / 'report-hash-unexpected.json', 2, 'SHA-256 mismatch')))
    require(all(digest(ROOT / path) == pin for path, pin in source_pins.items()), 'sources changed during check')
    summary = dict(schema_version=1, pass_checks=True, controls=rows, independently_checked_temporal_frames=frames,
                   exhaustive_assignment_oracle=True, repeat_identical=True, native_invocations=native_runs,
                   native_model_forward_passes=0, isolated_real_report_admissions=real, rejected_inputs=rejected,
                   real_sequence_identity_accuracy_measured=False, subjectively_estimated_maturity_percent=20)
    write(out / 'summary.json', summary)
    print({key: value for key, value in summary.items() if key not in ('controls', 'isolated_real_report_admissions', 'rejected_inputs')})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.binary = args.binary.absolute()
    run(args)
