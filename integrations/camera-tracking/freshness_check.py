#!/usr/bin/env python3
"""Two-clock contracts, injected faults on saved real reports, independent replay.

Uses only the standard library. Recorded faults/timing are authored controls,
not measured sensor failures. No images, weights, ground truth or inference.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
TRACK_CHECK_SHA = '575669cb4e62943cf211138c432969c08da4cd3ec5419b4326d8e7ae8f02ee08'
RECORDED_SEQUENCE_SHA = '5fa15877e7d910fe102dae175af40b5ee473c6f614b0740f36127b58b77f864c'
POLICY = dict(max_age_s=0.2, recovery_samples=2)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def load(path):
    require(path.stat().st_size <= 16 * 1024 * 1024, 'bounded JSON')
    return json.loads(path.read_text())


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def flow_optimum(edges):
    # Residual min-cost maximum flow, independent of native Hungarian.
    # Same solver used in the earlier camera_sequence_check.py; checked below
    # against the immutable exhaustive oracle on 96 graphs before each run.
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
    path = HERE / 'check.py'
    require(digest(path) == TRACK_CHECK_SHA, 'unchanged independent tracker oracle')
    spec = importlib.util.spec_from_file_location('freshness_tracking_oracle', path)
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    rng = random.Random(20261016)
    for _ in range(96):
        edges = [[(j, rng.randrange(1, 17) / 16.) for j in range(4) if rng.random() > .35] for _ in range(4)]
        count, score = support.optimum(edges)
        other_count, other_score = flow_optimum(edges)
        require(count == other_count and abs(score - other_score) < 1e-10, 'flow versus exhaustive assignment')
    support.optimum = flow_optimum
    return support


def sample(name, acquired, now=None, detections=None):
    return dict(now_s=acquired if now is None else now, input=dict(kind='detections', acquired_s=acquired,
        image_sha256=hashlib.sha256(name.encode()).hexdigest(),
        detections=[dict(class_index=2, confidence=0.8, bbox=[10. + acquired, 10., 30. + acquired, 30.])]
        if detections is None else detections))


def tick(now, kind='tick'):
    return dict(now_s=now, input=dict(kind=kind))


def synthetic_cases():
    a, b = sample('a', 0.), sample('b', .1)
    yield 'delayed-after-poll', {}, {}, [a, b, tick(.15), sample('c', .14, .18)]
    yield 'deadline-and-recovery', {}, {}, [a, b, tick(.31), tick(.6), sample('old', .2, .61),
        sample('new0', .62), sample('new1', .7)]
    yield 'fresh-after-gap', {}, {}, [a, b, sample('new0', .4, .41), sample('new1', .5, .51)]
    yield 'explicit-failure', {}, {}, [a, b, tick(.15, 'failed'), sample('old', .14, .16),
        sample('new0', .17), sample('new1', .18)]
    reused = sample('a', .05)
    yield 'rejected-breaks-recovery', {}, {}, [a, reused, b, sample('c', .15)]
    yield 'empty-images', {}, {}, [sample('empty0', 0., detections=[]), sample('empty1', .1, detections=[]), tick(.2)]
    yield 'per-object-age', {}, {}, [a, b, sample('empty0', .2, detections=[]),
        sample('empty1', .29, detections=[]), tick(.31)]
    yield 'short-object-coast', dict(coast_seconds=.05), {}, [a, sample('b-short', .03),
        sample('empty', .06, detections=[]), tick(.09)]
    for label, epoch in [('relative', 0.), ('epoch', 1_800_000_000.)]:
        second = epoch + .1
        deadline = second + .2
        yield 'inclusive-' + label, {}, {}, [sample(label+'a', epoch, detections=[]),
            sample(label+'b', second, detections=[]), tick(deadline), tick(deadline + 1e-5)]
    yield 'invalid-source-clocks', {}, {}, [sample('stale', 0., 1.), sample('negative', -1., 1.1),
        sample('future', 2., 1.2), sample('range', 1e13, 1.25), sample('ok0', 1.3, detections=[]),
        sample('reordered', 1.3, 1.31, detections=[]), sample('ok1', 1.4, 1.41, []), sample('ok2', 1.45, 1.46, [])]
    malformed = [dict(class_index=2, confidence=1.1, bbox=[10., 10., 30., 30.])]
    outside = [dict(class_index=2, confidence=.8, bbox=[-1., 10., 30., 30.])]
    wrong_class = [dict(class_index=80, confidence=.8, bbox=[10., 10., 30., 30.])]
    bad_hash = sample('bad-hash', .08)
    bad_hash['input']['image_sha256'] = 'INVALID'
    yield 'invalid-payloads', {}, {}, [a, sample('bad-score', .02, detections=malformed),
        sample('outside', .04, detections=outside), sample('wrong-class', .06, detections=wrong_class),
        bad_hash, b, sample('c', .15)]
    yield 'three-sample-recovery', {}, dict(recovery_samples=3), [a, tick(.03), b, sample('c', .15)]
    yield 'fault-before-first-image', {}, {}, [tick(0., 'failed'), tick(.1), sample('old', 0., .2),
        sample('ok0', .3), sample('ok1', .4)]
    rng = random.Random(20261016)
    for trial in range(32):
        events = [sample(f'r{trial}-0', 0.), sample(f'r{trial}-1', .05)]
        now = .05
        for index in range(2, 30):
            now = round(now + rng.choice([.01, .03, .07, .25]), 6)
            mode = rng.randrange(9)
            if mode < 2:
                events.append(tick(now, 'failed' if mode else 'tick'))
                continue
            source = round(now - rng.choice([0., .005, .04, .23]), 6)
            event = sample(f'r{trial}-{index}', source, now, [] if mode == 2 else
                [dict(class_index=2, confidence=.8, bbox=[10.+index, 10., 30.+index, 30.])])
            if mode == 3:
                event['input']['image_sha256'] = events[0]['input']['image_sha256']
            elif mode == 4:
                event['input']['acquired_s'] = now + .01
            elif mode == 5:
                event['input']['detections'][0]['confidence'] = 2.
            events.append(event)
        yield f'random-{trial:02}', dict(max_tracks=4, min_iou=.1), {}, events


def normalized(manifest, directory):
    """Independent conversion of the saved model's f32 measurements."""
    converted = copy.deepcopy(manifest['events'])
    receipts = []
    for index, event in enumerate(converted):
        data = event['input']
        if data['kind'] != 'analysis':
            continue
        path = directory / data['report']
        require(digest(path) == data['sha256'], 'source report hash')
        report = load(path)
        f32 = lambda value: struct.unpack('<f', struct.pack('<f', value))[0]
        detections = [dict(class_index=d['class_index'], confidence=f32(d['confidence']),
                           bbox=list(map(f32, d['bbox']))) for d in report['result']['detections']]
        event['input'] = dict(kind='detections', acquired_s=data['acquired_s'],
                             image_sha256=report['source_image_sha256'], detections=detections)
        receipts.append(dict(event_index=index, sha256=data['sha256']))
    return converted, receipts


def payload_ok(data, width, height, recent, backend_time):
    identity, detections = data['image_sha256'], data['detections']
    if (len(identity) != 64 or any(c not in '0123456789abcdef' for c in identity)
            or identity in recent or len(detections) > 300
            or backend_time is not None and data['acquired_s'] - backend_time < 1e-6):
        return False
    for d in detections:
        l, t, r, b = d['bbox']
        if (not 0 <= d['class_index'] < 80 or not 0 <= d['confidence'] <= 1
                or not all(map(math.isfinite, d['bbox']))
                or not 0 <= l < r <= width or not 0 <= t < b <= height):
            return False
    return True


def check_contract(manifest, events, results, support):
    """Reconstruct admission from the input ledger; replay motion separately.

    Polls are absent from backend time. Expected failure frames follow the last
    accepted acquisition deadline or the explicit failure, independently of
    whichever snapshots the executable returned.
    """
    policy = POLICY | manifest.get('policy', {})
    config = support.DEFAULT | manifest.get('config', {})
    accepted, deadline, barrier, backend_time = None, None, None, None
    hold, streak, recent, last_snapshot = 'initializing', 0, [], None
    backend_events, snapshots = [], []
    require(len(events) == len(results), 'one output per delivery/poll')
    for index, (event, result) in enumerate(zip(events, results)):
        now, data = event['now_s'], event['input']
        kind = data['kind']
        expected_updates = []
        if kind == 'failed' or deadline is not None and now > deadline:
            explicit = kind == 'failed'
            barrier = now if explicit else deadline
            hold = 'acquisition_failed' if explicit else 'deadline_exceeded'
            expected_updates.append(('acquisition_failure' if explicit else 'deadline_expiry',
                                     dict(timestamp_s=barrier, acquisition=dict(status='failed'))))
            backend_time, deadline, streak, last_snapshot = barrier, None, 0, None
        if kind == 'failed':
            decision = 'acquisition_failed'
        elif kind == 'tick':
            decision = 'polled'
        else:
            source = data['acquired_s']
            # This list defines the public precedence for multiple faults.
            invalid = [
                (not math.isfinite(source) or not 0 <= source <= 1e12, 'invalid_acquisition_time'),
                (source > now, 'future_sample'),
                (barrier is not None and source <= barrier, 'before_fault'),
                (accepted is not None and source - accepted < 1e-6, 'out_of_order'),
                (now > source + policy['max_age_s'], 'stale_sample'),
                (not payload_ok(data, manifest['width'], manifest['height'], recent, backend_time), 'invalid_payload')]
            decision = next((reason for condition, reason in invalid if condition), 'accepted')
            if decision == 'accepted':
                expected_updates.append(('sample', dict(timestamp_s=source, acquisition=dict(status='detected',
                    image_sha256=data['image_sha256'], detections=data['detections']))))
                backend_time = accepted = source
                deadline = source + policy['max_age_s']
                recent = (recent + [data['image_sha256']])[-256:]
                streak = min(streak + 1, policy['recovery_samples'])
                if streak == policy['recovery_samples']:
                    hold = None
            elif hold is not None:
                streak = 0
        require(result['decision'] == decision, f'input-derived decision at {index}')
        require(result['now_s'] == now and result['ready'] == (hold is None), 'delivery/readiness')
        require(result['hold_reason'] == hold and result['recovery_count'] == streak, 'hold/recovery count')
        require(result['fault_epoch_s'] == barrier and result['last_accepted_acquired_s'] == accepted, 'source clocks/barrier')
        require(result['result_age_s'] == (None if accepted is None else now - accepted), 'source age never renewed by a poll')
        updates = result['backend_updates']
        require(len(updates) == len(expected_updates), 'exact backend update count')
        for update, (trigger, frame) in zip(updates, expected_updates):
            require(update['trigger'] == trigger, 'update cause')
            support.same(frame, update['frame'])
            require(frame['timestamp_s'] == update['frame']['timestamp_s'], 'exact acquisition clock')
            acquisition = frame['acquisition']
            backend_events.append(dict(timestamp_s=frame['timestamp_s'], input=dict(
                kind='detections' if acquisition['status'] == 'detected' else 'failed',
                **{k: v for k, v in acquisition.items() if k != 'status'})))
            snapshots.append(update['snapshot'])
            last_snapshot = update['snapshot'] if trigger == 'sample' else None
        expected_tracks = []
        if hold is None and last_snapshot is not None:
            for track in last_snapshot['tracks']:
                age = now - track['last_observed_timestamp_s']
                if (track['phase'] == 'tentative' or
                    now > track['last_observed_timestamp_s'] + min(policy['max_age_s'], config['coast_seconds'])):
                    continue
                expected_tracks.append(dict(track_id=track['track_id'], class_index=track['class_index'],
                    bbox_at_now=[v + track['velocity_px_s'][a % 2] * age for a, v in enumerate(track['last_observed_bbox'])],
                    velocity_px_s=track['velocity_px_s'], observed_at_s=track['last_observed_timestamp_s'],
                    observation_age_s=age, last_observed_confidence=track['last_observed_confidence'], extrapolated=age > 0.))
        support.same(expected_tracks, result['tracks'])
    support.check_trace(backend_events, snapshots, manifest.get('config', {}))
    return backend_events, snapshots


def golden(name, results):
    """Small, hand-authored expectations complement the ledger reconstruction."""
    ready = [r['ready'] for r in results]
    if name == 'delayed-after-poll':
        require(ready == [False, True, True, True], 'delayed frame remains usable after a poll')
        require(results[-1]['decision'] == 'accepted' and results[-1]['tracks'][0]['track_id'] == 1, 'delayed frame identity')
        require(not results[2]['backend_updates'], 'poll cannot advance backend clock')
    elif name == 'deadline-and-recovery':
        require(ready == [False, True, False, False, False, False, True], 'two fresh frames after deadline')
        require(results[-1]['tracks'][0]['track_id'] == 2, 'pre-expiry ID cannot return')
    elif name == 'fresh-after-gap':
        require(ready == [False, True, False, True] and len(results[2]['backend_updates']) == 2, 'fault before fresh arrival')
    elif name == 'explicit-failure':
        require(ready == [False, True, False, False, False, True], 'failure then consecutive fresh recovery')
        require(results[-1]['tracks'][0]['track_id'] == 2, 'pre-failure ID cannot return')
    elif name == 'rejected-breaks-recovery':
        require(ready == [False, False, False, True] and results[1]['recovery_count'] == 0, 'rejected delivery breaks streak')
    elif name == 'empty-images':
        require(ready == [False, True, True] and not any(r['tracks'] for r in results), 'negative images can be fresh')
    elif name in ('per-object-age', 'short-object-coast'):
        require(results[-1]['ready'] and not results[-1]['tracks'] and results[-2]['tracks'], 'per-object age independent of image age')
    elif name.startswith('inclusive-'):
        require(ready == [False, True, True, False], 'inclusive deadline and first tick beyond it')
    elif name == 'three-sample-recovery':
        require(ready == [False, False, False, True], 'configured third fresh image required')


def recorded_cases(out):
    original = ROOT / 'assets/camera-sequence-v1'
    require(digest(original / 'sequence.json') == RECORDED_SEQUENCE_SHA, 'unchanged original eight-frame selection')
    sequence = load(original / 'sequence.json')
    observed = load(original / 'recorded/first.json')['events']
    directory = out / 'reports'
    directory.mkdir()
    images = []
    for index, (source, saved) in enumerate(zip(sequence['events'], observed)):
        path = directory / f'{index}.json'
        shutil.copyfile(original / f'recorded/exports/first-{index}.json', path)
        require(saved['timestamp_s'] == float(source['timestamp_s']), 'original epoch acquisition preserved')
        require(load(path)['source_image_sha256'] == source['input']['sha256'], 'original RGB identity')
        images.append(dict(kind='analysis', acquired_s=float(source['timestamp_s']),
                           report=f'../reports/{index}.json', sha256=digest(path)))
    times = [data['acquired_s'] for data in images]
    write(out / 'recorded-design.json', dict(source_sequence_sha256=RECORDED_SEQUENCE_SHA,
        original_indices=list(range(102, 110)), acquired_s=[e['timestamp_s'] for e in sequence['events']],
        report_sha256=[image['sha256'] for image in images],
        data_role='Previously viewed eight-frame indoor development slice; saved detections reused unchanged',
        injected='All delivery times, ticks, dropped deliveries and explicit faults below are authored controls',
        real_object_identity_ground_truth=False, model_inference_this_check=False))
    event = lambda index, now: dict(now_s=now, input=copy.deepcopy(images[index]))
    yield 'recorded-healthy', [event(i, time + .005) for i, time in enumerate(times)]
    delayed = [event(0, times[0] + .005), tick(times[1] + .005), event(1, times[1] + .01)]
    delayed += [event(i, times[i] + .005) for i in range(2, 8)]
    yield 'recorded-delayed-after-poll', delayed
    failed = [event(i, times[i] + .005) for i in range(4)]
    failed += [tick(times[3] + .01, 'failed'), event(3, times[3] + .015)]
    failed += [event(i, times[i] + .005) for i in range(4, 8)]
    yield 'recorded-explicit-fault-recovery', failed
    deadline = times[1] + POLICY['max_age_s']
    expired = [event(0, times[0] + .005), event(1, times[1] + .005), tick(deadline + .00001),
               tick(deadline + .00002), event(2, deadline + .00003), event(7, times[7] + .00005)]
    yield 'recorded-deadline-still-held', expired


def run(args):
    out = args.output.absolute()
    require(not out.exists() and not out.is_symlink(), 'fresh evidence directory')
    out.mkdir(parents=True)
    cases = out / 'cases'
    cases.mkdir()
    failures = out / 'failures'
    failures.mkdir()
    support = oracle()
    source_pins = {str(p.relative_to(ROOT)): digest(p) for p in [
        HERE / 'Cargo.toml', HERE / 'Cargo.lock', ROOT / 'rust-toolchain.toml', HERE / 'producer-sources.json',
        HERE / 'profile-freshness.json', *sorted((HERE / 'src').rglob('*.rs')),
        *sorted((HERE / 'tests').rglob('*.rs')), HERE / 'check.py', Path(__file__)]}
    write(out / 'before.json', dict(source_sha256=source_pins, binary_sha256=digest(args.binary),
        tracker_binary_sha256=digest(args.tracker), faults_and_receive_times_are_injected=True))
    runs, replay_runs, records = 0, 0, []

    def native(binary, manifest, destination, expected=0, error=None):
        nonlocal runs, replay_runs
        process = subprocess.run([str(binary), '--sequence', str(manifest), '--output', str(destination)],
                                 capture_output=True, text=True, timeout=30)
        if binary == args.binary:
            runs += 1
        else:
            replay_runs += 1
        require(process.returncode == expected, 'native exit: ' + process.stderr)
        require(not list(destination.parent.glob('.rustdriving-guard-*.partial')), 'guard staging leaked')
        if error:
            require(error in process.stderr, 'failure stage: ' + process.stderr)
        if expected:
            require(not destination.exists(), 'failure published partial output')
            return dict(exit=process.returncode, stderr=process.stderr)
        return load(destination)

    def check_case(name, config, policy, events, width=100, height=100):
        manifest = dict(schema_version=1, camera_id='freshness-controls', width=width, height=height,
                        config=config, policy=policy, events=events)
        path = cases / (name + '-input.json')
        write(path, manifest)
        first = native(args.binary, path, cases / (name + '-first.json'))
        repeat = native(args.binary, path, cases / (name + '-repeat.json'))
        require(first == repeat, 'byte-equivalent deterministic JSON values')
        require(first['algorithm'] == 'image_tracking_freshness_v1' and first['schema_version'] == 1, 'result schema')
        require(first['source_sequence_sha256'] == digest(path), 'sequence receipt')
        require(first['config'] == support.DEFAULT | config and first['policy'] == POLICY | policy, 'effective config')
        require(first['control_permission'] is False and not any(first[k] for k in ('model_read', 'images_read', 'inference_run')), 'offline only')
        for filename, pin in first['compiled_sources'].items():
            require(pin == digest(ROOT / filename), 'current compiled source receipt')
        converted, receipts = normalized(manifest, path.parent)
        require(first['source_reports'] == receipts and first['inline_detection_events'] == sum(e['input']['kind'] == 'detections' for e in events), 'input lineage')
        backend_events, snapshots = check_contract(manifest, converted, first['results'], support)
        golden(name, first['results'])
        if backend_events:
            replay = dict(schema_version=1, camera_id='freshness-controls', width=width, height=height,
                          config=config, events=backend_events)
            replay_path = cases / (name + '-backend-input.json')
            write(replay_path, replay)
            result = native(args.tracker, replay_path, cases / (name + '-backend-result.json'))
            require(result['snapshots'] == snapshots, 'exact standalone backend replay')
        if name in ('recorded-healthy', 'recorded-delayed-after-poll'):
            original = [event['tracking'] for event in load(ROOT / 'assets/camera-sequence-v1/recorded/first.json')['events']]
            require(snapshots == original, 'unchanged recorded acquisition/association history')
        if name == 'recorded-explicit-fault-recovery':
            results = first['results']
            require([r['ready'] for r in results] == [False, True, True, True, False, False, False, True, True, True], 'real reports recover after two post-fault acquisitions')
            old_ids = {t['track_id'] for t in results[3]['tracks']}
            require(old_ids.isdisjoint(t['track_id'] for t in results[7]['tracks']), 'real reports cannot resurrect old IDs')
        if name == 'recorded-deadline-still-held':
            require(not first['results'][-1]['ready'] and first['results'][-1]['recovery_count'] == 1, 'one remaining real frame cannot satisfy two-frame recovery')
            require(first['results'][4]['decision'] == 'before_fault', 'late queued report excluded')
        records.append(dict(name=name, events=len(events), backend_updates=len(snapshots),
            accepted=sum(r['decision'] == 'accepted' for r in first['results']),
            held=sum(not r['ready'] for r in first['results']),
            final_ready=first['results'][-1]['ready'], repeat_identical=True, standalone_replay_identical=True))

    for name, config, policy, events in synthetic_cases():
        check_case(name, config, policy, events)
    synthetic_count = len(records)
    for name, events in recorded_cases(out):
        check_case(name, {}, {}, events, 640, 480)

    # Transport/schema failures abort the whole batch even after a valid event.
    valid = dict(schema_version=1, camera_id='failure-controls', width=100, height=100,
                 events=[sample('first', 0.), sample('second', .1)])
    operations = {
        'delivery-reversal': lambda s: s['events'][1].__setitem__('now_s', -.1),
        'delivery-duplicate': lambda s: s['events'][1].__setitem__('now_s', 0.),
        'delivery-too-close': lambda s: s['events'][1].__setitem__('now_s', .0000001),
        'delivery-range': lambda s: s['events'][1].__setitem__('now_s', 1e13),
        'delivery-type': lambda s: s['events'][1].__setitem__('now_s', 'NaN'),
        'delivery-null': lambda s: s['events'][1].__setitem__('now_s', None),
        'bad-schema': lambda s: s.__setitem__('schema_version', 2),
        'empty-events': lambda s: s.__setitem__('events', []),
        'too-many-events': lambda s: s.__setitem__('events', s['events'] * 65),
        'bad-camera-id': lambda s: s.__setitem__('camera_id', 'bad camera'),
        'bad-dimensions': lambda s: s.__setitem__('width', 0),
        'bad-policy-age': lambda s: s.__setitem__('policy', dict(max_age_s=.0001)),
        'bad-policy-count': lambda s: s.__setitem__('policy', dict(recovery_samples=0)),
        'unknown-policy': lambda s: s.__setitem__('policy', dict(unexpected=True)),
        'bad-config': lambda s: s.__setitem__('config', dict(max_tracks=301)),
        'unknown-root': lambda s: s.__setitem__('unexpected', True),
        'unknown-input': lambda s: s['events'][1]['input'].__setitem__('unexpected', True),
    }
    rejections = []
    for name, change in operations.items():
        value = copy.deepcopy(valid)
        change(value)
        path = failures / (name + '-input.json')
        write(path, value)
        rejections.append(dict(name=name, **native(args.binary, path, failures / (name + '-unexpected.json'), 2)))
    report = out / 'reports/0.json'
    invalid_report = failures / 'corrupt-report.json'
    invalid_report.write_bytes(b'not JSON\n')
    wrong_profile = load(report)
    wrong_profile['compiled_sources'] = {}
    write(failures / 'wrong-profile.json', wrong_profile)
    symlink = failures / 'report-link.json'
    symlink.symlink_to('../reports/0.json')
    fifo = failures / 'report-fifo.json'
    os.mkfifo(fifo)
    oversized = failures / 'oversized-report.json'
    with oversized.open('xb') as stream:
        stream.truncate(4 * 1024 * 1024 + 1)
    for name, relative, sha in [
        ('report-hash', '../reports/0.json', '0' * 64),
        ('invalid-report-hash', '../reports/0.json', 'BAD'),
        ('corrupt-report', 'corrupt-report.json', digest(invalid_report)),
        ('wrong-profile', 'wrong-profile.json', digest(failures / 'wrong-profile.json')),
        ('report-symlink', symlink.name, digest(report)),
        ('report-fifo', fifo.name, digest(report)),
        ('report-byte-budget', oversized.name, digest(oversized)),
        ('missing-report', 'missing.json', digest(report)),
        ('wrong-report-dimensions', '../reports/0.json', digest(report)),
    ]:
        value = copy.deepcopy(valid)
        value.update(width=640, height=480)
        if name == 'wrong-report-dimensions':
            value['width'] = 639
        value['events'][1]['input'] = dict(kind='analysis', acquired_s=.1, report=relative, sha256=sha)
        path = failures / (name + '-input.json')
        write(path, value)
        rejections.append(dict(name=name, **native(args.binary, path, failures / (name + '-unexpected.json'), 2)))
    oversized_sequence = failures / 'oversized-sequence.json'
    with oversized_sequence.open('xb') as stream:
        stream.truncate(8 * 1024 * 1024 + 1)
    rejections.append(dict(name='sequence-byte-budget', **native(args.binary, oversized_sequence, failures / 'sequence-budget-unexpected.json', 2, 'bounded JSON')))
    for name, content in [('malformed-json', b'{'), ('nonfinite-number', b'{"now_s":1e9999}')]:
        path = failures / (name + '-input.json')
        path.write_bytes(content)
        rejections.append(dict(name=name, **native(args.binary, path, failures / (name + '-unexpected.json'), 2)))
    boxes = [dict(class_index=2, confidence=.8, bbox=[float(i % 10 * 30), float(i // 10 * 30),
              float(i % 10 * 30 + 20), float(i // 10 * 30 + 20)]) for i in range(100)]
    report_budget = dict(schema_version=1, camera_id='report-budget', width=1000, height=1000,
        events=[sample(f'budget-{i}', i * .01, detections=boxes) for i in range(128)])
    budget_path = failures / 'output-budget-input.json'
    with budget_path.open('x') as stream:
        json.dump(report_budget, stream, separators=(',', ':'), allow_nan=False)
    require(budget_path.stat().st_size < 8 * 1024 * 1024, 'output budget uses an admissible input')
    rejections.append(dict(name='output-byte-budget', **native(args.binary, budget_path,
        failures / 'output-budget-unexpected.json', 2, 'report byte budget exceeded')))
    existing = failures / 'existing.json'
    existing.write_bytes(b'preserve\n')
    dangling = failures / 'dangling-output.json'
    dangling.symlink_to('absent.json')
    output_rejections = []
    for path in (existing, dangling):
        process = subprocess.run([str(args.binary), '--sequence', 'missing-source', '--output', str(path)],
                                 capture_output=True, text=True, timeout=30)
        runs += 1
        require(process.returncode == 2 and 'fresh output required before input reads' in process.stderr, 'output checked before inputs')
        output_rejections.append(dict(name=path.name, exit=process.returncode, stderr=process.stderr))
    require(existing.read_bytes() == b'preserve\n' and dangling.is_symlink(), 'existing outputs preserved')
    special = dict(symlinks={symlink.name:os.readlink(symlink), dangling.name:os.readlink(dangling)},
                   fifo=fifo.name, sparse_files={p.name:p.stat().st_size for p in (oversized, oversized_sequence)})
    write(failures / 'special-files.json', special)
    for path in (symlink, dangling, fifo, oversized, oversized_sequence):
        path.unlink()
    for filename, pin in source_pins.items():
        require(pin == digest(ROOT / filename), 'sources unchanged during verification')
    write(out / 'summary.json', dict(pass_checks=True, synthetic_sequences=synthetic_count,
        synthetic_events=sum(r['events'] for r in records[:synthetic_count]), recorded_control_sequences=len(records)-synthetic_count,
        recorded_unique_images=8, recorded_model_forwards_this_check=0, actual_sensor_failures_measured=False,
        all_delivery_times_and_faults_injected=True, real_object_identity_accuracy_measured=False,
        exact_repeated_runs=True, independent_motion_and_assignment_verified=True, flow_exhaustive_selfchecks=96,
        exact_standalone_backend_replay=True, guard_process_runs=runs, tracker_replay_process_runs=replay_runs,
        transport_rejections=rejections, output_rejections=output_rejections, cases=records,
        binary_sha256=digest(args.binary), tracker_binary_sha256=digest(args.tracker), source_sha256=source_pins))
    print(json.dumps(dict(pass_checks=True, synthetic_sequences=synthetic_count, recorded_control_sequences=4,
        transport_rejections=len(rejections), output_rejections=len(output_rejections), guard_runs=runs)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=HERE / 'target/release/rustdriving-camera-guard')
    parser.add_argument('--tracker', type=Path, default=HERE / 'target/release/rustdriving-camera-track')
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    arguments.binary = arguments.binary.absolute()
    arguments.tracker = arguments.tracker.absolute()
    run(arguments)
