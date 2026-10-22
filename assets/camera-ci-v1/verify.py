#!/usr/bin/env python3
"""Check the preserved confidence-portability study without model execution."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def raw(name):
    path = HERE / name
    if not path.exists():
        path = HERE / (name + '.gz')
    data = path.read_bytes()
    return gzip.decompress(data) if path.suffix == '.gz' else data


def read(name):
    return json.loads(raw(name))


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    files = read('files.json')
    actual = {str(p.relative_to(HERE)) for p in HERE.rglob('*')
              if p.is_file() and p.name != 'files.json' and '__pycache__' not in p.parts}
    assert actual == files.keys(), 'complete packet inventory'
    for name, pin in files.items():
        data = (HERE / name).read_bytes()
        assert len(data) == pin['bytes'] and digest(data) == pin['sha256'], name
    for name, pin in read('raw-files.json').items():
        data = raw(name)
        assert len(data) == pin['original_bytes'] and digest(data) == pin['original_sha256'], name
    for name, pin in read('source-sha256.json').items():
        assert digest((ROOT / name).read_bytes()) == pin, 'current source: ' + name
        assert digest((HERE / 'source' / name).read_bytes()) == pin, 'archived source: ' + name
    previous_count = 0
    for name, pin in read('before.json')['prior_indexes'].items():
        path = ROOT / name
        assert digest(path.read_bytes()) == pin, name
        for member, item in json.loads(path.read_text()).items():
            data = (path.parent / member).read_bytes()
            assert len(data) == item['bytes'] and digest(data) == item['sha256'], member
            previous_count += 1
    assert previous_count == 3961

    history = module('packet_history', 'integrations/onnx/camera_history.py')
    parallel = module('packet_parallel', 'integrations/onnx/camera_parallel_check.py')
    latency, freshness, tracking = parallel.support()
    fixture = read('avx2-freshness.json')
    comparison = history.Comparison()
    comparison.same(fixture['baseline_freshness'], fixture['fallback_freshness'])
    assert comparison.differences
    original = json.loads((ROOT / 'assets/camera-freshness-v1/verification/cases/recorded-healthy-first.json').read_text())
    assert fixture['baseline_freshness'] == original['results']
    assert len(fixture['guard_manifest']['events']) == len(fixture['fallback_freshness']) == 8

    pipeline_forwards = 0
    for case in ('recorded-healthy', 'recorded-delayed-after-poll',
                 'recorded-explicit-fault-recovery', 'recorded-deadline-still-held'):
        prefix = 'pipeline-validation/' + case
        first = read(prefix + '-first.json')
        repeat = read(prefix + '-repeat.json')
        assert latency.without_timing(first) == latency.without_timing(repeat)
        expected = [row['freshness'] for row in first['events']]
        assert read(prefix + '-guard-result.json')['results'] == expected
        snapshots = [step['snapshot'] for row in expected for step in row['backend_updates']]
        assert read(prefix + '-tracker-result.json')['snapshots'] == snapshots
        pipeline_forwards += first['execution']['model_forwards'] + repeat['execution']['model_forwards']
    assert pipeline_forwards == read('pipeline-validation/summary.json')['actual_model_forwards'] == 216

    latency_forwards = 0
    source = read('latency-validation/sequence.json')
    inspected = read('latency-validation/inspection.json')
    for label in ('first', 'repeat'):
        report = read('latency-validation/' + label + '.json')
        manifest, _, _ = latency.audit(report, source, inspected, tracking, freshness)
        assert manifest == read('latency-validation/' + label + '-guard-input.json')
        outputs = [event['output'] for row in report['events'] for event in row['measured']['guard_events']]
        assert outputs == read('latency-validation/' + label + '-guard-result.json')['results']
        latency_forwards += report['model_forwards']
    assert latency_forwards == 8

    source = json.loads((ROOT / 'assets/camera-sequence-v1/sequence.json').read_text())
    totals = {}
    for mode in ('parallel', 'cpu'):
        prefix = mode + '-validation/'
        summary = read(prefix + 'summary.json')
        assert summary['pass_checks'] and not summary['inherited_quality_gates_pass']
        plan = read(prefix + ('before-verification.json' if mode == 'parallel' else 'before-inference.json'))
        trial_prefix = prefix + 'benchmark/' if mode == 'parallel' else prefix
        trials = read(trial_prefix + 'trials.json')
        timed_forwards = 0
        for trial in trials:
            label = trial['label']
            producer = label.split('-')[0]
            report = read(trial_prefix + label + '.json')
            inspection = read(prefix + producer + '-inspection.json')
            if producer == 'serial':
                manifest, _, _ = latency.audit(report, source, inspection, tracking, freshness)
            else:
                manifest, _, _ = parallel.audit_parallel(report, source, inspection, freshness, tracking)
            assert manifest == read(prefix + label + '-guard-input.json')
            outputs = [event['output'] for row in report['events'] for event in row['measured']['guard_events']]
            assert outputs == read(prefix + label + '-guard-result.json')['results']
            assert not report['summary']['deadline_gate_pass']
            timed_forwards += report['model_forwards']
        images_forwards = 0
        for image in plan['image_selection']:
            name = image['name']
            references = 'current-serial/' if mode == 'parallel' else 'current-baseline/'
            reference = read(prefix + references + name + '.json')
            images_forwards += reference['execution']['model_forwards']
            for repeat in ('first', 'repeat'):
                report = read(prefix + 'parity/' + name + '-' + repeat + '.json')
                if mode == 'parallel':
                    parallel.parallel_analysis(report, reference)
                else:
                    assert parallel.stripped(report) == parallel.stripped(reference)
                images_forwards += report['execution']['model_forwards']
        totals[mode] = images_forwards + timed_forwards
        assert totals[mode] == summary['total_actual_model_forwards'] == 208
        assert summary['same_host_reference_forwards'] == 64
    assert pipeline_forwards + latency_forwards + sum(totals.values()) + 64 == 704
    print(json.dumps(dict(pass_checks=True, packet_files=len(files), prior_evidence_entries=previous_count,
                          captured_local_model_forwards=704, model_execution_this_check=False)))


if __name__ == '__main__':
    main()
