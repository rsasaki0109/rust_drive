#!/usr/bin/env python3
"""Replay the current and frozen optimization evidence without model execution."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tempfile

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


def saved(name):
    relative = (f'assets/camera-sequence-v1/recorded/exports/first-{name[4:]}.json'
                if name.startswith('tum-') else
                f'assets/camera-analysis-v1/publication-repair/regression/first/{name}.json')
    return json.loads((ROOT / relative).read_text())


def verify_frozen_v1():
    # v1 deliberately checks its own exact source version. Restore those bytes
    # in a temporary tree; never update its frozen snapshots to match new code.
    packet = ROOT / 'assets/camera-operators-v1'
    with tempfile.TemporaryDirectory(prefix='camera-operators-v1-') as directory:
        root = Path(directory)
        for name, pin in json.loads((packet / 'source-sha256.json').read_text()).items():
            data = (packet / 'source' / name).read_bytes()
            assert digest(data) == pin, name
            destination = root / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
        assets = root / 'assets'
        assets.mkdir(exist_ok=True)
        for path in (ROOT / 'assets').iterdir():
            if path.name != 'camera-operators-v1':
                (assets / path.name).symlink_to(path, target_is_directory=path.is_dir())
        shutil.copytree(packet, assets / packet.name, ignore=shutil.ignore_patterns('__pycache__'))
        result = subprocess.run([sys.executable, '-B', str(assets / packet.name / 'verify.py')],
                                cwd=root, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stderr
        return json.loads(result.stdout)


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
    assert previous_count == 4901
    legacy = verify_frozen_v1()
    assert legacy['operator_forwards_audited'] == 264
    assert legacy['parallel_forwards_audited'] == 144

    operator = module('packet_operator', 'integrations/onnx/camera_operator_check.py')
    parallel = module('packet_repeat', 'integrations/onnx/camera_repeat_check.py')
    history = operator.latency.camera_history.Comparison()
    before = read('operator-validation/before-inference.json')
    pins = operator.expected_sources()
    baseline = read('operator-validation/baseline-profile.json')
    operator.audit(baseline, saved('tum-0'), pins, False, False, history)
    operator_forwards = baseline['execution']['model_forwards']
    optimized, original = [], []
    for selected in before['image_selection']:
        projections = []
        for repeat, reference_first in [('first', False), ('repeat', True)]:
            report = read(f"operator-validation/reports/{selected['name']}-{repeat}.json")
            operator.audit(report, saved(selected['name']), pins, True, reference_first, history)
            projections.append(operator.numerical_projection(report))
            operator_forwards += report['execution']['model_forwards']
            optimized.append(sum(v['profile']['forward_wall_ns'] for v in report['views']))
            original.append(sum(v['reference_forward_wall_ns'] for v in report['views']))
        assert projections[0] == projections[1]
    summary = read('operator-validation/summary.json')
    assert operator_forwards == summary['validation_actual_model_forwards'] == 264
    assert summary['historical_comparison'] == history.report()
    assert summary['aggregate_inference_speedup'] == sum(original) / sum(optimized)

    latency, freshness, tracking = parallel.previous.support()
    history = latency.camera_history.Comparison()
    source = json.loads((ROOT / 'assets/camera-sequence-v1/sequence.json').read_text())
    trials = read('parallel-validation/trials.json')
    assert [row['label'] for row in trials] == parallel.ORDER
    timed_forwards = image_forwards = reference_forwards = 0
    for trial in trials:
        label = trial['label']
        mode = label.split('-')[0]
        report = read('parallel-validation/' + label + '.json')
        inspection = read('parallel-validation/' + mode + '-inspection.json')
        manifest, _, _ = parallel.previous.audit_parallel(report, source, inspection, freshness, tracking)
        assert manifest == read('parallel-validation/' + label + '-guard-input.json')
        assert read('parallel-validation/' + label + '-guard-result.json')['results'] == [
            event['output'] for row in report['events'] for event in row['measured']['guard_events']]
        assert report['summary'] == trial['summary']
        for row in report['events']:
            actual = row['measured']['analysis']
            if actual is not None:
                name = 'tum-' + str(row['source']['event_index'])
                reference = read('parallel-validation/current-reference/' + name + '.json')
                if mode == 'baseline':
                    parallel.previous.parallel_analysis(actual, saved(name), history)
                    parallel.previous.parallel_analysis(actual, reference)
                else:
                    parallel.check_analysis(actual, saved(name), history)
                    parallel.check_analysis(actual, reference)
        timed_forwards += report['model_forwards']
    for selected in read('parallel-validation/before-inference.json')['image_selection']:
        name = selected['name']
        reference = read('parallel-validation/current-reference/' + name + '.json')
        parallel.previous.parallel_analysis(reference, saved(name), history)
        reference_forwards += reference['execution']['model_forwards']
        projections = []
        for repeat in ('first', 'repeat'):
            actual = read(f'parallel-validation/reports/{name}-{repeat}.json')
            parallel.check_analysis(actual, reference)
            projections.append(parallel.previous.stripped(actual))
            image_forwards += actual['execution']['model_forwards']
        assert projections[0] == projections[1]
    summary = read('parallel-validation/summary.json')
    assert timed_forwards + image_forwards + reference_forwards == summary['total_actual_model_forwards'] == 208
    assert reference_forwards == 64
    assert summary['historical_comparison'] == history.report()
    baseline = [r['processing_s'][0] for r in trials if r['label'].startswith('baseline')]
    lowered = [r['processing_s'][0] for r in trials if r['label'].startswith('repeat2x')]
    assert statistics.mean(baseline) / statistics.mean(lowered) == summary['mean_speedup']
    assert summary['deadline_gates'] == {r['label']: r['summary']['deadline_gate_pass'] for r in trials}
    assert not summary['inherited_quality_gates_pass']
    probe = read('avx2-profile.json')
    probe_pins = read('avx2-executable.json')['compiled_sources']
    changed = {name for name in pins if pins[name] != probe_pins[name]}
    assert changed == {'integrations/onnx/Cargo.toml', 'integrations/onnx/Cargo.lock'}
    for name in ('Cargo.toml', 'Cargo.lock'):
        assert digest(raw('probe/' + name)) == probe_pins['integrations/onnx/' + name]
    library = read('probe/library-sources.json')
    assert len(library) == 312
    changed_library = {name for name, item in library.items() if item['original_sha256'] != item['probe_sha256']}
    assert changed_library == {'src/x86_64_fma.rs', 'src/x86_64_fma/mmm.rs'}
    for row in read('probe/design.json')['library_patch']:
        assert library[row['path']] == {key: row[key] for key in ('original_sha256', 'probe_sha256')}
    assert raw('probe/disable-avx512.patch') == (ROOT / 'assets/camera-ci-v1/probe/disable-avx512.patch').read_bytes()
    history = operator.latency.camera_history.Comparison()
    operator.audit(probe, saved('tum-0'), probe_pins, True, False, history)
    comparison = read('avx2-comparison.json')
    assert comparison['historical_comparison'] == history.report()
    assert any(row['absolute_f32_difference'] > 0 for row in history.differences)
    try:
        operator.audit(probe, saved('tum-0'), probe_pins, True, False)
    except ValueError as error:
        assert str(error) == comparison['historical_exact_rejected']
    else:
        raise AssertionError('historical exact confidence mismatch required')
    assert probe['execution']['model_forwards'] == comparison['actual_model_forwards'] == 8
    kernels = sorted({kernel for view in probe['views'] for row in view['profile']['operators']
                      for kernel in row['matrix_kernel_choices']})
    assert kernels == comparison['matrix_kernel_choices']
    assert kernels and not any('avx512' in name.lower() for name in kernels)
    print(json.dumps(dict(pass_checks=True, packet_files=len(files), prior_evidence_entries=previous_count,
                          current_model_forwards_audited=operator_forwards + timed_forwards + image_forwards + reference_forwards + 8,
                          frozen_v1_replayed=True, model_execution_this_check=False)))


if __name__ == '__main__':
    main()
