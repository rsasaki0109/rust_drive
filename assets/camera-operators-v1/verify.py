#!/usr/bin/env python3
"""Verify the preserved packet without images, weights, or model execution."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read(name):
    path = HERE / name
    if not path.exists():
        path = HERE / (name + '.gz')
    data = path.read_bytes()
    if path.suffix == '.gz':
        data = gzip.decompress(data)
    return json.loads(data)


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def reference(name):
    if name.startswith('tum-'):
        return ROOT / f'assets/camera-sequence-v1/recorded/exports/first-{name[4:]}.json'
    return ROOT / f'assets/camera-analysis-v1/publication-repair/regression/first/{name}.json'


def main():
    files = read('files.json')
    actual = {str(p.relative_to(HERE)) for p in HERE.rglob('*')
              if p.is_file() and '__pycache__' not in p.parts and p.name != 'files.json'}
    assert actual == files.keys(), 'complete packet file inventory'
    for name, pin in files.items():
        data = (HERE / name).read_bytes()
        assert len(data) == pin['bytes'] and digest(data) == pin['sha256'], name
    for name, pin in read('raw-files.json').items():
        data = (HERE / name).read_bytes()
        if name.endswith('.gz'):
            data = gzip.decompress(data)
        assert len(data) == pin['original_bytes'] and digest(data) == pin['original_sha256'], name
    for name, pin in read('source-sha256.json').items():
        assert digest((ROOT / name).read_bytes()) == pin, 'current source: ' + name
        assert digest((HERE / 'source' / name).read_bytes()) == pin, 'archived source: ' + name

    operator = module('packet_operators', 'integrations/onnx/camera_operator_check.py')
    parallel = module('packet_parallel', 'integrations/onnx/camera_repeat_check.py')
    before = read('validation-final/before-inference.json')
    pins = operator.expected_sources()
    assert len(pins) == 27
    first_saved = json.loads(reference('tum-0').read_text())
    operator.audit(read('validation-final/baseline-profile.json'), first_saved, pins, False, False)
    optimized, original = [], []
    forwards = 8
    for selected in before['image_selection']:
        saved_path = reference(selected['name'])
        assert digest(saved_path.read_bytes()) == selected['reference_report_sha256']
        saved = json.loads(saved_path.read_text())
        projections = []
        for repeat, reference_first in [('first', False), ('repeat', True)]:
            report = read(f"validation-final/reports/{selected['name']}-{repeat}.json")
            operator.audit(report, saved, pins, True, reference_first)
            projections.append(operator.numerical_projection(report))
            optimized.append(sum(v['profile']['forward_wall_ns'] for v in report['views']))
            original.append(sum(v['reference_forward_wall_ns'] for v in report['views']))
            forwards += report['execution']['model_forwards']
        assert projections[0] == projections[1], 'complete repeat'
    summary = read('validation-final/summary.json')
    assert forwards == summary['validation_actual_model_forwards'] == 264
    assert statistics.mean(optimized) / 1e9 == summary['mean_optimized_four_view_inference_s']
    assert statistics.mean(original) / 1e9 == summary['mean_original_four_view_inference_s']
    assert sum(original) / sum(optimized) == summary['aggregate_inference_speedup']

    source = json.loads((ROOT / 'assets/camera-sequence-v1/sequence.json').read_text())
    _, freshness, tracking = parallel.previous.support()
    trials = read('parallel-validation/trials.json')
    assert [row['label'] for row in trials] == parallel.ORDER
    timed_forwards = 0
    for trial in trials:
        label = trial['label']
        mode = label.split('-')[0]
        report = read('parallel-validation/' + label + '.json')
        inspection = read('parallel-validation/' + mode + '-inspection.json')
        manifest, _, _ = parallel.previous.audit_parallel(report, source, inspection, freshness, tracking)
        assert manifest == read('parallel-validation/' + label + '-guard-input.json')
        expected = [event['output'] for row in report['events'] for event in row['measured']['guard_events']]
        assert expected == read('parallel-validation/' + label + '-guard-result.json')['results']
        assert report['summary'] == trial['summary']
        assert trial['processing_s'] == [row['measured']['timing']['read_analysis_and_probe_wall_s']
                                         for row in report['events'] if row['measured']['analysis'] is not None]
        timed_forwards += report['model_forwards']
    image_forwards = 0
    for selected in before['image_selection']:
        saved = json.loads(reference(selected['name']).read_text())
        for repeat in ('first', 'repeat'):
            report = read(f"parallel-validation/reports/{selected['name']}-{repeat}.json")
            parallel.check_analysis(report, saved)
            image_forwards += report['execution']['model_forwards']
    lowered = [r['processing_s'][0] for r in trials if r['label'].startswith('repeat2x')]
    baseline = [r['processing_s'][0] for r in trials if r['label'].startswith('baseline')]
    summary = read('parallel-validation/summary.json')
    assert statistics.mean(baseline) / statistics.mean(lowered) == summary['mean_speedup']
    assert timed_forwards + image_forwards == summary['total_actual_model_forwards'] == 144
    assert summary['deadline_gates'] == {r['label']: r['summary']['deadline_gate_pass'] for r in trials}
    assert not any(summary['deadline_gates'].values())
    assert not summary['inherited_quality_gates_pass']
    assert read('f32-canonical-audit.json')['adjacent_f32_change_rejected']
    assert read('ci-investigation/diagnostic-selfchecks.json')['exact_equality_preserved']
    print(json.dumps(dict(pass_checks=True, packet_files=len(files), operator_forwards_audited=forwards,
                          parallel_forwards_audited=timed_forwards + image_forwards,
                          model_execution_this_check=False)))


if __name__ == '__main__':
    main()
