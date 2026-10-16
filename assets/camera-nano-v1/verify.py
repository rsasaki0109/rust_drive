#!/usr/bin/env python3
"""Audit all captured Nano observations and original labels without inference."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def raw(name):
    path = HERE / name
    if not path.exists():path = HERE / (name + '.gz')
    data = path.read_bytes()
    return gzip.decompress(data) if path.suffix == '.gz' else data


def read(name):
    return json.loads(raw(name))


def reference(name):
    path = (f'assets/camera-sequence-v1/recorded/exports/first-{name[4:]}.json' if name.startswith('tum-')
            else f'assets/camera-analysis-v1/publication-repair/regression/first/{name}.json')
    return json.loads((ROOT / path).read_text())


def quality(checker, candidate=True):
    recorded = read('validation-checked/' + ('quality.json' if candidate else 'original-quality.json'))
    protocol = checker.matcher.protocol(); rows = []
    for image in protocol['images']:
        data = raw('annotations/' + image['annotation']['file'])
        assert len(data) == image['annotation']['bytes'] and sha(data) == image['annotation']['sha256']
        labels = json.loads(data)
        assert labels['name'] == image['id'] and len(labels['frames']) == 1
        assert labels['attributes'] == image['attributes'] and labels['frames'][0]['timestamp'] == 10000
        refs, excluded, seen = [], [], set()
        for label in labels['frames'][0]['objects']:
            assert label['id'] not in seen
            seen.add(label['id'])
            if 'box2d' not in label:continue
            box = [label['box2d'][key] for key in ('x1', 'y1', 'x2', 'y2')]
            assert checker.matcher.valid_box(box, image['width'], image['height'])
            ref = dict(id=label['id'], category=label['category'], bbox=box, attributes=label['attributes'])
            if label['category'] in protocol['class_mapping']:
                refs.append(dict(ref, class_index=protocol['class_mapping'][label['category']]))
            else:
                assert label['category'] in protocol['unscored_category_reason']
                excluded.append(ref)
        report = read(f"validation-checked/{'first' if candidate else 'original'}/{image['id']}.json")
        assert report['source_image_sha256'] == image['image']['sha256']
        detections = report['result']['detections'] if candidate else report['detections']
        selected = [row for row in detections if row['class_index'] in protocol['class_mapping'].values()]
        matches, fp, fn = checker.matcher.match_boxes(refs, selected, protocol['match_iou_threshold'])
        row = next(row for row in recorded['images'] if row['image_id'] == image['id'])
        assert row['reference_count'] == len(refs) and row['excluded_original_sign_boxes'] == excluded
        assert row['matches'] == matches and row['false_positives'] == fp and row['false_negatives'] == fn
        assert row['original_annotation_sha256'] == image['annotation']['sha256']
        rows.append(dict(matches=matches, false_positives=fp, false_negatives=fn))
    assert checker.matcher.aggregate(rows) == recorded['overall']
    return recorded


def main():
    files = read('files.json')
    actual = {str(p.relative_to(HERE)) for p in HERE.rglob('*')
              if p.is_file() and p.name != 'files.json' and '__pycache__' not in p.parts}
    assert actual == files.keys()
    for name, pin in files.items():
        data = (HERE / name).read_bytes()
        assert len(data) == pin['bytes'] and sha(data) == pin['sha256'], name
    for name, pin in read('raw-files.json').items():
        data = raw(name)
        assert len(data) == pin['original_bytes'] and sha(data) == pin['original_sha256'], name
    for name, pin in read('source-sha256.json').items():
        assert sha((ROOT / name).read_bytes()) == pin, 'current source: ' + name
        assert sha((HERE / 'source' / name).read_bytes()) == pin, 'archived source: ' + name
    old_count = 0
    for name, pin in read('before.json')['prior_indexes'].items():
        index = ROOT / name
        assert sha(index.read_bytes()) == pin, name
        for member, item in json.loads(index.read_text()).items():
            data = (index.parent / member).read_bytes()
            assert len(data) == item['bytes'] and sha(data) == item['sha256'], member
            old_count += 1
    assert old_count == 5171

    sys.path.insert(0, str(ROOT / 'integrations/onnx'))
    spec = importlib.util.spec_from_file_location('packet_nano', ROOT / 'integrations/onnx/camera_nano_check.py')
    checker = importlib.util.module_from_spec(spec); spec.loader.exec_module(checker)
    checker.check_clock_source()
    before = read('validation-checked/before-inference.json')
    summary = read('validation-checked/summary.json')
    for name, pin in before['source_sha256'].items():assert sha((ROOT / name).read_bytes()) == pin, name
    assert read('validation-checked/nano-executable.json')['compiled_sources'] == checker.expected('runtime')
    source = json.loads((ROOT / 'assets/camera-sequence-v1/sequence.json').read_text())
    trials = read('validation-checked/trials.json')
    assert [row['label'] for row in trials] == checker.ORDER
    timed_forwards = 0
    for trial in trials:
        label = trial['label']; mode = label.split('-')[0]
        for suffix in ('.json', '-guard-input.json', '-guard-result.json'):
            assert raw('validation/' + label + suffix) == raw('validation-final/' + label + suffix)
            assert raw('validation/' + label + suffix) == raw('validation-checked/' + label + suffix)
        report = read('validation-checked/' + label + '.json')
        inspected = read('validation-checked/' + mode + '-inspection.json')
        audit = checker.clock.audit_measured if mode == 'nano' else checker.previous.audit_parallel
        manifest, _, _ = audit(report, source, inspected, checker.freshness, checker.tracking)
        assert manifest == read('validation-checked/' + label + '-guard-input.json')
        assert read('validation-checked/' + label + '-guard-result.json')['results'] == [
            event['output'] for row in report['events'] for event in row['measured']['guard_events']]
        assert trial['summary'] == report['summary']
        assert trial['processing_s'] == [row['measured']['timing']['read_analysis_and_probe_wall_s']
                                        for row in report['events'] if row['measured']['analysis'] is not None]
        for row in report['events']:
            actual = row['measured']['analysis']
            if actual is None:continue
            name = 'tum-' + str(row['source']['event_index'])
            if mode == 'nano':checker.check_image(actual, reference(name), read('validation-checked/serial/' + name + '.json'))
            else:checker.repeat.check_analysis(actual, reference(name), checker.latency.camera_history.Comparison())
        timed_forwards += report['model_forwards']
    parity_forwards = 0
    for selected in before['image_selection']:
        name = selected['name']; saved = reference(name)
        serial = read('validation-checked/serial/' + name + '.json')
        checker.check_image(serial, saved, parallel=False)
        original = read('validation-checked/original/' + name + '.json')
        assert checker.operator.measurement_bits(original['detections']) == checker.operator.measurement_bits(serial['base_prediction']['views'][0]['prediction']['detections'])
        projections = []
        for directory in ('serial', 'original', 'first', 'repeat'):
            assert raw(f'validation-final/{directory}/{name}.json') == raw(f'validation-checked/{directory}/{name}.json')
        for directory in ('first', 'repeat'):
            report = read(f'validation-checked/{directory}/{name}.json')
            checker.check_image(report, saved, serial)
            projections.append(checker.previous.stripped(report))
        assert projections[0] == projections[1]
        parity_forwards += 13
    assert raw('validation/serial/tum-0.json') == raw('validation-checked/serial/tum-0.json')
    assert timed_forwards == summary['timed_model_forwards'] == 40
    assert parity_forwards == summary['parity_model_forwards'] == 208
    assert summary['actual_model_forwards'] == summary['imported_model_forwards'] == 248
    assert summary['model_forwards_this_invocation'] == 0
    for directory, expected in [('validation', 44), ('validation-final', 248)]:
        old = read(directory + '/before-inference.json')['source_sha256']
        assert sha(raw(directory + '/failed-checker.py')) == old['integrations/onnx/camera_nano_check.py']
        assert read(directory + '/failed-audit.json')['captured_model_forwards'] == expected
    candidate, original = quality(checker), quality(checker, False)
    assert candidate['overall'] == summary['quality'] and original['overall'] == summary['original_single_view_quality']
    assert not candidate['quality_gates']['pass'] and not summary['quality_gate_pass']
    nano_times = [row['processing_s'][0] for row in trials if row['label'].startswith('nano')]
    s_times = [row['processing_s'][0] for row in trials if row['label'].startswith('s')]
    assert statistics.mean(s_times) / statistics.mean(nano_times) == summary['ratio_of_first_image_means']
    assert summary['deadline_gates'] == {row['label']: row['summary']['deadline_gate_pass'] for row in trials}
    assert summary['actual_availability'] == {row['label']: row['summary']['available_at_check_images'] for row in trials}
    print(json.dumps(dict(pass_checks=True, packet_files=len(files), prior_evidence_entries=old_count,
                          unique_model_forwards_audited=248, exact_guard_replays=4,
                          independently_scored_reference_boxes=candidate['reference_count'], model_execution_this_check=False)))


if __name__ == '__main__':main()
