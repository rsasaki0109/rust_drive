#!/usr/bin/env python3
"""Audit complete saved grid results without original images or model execution."""
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(body):
    return hashlib.sha256(body).hexdigest()


def raw(name):
    path = HERE / name
    return path.read_bytes() if path.exists() else gzip.decompress(Path(str(path) + '.gz').read_bytes())


def read(name):
    return json.loads(raw(name))


def main():
    index = read('files.json')
    assert set(index) == {str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p != HERE / 'files.json'}
    for name, pin in index.items():
        body = (HERE / name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    for name, pin in read('raw-files.json').items():
        body = raw(name)
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    before = read('before.json')
    old = json.loads((ROOT / 'assets/camera-linear-v1/source-sha256.json').read_text())
    assert before['git_head'] == '739bac8bd23e3032e29d9010e770d105439e3694'
    assert before['previous_source_sha256'] == old and len(old) == 167
    assert len(before['prior_indexes']) == 79 and len(before['prior_entries']) == 6795
    for name, pin in before['prior_indexes'].items():
        assert sha((ROOT / name).read_bytes()) == pin, name
    for name, pin in before['prior_entries'].items():
        body = (ROOT / name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    pins = read('source-sha256.json')
    assert len(pins) == 177 and all(pins[k] == v for k, v in old.items())
    parents = [(ROOT / 'assets' / folder, json.loads((ROOT / 'assets' / folder / 'source-sha256.json').read_text()))
               for folder in ('camera-nano-v1', 'camera-latest-v1', 'camera-nano-repeat2x-v1',
                              'camera-supervised-v1', 'camera-depthwise-v1', 'camera-rowzip-v1', 'camera-linear-v1')]
    for name, pin in pins.items():
        assert sha((ROOT / name).read_bytes()) == pin, name
        folder = next((p for p, mapping in parents if name in mapping), HERE)
        assert sha((folder / 'source' / name).read_bytes()) == pin, name
    assert read('core-output-sha256.json') == before['core_outputs'] == json.loads(
        (ROOT / 'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert len(before['core_outputs']) == 245
    assert read('immutability.json') == dict(previous_source_files=167, prior_indexes=79,
        prior_evidence_entries=6795, core_outputs=245, all_unchanged=True)
    assert not any(p.suffix in ('.onnx', '.jpg', '.png', '.whl', '.exe') for p in HERE.rglob('*'))

    sys.path.insert(0, str(ROOT / 'integrations/onnx'))
    import camera_grid_check as c
    design = json.loads((ROOT / 'integrations/onnx/profile-grid.json').read_text())
    tree_body = gzip.decompress((ROOT / 'assets/camera-linear-v1/tree.json.gz').read_bytes())
    assert sha(tree_body) == design['tree_sha256']
    tree = json.loads(tree_body)
    assert tree['truncated'] is False
    paths = {r['path']: r for r in tree['tree']}
    candidates = []
    for path, pin in sorted(paths.items()):
        if path.startswith('bdd/images/val/') and path.endswith('.jpg'):
            identity = Path(path).stem
            label = 'bdd/json_labels/val/' + identity + '.json'
            if identity not in design['excluded_image_stems'] and label in paths:
                candidates.append(dict(id=identity, image=pin, annotation=paths[label]))
    assert candidates[:16] == design['images']
    assert before['selection_protocol_sha256'] == c.digest(c.HERE / 'profile-grid.json')
    assert design['grid']['maximum_views'] == 16 and design['grid']['maximum_concurrent_views'] == 4
    assert design['quality_gates'] == dict(minimum_precision=.7, minimum_recall=.6,
                                         minimum_per_class_recall_when_present=.5)
    control = read('controls/summary.json')
    assert control['checker_sha256'] == c.digest(c.__file__)
    assert control['compiled_sources'] == c.expected() and len(c.expected()) == 70
    assert control['pass_controls'] and control['model_forwards'] == 0
    assert control['valid_fixtures'] == 8 and len(control['invalid_fixtures']) == 9 and control['existing_output_preserved']
    assert len(control['complete_input_pixels']) == 40
    for case_index in range(8):
        f = read(f'controls/inputs/{case_index}.json')
        c.prior.nano.tiles.same(c.merge(f), read(f'controls/{case_index}-merge.json'))
        c.prior.nano.tiles.same(c.oracle(f), read(f'controls/{case_index}-consistency.json'))

    with tempfile.TemporaryDirectory(prefix='rustdrive-grid-audit-') as directory:
        out = Path(directory)
        for name in read('raw-files.json'):
            if name.startswith('validation/'):
                target = out / name.removeprefix('validation/')
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(raw(name))
        observed = read('validation/before-inference.json')
        assert observed['checker_sha256'] == c.digest(c.__file__)
        assert observed['fetcher_sha256'] == c.digest(c.HERE / 'camera_grid_fetch.py')
        assert observed['compiled_sources'] == c.expected() and observed['baseline_sources'] == c.prior.expected()
        assert observed['executable_sha256'] == control['executable_sha256']
        assert observed['baseline_executable_sha256'] == control['baseline_executable_sha256']
        assert observed['control_summary_sha256'] == sha(raw('controls/summary.json'))
        assert observed['prior_model_forwards_this_experiment'] == observed['warmup_model_forwards'] == 0
        rows = observed['input_rows']
        assert len(rows) == len({r['id'] for r in rows}) == 40
        assert [r['id'] for r in rows[:24]] == design['viewed_ids']
        assert [r['id'] for r in rows[24:]] == [r['id'] for r in design['images']]
        assert [r['partition'] for r in rows] == ['viewed_development'] * 24 + ['fresh_sample'] * 16
        previous_rows = json.loads(gzip.decompress((ROOT / 'assets/camera-linear-v1/validation/before-inference.json.gz').read_bytes()))['input_rows']
        for row, old_row in zip(rows[:24], previous_rows):
            assert all(row[k] == old_row[k] for k in ('id', 'image_sha256', 'annotation_sha256', 'original_split'))
        def annotation(row):
            name = 'annotations/' + row['id'] + '.json'
            if (HERE / (name + '.gz')).exists():
                return raw(name)
            return gzip.decompress((ROOT / 'assets/camera-linear-v1' / (name + '.gz')).read_bytes())
        for row in rows:
            assert sha(annotation(row)) == row['annotation_sha256']
            report = read('controls/' + row['id'] + '-inventory.json')
            assert report['compiled_sources'] == c.expected() and report['views'] == c.rectangles(1280, 720)
            assert len(report['views']) == 12 and report['source_image_sha256'] == row['image_sha256']
            assert report['model_read'] is report['inference_run'] is report['control_permission'] is False
            pixels = control['complete_input_pixels'][row['id']]
            assert pixels['source_image_sha256'] == row['image_sha256'] and [v['rectangle'] for v in pixels['views']] == report['views']
        inputs = read('inputs.json')
        for row, item in zip(rows[24:], design['images']):
            for kind in ('image', 'annotation'):
                receipt = inputs[Path(item[kind]['path']).name]
                assert receipt['git_blob_sha1'] == item[kind]['sha'] and receipt['bytes'] == item[kind]['size']
                assert receipt['sha256'] == row[kind + '_sha256']
                if kind == 'annotation':
                    body = annotation(row)
                    assert hashlib.sha1(b'blob ' + str(len(body)).encode() + b'\0' + body).hexdigest() == item[kind]['sha']
        frozen = read('validation/summary.json')
        assert c.audit_saved(out, rows, control, annotation) == frozen
        receipt = read('validation/before-label-scoring.json')
        assert receipt['numeric_annotations_parsed_by_this_process'] is False
        assert len(receipt['native_report_sha256']) == 200
        for name, pin in receipt['native_report_sha256'].items():
            assert sha((out / name).read_bytes()) == pin
        assert frozen['execution'] == dict(native_image_runs=200, model_forwards=1760,
            baseline_forwards=320, grid_forwards=960, current_original_graph_forwards=480,
            repeat_per_view_pairs=640, original_graph_per_view_pairs=480, original_prefix_per_view_pairs=320)
        counts = {g: {m: [v['overall'][k] for k in ('true_positives', 'false_positives', 'false_negatives')]
                      for m, v in group.items()} for g, group in frozen['groups'].items()}
        assert counts == dict(viewed_development=dict(baseline=[154, 99, 252], grid=[191, 173, 215]),
                              fresh_sample=dict(baseline=[112, 83, 222], grid=[146, 143, 188]))
        assert frozen['adopt_candidate'] is False
        assert all(not v['quality_gates']['passed'] for g in frozen['groups'].values() for v in g.values())
        license_pin = json.loads((ROOT / 'integrations/onnx/bdd_protocol.json').read_text())['license']['sha256']
        assert sha(raw('LICENSE-BDD-data.txt')) == license_pin
        # Report observed quality gates independently of correct execution.
        assert frozen['control_permission'] is False and frozen['new_latency_or_availability_claim'] is False
        print(json.dumps(dict(pass_checks=True, indexed_files=len(index), preserved_entries=6795,
                              saved_model_forwards=1760, fresh_images=16,
                              adopt_candidate=frozen['adopt_candidate'], model_execution_this_check=False)))


if __name__ == '__main__':
    main()
