#!/usr/bin/env python3
"""Rebuild all saved support decisions and quality scores without any model."""
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
    p = HERE / name
    return p.read_bytes() if p.exists() else gzip.decompress(Path(str(p) + '.gz').read_bytes())


def read(name):
    return json.loads(raw(name))


def main():
    files_index = read('files.json')
    assert set(files_index) == {str(p.relative_to(HERE)) for p in HERE.rglob('*') if p.is_file() and p != HERE / 'files.json'}
    for name, pin in files_index.items():
        body = (HERE / name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    for name, pin in read('raw-files.json').items():
        body = raw(name)
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    before = read('before.json')
    old = json.loads((ROOT / 'assets/camera-grid-v1/source-sha256.json').read_text())
    assert before['git_head'] == '38caf71df92cba88687caf493ead7d0f764bf02a'
    assert before['previous_sources'] == old and len(old) == 177
    assert len(before['prior_indexes']) == 80 and len(before['prior_entries']) == 7122
    for name, pin in before['prior_indexes'].items():
        assert sha((ROOT / name).read_bytes()) == pin, name
    for name, pin in before['prior_entries'].items():
        body = (ROOT / name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    pins = read('source-sha256.json')
    assert len(pins) == 187 and all(pins[n] == h for n, h in old.items())
    parents = [(ROOT / 'assets' / p, json.loads((ROOT / 'assets' / p / 'source-sha256.json').read_text()))
               for p in ('camera-nano-v1', 'camera-latest-v1', 'camera-nano-repeat2x-v1',
                         'camera-supervised-v1', 'camera-depthwise-v1', 'camera-rowzip-v1', 'camera-linear-v1', 'camera-grid-v1')]
    for name, pin in pins.items():
        assert sha((ROOT / name).read_bytes()) == pin, name
        folder = next((p for p, mapping in parents if name in mapping), HERE)
        assert sha((folder / 'source' / name).read_bytes()) == pin, name
    assert read('core-output-sha256.json') == before['core_outputs'] == json.loads(
        (ROOT / 'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert len(before['core_outputs']) == 245
    assert read('immutability.json') == dict(previous_source_files=177, prior_indexes=80,
                                           prior_evidence_entries=7122, core_outputs=245, all_unchanged=True)
    assert not any(p.suffix in ('.jpg', '.png', '.onnx', '.whl', '.exe') for p in HERE.rglob('*'))

    sys.path.insert(0, str(ROOT / 'integrations/onnx'))
    import camera_supported_check as c
    design = json.loads((ROOT / 'integrations/onnx/profile-supported.json').read_text())
    tree_body = gzip.decompress((ROOT / 'assets/camera-linear-v1/tree.json.gz').read_bytes())
    assert sha(tree_body) == design['tree_sha256']
    tree = json.loads(tree_body)
    assert tree['truncated'] is False
    paths = {row['path']: row for row in tree['tree']}
    eligible = []
    for path, pin in sorted(paths.items()):
        if path.startswith('bdd/images/val/') and path.endswith('.jpg'):
            identity = Path(path).stem
            label = 'bdd/json_labels/val/' + identity + '.json'
            if identity not in design['excluded_image_stems'] and label in paths:
                eligible.append(dict(id=identity, image=pin, annotation=paths[label]))
    assert eligible[:16] == design['images']
    assert before['protocol_sha256'] == c.digest(c.HERE / 'profile-supported.json')
    assert design['quality_gates'] == dict(minimum_precision=.7, minimum_recall=.6,
                                         minimum_per_class_recall_when_present=.5)
    production = (ROOT / 'integrations/onnx/src/yolox_vehicle_context.rs').read_text().split('#[cfg(test)]')[0]
    adapted = (ROOT / 'integrations/onnx/src/camera_supported_coarse_context.rs').read_text().split('#[cfg(test)]')[0]
    assert production == adapted
    control = read('controls/summary.json')
    assert control['checker_sha256'] == c.digest(c.__file__) and control['compiled_sources'] == c.expected()
    assert len(c.expected()) == 75 and control['pass_controls'] and control['model_forwards'] == 0
    assert control['valid_fixtures'] == 11 and len(control['invalid_fixtures']) == 9
    assert control['existing_output_preserved'] and len(control['complete_input_pixels']) == 56
    note = read('pre-inference-dependency-note.json')
    initial_checker = raw('initial-checker.py')
    assert sha(initial_checker) == read('initial-controls-summary.json')['checker_sha256'] == note['initial_checker_sha256']
    assert initial_checker.decode().replace('import json\n', 'import json\nimport math\n', 1) == Path(c.__file__).read_text()
    assert note['final_checker_sha256'] == c.digest(c.__file__) and note['new_model_forwards_before_correction'] == 0

    with tempfile.TemporaryDirectory(prefix='rustdrive-supported-audit-') as directory:
        out = Path(directory)
        for name in read('raw-files.json'):
            if name.startswith('validation/') or name.startswith('controls/'):
                target = out / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(raw(name))
        for case in range(11):
            path = out / 'controls/inputs' / (str(case) + '.json')
            c.check_fixture_report(read('controls/' + str(case) + '-actual.json'), path)
        observed = read('validation/before-inference.json')
        assert observed['checker_sha256'] == c.digest(c.__file__) and observed['compiled_sources'] == c.expected()
        assert observed['fetcher_sha256'] == c.digest(c.HERE / 'camera_supported_fetch.py')
        assert observed['executable_sha256'] == control['executable_sha256']
        assert observed['control_summary_sha256'] == sha(raw('controls/summary.json'))
        assert observed['prior_new_model_forwards'] == observed['warmup_model_forwards'] == 0
        assert len(observed['frozen_parent_sha256']) == 120
        for name, pin in observed['frozen_parent_sha256'].items():
            assert sha((ROOT / name).read_bytes()) == pin, name
        rows = observed['input_rows']
        assert len(rows) == len({r['id'] for r in rows}) == 56
        assert [r['id'] for r in rows[:40]] == design['viewed_ids']
        assert [r['id'] for r in rows[40:]] == [r['id'] for r in design['images']]
        assert [r['partition'] for r in rows] == ['viewed_development'] * 40 + ['fresh_sample'] * 16
        old_rows = json.loads(gzip.decompress((ROOT / 'assets/camera-grid-v1/validation/before-inference.json.gz').read_bytes()))['input_rows']
        for row, previous in zip(rows[:40], old_rows):
            assert all(row[k] == previous[k] for k in ('id', 'image_sha256', 'annotation_sha256', 'original_split'))
        def annotation(row):
            name = 'annotations/' + row['id'] + '.json.gz'
            for folder in (HERE, ROOT / 'assets/camera-grid-v1', ROOT / 'assets/camera-linear-v1'):
                p = folder / name
                if p.exists():
                    return gzip.decompress(p.read_bytes())
            raise AssertionError('missing pinned original annotation')
        for row in rows:
            assert sha(annotation(row)) == row['annotation_sha256']
            pixel = control['complete_input_pixels'][row['id']]
            assert pixel['source_image_sha256'] == row['image_sha256']
            assert [v['rectangle'] for v in pixel['views']] == c.grid.rectangles(1280, 720)
        inputs = read('inputs.json')
        for row, image in zip(rows[40:], design['images']):
            for kind in ('image', 'annotation'):
                pin = image[kind]
                receipt = inputs[Path(pin['path']).name]
                assert receipt['sha256'] == row[kind + '_sha256']
                assert receipt['bytes'] == pin['size'] and receipt['git_blob_sha1'] == pin['sha']
                if kind == 'annotation':
                    body = annotation(row)
                    assert hashlib.sha1(b'blob ' + str(len(body)).encode() + b'\0' + body).hexdigest() == pin['sha']
        frozen = read('validation/summary.json')
        assert c.audit_saved(out / 'validation', rows, control, annotation) == frozen
        receipt = read('validation/before-label-scoring.json')
        assert receipt['numeric_annotations_parsed_by_this_process'] is False
        assert len(receipt['output_sha256']) == 288
        for name, pin in receipt['output_sha256'].items():
            assert sha((out / 'validation' / name).read_bytes()) == pin, name
        assert frozen['execution'] == dict(new_model_forwards=576, new_native_image_runs=48,
            reused_native_parent_reports=120, reused_parent_forwards_not_counted_as_new=1440,
            native_fixture_postprocessing_runs=120, postprocessing_model_forwards=0)
        counts = {g: {m: [v['overall'][k] for k in ('true_positives', 'false_positives', 'false_negatives')]
                      for m, v in group.items()} for g, group in frozen['groups'].items()}
        assert counts == dict(viewed_development=dict(coarse=[266, 182, 474], naive=[337, 316, 403], supported=[293, 207, 447]),
                              fresh_sample=dict(coarse=[118, 72, 151], naive=[151, 113, 118], supported=[134, 90, 135]))
        assert frozen['adopt_candidate'] is False
        assert all(not v['quality_gates']['passed'] for g in frozen['groups'].values() for v in g.values())
        assert all(not v['lost'] for v in frozen['reference_changes'].values())
        assert all(not v['precision_no_coarse_regression'] for v in frozen['relative_gates'].values())
        license_pin = json.loads((ROOT / 'integrations/onnx/bdd_protocol.json').read_text())['license']['sha256']
        assert sha(raw('LICENSE-BDD-data.txt')) == license_pin
    print(json.dumps(dict(pass_checks=True, indexed_files=len(files_index), preserved_entries=7122,
                          new_model_forwards=576, old_parent_forwards_not_recounted=1440,
                          fresh_images=16, adopt_candidate=False, model_execution_this_check=False)))


if __name__ == '__main__':
    main()
