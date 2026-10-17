#!/usr/bin/env python3
"""Reconstruct frozen preprocessing diagnostics without images, weights or cv2."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import struct
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
    assert set(index) == {str(p.relative_to(HERE)) for p in HERE.rglob('*')
                          if p.is_file() and p != HERE / 'files.json'}
    for name, pin in index.items():
        body = (HERE / name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    for name, pin in read('raw-files.json').items():
        body = raw(name)
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    before = read('before.json')
    assert before['git_head'] == '7026d0c39a00787d3b68db48d0499b910cddc3e4'
    old = json.loads((ROOT / 'assets/camera-rowzip-v1/source-sha256.json').read_text())
    assert before['source_sha256'] == old and len(old) == 157
    assert len(before['prior_indexes']) == 78 and len(before['prior_entries']) == 6510
    for name, pin in before['prior_indexes'].items():
        assert sha((ROOT / name).read_bytes()) == pin, name
    for name, pin in before['prior_entries'].items():
        body = (ROOT / name).read_bytes()
        assert len(body) == pin['bytes'] and sha(body) == pin['sha256'], name
    pins = read('source-sha256.json')
    assert len(pins) == 167 and all(pins[k] == v for k, v in old.items())
    folders = [(ROOT / 'assets' / folder,
                json.loads((ROOT / 'assets' / folder / 'source-sha256.json').read_text()))
               for folder in ('camera-nano-v1', 'camera-latest-v1', 'camera-nano-repeat2x-v1',
                              'camera-supervised-v1', 'camera-depthwise-v1', 'camera-rowzip-v1')]
    for name, pin in pins.items():
        assert sha((ROOT / name).read_bytes()) == pin, name
        folder = next((p for p, mapping in folders if name in mapping), HERE)
        assert sha((folder / 'source' / name).read_bytes()) == pin, name
    assert read('core-output-sha256.json') == before['core_outputs'] == json.loads(
        (ROOT / 'assets/projective-fusion-bias-v1/core-output-sha256.json').read_text())
    assert len(before['core_outputs']) == 245
    assert read('immutability.json') == dict(previous_source_files=157, prior_indexes=78,
        prior_evidence_entries=6510, core_outputs=245, all_unchanged=True)
    assert not any(p.suffix in ('.onnx', '.jpg', '.png', '.whl', '.exe') for p in HERE.rglob('*'))

    sys.path.insert(0, str(ROOT / 'integrations/onnx'))
    import camera_linear_check as c
    protocol = json.loads((ROOT / 'integrations/onnx/profile-linear.json').read_text())
    assert sha(raw('tree.json')) == protocol['tree_sha256']
    tree = read('tree.json')
    assert tree['truncated'] is False
    paths = {r['path']: r for r in tree['tree']}
    candidates = []
    for path, row in sorted(paths.items()):
        if path.startswith('bdd/images/val/') and path.endswith('.jpg'):
            identity = Path(path).stem
            annotation = 'bdd/json_labels/val/' + identity + '.json'
            if identity not in protocol['excluded_image_stems'] and annotation in paths:
                candidates.append(dict(id=identity, image=row, annotation=paths[annotation]))
    assert candidates[:16] == protocol['images']
    assert before['selection_protocol_sha256'] == c.digest(c.HERE / 'profile-linear.json')
    assert protocol['quality_gates'] == dict(minimum_precision=.7, minimum_recall=.6,
                                           minimum_per_class_recall_when_present=.5)
    control = read('controls/summary.json')
    initial = raw('initial-checker.py').decode()
    correction = read('audit-correction.json')
    assert sha(initial.encode()) == correction['initial_checker_sha256'] == control['checker_sha256']
    assert sha((ROOT / 'integrations/onnx/camera_linear_check.py').read_bytes()) == correction['corrected_checker_sha256']
    replace = correction['exact_replacement']
    assert initial.count(replace['before']) == 1
    assert initial.replace(replace['before'], replace['after']) == (ROOT / 'integrations/onnx/camera_linear_check.py').read_text()
    assert correction['model_forwards_for_correction'] == 0 and correction['pass_corrected_audit']
    assert 'ValueError: saved independently reconstructed summary' in raw('initial-audit-failure.log').decode()
    assert control['compiled_sources'] == c.expected() and len(c.expected()) == 69
    assert control['model_forwards'] == 0 and control['pass_controls']
    assert control['reference'] == dict(opencv='4.13.0', numpy='2.3.5', optimized=False,
                                      ipp=False, opencl=False, threads=1)
    assert len(control['generated_cases']) == 48 and len(control['actual_image_pixels']) == 24
    assert control['compared_generated_values'] == sum(r['compared_values'] for r in control['generated_cases']) == 19260999
    for case in control['generated_cases']:
        actual = read('controls/' + str(case['index']) + '-actual.json')
        assert actual['compiled_sources'] == c.expected()
        assert actual['algorithm'] == 'yolox_generic_linear_preprocessing_v1'
        assert (actual['width'], actual['height'], actual['input_size']) == (case['width'], case['height'], case['size'])
        assert all(actual[k] == case[k] for k in ('rgb_sha256', 'resized', 'tensor_f32le_sha256'))
        assert struct.pack('<f', actual['decode_ratio_f32']).hex() == case['decode_ratio_f32_bits']
        assert actual['model_read'] is actual['inference_run'] is actual['control_permission'] is False
    assert len(control['invalid_inputs']) == 6 and control['existing_output_preserved']
    first_pixel_pins = read('initial-generated-control-sha256.json')
    assert len(first_pixel_pins) == 48
    for name, pin in first_pixel_pins.items():
        assert sha(raw('controls/' + name)) == pin
    assert 'FileNotFoundError' in raw('initial-controls-failure.log').decode()

    with tempfile.TemporaryDirectory(prefix='rustdrive-linear-audit-') as directory:
        temporary = Path(directory)
        for name in read('raw-files.json'):
            if name.startswith('validation/'):
                target = temporary / name.removeprefix('validation/')
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(raw(name))
        observed = read('validation/before-inference.json')
        assert observed['checker_sha256'] == correction['initial_checker_sha256']
        assert observed['compiled_sources'] == c.expected()
        assert observed['warmup_model_forwards'] == observed['prior_valid_model_forwards_this_experiment'] == 0
        assert observed['control_summary_sha256'] == sha(raw('controls/summary.json'))
        assert observed['fetcher_sha256'] == c.digest(c.HERE / 'camera_linear_fetch.py')
        rows = observed['input_rows']
        assert len(rows) == 24 and len({r['id'] for r in rows}) == 24
        assert [r['id'] for r in rows[8:]] == [r['id'] for r in protocol['images']]
        assert [r['partition'] for r in rows] == ['viewed_development'] * 8 + ['fresh_sample'] * 16
        annotation = lambda row: raw('annotations/' + row['id'] + '.json')
        for row in rows:
            assert sha(annotation(row)) == row['annotation_sha256']
            actual = read('controls/' + row['id'] + '-pixels.json')
            oracle = control['actual_image_pixels'][row['id']]['views'][0]
            assert actual['source_image_sha256'] == row['image_sha256']
            assert actual['compiled_sources'] == c.expected()
            assert all(actual[k] == oracle[k] for k in ('rgb_sha256', 'resized', 'tensor_f32le_sha256'))
            assert actual['model_read'] is actual['inference_run'] is actual['control_permission'] is False
        original = json.loads((ROOT / 'integrations/onnx/bdd_protocol.json').read_text())
        assert sha(raw('LICENSE-BDD-data.txt')) == original['license']['sha256']
        for row, pin in zip(rows[:8], original['images']):
            assert row['id'] == pin['id'] and row['image_sha256'] == pin['image']['sha256']
            assert row['annotation_sha256'] == pin['annotation']['sha256'] and row['original_split'] == pin['original_split']
        inputs = read('inputs.json')
        for row, pin in zip(rows[8:], protocol['images']):
            for kind in ('image', 'annotation'):
                receipt = inputs[Path(pin[kind]['path']).name]
                assert receipt['git_blob_sha1'] == pin[kind]['sha'] and receipt['bytes'] == pin[kind]['size']
                assert receipt['sha256'] == row[kind + '_sha256']
                if kind == 'annotation':
                    body = annotation(row)
                    assert hashlib.sha1(b'blob ' + str(len(body)).encode() + b'\0' + body).hexdigest() == pin[kind]['sha']
        frozen = read('validation/summary.json')
        current = c.audit_saved(temporary, rows, control, annotation)
        assert current == frozen
        # Reconstruct with the exact pre-inference checker as well; normalize
        # only JSON representation, demonstrating unchanged matches and scores.
        initial_path = temporary / 'initial_checker.py'
        initial_path.write_text(initial)
        spec = importlib.util.spec_from_file_location('linear_initial_auditor', initial_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        assert json.loads(json.dumps(module.audit_saved(temporary, rows, control, annotation), allow_nan=False)) == frozen
        scored_pins = read('validation/before-label-scoring.json')
        assert scored_pins['annotations_numerically_parsed_by_this_process'] is False
        assert len(scored_pins['native_report_sha256']) == 144
        for name, pin in scored_pins['native_report_sha256'].items():
            assert sha((temporary / name).read_bytes()) == pin
        assert frozen['adopt_candidate'] is False
        counts = {p: {m: [v['overall'][k] for k in ('true_positives', 'false_positives', 'false_negatives')]
                      for m, v in group.items()} for p, group in frozen['groups'].items()}
        assert counts == dict(viewed_development=dict(triangle=[55, 39, 83], linear=[56, 36, 82]),
                              fresh_sample=dict(triangle=[99, 60, 169], linear=[101, 62, 167]))
        assert all(not v['quality_gates']['passed'] for g in frozen['groups'].values() for v in g.values())
        assert frozen['execution']['model_forwards'] == 576
    print(json.dumps(dict(pass_checks=True, indexed_files=len(index), preserved_entries=6510,
                          saved_model_forwards=576, fresh_images=16, adopt_candidate=False,
                          model_execution_this_check=False)))


if __name__ == '__main__':
    main()
