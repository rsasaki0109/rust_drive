#!/usr/bin/env python3
"""Independent OpenCV pixels, native graph parity and fixed BDD quality checks."""
import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys

import numpy as np
from PIL import Image
import bdd_score as matcher
import camera_depthwise_check as prior

HERE, ROOT = prior.HERE, prior.ROOT
require, digest, load, write, fresh = prior.require, prior.digest, prior.load, prior.write, prior.fresh
NEW = ['integrations/onnx/src/camera_linear.rs',
       'integrations/onnx/src/camera_linear_detector.rs',
       'integrations/onnx/src/bin/rustdriving-camera-linear.rs',
       'integrations/onnx/profile-linear.json']
MODES = ('triangle', 'linear')


def expected():
    return dict(prior.expected(), **{name: digest(ROOT / name) for name in NEW})


def native(binary, arguments, output, success=True):
    result = subprocess.run([str(binary), *map(str, arguments), '--output', str(output)],
                            text=True, capture_output=True, timeout=90)
    require(result.returncode == (0 if success else 2), 'native exit: ' + result.stderr[:500])
    if success:
        return load(output)
    require(not output.exists(), 'fatal input published output')
    return dict(returncode=result.returncode, stderr=result.stderr)


def records(data):
    old = prior.nano.load(HERE / 'bdd_protocol.json')
    design = load(HERE / 'profile-linear.json')
    require(digest(ROOT / 'artifacts/bdd-camera' / old['license']['file']) == old['license']['sha256'], 'original research-data license')
    pins = load(data / 'inputs.json')
    result = []
    for image in old['images']:
        result.append(dict(id=image['id'], partition='viewed_development', original_split=image['original_split'],
                           path=ROOT / 'artifacts/bdd-camera' / image['image']['file'],
                           annotation=ROOT / 'artifacts/bdd-camera' / image['annotation']['file'],
                           image_sha256=image['image']['sha256'],
                           annotation_sha256=image['annotation']['sha256']))
    for image in design['images']:
        names = [Path(image[kind]['path']).name for kind in ('image', 'annotation')]
        for kind, name in zip(('image', 'annotation'), names):
            body = (data / name).read_bytes()
            pin = image[kind]
            blob = hashlib.sha1(b'blob ' + str(len(body)).encode() + b'\0' + body).hexdigest()
            require(len(body) == pin['size'] and blob == pin['sha']
                    and digest(data / name) == pins[name]['sha256'], 'original Git input')
        result.append(dict(id=image['id'], partition='fresh_sample', original_split='val', path=data / names[0],
                           annotation=data / names[1], image_sha256=pins[names[0]]['sha256'],
                           annotation_sha256=pins[names[1]]['sha256']))
    require(len(result) == 24 and len({r['id'] for r in result}) == 24, 'complete fixed image inventory')
    for row in result:
        require(digest(row['path']) == row['image_sha256']
                and digest(row['annotation']) == row['annotation_sha256'], 'unchanged image/annotation')
    return result


def cv_reference(rgb, size, cv2):
    height, width = rgb.shape[:2]
    ratio = min(size / height, size / width)
    rw, rh = int(width * ratio), int(height * ratio)
    require(rw > 0 and rh > 0, 'upstream resize nonempty')
    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    resized = cv2.resize(bgr, (rw, rh), interpolation=cv2.INTER_LINEAR)
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[:rh, :rw] = resized
    values = np.ascontiguousarray(padded.transpose(2, 0, 1), dtype='<f4')
    return dict(rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(), resized=[rw, rh],
                decode_ratio_f32_bits=struct.pack('<f', ratio).hex(),
                tensor_f32le_sha256=hashlib.sha256(values.tobytes()).hexdigest(),
                compared_values=values.size)


def check_pixels(report, rgb, size, cv2):
    require(report['algorithm'] == 'yolox_generic_linear_preprocessing_v1'
            and report['compiled_sources'] == expected(), 'pixel producer/source closure')
    require(report['model_read'] is False and report['inference_run'] is False
            and report['control_permission'] is False, 'model-free pixel diagnostic')
    require((report['width'], report['height']) == (rgb.shape[1], rgb.shape[0])
            and report['input_size'] == size, 'pixel diagnostic geometry')
    oracle = cv_reference(rgb, size, cv2)
    require(all(report[key] == oracle[key] for key in
                ('rgb_sha256', 'resized', 'tensor_f32le_sha256')), 'every OpenCV input-tensor byte')
    require(struct.pack('<f', report['decode_ratio_f32']).hex() == oracle['decode_ratio_f32_bits'],
            'original f32 decode-ratio bits')
    return oracle


def opencv():
    import cv2
    require(cv2.__version__ == '4.13.0', 'pinned OpenCV reference')
    cv2.setUseOptimized(False)
    cv2.ipp.setUseIPP(False)
    cv2.ocl.setUseOpenCL(False)
    cv2.setNumThreads(1)
    return cv2


def controls(args):
    cv2 = opencv()
    out = fresh(args.output)
    geometry = [(1, 1, 1), (1, 3, 7), (3, 1, 7), (2, 2, 1), (4, 4, 2),
                (8, 8, 8), (17, 31, 16), (31, 17, 16), (63, 65, 416),
                (1280, 720, 416), (720, 1280, 416), (720, 720, 416),
                (641, 640, 416), (1920, 1080, 416), (7, 13, 416), (639, 638, 416)]
    rng = np.random.default_rng(20261017)
    geometry += [(int(w), int(h), int(s)) for w, h, s in
                 zip(rng.integers(2, 101, 32), rng.integers(2, 101, 32), rng.integers(32, 641, 32))]
    cases = []
    for index, (width, height, size) in enumerate(geometry):
        rgb = rng.integers(0, 256, (height, width, 3), dtype=np.uint8)
        source = out / f'{index}-input.png'
        Image.fromarray(rgb).save(source)
        report = native(args.binary, ['--preprocess', source, '--input-size', size], out / f'{index}-actual.json')
        oracle = check_pixels(report, rgb, size, cv2)
        cases.append(dict(index=index, width=width, height=height, size=size, **oracle))
    # These images are decoded once by the same Rust reader used for inference.
    # Lossless local exports let OpenCV inspect those exact pixels, independently
    # of any differences between JPEG decoder libraries. Never archive exports.
    actual_pixels = {}
    for row in records(args.data):
        path = out / (row['id'] + '-pixel-export.json')
        report = native(args.binary, ['--preprocess', row['path'], '--input-size', 416,
                                     '--include-pixels'], path)
        png = bytes.fromhex(report['decoded_rgb_png_hex'])
        rgb = np.asarray(Image.open(io.BytesIO(png)).convert('RGB'))
        check_pixels(report, rgb, 416, cv2)
        views = []
        for rectangle in prior.nano.tiles.rectangles(rgb.shape[1], rgb.shape[0]):
            x, y, width, height = [rectangle[k] for k in ('x', 'y', 'width', 'height')]
            cropped = np.ascontiguousarray(rgb[y:y + height, x:x + width])
            views.append(dict(rectangle=rectangle, **cv_reference(cropped, 416, cv2)))
        actual_pixels[row['id']] = dict(source_image_sha256=row['image_sha256'], views=views,
                                        exported_png_sha256=hashlib.sha256(png).hexdigest(),
                                        raw_local_export_report_sha256=digest(path))
        # Keep only hashes in the bounded report packet; local exports stay ignored.
        public = {k: v for k, v in report.items() if k != 'decoded_rgb_png_hex'}
        write(out / (row['id'] + '-pixels.json'), public)
    invalid = []
    tiny = out / '0-input.png'
    for index, (name, command) in enumerate([
        ('zero-size', ['--preprocess', tiny, '--input-size', 0]),
        ('oversize-input', ['--preprocess', tiny, '--input-size', 641]),
        ('invalid-size', ['--preprocess', tiny, '--input-size', 'x']),
        ('missing-image', ['--preprocess', out / 'absent', '--input-size', 416]),
        ('bad-mode', ['--model', 'absent', '--image', tiny, '--preprocessing', 'invalid']),
        ('bad-model', ['--model', tiny, '--image', tiny, '--preprocessing', 'linear']),
    ]):
        receipt = native(args.binary, command, out / f'invalid-{index}.json', False)
        invalid.append(dict(name=name, **receipt))
    keep = out / 'preserve.txt'
    keep.write_text('preserve\n')
    result = subprocess.run([str(args.binary), '--preprocess', str(tiny), '--input-size', '1',
                             '--output', str(keep)], capture_output=True, text=True)
    require(result.returncode == 2 and keep.read_text() == 'preserve\n', 'existing output preserved')
    summary = dict(schema_version=1, checker_sha256=digest(__file__),
                   executable_sha256=digest(args.binary), compiled_sources=expected(),
                   reference=dict(opencv=cv2.__version__, numpy=np.__version__,
                                  optimized=cv2.useOptimized(), ipp=cv2.ipp.useIPP(),
                                  opencl=cv2.ocl.useOpenCL(), threads=cv2.getNumThreads()),
                   generated_cases=cases, actual_image_pixels=actual_pixels,
                   compared_generated_values=sum(r['compared_values'] for r in cases),
                   invalid_inputs=invalid, existing_output_preserved=True,
                   model_forwards=0, pass_controls=True)
    write(out / 'summary.json', summary)
    print(json.dumps(dict(pixel_controls=len(cases), real_images=len(actual_pixels), model_forwards=0)), flush=True)


def audit_image(report, row, pixel, mode, reference=False):
    require(report['schema_version'] == 1 and report['algorithm'] == 'nano_controlled_preprocessing_analysis_v1', 'own analysis identity')
    require(report['compiled_sources'] == expected() and report['profile_sha256'] == digest(HERE / 'profile-linear.json'), 'analysis source/profile closure')
    require(report['preprocessing'] == mode and report['original_graph_reference'] is reference
            and report['control_permission'] is False, 'declared preprocessing/reference and no control')
    require(report['source_image_sha256'] == row['image_sha256'], 'original source image')
    base = report['base_prediction']
    require(base['algorithm'] == 'nano_controlled_preprocessing_fixed_views_v1'
            and base['compiled_sources'] == expected(), 'own complete fixed-view producer')
    require(base['source_image_sha256'] == row['image_sha256']
            and base['model_sha256'] == prior.MODEL_SHA, 'fixed image/model')
    require(len(base['views']) == len(pixel['views']) == 4
            and (base['width'], base['height']) == (1280, 720), 'complete BDD view geometry')
    require(prior.nano.exact_f32(base['score_threshold'], .3)
            and prior.nano.exact_f32(base['nms_iou_threshold'], .45), 'fixed merge thresholds')
    for view, oracle in zip(base['views'], pixel['views']):
        require(view['rectangle'] == oracle['rectangle'] and view['rgb_sha256'] == oracle['rgb_sha256'], 'independent original crop pixels')
        p = view['prediction']
        require(p['algorithm'] == 'pinned_yolox_nano_controlled_preprocessing_v1'
                and p['model_profile'] == 'yolox-nano-controlled-preprocessing-v1'
                and p['preprocessing'] == mode and p['original_graph_reference'] is reference,
                'detector mode and graph identity')
        require(p['compiled_sources'] == expected()
                and p['model_profile_sha256'] == digest(HERE / 'profile-linear.json'), 'complete detector source closure')
        require(p['model_sha256'] == prior.MODEL_SHA and p['input_shape'] == [1, 3, 416, 416]
                and p['output_shape'] == [1, 3549, 85], 'original model and tensor geometry')
        require((p['width'], p['height']) == (oracle['rectangle']['width'], oracle['rectangle']['height'])
                and p['source_image_sha256'] is None, 'local detector coordinates')
        require(prior.nano.exact_f32(p['score_threshold'], .3)
                and prior.nano.exact_f32(p['nms_iou_threshold'], .45), 'fixed detector thresholds')
        for key in ('input_tensor_f32le_sha256', 'output_tensor_f32le_sha256'):
            require(isinstance(p[key], str) and re.fullmatch('[0-9a-f]{64}', p[key]) is not None, 'canonical tensor hash')
        if mode == 'linear':
            require(p['input_tensor_f32le_sha256'] == oracle['tensor_f32le_sha256']
                    and p['resized'] == oracle['resized'], 'every real crop tensor agrees with independent cv2')
        require(all(type(p[k]) in (float, int) and math.isfinite(p[k]) and p[k] > 0
                    for k in ('inference_seconds', 'model_load_seconds')), 'actual finite observations')
    fixture = dict(width=base['width'], height=base['height'],
                   views=[dict(rectangle=v['rectangle'], detections=v['prediction']['detections']) for v in base['views']])
    prior.nano.tiles.same(prior.nano.tiles.merge(fixture), {k: base[k] for k in ('detections', 'detection_origins', 'merge')})
    prior.nano.tiles.same(prior.nano.consistency.oracle(fixture, prior.nano.tiles), report['result'])
    execution = report['execution']
    require({k: v for k, v in execution.items() if k != 'analysis_wall_s'} ==
            dict(model_read=True, images_read=1, model_forwards=4, inference_run=True,
                 view_workers=4, shared_model_plans=1), 'bounded actual inference inventory')
    require(math.isfinite(execution['analysis_wall_s']) and execution['analysis_wall_s'] >=
            max(v['prediction']['inference_seconds'] for v in base['views']), 'analysis covers every forward')


def compare_pair(first, second):
    for a, b in zip(first['base_prediction']['views'], second['base_prediction']['views']):
        for field in ('input_tensor_f32le_sha256', 'output_tensor_f32le_sha256', 'resized'):
            require(a['prediction'][field] == b['prediction'][field], 'current repeat/reference tensor hashes and resize')
        prior.compare(a['prediction']['detections'], b['prediction']['detections'], 'every current per-view measurement bit')
    for field in ('detections', 'detection_origins', 'merge'):
        prior.compare(first['base_prediction'][field], second['base_prediction'][field], 'current merged result')
    prior.compare(first['result'], second['result'], 'current consistency result')


def score(rows, predictions, read_annotation=None):
    design = load(HERE / 'profile-linear.json')
    read_annotation = read_annotation or (lambda row: Path(row['annotation']).read_bytes())
    images = []
    for row in rows:
        body = read_annotation(row)
        require(hashlib.sha256(body).hexdigest() == row['annotation_sha256'], 'original annotation hash')
        annotation = json.loads(body)
        require(annotation['name'] == row['id'] and len(annotation['frames']) == 1
                and annotation['frames'][0]['timestamp'] == 10000, 'original legacy frame identity')
        refs, signs, seen = [], [], set()
        for label in annotation['frames'][0]['objects']:
            require(label['id'] not in seen, 'unique original reference identities')
            seen.add(label['id'])
            if 'box2d' not in label:
                continue
            box = [label['box2d'][key] for key in ('x1', 'y1', 'x2', 'y2')]
            require(matcher.valid_box(box, 1280, 720), 'original reference bounds')
            ref = dict(id=label['id'], category=label['category'], bbox=box, attributes=label['attributes'])
            if label['category'] in design['class_mapping']:
                refs.append(dict(ref, class_index=design['class_mapping'][label['category']]))
            else:
                require(label['category'] in design['unscored_category_reason'], 'frozen category scope')
                signs.append(ref)
        emitted = predictions[row['id']]['result']['detections']
        require(len(emitted) <= 300, 'bounded final predictions')
        for d in emitted:
            require(type(d['class_index']) is int and 0 <= d['class_index'] < 80
                    and matcher.valid_box(d['bbox'], 1280, 720)
                    and type(d['confidence']) in (float, int) and math.isfinite(d['confidence'])
                    and float(np.float32(.3)) <= d['confidence'] <= 1, 'valid emitted predictions')
        selected = [d for d in emitted if d['class_index'] in design['class_mapping'].values()]
        matches, fp, fn = matcher.match_boxes(refs, selected, .5)
        images.append(dict(image_id=row['id'], partition=row['partition'], original_split=row['original_split'],
                           attributes=annotation['attributes'], reference_count=len(refs),
                           excluded_original_sign_boxes=signs, matches=matches,
                           false_positives=fp, false_negatives=fn,
                           original_annotation_sha256=row['annotation_sha256'],
                           original_image_sha256=row['image_sha256']))
    classes = sorted(set(design['class_mapping'].values()))
    metrics = dict(overall=matcher.aggregate(images),
                   per_class={str(k): matcher.aggregate(images, k) for k in classes}, images=images)
    gate = design['quality_gates']
    overall = metrics['overall']
    checks = dict(precision=overall['precision'] is not None and overall['precision'] >= gate['minimum_precision'],
                  recall=overall['recall'] is not None and overall['recall'] >= gate['minimum_recall'],
                  per_present_class_recall=all(v['recall'] >= gate['minimum_per_class_recall_when_present']
                                              for v in metrics['per_class'].values() if v['recall'] is not None))
    metrics['quality_gates'] = dict(thresholds=gate, checks=checks, passed=all(checks.values()))
    return metrics


def audit_saved(out, rows, control, read_annotation=None):
    reports = {}
    for repeat in (0, 1):
        for mode in MODES:
            reports[repeat, mode] = {}
            for row in rows:
                report = load(out / f'pass-{repeat}' / mode / (row['id'] + '.json'))
                audit_image(report, row, control['actual_image_pixels'][row['id']], mode)
                reports[repeat, mode][row['id']] = report
    for row in rows:
        for mode in MODES:
            compare_pair(reports[0, mode][row['id']], reports[1, mode][row['id']])
            reference = load(out / 'original-graph' / mode / (row['id'] + '.json'))
            audit_image(reference, row, control['actual_image_pixels'][row['id']], mode, True)
            compare_pair(reports[0, mode][row['id']], reference)
    groups = {}
    for partition in ('viewed_development', 'fresh_sample'):
        selected = [r for r in rows if r['partition'] == partition]
        groups[partition] = {mode: score(selected, reports[0, mode], read_annotation) for mode in MODES}
    relative = all(groups[p]['linear']['overall'][metric] >= groups[p]['triangle']['overall'][metric]
                   for p in groups for metric in ('precision', 'recall'))
    # Explicitly retain gained and lost original reference identities.
    changes = {}
    for partition, group in groups.items():
        ids = lambda mode: {(r['image_id'], m['reference_id']) for r in group[mode]['images'] for m in r['matches']}
        before, after = ids('triangle'), ids('linear')
        changes[partition] = dict(retained=len(before & after), gained=[list(x) for x in sorted(after - before)], lost=[list(x) for x in sorted(before - after)])
    return dict(schema_version=1, protocol_sha256=digest(HERE / 'profile-linear.json'), groups=groups,
                original_reference_changes=changes, relative_no_precision_or_recall_regression=relative,
                adopt_candidate=relative and groups['fresh_sample']['linear']['quality_gates']['passed'],
                execution=dict(native_image_runs=144, model_forwards=576, current_original_graph_runs=48,
                               repeat_raw_tensor_hash_pairs=192, original_graph_raw_tensor_hash_pairs=192,
                               unique_raw_values_per_tensor=301665, independent_crop_pixel_checks=96),
                timing_claim=False, control_permission=False)


def run(args):
    control = load(args.controls / 'summary.json')
    require(control['pass_controls'] and control['checker_sha256'] == digest(__file__)
            and control['executable_sha256'] == digest(args.binary)
            and control['compiled_sources'] == expected(), 'same controlled source/executable')
    rows = records(args.data)
    out = fresh(args.output)
    before = dict(checker_sha256=digest(__file__), executable_sha256=digest(args.binary),
                  fetcher_sha256=digest(HERE / 'camera_linear_fetch.py'),
                  compiled_sources=expected(), protocol_sha256=digest(HERE / 'profile-linear.json'),
                  control_summary_sha256=digest(args.controls / 'summary.json'),
                  input_rows=[{k: str(v) if isinstance(v, Path) else v for k, v in row.items()} for row in rows],
                  annotations_numerically_parsed_by_this_process=False,
                  prior_valid_model_forwards_this_experiment=0, warmup_model_forwards=0,
                  purpose='Fixed preprocessing quality comparison, not a latency or availability experiment')
    write(out / 'before-inference.json', before)
    for repeat in (0, 1):
        for mode in MODES:
            fresh(out / f'pass-{repeat}' / mode)
        for index, row in enumerate(rows):
            order = ('linear', 'triangle') if (index + repeat) % 2 == 0 else MODES
            for mode in order:
                native(args.binary, ['--model', args.model, '--image', row['path'], '--preprocessing', mode],
                       out / f'pass-{repeat}' / mode / (row['id'] + '.json'))
            print(f"pass {repeat} {row['partition']} {row['id']}: both modes retained", flush=True)
    for mode in MODES:
        fresh(out / 'original-graph' / mode)
    for index, row in enumerate(rows):
        for mode in (('linear', 'triangle') if index % 2 == 0 else MODES):
            native(args.binary, ['--model', args.model, '--image', row['path'], '--preprocessing', mode, '--original-plan'],
                   out / 'original-graph' / mode / (row['id'] + '.json'))
    write(out / 'before-label-scoring.json', dict(annotations_numerically_parsed_by_this_process=False,
          native_report_sha256={str(p.relative_to(out)): digest(p) for p in sorted(out.glob('*/*/*.json'))}))
    summary = audit_saved(out, rows, control)
    require(before['compiled_sources'] == expected() and before['checker_sha256'] == digest(__file__)
            and before['executable_sha256'] == digest(args.binary)
            and before['fetcher_sha256'] == digest(HERE / 'camera_linear_fetch.py'), 'source/executable unchanged throughout execution')
    records(args.data)
    write(out / 'summary.json', summary)
    print(json.dumps({p: {m: v['overall'] for m, v in group.items()} for p, group in summary['groups'].items()}), flush=True)
    print(json.dumps(dict(adopt_candidate=summary['adopt_candidate'], model_forwards=576)), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('controls', 'run', 'audit'))
    parser.add_argument('--binary', type=Path, default=ROOT / 'integrations/onnx/target/release/rustdriving-camera-linear')
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--data', type=Path, default=ROOT / 'artifacts/camera-linear-v1/data')
    parser.add_argument('--controls', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.binary = args.binary.resolve()
    if args.command == 'controls':
        controls(args)
    elif args.command == 'run':
        require(args.controls is not None, 'controls required')
        run(args)
    else:
        result = audit_saved(args.output, records(args.data), load(args.controls / 'summary.json'))
        require(result == load(args.output / 'summary.json'), 'saved independently reconstructed summary')
        print(json.dumps(dict(pass_audit=True, model_execution_this_check=False)))


if __name__ == '__main__':
    main()
