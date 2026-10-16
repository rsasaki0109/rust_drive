#!/usr/bin/env python3
"""Check one-command camera analysis, output failures and unchanged viewed results."""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import zlib

from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
CONSISTENCY_SHA = '37a7a41953d9d4804b2d4796c6dc89054c6633dc468994b45227adbc4e85b60e'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def context():
    path = HERE / 'bdd_vehicle_consistency_check.py'
    require(digest(path) == CONSISTENCY_SHA, 'immutable consistency checker')
    spec = importlib.util.spec_from_file_location('analysis_consistency_oracle', path)
    support = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(support)
    return support, support.context()


def invoke(binary, arguments, destination, expected=0, error=None):
    completed = subprocess.run(
        [str(binary), *map(str, arguments), '--output', str(destination)],
        capture_output=True, text=True, timeout=180,
    )
    require(completed.returncode == expected, 'native exit: ' + completed.stderr[-2000:])
    if error:
        require(error in completed.stderr, 'expected failure stage: ' + completed.stderr)
    return dict(exit=completed.returncode, stderr=completed.stderr)


def receipt(args, support, output):
    invoke(args.binary, ['--executable-receipt'], output)
    result = support.load(output)
    require(result['executable_sha256'] == digest(args.binary), 'executable receipt')
    require(result['execution'] == dict(model_read=False, images_read=0,
                                        model_forwards=0, inference_run=False), 'receipt has no inference')
    for path, pin in result['compiled_sources'].items():
        require(digest(ROOT / path) == pin, 'compiled source: ' + path)
    return result


def controls(args):
    support, _ = context()
    out = support.fresh(args.output)
    inputs = support.fresh(out / 'inputs')
    valid = inputs / 'valid.png'
    Image.new('RGB', (1, 1), (12, 34, 56)).save(valid)
    invalid = inputs / 'invalid.png'
    invalid.write_bytes(b'not an image')
    huge = inputs / 'too-many-bytes.png'
    with huge.open('xb') as stream:
        stream.truncate(16 * 1024 * 1024 + 1)
    pixels = bytearray(valid.read_bytes())
    pixels[16:24] = struct.pack('>II', 5000, 5000)
    pixels[29:33] = struct.pack('>I', zlib.crc32(pixels[12:29]))
    (inputs / 'too-many-pixels.png').write_bytes(pixels)
    bad_model = inputs / 'bad-model.onnx'
    bad_model.write_bytes(b'invalid model')
    model_sha = inputs / 'wrong-sha.onnx'
    with model_sha.open('xb') as stream:
        stream.truncate(35_858_002)
    symlink = inputs / 'symlink.png'
    symlink.symlink_to(valid.name)
    fifo = inputs / 'fifo.png'
    os.mkfifo(fifo)
    rows = []

    def reject(name, image, model, error):
        destination = out / (name + '-unexpected.json')
        result = invoke(args.binary, ['--model', model, '--image', image], destination, 2, error)
        require(not destination.exists() and not destination.is_symlink(), 'failure published a report')
        require(not list(out.glob('.rustdriving-camera-*.partial')), 'failure left staging')
        rows.append(dict(name=name, **result))

    for name, image, error in [
        ('missing-image', inputs / 'missing', 'read image:'),
        ('invalid-image', invalid, 'read image:'),
        ('image-directory', inputs, 'regular non-symlink'),
        ('image-symlink', symlink, 'regular non-symlink'),
        ('image-fifo', fifo, 'regular non-symlink'),
        ('encoded-budget', huge, 'encoded-byte budget'),
        ('pixel-budget', inputs / 'too-many-pixels.png', 'pixel budget'),
    ]:
        reject(name, image, inputs / 'missing-model', error)
    reject('missing-model', valid, inputs / 'missing-model', 'load model:')
    reject('model-size', valid, bad_model, 'model size')
    reject('model-sha', valid, model_sha, 'model SHA-256')
    existing = out / 'existing.json'
    existing.write_bytes(b'preserve existing output\n')
    invoke(args.binary, ['--model', 'missing', '--image', 'missing'], existing, 2, 'fresh output')
    require(existing.read_bytes() == b'preserve existing output\n', 'existing output changed')
    dangling = out / 'dangling.json'
    dangling.symlink_to('absent.json')
    invoke(args.binary, ['--model', 'missing', '--image', 'missing'], dangling, 2, 'fresh output')
    require(dangling.is_symlink() and os.readlink(dangling) == 'absent.json', 'dangling link changed')
    directory = support.fresh(out / 'existing-directory')
    invoke(args.binary, ['--executable-receipt'], directory, 2, 'fresh output')
    invoke(args.binary, ['--model', 'missing', '--image', 'missing'],
           out / 'missing-parent' / 'report.json', 2, 'prepare output:')
    invoke(args.binary, ['--wrong-option'], out / 'bad-cli.json', 2, 'expected --model')
    require(not (out / 'bad-cli.json').exists(), 'invalid arguments published report')
    # The child knows the PID that exec preserves. Its destination would equal
    # the first internal staging name, so the publisher must choose another.
    collision = support.fresh(out / 'staging-name-collision')
    code = ('import os,sys; from pathlib import Path; '
            'p=Path(sys.argv[2])/f".rustdriving-camera-{os.getpid()}-0.partial"; '
            'os.execv(sys.argv[1],[sys.argv[1],"--executable-receipt","--output",str(p)])')
    completed = subprocess.run([sys.executable, '-c', code, str(args.binary), str(collision)],
                               capture_output=True, text=True, timeout=180)
    require(completed.returncode == 0, 'destination/staging collision: ' + completed.stderr)
    published = list(collision.iterdir())
    require(len(published) == 1 and support.load(published[0])['execution']['model_forwards'] == 0,
            'collision must publish one complete receipt without leftover staging')
    published[0].rename(out / 'staging-name-collision-receipt.json')
    actual = receipt(args, support, out / 'executable.json')
    require(not list(out.glob('.rustdriving-camera-*.partial')), 'staging left after controls')
    support.write(out / 'summary.json', dict(
        schema_version=1, pass_controls=True, cases=rows, admission_checks=5,
        executable_sha256=actual['executable_sha256'], checker_sha256=digest(__file__),
        compiled_sources=actual['compiled_sources'], model_forwards=0,
        oversized_inputs_generated=True, destination_staging_name_collision_passed=True,
    ))


def run(args):
    support, (tiles, old, design, matcher) = context()
    control = support.load(args.controls / 'summary.json')
    require(control['pass_controls'] and control['checker_sha256'] == digest(__file__)
            and control['executable_sha256'] == digest(args.binary), 'matching controls')
    out = support.fresh(args.output)
    executable = receipt(args, support, out / 'executable.json')
    image_pins = {image['id']: digest(args.data / (image['id'] + '.jpg')) for image in design['images']}
    require(all(image_pins[image['id']] == image['image']['sha256'] for image in design['images']), 'fixed images')
    require(digest(args.model) == executable['model_sha256'], 'fixed model')
    support.write(out / 'before-inference.json', dict(
        compiled_sources=executable['compiled_sources'], executable_sha256=digest(args.binary),
        checker_sha256=digest(__file__), profile_sha256=digest(HERE / 'profile-analysis.json'),
        image_sha256=image_pins, model_sha256=digest(args.model),
        data_role='all eight images previously viewed; existing thresholds and policy unchanged',
        reference_annotations_parsed_in_this_process=False,
    ))
    passes = []
    for mode in ('first', 'repeat'):
        directory = support.fresh(out / mode)
        reports = {}
        for image in design['images']:
            destination = directory / (image['id'] + '.json')
            invoke(args.binary, ['--model', args.model, '--image', args.data / (image['id'] + '.jpg')], destination)
            reports[image['id']] = support.load(destination)
            require(not list(directory.glob('.rustdriving-camera-*.partial')), 'successful run left staging')
        passes.append(reports)
    support.write(out / 'before-reference-parsing.json', dict(
        reference_annotations_parsed_in_this_process=False,
        reports={str(p.relative_to(out)): digest(p) for p in sorted(out.glob('*/*.json'))},
    ))
    projected, selected = {}, {}
    for image in design['images']:
        first, repeat = (mode[image['id']] for mode in passes)
        require(first['schema_version'] == 1 and first['algorithm'] == 'yolox_s_image_analysis_v1', 'analysis identity')
        require(first['compiled_sources'] == executable['compiled_sources']
                and first['source_image_sha256'] == image['image']['sha256'], 'analysis provenance')
        base = first['base_prediction']
        require(first['execution'] == dict(model_read=True, images_read=1,
                                           model_forwards=len(base['views']), inference_run=True), 'execution disclosure')
        require(1 <= len(base['views']) <= 4, 'bounded inference')
        tiles.same(tiles.without_timing(first), tiles.without_timing(repeat))
        fixture = dict(width=base['width'], height=base['height'], views=[
            dict(rectangle=v['rectangle'], detections=v['prediction']['detections']) for v in base['views']])
        expected = support.oracle(fixture, tiles)
        tiles.same(expected, first['result'])
        historical = support.load(ROOT / 'assets/yolox-vehicle-consistency-v1/final/first' / (image['id'] + '.json'))
        tiles.historical_same(historical['result'], first['result'])
        raw = support.load(ROOT / 'assets/yolox-s-tiles-v1/final-regression/first' / (image['id'] + '.json'))
        tiles.historical_same(tiles.without_timing(raw), tiles.without_timing(base))
        inner = copy.deepcopy(base['views'][0]['prediction'])
        tiles.thresholds(inner)
        inner['source_image_sha256'] = image['image']['sha256']
        inner['detections'] = expected['detections']
        projected[image['id']] = inner
        selected[image['id']] = dict(detections=expected['detections'])
    compiled = next(iter(projected.values()))['compiled_sources']
    scored = old.score(args.data, projected, design, compiled, matcher)
    baseline = support.load(ROOT / 'assets/yolox-vehicle-consistency-v1/results.json')
    identities = lambda doc: {(row['image_id'], match['reference_id'])
                             for row in doc['images'] for match in row['matches']}
    require(identities(scored) == identities(baseline), 'all matched identities preserved')
    tiles.historical_same(baseline['overall'], scored['overall'])
    tiles.historical_same(baseline['per_class'], scored['per_class'])
    require(control['compiled_sources'] == executable['compiled_sources']
            and digest(args.binary) == executable['executable_sha256']
            and digest(__file__) == control['checker_sha256'], 'sources unchanged during inference')
    for path, pin in executable['compiled_sources'].items():
        require(digest(ROOT / path) == pin, 'source changed during inference')
    scored.update(
        schema_version=1, algorithm='yolox_s_image_analysis_v1',
        data_role='viewed development; no fresh holdout',
        quality_gates=dict(design['evaluation']['quality_gates'], **{'pass': scored['viewed_quality_gates_passed']}),
        calibration=tiles.calibration(scored, selected, design['class_mapping']),
        integrity=dict(pass_checks=True, independent_reconstruction=True, repeated_f32_identical=True,
                       unchanged_historical_accuracy=True, native_image_processes=16,
                       model_forward_passes=sum(v['execution']['model_forwards'] for mode in passes for v in mode.values())),
    )
    support.write(out / 'results.json', scored)
    print(dict(overall=scored['overall'], integrity=scored['integrity'], original_quality_passed=scored['quality_gates']['pass']))
    # This command evaluates engineering equivalence. Accuracy remains explicitly failed.
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('controls', 'run'))
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--controls', type=Path)
    parser.add_argument('--model', type=Path, default=ROOT / 'artifacts/camera-model-s/yolox_s.onnx')
    parser.add_argument('--data', type=Path, default=ROOT / 'artifacts/bdd-camera')
    args = parser.parse_args()
    args.binary = args.binary.absolute()
    if args.command == 'controls':
        controls(args)
        return 0
    require(args.controls is not None, 'controls required')
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
