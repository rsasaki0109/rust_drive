#!/usr/bin/env python3
"""Frozen, provided-file KITTI sparse-depth evaluation; no automatic downloads.

Conditional errors and reference coverage are separate. There is no filling,
cropping, depth clipping, benchmark submission, or implicit accuracy pass gate.
"""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
import time

import numpy as np
from PIL import Image

MAX_JSON = 64 * 1024 * 1024
MAX_PNG = 32 * 1024 * 1024
PROTOCOL = dict(schema_version=1, reference_encoding='grayscale uint16 PNG; metres = value / 256',
                invalid_reference_value=0, crop=None, depth_clip=None, interpolation=None,
                comparison='exact same pixel on identical rectified full canvases',
                weighting='all comparable pixels pooled across all selected frames',
                error_scope='conditional on prediction/reference overlap',
                availability='overlap pixels / all valid reference pixels',
                quality_gate=None, official_benchmark_equivalence=False,
                max_frames=20, max_frame_points=200000)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def strict_json(data):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, 'duplicate JSON key: '+key)
            result[key] = value
        return result

    def nonfinite(value):
        raise ValueError('nonfinite JSON constant: '+value)
    return json.loads(data, object_pairs_hook=pairs, parse_constant=nonfinite)


def bounded(path, maximum):
    require(path.is_file() and not path.is_symlink(), 'regular file required: '+str(path))
    require(path.stat().st_size <= maximum, 'file byte bound: '+str(path))
    with path.open('rb') as stream:
        data = stream.read(maximum+1)
    require(len(data) <= maximum, 'file grew over byte bound')
    return data


def local(root, name):
    require(type(name) is str and name and '\\' not in name, 'relative POSIX file name required')
    parts = name.split('/')
    require(not name.startswith('/') and not any(p in ('', '.', '..') for p in parts),
            'input must remain under the selected root')
    result = root
    for part in parts:
        result = result/part
        require(not result.is_symlink(), 'input symlink not allowed')
    return result


def fresh_directory(path):
    require(not any(p in ('.', '..') for p in str(path).split('/')), 'plain output path required')
    path = path.absolute()
    require(not path.exists() and not path.is_symlink(), 'fresh output directory required')
    require(not any(p.is_symlink() for p in path.parents), 'real output parents required')
    path.mkdir(parents=True, exist_ok=False)
    return path


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def inventory(manifest_bytes, root):
    """Hash encoded sources without decoding reference PNGs or scoring any pose."""
    manifest = strict_json(manifest_bytes)
    require(manifest['schema_version'] == 1, 'manifest version')
    require(manifest['reference_kind'] == 'kitti_annotated_depth', 'annotated reference required')
    require(manifest['reference_encoding'] == 'uint16_png_div_256', 'fixed PNG encoding required')
    require(manifest['split'] in ('train', 'val'), 'labelled train/val split required')
    require(type(manifest['recording_id']) is str and
            re.fullmatch(r'\d{4}_\d{2}_\d{2}_drive_\d{4}_sync', manifest['recording_id']),
            'explicit KITTI recording identity required')
    require(type(manifest['camera_index']) is int and manifest['camera_index'] in (2, 3),
            'annotated left/right color camera 2 or 3 required')
    require(type(manifest['provenance']) is str and manifest['provenance'].strip(),
            'record supplied-file provenance; a declaration does not authenticate it')
    require(set(manifest['calibration']) == {'camera', 'velo', 'imu'}, 'three calibration files required')
    frames = manifest['frames']
    require(type(frames) is list and 1 <= len(frames) <= PROTOCOL['max_frames'], 'select 1..20 frames')
    previous = None
    receipts = {}

    def receipt(name, maximum):
        body = bounded(local(root, name), maximum)
        receipts[name] = dict(bytes=len(body), sha256=sha(body))

    for name in manifest['calibration'].values():
        receipt(name, 65536)
    used = set(manifest['calibration'].values())
    for frame in frames:
        identity = frame['frame_id']
        require(type(identity) is str and re.fullmatch(r'\d{10}', identity), 'ten-digit frame ID')
        require(previous is None or previous < identity, 'strict frame order; no duplicates')
        previous = identity
        for field in ('points', 'reference_depth'):
            name = frame[field]
            require(type(name) is str and name not in used, 'distinct frame inputs required')
            used.add(name)
        require(Path(frame['points']).name == identity+'.bin', 'point file/frame ID mismatch')
        require(Path(frame['reference_depth']).name == identity+'.png', 'reference/frame ID mismatch')
        receipt(frame['points'], 16*PROTOCOL['max_frame_points'])
        receipt(frame['reference_depth'], MAX_PNG)
    return manifest, receipts


def decode_reference(raw, size):
    require(len(raw) >= 33 and raw[:8] == b'\x89PNG\r\n\x1a\n' and raw[12:16] == b'IHDR',
            'reference must be a PNG with IHDR')
    width, height, bits, color = struct.unpack('>IIBB', raw[16:26])
    require([width, height] == size and 1 <= width <= 8192 and 1 <= height <= 8192
            and width*height <= 16777216, 'identical full canvas required; no implicit crop/resize')
    require(bits == 16 and color == 0, 'reference must be grayscale 16-bit PNG')
    with Image.open(io.BytesIO(raw)) as image:
        require(image.format == 'PNG' and image.size == (width, height), 'decoded PNG dimensions')
        image.load()
        array = np.asarray(image)
    require(array.shape == (height, width) and array.dtype.kind in 'ui'
            and np.all(array <= 65535), 'uint16 reference samples')
    return array


def score(report, reference_raw, camera):
    """Score sparse cells directly. Missing predictions never get a filled depth."""
    require(report['schema_version'] == 1 and report['kind'] == 'kitti_sparse_axial_depth'
            and report['camera_index'] == camera, 'prediction schema/camera')
    depth = report['depth']
    size = depth['rectified_size']
    require(type(size) is list and len(size) == 2 and all(type(v) is int for v in size), 'integer canvas')
    reference = decode_reference(reference_raw, size)
    pixels = depth['pixels']
    require(type(pixels) is list and len(pixels) <= 200000, 'sparse prediction bound')
    previous, winners, errors, inverse_errors = None, set(), [], []
    support = 0
    for cell in pixels:
        coordinate = cell['pixel']
        require(type(coordinate) is list and len(coordinate) == 2
                and all(type(v) is int for v in coordinate), 'integer pixel cell')
        x, y = coordinate
        require(0 <= x < size[0] and 0 <= y < size[1], 'pixel outside canvas')
        require(previous is None or previous < (y, x), 'unique row-major cells required')
        previous = (y, x)
        z, index, count = cell['depth_m'], cell['point_index'], cell['support_count']
        require(type(z) in (int, float) and math.isfinite(z) and z > 0, 'positive finite axial depth')
        require(type(index) is int and 0 <= index < 200000 and index not in winners, 'unique source row')
        require(type(count) is int and count > 0, 'positive cell support')
        winners.add(index)
        support += count
        truth = int(reference[y, x])/256.
        if truth > 0:
            errors.append(z-truth)
            inverse_errors.append(1000.*(1./z-1./truth))
    summary = depth['summary']
    require(summary['occupied_pixels'] == len(pixels) and summary['inframe_points'] == support
            and summary['occluded_points'] == support-len(pixels), 'consistent prediction counts')
    require(all(type(summary[k]) is int and summary[k] >= 0 for k in summary), 'nonnegative counts')
    require(summary['points'] == support+summary['outside']+summary['behind'] <= 200000
            and all(index < summary['points'] for index in winners), 'complete point denominator')
    return dict(canvas_pixels=size[0]*size[1], reference_pixels=int(np.count_nonzero(reference)),
                predicted_pixels=len(pixels), paired_pixels=len(errors),
                squared_error_sum_m2=math.fsum(v*v for v in errors),
                absolute_error_sum_m=math.fsum(abs(v) for v in errors),
                squared_inverse_error_sum=math.fsum(v*v for v in inverse_errors),
                absolute_inverse_error_sum=math.fsum(abs(v) for v in inverse_errors))


def summarize(totals):
    result = dict(totals)
    count = totals['paired_pixels']
    refs, predictions = totals['reference_pixels'], totals['predicted_pixels']
    result.update(missing_prediction_reference_pixels=refs-count,
                  predicted_without_reference_pixels=predictions-count,
                  reference_coverage=count/refs if refs else None,
                  prediction_density=predictions/totals['canvas_pixels'] if totals['canvas_pixels'] else None,
                  rmse_m=math.sqrt(totals['squared_error_sum_m2']/count) if count else None,
                  mae_m=totals['absolute_error_sum_m']/count if count else None,
                  irmse_1_per_km=math.sqrt(totals['squared_inverse_error_sum']/count) if count else None,
                  imae_1_per_km=totals['absolute_inverse_error_sum']/count if count else None,
                  full_reference_coverage=bool(refs and count == refs))
    require(all(not isinstance(v, float) or math.isfinite(v) for v in result.values()), 'finite metrics required')
    return result


def execute(args):
    output = fresh_directory(args.output)  # Admission before reading any manifest/data.
    root = args.root.absolute()
    require(root.is_dir() and not root.is_symlink()
            and not any(p.is_symlink() for p in root.parents), 'real data root required')
    manifest_bytes = bounded(args.manifest, 512*1024)
    manifest, files = inventory(manifest_bytes, root)
    binaries = {key: path.absolute() for key, path in
                [('projection', args.projection_binary), ('depth', args.depth_binary)]}
    identity = dict(protocol=PROTOCOL, manifest_sha256=sha(manifest_bytes), files=files,
                    binaries={key: sha(bounded(path, 32*1024*1024)) for key, path in binaries.items()},
                    evaluator_sha256=sha(Path(__file__).read_bytes()))
    if args.prepare_freeze:
        write(output/'freeze.json', identity)
        print(json.dumps(dict(prepared=True, frames=len(manifest['frames']), reference_images_decoded=False)))
        return 0
    require(args.freeze is not None, '--freeze from metadata-only preparation is required')
    require(strict_json(bounded(args.freeze, 2*1024*1024)) == identity, 'freeze changed; do not silently reselect/retune')
    write(output/'admission.json', identity)
    frame_results, journal = [], []

    def invoke(command):
        begin = time.perf_counter()
        result = subprocess.run([str(v) for v in command], capture_output=True, text=True, timeout=120)
        elapsed = time.perf_counter()-begin
        journal.append(dict(command=[str(v) for v in command], exit_code=result.returncode,
                            elapsed_s=elapsed, stdout=result.stdout, stderr=result.stderr))
        require(result.returncode == 0, 'estimator exit '+str(result.returncode)+': '+result.stderr[:1000])

    # All operational projections/rasterizations finish before any reference decode.
    for frame in manifest['frames']:
        item = dict(frame_id=frame['frame_id'], estimator_completed=False)
        frame_results.append(item)
        directory = output/frame['frame_id']
        directory.mkdir()
        projection, depth_path = directory/'projection.json', directory/'depth.json'
        try:
            command = [binaries['projection']]
            for key, flag in [('camera', '--calib-cam'), ('velo', '--calib-velo'), ('imu', '--calib-imu')]:
                command.extend([flag, local(root, manifest['calibration'][key])])
            command += ['--points', local(root, frame['points']), '--camera', manifest['camera_index'], '--output', projection]
            invoke(command)
            report = strict_json(bounded(projection, MAX_JSON))
            for key, name in [('calib_cam_to_cam', manifest['calibration']['camera']),
                              ('calib_velo_to_cam', manifest['calibration']['velo']),
                              ('calib_imu_to_velo', manifest['calibration']['imu']), ('velodyne_points', frame['points'])]:
                require(report['source_hashes'][key] == files[name], 'estimator input changed after freeze')
            invoke([binaries['depth'], '--projection', projection, '--output', depth_path])
            depth_raw = bounded(depth_path, MAX_JSON)
            depth = strict_json(depth_raw)
            projection_bytes = bounded(projection, MAX_JSON)
            require(depth['input_projection'] == dict(bytes=len(projection_bytes), sha256=sha(projection_bytes)),
                    'depth is not bound to the generated projection')
            item.update(estimator_completed=True, depth_sha256=sha(depth_raw))
        except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
            item['failure'] = str(error)
    write(output/'estimation-journal.json', journal)
    write(output/'estimation-status.json', frame_results)
    totals = dict(canvas_pixels=0, reference_pixels=0, predicted_pixels=0, paired_pixels=0,
                  squared_error_sum_m2=0., absolute_error_sum_m=0.,
                  squared_inverse_error_sum=0., absolute_inverse_error_sum=0.)
    for frame, item in zip(manifest['frames'], frame_results):
        if not item['estimator_completed']:
            continue
        try:
            reference_raw = bounded(local(root, frame['reference_depth']), MAX_PNG)
            require(files[frame['reference_depth']] == dict(bytes=len(reference_raw), sha256=sha(reference_raw)),
                    'reference changed after freeze')
            depth_raw = bounded(output/frame['frame_id']/'depth.json', MAX_JSON)
            require(sha(depth_raw) == item['depth_sha256'], 'prediction changed before scoring')
            values = score(strict_json(depth_raw), reference_raw, manifest['camera_index'])
            item['metrics'] = summarize(values)
            for key, value in values.items():
                totals[key] += value
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as error:
            item['failure'] = str(error)
    scored = sum('metrics' in item for item in frame_results)
    comparable = sum(item.get('metrics', {}).get('paired_pixels', 0) > 0 for item in frame_results)
    report = dict(schema_version=1, kind='provided_file_sparse_depth_evaluation', protocol=PROTOCOL,
                  manifest_sha256=sha(manifest_bytes), recording_id=manifest['recording_id'],
                  camera_index=manifest['camera_index'], selected_frames=len(frame_results),
                  scored_frames=scored, comparable_frames=comparable,
                  complete_evaluation=comparable == len(frame_results),
                  metrics_over_scored_frames=summarize(totals), frames=frame_results,
                  quality_gate_passed=None, actual_data_provenance=manifest['provenance'],
                  provenance_authenticated=False, official_benchmark_equivalence=False,
                  reference_note='Supplied annotated references may share LiDAR measurements with inputs; not independent sensor calibration truth.')
    write(output/'results.json', report)
    print(json.dumps({key: report[key] for key in ('selected_frames', 'scored_frames', 'comparable_frames',
                                                  'complete_evaluation', 'metrics_over_scored_frames')}))
    return 0 if report['complete_evaluation'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--projection-binary', type=Path, required=True)
    parser.add_argument('--depth-binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare-freeze', action='store_true')
    parser.add_argument('--freeze', type=Path)
    args = parser.parse_args()
    require(not (args.prepare_freeze and args.freeze), 'choose preparation or evaluation')
    return execute(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, OverflowError, subprocess.SubprocessError) as error:
        print('KITTI depth evaluation: '+str(error), file=sys.stderr)
        sys.exit(2)
