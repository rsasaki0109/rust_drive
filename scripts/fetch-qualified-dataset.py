#!/usr/bin/env python3
"""Fetch exact optional preregistered FR1 room RGB-D bytes; no image decoding, pose reading or fitting."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent.parent
TOTAL_BYTES = 32 * 1024 * 1024
FILE_BYTES = 4 * 1024 * 1024
SOURCES = {
    'tum-fr1-room-qualified': ('edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting', '1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1', 'data/rgbd_dataset_freiburg1_room/', 'TUM1.yaml', '5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf'),
}
CALIBRATION_REPO = 'luigifreda/pyslam'
CALIBRATION_REV = '96019cfafcfc099ac9866884d7143a9ed1451a0d'
PINNED_METADATA = {
    'depth.txt': (62636, '86757dd3c8606f940477a353915ede7143811817b2288647414fe1459ed22c14', '6a8c4f13bdeaa5ebca5fd6a78de7230eb02d1995'),
    'rgb.txt': (60006, 'fe2f438ebe1b3d13a061e4db3b154df1136a82d95a723e4a6457b0e516c2451b', 'a8b9fe8ce86576c0353b1182546163bea569188c'),
    'groundtruth.txt': (332920, '66b6770d887a5a3b69c94c3b186c3f76fe092ca25399772177f41aed984bd1b0', '0adacc2b8c371eefc7f82993eb8e30128283e888'),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def item_safe(item):
    name, source = item['file'], item['source_path']
    if Path(name).name != name or '/' in name or '\\' in name:
        raise ValueError('unsafe local filename')
    if source.startswith('/') or '\\' in source or any(x in ('', '.', '..') for x in source.split('/')):
        raise ValueError('unsafe source path')
    if not isinstance(item['bytes'], int) or not 0 < item['bytes'] <= FILE_BYTES:
        raise ValueError('unsafe transfer bound')
    if not re.fullmatch(r'[0-9a-f]{64}', item['sha256']):
        raise ValueError('unsafe SHA-256')


def verify_manifest(manifest):
    name = manifest['dataset']
    if name not in SOURCES or manifest.get('redistribute_raw') is not False:
        raise ValueError('unknown dataset or unauthorized redistribution')
    repo, revision, prefix, calibration_name, calibration_sha = SOURCES[name]
    if manifest['repository'] != repo or manifest['revision'] != revision:
        raise ValueError('changed source commit')
    expected_profile = {'width': 640, 'height': 480, 'fx': 517.306408, 'fy': 516.469215, 'cx': 318.643040, 'cy': 255.313989, 'units_per_metre': 5000.0, 'invalid_depth': 0}
    if any(manifest['depth_calibration'].get(k) != v for k, v in expected_profile.items()):
        raise ValueError('changed pinned FR1 source model')
    frames, files = manifest['frames'], manifest['files']
    if [f['source_index'] for f in frames] != list(range(100, 136)):
        raise ValueError('changed preregistered consecutive depth selection')
    expected = {'depth.txt', 'rgb.txt', 'groundtruth.txt'}
    last_depth = None
    last_rgb = None
    last_rgb_index = None
    recorded_rgb = {}
    for frame in frames:
        depth_time, rgb_time = frame['depth_timestamp'], frame['rgb_timestamp']
        gap, rgb_index = frame['pair_gap_seconds'], frame['rgb_source_index']
        if (not isinstance(depth_time, (float, int)) or not isinstance(rgb_time, (float, int))
                or not math.isfinite(depth_time) or not math.isfinite(rgb_time)
                or not isinstance(gap, (float, int)) or not math.isfinite(gap) or gap < 0
                or abs(depth_time - rgb_time) > 0.0200001
                or abs(gap - abs(depth_time-rgb_time)) > 1e-9
                or not isinstance(rgb_index, int) or isinstance(rgb_index, bool) or rgb_index < 0
                or (last_rgb_index is not None and rgb_index < last_rgb_index)
                or (last_depth is not None and depth_time <= last_depth)
                or (last_rgb is not None and rgb_time < last_rgb)
                or frame['file'] != frame['depth_file'] or frame['timestamp'] != depth_time):
            raise ValueError('unsafe or changed RGB/depth association')
        # Repeated recorded RGB observations are intentionally retained. The
        # operational visual localizer must reject them without refreshing time.
        identity = (rgb_time, frame['rgb_file'])
        if rgb_index in recorded_rgb and recorded_rgb[rgb_index] != identity:
            raise ValueError('changed repeated RGB acquisition identity')
        recorded_rgb[rgb_index] = identity
        last_depth, last_rgb, last_rgb_index = depth_time, rgb_time, rgb_index
        expected.update((frame['depth_file'], frame['rgb_file']))
    if len(files) != len(expected) or {f['file'] for f in files} != expected:
        raise ValueError('changed exact paired input inventory')
    for item in files:
        item_safe(item)
        role = item['role']
        local = item['file']
        if role == 'depth_frame':
            expected_source = prefix + 'depth/' + local.removeprefix('depth-')
            if not local.startswith('depth-'):
                raise ValueError('unsafe depth filename')
        elif role == 'rgb_frame':
            expected_source = prefix + 'rgb/' + local.removeprefix('rgb-')
            if not local.startswith('rgb-'):
                raise ValueError('unsafe RGB filename')
        elif (role, local) in (('depth_index', 'depth.txt'), ('rgb_index', 'rgb.txt'), ('evaluation_only_mocap_ground_truth', 'groundtruth.txt')):
            expected_source = prefix + local
        else:
            raise ValueError('unknown source role')
        if item['source_path'] != expected_source:
            raise ValueError('source outside fixed sequence')
        if not re.fullmatch(r'[0-9a-f]{40}', item.get('git_blob_sha1', '')):
            raise ValueError('missing or invalid source Git blob identity')
        if local in PINNED_METADATA and (item['bytes'], item['sha256'], item['git_blob_sha1']) != PINNED_METADATA[local]:
            raise ValueError('changed qualified full-source metadata')
    calibration = manifest['calibration_source']
    item_safe(calibration)
    if (calibration['repository'] != CALIBRATION_REPO or calibration['revision'] != CALIBRATION_REV
            or calibration['source_path'] != 'settings/' + calibration_name
            or calibration['file'] != 'camera-calibration.yaml' or calibration['sha256'] != calibration_sha):
        raise ValueError('changed pinned camera calibration')
    if sum(f['bytes'] for f in files) + calibration['bytes'] > TOTAL_BYTES:
        raise ValueError('full paired subset exceeds 32 MiB bound')
    return [(repo, revision, item) for item in files] + [(CALIBRATION_REPO, CALIBRATION_REV, calibration)]


def fetch(destination, verify_only):
    manifest = json.loads((destination / 'manifest.json').read_text())
    inputs = verify_manifest(manifest)
    raw = destination / 'raw'
    raw.mkdir(parents=True, exist_ok=True)
    for repo, revision, item in inputs:
        target = raw / item['file']
        def valid():
            if not target.is_file() or target.stat().st_size != item['bytes']:
                return False
            data = target.read_bytes()
            if hashlib.sha256(data).hexdigest() != item['sha256']:
                return False
            expected_git_blob = item.get('git_blob_sha1')
            return expected_git_blob is None or hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() == expected_git_blob
        if not valid():
            if verify_only:
                raise ValueError('missing or changed input ' + item['file'])
            partial = target.with_suffix('.download')
            try:
                subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                                '--connect-timeout', '20', '--max-time', '60',
                                '--max-filesize', str(item['bytes']),
                                f'https://raw.githubusercontent.com/{repo}/{revision}/{item["source_path"]}',
                                '--output', str(partial)], check=True)
                data = partial.read_bytes()
                if (len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']
                        or (item.get('git_blob_sha1') is not None and hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() != item['git_blob_sha1'])):
                    raise ValueError('pinned transfer SHA/Git blob mismatch')
                partial.replace(target)
            finally:
                partial.unlink(missing_ok=True)
        if not valid():
            raise ValueError('raw SHA mismatch')
    print(f'{manifest["dataset"]}: {len(inputs)} exact source files SHA verified; no pixel decoding or fitting')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', choices=sorted(SOURCES))
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    fetch(ROOT / 'data' / args.dataset, args.verify_only)


if __name__ == '__main__':
    main()
