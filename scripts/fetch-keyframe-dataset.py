#!/usr/bin/env python3
"""Hash-bounded optional TUM temporal subset; never decode depth or fit geometry."""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent.parent
NAME = 'tum-fr1-xyz-keyframes'
REPO = 'MarcelBruckner-TUMProjects/3D-Scanning-Motion-Capture'
REV = 'f367047ee71f5304c6d7deaec55c4874bb8b035e'
PREFIX = 'Exercise_1/data/rgbd_dataset_freiburg1_xyz/'
TOTAL_BYTES = 8 * 1024 * 1024
FILE_BYTES = 512 * 1024


def digest(data):
    return hashlib.sha256(data).hexdigest()


def transfer(source, target, bound):
    if not source.startswith(PREFIX) or '..' in source.split('/'):
        raise ValueError('source outside fixed dataset')
    subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                    '--connect-timeout', '20', '--max-time', '60',
                    '--max-filesize', str(bound),
                    f'https://raw.githubusercontent.com/{REPO}/{REV}/{source}',
                    '--output', str(target)], check=True)
    if not 0 < target.stat().st_size <= bound:
        raise ValueError('transfer exceeds bound')


def prepare(destination):
    path = destination / 'manifest.json'
    if path.exists():
        raise ValueError('refuse to overwrite preregistered manifest')
    baseline = json.loads((ROOT / 'data/tum-fr1-xyz-tight/manifest.json').read_text())
    source_raw = ROOT / 'data/tum-fr1-xyz-tight/raw'
    rows = [x.split() for x in (source_raw / 'depth.txt').read_text().splitlines()
            if x and not x.startswith('#')]
    # Selection uses timestamp availability alone, never reference pose values.
    times = [float(x.split()[0]) for x in (source_raw / 'groundtruth.txt').read_text().splitlines()
             if x and not x.startswith('#')]
    gaps = []
    frames = []
    for index in range(340, 376):
        stamp = float(rows[index][0])
        k = bisect.bisect_left(times, stamp)
        gap = 0 if k < len(times) and times[k] == stamp else (
            times[k] - times[k-1] if 0 < k < len(times) else float('inf'))
        if not 0 <= gap <= .02:
            raise ValueError('fixed selection lacks bounded mocap timestamp bracket')
        gaps.append(gap)
        frames.append(dict(file=Path(rows[index][1]).name, timestamp=stamp,
                           source_index=index,
                           split='initialization' if index == 340 else 'held_out'))
    manifest = dict(baseline, dataset=NAME, frames=frames)
    manifest['selection_policy'] = (
        'Fixed original indices 340..375 inclusive, declared before acquisition, '
        'depth decoding and fitting. Selection uses mocap timestamp-bracket availability '
        'only (maximum 0.02 s), never reference transforms or fit errors. All 35 updates '
        'are held out; frame 340 initializes the measured map. Original indices 0..271 '
        'in prior subsets are viewed regression only. Same indoor room/camera/sequence, '
        'not cross-environment or automotive validation. No numerical settings tuned '
        'using this subset; freeze precedes first depth decoding.')
    manifest['timestamp_selection_audit'] = dict(maximum_bracket_seconds=max(gaps),
                                                frames=36, invalid_frames=0)
    manifest['files'] = baseline['files'][:2]
    raw = destination / 'raw'
    raw.mkdir(parents=True, exist_ok=True)
    for item in manifest['files']:
        content = (source_raw / item['file']).read_bytes()
        if len(content) != item['bytes'] or digest(content) != item['sha256']:
            raise ValueError('existing metadata SHA mismatch')
        shutil.copyfile(source_raw / item['file'], raw / item['file'])
    for frame in frames:
        name = frame['file']
        source = PREFIX + 'depth/' + name
        target = raw / name
        transfer(source, target, FILE_BYTES)
        content = target.read_bytes()
        manifest['files'].append(dict(file=name, source_path=source, bytes=len(content),
                                     sha256=digest(content), role='depth_frame',
                                     split=frame['split'], timestamp=frame['timestamp'],
                                     encoding='PNG uint16 grayscale 640x480'))
        if sum(item['bytes'] for item in manifest['files']) > TOTAL_BYTES:
            raise ValueError('subset exceeds total transfer bound')
    path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-manifest', action='store_true')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    destination = ROOT / 'data' / NAME
    destination.mkdir(parents=True, exist_ok=True)
    if args.prepare_manifest:
        prepare(destination)
    manifest = json.loads((destination / 'manifest.json').read_text())
    if (manifest['dataset'] != NAME or manifest['revision'] != REV
            or manifest['repository'] != REPO
            or [f['source_index'] for f in manifest['frames']] != list(range(340, 376))
            or len(manifest['files']) != 38
            or sum(f['bytes'] for f in manifest['files']) > TOTAL_BYTES):
        raise ValueError('changed fixed source selection or bounds')
    raw = destination / 'raw'
    raw.mkdir(parents=True, exist_ok=True)
    expected_names = {'depth.txt', 'groundtruth.txt'} | {f['file'] for f in manifest['frames']}
    if {f['file'] for f in manifest['files']} != expected_names:
        raise ValueError('changed fixed input inventory')
    for item in manifest['files']:
        name = item['file']
        if (Path(name).name != name or '/' in name or '\\' in name
                or not 0 < item['bytes'] <= FILE_BYTES
                or len(item['sha256']) != 64
                or any(x not in '0123456789abcdef' for x in item['sha256'])
                or item['source_path'] != PREFIX + ('' if name in ('depth.txt', 'groundtruth.txt') else 'depth/') + name):
            raise ValueError('unsafe manifest input')
        target = raw / name
        valid = lambda: (target.is_file() and target.stat().st_size == item['bytes']
                         and digest(target.read_bytes()) == item['sha256'])
        if not valid():
            if args.verify_only:
                raise ValueError('missing or changed raw input ' + name)
            partial = target.with_suffix('.download')
            transfer(item['source_path'], partial, item['bytes'])
            if partial.stat().st_size != item['bytes'] or digest(partial.read_bytes()) != item['sha256']:
                raise ValueError('pinned transfer SHA mismatch')
            partial.replace(target)
        if not valid():
            raise ValueError('raw SHA mismatch')
    print(f'{NAME}: 38 source files SHA verified; no PNG decoding or geometry fitting')


if __name__ == '__main__':
    main()
