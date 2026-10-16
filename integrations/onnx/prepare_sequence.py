#!/usr/bin/env python3
"""Prepare an image-only sequence from an existing pinned TUM RGB inventory."""
import argparse
import hashlib
import json
import os
from pathlib import Path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def prepare(args):
    if args.output.exists() or args.output.is_symlink():
        raise ValueError('fresh sequence filename required')
    if not 1 <= args.count <= 32:
        raise ValueError('select 1..32 consecutive original RGB indices')
    dataset = args.dataset.absolute()
    manifest_path = dataset / 'manifest.json'
    if manifest_path.stat().st_size > 1024 * 1024:
        raise ValueError('bounded source manifest')
    manifest = json.loads(manifest_path.read_text())
    files = {item['file']: item for item in manifest['files']}
    index = dataset / 'raw/rgb.txt'
    if index.stat().st_size > 4 * 1024 * 1024 or digest(index) != files['rgb.txt']['sha256']:
        raise ValueError('original RGB timestamp table SHA mismatch')
    rows = [line.split() for line in index.read_text().splitlines() if line and not line.startswith('#')]
    first = args.first if args.first is not None else manifest['frames'][0]['rgb_source_index']
    if first < 0 or first + args.count > len(rows):
        raise ValueError('RGB index range outside original table')
    events = []
    for row in rows[first:first + args.count]:
        if len(row) != 2 or Path(row[1]).parts[:1] != ('rgb',) or len(Path(row[1]).parts) != 2:
            raise ValueError('original RGB table row format')
        timestamp, original = row
        name = 'rgb-' + Path(original).name
        path = dataset / 'raw' / name
        item = files[name]
        if path.is_symlink() or not path.is_file() or path.stat().st_size != item['bytes'] or digest(path) != item['sha256']:
            raise ValueError('missing or changed pinned RGB file: ' + name)
        events.append(dict(timestamp_s=timestamp, input=dict(kind='image',
            path=os.path.relpath(path, args.output.absolute().parent), sha256=item['sha256'])))
    calibration = manifest['depth_calibration']
    body = dict(schema_version=1, camera_id=args.camera_id,
                width=calibration['width'], height=calibration['height'], events=events)
    # No image decoding, depth, camera poses or object identity references read.
    with args.output.open('x') as stream:
        json.dump(body, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(dict(sequence=str(args.output), first_rgb_index=first, images=args.count,
               source_manifest_sha256=digest(manifest_path), pixel_decode=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--first', type=int)
    parser.add_argument('--count', type=int, default=8)
    parser.add_argument('--camera-id', default='recorded-rgb')
    parser.add_argument('--output', type=Path, required=True)
    prepare(parser.parse_args())
