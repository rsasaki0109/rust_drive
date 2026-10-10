#!/usr/bin/env python3
"""Acquire only manifest-pinned optional submap input bytes; never decode depth."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent.parent
TOTAL_BYTES = 12 * 1024 * 1024
FILE_BYTES = 1024 * 1024


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_manifest(manifest):
    if manifest.get('redistribute_raw') is not False:
        raise ValueError('raw redistribution is not authorized')
    repo = manifest.get('repository', '')
    revision = manifest.get('revision', '')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', repo):
        raise ValueError('unsafe source repository')
    if not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('source must be an exact Git commit')
    frames, files = manifest['frames'], manifest['files']
    indices = [f['source_index'] for f in frames]
    if len(frames) != 36 or indices != list(range(indices[0], indices[0] + 36)):
        raise ValueError('expected preregistered 36 consecutive source frames')
    if len(files) != 38 or len({f['file'] for f in files}) != 38:
        raise ValueError('expected exact metadata/depth inventory')
    if sum(f['bytes'] for f in files) > TOTAL_BYTES:
        raise ValueError('subset exceeds transfer bound')
    expected = {'depth.txt', 'groundtruth.txt'} | {f['file'] for f in frames}
    if {f['file'] for f in files} != expected:
        raise ValueError('unexpected source inventory')
    for item in files:
        name, source = item['file'], item['source_path']
        if Path(name).name != name or '/' in name or '\\' in name:
            raise ValueError('unsafe local filename')
        if source.startswith('/') or '\\' in source or any(x in ('', '.', '..') for x in source.split('/')):
            raise ValueError('unsafe source path')
        if not 0 < item['bytes'] <= FILE_BYTES or not re.fullmatch(r'[0-9a-f]{64}', item['sha256']):
            raise ValueError('unsafe transfer bound or digest')
    calibration = manifest.get('calibration_source')
    if calibration is not None:
        if (not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', calibration['repository'])
                or not re.fullmatch(r'[0-9a-f]{40}', calibration['revision'])
                or calibration['file'] != 'camera-calibration.yaml'
                or not 0 < calibration['bytes'] <= FILE_BYTES
                or not re.fullmatch(r'[0-9a-f]{64}', calibration['sha256'])
                or calibration['source_path'].startswith('/')
                or any(x in ('', '.', '..') for x in calibration['source_path'].split('/'))
                or '\\' in calibration['source_path']):
            raise ValueError('unsafe calibration provenance')
        if sum(f['bytes'] for f in files) + calibration['bytes'] > TOTAL_BYTES:
            raise ValueError('full subset exceeds transfer bound')
    return repo, revision, files


def fetch(destination, verify_only):
    manifest = json.loads((destination / 'manifest.json').read_text())
    repo, revision, files = verify_manifest(manifest)
    raw = destination / 'raw'
    raw.mkdir(parents=True, exist_ok=True)
    calibration = manifest.get('calibration_source')
    inputs = [(repo, revision, item) for item in files]
    if calibration is not None:
        inputs.append((calibration['repository'], calibration['revision'], calibration))
    for source_repo, source_revision, item in inputs:
        target = raw / item['file']
        valid = lambda: target.is_file() and target.stat().st_size == item['bytes'] and sha256(target) == item['sha256']
        if not valid():
            if verify_only:
                raise ValueError('missing or changed raw input ' + item['file'])
            partial = target.with_suffix('.download')
            url = f'https://raw.githubusercontent.com/{source_repo}/{source_revision}/{item["source_path"]}'
            try:
                subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                                '--connect-timeout', '20', '--max-time', '60',
                                '--max-filesize', str(item['bytes']), url,
                                '--output', str(partial)], check=True)
                if partial.stat().st_size != item['bytes'] or sha256(partial) != item['sha256']:
                    raise ValueError('pinned transfer SHA mismatch')
                partial.replace(target)
            finally:
                partial.unlink(missing_ok=True)
        if not valid():
            raise ValueError('raw SHA mismatch')
    print(f'{manifest["dataset"]}: {len(inputs)} source files SHA verified; no depth decoding or fitting')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', help='committed data/*submap*/ dataset name')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z0-9-]+submaps?', args.dataset):
        raise ValueError('expected an optional submap dataset name')
    fetch(ROOT / 'data' / args.dataset, args.verify_only)


if __name__ == '__main__':
    main()
