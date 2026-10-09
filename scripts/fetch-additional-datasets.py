#!/usr/bin/env python3
"""Acquire small SHA-pinned measured terrain/motion fixtures; raw inputs stay ignored."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
DATASETS = ('pdal-autzen', 'tum-fr1-xyz', 'tum-fr1-xyz-fast', 'tum-fr1-xyz-tight')
MAX_TOTAL_BYTES = 30_000_000
MAX_FILE_BYTES = 4_000_000


def read_manifest(name):
    if name not in DATASETS:
        raise ValueError('unknown additional dataset')
    path = ROOT/'data'/name/'manifest.json'
    if path.stat().st_size > 128_000:
        raise ValueError('manifest exceeds 128 KB')
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if (not isinstance(manifest, dict) or manifest.get('dataset') != name
            or not isinstance(manifest.get('repository'), str)
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*/[A-Za-z0-9][A-Za-z0-9_.-]*', manifest['repository'])
            or not isinstance(manifest.get('revision'), str)
            or not re.fullmatch(r'[0-9a-f]{40}', manifest['revision'])
            or manifest.get('redistribute_raw') is not False):
        raise ValueError('invalid identity, repository, pinned revision or redistribution policy')
    files = manifest.get('files')
    if not isinstance(files, list) or not 1 <= len(files) <= 128:
        raise ValueError('manifest requires one to 128 files')
    names = set()
    for entry in files:
        if not isinstance(entry, dict):
            raise ValueError('invalid file entry')
        filename, source = entry.get('file'), entry.get('source_path')
        if (not isinstance(filename, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,127}', filename)
                or filename.casefold() in names or not isinstance(source, str)
                or not re.fullmatch(r'[A-Za-z0-9_./-]{1,512}', source)
                or any(part in ('', '.', '..') for part in source.split('/'))
                or PurePosixPath(source).is_absolute() or PurePosixPath(source).name != filename
                or type(entry.get('bytes')) is not int or not 1 <= entry['bytes'] <= MAX_FILE_BYTES
                or not isinstance(entry.get('sha256'), str)
                or not re.fullmatch(r'[0-9a-f]{64}', entry['sha256'])):
            raise ValueError('invalid file path, bounded size, SHA or duplicate name')
        names.add(filename.casefold())
    if sum(entry['bytes'] for entry in files) > MAX_TOTAL_BYTES:
        raise ValueError('dataset exceeds 30 MB')
    return manifest


def valid(path, entry):
    return (path.is_file() and path.stat().st_size == entry['bytes']
            and hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=('all',)+DATASETS, default='all')
    parser.add_argument('--output', type=Path, default=ROOT/'data')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    selected = DATASETS if args.dataset == 'all' else (args.dataset,)
    manifests = [read_manifest(name) for name in selected]
    total = sum(entry['bytes'] for manifest in manifests for entry in manifest['files'])
    if total > MAX_TOTAL_BYTES:
        raise ValueError('additional datasets exceed 30 MB')
    for manifest in manifests:
        destination = args.output/manifest['dataset']/'raw'
        destination.mkdir(parents=True, exist_ok=True)
        for entry in manifest['files']:
            target = destination/entry['file']
            if valid(target, entry):
                continue
            if args.verify_only:
                raise ValueError(f'{target}: missing or differs from pinned SHA/size')
            url = f'https://raw.githubusercontent.com/{manifest["repository"]}/{manifest["revision"]}/{entry["source_path"]}'
            temporary = target.with_name(target.name+'.download')
            try:
                subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                    '--connect-timeout', '20', '--max-time', '60', '--max-filesize', str(entry['bytes']),
                    url, '--output', str(temporary)], check=True)
                if not valid(temporary, entry):
                    raise ValueError(f'{target}: downloaded bytes differ from pinned SHA/size')
                temporary.replace(target)
            finally:
                temporary.unlink(missing_ok=True)
        print(f'{manifest["dataset"]}: verified {len(manifest["files"])} files; see data/{manifest["dataset"]}/SOURCE.md')
    print(f'{total:,} additional raw bytes; acquisition does not establish algorithm accuracy')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'additional dataset acquisition: {error}', file=sys.stderr)
        sys.exit(2)
