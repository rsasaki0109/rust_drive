#!/usr/bin/env python3
"""Fetch or verify small revision/SHA-pinned research data; raw files stay ignored."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
DATASETS = ('isprs-terrain', 'libpointmatcher')
MAX_BYTES = 15_000_000
MAX_FILE_BYTES = 2_000_000


def read_manifest(name):
    path = ROOT/'data'/name/'manifest.json'
    if path.stat().st_size > 128_000:
        raise ValueError('dataset manifest exceeds the 128 KB bound')
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if (not isinstance(manifest, dict) or manifest.get('dataset') != name
            or not isinstance(manifest.get('repository'), str)
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*/[A-Za-z0-9][A-Za-z0-9_.-]*', manifest['repository'])
            or not isinstance(manifest.get('revision'), str)
            or not re.fullmatch(r'[0-9a-f]{40}', manifest['revision'])):
        raise ValueError('invalid dataset identity, GitHub repository or pinned revision')
    files = manifest.get('files')
    if not isinstance(files, list) or not 1 <= len(files) <= 128:
        raise ValueError('dataset manifest requires one to 128 files')
    names = set()
    for entry in files:
        if not isinstance(entry, dict):
            raise ValueError('invalid dataset file entry')
        filename, source = entry.get('file'), entry.get('source_path')
        if (not isinstance(filename, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]{0,127}', filename)
                or filename.casefold() in names or not isinstance(source, str)
                or not re.fullmatch(r'[A-Za-z0-9_./-]{1,512}', source)
                or any(part in ('', '.', '..') for part in source.split('/'))
                or PurePosixPath(source).is_absolute() or PurePosixPath(source).name != filename
                or type(entry.get('bytes')) is not int or not 1 <= entry['bytes'] <= MAX_FILE_BYTES
                or not isinstance(entry.get('sha256'), str)
                or not re.fullmatch(r'[0-9a-f]{64}', entry['sha256'])):
            raise ValueError('invalid dataset file path, size, hash or duplicate name')
        names.add(filename.casefold())
    return manifest


def valid(path, entry):
    return (path.is_file() and path.stat().st_size == entry['bytes']
            and hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=('all',)+DATASETS, default='all')
    parser.add_argument('--output', type=Path, default=ROOT/'data',
                        help='raw files go in OUTPUT/DATASET/raw; default: repository data/')
    parser.add_argument('--verify-only', action='store_true', help='require every selected file without downloading')
    args = parser.parse_args()
    selected = DATASETS if args.dataset == 'all' else (args.dataset,)
    manifests = [read_manifest(name) for name in selected]
    total = sum(entry['bytes'] for manifest in manifests for entry in manifest['files'])
    if total > MAX_BYTES:
        raise ValueError('manifest exceeds the 15 MB raw-data budget')
    count = 0
    for manifest in manifests:
        name = manifest['dataset']
        destination = args.output/name/'raw'
        destination.mkdir(parents=True, exist_ok=True)
        for entry in manifest['files']:
            target = destination/entry['file']
            if not valid(target, entry):
                if args.verify_only:
                    raise ValueError(f'{target}: missing or differs from the pinned SHA/size')
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
            count += 1
        print(f'{name}: verified {len(manifest["files"])} files; license/provenance: data/{name}/SOURCE.md')
    print(f'{count} pinned files, {total:,} bytes; acquisition is not algorithm acceptance')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'dataset acquisition: {error}', file=sys.stderr)
        sys.exit(2)
