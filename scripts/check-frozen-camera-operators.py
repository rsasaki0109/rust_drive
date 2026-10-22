#!/usr/bin/env python3
"""Audit the immutable optimization packet with its own recorded source bytes."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def main():
    packet = ROOT / 'assets/camera-operators-v2'
    with tempfile.TemporaryDirectory(prefix='camera-operators-v2-') as directory:
        root = Path(directory)
        for name, pin in json.loads((packet / 'source-sha256.json').read_text()).items():
            target = root / name
            if not target.resolve().is_relative_to(root):
                raise ValueError('unsafe archived source path')
            data = (packet / 'source' / name).read_bytes()
            if hashlib.sha256(data).hexdigest() != pin:
                raise ValueError('archived optimization source changed: ' + name)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        assets = root / 'assets'
        assets.mkdir(exist_ok=True)
        for path in (ROOT / 'assets').iterdir():
            if path.name != packet.name:
                (assets / path.name).symlink_to(path, target_is_directory=path.is_dir())
        shutil.copytree(packet, assets / packet.name, dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('__pycache__'))
        subprocess.run([sys.executable, '-B', str(assets / packet.name / 'verify.py')],
                       cwd=root, check=True)


if __name__ == '__main__':
    main()
