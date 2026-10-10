#!/usr/bin/env python3
"""Opt-in official YOLOX-S only; no images, inference, or baseline replacement."""
import argparse
import hashlib
import os
from pathlib import Path
import urllib.request

URL = 'https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.onnx'
SIZE = 35858002
SHA256 = 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'


def verify(path):
    if path.is_symlink() or not path.is_file() or path.stat().st_size != SIZE:
        raise ValueError('YOLOX-S must be a regular file with pinned release size')
    sha, count = hashlib.sha256(), 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            count += len(chunk)
            if count > SIZE:
                raise ValueError('YOLOX-S changed size during verification')
            sha.update(chunk)
    if count != SIZE or sha.hexdigest() != SHA256:
        raise ValueError('YOLOX-S pinned SHA-256/size mismatch')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/camera-model-s'))
    args = parser.parse_args()
    if args.output.is_symlink():
        raise ValueError('unsafe model output directory')
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output / 'yolox_s.onnx'
    if target.exists() or target.is_symlink():
        verify(target)
    else:
        partial = target.with_suffix('.onnx.part')
        sha, count = hashlib.sha256(), 0
        with urllib.request.urlopen(URL, timeout=60) as response, partial.open('xb') as stream:
            for chunk in iter(lambda: response.read(65536), b''):
                count += len(chunk)
                if count > SIZE:
                    raise ValueError('official model exceeds pinned byte cap')
                sha.update(chunk)
                stream.write(chunk)
        if count != SIZE or sha.hexdigest() != SHA256:
            raise ValueError('downloaded YOLOX-S SHA-256/size mismatch')
        # Publish without replacing a file that appeared during acquisition.
        os.link(partial, target)
        partial.unlink()
        verify(target)
    print('yolox_s.onnx: pinned size/SHA verified; no image inference performed')


if __name__ == '__main__':
    main()
