#!/usr/bin/env python3
"""Fetch only SHA-pinned public upstream assets; no default build downloads."""
import argparse
import hashlib
from pathlib import Path
import urllib.request

FILES = {
    'yolox_nano.onnx': ('https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.onnx', 3659407, 'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d'),
    'astronaut.jpg': ('https://raw.githubusercontent.com/pytorch/vision/9a8d5453bdbcd882f7c7f401064b7514581636c3/gallery/assets/astronaut.jpg', 40344, '874dba0332a5a9a6a9268e732745f57c7ba21bc733867463daaae4a766f0a03f'),
    'instances.json': ('https://raw.githubusercontent.com/pytorch/vision/9a8d5453bdbcd882f7c7f401064b7514581636c3/gallery/assets/coco/instances.json', 1528, '11e721e049f44f43cba66f240c249869116380a7fb17a5880dd3102512efdba9'),
}

def checked(name, body):
    _, size, sha = FILES[name]
    if len(body) != size or hashlib.sha256(body).hexdigest() != sha:
        raise ValueError(f'{name}: pinned size/SHA-256 mismatch')
    return body

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/camera-model'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, (url, size, _) in FILES.items():
        target = args.output / name
        if target.exists():
            checked(name, target.read_bytes())
        else:
            with urllib.request.urlopen(url, timeout=60) as response:
                body = checked(name, response.read(size + 1))
            temp = target.with_suffix(target.suffix + '.part')
            temp.write_bytes(body)
            temp.replace(target)
        print(f'{name}: verified')

if __name__ == '__main__':
    main()
