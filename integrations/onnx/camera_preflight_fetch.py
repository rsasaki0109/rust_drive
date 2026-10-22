#!/usr/bin/env python3
"""Fetch only the preselected original BDD inputs; do not redistribute JPEGs."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import urllib.error
import urllib.request

HERE = Path(__file__).resolve().parent


def download(url, size):
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url, timeout=45) as response:
                return response.read(size + 1)
        except urllib.error.HTTPError as error:
            if error.code not in (429, 500, 502, 503, 504) or attempt == 2:
                raise
            print('transient HTTP', error.code, 'retrying the same pinned URL', flush=True)
            time.sleep(.5 * (attempt + 1))
    raise AssertionError('unreachable retry state')


def fetch(output):
    design = json.loads((HERE / 'profile-preflight.json').read_text())
    output.mkdir(parents=True, exist_ok=True)
    receipt = {}
    total = 0
    for image in design['images']:
        for kind in ('image', 'annotation'):
            pin = image[kind]
            path = pin['path']
            name = Path(path).name
            if (pin['type'] != 'blob' or Path(name).stem != image['id']
                    or not 0 < pin['size'] <= 4 * 1024 * 1024):
                raise ValueError('unsafe pinned input')
            total += pin['size']
            if total > design['input_byte_budget']:
                raise ValueError('aggregate input bound')
            target = output / name
            url = (f"https://raw.githubusercontent.com/{design['repository']}/"
                   f"{design['revision']}/{path}")
            if target.exists():
                if not target.is_file() or target.is_symlink() or target.stat().st_size != pin['size']:
                    raise ValueError('cached input type/size mismatch')
                body = target.read_bytes()
            else:
                body = download(url, pin['size'])
            blob = hashlib.sha1(b'blob ' + str(len(body)).encode() + b'\0' + body).hexdigest()
            if len(body) != pin['size'] or blob != pin['sha']:
                raise ValueError('original Git blob size/hash mismatch')
            if not target.exists():
                with target.open('xb') as stream:
                    stream.write(body)
            receipt[name] = dict(bytes=len(body), sha256=hashlib.sha256(body).hexdigest(),
                                 git_blob_sha1=blob, url=url)
            print(name, 'original Git blob verified', flush=True)
    body = json.dumps(receipt, indent=2, sort_keys=True) + '\n'
    path = output / 'inputs.json'
    if path.exists() and path.read_text() != body:
        raise ValueError('input receipt changed')
    path.write_text(body)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/camera-preflight-v1/data'))
    fetch(parser.parse_args().output)
