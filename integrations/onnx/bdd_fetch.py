#!/usr/bin/env python3
"""Fetch bounded original BDD research inputs; no raw data redistribution."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

HERE = Path(__file__).resolve().parent
PROTOCOL = HERE / 'bdd_protocol.json'
CLASS_MAPPING = {'person': 0, 'rider': 0, 'bike': 1, 'car': 2, 'motor': 3,
                 'bus': 5, 'truck': 7, 'train': 6, 'traffic light': 9}


def digest(body):
    return hashlib.sha256(body).hexdigest()


def protocol():
    p = json.loads(PROTOCOL.read_text())
    ids = [i['id'] for i in p['images']]
    if len(ids) != 8 or ids != sorted(set(ids)) or p['class_mapping'] != CLASS_MAPPING:
        raise ValueError('fixed BDD selection/classes changed')
    if (p['confidence_threshold'], p['nms_iou_threshold'], p['match_iou_threshold']) != (.3, .45, .5):
        raise ValueError('fixed BDD thresholds changed')
    if sum(pin['bytes'] for _, pin in pins(p)) > 10 * 1024 * 1024:
        raise ValueError('research input aggregate exceeds 10MiB bound')
    return p


def pins(p):
    yield p['license']['file'], p['license']
    for image in p['images']:
        for kind in ['image', 'annotation']:
            yield image[kind]['file'], image[kind]


def checked(body, pin):
    if len(body) != pin['bytes'] or digest(body) != pin['sha256']:
        raise ValueError('pinned BDD input size/SHA mismatch')
    if 'git_blob_sha1' in pin:
        git = hashlib.sha1(b'blob '+str(len(body)).encode()+b'\0'+body).hexdigest()
        if git != pin['git_blob_sha1']:
            raise ValueError('original and licensed mirror Git blob mismatch')
    return body


def checked_path(path, pin):
    if path.stat().st_size != pin['bytes']:
        raise ValueError('pinned BDD file size changed')
    with path.open('rb') as stream:
        return checked(stream.read(pin['bytes']+1), pin)


def fetch(output):
    output.mkdir(parents=True, exist_ok=True)
    for name, pin in pins(protocol()):
        if Path(name).name != name or '\\' in name or not 0 < pin['bytes'] <= 4 * 1024 * 1024:
            raise ValueError('unsafe input path/size')
        target = output / name
        if target.exists():
            checked_path(target, pin)
        else:
            with urllib.request.urlopen(pin['url'], timeout=60) as response:
                body = checked(response.read(pin['bytes']+1), pin)
            temporary = target.with_suffix(target.suffix+'.part')
            temporary.write_bytes(body)
            temporary.replace(target)
        print(f'{name}: original bytes and SHA verified')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/bdd-camera'))
    fetch(parser.parse_args().output)
