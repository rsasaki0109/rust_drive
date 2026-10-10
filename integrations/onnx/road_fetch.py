#!/usr/bin/env python3
"""Opt-in, bounded, SHA-pinned research inputs; never redistributes JPEGs."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

HERE = Path(__file__).resolve().parent
PROTOCOL = HERE / 'road-protocol.json'


def protocol():
    return json.loads(PROTOCOL.read_text())


def checked(body, pin):
    if len(body) != pin['size_bytes'] or hashlib.sha256(body).hexdigest() != pin['sha256']:
        raise ValueError('pinned research input size/SHA-256 mismatch')
    return body


def fetch(output):
    output.mkdir(parents=True, exist_ok=True)
    p = protocol()
    pins = [('instances_train2017.json', p['annotation'])]
    pins += [(i['file_name'], i) for i in p['images']]
    for name, pin in pins:
        target = output / name
        if target.exists():
            checked(target.read_bytes(), pin)
        else:
            with urllib.request.urlopen(pin['url'], timeout=60) as response:
                body = checked(response.read(pin['size_bytes'] + 1), pin)
            temp = target.with_suffix(target.suffix + '.part')
            temp.write_bytes(body)
            temp.replace(target)
        print(f'{name}: size/SHA verified')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/road-camera'))
    fetch(parser.parse_args().output)
