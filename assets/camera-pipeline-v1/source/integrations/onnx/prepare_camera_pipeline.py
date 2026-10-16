#!/usr/bin/env python3
"""Add an explicit, authored replay delivery lag to an existing image sequence.

No images, model or ground truth are read. Lag is not measured execution latency.
"""
import argparse
from decimal import Decimal, InvalidOperation
import hashlib
import json
import os
from pathlib import Path


def convert(source, destination, lag_text):
    if not source.is_file() or source.is_symlink() or source.stat().st_size > 512 * 1024:
        raise ValueError('bounded regular non-symlink sequence required')
    if destination.exists() or destination.is_symlink():
        raise ValueError('fresh output required')
    lag = Decimal(lag_text)
    if not lag.is_finite() or not Decimal(0) <= lag <= Decimal(2):
        raise ValueError('replay lag must be finite in [0,2] seconds')
    content = source.read_bytes()
    original = json.loads(content)
    if (original.keys() - {'schema_version', 'camera_id', 'width', 'height', 'config', 'events'}
            or original['schema_version'] != 1 or not 1 <= len(original['events']) <= 32):
        raise ValueError('expected original 1..32-event image sequence')
    result = {key: value for key, value in original.items() if key != 'events'}
    events = []
    for event in original['events']:
        if event.keys() != {'timestamp_s', 'input'} or not isinstance(event['timestamp_s'], str):
            raise ValueError('original decimal acquisition text required')
        text = event['timestamp_s']
        if not 1 <= len(text) <= 64:
            raise ValueError('bounded original clock text required')
        timestamp = Decimal(text)
        if not timestamp.is_finite() or not Decimal(0) <= timestamp <= Decimal('1e12') - lag:
            raise ValueError('clock outside supported range')
        data = event['input']
        if data['kind'] == 'image' and data.keys() == {'kind', 'path', 'sha256'}:
            # Lexical absolute paths preserve any final symlink for CLI checks.
            image = os.path.abspath(source.parent / data['path'])
            converted = dict(kind='image', acquired_s=text,
                path=os.path.relpath(image, destination.absolute().parent), sha256=data['sha256'])
        elif data['kind'] in ('missing', 'failed') and data.keys() == {'kind'}:
            converted = dict(kind='tick' if data['kind'] == 'missing' else 'failed')
        else:
            raise ValueError('unknown original image input')
        events.append(dict(now_s=format(timestamp + lag, 'f'), input=converted))
    result['events'] = events
    encoded = (json.dumps(result, indent=2, allow_nan=False) + '\n').encode()
    if len(encoded) > 512 * 1024:
        raise ValueError('converted sequence exceeds 512 KiB')
    with destination.open('xb') as stream:
        stream.write(encoded)
    return dict(source_sequence_sha256=hashlib.sha256(content).hexdigest(), replay_lag_s=str(lag),
                events=len(events), images=sum(e['input']['kind'] == 'image' for e in events), images_read=0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', type=Path, required=True)
    parser.add_argument('--replay-lag-s', default='0.005')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        print(json.dumps(convert(args.sequence, args.output, args.replay_lag_s)))
    except (OSError, ValueError, InvalidOperation, KeyError, TypeError) as error:
        parser.exit(2, f'{error}\n')
