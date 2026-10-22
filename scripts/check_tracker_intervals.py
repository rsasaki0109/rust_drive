#!/usr/bin/env python3
"""Audit per-track acquisition intervals in complete schema-1 sensor logs.

Reconstruct velocity corrections from consecutive track positions and timestamps.
This checks the filter's time base, not clustering or correspondence accuracy.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from tracking_frame import continuous_observations


def vector(point):
    result = point['x'], point['y']
    if not all(math.isfinite(x) for x in result):
        raise ValueError('nonfinite track vector')
    return result


def audit(path):
    digest = hashlib.sha256()
    previous, clock, header, footer = {}, None, False, False
    ticks = corrections = coasts = new_tracks = long_intervals = capped = failures = 0
    largest_error = largest_interval = 0.
    examples = []
    with path.open('rb') as source:
        for line in source:
            digest.update(line)
            record = json.loads(line)
            if footer:
                raise ValueError(f'{path}: records after footer')
            if record['kind'] == 'header':
                if header or ticks or record['header']['schema_version'] != 1:
                    raise ValueError(f'{path}: invalid header')
                header = True
                continue
            if record['kind'] == 'end':
                if not header or not ticks or record['ticks'] != ticks:
                    raise ValueError(f'{path}: wrong footer count')
                footer = True
                continue
            if not header or record['kind'] != 'tick':
                raise ValueError(f'{path}: expected header then ticks')
            tick = record['tick']
            output = tick['expected']
            now = tick['input']['time']
            if not math.isfinite(now) or now < 0. or (clock is not None and now <= clock) or output['time'] != now:
                raise ValueError(f'{path}: invalid frame clock')
            current = {}
            tracks, _ = continuous_observations(output)
            for track in tracks:
                identity, stamp = track['id'], track['last_seen']
                if identity in current or not math.isfinite(stamp) or not 0. <= stamp <= now:
                    raise ValueError(f'{path}: invalid track identity/time at tick {ticks}')
                current[identity] = track
                position, velocity = vector(track['position']), vector(track['velocity'])
                old = previous.get(identity)
                if old is None:
                    new_tracks += 1
                    expected = (0., 0.)
                else:
                    elapsed = stamp - old['last_seen']
                    if elapsed < 0. or elapsed >= 0.6:
                        raise ValueError(f'{path}: invalid retained-track interval at tick {ticks}')
                    if elapsed == 0.:
                        # World values may have been reprojected by a new pose
                        # correction. Compare in the unchanged continuous frame.
                        if (any(track[n] != old[n] for n in ['id', 'radius', 'last_seen'])
                                or math.dist(position, vector(old['position'])) > 1e-8
                                or math.dist(velocity, vector(old['velocity'])) > 1e-8):
                            raise ValueError(f'{path}: cached track changed at tick {ticks}')
                        coasts += 1
                        continue
                    corrections += 1
                    long_intervals += elapsed > 0.2 + 1e-9
                    largest_interval = max(largest_interval, elapsed)
                    # Alpha=0.75: the corrected position gives the innovation
                    # independently of the reported new velocity. Its time span
                    # is the two observations of this identity, not other scans.
                    old_position, old_velocity = vector(old['position']), vector(old['velocity'])
                    innovation = tuple((p - q - v * elapsed) / 0.75
                                       for p, q, v in zip(position, old_position, old_velocity))
                    expected = tuple(v + 0.12 * r / max(0.01, elapsed)
                                     for v, r in zip(old_velocity, innovation))
                    speed = math.hypot(*expected)
                    if speed > 15.:
                        expected = tuple(v * 15. / speed for v in expected)
                        capped += 1
                error = math.dist(velocity, expected)
                largest_error = max(largest_error, error)
                if error > 1e-8 * max(1., math.hypot(*expected)):
                    failures += 1
                    if len(examples) < 5:
                        examples.append({'tick': ticks, 'time': now, 'id': identity,
                                         'last_seen': stamp, 'reported_velocity': velocity,
                                         'independently_reconstructed_velocity': expected,
                                         'error_m_s': error})
            previous, clock = current, now
            ticks += 1
    if not footer:
        raise ValueError(f'{path}: missing footer')
    return {'log': str(path), 'sha256': digest.hexdigest(), 'ticks': ticks,
            'corrected_tracks': corrections, 'cached_tracks': coasts, 'new_tracks': new_tracks,
            'intervals_over_0_2_s': long_intervals, 'capped_corrections': capped,
            'maximum_interval_s': largest_interval, 'maximum_velocity_error_m_s': largest_error,
            'incorrect_corrections': failures, 'failure_examples': examples, 'passed': failures == 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    rows = [audit(path) for path in args.log]
    passed = all(row['passed'] for row in rows)
    report = {'passed': passed, 'audit_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'scope': 'Track-filter acquisition-time arithmetic; no physical accuracy or identity qualification.',
              'results': rows}
    with args.output.open('x') as out:
        json.dump(report, out, indent=2, allow_nan=False)
        out.write('\n')
    print(json.dumps({'passed': passed, 'logs': len(rows), 'ticks': sum(r['ticks'] for r in rows),
                      'incorrect_corrections': sum(r['incorrect_corrections'] for r in rows)}))
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
