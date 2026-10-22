#!/usr/bin/env python3
"""Independently score physical speed intervals, or run fixed reference fixtures."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess

TOLERANCE_M_S = 0.15
CASES = ['single', 'sequential', 'overlap', 'thin', 'stop-sign', 'north']


def progress(point, route):
    candidates = []
    total = 0.0
    for a, b in zip(route, route[1:]):
        dx, dy = b['x'] - a['x'], b['y'] - a['y']
        length = math.hypot(dx, dy)
        if length <= 0:
            raise ValueError('degenerate reference route')
        u = max(0.0, min(1.0, ((point['x']-a['x'])*dx + (point['y']-a['y'])*dy) / length**2))
        distance = (point['x']-a['x']-u*dx)**2 + (point['y']-a['y']-u*dy)**2
        candidates.append((distance, total + u*length))
        total += length
    return min(candidates)[1]


def evaluate(run, zones=None):
    override = zones is not None
    if zones is None:
        zones = run['scenario'].get('speed_zones', [])
    radius = run['vehicle']['radius']
    frames = run['frames']
    full_ticks = len(frames) == run['summary']['steps']
    if zones and not override and not full_ticks:
        raise ValueError('configured speed zones require every physical integration frame')
    previous, violations, maximum = None, 0, 0.0
    covered = [0] * len(zones)
    maximum_by_zone = [0.0] * len(zones)
    for frame in frames:
        end = (progress(frame['truth']['pose']['position'], run['route']['points']), frame['truth']['speed'])
        start = previous or end
        previous = end
        violated = False
        for i, zone in enumerate(zones):
            lower, upper = zone['start_s_m']-radius, zone['end_s_m']+radius
            # Clip time fractions of the swept reference point. A zone wholly
            # between frame endpoints must still participate in evaluation.
            delta = end[0]-start[0]
            if abs(delta) <= 1e-12:
                fractions = [0.0, 1.0] if lower <= start[0] <= upper else []
            else:
                roots = sorted([(lower-start[0])/delta, (upper-start[0])/delta])
                a, b = max(0.0, roots[0]), min(1.0, roots[1])
                fractions = [a, b] if a <= b else []
            if not fractions:
                continue
            covered[i] += 1
            speeds = [(1-f)*start[1] + f*end[1] for f in fractions]
            excess = max(speeds)-zone['speed_m_s']
            maximum = max(maximum, excess)
            maximum_by_zone[i] = max(maximum_by_zone[i], excess)
            violated |= excess > TOLERANCE_M_S + 1e-9
        violations += violated
    if any(n == 0 for n in covered):
        raise ValueError('a configured speed interval was never physically exercised')
    return {
        'passed': violations == 0, 'violation_intervals': violations,
        'maximum_excess_m_s': maximum, 'zone_covered_intervals': covered,
        'maximum_excess_by_zone_m_s': maximum_by_zone,
        'tolerance_m_s': TOLERANCE_M_S, 'physical_frames': len(frames),
        'full_tick_coverage': full_ticks, 'evaluation_only_policy_override': override,
    }


def reacceleration(run):
    zones = run['scenario']['speed_zones']
    lower = max(z['end_s_m'] for z in zones)+run['vehicle']['radius']+8.0
    points = run['route']['points']
    upper = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a, b in zip(points, points[1:]))-10.0
    speeds = (f['truth']['speed'] for f in run['frames']
              if lower <= progress(f['truth']['pose']['position'], points) <= upper)
    maximum = max(speeds, default=0.0)
    return {
        'passed': maximum >= 4.0, 'maximum_speed_m_s': maximum,
        'minimum_required_max_speed_m_s': 4.0,
        'interval_start_s_m': lower, 'interval_end_s_m': upper,
    }


def execute(binary, arguments, prefix):
    with prefix.with_suffix('.stdout').open('w') as out, prefix.with_suffix('.stderr').open('w') as err:
        return subprocess.run([str(binary), *map(str, arguments)], stdout=out, stderr=err).returncode


def acceptance(binary, output, seeds):
    repo = Path(__file__).resolve().parents[1]
    rows = []
    output.mkdir(parents=True, exist_ok=True)
    for case in CASES:
        for seed in seeds:
            folder = output/case/f'seed-{seed}'
            folder.mkdir(parents=True, exist_ok=False)
            code = execute(binary, ['run', '--scenario', repo/'scenarios'/f'speed-zone-{case}.json',
                                   '--seed', seed, '--output', folder/'run'], folder/'run-process')
            replay_code = execute(binary, ['replay', '--log', folder/'run/sensors.jsonl',
                                          '--output', folder/'replay'], folder/'replay-process')
            run = json.loads((folder/'run/run.json').read_text())
            replay = json.loads((folder/'replay/replay.json').read_text())
            score, release = evaluate(run), reacceleration(run)
            row = {
                'case': case, 'seed': seed, 'run_exit_code': code, 'replay_exit_code': replay_code,
                'summary': run['summary'], 'speed_compliance': score,
                'post_zone_reacceleration': release, 'replay': replay,
            }
            row['passed'] = (
                code == replay_code == 0 and run['summary']['passed'] and score['passed']
                and release['passed'] and replay['verified'] and replay['ticks'] == run['summary']['steps']
            )
            rows.append(row)
            report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
            (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--run', type=Path)
    group.add_argument('--output', type=Path)
    name = 'rustdriving.exe' if os.name == 'nt' else 'rustdriving'
    parser.add_argument('--binary', type=Path, default=repo/'target/release'/name)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    args = parser.parse_args()
    if args.run:
        result = evaluate(json.loads(args.run.read_text()))
    else:
        result = acceptance(args.binary.resolve(), args.output.resolve(), args.seeds)
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
