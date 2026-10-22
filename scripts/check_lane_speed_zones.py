#!/usr/bin/env python3
"""Independently reconstruct selected lane speeds and score reference driving."""
import argparse
import json
import math
import os
from pathlib import Path

from check_speed_zones import evaluate, execute, reacceleration

REPO = Path(__file__).resolve().parents[1]
CASES = [
    {'case': name, 'map': 'maps/lanes/speed-zones'+('' if name == 'single' else '-'+name)+'.json',
     'start': 'approach', 'goal': 'exit', 'closed_lanes': ['school'] if name == 'closed' else [],
     'expected_lane_ids': ['approach', 'detour' if name == 'closed' else 'school', 'exit']}
    for name in ['single', 'thin', 'north', 'ceiling', 'closed']
]
CASES.insert(1, {'case': 'sequential', 'map': 'maps/lanes/speed-zones-sequential.json',
                 'start': 'lane-0', 'goal': 'lane-4', 'closed_lanes': [],
                 'expected_lane_ids': [f'lane-{i}' for i in range(5)]})


def reference_sections(spec, lane_ids, closed=()):
    """Use original map geometry/limits, independent of Rust's compiled profile."""
    lanes = {lane['id']: lane for lane in spec['lanes']}
    points, zones, total, previous = [], [], 0.0, None
    for name in lane_ids:
        if name in closed:
            raise ValueError('selected itinerary traverses a closed lane')
        lane = lanes[name]
        line = lane['centerline']
        if previous is not None:
            if name not in previous['successors'] or previous['centerline'][-1] != line[0]:
                raise ValueError('itinerary invents a longitudinal connection')
        start = total
        for a, b in zip(line, line[1:]):
            total += math.hypot(b['x']-a['x'], b['y']-a['y'])
        if total <= start:
            raise ValueError('nonpositive lane length')
        zones.append({'start_s_m': start, 'end_s_m': total, 'speed_m_s': min(8.0, lane['speed_limit_m_s'])})
        points.extend(line[1:] if points else line)
        previous = lane
    return points, zones


def check_policy(actual, expected):
    """Compare effective caps on the union of boundaries; allow equal-cap merging."""
    if not actual or abs(actual[0]['start_s_m']) > 1e-9:
        raise ValueError('compiled speed policy does not start at the route origin')
    if abs(actual[-1]['end_s_m']-expected[-1]['end_s_m']) > 1e-9:
        raise ValueError('compiled speed policy does not cover the route endpoint')
    for i, zone in enumerate(actual):
        values = [zone[k] for k in ['start_s_m', 'end_s_m', 'speed_m_s']]
        if not all(math.isfinite(v) for v in values) or values[1] <= values[0] or not 0.1 <= values[2] <= 8.0:
            raise ValueError('invalid compiled interval')
        if i and abs(actual[i-1]['end_s_m']-zone['start_s_m']) > 1e-9:
            raise ValueError('compiled intervals leave a gap or overlap')
    boundaries = sorted({z[k] for z in actual+expected for k in ['start_s_m', 'end_s_m']})
    for a, b in zip(boundaries, boundaries[1:]):
        if b-a < 1e-8:
            continue  # Floating-point representations of the same map boundary.
        midpoint = (a+b)/2
        cap = lambda zones: min((z['speed_m_s'] for z in zones if z['start_s_m'] <= midpoint <= z['end_s_m']), default=math.inf)
        if cap(actual) != cap(expected):
            raise ValueError('compiled policy changes a mapped speed interval')


def audit(folder, case, local=True):
    report = json.loads((folder/'export-process.stdout').read_text())
    scenario = json.loads((folder/'scenario.json').read_text())
    run = json.loads((folder/'run/run.json').read_text())
    replay = json.loads((folder/'replay/replay.json').read_text())
    spec = json.loads((REPO/case['map']).read_text())
    if report['plan']['lane_ids'] != case['expected_lane_ids']:
        raise ValueError('selected lane IDs differ from the frozen itinerary')
    points, expected = reference_sections(spec, case['expected_lane_ids'], case['closed_lanes'])
    if run['route']['points'] != points or report['plan']['route'] != run['route']:
        raise ValueError('resolved route differs from original lane geometry')
    with (folder/'run/sensors.jsonl').open() as stream:
        config = json.loads(next(stream))['header']['config']
    if {'objects', 'goal_hold_seconds', 'following', 'min_clearance_m'} & config.keys():
        raise ValueError('evaluation/world fields entered the sensor replay configuration')
    if config.get('speed_zones', []) != scenario.get('speed_zones', []):
        raise ValueError('mapped policy missing from sensor replay configuration')
    if local:
        check_policy(report['speed_zones'], expected)
        if scenario['speed_zones'] != report['speed_zones'] or scenario['cruise_speed'] != 6.0:
            raise ValueError('scenario does not preserve the requested local policy and cruise')
        configured = evaluate(run)  # Includes the full-physical-frame gate.
    else:
        configured = None
        if scenario.get('speed_zones') or 'speed_zones' in report:
            raise ValueError('legacy export unexpectedly enabled speed intervals')
    physical = evaluate(run, expected)
    if configured and configured['violation_intervals'] != physical['violation_intervals']:
        raise ValueError('configured and independently mapped physical scores disagree')
    restrictive = [z for z in expected if z['speed_m_s'] < 6.0]
    probe = dict(run, scenario=dict(run['scenario'], speed_zones=restrictive))
    release = reacceleration(probe)
    return {
        'summary': run['summary'], 'replay': replay,
        'mapped_speed_compliance': physical, 'configured_speed_compliance': configured,
        'post_zone_reacceleration': release, 'map_reference_sections': expected,
        'map_reference_independently_reconstructed': True,
        'passed': (run['summary']['passed'] and physical['passed'] and replay['verified']
                   and replay['ticks'] == run['summary']['steps'] and (not local or release['passed'])),
    }


def run_case(binary, folder, case, seed, local=True):
    folder.mkdir(parents=True, exist_ok=False)
    arguments = ['lane-route', '--map', REPO/case['map'], '--start', case['start'], '--goal', case['goal'],
                 '--cruise-speed', 6, '--duration', 120, '--scenario-output', folder/'scenario.json']
    for name in case['closed_lanes']:
        arguments.extend(['--closed-lane', name])
    if local:
        arguments.append('--local-speed-limits')
    code = execute(binary, arguments, folder/'export-process')
    row = {'case': case['case'], 'seed': seed, 'local_speed_limits': local, 'export_exit_code': code, 'passed': False}
    if code:
        return row
    row['run_exit_code'] = execute(binary, ['run', '--scenario', folder/'scenario.json', '--seed', seed, '--output', folder/'run'], folder/'run-process')
    row['replay_exit_code'] = execute(binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        row.update(audit(folder, case, local))
    except Exception as error:
        row['audit_error'] = repr(error)
    row['passed'] &= row['run_exit_code'] == row['replay_exit_code'] == 0
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    binary = 'rustdriving.exe' if os.name == 'nt' else 'rustdriving'
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/binary)
    args = parser.parse_args()
    rows = []
    for case in CASES:
        for seed in args.seeds:
            folder = args.output.resolve()/case['case']/f'seed-{seed}'
            rows.append(run_case(args.binary.resolve(), folder, case, seed))
            report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
            (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
