#!/usr/bin/env python3
"""Independently reconstruct connector speed limits and score reference driving."""
import argparse
import json
import math
import os
from pathlib import Path

from check_lane_speed_zones import check_policy
from check_speed_zones import evaluate, execute, progress

REPO = Path(__file__).resolve().parents[1]
CASES = []
for name in ['target-slow', 'source-slow', 'right', 'north', 'end-at-boundary', 'thin-tail']:
    objects = []
    if name == 'target-slow':
        objects = [{'s': 0., 'lateral': 0., 'radius': .75, 'moving_from': 12.,
                    'world_motion': {'position': {'x': 40., 'y': 3.5}, 'velocity': {'x': 4., 'y': 0.}}}]
    CASES.append({'case': name, 'map': f'maps/lanes/change-speeds-{name}.json',
                  'side': 'right' if name == 'right' else 'left',
                  'start_m': 50. if name == 'end-at-boundary' else 25., 'length_m': 30.,
                  'gated': name not in ['end-at-boundary', 'thin-tail'], 'objects': objects,
                  'expected_lane_ids': ['source', 'target']+(['target-mid', 'target-thin'] if name == 'thin-tail' else [])+['target-exit']})


def blend(u):
    # Independent de Casteljau evaluation of the degree-five blend.
    values = [0., 0., 0., 1., 1., 1.]
    while len(values) > 1:
        values = [(1-u)*a+u*b for a, b in zip(values, values[1:])]
    return values[0]


def reference(spec, case):
    lanes = {lane['id']: lane for lane in spec['lanes']}
    source, target = lanes['source'], lanes['target']
    neighbor = source.get(case['side'], {})
    if not neighbor.get('change_allowed') or neighbor.get('lane_id') != 'target':
        raise ValueError('map does not authorize this directed change')
    start, length = case['start_m'], case['length_m']
    origin, end = source['centerline'][0], source['centerline'][-1]
    section_length = math.hypot(end['x']-origin['x'], end['y']-origin['y'])
    def point(lane, distance):
        a, b = lane['centerline'][0], lane['centerline'][-1]
        return {key: a[key]+(b[key]-a[key])*distance/section_length for key in ['x', 'y']}
    points, zones, total = [], [], 0.
    def append(line, cap):
        nonlocal total
        if len(line) < 2:
            return
        before = total
        for a, b in zip(line, line[1:]):
            total += math.hypot(b['x']-a['x'], b['y']-a['y'])
        points.extend(line[1:] if points else line)
        zones.append({'start_s_m': before, 'end_s_m': total, 'speed_m_s': min(8., cap)})
    if start > 0:
        append([source['centerline'][0], point(source, start)], source['speed_limit_m_s'])
    transition = [point(source, start)]
    segments = math.ceil(length/.25)
    for i in range(1, segments+1):
        u = i/segments
        a, b = point(source, start+length*u), point(target, start+length*u)
        f = blend(u)
        transition.append({key: a[key]+f*(b[key]-a[key]) for key in ['x', 'y']})
    if start+length == section_length:
        transition[-1] = target['centerline'][-1]
    append(transition, min(source['speed_limit_m_s'], target['speed_limit_m_s']))
    if start+length < section_length:
        append([transition[-1], target['centerline'][-1]], target['speed_limit_m_s'])
    previous = target
    for name in case['expected_lane_ids'][2:]:
        lane = lanes[name]
        if name not in previous['successors'] or lane['centerline'][0] != previous['centerline'][-1]:
            raise ValueError('tail invents a longitudinal connection')
        append(lane['centerline'], lane['speed_limit_m_s'])
        previous = lane
    return points, zones


def sweep(run):
    if len(run['frames']) != run['summary']['steps']:
        return {'full_tick_coverage': False, 'not_claimed': True}
    previous, minimum, contacts = None, 1000., 0
    xy = lambda p: (p['x'], p['y'])
    for frame in run['frames']:
        p = xy(frame['truth']['pose']['position'])
        old = {o['id']: o for o in previous['objects']} if previous else {}
        tick = 1000.
        for obj in frame['objects']:
            q = xy(obj['position'])
            distance = math.dist(p, q)
            if obj['id'] in old:
                a = [v-w for v, w in zip(xy(previous['truth']['pose']['position']), xy(old[obj['id']]['position']))]
                b = [p[i]-q[i]-a[i] for i in range(2)]
                norm = sum(v*v for v in b)
                t = max(0., min(1., -sum(v*w for v, w in zip(a, b))/norm)) if norm else 0.
                distance = min(distance, math.hypot(*(a[i]+t*b[i] for i in range(2))))
            tick = min(tick, distance-run['vehicle']['radius']-obj['radius'])
        minimum = min(minimum, tick)
        contacts += tick < 0.
        previous = frame
    if abs(minimum-run['summary']['min_clearance']) > 1e-8 or minimum < 1. or contacts:
        raise ValueError('independent physical clearance failed')
    return {'full_tick_coverage': True, 'minimum_swept_clearance_m': minimum, 'contact_intervals': contacts}


def audit_data(run, scenario, report, spec, case, config, local):
    points, zones = reference(spec, case)
    actual = run['route']['points']
    if len(points) != len(actual) or max(math.hypot(p['x']-q['x'], p['y']-q['y']) for p, q in zip(points, actual)) > 1e-8:
        raise ValueError('recorded connector differs from independent map reconstruction')
    if report['plan']['route'] != run['route'] or report['plan']['lane_ids'] != case['expected_lane_ids']:
        raise ValueError('recorded geometry/itinerary differs from the map report')
    if {'objects', 'min_clearance_m', 'goal_hold_seconds', 'world_motion'} & config.keys():
        raise ValueError('world/evaluation data entered the driver configuration')
    if config.get('speed_zones', []) != scenario.get('speed_zones', []):
        raise ValueError('speed configuration missing from sensor replay')
    if local:
        check_policy(scenario['speed_zones'], zones)
        if scenario['cruise_speed'] != 4. or scenario['speed_zones'] != report['speed_zones']:
            raise ValueError('requested cruise or compiled policy changed')
        configured = evaluate(run)
    else:
        configured = None
    score = evaluate(run, zones)
    if configured and configured['violation_intervals'] != score['violation_intervals']:
        raise ValueError('mapped and configured physical scores disagree')
    last = max(z['end_s_m'] for z in zones if z['speed_m_s'] < 4.)
    lower, upper = last+run['vehicle']['radius']+8., zones[-1]['end_s_m']-10.
    maximum = max((f['truth']['speed'] for f in run['frames'] if lower <= progress(f['truth']['pose']['position'], actual) <= upper), default=0.)
    release = {'passed': maximum >= 3.5, 'maximum_speed_m_s': maximum, 'minimum_required_max_speed_m_s': 3.5,
               'interval_start_s_m': lower, 'interval_end_s_m': upper}
    result = {'summary': run['summary'], 'mapped_speed_compliance': score, 'configured_speed_compliance': configured,
              'post_zone_reacceleration': release, 'independent_sweep': sweep(run), 'map_reference_sections': zones}
    if case['gated'] and not any(f.get('lane_change', {}).get('phase') == 'Completed' for f in run['frames']):
        raise ValueError('fixed gap-admission maneuver never completed')
    if local and case['case'] == 'target-slow':
        frames = run['frames']
        leaves = [f['time'] for f in frames if abs(f['truth']['pose']['position']['y'])+run['vehicle']['radius'] > 1.75]
        waiting = [f for f in frames if f.get('lane_change', {}).get('phase') == 'Waiting' and f['lane_change']['blocking_tracks'] and f['truth']['speed'] < .2]
        held = all(abs(f['truth']['pose']['position']['y'])+run['vehicle']['radius'] <= 1.75+1e-6 for f in frames if f['time'] < 12.)
        if not held or not waiting or not leaves or min(leaves) < 13.:
            raise ValueError('physical source hold under observed traffic failed')
        result['traffic_source_hold'] = {'passed': True, 'source_contained_until_s': 12., 'first_physical_source_leave_s': min(leaves),
                                        'stationary_blocked_frames': len(waiting)}
    result['passed'] = run['summary']['passed'] and score['passed'] and (not local or release['passed'])
    return result


def run_case(binary, folder, case, seed, local=True):
    folder.mkdir(parents=True, exist_ok=False)
    args = ['lane-route', '--map', REPO/case['map'], '--start', 'source', '--goal', 'target-exit',
            '--change-side', case['side'], '--change-start', case['start_m'], '--change-length', case['length_m'],
            '--cruise-speed', 4, '--duration', 120, '--scenario-output', folder/'exported-scenario.json']
    if case['gated']: args.append('--wait-for-gap')
    if local: args.append('--local-speed-limits')
    code = execute(binary, args, folder/'export-process')
    row = {'case': case['case'], 'seed': seed, 'local_speed_limits': local, 'export_exit_code': code, 'passed': False}
    if code: return row
    scenario = json.loads((folder/'exported-scenario.json').read_text())
    scenario['objects'] = case['objects']
    scenario['min_clearance_m'] = 1.
    (folder/'scenario.json').write_text(json.dumps(scenario, indent=2)+'\n')
    row['run_exit_code'] = execute(binary, ['run', '--scenario', folder/'scenario.json', '--seed', seed, '--output', folder/'run'], folder/'run-process')
    row['replay_exit_code'] = execute(binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        run = json.loads((folder/'run/run.json').read_text())
        report = json.loads((folder/'export-process.stdout').read_text())
        with (folder/'run/sensors.jsonl').open() as stream: config = json.loads(next(stream))['header']['config']
        row.update(audit_data(run, scenario, report, json.loads((REPO/case['map']).read_text()), case, config, local))
        replay = json.loads((folder/'replay/replay.json').read_text())
        row['replay'] = replay
        row['passed'] &= replay['verified'] and replay['ticks'] == run['summary']['steps']
    except Exception as error:
        row['audit_error'] = repr(error)
    row['passed'] &= row['run_exit_code'] == row['replay_exit_code'] == 0
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    name = 'rustdriving.exe' if os.name == 'nt' else 'rustdriving'
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/name)
    args = parser.parse_args()
    rows = []
    for case in CASES:
        for seed in args.seeds:
            rows.append(run_case(args.binary.resolve(), args.output.resolve()/case['case']/f'seed-{seed}', case, seed))
            report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
            (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__': main()
