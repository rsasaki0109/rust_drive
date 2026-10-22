#!/usr/bin/env python3
"""Independently reconstruct and score early-return mission retry speed policies."""
import argparse
import json
import math
import os
from pathlib import Path

from check_lane_change_speeds import reference, sweep
from check_lane_speed_zones import check_policy
from check_speed_zones import execute, progress

REPO = Path(__file__).resolve().parents[1]
CASES = []
for name in ['left', 'right', 'north', 'thin-tail', 'blocked', 'launch-blocked']:
    def transform(x, y):
        if name == 'right': return {'x': x, 'y': -y}
        if name == 'north': return {'x': -y, 'y': x}
        return {'x': x, 'y': y}
    objects = [{'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': 20., 'moving_from': 40.,
                'world_motion': {'position': transform(52., 3.5), 'velocity': transform(0. if name == 'blocked' else 4., 0.)}}]
    if name == 'launch-blocked':
        objects.append({'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': 49.,
                        'world_motion': {'position': {'x': 54., 'y': 3.5}, 'velocity': {'x': 0., 'y': 0.}}})
    CASES.append({'case': name, 'map': f'maps/lanes/resume-speeds-{name}.json',
                  'side': 'right' if name == 'right' else 'left', 'start_m': 25., 'length_m': 30.,
                  'expected_lane_ids': ['source', 'target']+(['target-mid', 'target-thin'] if name == 'thin-tail' else [])+['target-exit'],
                  'cruise_speed': 2. if name == 'launch-blocked' else 4., 'objects': objects,
                  'goal': name not in ['blocked', 'launch-blocked'], 'require_launch_hold': name == 'launch-blocked'})


def resume_reference(original, zones, request, cap):
    """Independent Bernstein connector, original-tail clipping and new arc origin."""
    a, b = request['target_start'], request['target_end']
    length = math.dist((a['x'], a['y']), (b['x'], b['y']))
    axis = ((b['x']-a['x'])/length, (b['y']-a['y'])/length)
    start = request['start']['position']
    dx, dy = start['x']-a['x'], start['y']-a['y']
    x0, y0 = dx*axis[0]+dy*axis[1], axis[0]*dy-axis[1]*dx
    distance = request['distance_m']
    derivative = distance*math.tan(request['start']['yaw']-math.atan2(axis[1], axis[0]))
    controls = [y0, y0+derivative/5, y0+2*derivative/5, 0., 0., 0.]
    count = math.ceil(distance/.25)
    points = []
    for i in range(count+1):
        u, values = i/count, list(controls)
        while len(values) > 1: values = [(1-u)*v+u*w for v, w in zip(values, values[1:])]
        x, y = x0+distance*u, values[0]
        points.append({'x': a['x']+axis[0]*x-axis[1]*y, 'y': a['y']+axis[1]*x+axis[0]*y})
    connector = sum(math.dist((p['x'], p['y']), (q['x'], q['y'])) for p, q in zip(points, points[1:]))
    join = progress(points[-1], original)
    s = 0.
    for i in range(1, len(original)):
        p, q = original[i-1], original[i]
        s += math.dist((p['x'], p['y']), (q['x'], q['y']))
        if s > join+1e-6: points.append(dict(q))
    mapped = [{'start_s_m': 0., 'end_s_m': connector, 'speed_m_s': cap}]
    mapped += [{'start_s_m': connector+max(0., z['start_s_m']-join),
                'end_s_m': connector+z['end_s_m']-join, 'speed_m_s': z['speed_m_s']}
               for z in zones if z['end_s_m'] > join]
    return points, mapped, join, connector


def interval_excess(points, zones, previous, current, radius):
    start = (progress(previous['pose']['position'], points), previous['speed'])
    end = (progress(current['pose']['position'], points), current['speed'])
    maximum, covered = 0., []
    for i, zone in enumerate(zones):
        low, high = zone['start_s_m']-radius, zone['end_s_m']+radius
        delta = end[0]-start[0]
        if abs(delta) < 1e-12:
            fractions = [0., 1.] if low <= start[0] <= high else []
        else:
            roots = sorted([(low-start[0])/delta, (high-start[0])/delta])
            a, b = max(0., roots[0]), min(1., roots[1])
            fractions = [a, b] if a <= b else []
        if fractions:
            maximum = max(maximum, max(start[1]+f*(end[1]-start[1]) for f in fractions)-zone['speed_m_s'])
            covered.append(i)
    return maximum, covered


def speed_audit(run, original, original_zones, cap):
    if len(run['frames']) != run['summary']['steps']:
        raise ValueError('retry scoring requires every physical integration frame')
    points, zones, phase = original, original_zones, 'original'
    previous, request, resumed, covered = None, None, None, set()
    maximum, violations = 0., 0
    for frame in run['frames']:
        truth, state = frame['truth'], frame['lane_change']
        excess, touched = interval_excess(points, zones, previous or truth, truth, run['vehicle']['radius'])
        if phase == 'resume': covered.update(touched)
        retry = state.get('return_state', {}).get('resume_state', {})
        new_request = retry.get('request')
        switched = False
        if new_request is not None:
            if request is not None and new_request != request: raise ValueError('retry connector was replaced')
            if phase != 'resume':
                request = new_request
                points, zones, join, connector = resume_reference(original, original_zones, request, cap)
                resumed = {'points': points, 'zones': zones, 'join_s_m': join, 'connector_length_m': connector}
                phase, switched = 'resume', True
                if abs(retry['connector_length_m']-connector) > 1e-8: raise ValueError('reported retry connector differs from independent geometry')
        elif phase == 'original' and state['phase'] in ['Aborting', 'Aborted']:
            zones = [{'start_s_m': 0., 'end_s_m': original_zones[-1]['end_s_m'], 'speed_m_s': cap}]
            phase, switched = 'return', True
        if switched:
            instant, touched = interval_excess(points, zones, truth, truth, run['vehicle']['radius'])
            excess = max(excess, instant)
            if phase == 'resume': covered.update(touched)
        maximum = max(maximum, excess)
        violations += excess > .15+1e-9
        previous = truth
    return {'passed': violations == 0, 'violation_intervals': violations, 'maximum_excess_m_s': maximum,
            'physical_frames': len(run['frames']), 'tolerance_m_s': .15,
            'resumed_zone_covered_indices': sorted(covered)}, resumed


def audit_data(run, scenario, report, spec, case, config, local):
    original, zones = reference(spec, case)
    actual = run['route']['points']
    if len(actual) != len(original) or max(math.dist((p['x'], p['y']), (q['x'], q['y'])) for p, q in zip(original, actual)) > 1e-8:
        raise ValueError('initial geometry differs from the original lane map')
    lanes = {lane['id']: lane for lane in spec['lanes']}
    permission = lanes['target'].get('left' if case['side'] == 'right' else 'right', {})
    if permission.get('lane_id') != 'source' or not permission.get('change_allowed'):
        raise ValueError('map does not permit the return crossing')
    cap = min(8., lanes['source']['speed_limit_m_s'], lanes['target']['speed_limit_m_s'])
    if config['lane_change'] != scenario['lane_change'] or config.get('speed_zones', []) != scenario.get('speed_zones', []):
        raise ValueError('replay omitted the original or alternate speed policy')
    if {'objects', 'min_clearance_m', 'goal_hold_seconds', 'world_motion'} & config.keys(): raise ValueError('world state entered operational configuration')
    if report['lane_change_gate'] != scenario['lane_change'] or report['plan']['lane_ids'] != case['expected_lane_ids']:
        raise ValueError('export and scenario disagree')
    if local:
        check_policy(scenario['speed_zones'], zones)
        if scenario['lane_change']['return_on_conflict']['speed_limit_m_s'] != cap or scenario['cruise_speed'] != case['cruise_speed']:
            raise ValueError('mapped cap or requested cruise changed')
    score, resumed = speed_audit(run, original, zones, cap)
    result = {'passed': run['summary']['passed'] and score['passed'], 'summary': run['summary'],
              'mapped_speed_compliance': score, 'independent_sweep': sweep(run)}
    if not local: return result
    a, b = lanes['source']['centerline'][0], lanes['source']['centerline'][-1]
    length = math.dist((a['x'], a['y']), (b['x'], b['y']))
    axis = ((b['x']-a['x'])/length, (b['y']-a['y'])/length)
    def contained(f):
        p = f['truth']['pose']['position']
        lateral = axis[0]*(p['y']-a['y'])-axis[1]*(p['x']-a['x'])
        return abs(lateral)+run['vehicle']['radius'] <= lanes['source']['width_m']/2+1e-9
    held_since, hold, returned = None, 0., False
    for f in run['frames']:
        returned |= f['lane_change']['phase'] == 'Aborting'
        if f['lane_change']['phase'] == 'Aborted' and contained(f) and abs(f['truth']['speed']) <= .05:
            if held_since is None: held_since = f['time']
            hold = max(hold, f['time']-held_since)
        else: held_since = None
    if not returned or hold < 1.: raise ValueError('missing physical source-contained return stop')
    last = run['frames'][-1]
    if case['goal']:
        if resumed is None or last['lane_change']['phase'] != 'Completed' or not run['summary']['reached_goal']:
            raise ValueError('retry failed to complete the original mission')
        if score['resumed_zone_covered_indices'] != list(range(len(resumed['zones']))): raise ValueError('a retry speed interval was never exercised')
        lower = max(z['end_s_m'] for z in resumed['zones'] if z['speed_m_s'] < 4.)+run['vehicle']['radius']+8.
        upper = resumed['zones'][-1]['end_s_m']-10.
        speed = max((f['truth']['speed'] for f in run['frames'] if f['lane_change']['return_state']['resume_state'].get('request') is not None and lower <= progress(f['truth']['pose']['position'], resumed['points']) <= upper), default=0.)
        if speed < 3.5: raise ValueError('cruise was not restored in the fixed post-limit probe')
        result['post_limit_maximum_speed_m_s'] = speed
    elif case['require_launch_hold']:
        hold_state = last['lane_change']['return_state']['resume_state'].get('launch_hold')
        if not hold_state or hold_state['phase'] == 'Released' or not contained(last) or last['truth']['speed'] > .05:
            raise ValueError('new launch conflict failed the physical source hold')
        result['launch_hold'] = hold_state
    elif resumed is not None or last['lane_change']['phase'] != 'Aborted': raise ValueError('blocked target incorrectly authorized a retry')
    result.update(source_stop_hold_s=hold, retry_started=bool(resumed))
    return result


def run_case(binary, folder, case, seed, local=True):
    folder.mkdir(parents=True, exist_ok=False)
    args = ['lane-route', '--map', REPO/case['map'], '--start', 'source', '--goal', 'target-exit',
            '--change-side', case['side'], '--change-start', 25, '--change-length', 30,
            '--wait-for-gap', '--return-on-conflict', '--resume-after-return', '--cruise-speed', case['cruise_speed'],
            '--duration', 120, '--scenario-output', folder/'exported-scenario.json']
    if local: args.append('--local-speed-limits')
    row = {'case': case['case'], 'seed': seed, 'local_speed_limits': local, 'passed': False}
    row['export_exit_code'] = execute(binary, args, folder/'export-process')
    if row['export_exit_code']: return row
    scenario = json.loads((folder/'exported-scenario.json').read_text())
    scenario.update(objects=case['objects'] if local else [], min_clearance_m=1., expected='goal' if (case['goal'] or not local) else 'stop')
    (folder/'scenario.json').write_text(json.dumps(scenario, indent=2)+'\n')
    row['run_exit_code'] = execute(binary, ['run', '--scenario', folder/'scenario.json', '--seed', seed, '--output', folder/'run'], folder/'run-process')
    row['replay_exit_code'] = execute(binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        run = json.loads((folder/'run/run.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream: config = json.loads(next(stream))['header']['config']
        row.update(audit_data(run, scenario, json.loads((folder/'export-process.stdout').read_text()), json.loads((REPO/case['map']).read_text()), case, config, local))
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        row['passed'] &= row['replay']['verified'] and row['replay']['ticks'] == run['summary']['steps']
    except Exception as error: row['audit_error'] = repr(error)
    row['passed'] &= row['run_exit_code'] == row['replay_exit_code'] == 0
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
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
