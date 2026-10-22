#!/usr/bin/env python3
"""Score mapped early returns from full physical frames and original lane maps."""
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
for name in ['left', 'right', 'north', 'target-slow', 'source-blocked', 'late']:
    onset = 24. if name == 'late' else 20.
    positions = [(52., 3.5)] + ([(40., 0.)] if name == 'source-blocked' else [])
    def transform(x, y):
        if name == 'right': return {'x': x, 'y': -y}
        if name == 'north': return {'x': -y, 'y': x}
        return {'x': x, 'y': y}
    objects = [{'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': onset,
                'world_motion': {'position': transform(x, y), 'velocity': {'x': 0., 'y': 0.}}}
               for x, y in positions]
    CASES.append({'case': name, 'map': f'maps/lanes/return-speeds-{name}.json',
                  'side': 'right' if name == 'right' else 'left', 'cruise_speed': 2. if name == 'target-slow' else 4.,
                  'onset_s': onset, 'objects': objects, 'expect_return': name not in ['source-blocked', 'late'],
                  'rejection': 'SourceBlocked' if name == 'source-blocked' else ('OutsideSource' if name == 'late' else None),
                  'start_m': 25., 'length_m': 30., 'expected_lane_ids': ['source', 'target', 'target-exit']})


def speed_audit(run, zones, return_cap):
    """The old policy applies through the switch; the new cap starts at it."""
    frames = run['frames']
    if len(frames) != run['summary']['steps']:
        raise ValueError('return scoring requires every physical integration frame')
    previous, returning, violations, maximum, returned_frames = None, False, 0, 0., 0
    radius = run['vehicle']['radius']
    for frame in frames:
        end = (progress(frame['truth']['pose']['position'], run['route']['points']), frame['truth']['speed'])
        start = previous or end
        excess = 0.
        if returning:
            excess = max(start[1], end[1])-return_cap
        else:
            for zone in zones:
                low, high = zone['start_s_m']-radius, zone['end_s_m']+radius
                delta = end[0]-start[0]
                if abs(delta) < 1e-12:
                    fractions = [0., 1.] if low <= start[0] <= high else []
                else:
                    roots = sorted([(low-start[0])/delta, (high-start[0])/delta])
                    a, b = max(0., roots[0]), min(1., roots[1])
                    fractions = [a, b] if a <= b else []
                if fractions:
                    excess = max(excess, max(start[1]+f*(end[1]-start[1]) for f in fractions)-zone['speed_m_s'])
        switched = frame['lane_change']['phase'] in ['Aborting', 'Aborted']
        if switched and not returning:
            excess = max(excess, end[1]-return_cap)
            returning = True
        returned_frames += returning
        maximum = max(maximum, excess)
        violations += excess > .15+1e-9
        previous = end
    return {'passed': violations == 0, 'violation_intervals': violations,
            'maximum_excess_m_s': maximum, 'tolerance_m_s': .15,
            'return_cap_m_s': return_cap, 'return_frames': returned_frames,
            'physical_frames': len(frames), 'full_tick_coverage': True}


def audit_data(run, scenario, report, spec, case, config, local):
    points, zones = reference(spec, case)
    actual = run['route']['points']
    if len(points) != len(actual) or max(math.dist((p['x'], p['y']), (q['x'], q['y'])) for p, q in zip(points, actual)) > 1e-8:
        raise ValueError('initial geometry differs from independent map reconstruction')
    lanes = {lane['id']: lane for lane in spec['lanes']}
    reverse = 'left' if case['side'] == 'right' else 'right'
    permission = lanes['target'].get(reverse, {})
    if permission.get('lane_id') != 'source' or not permission.get('change_allowed'):
        raise ValueError('the map does not authorize returning across the boundary')
    cap = min(8., lanes['source']['speed_limit_m_s'], lanes['target']['speed_limit_m_s'])
    gate = scenario['lane_change']
    if config.get('speed_zones', []) != scenario.get('speed_zones', []) or config['lane_change'] != gate:
        raise ValueError('replay omits the initial or alternate speed policy')
    if {'objects', 'min_clearance_m', 'world_motion', 'goal_hold_seconds'} & config.keys():
        raise ValueError('evaluation/world state entered operational configuration')
    if report['plan']['lane_ids'] != case['expected_lane_ids'] or report['lane_change_gate'] != gate:
        raise ValueError('export and recorded mission disagree')
    if local:
        check_policy(scenario['speed_zones'], zones)
        if gate['return_on_conflict']['speed_limit_m_s'] != cap or scenario['cruise_speed'] != case['cruise_speed']:
            raise ValueError('return cap or requested cruise differs from the map contract')
    elif gate['return_on_conflict'].get('speed_limit_m_s') is not None:
        raise ValueError('option-free export unexpectedly contains an alternate speed policy')
    score = speed_audit(run, zones, cap)
    a, b = lanes['source']['centerline'][0], lanes['source']['centerline'][-1]
    length = math.dist((a['x'], a['y']), (b['x'], b['y']))
    axis = ((b['x']-a['x'])/length, (b['y']-a['y'])/length)
    def lateral(frame):
        p = frame['truth']['pose']['position']
        return axis[0]*(p['y']-a['y'])-axis[1]*(p['x']-a['x'])
    frames = run['frames']
    returning = [f for f in frames if f['lane_change']['phase'] == 'Aborting']
    peak = max(abs(lateral(f)) for f in frames)
    if peak <= .1 or bool(returning) != case['expect_return']:
        raise ValueError('physical change/return outcome differs from the frozen expectation')
    stop_hold = 0.
    if case['expect_return']:
        if returning[0]['time'] < case['onset_s'] or frames[-1]['lane_change']['phase'] != 'Aborted':
            raise ValueError('return did not follow the conflict and finish at a measured stop')
        if abs(lateral(frames[-1])) > .25:
            raise ValueError('return failed source-center stopping accuracy')
        for f in reversed(frames):
            if (f['lane_change']['phase'] != 'Aborted' or abs(f['truth']['speed']) > .05
                    or abs(lateral(f))+run['vehicle']['radius'] > lanes['source']['width_m']/2):
                break
            stop_hold = frames[-1]['time']-f['time']
        if stop_hold < 2.:
            raise ValueError('physical source-contained standstill lasted less than two seconds')
    elif not any(f['lane_change']['return_state']['decision'] == case['rejection'] for f in frames):
        raise ValueError('unsafe return lacks the required observed rejection reason')
    return {'passed': run['summary']['passed'] and score['passed'], 'summary': run['summary'],
            'mapped_speed_compliance': score, 'independent_sweep': sweep(run),
            'return_started_s': returning[0]['time'] if returning else None,
            'final_lateral_m': lateral(frames[-1]), 'source_stop_hold_s': stop_hold,
            'peak_absolute_lateral_m': peak}


def run_case(binary, folder, case, seed, local=True):
    folder.mkdir(parents=True, exist_ok=False)
    args = ['lane-route', '--map', REPO/case['map'], '--start', 'source', '--goal', 'target-exit',
            '--change-side', case['side'], '--change-start', 25, '--change-length', 30,
            '--wait-for-gap', '--return-on-conflict', '--cruise-speed', case['cruise_speed'],
            '--duration', 70, '--scenario-output', folder/'exported-scenario.json']
    if local: args.append('--local-speed-limits')
    row = {'case': case['case'], 'seed': seed, 'local_speed_limits': local, 'passed': False}
    row['export_exit_code'] = execute(binary, args, folder/'export-process')
    if row['export_exit_code']: return row
    scenario = json.loads((folder/'exported-scenario.json').read_text())
    scenario.update(objects=case['objects'], min_clearance_m=1., expected='stop')
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
