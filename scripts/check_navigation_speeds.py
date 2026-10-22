#!/usr/bin/env python3
"""Independently score mapped edge speeds across stopped route handovers."""
import argparse
import heapq
import json
import math
import os
from pathlib import Path

from check_lane_change_speeds import sweep
from check_lane_resume_speeds import interval_excess
from check_speed_zones import execute, progress

REPO = Path(__file__).resolve().parents[1]
CASES = [{'case': name, 'scenario': f'scenarios/navigation-speed-{name}.json',
          'goal': name != 'blocked', 'expected_switches': 0 if name in ['blocked', 'static'] else 1}
         for name in ['handover', 'reopen', 'north', 'thin', 'blocked', 'uncapped', 'static']]


def map_plan(nav, closed):
    network = nav['network']
    if network.get('turn_restrictions'): raise ValueError('reference protocol requires its authored unrestricted graphs')
    nodes = {n['id']: n['position'] for n in network['nodes']}
    edges = {e['id']: e for e in network['edges']}
    def canonical(edge):
        return [nodes[edge['from']]]+edge['points'][1:-1]+[nodes[edge['to']]]
    def length(points):
        return sum(math.dist((a['x'], a['y']), (b['x'], b['y'])) for a, b in zip(points, points[1:]))
    pending, best = [(0., (), nav['start'])], {}
    selected = None
    while pending:
        distance, ids, node = heapq.heappop(pending)
        if node in best: continue
        best[node] = distance
        if node == nav['goal']:
            selected = ids
            break
        for eid, edge in sorted(edges.items()):
            if edge['from'] == node and eid not in closed and edge['to'] not in best:
                heapq.heappush(pending, (distance+length(canonical(edge)), ids+(eid,), edge['to']))
    if selected is None: return None
    points, zones, distance = [], [], 0.
    for eid in selected:
        edge = edges[eid]
        line = canonical(edge)
        start = distance
        distance += length(line)
        points.extend(line[1:] if points else line)
        if edge.get('speed_limit_m_s') is not None:
            zones.append({'start_s_m': start, 'end_s_m': distance, 'speed_m_s': min(8., edge['speed_limit_m_s'])})
    return {'edge_ids': list(selected), 'points': points, 'zones': zones, 'length_m': distance}


def closed_at(scenario, time):
    closed = scenario['navigation'].get('closed_edges', [])
    for update in scenario.get('navigation_updates', []):
        if update['stamp'] <= time+1e-9: closed = update['closed_edges']
    return closed


def audit(run, scenario, case, local):
    frames = run['frames']
    if local and len(frames) != run['summary']['steps']: raise ValueError('mapped speeds require full physical frames')
    initial = map_plan(scenario['navigation'], scenario['navigation'].get('closed_edges', []))
    history = run.get('route_history', [])
    transitions = history or [{'time': 0., 'plan': {'edge_ids': initial['edge_ids'], 'route': {'points': initial['points']}}}]
    if history and (history[0]['time'] != 0. or any(b['time'] <= a['time'] for a,b in zip(history,history[1:]))):
        raise ValueError('route history must start at zero and increase')
    expected = []
    for i, transition in enumerate(transitions):
        reference = map_plan(scenario['navigation'], closed_at(scenario, transition['time']))
        if reference is None or reference['edge_ids'] != transition['plan']['edge_ids']:
            raise ValueError('route switch disagrees with independent closure-aware search')
        actual = transition['plan']['route']['points']
        if len(actual) != len(reference['points']) or max(math.dist((a['x'], a['y']), (b['x'], b['y'])) for a, b in zip(actual, reference['points'])) > 1e-8:
            raise ValueError('selected geometry disagrees with original map')
        if i and (abs(transition['true_speed']) > .05 or abs(transition['estimated_speed']) > .05):
            raise ValueError('route changed without a physical and estimated stop')
        expected.append(reference)
    if len(transitions)-1 != case['expected_switches']: raise ValueError('wrong number of route handovers')
    active, previous, maximum, violations, covered = 0, None, 0., 0, set()
    peak, stationary_since, blocked_hold = 0., None, 0.
    for frame in frames:
        truth = frame['truth']
        plan = expected[active]
        excess, touched = interval_excess(plan['points'], plan['zones'], previous or truth, truth, run['vehicle']['radius'])
        covered.update(touched)
        if active+1 < len(transitions) and frame['time'] >= transitions[active+1]['time']-1e-9:
            active += 1
            plan = expected[active]
            covered = set()
            at_switch, touched = interval_excess(plan['points'], plan['zones'], truth, truth, run['vehicle']['radius'])
            covered.update(touched)
            excess = max(excess, at_switch)
        state = frame.get('navigation')
        if state and (state['active_edges'] != plan['edge_ids'] or state['switches'] != active):
            raise ValueError('reported active edges/policy timing disagree')
        maximum = max(maximum, excess)
        violations += excess > .15+1e-9
        if case['case'] in ['reopen', 'blocked'] and frame['time'] < 15.:
            initial_s = progress(truth['pose']['position'], initial['points'])
            if initial_s+run['vehicle']['radius'] >= 40.: raise ValueError('ego crossed the closed fork before reopening')
            if abs(truth['speed']) <= .05:
                if stationary_since is None: stationary_since = frame['time']
                blocked_hold = max(blocked_hold, frame['time']-stationary_since)
            else: stationary_since = None
        if local and active == len(expected)-1 and case['goal']:
            low = [z['end_s_m'] for z in plan['zones'] if z['speed_m_s'] < 4.]
            lower = (max(low) if low else 40.)+run['vehicle']['radius']+8.
            upper = plan['length_m']-10.
            if case['case'] == 'uncapped': upper = next(z['start_s_m'] for z in plan['zones'] if z['start_s_m'] > 40.)-10.
            if lower <= progress(truth['pose']['position'], plan['points']) <= upper:
                peak = max(peak, truth['speed'])
        previous = truth
    if local and case['goal'] and covered != set(range(len(expected[-1]['zones']))): raise ValueError('an active speed interval was not exercised')
    if local and case['goal'] and peak < 3.5: raise ValueError('cruise failed the fixed post-limit probe')
    if case['case'] == 'reopen' and blocked_hold < 2.: raise ValueError('no-route wait lacked two seconds of physical standstill')
    if not case['goal'] and (frames[-1]['truth']['speed'] > .05 or frames[-1]['navigation']['phase'] != 'Blocked'):
        raise ValueError('unreachable route did not remain stopped')
    return {'passed': run['summary']['passed'] and (not local or violations == 0), 'summary': run['summary'],
            'mapped_speed_compliance': {'passed': violations == 0, 'violation_intervals': violations, 'maximum_excess_m_s': maximum,
                                        'tolerance_m_s': .15, 'physical_frames': len(frames), 'full_tick_coverage': len(frames)==run['summary']['steps']},
            'post_limit_maximum_speed_m_s': peak, 'pre_reopening_stationary_hold_s': blocked_hold,
            'independent_sweep': sweep(run), 'route_switches': len(transitions)-1}


def run_case(binary, folder, case, seed, local=True):
    folder.mkdir(parents=True, exist_ok=False)
    scenario = json.loads((REPO/case['scenario']).read_text())
    if not local:
        for edge in scenario['navigation']['network']['edges']: edge.pop('speed_limit_m_s', None)
    (folder/'scenario.json').write_text(json.dumps(scenario, indent=2)+'\n')
    row = {'case': case['case'], 'seed': seed, 'local_speed_limits': local, 'passed': False}
    row['run_exit_code'] = execute(binary, ['run', '--scenario', folder/'scenario.json', '--seed', seed, '--output', folder/'run'], folder/'run-process')
    row['replay_exit_code'] = execute(binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        run = json.loads((folder/'run/run.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream: config = json.loads(next(stream))['header']['config']
        if local and config.get('navigation', {}).get('network') != scenario['navigation']['network']: raise ValueError('replay omitted original map speed metadata')
        if config.get('speed_zones') or {'objects', 'min_clearance_m', 'goal_hold_seconds'} & config.keys(): raise ValueError('ambiguous policy or world state entered configuration')
        row.update(audit(run, scenario, case, local))
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
