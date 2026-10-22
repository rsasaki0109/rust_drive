#!/usr/bin/env python3
"""Exhaustively reconstruct directed partial routes and score physical driving."""
import argparse
import hashlib
import itertools
import json
import math
import os
from pathlib import Path

from check_gnss_map_history import archive
from check_lane_change_speeds import sweep
from check_lane_resume_speeds import interval_excess
from check_speed_zones import execute, progress

REPO = Path(__file__).resolve().parents[1]
CASES = ['same-edge', 'obstacle', 'clipped', 'restricted', 'handover', 'reopen', 'blocked-goal', 'osm']
xy = lambda p: (p['x'], p['y'])


def reference(nav, closed):
    """Enumerate simple arrival-state paths, independent of producer Dijkstra.

    Positive edge costs mean an optimal path never repeats an arrival state.
    Endpoint prefixes are terminal transitions, including a repeated origin edge.
    This small-map evaluator explicitly refuses an excessive enumeration budget.
    """
    nodes = {n['id']: n['position'] for n in nav['network']['nodes']}
    edges = {e['id']: dict(e) for e in nav['network']['edges']}
    for edge in edges.values():
        edge['points'] = [nodes[edge['from']]]+edge['points'][1:-1]+[nodes[edge['to']]]
        edge['length'] = sum(math.dist(xy(a), xy(b)) for a, b in zip(edge['points'], edge['points'][1:]))
    if any(e not in edges for e in closed):
        raise ValueError('unknown closure')
    start, goal = nav['start'], nav['goal']
    for endpoint in [start, goal]:
        if isinstance(endpoint, str):
            if endpoint not in nodes: raise ValueError('unknown endpoint node')
        elif (set(endpoint) != {'edge', 's_m'} or endpoint['edge'] not in edges
              or not math.isfinite(endpoint['s_m']) or not 0. < endpoint['s_m'] < edges[endpoint['edge']]['length']):
            raise ValueError('invalid interior edge position')
        elif endpoint['edge'] in closed:
            return None
    if start == goal:
        return None

    def allowed(incoming, outgoing):
        for rule in nav['network'].get('turn_restrictions', []):
            if incoming == rule['from_edge']:
                if rule['kind'] == 'no' and outgoing == rule['to_edge']: return False
                if rule['kind'] == 'only' and outgoing != rule['to_edge']: return False
        return True

    best, visits = None, 0
    if isinstance(start, dict) and isinstance(goal, dict) and start['edge'] == goal['edge'] and start['s_m'] < goal['s_m']:
        best = (goal['s_m']-start['s_m'], [(start['edge'], start['s_m'], goal['s_m'])])
    else:
        def search(node, incoming, spans, cost, visited):
            nonlocal best, visits
            visits += 1
            if visits > 500_000: raise ValueError('independent exhaustive path budget exceeded')
            if best and cost >= best[0]: return
            terminal = node == goal if isinstance(goal, str) else node == edges[goal['edge']]['from'] and allowed(incoming, goal['edge'])
            if terminal:
                candidate = spans if isinstance(goal, str) else spans+[(goal['edge'], 0., goal['s_m'])]
                distance = cost+(0. if isinstance(goal, str) else goal['s_m'])
                if best is None or distance < best[0]: best = (distance, candidate)
                return
            for eid, edge in sorted(edges.items()):
                if edge['from'] == node and eid not in closed and eid not in visited and allowed(incoming, eid):
                    search(edge['to'], eid, spans+[(eid, 0., edge['length'])], cost+edge['length'], visited|{eid})
        if isinstance(start, str):
            search(start, None, [], 0., set())
        else:
            eid, s = start['edge'], start['s_m']
            search(edges[eid]['to'], eid, [(eid, s, edges[eid]['length'])], edges[eid]['length']-s, {eid})
    if best is None: return None
    points, zones, traversals, node_ids, distance = [], [], [], [start] if isinstance(start, str) else [], 0.
    width = min(edges[eid]['half_width'] for eid, _, _ in best[1])
    for eid, low, high in best[1]:
        edge = edges[eid]
        # Clip every source segment in arc coordinates, preserving interior knots.
        line, cursor = [], 0.
        for a, b in zip(edge['points'], edge['points'][1:]):
            length = math.dist(xy(a), xy(b))
            first, last = max(low, cursor), min(high, cursor+length)
            if first < last:
                for s in [first, last]:
                    p = {k: a[k]+(b[k]-a[k])*(s-cursor)/length for k in ['x', 'y']}
                    if not line or math.dist(xy(line[-1]), xy(p)) > 1e-10: line.append(p)
            cursor += length
        points.extend(line[1:] if points else line)
        traversals.append({'edge': eid, 'start_s_m': low, 'end_s_m': high})
        if high == edge['length']: node_ids.append(edge['to'])
        zone_start, distance = distance, distance+high-low
        if edge.get('speed_limit_m_s') is not None:
            zone = {'start_s_m': zone_start, 'end_s_m': distance, 'speed_m_s': min(8., edge['speed_limit_m_s'])}
            if zones and zones[-1]['end_s_m'] == zone_start and zones[-1]['speed_m_s'] == zone['speed_m_s']:
                zones[-1]['end_s_m'] = distance
            else: zones.append(zone)
    return {'edge_ids': [s['edge'] for s in traversals], 'node_ids': node_ids, 'points': points,
            'traversals': traversals, 'zones': zones, 'length_m': distance, 'half_width': width, 'enumerated_states': visits}


def compare(plan, expected, zones=None):
    if expected is None: raise ValueError('producer selected an unreachable route')
    for key in ['edge_ids', 'node_ids']:
        if plan[key] != expected[key]: raise ValueError('selected topology differs from exhaustive search: '+key)
    if abs(plan['distance_m']-expected['length_m']) > 1e-8 or plan['route']['half_width'] != expected['half_width']:
        raise ValueError('wrong clipped route distance/width')
    actual = plan['route']['points']
    if len(actual) != len(expected['points']) or any(math.dist(xy(a), xy(b)) > 1e-8 for a, b in zip(actual, expected['points'])):
        raise ValueError('clipped map geometry differs from independent segment reconstruction')
    if plan.get('traversals'):
        if len(plan['traversals']) != len(expected['traversals']): raise ValueError('missing traversal')
        for a, b in zip(plan['traversals'], expected['traversals']):
            if a['edge'] != b['edge'] or any(abs(a[k]-b[k]) > 1e-8 for k in ['start_s_m', 'end_s_m']):
                raise ValueError('wrong directed traversal bounds')
    if zones is not None:
        if len(zones) != len(expected['zones']): raise ValueError('wrong speed interval count')
        if any(abs(a[k]-b[k]) > 1e-8 for a, b in zip(zones, expected['zones']) for k in a):
            raise ValueError('speed policy did not clip/rebase original edge caps')


def closures(scenario, time):
    closed = scenario['navigation'].get('closed_edges', [])
    for update in scenario.get('navigation_updates', []):
        if update['stamp'] <= time+1e-9: closed = update['closed_edges']
    return closed


def physical(run, scenario, case, config):
    frames, radius = run['frames'], run['vehicle']['radius']
    if len(frames) != run['summary']['steps']: raise ValueError('physical audit requires all integration ticks')
    if config.get('navigation') != scenario['navigation']:
        raise ValueError('sensor replay must retain the original endpoint request and map')
    if {'objects', 'min_clearance_m', 'goal_hold_seconds'} & config.keys(): raise ValueError('truth leaked into operational configuration')
    initial = reference(scenario['navigation'], scenario['navigation'].get('closed_edges', []))
    transitions = run.get('route_history') or [{'time': 0., 'plan': run['navigation']}]
    if transitions[0]['time'] != 0. or any(b['time'] <= a['time'] for a, b in zip(transitions, transitions[1:])):
        raise ValueError('invalid route transition clocks')
    plans = []
    for i, transition in enumerate(transitions):
        expected = reference(scenario['navigation'], closures(scenario, transition['time']))
        compare(transition['plan'], expected)
        if i and max(abs(transition[k]) for k in ['estimated_speed', 'true_speed']) > .05+1e-9:
            raise ValueError('route changed before physical and measured standstill')
        plans.append(expected)
    wanted = int(case in ['handover', 'reopen'])
    if len(transitions)-1 != wanted or run['summary'].get('navigation_switches', 0) != wanted:
        raise ValueError('wrong number of stopped handovers')
    if math.dist(xy(frames[0]['truth']['pose']['position']), xy(initial['points'][0])) > 1e-8:
        raise ValueError('plant started at an unrequested node/position')
    errors, yaws, excess, active, previous, hold_start, hold = [], [], 0., 0, None, None, 0.
    covered = set()
    for frame in frames:
        truth = frame['truth']
        plan = plans[active]
        value, touched = interval_excess(plan['points'], plan['zones'], previous or truth, truth, radius)
        covered.update(touched)
        if active+1 < len(plans) and frame['time'] >= transitions[active+1]['time']-1e-9:
            active += 1
            plan = plans[active]
            covered = set()
            instant, touched = interval_excess(plan['points'], plan['zones'], truth, truth, radius)
            covered.update(touched)
            value = max(value, instant)
        excess = max(excess, value)
        state = frame['navigation']
        if state['active_edges'] != plan['edge_ids'] or state['switches'] != active:
            raise ValueError('active route/policy timing differs from independent reconstruction')
        s = progress(truth['pose']['position'], plan['points'])
        cursor = 0.
        for span in plan['traversals']:
            if span['edge'] in closures(scenario, frame['time']) and s+radius >= cursor:
                raise ValueError('physical vehicle front entered a closed edge')
            cursor += span['end_s_m']-span['start_s_m']
        if case == 'reopen' and 3. <= frame['time'] < 15.:
            if abs(truth['speed']) <= .05:
                hold_start = frame['time'] if hold_start is None else hold_start
                hold = max(hold, frame['time']-hold_start)
            else: hold_start = None
        estimate = frame['estimate']
        errors.append(math.dist(xy(truth['pose']['position']), xy(estimate['pose']['position'])))
        yaws.append(abs((truth['pose']['yaw']-estimate['pose']['yaw']+math.pi) % (2*math.pi)-math.pi))
        previous = truth
    if excess > .15+1e-9: raise ValueError('original physical speed gate failed')
    rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
    # Keep all original numerical gates, but finish the remaining independent
    # checks before publishing a failed quality result. The first frozen auditor
    # stopped here; retaining the metrics must not turn its failures into passes.
    failed_gates = [name for name, value, limit in [
        ('localization_rmse_m', rmse, .3), ('maximum_position_error_m', max(errors), .6),
        ('maximum_yaw_error_rad', max(yaws), .1)] if value > limit]
    if abs(rmse-run['summary']['localization_rmse']) > 1e-8: raise ValueError('incorrect producer localization metric')
    if case == 'reopen' and hold < 2.: raise ValueError('no-route wait lacked two continuous seconds of actual standstill')
    if case == 'blocked-goal':
        if frames[-1]['navigation']['phase'] != 'Blocked' or abs(frames[-1]['truth']['speed']) > .05:
            raise ValueError('closed destination did not remain stopped')
    else:
        if covered != set(range(len(plans[-1]['zones']))): raise ValueError('final clipped speed policy was not physically exercised')
        if not run['summary']['reached_goal'] or math.dist(xy(frames[-1]['truth']['pose']['position']), xy(plans[-1]['points'][-1])) > 3.5:
            raise ValueError('vehicle did not reach the clipped destination')
    clearance = sweep(run)
    if not run['summary']['passed']: raise ValueError('original scenario acceptance failed')
    return {'passed': not failed_gates, 'routing_and_physical_contract_passed': True,
            'failed_gates': failed_gates, 'physical_ticks': len(frames), 'route_switches': wanted,
            'maximum_speed_excess_m_s': excess, 'localization_rmse_m': rmse,
            'maximum_position_error_m': max(errors), 'maximum_yaw_error_rad': max(yaws),
            'reopening_standstill_s': hold, 'independent_clearance': clearance, 'summary': run['summary']}


def run_case(binary, folder, case, seed, plant):
    folder.mkdir(parents=True, exist_ok=False)
    source = REPO/f'scenarios/mid-edge-{case}.json'
    args = ['run', '--scenario', source] if plant == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = {'case': case, 'seed': seed, 'plant': plant, 'passed': False,
           'scenario_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    row['run_exit_code'] = execute(binary, args+['--seed', seed, '--output', folder/'run'], folder/'run-process')
    replay = REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving')
    row['replay_exit_code'] = execute(replay, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        with (folder/'run/sensors.jsonl').open() as stream: config = json.loads(next(stream))['header']['config']
        row.update(physical(json.loads((folder/'run/run.json').read_text()), json.loads(source.read_text()), case, config))
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        row['passed'] &= row['run_exit_code'] == row['replay_exit_code'] == 0 and row['replay']['verified']
        row['passed'] &= row['replay']['ticks'] == row['physical_ticks']
    except Exception as error:
        row['audit_error'] = repr(error)
        row['passed'] = False
    row['storage'] = archive(folder)
    return row


def corpus_map():
    nodes = [{'id': name, 'position': {'x': x, 'y': y}}
             for name, x, y in [('a', 0., 0.), ('b', 6., 8.), ('c', 17., 8.), ('d', 6., 21.)]]
    by_id = {n['id']: n['position'] for n in nodes}
    edges = [{'id': a+b, 'from': a, 'to': b, 'points': [by_id[a], by_id[b]], 'half_width': width, 'speed_limit_m_s': speed}
             for a, b, width, speed in [('a', 'b', 4., 3.), ('b', 'c', 2., 2.), ('b', 'd', 3., 4.),
                                        ('d', 'c', 3., 5.), ('c', 'a', 4., 4.)]]
    edges[0]['points'].insert(1, {'x': 3., 'y': 4.})
    return {'nodes': nodes, 'edges': edges}


def routing_corpus(binary, folder):
    folder.mkdir(parents=True, exist_ok=False)
    at = lambda edge, s: {'edge': edge, 's_m': s}
    requests = [(at('ab', 2.5), at('bc', 4.25)), ('a', at('bc', 4.25)), (at('ab', 7.), 'c'),
                (at('ab', 8.), at('ab', 2.)), (at('ab', 2.), at('ab', 8.)), ('b', at('bc', 4.)),
                (at('bc', 3.), 'a'), ('b', at('ab', 3.))]
    rows = []
    for kind in ['none', 'no', 'only']:
        network = corpus_map()
        if kind != 'none': network['turn_restrictions'] = [{'id': 'rule', 'from_edge': 'ab', 'via_node': 'b',
                                                         'to_edge': 'bc' if kind == 'no' else 'bd', 'kind': kind}]
        source = folder/f'{kind}-map.json'
        source.write_text(json.dumps(network, indent=2)+'\n')
        for bits, (start, goal) in itertools.product(range(32), requests):
            closed = [e['id'] for i, e in enumerate(network['edges']) if bits & (1 << i)]
            nav = {'network': network, 'start': start, 'goal': goal, 'closed_edges': closed}
            expected = reference(nav, closed)
            arguments = ['road-route', '--map', source]
            for label, endpoint in [('start', start), ('goal', goal)]:
                arguments += ['--'+label, endpoint] if isinstance(endpoint, str) else ['--'+label+'-edge', endpoint['edge'], '--'+label+'-s', endpoint['s_m']]
            for edge in closed: arguments += ['--closed-edge', edge]
            prefix = folder/f'query-{len(rows):04d}'
            code = execute(binary, arguments, prefix)
            row = {'rule': kind, 'start': start, 'goal': goal, 'closed_edges': closed, 'exit_code': code,
                   'reachable': expected is not None, 'passed': False}
            try:
                if expected is None:
                    if code != 2: raise ValueError('unreachable query was not rejected')
                else:
                    if code != 0: raise ValueError('reachable query was rejected')
                    report = json.loads(prefix.with_suffix('.stdout').read_text())
                    compare(report['plan'], expected, report['speed_zones'])
                    row['distance_m'] = expected['length_m']
                row['passed'] = True
            except Exception as error: row['audit_error'] = repr(error)
            rows.append(row)
    result = {'passed': all(r['passed'] for r in rows), 'queries': len(rows), 'reachable': sum(r['reachable'] for r in rows), 'results': rows}
    (folder/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--plant', choices=['reference-2d', 'rne-dynamic'], default='reference-2d')
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--routing-corpus-only', action='store_true')
    args = parser.parse_args()
    default = REPO/('target/release/rustdriving' if args.plant == 'reference-2d' else 'integrations/rne/target/release/rustdriving-rne')
    binary = (args.binary or default.with_suffix('.exe' if os.name == 'nt' else '')).resolve()
    if args.routing_corpus_only:
        result = routing_corpus(binary, args.output.resolve())
        print(json.dumps({k: v for k, v in result.items() if k != 'results'}))
        raise SystemExit(0 if result['passed'] else 1)
    rows = []
    for case, seed in itertools.product(CASES, args.seeds):
        rows.append(run_case(binary, args.output.resolve()/case/f'seed-{seed}', case, seed, args.plant))
        report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(case, seed, rows[-1]['passed'], rows[-1].get('audit_error', ''), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__': main()
