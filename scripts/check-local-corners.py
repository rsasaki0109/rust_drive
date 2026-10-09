#!/usr/bin/env python3
"""Drive unchanged sparse map bends with opt-in local geometry and exact replay.

The German fixture derives from OSM (ODbL 1.0); the second map is authored.
The default reference acceptance covers both corners. The optional native
probe currently fails on the tight German branch; it is not a native positive.
These clear-road tests use the original circular vehicle, not native body mode.
Recorded truth is used only by the independent corridor acceptance oracle.
"""
import argparse
import hashlib
import heapq
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('corner_hazards', ROOT/'scripts/check_hazards.py')
hazards = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hazards)
CASES = ['german-branch', 'authored-detour']
EXPECTED_EDGES = {
    'german-branch': ['osm-way-25216931-0-forward', 'osm-way-25216933-0-reverse'],
    'authored-detour': ['osm-way-30-0-forward', 'osm-way-30-1-forward'],
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xy(point):
    return point['x'], point['y']


def distance(point, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    length2 = dx*dx+dy*dy
    fraction = max(0., min(1., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length2))
    return math.dist(point, (a[0]+fraction*dx, a[1]+fraction*dy))


def original_route(scenario, case):
    """Independently choose the directed shortest open route; retain every point."""
    nav = scenario['navigation']
    closed = set(nav['closed_edges'])
    queue = [(0., nav['start'], [])]
    visited = set()
    edges = {e['id']: e for e in nav['network']['edges']}
    while queue:
        cost, node, chosen = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        if node == nav['goal']:
            break
        for edge in sorted(edges.values(), key=lambda e: e['id']):
            if edge['from'] == node and edge['id'] not in closed:
                length = sum(math.dist(xy(a), xy(b)) for a, b in zip(edge['points'], edge['points'][1:]))
                heapq.heappush(queue, (cost+length, edge['to'], chosen+[edge['id']]))
    else:
        raise ValueError('original map has no open route')
    require(chosen == EXPECTED_EDGES[case], 'fixture shortest route or closure changed')
    points = []
    for identity in chosen:
        for point in edges[identity]['points']:
            if not points or point != points[-1]:
                points.append(point)
    width = min(edges[identity]['half_width'] for identity in chosen)
    return points, chosen, width


def map_contract(scenarios):
    # Reuse the independent ECEF/ENU/XML source checks, not importer output.
    spec = importlib.util.spec_from_file_location('corner_osm_source', ROOT/'scripts/check-osm-scenes.py')
    osm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(osm)
    network, _, _ = osm.map_contract()
    require(scenarios['german-branch']['navigation']['network'] == network,
            'branch graph changed original external coordinates/topology')
    for case, width, deadline in [('german-branch', 3., 180.), ('authored-detour', 5.5, 250.)]:
        scenario = scenarios[case]
        require(scenario['local_route_geometry'] is True and scenario['half_width'] == width
                and scenario['duration'] == deadline and scenario['cruise_speed'] == 2.
                and scenario['objects'] == [] and scenario['expected'] == 'goal'
                and scenario['motion_limits'] == {'max_acceleration_m_s2': 1.,
                    'max_deceleration_m_s2': 2.5, 'max_lateral_acceleration_m_s2': 1.},
                'corner physical calibration or original acceptance deadline changed')
        original_route(scenario, case)


def check_run(run, log, scenario, case, require_goal=True):
    points, edges, width = original_route(scenario, case)
    require(run['navigation']['edge_ids'] == edges and run['route']['points'] == points
            and run['route']['half_width'] == width
            and run['scenario']['navigation'] == scenario['navigation']
            and run['scenario']['duration'] == scenario['duration']
            and run['scenario']['local_route_geometry'] is True,
            'operational episode changed the supplied route, closure, width or deadline')
    require(run['vehicle'] == {'wheelbase': 2.7, 'radius': 1.25, 'max_steer': .55},
            'corner fixture vehicle calibration changed')
    with log.open() as stream:
        header = json.loads(next(stream))['header']
    require(header['config']['route'] == run['route'] and header['config']['local_route_geometry'] is True,
            'replay config differs from original physical route')
    forbidden = {'truth', 'objects', 'traffic', 'scene', 'body_guard', 'static_cuboids', 'ground_cuboids'}
    def keys(value):
        if isinstance(value, dict):
            return set(value).union(*(keys(v) for v in value.values()))
        if isinstance(value, list):
            return set().union(*(keys(v) for v in value))
        return set()
    require(not (keys(header['config']) & forbidden), 'simulator truth leaked into operational configuration')
    with log.open() as stream:
        next(stream)
        for line in stream:
            value = json.loads(line)
            if value['kind'] == 'tick':
                require(not (keys(value['tick']['input']) & forbidden), 'simulator truth leaked into delivered sensors')
    segments = list(zip(map(xy, points), map(xy, points[1:])))
    margins = [width-run['vehicle']['radius']-min(distance(xy(frame['truth']['pose']['position']), a, b)
               for a, b in segments) for frame in run['frames']]
    goal_distance = math.dist(xy(points[-1]), xy(run['frames'][-1]['truth']['pose']['position']))
    require(min(margins) >= -1e-8 and (not require_goal or goal_distance <= 2.),
            'original corridor or 2 m goal acceptance failed')
    # Runs retain sampled physical telemetry; the simulator additionally scores every physics tick.
    return {'original_route_points_preserved': True, 'original_directed_edges': edges,
            'original_half_width_m': width, 'physical_samples': len(margins),
            'minimum_sampled_original_corridor_margin_m': min(margins), 'goal_distance_m': goal_distance,
            'sensor_only_boundary_verified': True, 'body_mode': False,
            'resolved_route_replay': True, 'map_search_replay': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['all', 'reference', 'rne-dynamic'], default='reference')
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 7, 42])
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/local-corners')
    parser.add_argument('--compact', action='store_true', help='archive verified successful raw evidence')
    parser.add_argument('--check-known-native-limit', action='store_true',
                        help='also verify the retained safe native German-branch deadlock at seed 1')
    args = parser.parse_args()
    require(len(set(args.seeds)) == len(args.seeds) and 0 < len(args.seeds) <= 3
            and all(0 <= n < 2**64 for n in args.seeds), 'use one to three distinct u64 seeds')
    backends = ['reference', 'rne-dynamic'] if args.backend == 'all' else [args.backend]
    suffix = '.exe' if sys.platform == 'win32' else ''
    cli = ROOT/'target/release'/('rustdrive'+suffix)
    native = ROOT/'integrations/rne/target/release'/('rustdrive-rne'+suffix)
    require(cli.is_file() and (not (args.check_known_native_limit or 'rne-dynamic' in backends) or native.is_file()),
            'build locked reference and selected native release binaries first')
    scenario_paths = {case: ROOT/'scenarios'/f'local-corners-{case}.json' for case in CASES}
    scenarios = {case: json.loads(path.read_text()) for case, path in scenario_paths.items()}
    map_contract(scenarios)
    dependencies = [Path(__file__), ROOT/'scripts/check_hazards.py', ROOT/'scripts/check-osm-scenes.py',
        ROOT/'scripts/check-ground-scenes.py', ROOT/'scripts/check-lidar-3d.py', ROOT/'scripts/check-native-scenes.py']
    fixtures = list(scenario_paths.values()) + list((ROOT/'maps/osm').glob('german-road-*.json'))
    fixtures += [ROOT/'maps/osm/german-road-extract.osm', ROOT/'maps/osm/SOURCE.md']
    hashes = lambda paths: {str(p.relative_to(ROOT)): sha(p) for p in sorted(paths)}
    report = {'schema_version': 1, 'source_fingerprint_sha256': hazards.source_fingerprint(),
        'checker_dependency_sha256': hashes(dependencies), 'data_fixture_sha256': hashes(fixtures),
        'rne_revision': (ROOT/'integrations/rne/rne-revision.txt').read_text().strip(),
        'attribution': 'German road: © OpenStreetMap contributors, ODbL 1.0; maps/osm/SOURCE.md. Detour: authored test map.',
        'backends': backends, 'seeds': args.seeds, 'runs': [], 'known_native_limits': [], 'passed': False}
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/'report.json'
    report_path.unlink(missing_ok=True)
    for backend in backends:
        for case in CASES:
            for seed in args.seeds:
                output = args.output/backend/case/f'seed-{seed}'
                output.mkdir(parents=True, exist_ok=True)
                for name in ['run.json', 'summary.json', 'sensors.jsonl', 'replay/replay.json', 'replay/outputs.jsonl']:
                    (output/name).unlink(missing_ok=True)
                command = [cli, 'run'] if backend == 'reference' else [native, '--plant', 'dynamic']
                code, stderr = hazards.invoke(command+['--scenario', scenario_paths[case], '--seed', seed, '--output', output])
                run = json.loads((output/'run.json').read_text())
                row = {'backend': backend, 'case': case, 'seed': seed, 'output': str(output),
                    'exit_code': code, 'summary': run['summary'], 'passed': False}
                report['runs'].append(row)
                report_path.write_text(json.dumps(report, indent=2)+'\n')
                require(code == 0 and run['summary']['passed'] and run['summary']['reached_goal']
                        and run['summary']['collisions'] == run['summary']['road_violations'] == 0,
                        f'{backend}/{case}/{seed} physical driving failed')
                replay_code, replay_error = hazards.invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                replay = json.loads((output/'replay/replay.json').read_text())
                require(replay_code == 0 and replay['verified'] and replay['ticks'] == run['summary']['steps'],
                        'full sensor-only exact replay failed')
                row.update(replay=replay, independent_corridor=check_run(run, output/'sensors.jsonl', scenarios[case], case),
                    speed_profiles=hazards.check_speed_profiles(output/'sensors.jsonl'),
                    control_metrics=hazards.control_metrics(output/'sensors.jsonl'),
                    gnss_integrity=hazards.check_gnss_innovation_hold(output/'sensors.jsonl'),
                    motion_predictions=hazards.check_motion_predictions(output/'sensors.jsonl'))
                require(row['control_metrics']['max_normal_commanded_steering_rate_rad_s'] <= .7+1e-8,
                        'steering rate calibration exceeded')
                row.update(passed=True, raw_sha256={name: sha(output/name) for name in ['run.json', 'sensors.jsonl']})
                if stderr or replay_error:
                    row['diagnostics'] = {'run': stderr, 'replay': replay_error}
                if args.compact:
                    spec = importlib.util.spec_from_file_location('corner_archive', ROOT/'scripts/check-lidar-3d.py')
                    archive = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(archive)
                    row['archive'] = archive.compact_case(output, row, False)
                report_path.write_text(json.dumps(report, indent=2)+'\n')
                print(f'{backend:11s} {case:15s} seed {seed:3d}: PASS; {run["summary"]["simulated_seconds"]:.2f} s', flush=True)
    if args.check_known_native_limit:
        output = args.output/'known-native-limit/german-branch/seed-1'
        output.mkdir(parents=True, exist_ok=True)
        for name in ['run.json', 'summary.json', 'sensors.jsonl', 'replay/replay.json', 'replay/outputs.jsonl']:
            (output/name).unlink(missing_ok=True)
        code, stderr = hazards.invoke([native, '--plant', 'dynamic', '--scenario', scenario_paths['german-branch'],
                                       '--seed', 1, '--output', output])
        run = json.loads((output/'run.json').read_text())
        summary = run['summary']
        require(code == 1 and not summary['passed'] and not summary['reached_goal']
                and summary['collisions'] == summary['road_violations'] == 0
                and summary['simulated_seconds'] == scenarios['german-branch']['duration']
                and summary['final_speed'] <= .2 and summary['emergency_steps'] > summary['steps']/2
                and run['frames'][-1]['emergency'], 'known native limit no longer matches the safe physical deadlock')
        replay_code, replay_error = hazards.invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
        replay = json.loads((output/'replay/replay.json').read_text())
        require(replay_code == 0 and replay['verified'] and replay['ticks'] == summary['steps'],
                'known-limit full sensor replay failed')
        row = {'backend': 'rne-dynamic', 'case': 'german-branch', 'seed': 1, 'output': str(output),
            'exit_code': code, 'summary': summary, 'expected_physical_failure': True,
            'replay': replay, 'independent_corridor': check_run(run, output/'sensors.jsonl',
                scenarios['german-branch'], 'german-branch', require_goal=False),
            'speed_profiles': hazards.check_speed_profiles(output/'sensors.jsonl'),
            'control_metrics': hazards.control_metrics(output/'sensors.jsonl'),
            'motion_predictions': hazards.check_motion_predictions(output/'sensors.jsonl'),
            'raw_sha256': {name: sha(output/name) for name in ['run.json', 'sensors.jsonl']},
            'known_limit_verified': True}
        if stderr or replay_error:
            row['diagnostics'] = {'run': stderr, 'replay': replay_error}
        report['known_native_limits'].append(row)
        report_path.write_text(json.dumps(report, indent=2)+'\n')
        print('rne-dynamic German branch seed 1: known safe deadlock retained; exact replay verified', flush=True)
    require(report['source_fingerprint_sha256'] == hazards.source_fingerprint()
            and report['checker_dependency_sha256'] == hashes(dependencies)
            and report['data_fixture_sha256'] == hashes(fixtures), 'source/checker/map changed during corner sweep')
    report.update(passed=True, complete=True, positive_run_count=len(report['runs']))
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_path}: {len(report["runs"])} physical corner runs and exact replays passed')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'local corner check: {error}', file=sys.stderr)
        sys.exit(2)
