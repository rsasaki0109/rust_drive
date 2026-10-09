#!/usr/bin/env python3
"""Actual imported OSM road, measured ground and native cuboid body acceptance.

Known OSM map geometry configures navigation. Authored flat ground and barriers
remain simulator-only; independent XYZ/ray/plane/body oracles score the run.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('osm_ground_checks', ROOT/'scripts/check-ground-scenes.py')
ground = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ground)
hazards, rays, require, sha = ground.hazards, ground.rays, ground.require, ground.sha
CASES = {
    'clear': ('osm-ground-clear', 'german-road-ground-scenario', 'goal'),
    'midbeam': ('osm-ground-midbeam', 'german-road-ground-stop-scenario', 'stop'),
}
DATA_HASHES = {
    'german-road-extract.osm': '280febb5b8b084cd4f29c138f5f65f8b219efdca7070bc7233a85ef972fab2a9',
    'german-road-extract.json': '77189ddb7f66915817d11b83765851709b409713487e5d2b9d8f357f665680c6',
}


def projected(lat, lon):
    def cartesian(lat, lon):
        a, b = math.radians(lat), math.radians(lon)
        e2 = 6.6943799901413165e-3
        radius = 6378137/math.sqrt(1-e2*math.sin(a)**2)
        return (radius*math.cos(a)*math.cos(b), radius*math.cos(a)*math.sin(b),
                radius*(1-e2)*math.sin(a))
    a, b = math.radians(48.136), math.radians(10.0695)
    delta = [p-o for p, o in zip(cartesian(lat, lon), cartesian(48.136, 10.0695))]
    return (-math.sin(b)*delta[0]+math.cos(b)*delta[1],
            -math.sin(a)*math.cos(b)*delta[0]-math.sin(a)*math.sin(b)*delta[1]+math.cos(a)*delta[2])


def map_contract():
    base = ROOT/'maps/osm'
    for name, digest in DATA_HASHES.items():
        require(sha(base/name) == digest, 'pinned real OSM source changed')
    document = json.loads((base/'german-road-extract.json').read_text())
    elements = {(e['type'], e['id']): e for e in document['elements']}
    original = ET.parse(base/'german-road-extract.osm').getroot()
    for element in original:
        if element.tag == 'node':
            item = elements[('node', int(element.attrib['id']))]
            require(item['lat'] == float(element.attrib['lat']) and item['lon'] == float(element.attrib['lon']),
                    'converted JSON changed original OSM node coordinates')
        elif element.tag == 'way':
            item = elements[('way', int(element.attrib['id']))]
            require(item['nodes'] == [int(n.attrib['ref']) for n in element.findall('nd')]
                    and item.get('tags', {}) == {t.attrib['k']: t.attrib['v'] for t in element.findall('tag')},
                    'converted JSON changed original OSM references or tags')
    network = json.loads((base/'german-road-network.json').read_text())
    positions = {i: projected(e['lat'], e['lon']) for (kind, i), e in elements.items() if kind == 'node'}
    require(len(network['nodes']) == 6 and len(network['edges']) == 10, 'real OSM graph topology changed')
    for node in network['nodes']:
        identity = int(node['id'].removeprefix('osm-node-'))
        require(math.dist((node['position']['x'], node['position']['y']), positions[identity]) < 1e-6,
                'graph node differs from independently projected external coordinate')
    for edge in network['edges']:
        identity = int(edge['id'].split('-')[2])
        require(identity in {25216931, 25216933, 275776236, 628913513} and edge['half_width'] == 3.0,
                'imported road identity or explicit width calibration changed')
        way = elements[('way', identity)]['nodes']
        first, last = int(edge['from'].split('-')[-1]), int(edge['to'].split('-')[-1])
        a, b = way.index(first), way.index(last)
        references = way[min(a, b):max(a, b)+1]
        if a > b:
            references.reverse()
        require(len(edge['points']) == len(references)
                and all(math.dist((p['x'], p['y']), positions[n]) < 1e-6 for p, n in zip(edge['points'], references)),
                'directed edge changed, dropped or fabricated original OSM way geometry')
    original_scenario = json.loads((base/'german-road-scenario.json').read_text())
    for _, scenario_name, expected in CASES.values():
        scenario = json.loads((base/f'{scenario_name}.json').read_text())
        require({k: v for k, v in scenario.items() if k not in {'name', 'duration', 'expected'}}
                == {k: v for k, v in original_scenario.items() if k not in {'name', 'duration', 'expected'}},
                'integrated ground/body case altered source route, width or motion calibration')
        require(scenario['duration'] == 100 and scenario['expected'] == expected
                and scenario['navigation']['network'] == network, 'integrated scenario no longer has the stricter 100 s deadline')
    ids = ['osm-way-25216931-0-forward', 'osm-way-25216931-1-forward']
    edges = {e['id']: e for e in network['edges']}
    points = edges[ids[0]]['points'] + edges[ids[1]]['points'][1:]
    return network, points, ids


def check_map_run(run, log, network, points, ids, evidence):
    header, ticks = rays.load_log(log)
    require(run['navigation']['edge_ids'] == ids and run['route']['points'] == points,
            'physical episode took a route different from the imported external road')
    require(run['scenario']['navigation']['network'] == network
            and header['config']['route'] == run['route'] and header['config']['initial_pose']['position'] == points[0],
            'operational resolved-route/initial calibration differs from imported known map')
    # Existing static navigation logs replay the resolved route; no claim of map-search replay.
    forbidden = {'truth', 'objects', 'traffic', 'body_guard', 'acquisitions', 'scene', 'static_cuboids',
                 'ground_cuboids', 'hit_role', 'hit_id', 'center_m', 'half_extents_m', 'native_overlap_samples'}
    def keys(value):
        if isinstance(value, dict):
            return set(value).union(*(keys(v) for v in value.values()))
        if isinstance(value, list):
            return set().union(*(keys(v) for v in value))
        return set()
    require(not (keys(header['config']) & forbidden), 'native scene/map truth leaked into operational configuration')
    for tick in ticks:
        require(not (keys(tick['input']) & forbidden), 'physical object/role truth leaked into delivered sensors')
    route_points = [(p['x'], p['y']) for p in points]
    minimum_margin = math.inf
    samples = evidence['body_guard']['motion_samples']
    for sample in samples:
        position = sample['pose']['position']
        distance = min(ground.scenes.point_segment_distance((position['x'], position['y']), a, b)
                       for a, b in zip(route_points, route_points[1:]))
        # Circumscribed circle contains every yaw of the calibrated body. The
        # distance function is 1-Lipschitz; reserve half a bounded 5 ms motion.
        margin = run['route']['half_width'] - run['vehicle']['radius'] - distance - 12*.005/2
        minimum_margin = min(minimum_margin, margin)
        require(margin >= -1e-8, '200 Hz imported-road body corridor reserve violated')
    return {'source_coordinates_verified': True, 'way_geometry_preserved': True,
            'imported_route_verified': True, 'sensor_only_boundary_verified': True,
            'map_search_replay': False, 'resolved_route_replay': True,
            'body_corridor_motion_samples': len(samples), 'minimum_body_corridor_reserve_m': minimum_margin}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/osm-scenes')
    parser.add_argument('--compact', action='store_true')
    parser.add_argument('--plants', nargs='+', choices=['kinematic', 'dynamic'], default=['kinematic', 'dynamic'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--cases', nargs='+', choices=list(CASES), default=list(CASES))
    args = parser.parse_args()
    require(len(set(args.plants)) == len(args.plants) and len(set(args.seeds)) == len(args.seeds)
            and len(set(args.cases)) == len(args.cases) and all(0 <= n < 2**64 for n in args.seeds),
            'plants/cases/seeds must be distinct with u64 seeds')
    network, points, ids = map_contract()
    cli, native = ROOT/'target/release/rustdrive', ROOT/'integrations/rne/target/release/rustdrive-rne'
    require(cli.is_file() and native.is_file(), 'build locked RustDrive and native release binaries first')
    pin = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    actual = subprocess.run(['git', '-C', ROOT.parent/'RobotNativeEngine', 'rev-parse', 'HEAD'],
                            capture_output=True, text=True, check=True).stdout.strip()
    require(actual == pin, 'RNE checkout differs from the committed pin')
    dependencies = [Path(__file__), ROOT/'scripts/check-ground-scenes.py', ROOT/'scripts/check-lidar-3d.py',
                    ROOT/'scripts/check-native-scenes.py', ROOT/'scripts/check_hazards.py']
    inputs = list((ROOT/'maps/osm').glob('german-road-*.json')) + [ROOT/'maps/osm/german-road-extract.osm']
    inputs += [ROOT/'scenes'/f'{scene}.json' for scene, _, _ in CASES.values()]
    report = {'schema_version': 1, 'source_fingerprint_sha256': hazards.source_fingerprint(),
              'checker_dependency_sha256': {str(p.relative_to(ROOT)): sha(p) for p in dependencies},
              'data_fixture_sha256': {str(p.relative_to(ROOT)): sha(p) for p in inputs},
              'rne_revision': pin, 'license': 'OSM geographic data ODbL 1.0; simulator geometry authored research calibration',
              'attribution': '© OpenStreetMap contributors; maps/osm/SOURCE.md',
              'plants': args.plants, 'seeds': args.seeds, 'cases': args.cases,
              'runs': [], 'passed': False, 'compact': args.compact}
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/'report.json'
    report_path.unlink(missing_ok=True)
    for plant in args.plants:
        for case in args.cases:
            # Stop matrix uses dynamic native dynamics; clear case exercises both native plants.
            if case == 'midbeam' and plant != 'dynamic':
                continue
            scene_name, scenario_name, expected = CASES[case]
            scene_path = ROOT/'scenes'/f'{scene_name}.json'
            scene = json.loads(scene_path.read_text())
            for seed in args.seeds:
                output = args.output/plant/case/f'seed-{seed}'
                output.mkdir(parents=True, exist_ok=True)
                for name in ['run.json', 'summary.json', 'scene.json', 'sensors.jsonl', 'replay/replay.json', 'replay/outputs.jsonl']:
                    (output/name).unlink(missing_ok=True)
                code, stderr = hazards.invoke([native, '--plant', plant, '--scenario', ROOT/'maps/osm'/f'{scenario_name}.json',
                    '--scene', scene_path, '--seed', seed, '--output', output, '--lidar-3d', '--ground-segmentation', '--vehicle-body'])
                run = json.loads((output/'run.json').read_text())
                evidence = json.loads((output/'scene.json').read_text())
                replay_code, replay_error = hazards.invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                replay = json.loads((output/'replay/replay.json').read_text())
                require(code == replay_code == 0 and run['summary']['passed'] and replay['verified']
                        and replay['ticks'] == run['summary']['steps'], 'OSM ground/body physical acceptance or exact replay failed')
                mapping = check_map_run(run, output/'sensors.jsonl', network, points, ids, evidence)
                measured = ground.check_ground(run, scene, evidence, output/'sensors.jsonl')
                require(measured['passed'] and measured['body_geometry_verified'] and measured['rapier_sensor_witnesses_verified']
                        and measured['confident_acquisitions'] == measured['acquisitions']
                        and measured['removed_actual_ground_returns'] > 0 and run['summary']['collisions'] == run['summary']['road_violations'] == 0,
                        'integrated imported road lacks measured ground support or independent body acceptance')
                if expected == 'goal':
                    require(run['summary']['reached_goal'] and run['summary']['max_tracks'] == 0
                            and measured['preserved_points'] == 0, 'clear imported road did not reach goal after removing actual ground')
                else:
                    require(not run['summary']['reached_goal'] and run['summary']['final_speed'] <= .2
                            and measured['height_eligible_static_returns'] > 0 and measured['removed_actual_static_returns'] == 0,
                            'measured obstacle was removed as ground or did not produce a physical stop')
                row = {'plant': plant, 'case': case, 'seed': seed, 'output': str(output), 'exit_code': code,
                       'summary': run['summary'], 'scene_summary': evidence['summary'], 'replay': replay,
                       'imported_map': mapping, 'independent_ground_body': measured,
                       'speed_profiles': hazards.check_speed_profiles(output/'sensors.jsonl'),
                       'motion_predictions': hazards.check_motion_predictions(output/'sensors.jsonl'),
                       'control_metrics': hazards.control_metrics(output/'sensors.jsonl'), 'passed': True,
                       'raw_sha256': {name: sha(output/name) for name in ['run.json', 'scene.json', 'sensors.jsonl']}}
                require(row['control_metrics']['max_normal_commanded_steering_rate_rad_s'] <= .7+1e-8,
                        'imported ground/body run exceeds the normal steering-rate calibration')
                if stderr:
                    row['stderr'] = stderr
                if replay_error:
                    row['replay_stderr'] = replay_error
                if args.compact:
                    row['archive'] = rays.compact_case(output, row, plant == 'dynamic' and seed == 7)
                report['runs'].append(row)
                report_path.write_text(json.dumps(report, indent=2)+'\n')
                print(f'{plant:10s} OSM {case:8s} seed {seed:3d}: PASS; ground removed {measured["removed_actual_ground_returns"]}', flush=True)
    require(report['runs'], 'selected OSM matrix contains no runs')
    require(report['source_fingerprint_sha256'] == hazards.source_fingerprint()
            and report['checker_dependency_sha256'] == {str(p.relative_to(ROOT)): sha(p) for p in dependencies}
            and report['data_fixture_sha256'] == {str(p.relative_to(ROOT)): sha(p) for p in inputs},
            'Rust source, map data, fixtures or independent checker changed during the sweep')
    report.update(passed=True, complete=True, positive_run_count=len(report['runs']))
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_path}: {len(report["runs"])} real OSM + ground + body episodes; passed=True')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'OSM scene check: {error}', file=sys.stderr)
        sys.exit(2)
