#!/usr/bin/env python3
"""Independent physical-scene acceptance; diagnostics never enter driving inputs.

The ray oracle uses a slab intersection, rather than Rapier. The clearance oracle
uses distance to rectangle edges, rather than the native adapter's minimizer.
Both inspect the same recorded simulator evidence and sensor-only replay.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('hazards', ROOT/'scripts/check_hazards.py')
hazards = importlib.util.module_from_spec(spec)
spec.loader.exec_module(hazards)
CASES = [
    ('ground-barrier', 'native-scene-ground-stop', False),
    ('raised-barrier', 'native-scene-raised-goal', False),
    ('rotated-barrier', 'native-scene-ground-stop', False),
    ('blind-low-slab', 'native-scene-blind-low', True),
]
HEIGHTS = [0.6, 0.15, 3.7]
RAY_COUNT = 720
RANGE_TOLERANCE_M = 0.06  # 7.5 times the configured 0.008 m range-noise sigma.


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xy(value):
    return value['x'], value['y']


def local(point, box):
    x, y = point[0]-box['center_m'][0], point[1]-box['center_m'][1]
    c, s = math.cos(box['yaw_rad']), math.sin(box['yaw_rad'])
    return c*x+s*y, -s*x+c*y


def slab_ray(origin, direction, box, height):
    if not box['center_m'][2]-box['half_extents_m'][2] <= height <= box['center_m'][2]+box['half_extents_m'][2]:
        return None
    start = local(origin, box)
    c, s = math.cos(box['yaw_rad']), math.sin(box['yaw_rad'])
    vector = (c*direction[0]+s*direction[1], -s*direction[0]+c*direction[1])
    lower, upper = 0.0, math.inf
    for p, d, half in zip(start, vector, box['half_extents_m'][:2]):
        if abs(d) < 1e-14:
            if abs(p) > half:
                return None
        else:
            t1, t2 = (-half-p)/d, (half-p)/d
            lower, upper = max(lower, min(t1, t2)), min(upper, max(t1, t2))
    return lower if upper >= lower else None


def capsule_ray(origin, direction, obj, height):
    # RNE traffic/query capsules share the vertical center .6 and half-axis .5.
    gap = max(0.1-height, height-1.1, 0.0)
    squared = obj['radius']**2-gap**2
    if squared <= 0:
        return None
    delta = (obj['position']['x']-origin[0], obj['position']['y']-origin[1])
    along = delta[0]*direction[0]+delta[1]*direction[1]
    discriminant = squared-(delta[0]**2+delta[1]**2-along**2)
    if discriminant < 0:
        return None
    near, far = along-math.sqrt(discriminant), along+math.sqrt(discriminant)
    if far < 0:
        return None
    return max(0.0, near)


def point_segment_distance(point, a, b):
    vx, vy = b[0]-a[0], b[1]-a[1]
    norm = vx*vx+vy*vy
    fraction = min(1.0, max(0.0, ((point[0]-a[0])*vx+(point[1]-a[1])*vy)/norm)) if norm else 0.0
    return math.hypot(point[0]-a[0]-fraction*vx, point[1]-a[1]-fraction*vy)


def intersect_segments(a, b, c, d):
    def cross(p, q, r):
        return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    signs = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
    if signs[0]*signs[1] > 0 or signs[2]*signs[3] > 0:
        return False
    return (max(min(a[0], b[0]), min(c[0], d[0])) <= min(max(a[0], b[0]), max(c[0], d[0]))+1e-12
            and max(min(a[1], b[1]), min(c[1], d[1])) <= min(max(a[1], b[1]), max(c[1], d[1]))+1e-12)


def segment_rectangle_distance(a, b, box):
    a, b = local(a, box), local(b, box)
    hx, hy = box['half_extents_m'][:2]
    if any(abs(p[0]) <= hx and abs(p[1]) <= hy for p in (a, b)):
        return 0.0
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    best = math.inf
    for c, d in zip(corners, corners[1:]+corners[:1]):
        if intersect_segments(a, b, c, d):
            return 0.0
        best = min(best, point_segment_distance(a, c, d), point_segment_distance(b, c, d),
                   point_segment_distance(c, a, b), point_segment_distance(d, a, b))
    return best


def vertical_gap(box):
    bottom = box['center_m'][2]-box['half_extents_m'][2]
    top = box['center_m'][2]+box['half_extents_m'][2]
    return max(bottom-1.1, 0.1-top, 0.0)


def check_scene(run, scene, evidence, log):
    require(evidence['schema_version'] == 1 and evidence['scene'] == scene,
            'physical scene evidence differs from the authored input')
    require(evidence['backend'] == run['backend'] and evidence['seed'] == run['summary']['seed']
            and evidence['scenario'] == run['scenario']['name'], 'scene evidence names a different physical run')
    capsule = evidence['ego_capsule']
    require(capsule == {'axis_bottom_m': 0.1, 'axis_top_m': 1.1,
                       'radius_m': run['vehicle']['radius'], 'speed_bound_m_s': 12.0,
                       'clearance_floor_m': 1.0}, 'capsule calibration or fixed floor changed')
    with log.open() as stream:
        header = json.loads(next(stream))['header']
        ticks = [record['tick'] for line in stream if (record := json.loads(line))['kind'] == 'tick']
    require(len(ticks) == run['summary']['steps'], 'sensor-log count differs from physical run')
    require(header['config']['vehicle'] == run['vehicle'], 'sensor replay vehicle differs from query scene')
    forbidden = ['scene', 'static_cuboids', 'center_m', 'half_extents_m', 'motion_samples', 'acquisitions', 'ego_capsule']
    config_text = json.dumps(header['config'])
    require(not any(f'"{key}"' in config_text for key in forbidden), 'simulator geometry leaked into driver header')
    samples = evidence['motion_samples']
    require(len(samples) == (len(ticks)-1)*10+1, 'native scene evidence omits 200 Hz motion samples')
    require(abs(samples[0]['time']) < 1e-10 and abs(samples[-1]['time']-run['summary']['simulated_seconds']) < 1e-7,
            'native motion evidence does not cover the entire run')
    minimum, overlap = math.inf, 0
    for first, second in zip(samples, samples[1:]):
        dt = second['time']-first['time']
        require(abs(dt-0.005) < 1e-8, 'native sample interval differs from ten substeps per 20 Hz tick')
        a, b = first['position'], second['position']
        require(all(math.isfinite(v) for v in [*a, *b, first['time'], second['time']]), 'nonfinite native motion')
        require(math.dist(a, b)/dt <= 12.0+1e-8, 'native motion exceeds its declared speed bound')
        guard = False
        for box in scene['static_cuboids']:
            distance = segment_rectangle_distance(a, b, box)
            # The pinned native integrator translates along these 5ms chords.
            # Reconstruct its extra horizontal guard reserve; this does not
            # assert continuous real chassis/tire motion or contact impulses.
            conservative = math.hypot(max(distance-12.0*dt/2, 0.0), vertical_gap(box))-run['vehicle']['radius']
            minimum = min(minimum, max(conservative, 0.0))
            guard |= conservative <= 0.0
        overlap += guard
    for frame in run['frames']:
        sample = samples[round(frame['time']/0.005)]
        require(math.dist(sample['position'], xy(frame['truth']['pose']['position'])) < 1e-7,
                'native motion trace differs from recorded vehicle truth')
    observations = evidence['observations']
    require(len(observations) == len(ticks), 'scene evidence omits 20 Hz acquisition poses')
    for index, observation in enumerate(observations):
        require(abs(observation['time']-ticks[index]['input']['time']) < 1e-8
                and math.dist(xy(observation['pose']['position']), samples[index*10]['position']) < 1e-7,
                'native observation poses differ from motion samples or sensor clocks')
    for frame in run['frames']:
        observation = observations[round(frame['time']/0.05)]
        require(abs(observation['pose']['yaw']-frame['truth']['pose']['yaw']) < 1e-8,
                'scene orientation evidence differs from recorded truth')
    summary = evidence['summary']
    require(summary['checks'] == (2*len(samples)-1)*len(scene['static_cuboids']), 'native scene check count is incomplete')
    require(abs(summary['min_clearance_m']-minimum) < 1e-7, 'native capsule/box clearance differs from edge-distance oracle')
    require(summary['guard_overlap_intervals'] == overlap, 'native conservative overlap count differs from oracle')
    geometry_passed = minimum >= 1.0 and overlap == 0
    require(summary['passed'] == geometry_passed, 'native scene acceptance differs from independent fixed floor')
    acquisitions = evidence['acquisitions']
    require(len(acquisitions) == (len(ticks)+1)//2, 'scene evidence misses native acquisition clocks')
    count, static_hits, maximum_error = 0, [0, 0, 0], 0.0
    for acquisition_index, acquisition in enumerate(acquisitions):
        tick_index = acquisition_index*2
        tick = ticks[tick_index]
        require(abs(acquisition['time']-tick['input']['time']) < 1e-8, 'diagnostic acquisition clock differs from driving clock')
        motion = samples[tick_index*10]
        pose = acquisition['pose']
        require(math.dist(xy(pose['position']), motion['position']) < 1e-7, 'ray mount differs from acquisition truth')
        require(pose == observations[tick_index]['pose'], 'ray mount orientation differs from its native observation')
        require([channel['height_m'] for channel in acquisition['channels']] == HEIGHTS,
                'diagnostic planes or operational single-plane height changed')
        for channel_index, channel in enumerate(acquisition['channels']):
            ranges = channel['ranges_m']
            require(len(ranges) == RAY_COUNT, 'ray ordinal array must contain every native azimuth column')
            returned = []
            for ordinal, value in enumerate(ranges):
                angle = -math.pi+2*math.pi*ordinal/(RAY_COUNT-1)
                bearing = pose['yaw']-angle
                direction = math.cos(bearing), math.sin(bearing)
                candidates = [(distance, True) for box in scene['static_cuboids']
                              if (distance := slab_ray(xy(pose['position']), direction, box, channel['height_m'])) is not None]
                candidates += [(distance, False) for obj in acquisition['objects']
                               if (distance := capsule_ray(xy(pose['position']), direction, obj, channel['height_m'])) is not None]
                nearest = min(candidates, default=(math.inf, False))
                expected_return = 0.2 <= nearest[0] <= 45.0
                require((value is not None) == expected_return,
                        f'nearest physical return mismatch at t={acquisition["time"]}, plane={channel["height_m"]}, ray={ordinal}')
                if value is not None:
                    require(math.isfinite(value) and 0.2-1e-9 <= value <= 45.0+1e-9, 'nonfinite or out-of-range ray return')
                    error = abs(value-nearest[0])
                    require(error <= RANGE_TOLERANCE_M, 'ray range differs from nearest independent physical intersection')
                    maximum_error = max(maximum_error, error)
                    count += 1
                    static_hits[channel_index] += nearest[1]
                    returned.append((value*math.cos(angle), -value*math.sin(angle)))
            if channel_index == 0:
                scan = tick['input']['lidar']
                require(scan is not None and abs(scan['stamp']-acquisition['time']) < 1e-8,
                        'actual driving sensor does not contain the primary acquisition')
                require(len(scan['points']) == len(returned), 'driving cloud size differs from primary physical ray returns')
                require(all(math.dist(xy(point), reconstructed) < 1e-7 for point, reconstructed in zip(scan['points'], returned)),
                        'diagnostic primary ranges differ from the actual body-frame driving cloud')
        require(not any(key in tick['input'] for key in forbidden), 'simulator diagnostic geometry entered sensor input')
    require(count > 0, 'no physical scene rays were reconstructed')
    return {'passed': geometry_passed, 'min_clearance_m': minimum,
            'guard_overlap_intervals': overlap, 'motion_samples': len(samples),
            'acquisitions': len(acquisitions), 'ray_returns_reconstructed': count,
            'static_hits_by_plane': dict(zip(map(str, HEIGHTS), static_hits)),
            'maximum_range_residual_m': maximum_error, 'range_tolerance_m': RANGE_TOLERANCE_M,
            'full_sensor_cloud_verified': True, 'sensor_only_boundary_verified': True}


def mutation_checks(run, scene, evidence, log):
    mutations = []
    removed = copy.deepcopy(evidence)
    removed['scene']['static_cuboids'] = []
    mutations.append(('removed-collider', removed))
    raised = copy.deepcopy(evidence)
    raised['scene']['static_cuboids'][0]['center_m'][2] += 10
    mutations.append(('changed-height', raised))
    forged = copy.deepcopy(evidence)
    forged['summary']['min_clearance_m'] += 1
    mutations.append(('forged-clearance', forged))
    ray = copy.deepcopy(evidence)
    for acquisition in ray['acquisitions']:
        channel = next((c for c in acquisition['channels'] if any(v is not None for v in c['ranges_m'])), None)
        if channel:
            index = next(i for i, value in enumerate(channel['ranges_m']) if value is not None)
            channel['ranges_m'][index] += 0.25
            break
    mutations.append(('changed-ray-return', ray))
    results = []
    for name, altered in mutations:
        try:
            # Change the oracle's authored geometry too, so geometry mutations
            # must fail physical reconstruction rather than an identity check.
            authored = altered['scene'] if name in ['removed-collider', 'changed-height'] else scene
            check_scene(run, authored, altered, log)
        except ValueError as error:
            results.append({'mutation': name, 'rejected': True, 'reason': str(error)})
        else:
            raise ValueError(f'independent scene check accepted mutation {name}')
    return results


def fixture_contracts():
    ground = json.loads((ROOT/'scenes/ground-barrier.json').read_text())['static_cuboids'][0]
    raised = json.loads((ROOT/'scenes/raised-barrier.json').read_text())['static_cuboids'][0]
    require({k: v for k, v in ground.items() if k != 'center_m'} == {k: v for k, v in raised.items() if k != 'center_m'}
            and ground['center_m'][:2] == raised['center_m'][:2], 'paired scene geometry must differ only in height')
    require(ground['center_m'][2]+ground['half_extents_m'][2] >= 0.6
            and raised['center_m'][2]-raised['half_extents_m'][2] > 2.35,
            'paired scenes no longer straddle sensor and vehicle height')
    low = json.loads((ROOT/'scenes/blind-low-slab.json').read_text())['static_cuboids'][0]
    require(low['center_m'][2]-low['half_extents_m'][2] < 0.15 < low['center_m'][2]+low['half_extents_m'][2] < 0.6,
            'retained blind-low scene no longer lies below the operational scan')
    ground_scenario = json.loads((ROOT/'scenarios/native-scene-ground-stop.json').read_text())
    raised_scenario = json.loads((ROOT/'scenarios/native-scene-raised-goal.json').read_text())
    require({k: v for k, v in ground_scenario.items() if k not in ['name', 'expected']}
            == {k: v for k, v in raised_scenario.items() if k not in ['name', 'expected']},
            'paired driving fixtures must differ only in their expected outcome')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/native-scenes')
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--plants', nargs='+', choices=['kinematic', 'dynamic'], default=['kinematic', 'dynamic'])
    args = parser.parse_args()
    require(args.seeds and len(set(args.seeds)) == len(args.seeds)
            and all(0 <= seed < 2**64 for seed in args.seeds), 'seeds must be distinct u64 values')
    require(len(set(args.plants)) == len(args.plants), 'plants must be distinct')
    cli, native = ROOT/'target/release/rustdrive', ROOT/'integrations/rne/target/release/rustdrive-rne'
    require(cli.is_file() and native.is_file(), 'build the locked reference and native release binaries first')
    fixture_contracts()
    args.output.mkdir(parents=True, exist_ok=True)
    report_file = args.output/'report.json'
    report_file.unlink(missing_ok=True)
    revision = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    actual = subprocess.run(['git', '-C', ROOT.parent/'RobotNativeEngine', 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    require(actual == revision, 'RNE checkout differs from the committed pin')
    report = {'schema_version': 1, 'source_fingerprint_sha256': hazards.source_fingerprint(),
              'checker_sha256': sha(Path(__file__)), 'rne_revision': revision,
              'seeds': args.seeds, 'plants': args.plants, 'runs': [], 'known_failures': [],
              'scene_inputs_sha256': {str(p.relative_to(ROOT)): sha(p) for p in sorted((ROOT/'scenes').glob('*.json'))},
              'passed': False}
    for plant in args.plants:
        for case, scenario_name, negative in CASES:
            scene_path = ROOT/'scenes'/f'{case}.json'
            scene = json.loads(scene_path.read_text())
            for seed in args.seeds:
                output = args.output/plant/case/f'seed-{seed}'
                output.mkdir(parents=True, exist_ok=True)
                for name in ['run.json', 'summary.json', 'scene.json', 'sensors.jsonl', 'replay/replay.json']:
                    (output/name).unlink(missing_ok=True)
                command = [native, '--plant', plant, '--scene', scene_path,
                           '--scenario', ROOT/'scenarios'/f'{scenario_name}.json', '--seed', seed, '--output', output]
                code, error = hazards.invoke(command)
                run = json.loads((output/'run.json').read_text())
                evidence = json.loads((output/'scene.json').read_text())
                replay_code, replay_error = hazards.invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                replay = json.loads((output/'replay/replay.json').read_text())
                geometry = check_scene(run, scene, evidence, output/'sensors.jsonl')
                require(replay_code == 0 and replay['verified'] and replay['ticks'] == run['summary']['steps'],
                        'scene run does not reproduce every sensor-only pipeline tick')
                profiles = hazards.check_speed_profiles(output/'sensors.jsonl')
                forecasts = hazards.check_motion_predictions(output/'sensors.jsonl')
                controls = hazards.control_metrics(output/'sensors.jsonl')
                require(controls['max_normal_commanded_steering_rate_rad_s'] <= 0.7+1e-8,
                        'scene run exceeds normal commanded steering rate')
                require(run['summary']['road_violations'] == 0 and run['summary']['collisions'] == 0,
                        'planar road/object acceptance regressed')
                expected_code = 1 if negative else 0
                require(code == expected_code, f'{case}: CLI exit {code} differs from expected {expected_code}')
                if negative:
                    require(not geometry['passed'] and geometry['guard_overlap_intervals'] > 0
                            and geometry['static_hits_by_plane']['0.6'] == 0
                            and geometry['static_hits_by_plane']['0.15'] > 0
                            and run['summary']['reached_goal'] and not run['summary']['passed']
                            and any('native scene conservative capsule guard' in failure for failure in run['summary']['failures']),
                            'low slab no longer demonstrates rejected physical overlap beneath operational LiDAR')
                else:
                    require(geometry['passed'] and run['summary']['passed'], 'positive physical scene acceptance failed')
                    if case == 'raised-barrier':
                        require(run['summary']['reached_goal'] and geometry['static_hits_by_plane']['0.6'] == 0
                                and geometry['static_hits_by_plane']['3.7'] > 0,
                                'raised scene did not exercise height-sensitive physical visibility and passage')
                    else:
                        require(not run['summary']['reached_goal'] and run['summary']['final_speed'] <= 0.2
                                and geometry['static_hits_by_plane']['0.6'] > 0,
                                'ground scene did not cause a sensed operational stop')
                row = {'plant': plant, 'scene': case, 'scenario': scenario_name, 'seed': seed,
                       'exit_code': code, 'output': str(output), 'summary': run['summary'],
                       'scene_summary': evidence['summary'], 'independent_scene': geometry,
                       'replay': replay, 'speed_profiles': profiles, 'motion_predictions': forecasts,
                       'control_metrics': controls, 'passed': True,
                       'raw_sha256': {name: sha(output/name) for name in ['run.json', 'sensors.jsonl', 'scene.json']}}
                if error:
                    row['stderr'] = error
                if replay_error:
                    row['replay_stderr'] = replay_error
                if seed == args.seeds[0]:
                    row['mutation_rejections'] = mutation_checks(run, scene, evidence, output/'sensors.jsonl')
                if negative:
                    row['acceptance_rejection_verified'] = True
                    report['known_failures'].append(row)
                else:
                    report['runs'].append(row)
                report_file.write_text(json.dumps(report, indent=2)+'\n')
                print(f'{plant:10s} {case:18s} seed {seed:3d}: {"KNOWN FAILURE REJECTED" if negative else "PASS"}; capsule clearance {geometry["min_clearance_m"]:.6f}m', flush=True)
    require(report['source_fingerprint_sha256'] == hazards.source_fingerprint(), 'source changed during native scene sweep')
    require(report['checker_sha256'] == sha(Path(__file__))
            and report['scene_inputs_sha256'] == {str(p.relative_to(ROOT)): sha(p) for p in sorted((ROOT/'scenes').glob('*.json'))},
            'checker or authored geometry changed during native scene sweep')
    report['complete'] = True
    report['passed'] = True
    report_file.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_file}: {len(report["runs"])} positive runs; {len(report["known_failures"])} retained rejections; passed=True')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'native-scene check: {error}', file=sys.stderr)
        sys.exit(2)
