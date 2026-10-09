#!/usr/bin/env python3
"""Independently reconstruct real inclined LiDAR rays and retained blind zones.

No renderer or driving algorithm supplies the ray oracle. XYZ returns remain
measured data; height selection produces a bounded planar driving input.
"""
import argparse
import copy
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

ROOT = Path(__file__).resolve().parent.parent


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


scenes = module('native_scene_checks', ROOT/'scripts/check-native-scenes.py')
hazards = scenes.hazards
require = scenes.require
MODES = {
    'lidar3d': [
        ('midbeam-barrier', 'native-scene-mid-stop', False),
        ('ground-barrier', 'native-scene-ground-stop', False),
        ('rotated-barrier', 'native-scene-ground-stop', False),
        ('raised-barrier', 'native-scene-raised-goal', False),
        ('blind-low-slab', 'native-scene-3d-low-stop', False),
        ('sub-low-slab', 'native-scene-3d-low-stop', False),
        ('near-high-barrier', 'native-scene-3d-fov-blind', True),
    ],
    'multi-height': [('midbeam-barrier', 'native-scene-mid-blind', True)],
}
COLUMNS, RINGS = 720, 16
MIN_ELEVATION, MAX_ELEVATION = -math.pi/12, math.pi/12
RANGE_TOLERANCE_M = 0.06


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        while chunk := stream.read(1024*1024):
            digest.update(chunk)
    return digest.hexdigest()


def calibration(radius):
    return {'azimuth_columns': COLUMNS, 'elevation_rings': RINGS,
            'min_elevation_rad': MIN_ELEVATION, 'max_elevation_rad': MAX_ELEVATION,
            'mount_height_m': 0.6, 'min_range_m': 0.2, 'max_range_m': 45.0,
            'collision_bottom_m': 0.1-radius, 'collision_top_m': 1.1+radius}


def beam_direction(index):
    azimuth = -math.pi+2*math.pi*(index//RINGS)/(COLUMNS-1)
    elevation = MIN_ELEVATION+(MAX_ELEVATION-MIN_ELEVATION)*(index % RINGS)/(RINGS-1)
    return math.cos(elevation)*math.cos(azimuth), -math.cos(elevation)*math.sin(azimuth), math.sin(elevation)


BEAMS = [beam_direction(index) for index in range(COLUMNS*RINGS)]


def box_ray(origin, direction, box):
    """Three independent slabs in the upright box's local ENU coordinates."""
    local_xy = scenes.local(origin, box)
    c, s = math.cos(box['yaw_rad']), math.sin(box['yaw_rad'])
    start = (*local_xy, origin[2]-box['center_m'][2])
    vector = (c*direction[0]+s*direction[1], -s*direction[0]+c*direction[1], direction[2])
    lower, upper = 0.0, math.inf
    for p, d, extent in zip(start, vector, box['half_extents_m']):
        if abs(d) < 1e-14:
            if abs(p) > extent:
                return None
        else:
            first, second = (-extent-p)/d, (extent-p)/d
            lower, upper = max(lower, min(first, second)), min(upper, max(first, second))
    return lower if upper >= lower else None


def capsule_ray(origin, direction, obj):
    """Analytic finite cylinder and two hemispheres; no Rapier calls."""
    offset = (origin[0]-obj['position']['x'], origin[1]-obj['position']['y'], origin[2])
    radius = obj['radius']
    gap = max(0.1-offset[2], offset[2]-1.1, 0.0)
    if offset[0]**2+offset[1]**2+gap**2 <= radius**2:
        return 0.0
    candidates = []
    horizontal = direction[0]**2+direction[1]**2
    linear = 2*(offset[0]*direction[0]+offset[1]*direction[1])
    constant = offset[0]**2+offset[1]**2-radius**2
    discriminant = linear**2-4*horizontal*constant
    if horizontal > 1e-14 and discriminant >= 0:
        for distance in [(-linear-math.sqrt(discriminant))/(2*horizontal),
                         (-linear+math.sqrt(discriminant))/(2*horizontal)]:
            height = offset[2]+distance*direction[2]
            if distance >= 0 and 0.1-1e-12 <= height <= 1.1+1e-12:
                candidates.append(distance)
    for center, lower in [(0.1, True), (1.1, False)]:
        delta = (*offset[:2], offset[2]-center)
        along = sum(a*b for a, b in zip(delta, direction))
        discriminant = along**2-(sum(value*value for value in delta)-radius**2)
        if discriminant >= 0:
            for distance in [-along-math.sqrt(discriminant), -along+math.sqrt(discriminant)]:
                height = offset[2]+distance*direction[2]
                hemisphere = height <= center+1e-12 if lower else height >= center-1e-12
                if distance >= 0 and hemisphere:
                    candidates.append(distance)
    return min(candidates, default=None)


def load_log(log):
    with log.open() as stream:
        header = json.loads(next(stream))['header']
        ticks = [record['tick'] for line in stream if (record := json.loads(line))['kind'] == 'tick']
    return header, ticks


def check_guard(run, scene, evidence, ticks):
    require(evidence['schema_version'] == 1 and evidence['scene'] == scene,
            'native scene differs from authored physical geometry')
    require(evidence['backend'] == run['backend'] and evidence['seed'] == run['summary']['seed']
            and evidence['scenario'] == run['scenario']['name'], 'scene belongs to a different native run')
    require(evidence['ego_capsule'] == {'axis_bottom_m': 0.1, 'axis_top_m': 1.1,
            'radius_m': run['vehicle']['radius'], 'speed_bound_m_s': 12.0, 'clearance_floor_m': 1.0},
            'native capsule geometry or fixed one-meter floor changed')
    require(len(ticks) == run['summary']['steps'], 'sensor count differs from physical run')
    samples = evidence['motion_samples']
    require(len(samples) == (len(ticks)-1)*10+1, 'native guard omits 200 Hz body translations')
    require(abs(samples[0]['time']) < 1e-10 and abs(samples[-1]['time']-run['summary']['simulated_seconds']) < 1e-7,
            'motion evidence does not cover complete run')
    minimum, overlaps, first_overlap = math.inf, 0, None
    for a, b in zip(samples, samples[1:]):
        dt = b['time']-a['time']
        require(abs(dt-0.005) < 1e-8 and math.dist(a['position'], b['position'])/dt <= 12.0+1e-8,
                'native translation interval or speed bound changed')
        overlap = False
        for box in scene['static_cuboids']:
            planar = scenes.segment_rectangle_distance(a['position'], b['position'], box)
            separation = math.hypot(max(planar-12*dt/2, 0.0), scenes.vertical_gap(box))-run['vehicle']['radius']
            minimum = min(minimum, max(separation, 0.0))
            overlap |= separation <= 0
        if overlap:
            overlaps += 1
            first_overlap = first_overlap if first_overlap is not None else b['time']
    observations = evidence['observations']
    require(len(observations) == len(ticks), 'native observation pose history is incomplete')
    for index, observation in enumerate(observations):
        require(abs(observation['time']-ticks[index]['input']['time']) < 1e-8
                and math.dist(scenes.xy(observation['pose']['position']), samples[index*10]['position']) < 1e-7,
                'ray observation pose differs from native translation clock')
    for frame in run['frames']:
        observation = observations[round(frame['time']/0.05)]
        require(math.dist(scenes.xy(observation['pose']['position']), scenes.xy(frame['truth']['pose']['position'])) < 1e-7
                and abs(observation['pose']['yaw']-frame['truth']['pose']['yaw']) < 1e-8,
                'scene pose history differs from recorded driving truth')
    summary = evidence['summary']
    require(summary['checks'] == (2*len(samples)-1)*len(scene['static_cuboids']), 'guard check count changed')
    require(abs(summary['min_clearance_m']-minimum) < 1e-7 and summary['guard_overlap_intervals'] == overlaps,
            'capsule/box guard differs from independent segment-edge oracle')
    passed = minimum >= 1.0 and overlaps == 0
    require(summary['passed'] == passed, 'native scene pass differs from fixed independent floor')
    return {'passed': passed, 'min_clearance_m': minimum, 'guard_overlap_intervals': overlaps,
            'first_guard_overlap_time_s': first_overlap, 'motion_samples': len(samples)}


def check_3d(run, scene, evidence, log):
    header, ticks = load_log(log)
    expected_config = calibration(run['vehicle']['radius'])
    require(evidence.get('operating_mode') == 'lidar3d' and evidence.get('lidar3d') == expected_config
            and header['config'].get('lidar3d') == expected_config,
            'inclined-ray sensor calibration differs between physics and replay')
    require(header['config'].get('multi_height_lidar') is None
            and header['config']['vehicle'] == run['vehicle'], 'operational LiDAR modes or vehicle calibration mixed')
    forbidden = ['scene', 'static_cuboids', 'center_m', 'half_extents_m', 'motion_samples', 'acquisitions', 'ego_capsule']
    require(not any(f'"{field}"' in json.dumps(header['config']) for field in forbidden),
            'physical scene labels entered operational configuration')
    result = check_guard(run, scene, evidence, ticks)
    acquisitions = evidence['acquisitions']
    require(len(acquisitions) == (len(ticks)+1)//2, 'physical ray acquisition clocks missing')
    ray_checks, return_count, eligible, voxels, error_max = 0, 0, 0, 0, 0.0
    first_return, pre_guard_returns, initial_returns = None, 0, 0
    diagnostics = [0, 0, 0]
    for acquisition_index, acquisition in enumerate(acquisitions):
        tick_index = acquisition_index*2
        tick = ticks[tick_index]
        require(abs(acquisition['time']-tick['input']['time']) < 1e-8
                and acquisition['pose'] == evidence['observations'][tick_index]['pose'],
                'inclined ray pose/time differs from acquisition truth')
        require(tick['input'].get('lidar') is None and tick['input'].get('multi_height_lidar') is None,
                'inclined-ray input also carries a legacy planar scan')
        scan = tick['input'].get('lidar3d')
        require(scan is not None and abs(scan['stamp']-acquisition['time']) < 1e-8,
                'typed XYZ acquisition stamp differs from actual firing clock')
        cloud = acquisition['cloud_3d']
        require(len(cloud['ranges_m']) == COLUMNS*RINGS and cloud['returns'] == scan['returns'],
                'operational XYZ returns differ from raw native cloud or complete firing ordinals')
        pose = acquisition['pose']
        origin = (*scenes.xy(pose['position']), 0.6)
        c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
        nonnull = []
        for index, value in enumerate(cloud['ranges_m']):
            dx, dy, dz = BEAMS[index]
            world_direction = (c*dx-s*dy, s*dx+c*dy, dz)
            candidates = [distance for box in scene['static_cuboids']
                          if (distance := box_ray(origin, world_direction, box)) is not None]
            candidates += [distance for obj in acquisition['objects']
                           if (distance := capsule_ray(origin, world_direction, obj)) is not None]
            nearest = min(candidates, default=math.inf)
            exists = 0.2 <= nearest <= 45.0
            require((value is not None) == exists,
                    f'3D nearest-return presence mismatch at t={acquisition["time"]}, ray={index}')
            ray_checks += 1
            if value is not None:
                require(math.isfinite(value) and 0.2-1e-9 <= value <= 45+1e-9,
                        '3D measured range is nonfinite or outside calibration')
                error_max = max(error_max, abs(value-nearest))
                require(abs(value-nearest) <= RANGE_TOLERANCE_M,
                        '3D range differs from independent nearest physical intersection')
                nonnull.append((index, value))
        require(len(scan['returns']) == len(nonnull), 'typed XYZ return count differs from measured firing grid')
        cells = set()
        for measured, (index, value) in zip(scan['returns'], nonnull):
            require(measured['ray_index'] == index, 'XYZ ray ordinal differs from native firing grid')
            point = measured['point']
            dx, dy, dz = BEAMS[index]
            expected = (value*dx, value*dy, 0.6+value*dz)
            actual = (point['x'], point['y'], point['z'])
            require(all(math.isfinite(v) for v in actual) and math.dist(actual, expected) < 1e-7,
                    'typed body XYZ point differs from real inclined return geometry')
            if expected_config['collision_bottom_m'] <= point['z'] <= expected_config['collision_top_m']:
                eligible += 1
                cells.add((math.floor(point['x']/0.05), math.floor(point['y']/0.05)))
        voxels += len(cells)
        returns_now = len(nonnull)
        return_count += returns_now
        if acquisition_index == 0:
            initial_returns = returns_now
        if returns_now and first_return is None:
            first_return = acquisition['time']
        if result['first_guard_overlap_time_s'] is not None and acquisition['time'] < result['first_guard_overlap_time_s']:
            pre_guard_returns += returns_now
        require([channel['height_m'] for channel in acquisition['channels']] == scenes.HEIGHTS,
                'legacy diagnostic height metadata changed in 3D mode')
        for channel_index, channel in enumerate(acquisition['channels']):
            require(len(channel['ranges_m']) == COLUMNS, 'legacy diagnostic firing grid incomplete')
            diagnostics[channel_index] += sum(value is not None for value in channel['ranges_m'])
        require(not any(field in tick['input'] for field in forbidden), 'scene truth entered XYZ sensor input')
    for index, tick in enumerate(ticks):
        if index % 2:
            require(all(tick['input'].get(key) is None for key in ['lidar', 'multi_height_lidar', 'lidar3d']),
                    'driver invents a LiDAR acquisition between native firing clocks')
    result.update({'acquisitions': len(acquisitions), 'ray_grid_entries_verified': ray_checks,
                   'xyz_returns_verified': return_count, 'height_eligible_returns': eligible,
                   'expected_5cm_voxel_returns': voxels, 'maximum_range_residual_m': error_max,
                   'range_tolerance_m': RANGE_TOLERANCE_M, 'initial_xyz_returns': initial_returns,
                   'first_xyz_return_time_s': first_return, 'xyz_returns_before_first_guard_overlap': pre_guard_returns,
                   'legacy_diagnostic_hits_by_plane': dict(zip(map(str, scenes.HEIGHTS), diagnostics)),
                   'typed_xyz_correspondence_verified': True, 'sensor_only_boundary_verified': True})
    return result


def mutations_3d(run, scene, evidence, log, cli):
    records = [json.loads(line) for line in log.read_text().splitlines()]
    first = next(index for index, record in enumerate(records)
                 if record.get('kind') == 'tick' and record['tick']['input'].get('lidar3d') is not None)
    nonempty = next((index for index, record in enumerate(records)
                     if record.get('kind') == 'tick' and (record['tick']['input'].get('lidar3d') or {}).get('returns')), first)
    changed = log.parent/'mutation-sensors.jsonl'
    replay_output = log.parent/'mutation-replay'
    results = []
    try:
        for name in ['changed-point-z', 'duplicate-ray-index', 'changed-ray-index', 'omitted-return',
                     'inconsistent-stamp', 'changed-ring-calibration', 'omitted-acquisition']:
            altered = copy.deepcopy(records)
            scan = altered[nonempty]['tick']['input']['lidar3d']
            if name == 'changed-point-z':
                if scan['returns']:
                    scan['returns'][0]['point']['z'] += 0.25
                else:
                    scan['returns'].append({'ray_index': 0, 'point': {'x': -1, 'y': 0, 'z': 0.85}})
            elif name == 'duplicate-ray-index':
                require(scan['returns'], 'duplicate-index probe requires measured return')
                scan['returns'].append(copy.deepcopy(scan['returns'][0]))
            elif name == 'changed-ray-index':
                require(scan['returns'], 'ray-index probe requires measured return')
                scan['returns'][0]['ray_index'] = (scan['returns'][0]['ray_index']+1) % (COLUMNS*RINGS)
            elif name == 'omitted-return':
                require(scan['returns'], 'omission probe requires measured return')
                scan['returns'].pop(0)
            elif name == 'inconsistent-stamp':
                altered[first]['tick']['input']['lidar3d']['stamp'] += 0.05
            elif name == 'changed-ring-calibration':
                altered[0]['header']['config']['lidar3d']['min_elevation_rad'] += 0.01
            else:
                altered[first]['tick']['input']['lidar3d'] = None
            changed.write_text(''.join(json.dumps(record, separators=(',', ':'))+'\n' for record in altered))
            try:
                check_3d(run, scene, evidence, changed)
            except ValueError as error:
                reason = str(error)
            else:
                raise ValueError(f'XYZ measurement oracle accepted mutation {name}')
            (replay_output/'replay.json').unlink(missing_ok=True)
            code, stderr = hazards.invoke([cli, 'replay', '--log', changed, '--output', replay_output])
            require(name == 'omitted-return' or code != 0,
                    f'Rust replay accepted incompatible XYZ mutation {name}')
            results.append({'mutation': name, 'rejected': True, 'oracle_reason': reason,
                            'replay_exit_code': code, 'replay_rejected': code != 0,
                            'mutated_log_sha256': sha(changed), 'replay_stderr': stderr[:1000]})
    finally:
        changed.unlink(missing_ok=True)
        for name in ['replay.json', 'outputs.jsonl']:
            (replay_output/name).unlink(missing_ok=True)
        if replay_output.exists():
            replay_output.rmdir()
    # Sidecar corruption does not enter replay, but must fail the independent oracle.
    altered = copy.deepcopy(evidence)
    cloud = next(a['cloud_3d'] for a in altered['acquisitions'] if a['cloud_3d']['returns'])
    ordinal = cloud['returns'][0]['ray_index']
    cloud['ranges_m'][ordinal] += 0.25
    try:
        check_3d(run, scene, altered, log)
    except ValueError as error:
        results.append({'mutation': 'changed-native-range', 'rejected': True, 'oracle_reason': str(error)})
    else:
        raise ValueError('independent oracle accepted changed native range')
    return results


def compact_case(output, row, preserve):
    """Archive only this generated case after replay/mutations; verify before deletion."""
    archive = output.with_name(output.name+'.tar.gz')
    archive.unlink(missing_ok=True)
    files = {str(path.relative_to(output)): path for path in output.rglob('*') if path.is_file()}
    require(not any(path.is_symlink() for path in output.rglob('*')), 'generated evidence contains an unexpected symlink')
    expected = {name: sha(path) for name, path in files.items()}
    require(all(expected[name] == digest for name, digest in row['raw_sha256'].items()),
            'raw evidence changed before compaction')
    with tarfile.open(archive, 'w:gz', compresslevel=6) as bundle:
        bundle.add(output, arcname=output.name)
    verified = {}
    with tarfile.open(archive, 'r:gz') as bundle:
        for member in bundle:
            if member.isfile():
                digest = hashlib.sha256()
                with bundle.extractfile(member) as stream:
                    while chunk := stream.read(1024*1024):
                        digest.update(chunk)
                verified[str(Path(member.name).relative_to(output.name))] = digest.hexdigest()
    require(verified == expected, 'compacted evidence differs from original full file hashes')
    with gzip.open(archive, 'rb') as stream:
        while stream.read(1024*1024):
            pass
    require({name: sha(path) for name, path in files.items()} == expected, 'raw files changed while archive was verified')
    result = {'path': str(archive), 'sha256': sha(archive), 'bytes': archive.stat().st_size,
              'files_sha256': expected, 'gzip_crc_verified': True, 'raw_preserved': preserve}
    manifest = archive.with_suffix('.manifest.json')
    manifest.write_text(json.dumps(result, indent=2)+'\n')
    if not preserve:
        shutil.rmtree(output)
    return result


def fixture_contracts():
    mid = json.loads((ROOT/'scenes/midbeam-barrier.json').read_text())['static_cuboids'][0]
    lower, upper = mid['center_m'][2]-mid['half_extents_m'][2], mid['center_m'][2]+mid['half_extents_m'][2]
    require(all(not lower <= height <= upper for height in scenes.HEIGHTS)
            and scenes.vertical_gap(mid) < 1.25, 'midbeam case no longer lies in the legacy sensing blind zone')
    blind = json.loads((ROOT/'scenarios/native-scene-mid-blind.json').read_text())
    stop = json.loads((ROOT/'scenarios/native-scene-mid-stop.json').read_text())
    require({k: v for k, v in blind.items() if k not in ['name', 'expected']}
            == {k: v for k, v in stop.items() if k not in ['name', 'expected']},
            'midbeam pair changes physical timing or driving configuration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/lidar-3d')
    parser.add_argument('--compact', action='store_true', help='verify case archives and retain only accepted dynamic midbeam seed 7 raw files')
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--plants', nargs='+', choices=['kinematic', 'dynamic'], default=['dynamic', 'kinematic'])
    parser.add_argument('--modes', nargs='+', choices=list(MODES), default=list(MODES))
    parser.add_argument('--cases', nargs='+', choices=sorted({case for entries in MODES.values() for case, _, _ in entries}))
    args = parser.parse_args()
    require(args.seeds and len(set(args.seeds)) == len(args.seeds) and all(0 <= seed < 2**64 for seed in args.seeds),
            'seeds must be distinct u64 values')
    require(len(set(args.plants)) == len(args.plants) and len(set(args.modes)) == len(args.modes), 'plants and modes must be distinct')
    fixture_contracts()
    cli, native = ROOT/'target/release/rustdrive', ROOT/'integrations/rne/target/release/rustdrive-rne'
    require(cli.is_file() and native.is_file(), 'build locked reference and native release binaries first')
    pin = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    head = subprocess.run(['git', '-C', ROOT.parent/'RobotNativeEngine', 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    require(head == pin, 'native engine differs from the pinned revision')
    args.output.mkdir(parents=True, exist_ok=True)
    report_file = args.output/'report.json'
    report_file.unlink(missing_ok=True)
    dependencies = [Path(__file__), ROOT/'scripts/check-native-scenes.py', ROOT/'scripts/check_hazards.py']
    scene_paths = sorted({ROOT/'scenes'/f'{case}.json' for entries in MODES.values() for case, _, _ in entries})
    report = {'schema_version': 1, 'source_fingerprint_sha256': hazards.source_fingerprint(),
              'checker_sha256': sha(Path(__file__)), 'checker_dependency_sha256': {str(path.relative_to(ROOT)): sha(path) for path in dependencies},
              'scene_inputs_sha256': {str(path.relative_to(ROOT)): sha(path) for path in scene_paths},
              'rne_revision': pin, 'seeds': args.seeds, 'plants': args.plants, 'modes': args.modes,
              'compact': args.compact, 'runs': [], 'known_failures': [], 'passed': False}
    for plant in args.plants:
        for mode in args.modes:
            for case, scenario_name, negative in MODES[mode]:
                if args.cases and case not in args.cases:
                    continue
                scene_path = ROOT/'scenes'/f'{case}.json'
                scene = json.loads(scene_path.read_text())
                for seed in args.seeds:
                    output = args.output/mode/plant/case/f'seed-{seed}'
                    output.mkdir(parents=True, exist_ok=True)
                    for name in ['run.json', 'scene.json', 'summary.json', 'sensors.jsonl', 'replay/replay.json']:
                        (output/name).unlink(missing_ok=True)
                    command = [native, '--scene', scene_path, '--scenario', ROOT/'scenarios'/f'{scenario_name}.json',
                               '--plant', plant, '--seed', seed, '--output', output,
                               '--lidar-3d' if mode == 'lidar3d' else '--multi-height']
                    code, stderr = hazards.invoke(command)
                    run = json.loads((output/'run.json').read_text())
                    evidence = json.loads((output/'scene.json').read_text())
                    replay_code, replay_stderr = hazards.invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                    replay = json.loads((output/'replay/replay.json').read_text())
                    require(replay_code == 0 and replay['verified'] and replay['ticks'] == run['summary']['steps'],
                            'inclined-ray or legacy baseline does not exactly replay every driving tick')
                    measured = check_3d(run, scene, evidence, output/'sensors.jsonl') if mode == 'lidar3d' else scenes.check_scene(run, scene, evidence, output/'sensors.jsonl', True)
                    profiles = hazards.check_speed_profiles(output/'sensors.jsonl')
                    forecasts = hazards.check_motion_predictions(output/'sensors.jsonl')
                    controls = hazards.control_metrics(output/'sensors.jsonl')
                    require(controls['max_normal_commanded_steering_rate_rad_s'] <= 0.7+1e-8,
                            'normal commanded steering exceeds calibrated rate')
                    require(run['summary']['collisions'] == 0 and run['summary']['road_violations'] == 0,
                            'existing planar road/object acceptance regressed')
                    require(code == (1 if negative else 0), f'{mode}/{case}: unexpected CLI exit {code}')
                    if negative:
                        require(not measured['passed'] and measured['guard_overlap_intervals'] > 0
                                and run['summary']['reached_goal'] and not run['summary']['passed']
                                and any('native scene conservative capsule guard' in failure for failure in run['summary']['failures']),
                                'authored blind-zone case did not preserve a physically rejected driving run')
                        if mode == 'lidar3d':
                            require(measured['initial_xyz_returns'] == 0
                                    and measured['xyz_returns_before_first_guard_overlap'] == 0,
                                    'near-high fixture no longer exercises absence of measurements before physical overlap')
                        else:
                            require(all(value == 0 for value in measured['static_hits_by_plane'].values()),
                                    'legacy midbeam barrier is visible to a calibrated horizontal plane')
                    else:
                        require(measured['passed'] and run['summary']['passed'], 'positive 3D sensing case failed the fixed physical floor')
                        if case == 'raised-barrier':
                            require(run['summary']['reached_goal'] and run['summary']['max_tracks'] == 0
                                    and measured['height_eligible_returns'] == 0 and measured['xyz_returns_verified'] > 0,
                                    'overhead returns did not exercise measured-height exclusion and passage')
                        else:
                            require(not run['summary']['reached_goal'] and run['summary']['final_speed'] <= 0.2
                                    and run['summary']['max_tracks'] > 0 and measured['height_eligible_returns'] > 0,
                                    'height-eligible inclined returns did not cause an operational stop')
                        if case in ['sub-low-slab', 'midbeam-barrier']:
                            require(all(value == 0 for value in measured['legacy_diagnostic_hits_by_plane'].values()),
                                    'new inclined-ray case no longer isolates a horizontal-plane blind zone')
                    row = {'mode': mode, 'plant': plant, 'scene': case, 'scenario': scenario_name, 'seed': seed,
                           'output': str(output), 'exit_code': code, 'summary': run['summary'], 'scene_summary': evidence['summary'],
                           'independent_scene': measured, 'replay': replay, 'speed_profiles': profiles,
                           'motion_predictions': forecasts, 'control_metrics': controls, 'passed': True,
                           'raw_sha256': {name: sha(output/name) for name in ['run.json', 'scene.json', 'sensors.jsonl']}}
                    if stderr:
                        row['stderr'] = stderr
                    if replay_stderr:
                        row['replay_stderr'] = replay_stderr
                    if seed == args.seeds[0]:
                        if mode == 'lidar3d':
                            row['mutation_rejections'] = mutations_3d(run, scene, evidence, output/'sensors.jsonl', cli)
                        else:
                            row['mutation_rejections'] = scenes.mutation_checks(run, scene, evidence, output/'sensors.jsonl', True)
                            row['layered_mutation_rejections'] = scenes.layered_mutation_checks(run, scene, evidence, output/'sensors.jsonl', cli)
                    if negative:
                        row['acceptance_rejection_verified'] = True
                        report['known_failures'].append(row)
                    else:
                        report['runs'].append(row)
                    if args.compact:
                        preserve = mode == 'lidar3d' and plant == 'dynamic' and case == 'midbeam-barrier' and seed == 7
                        row['archive'] = compact_case(output, row, preserve)
                    report_file.write_text(json.dumps(report, indent=2)+'\n')
                    print(f'{mode:12s} {plant:10s} {case:18s} seed {seed:3d}: {"KNOWN FAILURE REJECTED" if negative else "PASS"}; guard clearance {measured["min_clearance_m"]:.6f} m', flush=True)
    require(report['runs'] or report['known_failures'], 'selected matrix contains no physical runs')
    require(report['source_fingerprint_sha256'] == hazards.source_fingerprint()
            and report['checker_dependency_sha256'] == {str(path.relative_to(ROOT)): sha(path) for path in dependencies}
            and report['scene_inputs_sha256'] == {str(path.relative_to(ROOT)): sha(path) for path in scene_paths},
            'source, checker or scene geometry changed during physical acceptance')
    report['complete'] = True
    report['passed'] = True
    report_file.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_file}: {len(report["runs"])} positive runs; {len(report["known_failures"])} retained rejections; passed=True')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'3D LiDAR check: {error}', file=sys.stderr)
        sys.exit(2)
