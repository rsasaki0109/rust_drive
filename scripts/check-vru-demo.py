#!/usr/bin/env python3
"""Audit actual RNE walking/cycling display-demo recordings and full sensor replay.

Actor IDs describe simulator evidence only. Operational perception is unlabeled
LiDAR; moving obstacles remain upright capsules and conservative planar circles.
Pedestrian/cyclist behavior is prescribed motion, not semantic recognition or
human decision modeling. The research ego body has no contact-force response.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('vru_ground_oracle', ROOT/'scripts/check-ground-scenes.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
SCENARIO = ROOT/'scenarios/native-vru-demo.json'
SCENE = ROOT/'scenes/ground-moving-traffic.json'
SCENE_SHA = 'f1c2e9c9493404647a23cec64a4ccefae911bdcf4a5ce9a6be3513bede515b32'


def supplied_fields(actual, expected):
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(k in actual and supplied_fields(actual[k], v) for k, v in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(supplied_fields(a, b) for a, b in zip(actual, expected))
    return actual == expected


def verify_actors(run, evidence, log):
    acquisitions = evidence['acquisitions']
    require = g.require
    require(evidence['operating_mode'] == 'lidar3d_ground_body', 'VRU demo changed native body/ground mode')
    require(len(acquisitions) == 221, 'VRU demo changed acquisition count/deadline')
    _, ticks = g.rays.load_log(log)
    beams = g.directions(evidence['lidar3d'])
    actor_counts = {str(i): {'measured_returns': 0, 'preserved_returns': 0} for i in range(3)}
    for index, acquisition in enumerate(acquisitions):
        time = acquisition['time']
        actors = {actor['id']: actor for actor in acquisition['objects']}
        require(set(actors) == {0, 1, 2}, 'actual stable car/pedestrian/cyclist IDs missing')
        for identity, expected in [(1, (18., -5. + 1.2 * max(time - 2., 0.))), (2, (8. + 3. * time, 5.))]:
            position = g.scenes.xy(actors[identity]['position'])
            require(math.dist(position, expected) < 1e-7, 'actual VRU motion differs from the authored route-frame schedule')
        require([actors[i]['radius'] for i in range(3)] == [1., .4, .85], 'physical VRU/lead capsule radius changed')
        tick = ticks[index * 2]
        removed, _ = g.check_plane(tick['input']['lidar3d'], tick['expected']['ground'])
        pose = acquisition['pose']
        origin = (*g.scenes.xy(pose['position']), evidence['lidar3d']['mount_height_m'])
        c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
        for point_index, measured in enumerate(acquisition['cloud_3d']['returns']):
            ordinal = measured['ray_index']
            value = acquisition['cloud_3d']['ranges_m'][ordinal]
            dx, dy, dz = beams[ordinal]
            direction = (c * dx - s * dy, s * dx + c * dy, dz)
            candidates = [(distance, actor['id']) for actor in actors.values()
                          if (distance := g.rays.capsule_ray(origin, direction, actor)) is not None]
            nearest, identity = min(candidates, default=(math.inf, None))
            ground = min((distance for box in evidence['scene']['ground_cuboids']
                          if (distance := g.rays.box_ray(origin, direction, box)) is not None), default=math.inf)
            # This stricter subset excludes ambiguous GJK boundary beams already
            # accounted for by the imported independent physical ray oracle.
            if identity is not None and nearest < ground and abs(value - nearest) <= .06:
                actor_counts[str(identity)]['measured_returns'] += 1
                actor_counts[str(identity)]['preserved_returns'] += not removed[point_index]
    require(all(row['measured_returns'] > 0 and row['preserved_returns'] > 0 for row in actor_counts.values()),
            'one actual actor lacks measured and ground-preserved LiDAR evidence')
    samples = evidence['motion_samples']
    cursor = 0
    minimum = {str(i): math.inf for i in range(3)}
    speeds = {0: 2., 1: 1.2, 2: 3.}
    for sample in samples:
        while cursor + 1 < len(acquisitions) and acquisitions[cursor + 1]['time'] <= sample['time'] + 1e-9:
            cursor += 1
        acquisition = acquisitions[cursor]
        elapsed = max(0., sample['time'] - acquisition['time'])
        for actor in acquisition['objects']:
            identity = actor['id']
            # Previous actual acquisition plus a declared actor-speed bound;
            # the extra half-substep reserve covers motion between 200 Hz samples.
            margin = speeds[identity] * elapsed + (12. + speeds[identity]) * .005 / 2
            reserve = math.dist(sample['position'], g.scenes.xy(actor['position'])) - run['vehicle']['radius'] - actor['radius'] - margin
            minimum[str(identity)] = min(minimum[str(identity)], reserve)
    require(all(math.isfinite(value) and value >= 1. for value in minimum.values()),
            'independent continuous circumscribed-circle clearance fell below the unchanged 1 m floor')
    # Confirm that the crossing actor actually traverses the ego corridor during
    # recording, instead of claiming a crossing from a stationary display mesh.
    require(acquisitions[0]['objects'][1]['position']['y'] < -3.
            and acquisitions[-1]['objects'][1]['position']['y'] > 3., 'pedestrian did not cross the actual road')
    crossing = min(acquisitions, key=lambda a: abs(a['time'] - (2. + 5./1.2)))
    pedestrian = next(actor for actor in crossing['objects'] if actor['id'] == 1)
    require(abs(pedestrian['position']['y']) < .1 and crossing['pose']['position']['x'] < 14.,
            'ego did not remain behind the crossing actor with a physical reserve')
    return {'stable_display_ids': {'0': 'lead_car', '1': 'walking_pedestrian', '2': 'cyclist'},
            'motion_scope': 'prescribed walking lateral motion and bicycle longitudinal motion; no human decision model',
            'per_actor_lidar_evidence': actor_counts,
            'minimum_continuous_circle_clearance_bound_m': minimum,
            'pedestrian_crossing': {'time_s': crossing['time'], 'lateral_m': pedestrian['position']['y'],
                                    'ego_x_m': crossing['pose']['position']['x']},
            'body_actor_contact_physics': False, 'semantic_recognition': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/vru-demo')
    parser.add_argument('--verify-existing', type=Path, help='root containing seed-1/seed-7/seed-42 captures')
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 7, 42])
    parser.add_argument('--report', type=Path, default=ROOT/'assets/vru-demo-results.json')
    parser.add_argument('--compact', action='store_true', help='archive/hash/CRC verify all; preserve seed-7 raw for rendering')
    args = parser.parse_args()
    require, sha = g.require, g.rays.sha
    require(sha(SCENE) == SCENE_SHA, 'existing physical ground fixture changed')
    fixture = json.loads(SCENARIO.read_text())
    require(fixture['duration'] == 22 and fixture['expected'] == 'stop'
            and fixture['min_clearance_m'] == 1 and len(fixture['objects']) == 3, 'VRU deadline/physical acceptance changed')
    dependencies = [Path(__file__), ROOT/'scripts/check-ground-scenes.py', ROOT/'scripts/check-lidar-3d.py',
                    ROOT/'scripts/check-native-scenes.py', ROOT/'scripts/check_hazards.py']
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in dependencies}
    fingerprint = g.hazards.source_fingerprint()
    pin = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    revision = subprocess.run(['git', '-C', str(ROOT.parent/'RobotNativeEngine'), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    require(revision == pin, 'native checkout differs from the pinned revision')
    report = {'schema_version': 1, 'passed': False, 'complete': False,
              'source_fingerprint_sha256': fingerprint, 'checker_dependency_sha256': hashes,
              'fixture_sha256': {'scenario': sha(SCENARIO), 'scene': sha(SCENE)}, 'rne_revision': pin,
              'scope': 'authored 22-second actual native body/ground demo with three prescribed circular actors',
              'semantic_recognition': False, 'human_decision_model': False,
              'cases': []}
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/'report.json'
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    cli, native = ROOT/'target/release/rustdrive', ROOT/'integrations/rne/target/release/rustdrive-rne'
    for seed in args.seeds:
        raw = (args.verify_existing or args.output)/f'seed-{seed}'
        if args.verify_existing is None:
            code, error = g.hazards.invoke([native, '--plant', 'dynamic', '--scenario', SCENARIO, '--scene', SCENE,
                '--lidar-3d', '--ground-segmentation', '--vehicle-body', '--seed', seed, '--output', raw])
            require(code == 0, f'native VRU episode failed: {error}')
        require(sum(p.stat().st_size for p in raw.rglob('*') if p.is_file()) <= 256 * 1024 * 1024,
                'VRU raw case exceeds the 256 MiB evidence budget')
        code, error = g.hazards.invoke([cli, 'replay', '--log', raw/'sensors.jsonl', '--output', raw/'replay'])
        require(code == 0, f'whole raw sensor-only replay failed: {error}')
        run = json.load((raw/'run.json').open())
        evidence = json.load((raw/'scene.json').open())
        replay = json.load((raw/'replay/replay.json').open())
        summary = run['summary']
        require(supplied_fields(run['scenario'], fixture), 'serialized actual scenario changed')
        require(summary['passed'] and summary['seed'] == seed and summary['simulated_seconds'] == 22
                and summary['steps'] == 441 and summary['collisions'] == summary['road_violations'] == 0
                and summary['min_clearance'] >= 1 and summary['final_speed'] <= .2
                and summary['progress'] >= 20, 'unchanged VRU physical acceptance failed')
        require(replay['verified'] and replay['ticks'] == 441, 'sensor-only replay omitted a control tick')
        physical = g.check_ground(run, json.loads(SCENE.read_text()), evidence, raw/'sensors.jsonl')
        actors = verify_actors(run, evidence, raw/'sensors.jsonl')
        row = {'seed': seed, 'summary': summary, 'replay': replay, 'physical_xyz_ground_body': physical,
               'actors': actors, 'raw_sha256': {name: sha(raw/name) for name in ['run.json', 'scene.json', 'sensors.jsonl']}}
        if args.compact:
            row['archive'] = g.rays.compact_case(raw, row, seed == 7)
        report['cases'].append(row)
        report_path.write_text(json.dumps(report, indent=2)+'\n')
    require(fingerprint == g.hazards.source_fingerprint() and hashes == {str(p.relative_to(ROOT)): sha(p) for p in dependencies},
            'source/checker changed during the independent proof')
    report.update(passed=True, complete=True, cases_verified=len(report['cases']),
                  replay_ticks_verified=sum(row['replay']['ticks'] for row in report['cases']))
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{args.report}: {len(report["cases"])} actual native VRU episodes; passed=True')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'VRU demo check: {error}', file=sys.stderr)
        sys.exit(2)
