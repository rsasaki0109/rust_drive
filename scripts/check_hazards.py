#!/usr/bin/env python3
"""Run real hazard scenarios, verify sensor recomputation, and retain acceptance evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
CASES = {
    'reference': ['occluded-crossing', 'cut-in', 'multiple-blocked', 'opposing-crossings', 'route-direct', 'route-detour', 'route-south'],
    'rne-dynamic': ['occluded-crossing', 'cut-in', 'low-friction', 'low-friction-stop', 'multiple-blocked', 'opposing-crossings', 'route-direct', 'route-detour', 'route-south'],
}
# Fixed regression floors, chosen against the preceding measured fixture results.
# They are simulation test constraints, not a universal safe-distance specification.
CLEARANCE_FLOORS_M = {
    'occluded-crossing': 1.0, 'cut-in': 0.7, 'multiple-blocked': 3.0,
    'opposing-crossings': 0.7, 'low-friction': 0.4, 'low-friction-stop': 4.0,
    'route-direct': 0.5, 'route-detour': 0.5, 'route-south': 0.5,
}
EXPECTED_EDGES = {
    'route-direct': ['approach', 'main', 'east-exit'],
    'route-detour': ['approach', 'detour', 'east-exit'],
    'route-south': ['approach', 'south-branch'],
}


def clearance_regression(summary, case):
    floor = CLEARANCE_FLOORS_M[case]
    value = summary['min_clearance']
    return {'floor_m': floor, 'measured_m': value,
            'passed': math.isfinite(value) and value >= floor}


def check_navigation(run, case):
    """Check selected topology/geometry against fixture expectations independently."""
    nav = run['scenario']['navigation']
    plan = run['navigation']
    edges = {edge['id']: edge for edge in nav['network']['edges']}
    nodes = {node['id']: node['position'] for node in nav['network']['nodes']}
    at = nav['start']
    points = []
    distance = 0.0
    half_width = math.inf
    node_ids = [at]
    for id in plan['edge_ids']:
        edge = edges[id]
        if id in nav['closed_edges'] or edge['from'] != at:
            raise ValueError('selected path violates closure or connectivity')
        geometry = edge['points']
        if (math.dist(xy(geometry[0]), xy(nodes[edge['from']])) > 1e-6
                or math.dist(xy(geometry[-1]), xy(nodes[edge['to']])) > 1e-6):
            raise ValueError('edge geometry disagrees with its map endpoints')
        half_width = min(half_width, edge['half_width'])
        distance += sum(math.dist(xy(a), xy(b)) for a, b in zip(geometry, geometry[1:]))
        points.extend(geometry[1:] if points else geometry)
        at = edge['to']
        node_ids.append(at)
    if (at != nav['goal'] or plan['edge_ids'] != EXPECTED_EDGES[case]
            or plan['node_ids'] != node_ids or points != run['route']['points']
            or points != plan['route']['points'] or abs(distance - plan['distance_m']) > 1e-7
            or abs(distance - run['route']['lengths'][-1]) > 1e-7
            or half_width != run['route']['half_width'] or half_width != plan['route']['half_width']):
        raise ValueError('selected route differs from the expected mapped destination')
    end_distance = math.dist(xy(run['frames'][-1]['truth']['pose']['position']), xy(nodes[nav['goal']]))
    if end_distance > 2.0:
        raise ValueError('vehicle did not stop within 2 m of the mapped destination')
    return {'edge_ids': plan['edge_ids'], 'distance_m': distance,
            'goal_distance_m': end_distance, 'passed': True}


def xy(point):
    return point['x'], point['y']


def invoke(args):
    result = subprocess.run([str(a) for a in args], cwd=ROOT, capture_output=True, text=True)
    return result.returncode, result.stderr[-2000:]


def source_fingerprint():
    files = [ROOT/'Cargo.toml', ROOT/'Cargo.lock', ROOT/'rust-toolchain.toml']
    files += list((ROOT/'crates').glob('*/Cargo.toml')) + list((ROOT/'crates').glob('**/*.rs'))
    files += list((ROOT/'scenarios').glob('*.json'))
    files += [ROOT/'integrations/rne'/p for p in ['Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml', 'rne-revision.txt']]
    files += list((ROOT/'integrations/rne/src').glob('*.rs'))
    digest = hashlib.sha256()
    for file in sorted(set(files)):
        digest.update(str(file.relative_to(ROOT)).encode()+b'\0'+file.read_bytes()+b'\0')
    return digest.hexdigest()


def check_speed_profiles(log):
    """Check distance, speed and time kinematics independently of planner code."""
    active = holds = 0
    max_acceleration = max_deceleration = 0.0
    with log.open() as stream:
        header = json.loads(next(stream))
        limits = header['header']['config'].get('motion_limits') or {}
        forward = limits.get('max_acceleration_m_s2', 2.0)
        braking = limits.get('max_deceleration_m_s2', 2.5)
        for line in stream:
            record = json.loads(line)
            if record.get('kind') != 'tick':
                continue
            expected = record['tick']['expected']
            path = expected['trajectory']['points']
            if expected['trajectory']['mode'] == 'Emergency':
                continue
            if not path or path[0]['time'] != 0.0:
                raise ValueError('trajectory must start at relative time zero')
            if abs(path[0]['speed'] - max(0.0, expected['estimate']['speed'])) > 1e-8:
                raise ValueError('profile initial speed differs from the estimated state')
            active += 1
            for point in path:
                values = [point['time'], point['speed'], *point['position'].values()]
                if not all(math.isfinite(value) for value in values) or point['speed'] < 0:
                    raise ValueError('non-finite or negative profile state')
            for a, b in zip(path, path[1:]):
                duration = b['time'] - a['time']
                if duration <= 0:
                    raise ValueError('profile time must increase')
                distance = math.hypot(b['position']['x'] - a['position']['x'], b['position']['y'] - a['position']['y'])
                integrated_distance = 0.5 * (a['speed'] + b['speed']) * duration
                if abs(distance - integrated_distance) > 1e-7 * max(1.0, distance):
                    raise ValueError('speed integral disagrees with segment distance')
                acceleration = (b['speed'] - a['speed']) / duration
                if acceleration > forward + 1e-7 or acceleration < -braking - 1e-7:
                    raise ValueError('profile exceeds calibrated longitudinal authority')
                max_acceleration = max(max_acceleration, acceleration)
                max_deceleration = max(max_deceleration, -acceleration)
                if distance < 1e-10:
                    if a['speed'] != 0 or b['speed'] != 0:
                        raise ValueError('stationary segment has nonzero speed')
                    holds += 1
    if not active:
        raise ValueError('no active speed profiles checked')
    return {'verified': True, 'active_trajectories': active, 'stationary_holds': holds,
            'max_acceleration_m_s2': max_acceleration,
            'max_deceleration_m_s2': max_deceleration}


def control_metrics(log):
    """Measure actual emitted commands, including emergency transitions."""
    count = emergency = pairs = 0
    squared_acceleration_changes = max_normal_steering_rate = 0.0
    previous = None
    with log.open() as stream:
        for line in stream:
            record = json.loads(line)
            if record.get('kind') != 'tick':
                continue
            current = record['tick']['expected']
            count += 1
            emergency += int(current['emergency'])
            if previous:
                duration = current['time'] - previous['time']
                if duration <= 0:
                    raise ValueError('command clock must increase')
                acceleration_change = (current['command']['acceleration'] - previous['command']['acceleration']) / duration
                squared_acceleration_changes += acceleration_change**2
                pairs += 1
                if not current['emergency'] and not previous['emergency']:
                    rate = abs(current['command']['steering'] - previous['command']['steering']) / duration
                    max_normal_steering_rate = max(max_normal_steering_rate, rate)
            previous = current
    if not count or not pairs:
        raise ValueError('insufficient command samples')
    return {'ticks': count, 'emergency_ticks': emergency, 'emergency_fraction': emergency / count,
            'commanded_acceleration_change_rms_m_s3': math.sqrt(squared_acceleration_changes / pairs),
            'max_normal_commanded_steering_rate_rad_s': max_normal_steering_rate}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['all', *CASES], default='all')
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/hazards')
    args = parser.parse_args()
    if any(seed < 0 or seed > 2**64-1 for seed in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('seeds must be distinct unsigned 64-bit integers')
    args.output.mkdir(parents=True, exist_ok=True)
    report_file = args.output/'report.json'
    report_file.unlink(missing_ok=True)
    backends = list(CASES) if args.backend == 'all' else [args.backend]
    cli = ROOT/'target/release/rustdrive'
    rne = ROOT/'integrations/rne/target/release/rustdrive-rne'
    if not cli.is_file() or ('rne-dynamic' in backends and not rne.is_file()):
        raise SystemExit('Build release binaries first: bash scripts/check-hazards.sh')
    revision = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    if 'rne-dynamic' in backends:
        head = subprocess.run(['git', '-C', str(ROOT.parent/'RobotNativeEngine'), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
        if head != revision:
            raise SystemExit('RNE revision differs from the committed pin; existing checkout is preserved')
    report = {'schema_version': 1, 'source_fingerprint_sha256': source_fingerprint(),
              'rne_revision': revision if 'rne-dynamic' in backends else None,
              'seeds': args.seeds, 'runs': [], 'passed': True}
    for backend in backends:
        for case in CASES[backend]:
            for seed in args.seeds:
                output = args.output/backend/case/f'seed-{seed}'
                output.mkdir(parents=True, exist_ok=True)
                # Remove previous success evidence before invoking a fresh run.
                summary_file = output/'summary.json'
                summary_file.unlink(missing_ok=True)
                replay_file = output/'replay/replay.json'
                replay_file.unlink(missing_ok=True)
                command = [cli, 'run'] if backend == 'reference' else [rne, '--plant', 'dynamic']
                command += ['--scenario', ROOT/'scenarios'/f'{case}.json', '--seed', seed, '--output', output]
                code, error = invoke(command)
                row = {'backend': backend, 'scenario': case, 'seed': seed, 'exit_code': code,
                       'output': str(output), 'passed': False}
                if summary_file.exists():
                    summary = json.loads(summary_file.read_text())
                    row['summary'] = summary
                    code_replay, replay_error = invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                    row['replay_exit_code'] = code_replay
                    if replay_file.exists():
                        row['replay'] = json.loads(replay_file.read_text())
                    ok = (code == 0 and summary['passed'] and summary['collisions'] == 0
                          and summary['road_violations'] == 0 and code_replay == 0
                          and row.get('replay', {}).get('verified') is True
                          and row['replay']['ticks'] == summary['steps'])
                    row['speed_profiles'] = check_speed_profiles(output/'sensors.jsonl')
                    row['control_metrics'] = control_metrics(output/'sensors.jsonl')
                    row['clearance_regression'] = clearance_regression(summary, case)
                    ok &= row['clearance_regression']['passed']
                    ok &= row['control_metrics']['max_normal_commanded_steering_rate_rad_s'] <= 0.7 + 1e-8
                    if backend == 'rne-dynamic' and case == 'low-friction':
                        row['tracking_regression_passed'] = summary['emergency_steps'] <= 20
                        ok &= row['tracking_regression_passed']
                    run = json.loads((output/'run.json').read_text())
                    if case in EXPECTED_EDGES:
                        row['navigation'] = check_navigation(run, case)
                    if run['scenario'].get('dynamics'):
                        frames = run['frames']
                        acceleration = max(abs((b['truth']['speed']-a['truth']['speed'])/(b['time']-a['time'])) for a,b in zip(frames, frames[1:]))
                        row['max_measured_abs_longitudinal_acceleration_m_s2'] = acceleration
                        ok &= acceleration <= run['scenario']['dynamics']['friction_coefficient']*9.81+1e-8
                    row['passed'] = bool(ok)
                    if replay_error:
                        row['replay_stderr'] = replay_error
                if error:
                    row['stderr'] = error
                report['runs'].append(row)
                report['passed'] &= row['passed']
                print(f"{backend:11s} {case:20s} seed {seed:3d}: {'PASS' if row['passed'] else 'FAIL'}", flush=True)
    report_file.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_file}: {len(report["runs"])} runs; passed={report["passed"]}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'hazard check: {error}', file=sys.stderr)
        sys.exit(2)
