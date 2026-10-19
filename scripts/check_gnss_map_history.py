#!/usr/bin/env python3
"""Audit GNSS/map event ordering with a full-history joint-matrix EKF."""
import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil

from check_gnss_latency import Filter, add, angle, product, transpose
from check_lane_change_speeds import sweep
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
CASES = ['recovery', 'prolonged', 'lidar-loss', 'no-overlap', 'disabled']


def inverse(matrix):
    """Pivoted Gauss-Jordan elimination, independent of producer Cholesky."""
    n = len(matrix)
    rows = [list(row)+[float(i == j) for j in range(n)] for i, row in enumerate(matrix)]
    for column in range(n):
        pivot = max(range(column, n), key=lambda i: abs(rows[i][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        scale = rows[column][column]
        if abs(scale) < 1e-18:
            raise ValueError('singular innovation covariance')
        rows[column] = [v/scale for v in rows[column]]
        for i in range(n):
            if i != column:
                scale = rows[i][column]
                rows[i] = [a-scale*b for a, b in zip(rows[i], rows[column])]
    return [row[n:] for row in rows]


class MapFilter(Filter):
    def map_correct(self, pose, covariance):
        innovation = [pose['position']['x']-self.x[0], pose['position']['y']-self.x[1],
                      angle(pose['yaw']-self.x[2])]
        inv = inverse(add(self.p, covariance))
        nis = sum(innovation[i]*inv[i][j]*innovation[j] for i in range(3) for j in range(3))
        if nis > 36.:
            return False
        gain = product(self.p, inv)
        residual = [[float(i == j)-gain[i][j] for j in range(3)] for i in range(3)]
        self.p = add(product(product(residual, self.p), transpose(residual)),
                     product(product(gain, covariance), transpose(gain)))
        self.x = [self.x[i]+sum(gain[i][j]*innovation[j] for j in range(3)) for i in range(3)]
        self.x[2] = angle(self.x[2])
        return True


def chronological(config, motions, fixes, maps):
    """Rebuild the complete past; producer history snapshots are never read.

    Map poses/covariances are registration outputs, not an independent scan
    registration implementation. Raw scan fitting is checked by exact replay.
    Only originally admitted map measurements may reenter this transaction.
    """
    events = sorted([(t, 0, fix, None) for t, fix in fixes]
                    + [(t, 1, pose, covariance) for t, pose, covariance in maps], key=lambda e: (e[0], e[1]))
    result, index = MapFilter(config), 0

    def apply(event):
        _, kind, measurement, covariance = event
        if kind == 0:
            result.correct(measurement)
        elif not result.map_correct(measurement, covariance):
            raise ValueError('GNSS conflicts with an originally accepted map measurement')

    while index < len(events) and events[index][0] <= 1e-9:
        apply(events[index])
        index += 1
    for start, end, motion in motions:
        cursor = start
        while index < len(events) and events[index][0] <= end+1e-9:
            stamp = events[index][0]
            if stamp < start-1e-9:
                raise ValueError('observation outside measured motion coverage')
            result.predict(motion, max(0., stamp-cursor))
            apply(events[index])
            cursor, index = stamp, index+1
        result.predict(motion, max(0., end-cursor))
    if index != len(events):
        raise ValueError('future observation entered full-history reconstruction')
    return result


def verify_ticks(config, ticks):
    if not config.get('gnss_latency_compensation'):
        raise ValueError('this protocol requires acquisition-time fusion')
    motions, fixes, maps = [], [], []
    previous, wheel = None, None
    oracle, maximum = MapFilter(config), 0.
    compensated = replayed_maps = covered_ticks = 0
    for tick in ticks:
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        wheel = inp.get('odometry') or wheel
        if previous is not None:
            if wheel is None or now-wheel['stamp'] > .15+1e-9 or now-previous > .25+1e-9:
                raise ValueError('physical protocol requires continuous healthy measured odometry')
            motions.append((previous, now, wheel))
            oracle.predict(wheel, now-previous)
        fix = inp.get('gnss')
        if fix is not None:
            if not 0. <= now-fix['stamp'] <= .5+1e-9:
                raise ValueError('unsupported GNSS acquisition age')
            accepted_before = oracle.accepted
            fixes.append((fix['stamp'], fix))
            oracle = chronological(config, motions, fixes, maps)
            if oracle.accepted > accepted_before and now-fix['stamp'] > 1e-9:
                compensated += 1
                replayed_maps += sum(stamp >= fix['stamp']-1e-9 for stamp, _, _ in maps)
        diagnostic = out.get('map_localization')
        if bool(diagnostic) != bool(config.get('localization_map')):
            raise ValueError('map enablement differs from sensor configuration')
        if diagnostic:
            if diagnostic['last_observed_stamp'] == now and diagnostic['matched_pose'] is not None:
                accepted = oracle.map_correct(diagnostic['matched_pose'], diagnostic['covariance'])
                if accepted != (diagnostic['decision'] == 'Accepted'):
                    raise ValueError('independent map innovation gate disagrees')
                if accepted:
                    scan = inp.get('lidar')
                    if not scan or scan['stamp'] != now or inp['lidar_failed']:
                        raise ValueError('map acceptance lacks a new synchronous measured scan')
                    if any(diagnostic['covariance'][i][i]+1e-12 < f for i, f in enumerate([.01, .01, 1e-4])):
                        raise ValueError('map measurement bypassed conservative variance floors')
                    maps.append((now, diagnostic['matched_pose'], diagnostic['covariance']))
            last_map = maps[-1][0] if maps else None
            if diagnostic['last_accepted_stamp'] != last_map:
                raise ValueError('map freshness was renewed without an original accepted acquisition')
        covered = bool(diagnostic and oracle.accepted_stamp is not None
                       and .75+1e-9 < now-oracle.accepted_stamp <= config['localization_map'].get('max_gnss_outage_s', 10.)+1e-9
                       and diagnostic['decision'] == 'Accepted' and maps and now-maps[-1][0] <= .2+1e-9)
        if diagnostic and diagnostic['gnss_outage_covered'] != covered:
            raise ValueError('map outage permission differs from actual acquisition clocks')
        covered_ticks += covered
        stale = oracle.accepted_stamp is None or (now-oracle.accepted_stamp > .75+1e-9 and not covered)
        if ('StaleGnss' in out['health']) != stale or (stale and out['command']['acceleration'] > -5.99):
            raise ValueError('stale localization did not retain the original braking rule')
        pose = out['estimate']['pose']
        delta = max(abs(pose['position']['x']-oracle.x[0]), abs(pose['position']['y']-oracle.x[1]), abs(angle(pose['yaw']-oracle.x[2])))
        maximum = max(maximum, delta)
        if delta > 1e-8 or abs(out['estimate']['speed']-oracle.speed) > 1e-8:
            raise ValueError(f'pose differs from full-history joint reconstruction at {now}: {delta}')
        if abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8:
            raise ValueError('posterior covariance differs from independent joint reconstruction')
        for key, value in [('accepted_fixes', oracle.accepted), ('rejected_fixes', oracle.rejected),
                           ('last_accepted_stamp', oracle.accepted_stamp), ('last_observed_stamp', oracle.observed),
                           ('last_decision', oracle.decision)]:
            if out['localization'][key] != value:
                raise ValueError(f'GNSS {key} changed without its original acquisition')
        if oracle.nis is not None and not math.isclose(out['localization']['last_nis'], oracle.nis, abs_tol=1e-8, rel_tol=1e-9):
            raise ValueError('GNSS innovation is inconsistent with acquisition-time map history')
        latency = out['gnss_latency']
        if (latency['compensated_fixes'] != compensated or latency['history_unavailable_fixes']
                or latency.get('history_conflict_fixes', 0) or latency['history_predictions'] > 64
                or latency.get('history_map_corrections', 0) > 64
                or latency.get('replayed_map_corrections', 0) != replayed_maps):
            raise ValueError('bounded event history diagnostics disagree')
        previous = now
    return {'passed': True, 'verified_ticks': len(ticks), 'maximum_pose_difference': maximum,
            'accepted_map_measurements': len(maps), 'replayed_map_corrections': replayed_maps,
            'compensated_fixes': compensated, 'covered_outage_ticks': covered_ticks,
            'accepted_fixes': oracle.accepted, 'rejected_fixes': oracle.rejected}


def audit(run, sensor_path, case):
    with sensor_path.open() as stream:
        config = json.loads(next(stream))['header']['config']
        ticks = [item['tick'] for line in stream if 'tick' in (item := json.loads(line))]
    if {'sensor_timing', 'gnss_delay_ticks', 'gnss_dropout_windows', 'objects', 'truth'} & config.keys():
        raise ValueError('evaluation truth or fault schedule crossed the operational boundary')
    scenario, frames = run['scenario'], run['frames']
    if len(ticks) != len(frames) or len(frames) != run['summary']['steps']:
        raise ValueError('full physical/control coverage is required')
    errors, yaws, outage_frames, accepted_errors, accepted_yaws = [], [], [], [], []
    first_outage_brake = None
    recovery = False
    for index, (tick, frame) in enumerate(zip(ticks, frames)):
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        if abs(now-index*.05) > 1e-8 or frame['time'] != now or frame['estimate'] != out['estimate']:
            raise ValueError('physical/control observations are not complete and aligned')
        acquisition = (index-8)*.05
        dropout = (scenario.get('gnss_dropout') is not None and acquisition >= scenario['gnss_dropout']) or any(
            acquisition+1e-9 >= w['from'] and acquisition < w['until']-1e-9 for w in scenario.get('gnss_dropout_windows', []))
        expected_fix = index >= 8 and (index-8) % 4 == 0 and not dropout
        if bool(inp.get('gnss')) != expected_fix or (expected_fix and abs(inp['gnss']['stamp']-acquisition) > 1e-9):
            raise ValueError('delivery changed the original GNSS acquisition schedule')
        truth, estimate = frame['truth']['pose'], out['estimate']['pose']
        error = math.dist([truth['position'][k] for k in ['x', 'y']], [estimate['position'][k] for k in ['x', 'y']])
        yaw = abs(angle(truth['yaw']-estimate['yaw']))
        errors.append(error)
        yaws.append(yaw)
        if out.get('map_localization', {}).get('decision') == 'Accepted':
            accepted_errors.append(error)
            accepted_yaws.append(yaw)
        if 3. <= now < 8.:
            outage_frames.append(frame)
        within_outage = now >= 3. and (case != 'recovery' or now < 8.)
        if within_outage and out['emergency'] and first_outage_brake is None:
            first_outage_brake = now
        recovery |= bool(inp.get('gnss') and inp['gnss']['stamp'] >= 8. and not out['health'])
    reconstruction = verify_ticks(config, ticks)
    summary = run['summary']
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(summary['localization_rmse']-rmse) > 1e-10 or abs(summary['localization_max_error']-maximum) > 1e-10:
        raise ValueError('truth-based error score differs from independent scoring')
    clearance = sweep(run)
    if (not summary['passed'] or summary['collisions'] or summary['road_violations'] or summary['min_clearance'] < 1.
            or (accepted_errors and max(accepted_errors) > .5) or (accepted_yaws and max(accepted_yaws) > .1)):
        raise ValueError('unchanged physical/fresh-map accuracy gates failed')
    distance = sum(math.dist([a['truth']['pose']['position'][k] for k in ['x', 'y']],
                             [b['truth']['pose']['position'][k] for k in ['x', 'y']]) for a, b in zip(outage_frames, outage_frames[1:]))
    if case == 'recovery':
        if (rmse > .2 or maximum > .4 or max(yaws) > .1 or not summary['reached_goal'] or not recovery
                or first_outage_brake is not None or distance < 10. or min(f['truth']['speed'] for f in outage_frames) < 2.
                or reconstruction['covered_outage_ticks'] < 60 or reconstruction['replayed_map_corrections'] == 0):
            raise ValueError('moving outage/recovery failed frozen accuracy or actual-motion gates')
    else:
        if summary['reached_goal'] or summary['final_speed'] > .2 or first_outage_brake is None:
            raise ValueError('fault control did not stop physically')
        lower, upper = {'prolonged': (12.8, 12.9), 'lidar-loss': (6.1, 6.25),
                        'no-overlap': (3.55, 3.65), 'disabled': (3.55, 3.65)}[case]
        if not lower < first_outage_brake <= upper:
            raise ValueError('fault bypassed its original acquisition-age braking deadline')
    return {'oracle': reconstruction, 'rmse_m': rmse, 'maximum_error_m': maximum, 'maximum_yaw_error_rad': max(yaws),
            'independent_clearance': clearance, 'first_outage_brake_s': first_outage_brake,
            'outage_distance_m': distance, 'actual_gnss_recovery': recovery}


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def archive(directory):
    """Retain raw recordings losslessly after each trial, including failures."""
    storage = {}
    for path in sorted(directory.rglob('*.json*')):
        if path.name.endswith('.gz'):
            continue
        digest = sha(path)
        destination = path.with_name(path.name+'.gz')
        with path.open('rb') as source, destination.open('xb') as raw:
            with gzip.GzipFile(filename='', fileobj=raw, mode='wb', mtime=0, compresslevel=6) as target:
                shutil.copyfileobj(source, target)
        with gzip.open(destination, 'rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != digest:
                raise ValueError('lossless recording archive failed verification')
        storage[str(path.relative_to(directory))] = {'path': str(destination.relative_to(directory)),
                                                    'sha256': digest, 'stored_sha256': sha(destination)}
        path.unlink()
    return storage


def run_case(binary, folder, case, seed, plant):
    folder.mkdir(parents=True, exist_ok=False)
    source = REPO/f'scenarios/gnss-map-{case}.json'
    args = ['run', '--scenario', source] if plant == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = {'case': case, 'seed': seed, 'plant': plant, 'passed': False}
    row['run_exit_code'] = execute(binary, args+['--seed', seed, '--output', folder/'run'], folder/'run-process')
    replay_binary = REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving')
    row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row.update(audit(run, folder/'run/sensors.jsonl', case))
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        row['passed'] = row['run_exit_code'] == 0 and row['replay_exit_code'] == 0 and row['replay']['verified'] and row['replay']['ticks'] == len(run['frames'])
    except Exception as error:
        row['audit_error'] = repr(error)
    row['storage'] = archive(folder)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--cases', nargs='+', choices=CASES, default=CASES)
    parser.add_argument('--plant', choices=['reference-2d', 'rne-dynamic'], default='reference-2d')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    args = parser.parse_args()
    rows = []
    for case in args.cases:
        for seed in args.seeds:
            rows.append(run_case(args.binary.resolve(), args.output.resolve()/case/f'seed-{seed}', case, seed, args.plant))
            report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
            (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps({k: v for k, v in rows[-1].items() if k not in ['storage', 'summary', 'replay']}), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
