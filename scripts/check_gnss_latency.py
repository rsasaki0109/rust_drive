#!/usr/bin/env python3
"""Independently rebuild delayed GNSS fusion from delivered observations."""
import argparse
import json
import math
import os
from pathlib import Path

from check_speed_zones import execute
from check_lane_change_speeds import sweep

REPO = Path(__file__).resolve().parents[1]
CASES = ['straight', 'curve', 'curve-fast', 'dropout', 'outlier', 'obstacle', 'lead']


def transpose(a):
    return list(map(list, zip(*a)))


def product(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def add(a, b):
    return [[x+y for x, y in zip(row, other)] for row, other in zip(a, b)]


def angle(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


class Filter:
    def __init__(self, config):
        pose = config['initial_pose']
        self.x = [pose['position']['x'], pose['position']['y'], pose['yaw']]
        initial = config.get('initial_pose_covariance')
        self.p = [list(row) for row in (initial if initial is not None else [[1., 0., 0.], [0., 1., 0.], [0., 0., .03]])]
        self.speed = 0.
        self.offset = config.get('rear_axle_offset_m') or 0.
        self.gnss_shape = config.get('gnss_covariance_shape') or [[1., 0.], [0., 1.]]
        self.antenna = config.get('gnss_antenna_offset_m') or {'x': 0., 'y': 0.}
        self.accepted, self.rejected = 0, 0
        self.observed, self.accepted_stamp, self.nis, self.decision = None, None, None, None

    def predict(self, motion, dt):
        if dt <= 1e-12:
            return
        yaw, speed, rate = self.x[2], max(0., motion['speed']), motion['yaw_rate']
        if self.offset:
            middle, lateral = yaw+rate*dt/2, self.offset*rate
            dx = (speed*math.cos(middle)-lateral*math.sin(middle))*dt
            dy = (speed*math.sin(middle)+lateral*math.cos(middle))*dt
            self.speed = math.hypot(speed, lateral)
        else:
            dx, dy = speed*math.cos(yaw)*dt, speed*math.sin(yaw)*dt
            self.speed = speed
        jacobian = [[1., 0., -dy], [0., 1., dx], [0., 0., 1.]]
        noise = [[.015*dt, 0., 0.], [0., .015*dt, 0.], [0., 0., .0005*dt]]
        self.p = add(product(product(jacobian, self.p), transpose(jacobian)), noise)
        self.x = [self.x[0]+dx, self.x[1]+dy, angle(yaw+rate*dt)]

    def correct(self, fix):
        # Joint two-dimensional inverse/gain and Joseph update, rather than
        # the producer's whitening and sequential scalar corrections.
        if self.observed is not None and fix['stamp'] <= self.observed:
            return
        self.observed = fix['stamp']
        yaw = self.x[2]
        forward, left = self.antenna['x'], self.antenna['y']
        arm = [forward*math.cos(yaw)-left*math.sin(yaw), forward*math.sin(yaw)+left*math.cos(yaw)]
        h = [[1., 0., -arm[1]], [0., 1., arm[0]]]
        innovation = [fix['position'][key]-self.x[i]-arm[i] for i, key in enumerate(['x', 'y'])]
        r = fix['variance']
        noise = [[r*v for v in row] for row in self.gnss_shape]
        s = add(product(product(h, self.p), transpose(h)), noise)
        a, b, c = s[0][0], s[0][1], s[1][1]
        determinant = a*c-b*b
        inverse = [[c/determinant, -b/determinant], [-b/determinant, a/determinant]]
        self.nis = sum(innovation[i]*inverse[i][j]*innovation[j] for i in range(2) for j in range(2))
        if self.nis > 36.:
            self.rejected += 1
            self.decision = 'RejectedInnovation'
            return
        gain = product(product(self.p, transpose(h)), inverse)
        kh = product(gain, h)
        residual = [[float(i==j)-kh[i][j] for j in range(3)] for i in range(3)]
        self.p = add(product(product(residual, self.p), transpose(residual)), product(product(gain, noise), transpose(gain)))
        self.x = [self.x[i]+sum(gain[i][j]*innovation[j] for j in range(2)) for i in range(3)]
        self.x[2] = angle(self.x[2])
        self.accepted += 1
        self.accepted_stamp, self.decision = fix['stamp'], 'Accepted'


def chronological(config, motions, delivered):
    """Rebuild the complete past in time order; no bounded producer snapshots."""
    result = Filter(config)
    fixes = sorted(delivered, key=lambda event: event[0])
    cursor, index = 0., 0
    while index < len(fixes) and fixes[index][0] <= 1e-9:
        result.correct(fixes[index][1])
        index += 1
    for start, end, motion in motions:
        cursor = start
        while index < len(fixes) and fixes[index][0] <= end+1e-9:
            stamp, fix = fixes[index]
            if stamp < start-1e-9:
                raise ValueError('measurement outside reconstructed motion coverage')
            result.predict(motion, max(0., stamp-cursor))
            result.correct(fix)
            cursor, index = stamp, index+1
        result.predict(motion, max(0., end-cursor))
    if index != len(fixes):
        raise ValueError('future observation entered the chronological oracle')
    return result


def verify_ticks(config, ticks):
    if config.get('localization_map'):
        raise ValueError('external map correction replay is outside this contract')
    enabled = config.get('gnss_latency_compensation', False)
    motions, delivered, previous, wheel = [], [], None, None
    oracle, maximum, compensated = Filter(config), 0., 0
    for tick in ticks:
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        if inp.get('odometry') is not None:
            wheel = inp['odometry']
        if previous is not None:
            if wheel is None or now-wheel['stamp'] > .15+1e-9 or now-previous > .25+1e-9:
                raise ValueError('this physical audit requires covered healthy motion intervals')
            motions.append((previous, now, wheel))
            oracle.predict(wheel, now-previous)
        fix = inp.get('gnss')
        if fix is not None:
            if not 0. <= now-fix['stamp'] <= .5+1e-9:
                raise ValueError('physical positive received an unsupported acquisition age')
            delivered.append((fix['stamp'] if enabled else now, fix))
            oracle = chronological(config, motions, delivered)
            if oracle.decision == 'Accepted' and enabled and now-fix['stamp'] > 1e-9:
                compensated += 1
        expected = out['estimate']
        values = [expected['pose']['position']['x'], expected['pose']['position']['y'], expected['pose']['yaw']]
        delta = max(abs(values[i]-oracle.x[i]) if i<2 else abs(angle(values[i]-oracle.x[i])) for i in range(3))
        maximum = max(maximum, delta)
        if delta > 1e-8 or abs(expected['speed']-oracle.speed) > 1e-8:
            raise ValueError(f'pose differs from chronological oracle at {now}: {delta}')
        if abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8:
            raise ValueError('posterior covariance differs from independent joint solution')
        verify_pose_covariance(out, oracle.p, config)
        diagnostics = out['localization']
        for key, value in [('accepted_fixes', oracle.accepted), ('rejected_fixes', oracle.rejected),
                           ('last_accepted_stamp', oracle.accepted_stamp), ('last_observed_stamp', oracle.observed),
                           ('last_decision', oracle.decision)]:
            if diagnostics[key] != value:
                raise ValueError(f'localization diagnostic {key} disagrees with original acquisition history')
        if oracle.nis is not None and not math.isclose(diagnostics['last_nis'], oracle.nis, abs_tol=1e-8, rel_tol=1e-9):
            raise ValueError('innovation was not evaluated at the claimed acquisition time')
        if oracle.accepted_stamp is None or now-oracle.accepted_stamp > .75+1e-9:
            if 'StaleGnss' not in out['health'] or out['command']['acceleration'] > -5.99:
                raise ValueError('delivery refreshed stale GNSS health')
        if enabled:
            latency = out['gnss_latency']
            if latency['compensated_fixes'] != compensated or latency['history_predictions'] > 64 or latency['history_unavailable_fixes']:
                raise ValueError('bounded history/compensation evidence disagrees')
        previous = now
    return {'passed': True, 'verified_ticks': len(ticks), 'maximum_pose_difference': maximum,
            'compensated_fixes': compensated, 'accepted_fixes': oracle.accepted, 'rejected_fixes': oracle.rejected}


def verify_pose_covariance(output, expected, config):
    """Independent full-matrix comparison and Sylvester positivity check."""
    if all(config.get(key) is None for key in ['initial_pose_covariance', 'gnss_covariance_shape', 'gnss_recovery', 'gnss_antenna_offset_m']):
        if output.get('pose_covariance') is not None: raise ValueError('legacy output unexpectedly claims a full covariance')
        return 0.
    p = output.get('pose_covariance')
    if p is None or len(p) != 3 or any(len(row) != 3 for row in p): raise ValueError('explicit localization calibration requires a full posterior')
    if any(not math.isfinite(p[i][j]) or abs(p[i][j]-p[j][i]) > 1e-12 for i in range(3) for j in range(3)):
        raise ValueError('nonfinite or asymmetric posterior')
    determinant = (p[0][0]*(p[1][1]*p[2][2]-p[1][2]*p[2][1])
                   -p[0][1]*(p[1][0]*p[2][2]-p[1][2]*p[2][0])+p[0][2]*(p[1][0]*p[2][1]-p[1][1]*p[2][0]))
    if p[0][0] <= 0. or p[0][0]*p[1][1]-p[0][1]*p[1][0] <= 0. or determinant <= 0.:
        raise ValueError('posterior is not positive definite')
    difference = max(abs(p[i][j]-expected[i][j]) for i in range(3) for j in range(3))
    if difference > 1e-8: raise ValueError('full posterior differs from independent reconstruction')
    return difference


def audit(run, sensor_path, case):
    with sensor_path.open() as stream:
        header = json.loads(next(stream))['header']
        ticks = [item['tick'] for line in stream if 'tick' in (item := json.loads(line))]
    config = header['config']
    if {'sensor_timing', 'gnss_delay_ticks', 'gnss_bias_windows', 'gnss_dropout_windows', 'objects'} & config.keys():
        raise ValueError('simulator schedule or truth entered operational configuration')
    reconstruction = verify_ticks(config, ticks)
    frames = run['frames']
    if len(frames) != run['summary']['steps'] or len(ticks) != len(frames):
        raise ValueError('full physical interval coverage is required')
    errors = [math.dist((f['estimate']['pose']['position'][k] for k in ['x','y']),
                        (f['truth']['pose']['position'][k] for k in ['x','y'])) for f in frames]
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(rmse-run['summary']['localization_rmse']) > 1e-10 or abs(maximum-run['summary']['localization_max_error']) > 1e-10:
        raise ValueError('independent physical localization score disagrees')
    fault = None
    if case in ['dropout', 'outlier']:
        relevant = [f for f in frames if 9.5 <= f['time'] < (11. if case=='dropout' else 10.4)]
        if not relevant or min(f['truth']['speed'] for f in relevant) > .05:
            raise ValueError('GNSS fault did not produce an actual physical stop')
        recovered = [t for t in ticks if t['input']['time'] > 12. and not t['expected']['health']]
        if not recovered or not run['summary']['reached_goal']:
            raise ValueError('new healthy acquisitions did not recover driving')
        fault = {'physical_stop': True, 'healthy_recovery': True}
    if case == 'lead':
        waiting = [f for f in frames if 14. <= f['time'] < 24.]
        moving = [f for f in frames if f['time'] > 25.]
        stopped = min(f['truth']['speed'] for f in waiting)
        resumed = max(f['truth']['speed'] for f in moving)
        if stopped > .05 or resumed < 2.:
            raise ValueError('lead interaction lacked its frozen physical stop/resumption')
        return {'oracle': reconstruction, 'independent_clearance': sweep(run), 'rmse_m': rmse, 'maximum_error_m': maximum,
                'accuracy_passed': rmse <= .2 and maximum <= .4, 'fault': fault,
                'interaction': {'observed_lead_stop_speed_m_s': stopped, 'observed_resumed_speed_m_s': resumed}}
    return {'oracle': reconstruction, 'independent_clearance': sweep(run), 'rmse_m': rmse, 'maximum_error_m': maximum,
            'accuracy_passed': rmse <= .2 and maximum <= .4, 'fault': fault}


def run_case(binary, folder, case, seed, plant='reference-2d', compensated=True):
    folder.mkdir(parents=True, exist_ok=False)
    scenario = json.loads((REPO/f'scenarios/gnss-delay-{case}.json').read_text())
    scenario['gnss_latency_compensation'] = compensated
    source = folder/'scenario.json'
    source.write_text(json.dumps(scenario, indent=2)+'\n')
    args = ['run', '--scenario', source] if plant=='reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = {'case': case, 'seed': seed, 'plant': plant, 'compensated': compensated, 'passed': False}
    row['run_exit_code'] = execute(binary, args+['--seed', seed, '--output', folder/'run'], folder/'run-process')
    replay_binary = REPO/'target/release'/('rustdriving.exe' if os.name=='nt' else 'rustdriving')
    row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row.update(audit(run, folder/'run/sensors.jsonl', case))
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        row['passed'] = row['replay_exit_code']==0 and row['replay']['verified'] and row['replay']['ticks']==len(run['frames'])
        if compensated:
            row['passed'] &= row['run_exit_code']==0 and row['summary']['passed'] and row['accuracy_passed'] and row['oracle']['compensated_fixes']>0
    except Exception as error:
        row['audit_error'] = repr(error)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name=='nt' else 'rustdriving'))
    parser.add_argument('--plant', choices=['reference-2d', 'rne-dynamic'], default='reference-2d')
    args = parser.parse_args()
    rows=[]
    for case in CASES:
        for seed in args.seeds:
            rows.append(run_case(args.binary.resolve(), args.output.resolve()/case/f'seed-{seed}', case, seed, args.plant))
            report={'passed':all(r['passed'] for r in rows), 'episodes':rows}
            (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':
    main()
