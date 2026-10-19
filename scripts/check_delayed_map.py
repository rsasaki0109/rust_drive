#!/usr/bin/env python3
"""Independently reconstruct asynchronous GNSS/map fusion and physical recovery."""
import argparse
import json
import math
import os
from pathlib import Path

from check_gnss_latency import angle, verify_pose_covariance
from check_gnss_map_history import MapFilter, archive
from check_lane_change_speeds import sweep
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
CASES = ['gnss-late', 'gnss-early', 'recovery', 'expired']


class Conflict(ValueError):
    pass


def rebuild(config, motions, fixes, maps, until, priority=2):
    """Unbounded reconstruction using independent joint-matrix corrections.

    Equal-time GNSS (0) precedes map (1). A prior excludes observations of
    equal/higher priority. Only originally admitted measurements enter replay.
    """
    events = [(g['stamp'], 0, g, None) for g in fixes] + [(t, 1, p, c) for t, p, c in maps]
    events = sorted([e for e in events if e[0] < until-1e-9 or (abs(e[0]-until) <= 1e-9 and e[1] < priority)], key=lambda e: (e[0], e[1]))
    result, index = MapFilter(config), 0

    def apply(event):
        _, kind, measurement, covariance = event
        if kind == 0:
            result.correct(measurement)
            if result.decision != 'Accepted':
                raise Conflict('an originally accepted GNSS fix fails its replay gate')
        elif not result.map_correct(measurement, covariance):
            raise Conflict('an originally accepted map pose fails its replay gate')

    while index < len(events) and events[index][0] <= 1e-9:
        apply(events[index])
        index += 1
    for start, end, motion in motions:
        if start >= until-1e-9:
            break
        cursor, end = start, min(end, until)
        while index < len(events) and events[index][0] <= end+1e-9:
            stamp = events[index][0]
            if stamp < start-1e-9:
                raise ValueError('observation is outside continuous measured motion')
            result.predict(motion, max(0., stamp-cursor))
            apply(events[index])
            cursor, index = stamp, index+1
        result.predict(motion, max(0., end-cursor))
    if index != len(events):
        raise ValueError('observations have no motion coverage')
    return result


def pose_difference(pose, values):
    return max(abs(pose['position']['x']-values[0]), abs(pose['position']['y']-values[1]), abs(angle(pose['yaw']-values[2])))


def verify_ticks(config, ticks):
    enabled = config['localization_map'].get('allow_delayed_scans', False)
    if not config.get('gnss_latency_compensation'):
        raise ValueError('GNSS event history must be explicitly enabled')
    motions, fixes, maps = [], [], []
    previous = wheel = fault = None
    observed = accepted_stamp = decision = nis = None
    accepted = rejected = compensated = replayed_maps = replayed_gnss = covered_ticks = 0
    maximum = maximum_prior = 0.
    oracle = MapFilter(config)
    for tick in ticks:
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        wheel = inp.get('odometry') or wheel
        if previous is not None:
            if wheel is None or now-wheel['stamp'] > .15+1e-9 or now-previous > .25+1e-9:
                raise ValueError('this physical protocol requires continuous healthy odometry')
            motions.append((previous, now, wheel))
            oracle.predict(wheel, now-previous)
        if inp['lidar_failed']:
            fault = now
            if out['command']['acceleration'] > -5.99:
                raise ValueError('an explicit LiDAR failure did not brake immediately')
        fix = inp.get('gnss')
        if fix is not None:
            if observed is not None and fix['stamp'] <= observed:
                raise ValueError('physical protocol unexpectedly received repeated GNSS')
            if not 0. <= now-fix['stamp'] <= .5+1e-9:
                raise ValueError('GNSS delay exceeds supported rewind')
            prior = rebuild(config, motions, fixes, maps, fix['stamp'], 0)
            prior.correct(fix)
            observed, decision, nis = fix['stamp'], prior.decision, prior.nis
            if decision == 'Accepted':
                fixes.append(fix)
                oracle = rebuild(config, motions, fixes, maps, now)
                accepted, accepted_stamp = accepted+1, fix['stamp']
                if now-fix['stamp'] > 1e-9:
                    compensated += 1
                replayed_maps += sum(t >= fix['stamp']-1e-9 for t, _, _ in maps)
            else:
                rejected += 1
        diagnostic = out['map_localization']
        scan = inp.get('lidar')
        if enabled and scan and diagnostic['last_observed_stamp'] == scan['stamp'] and diagnostic.get('acquisition_prior') is not None:
            stamp = scan['stamp']
            if now-stamp > .2+1e-9 or (fault is not None and stamp <= fault):
                raise ValueError('expired or pre-fault map acquisition entered registration')
            prior = rebuild(config, motions, fixes, maps, stamp, 1)
            delta = pose_difference(diagnostic['acquisition_prior'], prior.x)
            maximum_prior = max(maximum_prior, delta)
            if delta > 1e-8 or abs(diagnostic['acquisition_delay_s']-(now-stamp)) > 1e-9:
                raise ValueError('map registration seed is inconsistent with acquisition time')
            if diagnostic['matched_pose'] is not None:
                measurement = (stamp, diagnostic['matched_pose'], diagnostic['covariance'])
                admitted = prior.map_correct(measurement[1], measurement[2])
                candidate = None
                if admitted:
                    try:
                        candidate = rebuild(config, motions, fixes, maps+[measurement], now)
                    except Conflict:
                        admitted = False
                if admitted != (diagnostic['decision'] == 'Accepted'):
                    raise ValueError('map transaction disagrees with independent gate/replay')
                if admitted:
                    oracle = candidate
                    maps.append(measurement)
                    replayed_gnss += sum(g['stamp'] > stamp+1e-9 for g in fixes)
        last_map = maps[-1][0] if maps else None
        if diagnostic['last_accepted_stamp'] != last_map:
            raise ValueError('map accepted clock changed without an admitted acquisition')
        covered = (accepted_stamp is not None and .75+1e-9 < now-accepted_stamp
                   <= config['localization_map'].get('max_gnss_outage_s', 10.)+1e-9
                   and diagnostic['decision'] == 'Accepted' and last_map is not None and now-last_map <= .2+1e-9)
        if diagnostic['gnss_outage_covered'] != covered:
            raise ValueError('map support changed original acquisition freshness')
        covered_ticks += covered
        stale = accepted_stamp is None or (now-accepted_stamp > .75+1e-9 and not covered)
        if ('StaleGnss' in out['health']) != stale or (stale and out['command']['acceleration'] > -5.99):
            raise ValueError('stale localization bypassed its original brake')
        delta = pose_difference(out['estimate']['pose'], oracle.x)
        maximum = max(maximum, delta)
        if delta > 1e-8 or abs(out['estimate']['speed']-oracle.speed) > 1e-8:
            raise ValueError(f'full-history pose/speed disagreement at {now}: {delta}')
        if abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8:
            raise ValueError('posterior covariance differs from independent matrix reconstruction')
        verify_pose_covariance(out, oracle.p, config)
        for key, value in [('accepted_fixes', accepted), ('rejected_fixes', rejected), ('last_observed_stamp', observed),
                           ('last_accepted_stamp', accepted_stamp), ('last_decision', decision)]:
            if out['localization'][key] != value:
                raise ValueError(f'GNSS observation ownership differs: {key}')
        if nis is not None and not math.isclose(out['localization']['last_nis'], nis, abs_tol=1e-8, rel_tol=1e-9):
            raise ValueError('GNSS original-delivery NIS was overwritten by map replay')
        latency = out['gnss_latency']
        if (latency['compensated_fixes'] != compensated or latency.get('replayed_map_corrections', 0) != replayed_maps
                or latency.get('replayed_gnss_corrections', 0) != replayed_gnss or latency['history_unavailable_fixes']
                or latency.get('history_conflict_fixes', 0) or any(latency.get(k, 0) > 64 for k in
                    ['history_predictions', 'history_map_corrections', 'history_gnss_corrections'])):
            raise ValueError('bounded correction-event diagnostics disagree')
        previous = now
    return {'passed': True, 'verified_ticks': len(ticks), 'accepted_fixes': accepted, 'rejected_fixes': rejected,
            'admitted_map_acquisitions': len(maps), 'compensated_fixes': compensated, 'replayed_map_corrections': replayed_maps,
            'replayed_gnss_corrections': replayed_gnss, 'covered_outage_ticks': covered_ticks,
            'maximum_pose_difference': maximum, 'maximum_acquisition_prior_difference': maximum_prior}


def audit(run, sensor_path, case, control=False):
    with sensor_path.open() as stream:
        config = json.loads(next(stream))['header']['config']
        ticks = [r['tick'] for line in stream if 'tick' in (r := json.loads(line))]
    scenario, frames, summary = run['scenario'], run['frames'], run['summary']
    if config['localization_map'] != scenario['localization_map'] or {'truth', 'objects', 'sensor_timing', 'gnss_dropout_windows'} & config.keys():
        raise ValueError('operational map boundary differs from its declared offline prior')
    if len(frames) != len(ticks) or len(frames) != summary['steps']:
        raise ValueError('complete physical/control coverage is required')
    delay = scenario['sensor_timing'].get('gnss_delay_ticks', 0)
    errors, yaws, outage, stopped, resumed = [], [], [], [], []
    first_outage_brake = None
    for index, (tick, frame) in enumerate(zip(ticks, frames)):
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        if abs(now-index*.05) > 1e-8 or now != frame['time'] or out['estimate'] != frame['estimate']:
            raise ValueError('unaligned or incomplete physical/control ticks')
        acquisition = (index-delay)*.05
        dropout = any(acquisition+1e-9 >= w['from'] and acquisition < w['until']-1e-9 for w in scenario['gnss_dropout_windows'])
        delivered = index >= delay and (index-delay) % 4 == 0 and not dropout
        if bool(inp.get('gnss')) != delivered or (delivered and abs(inp['gnss']['stamp']-acquisition) > 1e-9):
            raise ValueError('GNSS raw acquisition/delivery schedule changed')
        truth, estimate = frame['truth']['pose'], out['estimate']['pose']
        errors.append(math.dist([truth['position'][k] for k in ['x', 'y']], [estimate['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(truth['yaw']-estimate['yaw'])))
        if 3. <= now < 8.:
            outage.append(frame)
            if 'StaleGnss' in out['health'] and first_outage_brake is None:
                first_outage_brake = now
        if 4.8 <= now < 5.5:
            stopped.append(frame['truth']['speed'])
        if 6. <= now < 8. and not out['health'] and out['map_localization']['gnss_outage_covered']:
            resumed.append(frame['truth']['speed'])
    reconstruction = verify_ticks(config, ticks)
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('physical localization score differs from independent scoring')
    clearance = sweep(run)
    if summary['collisions'] or summary['road_violations'] or clearance['minimum_swept_clearance_m'] < 1.:
        raise ValueError('unchanged independent physical clearance gates failed')
    distance = sum(math.dist([a['truth']['pose']['position'][k] for k in ['x', 'y']],
                             [b['truth']['pose']['position'][k] for k in ['x', 'y']]) for a, b in zip(outage, outage[1:]))
    minimum_speed = min(f['truth']['speed'] for f in outage)
    if control:
        if reconstruction['admitted_map_acquisitions'] or minimum_speed > .05 or (distance >= 10. and minimum_speed >= 2.):
            raise ValueError('old synchronous-only control did not fail the moving-outage gate')
    else:
        if not summary['passed'] or rmse > .2 or maximum > .4 or max(yaws) > .1:
            raise ValueError('whole-episode accuracy or physical acceptance failed')
        if case == 'expired':
            if (reconstruction['admitted_map_acquisitions'] or summary['reached_goal'] or summary['final_speed'] > .2
                    or first_outage_brake is None or abs(first_outage_brake-3.6) > 1e-8):
                raise ValueError('expired map observations did not preserve original GNSS-age stop')
        elif not summary['reached_goal'] or not any(t['input'].get('gnss') and t['input']['gnss']['stamp'] >= 8. for t in ticks):
            raise ValueError('new genuine GNSS did not recover to the goal')
        elif case == 'recovery':
            if not stopped or min(stopped) > .05 or not resumed or max(resumed) < 2.:
                raise ValueError('transient fault lacked an actual stop and map-only recovery')
        elif distance < 10. or minimum_speed < 2. or first_outage_brake is not None:
            raise ValueError('asynchronous map did not sustain the frozen moving outage')
    return {'oracle': reconstruction, 'rmse_m': rmse, 'maximum_error_m': maximum, 'maximum_yaw_rad': max(yaws),
            'independent_clearance': clearance, 'outage_distance_m': distance, 'outage_min_speed_m_s': minimum_speed,
            'first_outage_brake_s': first_outage_brake, 'fault_window_min_speed_m_s': min(stopped) if stopped else None,
            'map_only_recovery_max_speed_m_s': max(resumed) if resumed else None}


def run_case(binary, folder, case, seed, plant, control=False):
    folder.mkdir(parents=True, exist_ok=False)
    scenario = json.loads((REPO/f'scenarios/delayed-map-{case}.json').read_text())
    if control:
        scenario['localization_map']['allow_delayed_scans'] = False
    source = folder/'scenario.json'
    source.write_text(json.dumps(scenario, separators=(',', ':'))+'\n')
    args = ['run', '--scenario', source] if plant == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = {'case': case, 'seed': seed, 'plant': plant, 'control': control, 'passed': False}
    row['run_exit_code'] = execute(binary, args+['--seed', seed, '--output', folder/'run'], folder/'run-process')
    replay_binary = REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving')
    row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row.update(audit(run, folder/'run/sensors.jsonl', case, control))
        row['passed'] = row['replay_exit_code'] == 0 and row['replay']['verified'] and row['replay']['ticks'] == len(run['frames']) and (control or row['run_exit_code'] == 0)
    except Exception as error:
        row['audit_error'] = repr(error)
    row['storage'] = archive(folder)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[7])
    parser.add_argument('--cases', choices=CASES, nargs='+', default=CASES)
    parser.add_argument('--controls', action='store_true')
    parser.add_argument('--plant', choices=['reference-2d', 'rne-dynamic'], default='reference-2d')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    args = parser.parse_args()
    rows = []
    for case, control in [(c, False) for c in args.cases]+([('gnss-late', True)] if args.controls else []):
        for seed in args.seeds:
            rows.append(run_case(args.binary.resolve(), args.output.resolve()/('controls' if control else 'enabled')/case/f'seed-{seed}', case, seed, args.plant, control))
            report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
            (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(json.dumps({k: v for k, v in rows[-1].items() if k not in ['storage', 'summary', 'replay']}), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
