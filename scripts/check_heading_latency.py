#!/usr/bin/env python3
"""Qualify acquisition-time heading fusion without changing physical gates."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import os
from pathlib import Path

from check_absolute_heading import HeadingFilter, verify_covariance
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_heading_history import GNSS_FIELDS, chronological, materialize as original_input
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_road_width import road_margin, swept_clearance, validate_inputs
from check_gnss_covariance import contains_declared

REPO = Path(__file__).resolve().parents[1]
GATES = dict(rmse_m=.3, maximum_error_m=.6, maximum_yaw_rad=.1,
             minimum_clearance_m=1., minimum_road_margin_m=.1,
             maximum_heading_delay_s=.2, maximum_heading_age_s=.35,
             maximum_gnss_age_s=.75, original_deadline_s=45.,
             maximum_actual_fault_hold_speed_m_s=.05,
             equal_time_order=['prediction', 'gnss', 'heading', 'map'],
             default_enabled=False, all_frames_scored=True)


def specifications(plants, replay_composition=False):
    if replay_composition:
        return [dict(plant=p, world='curve', seed=251, heading_delay_ticks=4, delay_ticks=1,
                     heading_period_ticks=2, compensated=True, kind='base', mode='later-gnss-replay') for p in plants]
    rows = [dict(plant=p, world='curve', seed=s, heading_delay_ticks=h, delay_ticks=g,
                 compensated=c, kind='base', mode=f'h{h}-g{g}-'+('history' if c else 'sync'))
            for p in plants for s in [227, 229] for h, g in [(1, 7), (4, 1)] for c in [False, True]]
    rows += [dict(plant=p, world='curve', seed=233, heading_delay_ticks=4, delay_ticks=1,
                  compensated=True, kind=k, mode=k)
             for p in plants for k in ['heading-loss', 'heading-jump', 'gnss-loss']]
    rows += [dict(plant=p, world='curve', seed=239, heading_delay_ticks=3, delay_ticks=3,
                  compensated=True, kind='base', mode='equal-time') for p in plants]
    rows += [dict(plant=p, world='curve', seed=241, heading_delay_ticks=5, delay_ticks=1,
                  compensated=True, kind='unsupported-delay', mode='unsupported-delay') for p in plants]
    return rows


def materialize(spec):
    source, value = original_input(dict(spec, compensated=True))
    value['name'] = 'Delayed acquired heading '+spec['mode']
    value['heading_latency_compensation'] = spec['compensated']
    value['heading_sensor']['delay_ticks'] = spec['heading_delay_ticks']
    value['heading_sensor']['period_ticks'] = spec.get('heading_period_ticks', 4)
    value['initial_heading_offset_rad'] = (-1. if spec['seed'] == 227 else 1.)*math.pi/60.
    return source, value


def heading_delivery(oracle, sample, now, enabled, config, motions, fixes, headings):
    """Dense acquisition conditioning followed by a complete unbounded rebuild.

    No Rust checkpoints, recorded estimates or evaluation truth enter this fold.
    Delivery diagnostics belong to the live observer, not the replayed events.
    """
    if sample is None:
        return oracle, False, 0
    stamp, yaw, variance = (sample[k] for k in ['stamp', 'yaw_rad', 'variance_rad2'])
    valid = all(math.isfinite(x) for x in [stamp, yaw, variance]) and stamp >= 0. and abs(yaw) <= math.pi and 0. < variance <= math.pi**2
    if not valid:
        oracle.heading_fault = (now, 'InvalidAbsoluteHeading')
    elif oracle.heading['last_observed_stamp'] is None or stamp > oracle.heading['last_observed_stamp']:
        limit = .2 if enabled else 0.
        if stamp > now+1e-9 or now-stamp > limit+1e-9:
            oracle.heading_fault = (now, 'InvalidAbsoluteHeading')
        else:
            prior = chronological(config, motions, fixes, headings, stamp, priority=2)
            prior.heading = copy.deepcopy(oracle.heading)
            prior.heading_fault = oracle.heading_fault
            before = prior.heading['accepted_samples']
            prior.observe_heading(sample, stamp)
            if prior.heading['accepted_samples'] > before:
                candidate = chronological(config, motions, fixes, headings+[sample], now)
                for key in GNSS_FIELDS:
                    setattr(candidate, key, getattr(oracle, key))
                candidate.heading = prior.heading
                candidate.heading_fault = prior.heading_fault
                return candidate, True, sum(g['stamp'] > stamp+1e-9 for g in fixes)
            oracle.heading = prior.heading
            oracle.heading_fault = (now, 'HeadingInnovationHold')
    return oracle, False, 0


def reconstruct(config, ticks):
    if not config.get('absolute_heading_fusion') or not config.get('gnss_latency_compensation') or config.get('localization_map') or config.get('relocalization'):
        raise ValueError('physical oracle requires one continuously covered GNSS/heading epoch')
    oracle = HeadingFilter(config)
    motions, fixes, headings = [], [], []
    previous = None
    maximum = covariance_difference = 0.
    replayed_heading = replayed_gnss = delayed_gnss = 0
    health_counts = {}
    for tick in ticks:
        inp, out = tick['input'], tick['expected']; now = inp['time']
        if previous is not None:
            wheel = inp.get('odometry')
            if (wheel is None or not all(math.isfinite(wheel[k]) for k in ['stamp', 'speed', 'yaw_rate'])
                    or not 0. <= now-wheel['stamp'] <= .15+1e-9 or not 0. < now-previous <= .25+1e-9):
                raise ValueError('uncovered motion cannot support acquisition-time heading')
            motions.append((previous, now, wheel)); oracle.predict(wheel, now-previous)
        fix = inp.get('gnss')
        if fix is not None:
            if not 0. <= now-fix['stamp'] <= .5+1e-9:
                raise ValueError('unsupported GNSS acquisition age')
            prior = chronological(config, motions, fixes, headings, fix['stamp'], priority=1)
            for key in GNSS_FIELDS: setattr(prior, key, getattr(oracle, key))
            before = prior.accepted; prior.correct(fix)
            if prior.accepted > before:
                candidate = chronological(config, motions, fixes+[fix], headings, now)
                for key in GNSS_FIELDS: setattr(candidate, key, getattr(prior, key))
                candidate.heading, candidate.heading_fault = oracle.heading, oracle.heading_fault
                oracle = candidate; fixes.append(fix)
                delayed_gnss += now-fix['stamp'] > 1e-9
                replayed_heading += sum(h['stamp'] >= fix['stamp']-1e-9 for h in headings)
            else:
                for key in GNSS_FIELDS: setattr(oracle, key, getattr(prior, key))
        sample = inp.get('absolute_heading')
        oracle, accepted, count = heading_delivery(oracle, sample, now, config.get('heading_latency_compensation', False), config, motions, fixes, headings)
        if accepted: headings.append(sample)
        replayed_gnss += count
        issues = [] if oracle.heading_fault is None else [oracle.heading_fault[1]]
        if oracle.heading['last_accepted_stamp'] is None or now-oracle.heading['last_accepted_stamp'] > .35+1e-9:
            issues.append('StaleAbsoluteHeading')
        pose = out['estimate']['pose']
        if not all(math.isfinite(v) for v in [pose['position']['x'], pose['position']['y'], pose['yaw'], out['estimate']['speed']]):
            raise ValueError('nonfinite acquired-heading posterior')
        difference = max(abs(pose['position'][k]-oracle.x[i]) for i, k in enumerate(['x', 'y']))
        difference = max(difference, abs(angle(pose['yaw']-oracle.x[2])), abs(out['estimate']['speed']-oracle.speed))
        maximum = max(maximum, difference)
        if difference > 1e-8: raise ValueError(f'acquisition posterior differs at {now}: {difference}')
        covariance_difference = max(covariance_difference, verify_covariance(out.get('pose_covariance'), oracle.p))
        if abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8:
            raise ValueError('posterior covariance trace differs')
        for key, value in oracle.heading.items():
            actual = out.get('absolute_heading', {}).get(key)
            if isinstance(value, float):
                if actual is None or not math.isclose(actual, value, rel_tol=1e-8, abs_tol=1e-8):
                    raise ValueError('heading diagnostic ownership differs: '+key)
            elif actual != value: raise ValueError('heading diagnostic ownership differs: '+key)
        for key, value in [('last_observed_stamp', oracle.observed), ('last_accepted_stamp', oracle.accepted_stamp),
                           ('last_decision', oracle.decision), ('accepted_fixes', oracle.accepted), ('rejected_fixes', oracle.rejected)]:
            if out['localization'][key] != value: raise ValueError('GNSS diagnostic ownership differs: '+key)
        if oracle.nis is not None and not math.isclose(out['localization']['last_nis'], oracle.nis, rel_tol=1e-8, abs_tol=1e-8):
            raise ValueError('GNSS acquisition innovation differs')
        if [x for x in out['health'] if x in ['InvalidAbsoluteHeading', 'StaleAbsoluteHeading', 'HeadingInnovationHold']] != issues:
            raise ValueError('heading acquisition freshness or fault latch differs')
        stale_gnss = oracle.accepted_stamp is None or now-oracle.accepted_stamp > .75+1e-9
        if ('StaleGnss' in out['health']) != stale_gnss: raise ValueError('heading refreshed GNSS age')
        if (issues or stale_gnss) and out['command']['acceleration'] > -5.99: raise ValueError('required sensor fault did not brake')
        for issue in issues: health_counts[issue] = health_counts.get(issue, 0)+1
        latency = out.get('gnss_latency')
        if (latency is None or latency['compensated_fixes'] != delayed_gnss
                or latency.get('replayed_heading_corrections', 0) != replayed_heading
                or latency.get('replayed_gnss_corrections', 0) != replayed_gnss
                or latency.get('history_heading_corrections', 0) > 64):
            raise ValueError('history counters differ from admitted acquisitions')
        previous = now
    return dict(verified_ticks=len(ticks), maximum_state_difference=maximum,
                maximum_covariance_difference=covariance_difference, accepted_headings=len(headings),
                compensated_fixes=delayed_gnss, replayed_heading_corrections=replayed_heading,
                replayed_gnss_corrections=replayed_gnss, heading_health_counts=health_counts)


def verify_scenario(actual, declared):
    recorded = copy.deepcopy(actual)
    recorded.setdefault('heading_latency_compensation', False)
    if not contains_declared(recorded, declared): raise ValueError('scenario differs from frozen acquisition protocol')


def audit(run, config, ticks, spec, frozen):
    verify_scenario(run['scenario'], frozen)
    if {'heading_sensor', 'sensor_timing', 'objects', 'truth', 'initial_heading_offset_rad'} & config.keys():
        raise ValueError('simulator labels or sensor schedule entered operational calibration')
    if (config.get('heading_latency_compensation', False) != spec['compensated']
            or not config.get('gnss_latency_compensation') or not config.get('absolute_heading_fusion')):
        raise ValueError('operational fusion policy differs from frozen input')
    first, second = config['route']['points'][:2]
    initial = config['initial_pose']
    if initial['position'] != first or abs(angle(initial['yaw']-math.atan2(second['y']-first['y'], second['x']-first['x']))) > 1e-10:
        raise ValueError('physical startup heading entered operational calibration')
    frames, summary = run['frames'], run['summary']
    if not frames or len(frames) != len(ticks) or len(ticks) != summary['steps']:
        raise ValueError('incomplete physical/control evidence')
    errors, yaws = [], []
    sensor = frozen['heading_sensor']
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected']; now = i*.05
        if inp['time'] != frame['time'] or out['time'] != frame['time'] or abs(frame['time']-now) > 1e-8 or out['estimate'] != frame['estimate']:
            raise ValueError('physical/control clocks differ')
        acquired = i-sensor['delay_ticks']; stamp = acquired*.05
        due = acquired >= 0 and acquired % sensor['period_ticks'] == 0
        due &= not any(stamp+1e-9 >= w['from'] and stamp < w['until']-1e-9 for w in sensor.get('dropout_windows', []))
        sample = inp.get('absolute_heading')
        if bool(sample) != due: raise ValueError('heading acquisition/delivery schedule differs')
        if sample:
            bias = next((w['offset_rad'] for w in sensor.get('bias_windows', []) if stamp+1e-9 >= w['from'] and stamp < w['until']-1e-9), 0.)
            yaw = frames[acquired]['truth']['pose']['yaw']
            if abs(sample['stamp']-stamp) > 1e-9 or sample['variance_rad2'] != sensor['variance_rad2'] or abs(angle(sample['yaw_rad']-yaw-bias)) > math.sqrt(3*sensor['variance_rad2'])+1e-12:
                raise ValueError('delivered heading changed acquired value or timestamp')
        gnss_acquired = i-spec['delay_ticks']; stamp = gnss_acquired*.05
        due = gnss_acquired >= 0 and gnss_acquired % 4 == 0
        due &= not any(stamp+1e-9 >= w['from'] and stamp < w['until']-1e-9 for w in frozen.get('gnss_dropout_windows', []))
        fix = inp.get('gnss')
        if bool(fix) != due or (fix and abs(fix['stamp']-stamp) > 1e-9): raise ValueError('GNSS acquisition/delivery schedule differs')
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x', 'y']], [out['estimate']['pose']['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(frame['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
    oracle = reconstruct(config, ticks)
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('global physical errors were removed')
    clearance, road = swept_clearance(run), road_margin(run)
    strict = []
    if spec['mode'] == 'later-gnss-replay' and oracle['replayed_gnss_corrections'] == 0:
        strict.append('later_gnss_replay_not_exercised')
    if not summary['passed']: strict.append('source_acceptance')
    if rmse > .3 or maximum > .6 or max(yaws) > .1: strict.append('localization_accuracy')
    if clearance['minimum_swept_clearance_m'] < 1. or road['minimum_actual_road_margin_m'] < .1 or summary['collisions'] or summary['road_violations']:
        strict.append('physical_clearance_road_reserve')
    if not summary['reached_goal'] or summary['simulated_seconds'] > 45.: strict.append('goal_deadline')
    negative = not spec['compensated'] or spec['kind'] == 'unsupported-delay'
    failed = list(strict)
    hold = None
    if negative:
        failed = []
        if (oracle['accepted_headings'] or summary['reached_goal'] or max(abs(f['truth']['speed']) for f in frames) > .05
                or any(not t['expected']['emergency'] or t['expected']['command']['acceleration'] > -5.99 for t in ticks)
                or 'goal_deadline' not in strict):
            failed.append('negative_control_standstill')
    elif spec['kind'] != 'base':
        lo, hi = (6., 6.9) if spec['kind'] == 'gnss-loss' else (5.5, 5.9)
        hold = max(f['truth']['speed'] for f in frames if lo <= f['time'] <= hi+1e-9)
        pre = max(f['truth']['speed'] for f in frames if 3. <= f['time'] < 4.)
        resumed = max(f['truth']['speed'] for f in frames if 9. <= f['time'] <= 12.)
        if hold > .05 or pre < 1. or resumed < 1.:
            strict.append('actual_fault_stop_and_recovery')
            failed.append('actual_fault_stop_and_recovery')
    return dict(expected_run_exit_code=0 if summary['passed'] else 1, failed_gates=failed,
                strict_failed_gates=strict, strict_passed=not strict, negative_control=negative,
                oracle=oracle, rmse_m=rmse, maximum_error_m=maximum, maximum_yaw_rad=max(yaws),
                independent_clearance=clearance, road_containment=road,
                maximum_actual_fault_hold_speed_m_s=hold)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--replay-composition', action='store_true', help='Two additional fixed 10 Hz heading cases requiring later-GNSS replay')
    suffix = '.exe' if os.name == 'nt' else ''
    parser.add_argument('--binary', type=Path, default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary', type=Path, default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args = parser.parse_args(); output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze_inputs(output, specifications(plants, args.replay_composition), factory=materialize, gates=GATES)
    if args.freeze_only: return
    validate_inputs(output, manifest, args.binary.resolve())
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    rows = []
    report = dict(complete=False, manifest_sha256=sha(output/'frozen-manifest.json'), gates=GATES,
                  binaries={p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}, episodes=rows)
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda s: run_case(output, s, binaries[s['plant']], args.binary.resolve(), auditor=audit), manifest['episodes']):
            rows.append(row)
            report.update(complete=len(rows) == len(manifest['episodes']))
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(row['plant'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    report.update(passed=all(r['passed'] for r in rows), positive_strict_passes=sum(r.get('strict_passed', False) for r in rows if not r.get('negative_control', True)),
                  negative_controls=sum(r.get('negative_control', False) for r in rows))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    raise SystemExit(0 if report['complete'] and report['passed'] else 1)


if __name__ == '__main__': main()
