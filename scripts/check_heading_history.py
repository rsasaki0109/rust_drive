#!/usr/bin/env python3
"""Reconstruct combined GNSS/heading fusion from complete acquired inputs."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
from pathlib import Path

from check_absolute_heading import HeadingFilter, verify_covariance
from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_road_width import road_margin, swept_clearance, validate_inputs

REPO = Path(__file__).resolve().parents[1]
GATES = dict(rmse_m=.3, maximum_error_m=.6, maximum_yaw_rad=.1,
             minimum_clearance_m=1., minimum_road_margin_m=.1,
             maximum_heading_age_s=.35, maximum_gnss_age_s=.75,
             maximum_history_s=.5, maximum_heading_events=64,
             minimum_paired_rmse_reduction=.5, original_deadline_s=45.)
GNSS_FIELDS = ['accepted', 'rejected', 'observed', 'accepted_stamp', 'nis', 'decision']


def chronological(config, motions, fixes, headings, through, priority=3):
    """Rebuild the entire measured past without bounded producer snapshots.

    Only originally admitted observations reenter the fold. Heading conditioning
    uses the dense Gaussian formula; the Rust filter uses Joseph covariance.
    Equal-time GNSS precedes heading. No evaluation truth enters this function.
    """
    events = [(g['stamp'], 1, g) for g in fixes] + [(h['stamp'], 2, h) for h in headings]
    events = sorted((e for e in events if e[0] < through-1e-9
                     or (abs(e[0]-through) <= 1e-9 and e[1] < priority)), key=lambda e: (e[0], e[1]))
    result, index = HeadingFilter(config), 0

    def apply(event):
        stamp, kind, sample = event
        if kind == 1:
            before = result.accepted
            result.correct(sample)
            if result.accepted != before+1:
                raise ValueError('rewind conflicts with an accepted GNSS observation')
        else:
            before = result.heading['accepted_samples']
            result.observe_heading(sample, stamp)
            if result.heading['accepted_samples'] != before+1:
                raise ValueError('rewind conflicts with an accepted heading observation')

    while index < len(events) and events[index][0] <= 1e-9:
        apply(events[index]); index += 1
    for start, end, motion in motions:
        if start >= through-1e-9:
            break
        end, cursor = min(end, through), start
        while index < len(events) and events[index][0] <= end+1e-9:
            stamp = events[index][0]
            if stamp < cursor-1e-9:
                raise ValueError('observation is outside acquired motion coverage')
            result.predict(motion, max(0., stamp-cursor))
            apply(events[index]); cursor = stamp; index += 1
        result.predict(motion, max(0., end-cursor))
    if index != len(events):
        raise ValueError('future observation entered chronological reconstruction')
    return result


def reconstruct(config, ticks):
    if not config.get('absolute_heading_fusion') or config.get('localization_map') or config.get('relocalization'):
        raise ValueError('this physical oracle requires GNSS/heading in one epoch without map input')
    compensated = config.get('gnss_latency_compensation', False)
    oracle = HeadingFilter(config)
    motions, fixes, headings = [], [], []
    previous = None
    maximum = covariance_difference = 0.
    replayed = delayed = 0
    health_counts = {}
    for tick in ticks:
        inp, out = tick['input'], tick['expected']; now = inp['time']
        if previous is not None:
            wheel = inp.get('odometry')
            if wheel is None or now-wheel['stamp'] > .15+1e-9 or now-previous > .25+1e-9:
                raise ValueError('combined oracle requires continuously acquired motion')
            motions.append((previous, now, wheel)); oracle.predict(wheel, now-previous)
        fix = inp.get('gnss')
        if fix is not None:
            if not 0. <= now-fix['stamp'] <= .5+1e-9:
                raise ValueError('GNSS delivery is outside the declared history')
            if compensated:
                prior = chronological(config, motions, fixes, headings, fix['stamp'], priority=1)
                for key in GNSS_FIELDS: setattr(prior, key, getattr(oracle, key))
                accepted_before = prior.accepted; prior.correct(fix)
                if prior.accepted > accepted_before:
                    candidate = chronological(config, motions, fixes+[fix], headings, now)
                    for key in GNSS_FIELDS: setattr(candidate, key, getattr(prior, key))
                    candidate.heading = copy.deepcopy(oracle.heading)
                    candidate.heading_fault = oracle.heading_fault
                    oracle = candidate; fixes.append(fix)
                    delayed += now-fix['stamp'] > 1e-9
                    replayed += sum(h['stamp'] >= fix['stamp']-1e-9 for h in headings)
                else:
                    for key in GNSS_FIELDS: setattr(oracle, key, getattr(prior, key))
            else:
                oracle.correct(fix)
        sample = inp.get('absolute_heading')
        before = oracle.heading['accepted_samples']
        issues = oracle.observe_heading(sample, now)
        if oracle.heading['accepted_samples'] > before: headings.append(sample)
        pose = out['estimate']['pose']
        difference = max(abs(pose['position'][k]-oracle.x[i]) for i, k in enumerate(['x', 'y']))
        difference = max(difference, abs(angle(pose['yaw']-oracle.x[2])), abs(out['estimate']['speed']-oracle.speed))
        maximum = max(maximum, difference)
        if difference > 1e-8: raise ValueError(f'combined posterior state differs at {now}: {difference}')
        covariance_difference = max(covariance_difference, verify_covariance(out.get('pose_covariance'), oracle.p))
        if abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8:
            raise ValueError('combined covariance trace differs')
        actual = out.get('absolute_heading')
        if actual is None: raise ValueError('missing heading diagnostics')
        for key, value in oracle.heading.items():
            if key == 'last_nis' and value is not None:
                if actual[key] is None or not math.isclose(actual[key], value, rel_tol=1e-8, abs_tol=1e-8):
                    raise ValueError('heading innovation changed ownership on rewind')
            elif actual[key] != value: raise ValueError('heading clock/count/decision differs: '+key)
        actual_gnss = out['localization']
        for key, value in [('last_observed_stamp', oracle.observed), ('last_accepted_stamp', oracle.accepted_stamp),
                           ('last_decision', oracle.decision), ('accepted_fixes', oracle.accepted), ('rejected_fixes', oracle.rejected)]:
            if actual_gnss[key] != value: raise ValueError('GNSS clock/count/decision differs: '+key)
        if oracle.nis is not None and not math.isclose(actual_gnss['last_nis'], oracle.nis, rel_tol=1e-8, abs_tol=1e-8):
            raise ValueError('GNSS innovation differs from its acquisition prior')
        observed_issues = [issue for issue in out['health'] if issue in ['InvalidAbsoluteHeading', 'StaleAbsoluteHeading', 'HeadingInnovationHold']]
        if issues != observed_issues: raise ValueError('heading freshness/fault was changed by rewind')
        if issues and out['command']['acceleration'] > -5.99: raise ValueError('required heading fault did not brake')
        for issue in issues: health_counts[issue] = health_counts.get(issue, 0)+1
        stale_gnss = oracle.accepted_stamp is None or now-oracle.accepted_stamp > .75+1e-9
        if ('StaleGnss' in out['health']) != stale_gnss: raise ValueError('heading changed GNSS freshness')
        if stale_gnss and out['command']['acceleration'] > -5.99: raise ValueError('expired GNSS did not brake')
        latency = out.get('gnss_latency')
        if compensated:
            if (latency is None or latency['compensated_fixes'] != delayed
                    or latency.get('replayed_heading_corrections', 0) != replayed
                    or latency.get('history_heading_corrections', 0) > 64):
                raise ValueError('heading history diagnostics differ from acquired events')
        elif latency is not None: raise ValueError('disabled history emitted compensation diagnostics')
        previous = now
    return dict(verified_ticks=len(ticks), maximum_state_difference=maximum,
                maximum_covariance_difference=covariance_difference,
                accepted_headings=len(headings), compensated_fixes=delayed,
                replayed_heading_corrections=replayed, heading_health_counts=health_counts)


def specifications(plants):
    rows = [dict(plant=p, world='curve', seed=s, delay_ticks=d, compensated=c,
                 kind='base', mode=f'delay-{d}-'+('history' if c else 'naive'))
            for p in plants for s in [203, 211] for d in [3, 7] for c in [False, True]]
    rows += [dict(plant=p, world='curve', seed=223, delay_ticks=7, compensated=True,
                  kind=k, mode=k) for p in plants for k in ['heading-loss', 'heading-jump', 'gnss-loss']]
    return rows


def materialize(spec):
    source = 'scenarios/gnss-delay-curve.json'
    value = json.loads((REPO/source).read_text())
    value.update(name='Combined heading/GNSS '+spec['mode'], full_rate_recording=True,
                 absolute_heading_fusion=True, gnss_latency_compensation=spec['compensated'],
                 heading_sensor=dict(period_ticks=4, variance_rad2=1e-4),
                 local_route_geometry=True,
                 motion_limits=dict(max_acceleration_m_s2=2., max_deceleration_m_s2=2.5, max_lateral_acceleration_m_s2=2.5),
                 initial_heading_offset_rad=(-1. if spec['seed'] == 203 else 1.)*math.pi/60.)
    value['sensor_timing']['gnss_delay_ticks'] = spec['delay_ticks']
    value['objects'] = [dict(s=60., lateral=10., radius=.5)]
    if spec['kind'] == 'heading-loss': value['heading_sensor']['dropout_windows'] = [dict(**{'from':4.}, until=6.)]
    if spec['kind'] == 'heading-jump': value['heading_sensor']['bias_windows'] = [dict(**{'from':4.}, until=6., offset_rad=.8)]
    if spec['kind'] == 'gnss-loss': value['gnss_dropout_windows'] = [dict(**{'from':4.}, until=7.)]
    return source, value


def verify_scenario(actual, declared):
    recorded = dict(actual)
    # Serde omits this flag when false. An omitted true flag must still fail.
    recorded.setdefault('gnss_latency_compensation', False)
    if not contains_declared(recorded, declared):
        raise ValueError('physical scenario differs from frozen input')


def audit(run, config, ticks, spec, frozen):
    verify_scenario(run['scenario'], frozen)
    if {'heading_sensor', 'sensor_timing', 'objects', 'truth', 'initial_heading_offset_rad'} & config.keys():
        raise ValueError('simulation schedule or truth entered operational configuration')
    if config.get('gnss_latency_compensation', False) != spec['compensated'] or not config.get('absolute_heading_fusion'):
        raise ValueError('recorded fusion policy differs from declared input')
    if config.get('relocalization') or config.get('localization_map'): raise ValueError('undeclared map/reset observation')
    first, second = config['route']['points'][:2]
    initial = config['initial_pose']
    if initial['position'] != first or abs(angle(initial['yaw']-math.atan2(second['y']-first['y'], second['x']-first['x']))) > 1e-10:
        raise ValueError('physical startup heading entered operational calibration')
    frames, summary = run['frames'], run['summary']
    if len(frames) != len(ticks) or len(ticks) != summary['steps']: raise ValueError('incomplete physical/control evidence')
    errors, yaws = [], []
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected']; now = i*.05
        if inp['time'] != frame['time'] or out['time'] != frame['time'] or abs(frame['time']-now) > 1e-8 or out['estimate'] != frame['estimate']:
            raise ValueError('physical/control timing differs')
        sensor = frozen['heading_sensor']
        available = i % 4 == 0 and not any(now+1e-9 >= w['from'] and now < w['until']-1e-9 for w in sensor.get('dropout_windows', []))
        sample = inp.get('absolute_heading')
        if bool(sample) != available: raise ValueError('heading acquisition/dropout differs')
        if sample:
            bias = next((w['offset_rad'] for w in sensor.get('bias_windows', []) if now+1e-9 >= w['from'] and now < w['until']-1e-9), 0.)
            if sample['stamp'] != inp['time'] or sample['variance_rad2'] != 1e-4 or abs(angle(sample['yaw_rad']-frame['truth']['pose']['yaw']-bias)) > math.sqrt(3e-4)+1e-12:
                raise ValueError('heading is not the declared independently noisy acquisition')
        acquisition = (i-spec['delay_ticks'])*.05
        due = i >= spec['delay_ticks'] and (i-spec['delay_ticks']) % 4 == 0
        due &= not any(acquisition+1e-9 >= w['from'] and acquisition < w['until']-1e-9 for w in frozen.get('gnss_dropout_windows', []))
        fix = inp.get('gnss')
        if bool(fix) != due or (fix and abs(fix['stamp']-acquisition) > 1e-9): raise ValueError('GNSS acquisition/delivery schedule differs')
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x', 'y']], [out['estimate']['pose']['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(frame['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
    oracle = reconstruct(config, ticks)
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('physical accuracy summary differs')
    clearance, road = swept_clearance(run), road_margin(run)
    failed = []
    if not summary['passed']: failed.append('source_acceptance')
    if rmse > .3 or maximum > .6 or max(yaws) > .1: failed.append('localization_accuracy')
    if clearance['minimum_swept_clearance_m'] < 1. or road['minimum_actual_road_margin_m'] < .1 or summary['collisions'] or summary['road_violations']:
        failed.append('physical_clearance_road_reserve')
    if not summary['reached_goal'] or summary['simulated_seconds'] > 45.: failed.append('goal_deadline')
    hold = None
    if spec['kind'] != 'base':
        lo, hi = (6., 6.9) if spec['kind'] == 'gnss-loss' else (5.5, 5.9)
        hold = max(f['truth']['speed'] for f in frames if lo <= f['time'] <= hi+1e-9)
        pre_fault = max(f['truth']['speed'] for f in frames if 3. <= f['time'] < 4.)
        resumed = max(f['truth']['speed'] for f in frames if 9. <= f['time'] <= 12.)
        if hold > .05 or pre_fault < 1. or resumed < 1.: failed.append('actual_fault_stop_and_recovery')
    strict = not failed
    # Naive controls retain every strict failure; their role is a comparison.
    return dict(expected_run_exit_code=0 if summary['passed'] else 1,
                failed_gates=failed if spec['compensated'] else [], strict_failed_gates=failed,
                strict_passed=strict, oracle=oracle, rmse_m=rmse, maximum_error_m=maximum,
                maximum_yaw_rad=max(yaws), independent_clearance=clearance,
                road_containment=road, maximum_actual_fault_hold_speed_m_s=hold)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release/rustdriving')
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args(); output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze_inputs(output, specifications(plants), factory=materialize, gates=GATES)
    if args.freeze_only: return
    validate_inputs(output, manifest, args.binary.resolve())
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    rows = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda s: run_case(output, s, binaries[s['plant']], args.binary.resolve(), auditor=audit), manifest['episodes']):
            rows.append(row)
            report = dict(complete=len(rows) == len(manifest['episodes']), binaries=provenance,
                          manifest_sha256=sha(output/'frozen-manifest.json'), gates=GATES, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(row['plant'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('strict_failed_gates')), flush=True)
    pairs = []
    for row in rows:
        if row['kind'] != 'base' or not row['compensated']: continue
        baseline = next(r for r in rows if r['plant'] == row['plant'] and r['seed'] == row['seed'] and r['delay_ticks'] == row['delay_ticks'] and not r['compensated'])
        reduction = 1.-row.get('rmse_m', math.inf)/baseline.get('rmse_m', math.inf)
        pairs.append(dict(plant=row['plant'], seed=row['seed'], delay_ticks=row['delay_ticks'], rmse_reduction=reduction,
                          passed=row['passed'] and baseline['passed'] and reduction >= .5))
    report.update(pairs=pairs, passed=all(r['passed'] for r in rows) and all(p['passed'] for p in pairs))
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    raise SystemExit(0 if report['complete'] and report['passed'] else 1)


if __name__ == '__main__': main()
