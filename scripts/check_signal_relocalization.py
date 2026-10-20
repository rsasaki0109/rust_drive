#!/usr/bin/env python3
"""Independently reconstruct fixed-route signal permission across measured resets."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path

from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_gnss_recovery import POLICY as GNSS_POLICY
from check_lane_change_speeds import sweep
from check_navigation_relocalization import projection, scenario_matches
from check_relocalization import REPO, materialize as regional_materialize, verify_epochs
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_stop_relocalization import physical_stops, verify_stops

GATES = dict(duration_s=65., cruise_m_s=4., clearance_m=1., signal_margin_m=1.,
             valid_rmse_m=.2, valid_maximum_error_m=.4, valid_yaw_rad=.1,
             recovery_window_s=[11., 15.], permanent_red_hold_after_s=30.,
             disabled_hold_after_s=12., stationary_speed_m_s=.05)
MODES = ['red-green', 'stale-green', 'red-hold', 'after-passed', 'mixed-stop', 'healthy', 'disabled']


def specifications(plants, smoke=False):
    # Each state/fault composition remains in CI; these are authored seed-7 cases.
    return [dict(plant=p, world=w, mode=m, seed=7) for p in plants
            for w in ['straight', 'curve'] for m in MODES]


def materialize(spec):
    source, value = regional_materialize(dict(spec, mode='recover'))
    mode = spec['mode'];permanent = mode == 'red-hold'
    value.update(name=f"Measured signal recovery {spec['world']} {mode}", duration=65., cruise_speed=4.,
                 gnss_recovery=copy.deepcopy(GNSS_POLICY), expected='fault' if permanent or mode == 'disabled' else 'goal')
    value['relocalization'].update(map_only_grace_s=5., confirm_gnss_handoff=True, preserve_traffic_signals=True)
    phases = [dict(**{'from': 0., 'color': 'Green' if mode == 'stale-green' else 'Red'})]
    if not permanent and mode != 'stale-green':phases.append(dict(**{'from': 22., 'color': 'Green'}))
    value['traffic_signals'] = [dict(stop_line=dict(id='main', route_s_m=55.), phases=phases)]
    if mode == 'after-passed':
        value['traffic_signals'] = [dict(stop_line=dict(id='behind', route_s_m=20.), phases=[dict(**{'from':0., 'color':'Green'}), dict(**{'from':10., 'color':'Red'})]),
                                    dict(stop_line=dict(id='main', route_s_m=80.), phases=phases)]
    if mode == 'stale-green':value['signal_dropout_windows'] = [dict(**{'from':8., 'until':22.})]
    if mode == 'mixed-stop':
        value['stop_signs'] = [dict(id='stop', route_s_m=35.)]
        value['relocalization']['preserve_stop_signs'] = True
    value['sensor_timing']['odometry_dropout_windows'] = [] if mode == 'healthy' else [dict(**{'from':8., 'until':11.})]
    value['gnss_dropout_windows'] = [] if mode == 'healthy' else [dict(**{'from':8., 'until':16.})]
    if mode == 'disabled':value.pop('relocalization')
    return source, value


def freeze(output, specs):
    return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def verify_signals(config, ticks, reset_stamp=None):
    """Fold raw snapshots, measured poses and the previous semantic ownership.

    Reported colors, passage flags and reset admission never seed this fold.
    """
    lines = config['stop_lines'];route = config['route'];radius = config['vehicle']['radius']
    passed = {line['id']:False for line in lines};last = accepted = fault = reset = cutoff = None
    owners = None;records = [];accepted_count = 0;post_reset_first = None
    for tick in ticks:
        inp, out = tick['input'], tick['expected'];now = inp['time'];ego = out['estimate']
        distance, progress, _, yaw = projection(route['points'], ego['pose']['position'])
        yaw_error = abs(angle(ego['pose']['yaw']-yaw))
        if reset_stamp is not None and now == reset_stamp:
            owners = copy.deepcopy(passed);cutoff = now;last = accepted = fault = None
            reset = dict(stamp=now, progress_s_m=progress, distance_to_route_m=distance, yaw_error_rad=yaw_error,
                         admitted=abs(ego['speed']) <= .05 and distance+radius+.2 < route['half_width'] and yaw_error < .2,
                         retained_ids=[], rearmed_ids=[], unserved_too_close_ids=[])
            for line in lines:
                id = line['id'];margin = line['route_s_m']-progress-radius
                if passed[id]:
                    if margin <= 0.:
                        reset['retained_ids'].append(id);continue
                    reset['rearmed_ids'].append(id)
                passed[id] = False
                if margin < .5:
                    reset['admitted'] = False;reset['unserved_too_close_ids'].append(id)
        snapshot = inp.get('traffic_signal')
        if snapshot is not None:
            stamp = snapshot['stamp'];states = snapshot['states']
            valid = (math.isfinite(stamp) and 0. <= stamp <= now+1e-9 and len(states) == len(lines)
                     and all(sum(s['id'] == line['id'] for s in states) == 1 for line in lines))
            if not valid:fault = now
            elif ((last is None or stamp > last) and (cutoff is None or stamp > cutoff)
                  and (fault is None or (stamp > fault and now-stamp <= .5+1e-9))):
                last = stamp;accepted = {s['id']:s['color'] for s in states};fault = None;accepted_count += 1
                if reset is not None and post_reset_first is None:post_reset_first = stamp
        fresh = fault is None and last is not None and now-last <= .5+1e-9
        colors = {line['id']:accepted[line['id']] if fresh else 'Unknown' for line in lines}
        rejected = reset is not None and not reset['admitted']
        # Signal faults are handled by this fold. An independent stop-duty
        # admission fault also prevents passage, even before final actuation.
        healthy = not [h for h in out['health'] if h not in ['InvalidTrafficSignal', 'InvalidTrafficSignalRecovery']]
        for line in lines:
            if healthy and not rejected and colors[line['id']] == 'Green' and progress+radius >= line['route_s_m']:passed[line['id']] = True
        blocked = [line['route_s_m']-radius-1. for line in lines if not passed[line['id']] and (colors[line['id']] != 'Green' or rejected)]
        endpoint = min(blocked) if blocked else None;status = out['traffic_controls']
        expected = dict(last_accepted_stamp=last, fault=fault is not None,
                        signals=[dict(id=line['id'], color=colors[line['id']], passed=passed[line['id']]) for line in lines], stop_s_m=endpoint)
        if any(status[k] != v for k, v in expected.items()):raise ValueError('independent signal permission/passage/clock differs')
        observed_reset = status.get('localization_reset')
        if observed_reset is None or reset is None:
            if observed_reset != reset:raise ValueError('unproved signal reset')
        else:
            if set(observed_reset) != set(reset):raise ValueError('signal reset schema differs')
            for key, value in reset.items():
                if isinstance(value, float):
                    if abs(observed_reset[key]-value) > 1e-8:raise ValueError('signal reset geometry differs')
                elif observed_reset[key] != value:raise ValueError('signal reset ownership/admission differs')
        for active, issue in [(fault is not None, 'InvalidTrafficSignal'), (rejected, 'InvalidTrafficSignalRecovery')]:
            if active and (issue not in out['health'] or out['command']['acceleration'] > -5.99):raise ValueError('signal fault did not retain emergency braking')
        records.append(dict(stamp=now, colors=colors, last_accepted_stamp=last, passed=copy.deepcopy(passed)))
    return dict(passed=True, verified_ticks=len(ticks), accepted_snapshots=accepted_count, reset=reset,
                reset_ownership=owners, first_post_reset_acquisition_s=post_reset_first, records=records)


def phase(spec, now):
    return next(p['color'] for p in reversed(spec['phases']) if p['from'] <= now+1e-9)


def physical_signals(run, records):
    """Use actual fronts and external infrastructure phases, separate from driver claims."""
    specs = run['scenario']['traffic_signals'];radius = run['vehicle']['radius'];points = run['route']['points']
    previous = None;truth_margin = observed_margin = None;crossings = [];violations = 0
    for frame, record in zip(run['frames'], records):
        now = frame['time'];front = projection(points, frame['truth']['pose']['position'])[1]+radius
        if record['stamp'] != now:raise ValueError('physical/snapshot evidence clocks differ')
        for spec in specs:
            id = spec['stop_line']['id'];line = spec['stop_line']['route_s_m'];color = record['colors'][id]
            before = previous is None or previous < line
            if before and phase(spec, now) != 'Green':truth_margin = line-front if truth_margin is None else min(truth_margin, line-front)
            if before and color != 'Green':observed_margin = line-front if observed_margin is None else min(observed_margin, line-front)
            if before and front >= line:
                violation = phase(spec, now) != 'Green';violations += violation
                if violation or color != 'Green':raise ValueError('actual front crossed without true and newly observed green')
                crossings.append(dict(id=id, stamp=now, accepted_stamp=record['last_accepted_stamp']))
        previous = front
    summary = run['summary']
    if summary.get('signal_violations', 0) != violations or ((truth_margin is None) != (summary.get('signal_min_stopline_margin_m') is None)):
        raise ValueError('independent signal rule count/margin differs')
    if truth_margin is not None and abs(summary['signal_min_stopline_margin_m']-truth_margin) > 1e-8:raise ValueError('independent signal rule margin differs')
    return dict(violations=violations, minimum_true_nonpermissive_margin_m=truth_margin,
                minimum_observed_nonpermissive_margin_m=observed_margin, crossings=crossings)


def audit(run, config, ticks, spec, frozen):
    if not scenario_matches(run['scenario'], frozen):raise ValueError('frozen signal recovery scenario changed')
    if config['stop_lines'] != [s['stop_line'] for s in frozen['traffic_signals']] or config.get('relocalization') != frozen.get('relocalization'):raise ValueError('signal map/recovery policy changed')
    if config.get('navigation') or config.get('yield_intersections') or {'objects', 'sensor_timing', 'gnss_dropout_windows', 'traffic_signals', 'phases', 'signal_dropout_windows'} & config.keys():raise ValueError('simulation truth/schedule or unsupported controls entered pipeline config')
    if config['localization_map']['points'] != frozen['localization_map']['points'] or config['gnss_recovery'] != frozen['gnss_recovery']:raise ValueError('surveyed map/GNSS policy changed')
    frames = run['frames'];summary = run['summary'];mode = spec['mode'];healthy = mode == 'healthy'
    if len(frames) != len(ticks) or len(ticks) != summary['steps'] or run['route']['points'] != config['route']['points']:raise ValueError('incomplete/changed physical control route')
    errors = [];yaws = [];delay = frozen['sensor_timing']['gnss_delay_ticks']
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected'];now = inp['time']
        if abs(now-i*.05) > 1e-8 or frame['time'] != now or frame['estimate'] != out['estimate'] or frame['traffic_controls'] != out['traffic_controls']:raise ValueError('physical/control clock or telemetry differs')
        dropped = not healthy and 8.-1e-9 <= now < 11.-1e-9
        if bool(inp.get('odometry')) == dropped or (inp.get('odometry') and inp['odometry']['stamp'] != now):raise ValueError('motion acquisition schedule changed')
        acquisition = (i-delay)*.05;due = i >= delay and (i-delay)%4 == 0 and (healthy or not 8.-1e-9 <= acquisition < 16.-1e-9)
        if bool(inp.get('gnss')) != due or (due and abs(inp['gnss']['stamp']-acquisition) > 1e-9):raise ValueError('GNSS acquisition schedule changed')
        if bool(inp.get('lidar')) != (i%2 == 0) or (inp.get('lidar') and inp['lidar']['stamp'] != now) or inp['lidar_failed']:raise ValueError('map scan schedule changed')
        missing_signal = mode == 'stale-green' and 8.-1e-9 <= now < 22.-1e-9;signal_due = i%4 == 0 and not missing_signal
        snapshot = inp.get('traffic_signal')
        if bool(snapshot) != signal_due:raise ValueError('signal acquisition/loss schedule changed')
        if snapshot and (snapshot['stamp'] != now or snapshot['states'] != [dict(id=s['stop_line']['id'], color=phase(s, now)) for s in frozen['traffic_signals']]):raise ValueError('signal acquisition differs from infrastructure')
        truth, pose = frame['truth']['pose'], out['estimate']['pose']
        errors.append(math.dist([truth['position'][k] for k in ['x', 'y']], [pose['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(truth['yaw']-pose['yaw'])))
    epochs = verify_epochs(config, ticks);valid = epochs.pop('valid_indices');reset = epochs['reset_stamp']
    signals = verify_signals(config, ticks, reset);physical = physical_signals(run, signals.pop('records'))
    stop_oracle = stops = None
    if config.get('stop_signs'):
        if config['stop_signs'] != frozen['stop_signs']:raise ValueError('mixed stop map changed')
        stop_oracle = verify_stops(config, ticks, reset);stops = physical_stops(run)
    clearance = sweep(run);maximum = max(errors);rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
    valid_rmse = math.sqrt(sum(errors[i]**2 for i in valid)/len(valid));valid_max = max(errors[i] for i in valid);valid_yaw = max(yaws[i] for i in valid)
    if abs(summary['localization_rmse']-rmse) > 1e-10 or abs(summary['localization_max_error']-maximum) > 1e-10:raise ValueError('invalid epoch errors removed from global scores')
    explained = [f'localization max error {maximum:.3f} m exceeds 1 m'] if maximum > 1. and valid_max <= 1. else []
    failed = []
    if summary['failures'] != explained or summary['passed'] != (not explained):failed.append('source_acceptance')
    if valid_rmse > .2 or valid_max > .4 or valid_yaw > .1:failed.append('valid_epoch_accuracy')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance'] < 1. or summary['simulated_seconds'] > 65.:failed.append('physical_clearance_deadline')
    if any(m is not None and m < 1. for m in [physical['minimum_true_nonpermissive_margin_m'], physical['minimum_observed_nonpermissive_margin_m']]):failed.append('physical_signal_margin')
    goal = mode not in ['disabled', 'red-hold'];expected_crossings = len(frozen['traffic_signals']) if goal else 0
    if summary['reached_goal'] != goal or len(physical['crossings']) != expected_crossings:failed.append('original_goal_and_crossings')
    if healthy:
        if reset is not None:failed.append('healthy_no_reset')
    elif mode == 'disabled':
        if reset is not None or max(abs(f['truth']['speed']) for f in frames if f['time'] >= 12.) > .05:failed.append('disabled_stop_hold')
    else:
        if reset is None or not 11. <= reset <= 15.:failed.append('measured_stopped_recovery')
        elif min(abs(f['truth']['speed']) for f in frames if 10.5 <= f['time'] <= reset) > .05:failed.append('physical_stop_before_reset')
        elif mode == 'after-passed' and signals['reset']['retained_ids'] != ['behind']:failed.append('passed_signal_retained')
        if reset is not None and mode == 'stale-green' and (signals['first_post_reset_acquisition_s'] != 22. or physical['crossings'][0]['stamp'] <= 22.):failed.append('fresh_permission_required_after_loss')
    if mode == 'red-hold' and max(abs(f['truth']['speed']) for f in frames if f['time'] >= 30.) > .05:failed.append('permanent_red_standstill')
    if stops and (stops['violations'] or stops['minimum_unreleased_margin_m'] < 1. or len(stops['crossings']) != 1):failed.append('mixed_stop_compliance')
    return dict(oracle=epochs, signal_oracle=signals, physical_signals=physical, stop_oracle=stop_oracle, physical_stops=stops,
                independent_clearance=clearance, total_ticks=len(ticks), invalid_pose_ticks=len(ticks)-len(valid),
                global_rmse_m=rmse, global_maximum_error_m=maximum, valid_epoch_rmse_m=valid_rmse,
                valid_epoch_maximum_error_m=valid_max, valid_epoch_maximum_yaw_rad=valid_yaw,
                source_accuracy_failures_retained=explained, expected_run_exit_code=1 if explained else 0, failed_gates=failed)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all');parser.add_argument('--smoke', action='store_true');parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'));parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args();output = args.output.resolve();plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:print('Frozen signal recovery episodes:', len(manifest['episodes']));return
    binaries = {'reference-2d':args.binary.resolve(), 'rne-dynamic':args.native_binary.resolve()};provenance = {p:dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants};rows = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda spec:run_case(output, spec, binaries[spec['plant']], args.binary.resolve(), auditor=audit), manifest['episodes']):
            rows.append(row);report = dict(passed=all(r['passed'] for r in rows), complete=len(rows) == len(manifest['episodes']), manifest_sha256=sha(output/'frozen-manifest.json'), binaries=provenance, gates=GATES, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n');print(row['plant'], row['world'], row['mode'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':main()
