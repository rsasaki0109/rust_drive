#!/usr/bin/env python3
"""Freeze measured yield recovery and independently fold entry ownership and scans."""
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
from check_hazards import prediction_radius_at, segment_rectangle_distance_squared
from check_lane_change_speeds import sweep
from check_navigation_relocalization import projection, scenario_matches
from check_relocalization import REPO, materialize as regional_materialize, verify_epochs
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_signal_relocalization import physical_signals, phase, verify_signals
from check_stop_relocalization import physical_stops, verify_stops

GATES = dict(duration_s=95., clearance_m=1., priority_gap_s=2., waiting_margin_m=1.,
             valid_rmse_m=.2, valid_max_error_m=.4, valid_yaw_rad=.1,
             reset_delay_s=[0., 4.], hold_after_s=40., hold_speed_m_s=.05)
MODES = ['clear', 'priority', 'blocked', 'committed', 'passed', 'mixed', 'healthy', 'disabled']


def specifications(plants, smoke=False):
    return [dict(plant=p, world='straight', mode=m, seed=s,
                 **({'revision':'clear-landmarks-v2' if m == 'committed' else 'clear-landmarks-v3'} if m in ['committed', 'passed'] else {})) for p in plants
            for s in ([7] if smoke else [7, 42]) for m in MODES]


def loss_window(mode, revision=None):
    if mode == 'committed' and revision == 'clear-landmarks-v2':return [14., 17.]
    return [11., 14.] if mode == 'committed' else [16., 19.] if mode == 'passed' else [8., 11.]


def materialize(spec):
    source, value = regional_materialize(dict(spec, mode='recover'))
    mode = spec['mode'];start, end = loss_window(mode, spec.get('revision'))
    if spec.get('revision') in ['clear-landmarks-v2', 'clear-landmarks-v3']:
        # Viewed fixture repair: full-width conflict geometry and every physical
        # gate remain fixed. Surveyed roadside circles must not permanently
        # occupy the conservative crossing forecast. Retain the original runs.
        for i, obj in enumerate(value['objects']):
            selected = [22, 30, 83] if spec['revision'] == 'clear-landmarks-v3' else [22, 30]
            if obj['s'] not in selected:continue
            lateral = math.copysign(7.5, obj['lateral']);shift = lateral-obj['lateral'];obj['lateral'] = lateral
            for point in value['localization_map']['points'][i*360:(i+1)*360]:point['y'] += shift
    value.update(name=f"Measured yield recovery {mode}", duration=95., cruise_speed=4.,
                 gnss_recovery=copy.deepcopy(GNSS_POLICY), expected='fault' if mode in ['blocked', 'disabled'] else 'goal')
    value['relocalization'].update(map_only_grace_s=5., confirm_gnss_handoff=True, preserve_yield_intersections=True)
    def zone(id, line, lo, hi):
        return dict(stop_line=dict(id=id, route_s_m=line),
                    conflict_bounds=dict(min=dict(x=lo, y=-3.5), max=dict(x=hi, y=3.5)), exit_s_m=hi)
    value['yield_intersections'] = [zone('main', 55., 62., 68.)]
    if mode == 'committed':value['yield_intersections'] = [zone('main', 20., 27., 43.)]
    if mode == 'passed':value['yield_intersections'] = [zone('behind', 10., 17., 23.), zone('main', 70., 77., 83.)]
    if mode in ['priority', 'mixed']:
        value['objects'].append(dict(s=65., lateral=35., radius=1., lateral_speed=-2.5, moving_from=12.))
    if mode == 'blocked':value['objects'].append(dict(s=65., lateral=0., radius=1.))
    if mode == 'mixed':
        value['stop_signs'] = [dict(id='stop', route_s_m=35.)]
        value['traffic_signals'] = [dict(stop_line=dict(id='signal', route_s_m=85.),
                                         phases=[dict(**{'from':0., 'color':'Red'}), dict(**{'from':40., 'color':'Green'})])]
        value['relocalization'].update(preserve_stop_signs=True, preserve_traffic_signals=True)
    value['sensor_timing']['odometry_dropout_windows'] = [] if mode == 'healthy' else [dict(**{'from':start, 'until':end})]
    value['gnss_dropout_windows'] = [] if mode == 'healthy' else [dict(**{'from':start, 'until':start+8.})]
    if mode == 'disabled':value.pop('relocalization')
    return source, value


def freeze(output, specs):
    return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def blockers_for(zone, predictions):
    """Use the separate segment/rectangle distance implementation, including tail segments."""
    ids = []
    for prediction in predictions:
        points = prediction['positions'];dt = prediction['dt'];bounds = zone['conflict_bounds']
        malformed = (not points or not math.isfinite(dt) or dt <= 0
                     or any(not all(math.isfinite(p[a]) for a in ['x', 'y']) for p in points))
        if malformed:ids.append(prediction['id']);continue
        segments = [(points[0], points[0], 0., 0.)]
        for i, (a, b) in enumerate(zip(points, points[1:])):
            start = i*dt
            if start >= 8.:break
            end = min(8., (i+1)*dt);f = (end-start)/dt
            segments.append((a, {k:a[k]+(b[k]-a[k])*f for k in ['x', 'y']}, start, end))
        for a, b, start, end in segments:
            inflation = max(prediction_radius_at(prediction, start), prediction_radius_at(prediction, end))+.5
            inflated = {side:{axis:bounds[side][axis]+sign*inflation for axis in ['x', 'y']}
                        for side, sign in [('min', -1), ('max', 1)]}
            if not math.isfinite(inflation) or segment_rectangle_distance_squared(a, b, inflated) <= 1e-12:
                ids.append(prediction['id']);break
    return ids


def verify_yields(config, ticks, reset_stamp=None):
    """Reconstruct every state from raw scan acquisitions, poses, health and forecasts.

    No reported phase, commitment, blocker, timer or reset diagnostic seeds the fold.
    """
    route = config['route'];radius = config['vehicle']['radius'];zones = config['yield_intersections']
    states = [dict(id=z['stop_line']['id'], phase='Waiting', blocking_tracks=[], clear_since=None, committed=False) for z in zones]
    counts = [0]*len(zones);last_scan = clear_after = reset = None
    records = [];owners = None;releases = [];accepted = 0
    for tick in ticks:
        inp, out = tick['input'], tick['expected'];now = inp['time'];ego = out['estimate']
        distance, progress, _, heading = projection(route['points'], ego['pose']['position'])
        yaw_error = abs(angle(ego['pose']['yaw']-heading))
        if now == reset_stamp:
            owners = copy.deepcopy(states);counts = [0]*len(zones)
            last_scan = max(now, last_scan) if last_scan is not None else now
            clear_after = max(now, clear_after) if clear_after is not None else now
            reset = dict(stamp=now, progress_s_m=progress, distance_to_route_m=distance, yaw_error_rad=yaw_error,
                         admitted=abs(ego['speed']) <= .05 and distance+radius+.2 < route['half_width'] and yaw_error < .2,
                         retained_committed_ids=[], retained_passed_ids=[], rearmed_ids=[], unserved_too_close_ids=[])
            for z, s in zip(zones, states):
                id = s['id'];margin = z['stop_line']['route_s_m']-progress-radius
                s.update(blocking_tracks=[], clear_since=None)
                if s['committed'] and margin <= 0.:
                    past = progress-radius >= z['exit_s_m'];s['phase'] = 'Passed' if past else 'Proceeding'
                    reset['retained_passed_ids' if past else 'retained_committed_ids'].append(id)
                else:
                    if s['committed'] or s['phase'] == 'Proceeding':reset['rearmed_ids'].append(id)
                    s.update(committed=False, phase='Waiting')
                    if margin < .5:reset['admitted'] = False;reset['unserved_too_close_ids'].append(id)
        rejected = reset is not None and not reset['admitted']
        healthy = not out['health'] and not rejected
        scan = inp.get('lidar');old_scan = last_scan
        scan_valid = (out.get('motion_discontinuity') is None and scan is not None and not inp['lidar_failed'] and math.isfinite(scan['stamp'])
                      and 0. <= scan['stamp'] <= now and now-scan['stamp'] <= .35+1e-9
                      and len(scan['points']) <= 20000
                      and all(all(math.isfinite(p[a]) for a in ['x', 'y']) and math.hypot(p['x'], p['y']) <= 200. for p in scan['points']))
        new = scan_valid and (last_scan is None or scan['stamp'] > last_scan)
        if new:last_scan = scan['stamp'];accepted += 1
        gap = new and old_scan is not None and last_scan-old_scan > .2+1e-9
        aligned = yaw_error < .2 and distance+radius <= route['half_width']
        fresh = healthy and aligned and last_scan is not None and 0. <= now-last_scan <= .15+1e-9
        if not fresh:clear_after = now
        for i, (z, s) in enumerate(zip(zones, states)):
            if s['phase'] == 'Passed':continue
            s['blocking_tracks'] = blockers_for(z, out['predictions'])
            if not s['committed']:
                before = s['phase']
                if not fresh or s['blocking_tracks']:s.update(phase='Waiting', clear_since=None);counts[i] = 0
                elif new and (clear_after is None or last_scan > clear_after):
                    if gap:s.update(phase='Waiting', clear_since=None);counts[i] = 0
                    if s['clear_since'] is None:s['clear_since'] = last_scan
                    counts[i] += 1
                    if counts[i] >= 3 and last_scan-s['clear_since']+1e-9 >= 1.:s['phase'] = 'Proceeding'
                if s['phase'] == 'Proceeding' and before != 'Proceeding':releases.append(dict(id=s['id'], stamp=now, first_acquisition_s=s['clear_since']))
                if s['phase'] == 'Proceeding' and fresh and progress+radius >= z['stop_line']['route_s_m']:s['committed'] = True
            if s['committed'] and healthy and aligned and progress-radius >= z['exit_s_m']:s['phase'] = 'Passed'
        blocked = [z['stop_line']['route_s_m']-radius-1. for z, s in zip(zones, states) if s['phase'] == 'Waiting']
        expected = dict(zones=states, stop_s_m=min(blocked) if blocked else None)
        observed = out['intersections']
        if any(observed[k] != value for k, value in expected.items()):raise ValueError('independent yield ownership/clear timer/forecast/endpoint differs')
        observed_reset = observed.get('localization_reset')
        if observed_reset is None or reset is None:
            if observed_reset != reset:raise ValueError('unproved yield reset')
        else:
            if set(observed_reset) != set(reset):raise ValueError('yield reset schema differs')
            for key, value in reset.items():
                if isinstance(value, float):
                    if abs(observed_reset[key]-value) > 1e-8:raise ValueError('yield reset geometry differs')
                elif observed_reset[key] != value:raise ValueError('yield reset ownership/admission differs')
        if ('InvalidIntersectionRecovery' in out['health']) != rejected:raise ValueError('yield recovery health differs from measured admission')
        if rejected and out['command']['acceleration'] > -5.99:raise ValueError('invalid yield reset released emergency braking')
        records.append(dict(stamp=now, zones=copy.deepcopy(states)))
    return dict(passed=True, verified_ticks=len(ticks), accepted_scans=accepted, reset=reset, reset_ownership=owners, releases=releases, records=records)


def physical_yields(run, records):
    """Score actual circular occupancy using Euclidean distance, independent of forecasts."""
    zones = run['scenario']['yield_intersections'];radius = run['vehicle']['radius'];points = run['route']['points']
    intervals = {z['stop_line']['id']:{} for z in zones};opened = {id:{} for id in intervals};crossed = set();crossings = [];margin = None
    for frame, record in zip(run['frames'], records):
        now = frame['time']
        if record['stamp'] != now:raise ValueError('physical/yield evidence clock differs')
        front = projection(points, frame['truth']['pose']['position'])[1]+radius
        bodies = {'ego':(frame['truth']['pose']['position'], radius)}
        bodies.update({str(o['id']):(o['position'], o['radius']) for o in frame['objects']})
        for z, s in zip(zones, record['zones']):
            id = z['stop_line']['id'];bounds = z['conflict_bounds'];line = z['stop_line']['route_s_m']
            active = {body for body, (p, r) in bodies.items() if sum(max(bounds['min'][a]-p[a], 0., p[a]-bounds['max'][a])**2 for a in ['x', 'y']) <= r*r}
            for body in active:opened[id].setdefault(body, now)
            for body in list(opened[id]):
                if body not in active:intervals[id].setdefault(body, []).append([opened[id].pop(body), now])
            if id not in crossed and s['phase'] == 'Waiting':margin = line-front if margin is None else min(margin, line-front)
            if id not in crossed and front >= line:
                if s['phase'] not in ['Proceeding', 'Passed']:raise ValueError('physical front crossed without reconstructed permission')
                crossed.add(id);crossings.append(dict(id=id, stamp=now))
    gaps = [];violations = 0
    for id, occupants in intervals.items():
        for body, start in opened[id].items():occupants.setdefault(body, []).append([start, run['frames'][-1]['time']])
        zone_gaps = []
        for a in occupants.get('ego', []):
            for body, values in occupants.items():
                if body == 'ego':continue
                for b in values:zone_gaps.append(b[0]-a[1] if a[1] <= b[0] else a[0]-b[1] if b[1] <= a[0] else -(min(a[1], b[1])-max(a[0], b[0])))
        gaps.extend(zone_gaps);violations += any(g+1e-9 < 2. for g in zone_gaps)
    gap = min(gaps) if gaps else None;reported = run['summary'].get('intersection_min_gap_s')
    if ((gap is None) != (reported is None) or (gap is not None and abs(gap-reported) > 1e-8)
            or run['summary'].get('intersection_violations', 0) != violations):raise ValueError('independent physical priority score differs')
    return dict(violations=violations, minimum_gap_s=gap, minimum_waiting_margin_m=margin, crossings=crossings, occupancies=intervals)


def audit(run, config, ticks, spec, frozen):
    if not scenario_matches(run['scenario'], frozen):raise ValueError('frozen yield recovery scenario changed')
    if config['yield_intersections'] != frozen['yield_intersections'] or config.get('relocalization') != frozen.get('relocalization'):raise ValueError('yield map/reset policy changed')
    if config.get('navigation') or {'objects', 'sensor_timing', 'gnss_dropout_windows', 'traffic_signals', 'phases'} & config.keys():raise ValueError('simulation truth/schedule entered pipeline configuration')
    if config['localization_map']['points'] != frozen['localization_map']['points'] or config['gnss_recovery'] != frozen['gnss_recovery']:raise ValueError('surveyed map/GNSS policy changed')
    frames = run['frames'];summary = run['summary'];mode = spec['mode'];healthy = mode == 'healthy';start, end = loss_window(mode, spec.get('revision'))
    if len(frames) != len(ticks) or len(ticks) != summary['steps'] or run['route']['points'] != config['route']['points']:raise ValueError('incomplete or changed physical/control route')
    errors = [];yaws = [];delay = frozen['sensor_timing']['gnss_delay_ticks']
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected'];now = inp['time']
        if abs(now-i*.05) > 1e-8 or frame['time'] != now or any(frame[k] != out[k] for k in ['estimate', 'intersections', 'command']):raise ValueError('physical/control alignment changed')
        dropped = not healthy and start-1e-9 <= now < end-1e-9
        if bool(inp.get('odometry')) == dropped or (inp.get('odometry') and inp['odometry']['stamp'] != now):raise ValueError('motion acquisition schedule changed')
        acquisition = (i-delay)*.05;due = i >= delay and (i-delay)%4 == 0 and (healthy or not start-1e-9 <= acquisition < start+8.-1e-9)
        if bool(inp.get('gnss')) != due or (due and abs(inp['gnss']['stamp']-acquisition) > 1e-9):raise ValueError('GNSS acquisition schedule changed')
        if bool(inp.get('lidar')) != (i%2 == 0) or (inp.get('lidar') and inp['lidar']['stamp'] != now) or inp['lidar_failed']:raise ValueError('scan acquisition schedule changed')
        snapshot = inp.get('traffic_signal');signals = frozen.get('traffic_signals', [])
        if bool(snapshot) != (bool(signals) and i%4 == 0):raise ValueError('signal acquisition schedule changed')
        if snapshot and (snapshot['stamp'] != now or snapshot['states'] != [dict(id=s['stop_line']['id'], color=phase(s, now)) for s in signals]):raise ValueError('signal acquisition differs')
        truth, pose = frame['truth']['pose'], out['estimate']['pose']
        errors.append(math.dist([truth['position'][a] for a in ['x', 'y']], [pose['position'][a] for a in ['x', 'y']]))
        yaws.append(abs(angle(truth['yaw']-pose['yaw'])))
    epochs = verify_epochs(config, ticks);valid = epochs.pop('valid_indices');reset = epochs['reset_stamp']
    yields = verify_yields(config, ticks, reset);physical = physical_yields(run, yields.pop('records'));clearance = sweep(run)
    stops = stop_oracle = signals = signal_oracle = None
    if mode == 'mixed':
        if config.get('stop_signs') != frozen['stop_signs'] or config.get('stop_lines') != [s['stop_line'] for s in frozen['traffic_signals']]:raise ValueError('mixed mapped controls changed')
        stop_oracle = verify_stops(config, ticks, reset);stops = physical_stops(run)
        signal_oracle = verify_signals(config, ticks, reset);signals = physical_signals(run, signal_oracle.pop('records'))
    rmse = math.sqrt(sum(e*e for e in errors)/len(errors));maximum = max(errors)
    valid_rmse = math.sqrt(sum(errors[i]**2 for i in valid)/len(valid));valid_max = max(errors[i] for i in valid);valid_yaw = max(yaws[i] for i in valid)
    if abs(summary['localization_rmse']-rmse) > 1e-10 or abs(summary['localization_max_error']-maximum) > 1e-10:raise ValueError('invalid-epoch errors removed from global scores')
    explained = [f'localization max error {maximum:.3f} m exceeds 1 m'] if maximum > 1. and valid_max <= 1. else []
    failed = []
    if summary['failures'] != explained or summary['passed'] != (not explained):failed.append('source_acceptance')
    if valid_rmse > .2 or valid_max > .4 or valid_yaw > .1:failed.append('valid_epoch_accuracy')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance'] < 1. or summary['simulated_seconds'] > 95.:failed.append('physical_clearance_deadline')
    if physical['violations'] or (physical['minimum_gap_s'] is not None and physical['minimum_gap_s']+1e-9 < 2.):failed.append('physical_priority_separation')
    if physical['minimum_waiting_margin_m'] is not None and physical['minimum_waiting_margin_m'] < 1.:failed.append('physical_waiting_margin')
    goal = mode not in ['blocked', 'disabled']
    if summary['reached_goal'] != goal or len(physical['crossings']) != (len(frozen['yield_intersections']) if goal else 0):failed.append('original_goal_and_crossings')
    if healthy:
        if reset is not None:failed.append('healthy_no_reset')
    elif mode == 'disabled':
        if reset is not None:failed.append('disabled_no_reset')
    elif reset is None or not end <= reset <= end+4.:failed.append('measured_stopped_recovery')
    else:
        if min(abs(f['truth']['speed']) for f in frames if end-.5 <= f['time'] <= reset) > .05:failed.append('physical_stop_before_reset')
        if mode == 'committed' and yields['reset']['retained_committed_ids'] != ['main']:failed.append('committed_entry_coverage')
        if mode == 'passed' and yields['reset']['retained_passed_ids'] != ['behind']:failed.append('passed_entry_coverage')
        if any(r['first_acquisition_s'] <= reset for r in yields['releases'] if r['stamp'] > reset):failed.append('old_clear_credit')
    if not goal and max(abs(f['truth']['speed']) for f in frames if f['time'] >= 40.) > .05:failed.append('negative_actual_stop_hold')
    if stops and (stops['violations'] or stops['minimum_unreleased_margin_m'] < 1. or len(stops['crossings']) != 1):failed.append('mixed_stop_compliance')
    if signals and (signals['violations'] or len(signals['crossings']) != 1 or signals['minimum_true_nonpermissive_margin_m'] < 1.):failed.append('mixed_signal_compliance')
    return dict(oracle=epochs, yield_oracle=yields, physical_yields=physical, physical_stops=stops, stop_oracle=stop_oracle,
                physical_signals=signals, signal_oracle=signal_oracle, independent_clearance=clearance, total_ticks=len(ticks),
                global_rmse_m=rmse, global_maximum_error_m=maximum, valid_epoch_rmse_m=valid_rmse,
                valid_epoch_maximum_error_m=valid_max, valid_epoch_maximum_yaw_rad=valid_yaw,
                source_accuracy_failures_retained=explained, expected_run_exit_code=1 if explained else 0, failed_gates=failed)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all');parser.add_argument('--smoke', action='store_true');parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args();output = args.output.resolve();plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:print('Frozen yield recovery episodes:', len(manifest['episodes']));return
    binaries = {'reference-2d':args.binary.resolve(), 'rne-dynamic':args.native_binary.resolve()};provenance = {p:dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants};rows = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda spec:run_case(output, spec, binaries[spec['plant']], args.binary.resolve(), auditor=audit), manifest['episodes']):
            rows.append(row);report = dict(passed=all(r['passed'] for r in rows), complete=len(rows) == len(manifest['episodes']), manifest_sha256=sha(output/'frozen-manifest.json'), binaries=provenance, gates=GATES, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n');print(row['plant'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':main()
