#!/usr/bin/env python3
"""Independently audit fixed-route stop obligations across measured recovery."""
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

GATES = dict(duration_s=65., cruise_m_s=4., clearance_m=1., valid_rmse_m=.2,
             valid_maximum_error_m=.4, valid_yaw_rad=.1, minimum_unreleased_margin_m=1.,
             healthy_hold_s=2., healthy_motion_age_s=.05, physical_hold_s=2.,
             recovery_after_motion_return_s=[0.,4.], disabled_hold_delay_s=4.)


def specifications(plants, smoke=False):
    rows = []
    for plant in plants:
        for world in ['straight', 'curve']:
            for seed in ([7] if smoke else [1, 7, 42]):
                rows.append(dict(plant=plant, world=world, seed=seed, mode='before-first'))
            for mode in ['during-hold', 'released', 'after-passed', 'healthy', 'disabled']:
                rows.append(dict(plant=plant, world=world, seed=7, mode=mode))
    return rows


def loss_time(mode):
    return {'during-hold': 11.2, 'released': 12.9, 'after-passed': 16.}.get(mode, 8.)


def materialize(spec):
    source, value = regional_materialize(dict(spec, mode='recover'))
    mode = spec['mode'];loss = loss_time(mode)
    lines = [35., 80.] if mode in ['after-passed', 'healthy'] else [35.] if mode in ['during-hold', 'released'] else [55.]
    value.update(name=f"Measured stop recovery {spec['world']} {mode}", duration=65., cruise_speed=4.,
                 stop_signs=[dict(id=f'stop-{i+1}', route_s_m=s) for i, s in enumerate(lines)],
                 gnss_recovery=copy.deepcopy(GNSS_POLICY), expected='fault' if mode == 'disabled' else 'goal')
    value['relocalization'].update(map_only_grace_s=5., confirm_gnss_handoff=True, preserve_stop_signs=True)
    value['sensor_timing']['odometry_dropout_windows'] = [] if mode == 'healthy' else [dict(**{'from':loss, 'until':loss+3.})]
    value['gnss_dropout_windows'] = [] if mode == 'healthy' else [dict(**{'from':loss, 'until':loss+8.})]
    if mode == 'disabled':value.pop('relocalization')
    return source, value


def freeze(output, specs):
    return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def verify_stops(config, ticks, reset_stamp=None):
    """Fold raw acquisitions and surveyed geometry; never seed from stop diagnostics."""
    lines = config['stop_signs'];route = config['route'];radius = config['vehicle']['radius']
    states = [dict(id=line['id'], phase='Approaching', held_s=0.) for line in lines]
    holds = [None for _ in lines];watermark = wheel = reset = None
    releases = [];reset_owner = None;verified_holds = 0
    for tick in ticks:
        inp, out = tick['input'], tick['expected'];now = inp['time'];ego = out['estimate']
        wheel = inp.get('odometry') or wheel
        distance, progress, lateral, yaw = projection(route['points'], ego['pose']['position'])
        yaw_error = abs(angle(ego['pose']['yaw']-yaw))
        if reset_stamp is not None and now == reset_stamp:
            reset_owner = copy.deepcopy(states)
            reset = dict(stamp=now, progress_s_m=progress, distance_to_route_m=distance,
                         yaw_error_rad=yaw_error, admitted=abs(ego['speed']) <= .05 and distance+radius+.2 < route['half_width'] and yaw_error < .2,
                         retained_ids=[], rearmed_ids=[], unserved_too_close_ids=[])
            watermark = now if watermark is None else max(watermark, now)
            holds = [None for _ in lines]
            for line, state in zip(lines, states):
                margin = line['route_s_m']-progress-radius
                if state['phase'] in ['Released', 'Passed']:
                    if margin <= 0.:
                        reset['retained_ids'].append(line['id']);continue
                    reset['rearmed_ids'].append(line['id'])
                state.update(phase='Approaching', held_s=0.)
                if margin < .5:
                    reset['admitted'] = False;reset['unserved_too_close_ids'].append(line['id'])
        healthy = not out['health']
        motion = wheel if wheel and now-wheel['stamp'] <= .05+1e-9 else None
        fresh = bool(healthy and motion and (watermark is None or motion['stamp'] > watermark))
        observed = motion['stamp'] if healthy and motion else now
        watermark = observed if watermark is None else max(watermark, observed)
        fault = reset is not None and not reset['admitted']
        if not fault:
            aligned = yaw_error < .2 and abs(lateral)+radius <= route['half_width']
            for i, (line, state) in enumerate(zip(lines, states)):
                margin = line['route_s_m']-progress-radius
                if state['phase'] == 'Released' and healthy and aligned and margin <= 0.:state['phase'] = 'Passed'
                if state['phase'] in ['Released', 'Passed']:continue
                stopped = healthy and aligned and .5 <= margin <= 3.5 and abs(ego['speed']) <= .05 and motion and abs(motion['speed']) <= .05
                if stopped:
                    if fresh:
                        if holds[i] and motion['stamp']-holds[i]['last'] > .05+1e-9:holds[i] = None
                        if holds[i] is None:holds[i] = dict(start=now, last=motion['stamp'])
                        holds[i]['last'] = motion['stamp'];state['held_s'] = max(0., holds[i]['last']-holds[i]['start'])
                        state['phase'] = 'Released' if state['held_s']+1e-9 >= 2. else 'Holding'
                        verified_holds += 1
                        if state['phase'] == 'Released':releases.append(dict(id=line['id'], stamp=now, start=holds[i]['start'], held_s=state['held_s'], after_reset=reset_stamp is not None and now > reset_stamp))
                else:
                    holds[i] = None;state.update(phase='Approaching', held_s=0.)
        pending = [line['route_s_m']-radius-1. for line, state in zip(lines, states) if state['phase'] not in ['Released', 'Passed']]
        endpoint = min(pending) if pending else None
        actual = out['stop_signs']
        if [s['id'] for s in actual['stops']] != [s['id'] for s in states]:raise ValueError('stop obligation identity changed')
        for a, expected in zip(actual['stops'], states):
            if a['phase'] != expected['phase'] or abs(a['held_s']-expected['held_s']) > 1e-8:raise ValueError('independent stop phase/dwell differs')
        if actual['stop_s_m'] != endpoint:raise ValueError('independent remaining stop endpoint differs')
        evidence = actual.get('localization_reset')
        if reset is None or evidence is None:
            if evidence != reset:raise ValueError('unproved stop reset')
        else:
            for key, value in reset.items():
                if isinstance(value, float):
                    if abs(evidence[key]-value) > 1e-8:raise ValueError('stop reset geometry differs')
                elif evidence[key] != value:raise ValueError('stop reset ownership/admission differs')
        if fault:
            if 'InvalidStopSignRecovery' not in out['health'] or out['command']['acceleration'] > -5.99:raise ValueError('inadmissible stop reset did not keep emergency braking')
        elif endpoint is not None and endpoint+1.-progress <= 3.5 and abs(ego['speed']) <= .2 and out['command']['acceleration'] > -.5+1e-9:
            raise ValueError('remaining stop did not retain braking')
    return dict(passed=True, verified_ticks=len(ticks), verified_healthy_hold_samples=verified_holds,
                reset_ownership=reset_owner, reset=reset, releases=releases, final_stops=states)


def physical_stops(run):
    """Reconstruct physical dwell/crossing without using driver state."""
    lines = run['scenario']['stop_signs'];points = run['route']['points'];radius = run['vehicle']['radius']
    states = {line['id']:dict(start=None, satisfied=False, crossed=False) for line in lines}
    minimum = None;violations = 0;crossings = []
    for frame in run['frames']:
        now = frame['time'];truth = frame['truth'];front = projection(points, truth['pose']['position'])[1]+radius
        for line in lines:
            state = states[line['id']];margin = line['route_s_m']-front
            if state['crossed']:continue
            if not state['satisfied']:
                minimum = margin if minimum is None else min(minimum, margin)
                if 0. <= margin <= 3.5 and abs(truth['speed']) <= .1:
                    if state['start'] is None:state['start'] = now
                    if now-state['start']+1e-9 >= 2.:state['satisfied'] = True
                else:state['start'] = None
            if margin <= 0.:
                state['crossed'] = True;violations += not state['satisfied']
                crossings.append(dict(id=line['id'], stamp=now, physical_hold_started_at=state['start'], satisfied=state['satisfied']))
    if violations != run['summary'].get('stop_sign_violations', 0) or abs(minimum-run['summary']['stop_sign_min_margin_m']) > 1e-8:raise ValueError('independent physical stop scoring differs from summary')
    return dict(violations=violations, minimum_unreleased_margin_m=minimum, crossings=crossings)


def physical_recovered_holds(run, releases):
    """Check a new physical dwell even when the sign was served before reset."""
    lines = {line['id']:line for line in run['scenario']['stop_signs']}
    points = run['route']['points'];radius = run['vehicle']['radius'];result = []
    for release in releases:
        if not release['after_reset']:continue
        frames = [f for f in run['frames'] if release['start']-1e-9 <= f['time'] <= release['stamp']+1e-9]
        duration = frames[-1]['time']-frames[0]['time'] if frames else 0.
        maximum_speed = max((abs(f['truth']['speed']) for f in frames), default=math.inf)
        margins = [lines[release['id']]['route_s_m']-projection(points,f['truth']['pose']['position'])[1]-radius for f in frames]
        if duration+1e-9 < 2. or maximum_speed > .1 or not margins or min(margins) < 0. or max(margins) > 3.5:
            raise ValueError('new release lacks an independent physical two-second standstill')
        result.append(dict(id=release['id'],start_s=frames[0]['time'],end_s=frames[-1]['time'],duration_s=duration,
                           maximum_speed_m_s=maximum_speed,minimum_margin_m=min(margins),maximum_margin_m=max(margins),samples=len(frames)))
    return result


def audit(run, config, ticks, spec, frozen):
    if not scenario_matches(run['scenario'], frozen):raise ValueError('frozen stop recovery scenario changed')
    if config['stop_signs'] != frozen['stop_signs'] or config.get('relocalization') != frozen.get('relocalization'):raise ValueError('stop recovery map/policy changed')
    if config.get('navigation') or config.get('stop_lines') or config.get('yield_intersections') or {'objects', 'sensor_timing', 'gnss_dropout_windows'} & config.keys():raise ValueError('unsupported control composition or simulation schedule entered config')
    if config['localization_map']['points'] != frozen['localization_map']['points'] or config['gnss_recovery'] != frozen['gnss_recovery']:raise ValueError('surveyed map or GNSS policy changed')
    frames = run['frames'];summary = run['summary'];mode = spec['mode'];loss = loss_time(mode);healthy = mode == 'healthy'
    if len(frames) != len(ticks) or len(ticks) != summary['steps'] or run['route']['points'] != config['route']['points']:raise ValueError('incomplete or changed physical/control route')
    errors = [];yaws = [];delay = frozen['sensor_timing']['gnss_delay_ticks']
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected'];now = inp['time']
        if abs(now-i*.05) > 1e-8 or frame['time'] != now or frame['estimate'] != out['estimate'] or frame['stop_signs'] != out['stop_signs']:raise ValueError('physical/observation alignment differs')
        dropped = not healthy and loss-1e-9 <= now < loss+3.-1e-9
        if bool(inp.get('odometry')) == dropped or (inp.get('odometry') and inp['odometry']['stamp'] != now):raise ValueError('motion loss schedule changed')
        acquisition = (i-delay)*.05;due = i >= delay and (i-delay)%4 == 0 and (healthy or not loss-1e-9 <= acquisition < loss+8.-1e-9)
        if bool(inp.get('gnss')) != due or (due and abs(inp['gnss']['stamp']-acquisition) > 1e-9):raise ValueError('GNSS acquisition schedule changed')
        if bool(inp.get('lidar')) != (i%2 == 0) or (inp.get('lidar') and inp['lidar']['stamp'] != now) or inp['lidar_failed']:raise ValueError('map scan schedule changed')
        truth, pose = frame['truth']['pose'], out['estimate']['pose'];errors.append(math.dist([truth['position'][k] for k in ['x', 'y']], [pose['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(truth['yaw']-pose['yaw'])))
    epochs = verify_epochs(config, ticks);valid = epochs.pop('valid_indices');reset = epochs['reset_stamp'];stops = verify_stops(config, ticks, reset)
    physical = physical_stops(run);new_holds = physical_recovered_holds(run, stops['releases'])
    clearance = sweep(run);maximum = max(errors);rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
    valid_rmse = math.sqrt(sum(errors[i]**2 for i in valid)/len(valid));valid_max = max(errors[i] for i in valid);valid_yaw = max(yaws[i] for i in valid)
    if abs(summary['localization_rmse']-rmse) > 1e-10 or abs(summary['localization_max_error']-maximum) > 1e-10:raise ValueError('invalid epoch errors removed from global scores')
    explained = [f'localization max error {maximum:.3f} m exceeds 1 m'] if maximum > 1. and valid_max <= 1. else []
    failed = []
    if summary['failures'] != explained or summary['passed'] != (not explained):failed.append('source_acceptance')
    if valid_rmse > .2 or valid_max > .4 or valid_yaw > .1:failed.append('valid_epoch_accuracy')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance'] < 1. or summary['simulated_seconds'] > 65.:failed.append('physical_clearance_deadline')
    if physical['violations'] or physical['minimum_unreleased_margin_m'] < 1.:failed.append('physical_stop_sign_compliance')
    goal = mode != 'disabled'
    if summary['reached_goal'] != goal or len(physical['crossings']) != (len(frozen['stop_signs']) if goal else 0):failed.append('original_goal_and_stop_crossings')
    if healthy:
        if reset is not None:failed.append('healthy_no_reset')
    elif mode == 'disabled':
        if reset is not None or max(abs(f['truth']['speed']) for f in frames if f['time'] >= loss+4.) > .05:failed.append('disabled_recovery_hold')
    else:
        if reset is None or not loss+3. <= reset <= loss+7.:failed.append('measured_stop_recovery')
        else:
            if min(abs(f['truth']['speed']) for f in frames if loss+2.5 <= f['time'] <= reset) > .05:failed.append('actual_stop_before_reset')
            original = next(t['expected']['stop_signs']['stops'][0] for t in reversed(ticks) if t['input']['time'] < loss-1e-9)
            if mode == 'during-hold' and (original['phase'] != 'Holding' or not .1 < original['held_s'] < 2.):failed.append('partial_hold_interruption_exercised')
            if mode == 'released' and (original['phase'] != 'Released' or stops['reset']['rearmed_ids'] != ['stop-1']):failed.append('released_stop_rearmed')
            if mode == 'after-passed' and (original['phase'] != 'Passed' or stops['reset']['retained_ids'] != ['stop-1']):failed.append('passed_stop_retained')
            if not any(r['after_reset'] and r['start'] > reset and r['stamp'] >= reset+2.-1e-9 for r in stops['releases']):failed.append('fresh_full_stop_after_reset')
    return dict(oracle=epochs, stop_oracle=stops, physical_stops=physical, physical_recovered_holds=new_holds, independent_clearance=clearance,
                total_ticks=len(ticks), invalid_pose_ticks=len(ticks)-len(valid), global_rmse_m=rmse, global_maximum_error_m=maximum,
                valid_epoch_rmse_m=valid_rmse, valid_epoch_maximum_error_m=valid_max, valid_epoch_maximum_yaw_rad=valid_yaw,
                source_accuracy_failures_retained=explained, expected_run_exit_code=1 if explained else 0, failed_gates=failed)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all');parser.add_argument('--smoke', action='store_true');parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'));parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args();output = args.output.resolve();plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant];manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:print('Frozen stop recovery episodes:', len(manifest['episodes']));return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()};provenance = {p:dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants};rows = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda spec:run_case(output, spec, binaries[spec['plant']], args.binary.resolve(), auditor=audit), manifest['episodes']):
            rows.append(row);report = dict(passed=all(e['passed'] for e in rows), complete=len(rows) == len(manifest['episodes']), manifest_sha256=sha(output/'frozen-manifest.json'), binaries=provenance, gates=GATES, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n');print(row['plant'], row['world'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':main()
