#!/usr/bin/env python3
"""Freeze and independently score bounded post-reset map-only driving and stops."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path

from check_gnss_antenna import archive
from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_lane_change_speeds import sweep
from check_relocalization import GATES as REGIONAL_GATES, REPO, materialize as regional_materialize, verify_epochs
from check_speed_zones import execute

GATES = dict(REGIONAL_GATES, grace_s=5., gnss_resume_acquisition_s=16., lidar_loss_s=14.,
             movement_window_s=[12.3, 16.], minimum_map_only_distance_m=5.,
             expiry_stop_margin_s=1.5, lidar_loss_brake_window_s=[14.1, 14.3],
             lidar_loss_hold_after_s=15.6, minimum_pre_fault_speed_m_s=1.)


def specifications(plants, smoke=False):
    result = []
    for plant in plants:
        for world in ['straight', 'curve']:
            for seed in ([7] if smoke else [1, 7, 42]):
                result.append(dict(plant=plant, world=world, seed=seed, mode='handoff'))
            for mode in ['default', 'expiry', 'lidar-loss']:
                result.append(dict(plant=plant, world=world, seed=7, mode=mode))
    return result


def materialize(spec):
    source, scenario = regional_materialize(dict(spec, mode='recover'))
    scenario['name'] = f"Post-reset map-only {spec['world']} {spec['mode']}"
    if spec['mode'] != 'default':
        scenario['relocalization']['map_only_grace_s'] = GATES['grace_s']
    if spec['mode'] in ['handoff', 'default']:
        scenario['gnss_dropout_windows'] = [{'from': 8., 'until': 16.}]
    else:
        scenario['gnss_dropout'] = 8.
        scenario['expected'] = 'fault'
    if spec['mode'] == 'lidar-loss':
        scenario['lidar_dropout'] = GATES['lidar_loss_s']
    return source, scenario


def freeze(output, specs, *, factory=materialize, gates=GATES):
    manifest = dict(gates=gates, episodes=[])
    for index, spec in enumerate(specs):
        source, scenario = factory(spec)
        path = output/'frozen'/f'{index:03d}.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        encoded = json.dumps(scenario, indent=2)+'\n'
        if path.exists():
            if path.read_text() != encoded:
                raise ValueError('frozen map-only scenario changed')
        else:
            with path.open('x', newline='\n') as stream:
                stream.write(encoded)
        manifest['episodes'].append(dict(spec, scenario=str(path.relative_to(output)), sha256=sha(path),
                                         source=source, source_sha256=sha(REPO/source)))
    path = output/'frozen-manifest.json'
    if path.exists():
        if json.loads(path.read_text()) != manifest:
            raise ValueError('frozen map-only cohort changed')
    else:
        with path.open('x', newline='\n') as stream:
            stream.write(json.dumps(manifest, indent=2)+'\n')
    return manifest


def audit(run, config, ticks, spec, frozen):
    if not contains_declared(run['scenario'], frozen):
        raise ValueError('map-only scenario differs from frozen input')
    if {'truth', 'objects', 'sensor_timing', 'gnss_dropout', 'gnss_dropout_windows'} & config.keys():
        raise ValueError('simulation schedule/truth entered the operational configuration')
    if config.get('relocalization') != frozen['relocalization'] or config['localization_map']['points'] != frozen['localization_map']['points']:
        raise ValueError('map-only calibration/map differs from frozen input')
    frames, summary = run['frames'], run['summary']
    if len(frames) != len(ticks) or len(ticks) != summary['steps']:
        raise ValueError('incomplete physical/control coverage')
    errors, yaws = [], []
    delay = frozen['sensor_timing']['gnss_delay_ticks']
    for i, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        if abs(now-i*.05) > 1e-8 or frame['time'] != now or frame['estimate'] != out['estimate']:
            raise ValueError('physical/control alignment changed')
        dropout = 8.-1e-9 <= now < 11.-1e-9
        if bool(inp.get('odometry')) == dropout or (inp.get('odometry') and inp['odometry']['stamp'] != now):
            raise ValueError('measured motion acquisition schedule changed')
        acquisition = (i-delay)*.05
        available = acquisition < 8.-1e-9 or (spec['mode'] in ['handoff', 'default'] and acquisition >= 16.-1e-9)
        fix_due = i >= delay and (i-delay) % 4 == 0 and available
        if bool(inp.get('gnss')) != fix_due or (fix_due and abs(inp['gnss']['stamp']-acquisition) > 1e-9):
            raise ValueError('original GNSS acquisition/delivery schedule changed')
        scan_due = i % 2 == 0 and (spec['mode'] != 'lidar-loss' or now < 14.-1e-9)
        if bool(inp.get('lidar')) != scan_due or (scan_due and inp['lidar']['stamp'] != now) or inp['lidar_failed']:
            raise ValueError('original synchronous scan/loss schedule changed')
        truth, estimate = frame['truth']['pose'], out['estimate']['pose']
        errors.append(math.dist([truth['position'][k] for k in ['x', 'y']], [estimate['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(truth['yaw']-estimate['yaw'])))
    oracle = verify_epochs(config, ticks)
    valid = oracle.pop('valid_indices')
    rmse = math.sqrt(sum(e*e for e in errors)/len(errors))
    maximum = max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('global invalid-epoch errors were removed')
    valid_rmse = math.sqrt(sum(errors[i]**2 for i in valid)/len(valid))
    valid_max, valid_yaw = max(errors[i] for i in valid), max(yaws[i] for i in valid)
    reset = oracle['reset_stamp']
    failed, explained = [], []
    if maximum > 1. and valid_max <= 1.:
        explained = [f'localization max error {maximum:.3f} m exceeds 1 m']
    if summary['failures'] != explained or summary['passed'] != (not explained):
        failed.append('source_acceptance')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance'] < 1. or summary['simulated_seconds'] > 45.:
        failed.append('physical_clearance_deadline')
    clearance = sweep(run)
    if valid_rmse > .2 or valid_max > .4 or valid_yaw > .1:
        failed.append('valid_epoch_accuracy')
    if reset is None or not 11. <= reset <= 15. or oracle['searches'] < 3:
        raise ValueError('new epoch lacks a measured stopped recovery in the fixed window')
    if min(f['truth']['speed'] for f in frames if 10.5 <= f['time'] < 11.2) > .05:
        failed.append('physical_stop_before_reset')
    segment = [(f, t) for f, t in zip(frames, ticks) if f['time'] >= reset]
    first_fix = next((t['input']['time'] for _, t in segment if t['expected']['localization']['accepted_fixes']), None)
    for _, tick in segment:
        out = tick['expected']
        if first_fix is None or out['time'] < first_fix:
            if out['localization']['accepted_fixes'] or out['localization']['last_accepted_stamp'] is not None or out['localization']['last_observed_stamp'] is not None:
                failed.append('fabricated_gnss_freshness')
    window = [f for f in frames if 12.3 <= f['time'] <= 16.]
    distance = sum(math.dist([a['truth']['pose']['position'][k] for k in ['x', 'y']],
                             [b['truth']['pose']['position'][k] for k in ['x', 'y']]) for a, b in zip(window, window[1:]))
    first_brake = None
    if spec['mode'] in ['handoff', 'default']:
        if not summary['reached_goal'] or first_fix is None or not 16. <= first_fix <= 16.6:
            failed.append('actual_gnss_handoff_goal')
        if spec['mode'] == 'handoff' and (distance < 5. or oracle['post_epoch']['map_only_support_ticks'] < 60):
            failed.append('actual_map_only_movement')
        if spec['mode'] == 'default' and (distance > .1 or max(f['truth']['speed'] for f in window) > .05):
            failed.append('default_gnss_hold')
    else:
        pre_fault_speed = max(f['truth']['speed'] for f in frames if 12.3 <= f['time'] < 14.)
        if pre_fault_speed < GATES['minimum_pre_fault_speed_m_s']:
            failed.append('missing_actual_motion_before_stop')
        active = [t['input']['time'] for _, t in segment if (t['expected'].get('relocalization') or {}).get('map_only_support', {}).get('status') == 'Active']
        if not active:
            failed.append('missing_initial_map_support')
        first_brake = next((t['input']['time'] for _, t in segment if active and t['input']['time'] > active[0] and 'StaleGnss' in t['expected']['health']), None)
        if spec['mode'] == 'expiry':
            deadline = reset+5.
            if first_brake is None or not deadline < first_brake <= deadline+.051:
                failed.append('absolute_expiry_brake')
            hold = deadline+1.5
        else:
            if first_brake is None or not 14.1 < first_brake <= 14.3:
                failed.append('map_loss_brake')
            hold = 15.6
        if summary['reached_goal'] or first_fix is not None or max(f['truth']['speed'] for f in frames if f['time'] >= hold) > .05:
            failed.append('negative_physical_stop_hold')
    return dict(oracle=oracle, global_rmse_m=rmse, global_maximum_error_m=maximum,
                valid_epoch_rmse_m=valid_rmse, valid_epoch_maximum_error_m=valid_max,
                valid_epoch_maximum_yaw_rad=valid_yaw, invalid_pose_ticks=len(ticks)-len(valid),
                source_accuracy_failures_retained=explained, expected_run_exit_code=1 if explained else 0,
                independent_clearance=clearance, map_only_window_distance_m=distance,
                first_actual_gnss_acceptance_s=first_fix, first_post_support_brake_s=first_brake,
                failed_gates=failed)


def run_case(output, spec, binary, replay_binary, *, auditor=audit):
    folder = output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
    folder.mkdir(parents=True, exist_ok=False)
    source = output/spec['scenario']
    assert sha(source) == spec['sha256']
    row = dict(spec, passed=False, recordings=str(folder.relative_to(output)))
    args = ['run', '--scenario', source] if spec['plant'] == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    try:
        row['run_exit_code'] = execute(binary, args+['--seed', spec['seed'], '--output', folder/'run'], folder/'run-process')
        row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream:
            config = json.loads(next(stream))['header']['config']
            ticks = [r['tick'] for line in stream if 'tick' in (r := json.loads(line))]
        row.update(auditor(run, config, ticks, spec, json.loads(source.read_text())))
        row['passed'] = (row['run_exit_code'] == row['expected_run_exit_code'] and row['replay_exit_code'] == 0
                         and row['replay']['verified'] and row['replay']['ticks'] == len(ticks) and not row['failed_gates'])
    except Exception as error:
        row['audit_error'] = repr(error)
    finally:
        row['storage'] = archive(folder)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--workers', type=int, choices=[1, 2], default=2)
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args()
    output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:
        print('Frozen map-only episodes:', len(manifest['episodes']))
        return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    rows = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for row in pool.map(lambda s: run_case(output, s, binaries[s['plant']], args.binary.resolve()), manifest['episodes']):
            rows.append(row)
            report = dict(passed=all(r['passed'] for r in rows), complete=len(rows) == len(manifest['episodes']),
                          manifest_sha256=sha(output/'frozen-manifest.json'), binaries=provenance, gates=GATES, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(row['plant'], row['world'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':
    main()
