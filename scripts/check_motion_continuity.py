#!/usr/bin/env python3
"""Freeze motion-loss trials; independently check coverage, held state and physics."""
import argparse
import json
import math
import os
from pathlib import Path

from check_gnss_antenna import archive
from check_gnss_covariance import contains_declared
from check_gnss_latency import verify_ticks as verify_healthy
from check_gnss_map_history import sha
from check_lane_change_speeds import sweep
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
GATES = dict(clearance_m=1., rmse_m=.2, maximum_error_m=.4, stop_window_s=[10.5, 11.2],
             maximum_stop_speed_m_s=.05, all_stopped_after_s=12., fresh_after_s=13.)


def specifications(plants, smoke=False):
    rows = []
    for plant in plants:
        for world in ['straight', 'curve']:
            for delay in [0, 2]:
                for seed in ([7] if smoke else [1, 7, 42]):
                    rows.append(dict(plant=plant, world=world, lidar_delay_ticks=delay, seed=seed, mode='dropout'))
                rows.append(dict(plant=plant, world=world, lidar_delay_ticks=delay, seed=7, mode='healthy'))
        for delay in [0, 2]:
            rows.append(dict(plant=plant, world='straight', lidar_delay_ticks=delay, seed=7, mode='delayed-motion'))
    return rows


def materialize(spec):
    source = f"scenarios/gnss-delay-{spec['world']}.json"
    value = json.loads((REPO/source).read_text())
    value['name'] = f"Motion continuity {spec['world']} {spec['mode']} lidar-delay-{spec['lidar_delay_ticks']}"
    value['half_width'] = 5.5
    value['objects'] = [dict(s=45., lateral=4., radius=.5), dict(s=70., lateral=-4., radius=.5)]
    value['sensor_timing']['lidar_delay_ticks'] = spec['lidar_delay_ticks']
    if spec['mode'] != 'healthy':
        value['expected'] = 'fault'
    if spec['mode'] == 'dropout':
        value['sensor_timing']['odometry_dropout_windows'] = [{'from': 8., 'until': 11.}]
    if spec['mode'] == 'delayed-motion':
        value['sensor_timing']['odometry_delay_ticks'] = 4
    return source, value


def freeze(output, specs):
    manifest = dict(gates=GATES, episodes=[])
    for index, spec in enumerate(specs):
        source, value = materialize(spec)
        path = output/'frozen'/f'{index:03d}.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        content = json.dumps(value, indent=2)+'\n'
        if path.exists():
            if path.read_text() != content:
                raise ValueError('frozen motion scenario changed')
        else:
            with path.open('x', newline='\n') as stream:
                stream.write(content)
        manifest['episodes'].append(dict(spec, scenario=str(path.relative_to(output)), sha256=sha(path),
                                        source=source, source_sha256=sha(REPO/source)))
    path = output/'frozen-manifest.json'
    if path.exists():
        if json.loads(path.read_text()) != manifest:
            raise ValueError('frozen motion cohort changed')
    else:
        with path.open('x', newline='\n') as stream:
            stream.write(json.dumps(manifest, indent=2)+'\n')
    return manifest


def verify_continuity(config, ticks):
    """Derive missing coverage from raw motion/clocks, before reading latch outputs."""
    previous, wheel, gap = None, None, None
    for index, tick in enumerate(ticks):
        inp = tick['input']
        now, motion = inp['time'], inp.get('odometry')
        if motion is not None:
            if not (0. <= motion['stamp'] <= now and -.1 <= motion['speed'] <= 20.
                    and abs(motion['yaw_rate']) <= 3.):
                raise ValueError('physical cohort must supply finite valid motion acquisitions')
            if wheel is None or motion['stamp'] > wheel['stamp']:
                wheel = motion
        if previous is not None:
            cause = ('ClockGap' if now-previous > .25 else 'MissingOdometry' if wheel is None
                     else 'StaleOdometry' if now-wheel['stamp'] > .15+1e-9 else None)
            if cause is not None:
                gap = dict(index=index, last_covered_time=previous, detected_at=now,
                           last_accepted_odometry_stamp=wheel['stamp'] if wheel else None, cause=cause)
                break
        previous = now
    prefix = ticks[:gap['index']] if gap else ticks
    oracle = verify_healthy(config, prefix)
    if any(t['expected'].get('motion_discontinuity') is not None for t in prefix):
        raise ValueError('false discontinuity in independently covered motion')
    if gap is None:
        return dict(passed=True, prefix_oracle=oracle, discontinuity=None, held_ticks=0)
    prior = prefix[-1]['expected']
    diagnostic = {k: v for k, v in gap.items() if k != 'index'}
    for tick in ticks[gap['index']:]:
        out = tick['expected']
        if out.get('motion_discontinuity') != diagnostic or 'MotionDiscontinuity' not in out['health']:
            raise ValueError('missing, changed or prematurely cleared discontinuity epoch')
        if not out['emergency'] or out['command'] != dict(acceleration=-6., steering=0.):
            raise ValueError('uncovered motion resumed control')
        for key in ['estimate', 'tracking_frame', 'position_variance', 'pose_covariance', 'localization']:
            if out.get(key) != prior.get(key):
                raise ValueError(f'uncovered state/freshness changed: {key}')
        if out['tracks'] or out['predictions']:
            raise ValueError('scene evidence bridged missing body motion')
        if (out.get('gnss_latency') or {}).get('history_predictions', 0):
            raise ValueError('prediction history bridged an uncovered interval')
        if (out.get('map_localization') or {}).get('gnss_outage_covered', False):
            raise ValueError('map claimed coverage after missing motion')
    return dict(passed=True, prefix_oracle=oracle, discontinuity=gap, held_ticks=len(ticks)-gap['index'])


def source_contract(summary, mode, maximum, valid_maximum, discontinuity):
    """Retain the CLI's accuracy failure for an explicitly invalid pose epoch.

    The frozen fault protocol qualifies stopping, not a current pose. The
    general CLI still applies its 1m position gate to every logged estimate.
    Only that exact independently scored failure is explainable here; valid
    pose errors, collisions and every other source failure still reject.
    """
    expected = []
    if mode != 'healthy' and discontinuity and maximum > 1. and valid_maximum <= 1.:
        expected = [f'localization max error {maximum:.3f} m exceeds 1 m']
    return dict(passed=summary['failures'] == expected and summary['passed'] == (not expected),
                expected_run_exit_code=1 if expected else 0,
                explained_invalid_pose_failures=expected)


def audit(run, config, ticks, spec, frozen):
    if not contains_declared(run['scenario'], frozen):
        raise ValueError('physical scenario differs from frozen authored inputs')
    if {'sensor_timing', 'truth', 'objects', 'odometry_dropout_windows', 'odometry_delay_ticks'} & config.keys():
        raise ValueError('simulation transport/truth leaked into operational calibration')
    frames, summary = run['frames'], run['summary']
    if len(frames) != len(ticks) or len(frames) != summary['steps'] or not frames:
        raise ValueError('missing full physical/control coverage')
    timing = frozen['sensor_timing']
    delay = timing.get('odometry_delay_ticks', 0)
    errors, deliveries, fresh = [], 0, 0
    for index, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected']
        if abs(inp['time']-index*.05) > 1e-8 or frame['time'] != inp['time'] or frame['estimate'] != out['estimate']:
            raise ValueError('physical/control acquisition timeline changed')
        acquired = (index-delay)*.05
        dropout = any(acquired+1e-9 >= w['from'] and acquired < w['until']-1e-9
                      for w in timing.get('odometry_dropout_windows', []))
        expected = index >= delay and not dropout
        motion = inp.get('odometry')
        if bool(motion) != expected or (motion and abs(motion['stamp']-acquired) > 1e-9):
            raise ValueError('motion acquisition/delivery schedule differs')
        deliveries += bool(motion)
        fresh += (inp['time'] > GATES['fresh_after_s'] and motion is not None
                  and inp['time']-motion['stamp'] <= .15+1e-9 and inp.get('gnss') is not None
                  and inp.get('lidar') is not None)
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x', 'y']],
                                [frame['estimate']['pose']['position'][k] for k in ['x', 'y']]))
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('full-interval truth error score changed')
    oracle, clearance = verify_continuity(config, ticks), sweep(run)
    valid_maximum = max(errors[:oracle['discontinuity']['index']] if oracle['discontinuity'] else errors)
    source = source_contract(summary, spec['mode'], maximum, valid_maximum, oracle['discontinuity'])
    failed = []
    if not source['passed']:
        failed.append('source_acceptance')
    if (summary['collisions'] or summary['road_violations'] or summary['min_clearance'] < GATES['clearance_m']
            or summary['simulated_seconds'] > frozen['duration']):
        failed.append('physical_clearance_deadline')
    stop = None
    if spec['mode'] == 'healthy':
        if oracle['discontinuity'] or not summary['reached_goal'] or rmse > GATES['rmse_m'] or maximum > GATES['maximum_error_m']:
            failed.append('healthy_goal_accuracy')
    else:
        window = [f['truth']['speed'] for f in frames if GATES['stop_window_s'][0] <= f['time'] < GATES['stop_window_s'][1]]
        later = [f['truth']['speed'] for f in frames if f['time'] >= GATES['all_stopped_after_s']]
        stop = dict(minimum_window_speed_m_s=min(window) if window else None,
                    maximum_later_speed_m_s=max(later) if later else None,
                    fresh_joint_deliveries_after_13s=fresh)
        if (not oracle['discontinuity'] or summary['reached_goal'] or not window or not later
                or min(window) > GATES['maximum_stop_speed_m_s'] or max(later) > GATES['maximum_stop_speed_m_s']
                or (spec['mode'] == 'dropout' and fresh == 0)):
            failed.append('latched_physical_stop')
        if (spec['mode'] == 'dropout' and oracle['discontinuity']
                and not any(t['expected']['tracks'] for t in ticks[:oracle['discontinuity']['index']])):
            failed.append('missing_pre_fault_scene_witness')
    return dict(oracle=oracle, motion_deliveries_verified=deliveries, rmse_m=rmse, maximum_error_m=maximum,
                valid_prefix_maximum_error_m=valid_maximum, source_contract=source,
                localization_accuracy_qualified=spec['mode'] == 'healthy', independent_clearance=clearance,
                fault_stop=stop, failed_gates=failed)


def run_case(output, spec, binary, replay_binary):
    name = f"{spec['world']}-{spec['mode']}-lidar-{spec['lidar_delay_ticks']}-seed-{spec['seed']}"
    folder = output/'episodes'/spec['plant']/name
    folder.mkdir(parents=True, exist_ok=False)
    source = output/spec['scenario']
    if sha(source) != spec['sha256']:
        raise ValueError('frozen motion scenario changed')
    args = ['run', '--scenario', source] if spec['plant'] == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = dict(spec, passed=False, recordings=str(folder.relative_to(output)))
    try:
        row['run_exit_code'] = execute(binary, args+['--seed', spec['seed'], '--output', folder/'run'], folder/'run-process')
        row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream:
            config = json.loads(next(stream))['header']['config']
            ticks = [r['tick'] for line in stream if 'tick' in (r := json.loads(line))]
        row.update(audit(run, config, ticks, spec, json.loads(source.read_text())))
        row['passed'] = (row['run_exit_code'] == row['source_contract']['expected_run_exit_code']
                         and row['replay_exit_code'] == 0 and not row['failed_gates']
                         and row['replay']['verified'] and row['replay']['ticks'] == len(ticks))
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
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    output = args.output.resolve()
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:
        print('Frozen motion-continuity episodes:', len(manifest['episodes']))
        return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    rows = []
    for spec in manifest['episodes']:
        rows.append(run_case(output, spec, binaries[spec['plant']], args.binary.resolve()))
        report = dict(passed=all(r['passed'] for r in rows), complete=len(rows) == len(manifest['episodes']),
                      binaries=provenance, gates=GATES, manifest_sha256=sha(output/'frozen-manifest.json'), episodes=rows)
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        row = rows[-1]
        print(row['plant'], row['world'], row['mode'], row['lidar_delay_ticks'], row['seed'], row['passed'],
              row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':
    main()
