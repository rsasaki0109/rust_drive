#!/usr/bin/env python3
"""Freeze and independently audit correlated GNSS, faults and delayed map fusion."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path

from check_delayed_map import verify_ticks as verify_map_ticks
from check_gnss_latency import verify_ticks
from check_gnss_map_history import archive, sha
from check_lane_change_speeds import sweep
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
SHAPES = {'east_long': [[1.9, 0.], [0., .1]], 'diagonal_long': [[1., .9], [.9, 1.]]}
PRIOR = [[1., 0., 0.], [0., 1., 0.], [0., 0., (3.*math.pi/180.)**2/3.]]
WORLDS = ['straight', 'curve', 'north-bias', 'diagonal-bias', 'map-early', 'map-late']
GATES = {'rmse_m': .2, 'maximum_error_m': .4, 'clearance_m': 1.,
         'fault_window_s': [8., 11.], 'fault_norm_m': .8, 'stop_window_s': [9.5, 11.4],
         'maximum_stop_speed_m_s': .05, 'healthy_after_s': 13., 'paired_maximum_error_reduction': .4}


def scenario(world, calibrated=True):
    source = ('scenarios/delayed-map-gnss-'+('early' if world == 'map-early' else 'late')+'.json'
              if world.startswith('map-') else 'scenarios/gnss-delay-'+('curve' if world == 'curve' else 'straight')+'.json')
    result = json.loads((REPO/source).read_text())
    shape = SHAPES['east_long' if world in ['straight', 'north-bias', 'map-early'] else 'diagonal_long']
    result['initial_pose_covariance'] = copy.deepcopy(PRIOR)
    result['gnss_noise_shape'] = copy.deepcopy(shape)
    if calibrated:
        result['gnss_covariance_shape'] = copy.deepcopy(shape)
    if world in ['north-bias', 'diagonal-bias']:
        dx, dy = (0., .8) if world == 'north-bias' else (.8/math.sqrt(2.), -.8/math.sqrt(2.))
        result['gnss_bias_windows'] = [{'from': 8., 'until': 11., 'offset': {'x': dx, 'y': dy}}]
    result['name'] = f'GNSS covariance {world}: '+('calibrated' if calibrated else 'isotropic control')
    return source, result


def freeze(output, plants, seeds):
    """Write every case before executing any member of this cohort; refuse edits."""
    records = []
    for plant in plants:
        for world in WORLDS:
            for calibrated in ([True, False] if world.endswith('bias') else [True]):
                source, value = scenario(world, calibrated)
                name = world+('' if calibrated else '-isotropic')
                content = json.dumps(value, indent=2)+'\n'
                path = output/'frozen'/plant/(name+'.json')
                digest = hashlib.sha256(content.encode()).hexdigest()
                if path.exists():
                    if sha(path) != digest:
                        raise ValueError('frozen scenario changed: '+str(path))
                else:
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with path.open('x', newline='\n') as stream:
                        stream.write(content)
                for seed in seeds:
                    records.append({'plant': plant, 'world': world, 'calibrated': calibrated, 'seed': seed,
                                    'scenario': str(path.relative_to(output)), 'sha256': digest,
                                    'source': source, 'source_sha256': sha(REPO/source)})
    manifest = {'gates': GATES, 'shapes': SHAPES, 'initial_pose_covariance': PRIOR, 'episodes': records}
    path = output/'frozen-manifest.json'
    if path.exists():
        if json.loads(path.read_text()) != manifest:
            raise ValueError('cohort differs from frozen manifest')
    else:
        with path.open('x', newline='\n') as stream:
            stream.write(json.dumps(manifest, indent=2)+'\n')
    return manifest


def noise(seed, tick, shape):
    """Independent integer implementation of the declared simulation noise recipe."""
    def component(channel):
        mask = (1 << 64)-1
        bits = (seed+tick*0x9e3779b97f4a7c15+channel*0xd1b54a32d192ed03) & mask
        bits = ((bits ^ (bits >> 30))*0xbf58476d1ce4e5b9) & mask
        bits = ((bits ^ (bits >> 27))*0x94d049bb133111eb) & mask
        bits ^= bits >> 31
        return math.sqrt(.06)*((bits >> 11)/(1 << 53)*2.-1.)
    a = math.sqrt(shape[0][0])
    b = shape[0][1]/a
    return [a*component(1), b*component(1)+math.sqrt(shape[1][1]-b*b)*component(2)]


def contains_declared(actual, declared):
    """Serde may add nested defaults; every explicitly supplied value must survive."""
    if isinstance(declared, dict):
        return isinstance(actual, dict) and all(k in actual and contains_declared(actual[k], v) for k, v in declared.items())
    if isinstance(declared, list):
        return isinstance(actual, list) and len(actual) == len(declared) and all(contains_declared(a, d) for a, d in zip(actual, declared))
    return actual == declared


def audit(run, config, ticks, spec, frozen, reconstruction=None):
    recorded = copy.deepcopy(run['scenario'])
    if recorded.get('sensor_timing') is not None:
        # SensorTiming explicitly omits a zero GNSS delay on serialization.
        recorded['sensor_timing'].setdefault('gnss_delay_ticks', 0)
    if not contains_declared(recorded, frozen):
        raise ValueError('physical scenario differs from frozen inputs')
    if {'gnss_noise_shape', 'gnss_bias_windows', 'sensor_timing', 'truth', 'objects'} & config.keys():
        raise ValueError('simulation truth or noise model leaked into operational calibration')
    if config.get('gnss_covariance_shape') != frozen.get('gnss_covariance_shape'):
        raise ValueError('operational shape was inferred or changed')
    frames, summary = run['frames'], run['summary']
    if len(frames) != len(ticks) or len(frames) != summary['steps']:
        raise ValueError('missing full physical/control coverage')
    errors, samples = [], 0
    delay = frozen['sensor_timing']['gnss_delay_ticks']
    for index, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected']
        if abs(inp['time']-index*.05) > 1e-8 or frame['time'] != inp['time'] or frame['estimate'] != out['estimate']:
            raise ValueError('physical/control time alignment changed')
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x', 'y']],
                                [frame['estimate']['pose']['position'][k] for k in ['x', 'y']]))
        acquired = (index-delay)*.05
        dropout = any(acquired+1e-9 >= w['from'] and acquired < w['until']-1e-9 for w in frozen.get('gnss_dropout_windows', []))
        expected = index >= delay and (index-delay) % 4 == 0 and not dropout
        fix = inp.get('gnss')
        if bool(fix) != expected or (fix and abs(fix['stamp']-acquired) > 1e-9):
            raise ValueError('GNSS acquisition/delivery schedule differs')
        if fix:
            original = frames[index-delay]['truth']['pose']['position']
            modeled = noise(spec['seed'], index-delay, frozen['gnss_noise_shape'])
            for w in frozen.get('gnss_bias_windows', []):
                if w['from'] <= fix['stamp'] < w['until']:
                    modeled = [modeled[i]+w['offset'][k] for i, k in enumerate(['x', 'y'])]
            if fix['variance'] != .02 or any(abs(fix['position'][k]-original[k]-modeled[i]) > 1e-10 for i, k in enumerate(['x', 'y'])):
                raise ValueError('declared paired acquisition noise/fault was changed')
            samples += 1
    rmse, maximum = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('truth error score differs from independent full-interval score')
    oracle = reconstruction if reconstruction is not None else (verify_map_ticks(config, ticks) if config.get('localization_map') else verify_ticks(config, ticks))
    clearance = sweep(run)
    failed = []
    if not summary['passed']:
        failed.append('source_acceptance')
    if ((frozen['expected'] == 'goal' and not summary['reached_goal']) or summary['collisions'] or summary['road_violations']
            or summary['simulated_seconds'] > frozen['duration'] or summary['min_clearance'] < GATES['clearance_m']):
        failed.append('physical_goal_clearance_deadline')
    if spec['calibrated'] and (rmse > GATES['rmse_m'] or maximum > GATES['maximum_error_m']):
        failed.append('localization_accuracy')
    if delay > 0 and oracle['compensated_fixes'] == 0:
        failed.append('delayed_gnss_coverage')
    if config.get('localization_map') and (oracle['admitted_map_acquisitions'] == 0
            or oracle['replayed_map_corrections']+oracle['replayed_gnss_corrections'] == 0):
        failed.append('mixed_map_gnss_history_coverage')
    fault = None
    if spec['world'].endswith('bias'):
        stopped = min(f['truth']['speed'] for f in frames if GATES['stop_window_s'][0] <= f['time'] < GATES['stop_window_s'][1])
        healthy = any(t['input']['time'] > GATES['healthy_after_s'] and t['input'].get('gnss')
                      and t['input']['gnss']['stamp'] >= 11. and not t['expected']['health'] for t in ticks)
        fault = {'minimum_actual_speed_m_s': stopped, 'healthy_gnss_recovery': healthy}
        if spec['calibrated'] and (stopped > GATES['maximum_stop_speed_m_s'] or not healthy or oracle['rejected_fixes'] < 10):
            failed.append('fault_stop_recovery')
    return {'oracle': oracle, 'noise_samples_verified': samples, 'rmse_m': rmse, 'maximum_error_m': maximum,
            'independent_clearance': clearance, 'fault': fault, 'failed_gates': failed}


def run_case(output, spec, binary, replay_binary):
    name = spec['world']+('' if spec['calibrated'] else '-isotropic')
    folder = output/'episodes'/spec['plant']/name/f"seed-{spec['seed']}"
    folder.mkdir(parents=True, exist_ok=False)
    source = output/spec['scenario']
    if sha(source) != spec['sha256']:
        raise ValueError('scenario changed after freeze')
    args = ['run', '--scenario', source] if spec['plant'] == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = dict(spec, passed=False, recordings=str(folder.relative_to(output)))
    try:
        row['run_exit_code'] = execute(binary, args+['--seed', spec['seed'], '--output', folder/'run'], folder/'run-process')
        row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
        run = json.loads((folder/'run/run.json').read_text())
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream:
            config = json.loads(next(stream))['header']['config']
            ticks = [r['tick'] for line in stream if 'tick' in (r := json.loads(line))]
        row['summary'] = run['summary']
        row.update(audit(run, config, ticks, spec, json.loads(source.read_text())))
        row['passed'] = (row['run_exit_code'] == row['replay_exit_code'] == 0 and not row['failed_gates']
                         and row['replay']['verified'] and row['replay']['ticks'] == len(ticks))
    except Exception as error:
        row['audit_error'] = repr(error)
    finally:
        row['storage'] = archive(folder)
    return row


def comparisons(rows):
    pairs = []
    for row in rows:
        if row['calibrated'] and row['world'].endswith('bias'):
            control = next((r for r in rows if not r['calibrated'] and all(r[k] == row[k] for k in ['plant', 'world', 'seed'])), None)
            if control is None:
                continue
            reduction = (1.-row['maximum_error_m']/control['maximum_error_m']
                         if 'maximum_error_m' in row and control.get('maximum_error_m', 0.) > 0. else None)
            pairs.append({'plant': row['plant'], 'world': row['world'], 'seed': row['seed'],
                          'maximum_error_reduction': reduction,
                          'passed': bool(row['passed'] and control['passed'] and reduction is not None
                                         and reduction >= GATES['paired_maximum_error_reduction'])})
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 7, 42, 313])
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    parser.add_argument('--freeze-only', action='store_true')
    args = parser.parse_args()
    if any(s < 0 or s >= 1 << 64 for s in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('seeds must be distinct u64 values')
    output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, plants, args.seeds)
    if args.freeze_only:
        print(f"Frozen {len(manifest['episodes'])} episodes before execution")
        return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: {'path': str(binaries[p]), 'sha256': sha(binaries[p])} for p in plants}
    rows = []
    for spec in manifest['episodes']:
        rows.append(run_case(output, spec, binaries[spec['plant']], args.binary.resolve()))
        pairs = comparisons(rows)
        report = {'passed': all(r['passed'] for r in rows) and all(p['passed'] for p in pairs),
                  'complete': len(rows) == len(manifest['episodes']), 'manifest_sha256': sha(output/'frozen-manifest.json'),
                  'binaries': provenance, 'gates': GATES, 'episodes': rows, 'paired_controls': pairs}
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        row = rows[-1]
        print(row['plant'], row['world'], row['seed'], 'calibrated' if row['calibrated'] else 'control',
              row['passed'], row.get('failed_gates', row.get('audit_error')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':
    main()
