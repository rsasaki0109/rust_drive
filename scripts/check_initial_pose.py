#!/usr/bin/env python3
"""Evaluate explicit pose priors, physical spawn offsets and full covariance replay."""
import argparse
import copy
import hashlib
import itertools
import json
import math
import os
from pathlib import Path

from check_delayed_map import audit as map_audit
from check_gnss_latency import audit as gnss_audit, Filter, angle, verify_pose_covariance
from check_gnss_map_history import archive, inverse
from check_lane_change_speeds import sweep
from check_mid_edge import CASES, physical as route_audit
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
BOUND_RAD = 3.*math.pi/180.
PRIOR = [[1., 0., 0.], [0., 1., 0.], [0., 0., BOUND_RAD*BOUND_RAD/3.]]


def specifications():
    rows = []
    def add(name, source, seed, offset=0., positive=True, domain='route'):
        rows.append({'id': name, 'source': source, 'seed': seed, 'heading_offset_rad': offset,
                     'expected_quality_pass': positive, 'domain': domain})
    for case, seed in itertools.product(CASES, [1, 7, 42]):
        add(f'viewed-{case}-seed-{seed}', f'scenarios/mid-edge-{case}.json', seed)
    for case, direction, seed in itertools.product(['same-edge', 'obstacle', 'osm'], [-1, 1], [101, 202, 303]):
        add(f'offset-{case}-{direction:+d}-seed-{seed}', f'scenarios/mid-edge-{case}.json', seed, direction*2.*math.pi/180.)
    for case, seed in itertools.product(['gnss-curve', 'map-gnss-early', 'map-gnss-late'], [1, 7, 42]):
        source = 'scenarios/gnss-delay-curve.json' if case == 'gnss-curve' else f'scenarios/delayed-{case}.json'
        add(f'composed-{case}-seed-{seed}', source, seed, domain='gnss' if case == 'gnss-curve' else 'map')
    for direction in [-1, 1]:
        add(f'miscalibrated-{direction:+d}-seed-7', 'scenarios/mid-edge-same-edge.json', 7, direction*.12, False)
    return rows


def confirmation_specifications():
    """Both frozen follow-up cohorts, including the first cohort's failures."""
    rows = []
    for prefix, seeds, degrees in [('confirm', [404, 505, 606], 2.), ('confirm-C', [707, 808, 909], 2.5),
                                    ('confirm-D', [1001], 2.75)]:
        for case, direction, seed in itertools.product(['same-edge', 'obstacle', 'osm'], [-1, 1], seeds):
            rows.append({'id': f'{prefix}-offset-{case}-{direction:+d}-seed-{seed}',
                         'source': f'scenarios/mid-edge-{case}.json', 'seed': seed,
                         'heading_offset_rad': direction*degrees*math.pi/180.,
                         'expected_quality_pass': True, 'domain': 'route'})
    return rows


def materialize(spec):
    scenario = json.loads((REPO/spec['source']).read_text())
    scenario['initial_pose_covariance'] = copy.deepcopy(PRIOR)
    if spec['heading_offset_rad']:
        scenario['initial_heading_offset_rad'] = spec['heading_offset_rad']
    return scenario


def reconstruct(config, ticks):
    """Full-history ordinary GNSS oracle, independent joint matrix updates."""
    oracle = Filter(config)
    previous, maximum = None, 0.
    for tick in ticks:
        inp, out = tick['input'], tick['expected']
        if previous is not None:
            oracle.predict(inp['odometry'], inp['time']-previous)
        if inp.get('gnss') is not None:
            if abs(inp['gnss']['stamp']-inp['time']) > 1e-9: raise ValueError('ordinary audit received delayed GNSS')
            oracle.correct(inp['gnss'])
        actual = out['estimate']['pose']
        difference = max(abs(actual['position']['x']-oracle.x[0]), abs(actual['position']['y']-oracle.x[1]), abs(angle(actual['yaw']-oracle.x[2])))
        maximum = max(maximum, difference)
        if (difference > 1e-8 or abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8
                or abs(out['estimate']['speed']-oracle.speed) > 1e-8):
            raise ValueError('posterior pose/trace differs from independent reconstruction')
        verify_pose_covariance(out, oracle.p, config)
        diagnostics = out['localization']
        for key, value in [('accepted_fixes', oracle.accepted), ('rejected_fixes', oracle.rejected),
                           ('last_observed_stamp', oracle.observed), ('last_accepted_stamp', oracle.accepted_stamp),
                           ('last_decision', oracle.decision)]:
            if diagnostics[key] != value: raise ValueError('GNSS freshness/count ownership changed')
        if oracle.nis is not None and not math.isclose(diagnostics['last_nis'], oracle.nis, rel_tol=1e-9, abs_tol=1e-8):
            raise ValueError('original GNSS NIS differs')
        previous = inp['time']
    return {'passed': True, 'verified_ticks': len(ticks), 'maximum_pose_difference': maximum, 'full_covariance_verified': True}


def audit(run, scenario, spec, sensor_path):
    with sensor_path.open() as stream:
        config = json.loads(next(stream))['header']['config']
        ticks = [item['tick'] for line in stream if 'tick' in (item := json.loads(line))]
    if config.get('initial_pose_covariance') != PRIOR or config['initial_pose_covariance'] != scenario['initial_pose_covariance']:
        raise ValueError('recorded calibration differs from fixed prior')
    if {'initial_heading_offset_rad', 'objects', 'sensor_timing', 'gnss_dropout_windows'} & config.keys():
        raise ValueError('simulator-only state entered operational initialization')
    base = json.loads((REPO/spec['source']).read_text())
    if {k: v for k, v in scenario.items() if k not in ['initial_pose_covariance', 'initial_heading_offset_rad']} != base:
        raise ValueError('original world, plant calibration or deadline changed')
    frames = run['frames']
    if len(frames) != len(ticks) or len(frames) != run['summary']['steps']:
        raise ValueError('prior qualification requires every physical and control tick')
    start_yaw = config['initial_pose']['yaw']
    if abs(angle(frames[0]['truth']['pose']['yaw']-start_yaw)-spec['heading_offset_rad']) > 1e-12:
        raise ValueError('physical spawn offset was ignored or copied into calibration')
    if spec['domain'] == 'route':
        reference = route_audit(run, scenario, next(case for case in CASES if spec['source'].endswith(f'-{case}.json')), config)
        reconstruction = reconstruct(config, ticks)
    elif spec['domain'] == 'gnss':
        reference = gnss_audit(run, sensor_path, 'curve')
        if not reference['accuracy_passed']: raise ValueError('existing 0.2/0.4 m GNSS composition gates failed')
        reconstruction = reference['oracle']
    else:
        reference = map_audit(run, sensor_path, 'gnss-early' if 'early' in spec['id'] else 'gnss-late')
        reconstruction = reference['oracle']
    errors, yaws, nees, covariance_count = [], [], [], 0
    for frame, tick in zip(frames, ticks):
        truth, estimate = frame['truth']['pose'], tick['expected']['estimate']['pose']
        if frame['time'] != tick['input']['time'] or estimate != frame['estimate']['pose']:
            raise ValueError('physical and sensor estimates are not aligned')
        residual = [truth['position'][k]-estimate['position'][k] for k in ['x', 'y']]+[angle(truth['yaw']-estimate['yaw'])]
        errors.append(math.hypot(*residual[:2]));yaws.append(abs(residual[2]))
        covariance = tick['expected'].get('pose_covariance')
        if covariance is None: raise ValueError('explicit initialization omitted a posterior matrix')
        inv = inverse(covariance)
        nees.append(sum(residual[i]*inv[i][j]*residual[j] for i in range(3) for j in range(3)))
        covariance_count += 1
    rmse, maximum, yaw = math.sqrt(sum(e*e for e in errors)/len(errors)), max(errors), max(yaws)
    if abs(rmse-run['summary']['localization_rmse']) > 1e-8 or abs(maximum-run['summary']['localization_max_error']) > 1e-8:
        raise ValueError('whole-episode localization metrics differ')
    failures = [name for name, value, limit in [('rmse', rmse, .3), ('position', maximum, .6), ('yaw', yaw, .1)] if value > limit]
    if not run['summary']['passed']: failures.append('original_scenario_acceptance')
    clearance = sweep(run)
    return {'quality_passed': not failures, 'failed_gates': failures, 'oracle': reconstruction,
            'full_covariance_ticks': covariance_count, 'rmse_m': rmse, 'maximum_position_error_m': maximum,
            'maximum_yaw_error_rad': yaw, 'independent_clearance': clearance,
            'conditional_nees': {'inside_95_percent_ellipsoid_ticks': sum(v <= 7.814727903251179 for v in nees),
                                 'total_ticks': len(nees), 'maximum': max(nees),
                                 'note': 'Descriptive correlated simulation ticks under supplied calibration; not independent or real-vehicle uncertainty qualification.'},
            'domain_acceptance': reference, 'summary': run['summary']}


def run_case(binary, folder, spec, plant):
    folder.mkdir(parents=True, exist_ok=False)
    scenario = materialize(spec)
    source = folder/'scenario.json';source.write_text(json.dumps(scenario, indent=2)+'\n')
    args = ['run', '--scenario', source] if plant == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = {**spec, 'plant': plant, 'passed': False, 'scenario_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    row['run_exit_code'] = execute(binary, args+['--seed', spec['seed'], '--output', folder/'run'], folder/'run-process')
    replay_binary = REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving')
    row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
    try:
        result = audit(json.loads((folder/'run/run.json').read_text()), scenario, spec, folder/'run/sensors.jsonl')
        row.update(result)
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        row['passed'] = (row['run_exit_code'] in [0, 1] and row['replay_exit_code'] == 0 and row['replay']['verified']
                         and result['quality_passed'] == spec['expected_quality_pass'])
        if spec['expected_quality_pass']: row['passed'] &= row['run_exit_code'] == 0
        else: row['passed'] &= 'yaw' in result['failed_gates']
    except Exception as error: row['audit_error'] = repr(error)
    row['storage'] = archive(folder)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['reference-2d', 'rne-dynamic'], default='reference-2d')
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--smoke', action='store_true', help='CI seed-7 worlds, seed-303 offset regressions and miscalibration controls')
    parser.add_argument('--extended', action='store_true', help='Include all frozen follow-up cohorts (95 episodes per plant)')
    args = parser.parse_args()
    default = REPO/('target/release/rustdriving' if args.plant == 'reference-2d' else 'integrations/rne/target/release/rustdriving-rne')
    binary = (args.binary or default.with_suffix('.exe' if os.name == 'nt' else '')).resolve()
    specs = specifications()
    if args.extended: specs += confirmation_specifications()
    if args.smoke:
        specs = [s for s in specs if s['seed'] in [7, 303]]
        specs += [s for s in confirmation_specifications()
                  if s['source'].endswith('-obstacle.json') and s['heading_offset_rad'] < 0 and s['seed'] in [505, 606]]
    args.output.mkdir(parents=True, exist_ok=False)
    frozen = {'prior': PRIOR, 'specifications': specs, 'scenario_sha256': {s['id']: hashlib.sha256((json.dumps(materialize(s), indent=2)+'\n').encode()).hexdigest() for s in specs}}
    (args.output/'frozen-inputs.json').write_text(json.dumps(frozen, indent=2)+'\n')
    rows = []
    for spec in specs:
        rows.append(run_case(binary, args.output/spec['id'], spec, args.plant))
        if rows[-1]['scenario_sha256'] != frozen['scenario_sha256'][spec['id']]: raise ValueError('scenario changed after freeze')
        report = {'passed': all(r['passed'] for r in rows), 'episodes': rows}
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(spec['id'], rows[-1]['passed'], rows[-1].get('audit_error', ''), flush=True)
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__': main()
