#!/usr/bin/env python3
"""Verify optional surveyed-map localization and genuine GNSS-denied driving.

Truth is read only to score position/yaw error and motion. Operational input is
the fixed offline prior plus noisy body-frame LiDAR, GNSS and odometry. Every
episode is replayed from those sensors; compressed artifacts preserve evidence.
"""
import argparse
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parent.parent
CASES = ['map-gnss-recovery', 'map-gnss-prolonged', 'map-gnss-no-overlap',
         'map-gnss-degenerate', 'map-gnss-lidar-loss', 'map-gnss-disabled']


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite(value):
    if isinstance(value, dict): return all(finite(v) for v in value.values())
    if isinstance(value, list): return all(finite(v) for v in value)
    return not isinstance(value, float) or math.isfinite(value)


def positive_definite(covariance):
    if not covariance or len(covariance) != 3 or not finite(covariance): return False
    if any(len(row) != 3 for row in covariance): return False
    if any(abs(covariance[i][j]-covariance[j][i]) > 1e-10 for i in range(3) for j in range(3)): return False
    lower = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(i+1):
            residual = covariance[i][j]-sum(lower[i][k]*lower[j][k] for k in range(j))
            if i == j:
                if residual <= 0: return False
                lower[i][j] = math.sqrt(residual)
            else: lower[i][j] = residual/lower[j][j]
    return True


def verify_episode(run, log, case):
    scenario, frames = run['scenario'], run['frames']
    errors, yaw_errors, outage_errors, outage_positions, outage_speeds = [], [], [], [], []
    accepted_errors, accepted_yaw_errors = [], []
    accepted, rejected = set(), set()
    reasons = set()
    covered_ticks = 0
    first_brake = None
    first_outage_brake = None
    actual_last_gnss = None
    last_map_accept = None
    recovery_accepted = False
    max_rms = max_covariance = max_neighbor_checks = 0.0
    with log.open() as stream:
        header = json.loads(next(stream))['header']['config']
        if any(key in header for key in ['objects', 'gnss_dropout_windows', 'gnss_dropout', 'gnss_bias_windows', 'scenario', 'truth']):
            raise ValueError('simulator truth or acquisition fault labels entered the operational map header')
        expected_map = scenario.get('localization_map')
        if bool(header.get('localization_map')) != bool(expected_map):
            raise ValueError('the opt-in surveyed map boundary changed')
        if expected_map and header['localization_map']['points'] != expected_map['points']:
            raise ValueError('the pipeline map differs from the explicitly offline supplied prior')
        count = 0
        for line in stream:
            record = json.loads(line)
            if record['kind'] != 'tick': continue
            tick = record['tick']; now = count*.05
            inp, out, frame = tick['input'], tick['expected'], frames[count]
            if abs(inp['time']-now) > 1e-8 or abs(frame['time']-now) > 1e-8 or out['time'] != inp['time']:
                raise ValueError('map evidence lacks complete aligned 20 Hz physical/control ticks')
            if any(frame[key] != out[key] for key in ['estimate', 'tracks', 'predictions', 'trajectory', 'command', 'emergency']):
                raise ValueError('physical/control evidence has inconsistent sensor-derived outputs')
            if not finite(out) or not finite(frame): raise ValueError('nonfinite measured/control/evaluation output')
            dropout = (scenario.get('gnss_dropout') is not None and now >= scenario['gnss_dropout']) or any(
                now+1e-9 >= w['from'] and now < w['until']-1e-9 for w in scenario.get('gnss_dropout_windows', []))
            if bool(inp.get('gnss')) != (count%4 == 0 and not dropout):
                raise ValueError('actual GNSS acquisitions do not match the declared outage')
            local = out['localization']; latest = local['last_accepted_stamp']
            if latest != actual_last_gnss:
                if not inp.get('gnss') or latest != inp['gnss']['stamp']:
                    raise ValueError('map corrections forged an actual accepted GNSS timestamp')
                actual_last_gnss = latest
                recovery_accepted |= latest >= 8.0
            if dropout and latest is not None and latest >= 3.0:
                raise ValueError('GNSS accepted clock advanced during a genuine acquisition outage')
            truth, estimate = frame['truth']['pose'], out['estimate']['pose']
            error = math.hypot(truth['position']['x']-estimate['position']['x'], truth['position']['y']-estimate['position']['y'])
            yaw_error = abs(math.remainder(truth['yaw']-estimate['yaw'], math.tau))
            errors.append(error); yaw_errors.append(yaw_error)
            if dropout:
                outage_errors.append(error)
                outage_positions.append(truth['position'])
                outage_speeds.append(frame['truth']['speed'])
            if out['emergency'] and first_brake is None: first_brake = now
            if dropout and out['emergency'] and first_outage_brake is None: first_outage_brake = now
            diagnostic = out.get('map_localization')
            if expected_map:
                if not diagnostic: raise ValueError('configured map localization lacks output diagnostics')
                if diagnostic['last_accepted_stamp'] != last_map_accept:
                    if (diagnostic['decision'] != 'Accepted' or not inp.get('lidar')
                            or diagnostic['last_accepted_stamp'] != inp['lidar']['stamp']):
                        raise ValueError('map accepted clock changed without a new synchronous measured scan')
                    last_map_accept = diagnostic['last_accepted_stamp']
                    accepted.add(last_map_accept)
                if diagnostic['decision'] == 'Rejected':
                    if diagnostic['last_observed_stamp'] is not None: rejected.add(diagnostic['last_observed_stamp'])
                    if diagnostic['rejection']: reasons.add(diagnostic['rejection'])
                if diagnostic['decision'] == 'Accepted':
                    accepted_errors.append(error)
                    accepted_yaw_errors.append(yaw_error)
                    if (diagnostic['last_accepted_stamp'] is None or now-diagnostic['last_accepted_stamp'] > .2+1e-9
                            or diagnostic['rms_m'] > expected_map.get('max_rms_m', .2)+1e-9
                            or diagnostic['inlier_fraction'] < expected_map.get('min_overlap', .5)-1e-9
                            or diagnostic['inlier_count'] < expected_map.get('min_pairs', 20)
                            or not positive_definite(diagnostic['registration_covariance'])
                            or not positive_definite(diagnostic['covariance'])
                            or any(diagnostic['covariance'][i][i]+1e-12 < floor
                                   for i, floor in enumerate([.01, .01, 1e-4]))):
                        raise ValueError('accepted map match lacks fresh bounded fit/covariance evidence')
                    max_rms = max(max_rms, diagnostic['rms_m'])
                    max_covariance = max(max_covariance, max(diagnostic['covariance'][i][i] for i in range(2)))
                    max_neighbor_checks = max(max_neighbor_checks, diagnostic['neighbor_checks'])
                supported = (actual_last_gnss is not None and .75+1e-9 < now-actual_last_gnss
                             <= expected_map.get('max_gnss_outage_s', 10)+1e-9
                             and diagnostic['decision'] == 'Accepted' and last_map_accept is not None
                             and now-last_map_accept <= .2+1e-9)
                if diagnostic['gnss_outage_covered'] != supported:
                    raise ValueError('map bridge bypassed its real GNSS age or fresh accepted scan bound')
                covered_ticks += int(supported)
            elif diagnostic:
                raise ValueError('disabled control unexpectedly obtained map localization')
            count += 1
    if count != len(frames) or count != run['summary']['steps']:
        raise ValueError('map evidence omitted a full physical/control frame')
    summary = run['summary']
    accuracy_errors = errors if case == 'map-gnss-recovery' else accepted_errors
    accuracy_yaw = yaw_errors if case == 'map-gnss-recovery' else accepted_yaw_errors
    if (not summary['passed'] or summary['collisions'] or summary['road_violations']
            or summary['min_clearance'] < 1.0
            or (accuracy_errors and max(accuracy_errors) > .5)
            or (accuracy_yaw and max(accuracy_yaw) > .1)):
        raise ValueError('map fixture violated fixed physical clearance/localization accuracy criteria')
    outage_distance = sum(math.dist((a['x'], a['y']), (b['x'], b['y'])) for a,b in zip(outage_positions, outage_positions[1:]))
    if case == 'map-gnss-recovery':
        if (first_outage_brake is not None or not summary['reached_goal'] or not recovery_accepted
                or covered_ticks < 60 or outage_distance < 10 or min(outage_speeds) < 2.0):
            raise ValueError('GNSS-denied recovery did not actually drive and regain real GNSS')
    else:
        if first_brake is None or summary['reached_goal'] or summary['final_speed'] > .2:
            raise ValueError('negative map fixture did not fail closed with finite physical braking')
        if case == 'map-gnss-prolonged' and not (12.8 < first_brake <= 12.9 and covered_ticks > 150):
            raise ValueError('prolonged outage did not preserve then expire the actual ten-second bound')
        if case in ['map-gnss-no-overlap', 'map-gnss-degenerate', 'map-gnss-disabled'] and not (3.55 < first_brake <= 3.65):
            raise ValueError('untrusted/disabled map fixture escaped the original GNSS freshness brake')
        if case in ['map-gnss-no-overlap', 'map-gnss-degenerate'] and (accepted or not rejected):
            raise ValueError('invalid map geometry unexpectedly supplied outage permission')
        if case == 'map-gnss-lidar-loss' and not (accepted and covered_ticks > 20 and 6.1 < first_brake <= 6.25):
            raise ValueError('loss of accepted map acquisition did not revoke the GNSS outage bridge')
    return {'passed': True, 'control_ticks': count, 'accepted_map_scans': len(accepted),
            'rejected_map_scans': len(rejected), 'map_rejection_reasons': sorted(reasons),
            'covered_outage_ticks': covered_ticks, 'first_emergency_s': first_brake,
            'first_outage_emergency_s': first_outage_brake,
            'actual_gnss_recovery_observed': recovery_accepted,
            'gnss_denied_driven_distance_m': outage_distance,
            'gnss_denied_min_speed_m_s': min(outage_speeds),
            'position_rmse_m': math.sqrt(sum(x*x for x in errors)/len(errors)),
            'max_position_error_m': max(errors), 'max_outage_position_error_m': max(outage_errors),
            'max_yaw_error_rad': max(yaw_errors), 'max_accepted_rms_m': max_rms,
            'max_accepted_map_position_error_m': max(accepted_errors) if accepted_errors else None,
            'max_accepted_map_yaw_error_rad': max(accepted_yaw_errors) if accepted_yaw_errors else None,
            'accuracy_gate_scope': 'whole_positive_episode' if case == 'map-gnss-recovery' else 'fresh_accepted_map_ticks_only',
            'untrusted_map_whole_episode_error_is_reported_not_accuracy_acceptance': case != 'map-gnss-recovery',
            'max_fused_xy_variance_m2': max_covariance, 'max_neighbor_checks': max_neighbor_checks,
            'offline_prior_only': True, 'truth_used_only_for_evaluation': True}


def archive_evidence(directory):
    """Only archive files produced by this invocation after replay verification."""
    archived = {}
    for relative in ['run.json', 'sensors.jsonl', 'summary.json', 'replay/replay.json']:
        path = directory/relative
        destination = path.with_name(path.name+'.gz')
        with path.open('rb') as source, gzip.open(destination, 'wb', compresslevel=6) as target:
            shutil.copyfileobj(source, target)
        with gzip.open(destination, 'rb') as source:
            digest = hashlib.file_digest(source, 'sha256').hexdigest()
        if digest != sha256(path): raise ValueError('compressed evidence verification failed')
        archived[relative] = {'path': str(destination), 'uncompressed_sha256': digest,
                              'compressed_sha256': sha256(destination), 'compressed_bytes': destination.stat().st_size}
        path.unlink()
    return archived


def check_tampering(log, cli, directory):
    """Mutate operational prior, accepted diagnostics and valid measured points."""
    results = []
    directory.mkdir(parents=True, exist_ok=True)
    for mutation in ['offline_prior_shift', 'accepted_stamp', 'fused_covariance', 'body_scan_translation']:
        path = directory/f'{mutation}.jsonl'
        changed = False
        with log.open() as source, path.open('w') as target:
            for line in source:
                record = json.loads(line)
                if not changed and mutation == 'offline_prior_shift' and record['kind'] == 'header':
                    for point in record['header']['config']['localization_map']['points']:
                        point['x'] += 200
                    changed = True
                elif not changed and record['kind'] == 'tick':
                    tick = record['tick']; diagnostic = tick['expected'].get('map_localization')
                    if diagnostic and diagnostic['decision'] == 'Accepted' and tick['input'].get('lidar'):
                        if mutation == 'accepted_stamp': diagnostic['last_accepted_stamp'] += .05
                        elif mutation == 'fused_covariance': diagnostic['covariance'][0][0] *= 2
                        elif mutation == 'body_scan_translation':
                            for point in tick['input']['lidar']['points']: point['x'] += .4
                        changed = True
                target.write(json.dumps(record, separators=(',', ':'))+'\n')
        if not changed: raise ValueError(f'tamper probe never mutated {mutation}')
        outcome = subprocess.run([str(cli), 'replay', '--log', str(path), '--output', str(directory/mutation)],
                                 cwd=ROOT, capture_output=True, text=True)
        if outcome.returncode == 0: raise ValueError(f'replay accepted corrupted {mutation}')
        compressed = path.with_name(path.name+'.gz')
        with path.open('rb') as source, gzip.open(compressed, 'wb', compresslevel=6) as target:
            shutil.copyfileobj(source, target)
        digest = sha256(path)
        with gzip.open(compressed, 'rb') as source:
            if hashlib.file_digest(source, 'sha256').hexdigest() != digest:
                raise ValueError('corrupted sensor evidence compression did not verify')
        path.unlink()
        results.append({'mutation': mutation, 'rejected': True, 'replay_exit_code': outcome.returncode,
                        'replay_stderr': outcome.stderr.strip(), 'source_log_sha256': sha256(log),
                        'corrupted_log_sha256': digest, 'evidence': str(compressed),
                        'compressed_sha256': sha256(compressed)})
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['all', 'reference', 'rne-dynamic'], default='all')
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 7, 42])
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/map-localization')
    parser.add_argument('--cases', choices=CASES, nargs='+', default=CASES)
    args = parser.parse_args()
    if len(set(args.seeds)) != len(args.seeds) or any(s < 0 or s >= 2**64 for s in args.seeds):
        parser.error('seeds must be unique u64 integers')
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/'report.json'
    report_path.unlink(missing_ok=True)
    helper_spec = importlib.util.spec_from_file_location('hazards', ROOT/'scripts/check_hazards.py')
    helper = importlib.util.module_from_spec(helper_spec); helper_spec.loader.exec_module(helper)
    report = {'schema_version': 1, 'source_fingerprint_sha256': helper.source_fingerprint(),
              'checker_sha256': sha256(Path(__file__)),
              'offline_map_generator_sha256': sha256(ROOT/'scripts/make_localization_fixtures.py'),
              'map_provenance': 'Authored offline fixed landmark surface geometry; no live poses/labels',
              'truth_domain': 'planar evaluation; native cylinders are 3D Rapier query geometry',
              'max_allowed_position_error_m': .5, 'max_allowed_yaw_error_rad': .1,
              'accuracy_scope_counterexample': {
                  'backend': 'rne-dynamic', 'scenario': 'map-gnss-no-overlap', 'seed': 7,
                  'whole_episode_max_yaw_error_rad': .10396238209407008,
                  'whole_episode_max_position_error_m': .24230674085706386,
                  'first_emergency_s': 3.6, 'physical_acceptance_passed': True,
                  'preceding_whole_episode_yaw_gate_failed': True,
                  'interpretation': 'Untrusted map is a finite fail-closed braking test, not an accurate-localization claim'},
              'cases': args.cases, 'seeds': args.seeds, 'runs': [], 'replay_tamper_checks': [], 'passed': True}
    backends = ['reference', 'rne-dynamic'] if args.backend == 'all' else [args.backend]
    cli = ROOT/'target/release/rustdrive'
    for backend in backends:
        for case in args.cases:
            for seed in args.seeds:
                directory = args.output/backend/case/f'seed-{seed}'
                directory.mkdir(parents=True, exist_ok=True)
                command = ([str(cli), 'run'] if backend == 'reference' else
                           [str(ROOT/'integrations/rne/target/release/rustdrive-rne'), '--plant', 'dynamic'])
                command += ['--scenario', str(ROOT/'scenarios'/f'{case}.json'), '--seed', str(seed), '--output', str(directory)]
                start = time.perf_counter()
                outcome = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
                elapsed = time.perf_counter()-start
                if outcome.returncode:
                    raise ValueError(f'{backend}/{case}/{seed} run failed: {outcome.stderr} {outcome.stdout}')
                run = json.loads((directory/'run.json').read_text())
                metrics = verify_episode(run, directory/'sensors.jsonl', case)
                replay = subprocess.run([str(cli), 'replay', '--log', str(directory/'sensors.jsonl'),
                                         '--output', str(directory/'replay')], cwd=ROOT, capture_output=True, text=True)
                if replay.returncode: raise ValueError(f'sensor replay failed: {replay.stderr}')
                recomputed = json.loads((directory/'replay/replay.json').read_text())
                if not recomputed['verified'] or recomputed['ticks'] != run['summary']['steps']:
                    raise ValueError('replay did not verify every recorded sensor tick')
                if case == 'map-gnss-recovery' and not report['replay_tamper_checks']:
                    report['replay_tamper_checks'] = check_tampering(directory/'sensors.jsonl', cli, args.output/'tamper')
                row = {'backend': backend, 'scenario': case, 'seed': seed, 'command': command,
                       'scenario_sha256': sha256(ROOT/'scenarios'/f'{case}.json'),
                       'run_wall_seconds': elapsed, 'wall_timing_is_not_real_time_guarantee': True,
                       'expected_behavior': 'gnss_denied_driving_recovery' if case == 'map-gnss-recovery' else 'fail_closed_negative',
                       'summary': run['summary'], 'metrics': metrics, 'replay': recomputed,
                       'evidence': archive_evidence(directory), 'passed': True}
                report['runs'].append(row)
                report_path.write_text(json.dumps(report, indent=2)+'\n')
                print(f'{backend} {case} seed {seed}: PASS (wall {elapsed:.2f}s)', flush=True)
    print(f'{len(report["runs"])} map-localization episodes; passed=True')


if __name__ == '__main__':
    main()
