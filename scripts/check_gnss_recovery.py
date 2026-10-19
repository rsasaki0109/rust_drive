#!/usr/bin/env python3
"""Audit bounded GNSS recovery using full-history joint-matrix proposals."""
import argparse
import copy
import json
import math
import os
from pathlib import Path

from check_delayed_map import Conflict, pose_difference, rebuild
from check_gnss_covariance import audit as physical, scenario as covariance_scenario
from check_gnss_latency import angle, verify_pose_covariance
from check_gnss_map_history import MapFilter, MapOnlyOracle, archive, sha
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
POLICY = {'max_position_correction_m': .4, 'max_yaw_correction_rad': .1,
          'confirmation_fixes': 3, 'max_hold_s': 5.}
WORLDS = ['straight', 'curve', 'north-bias', 'diagonal-bias', 'map-early', 'map-late']


def specifications(plants, smoke=False):
    rows = []
    for plant in plants:
        for world in WORLDS+['map-bias-early', 'map-bias-late']:
            seeds = [7] if smoke else ([7, 42] if world.startswith('map-bias') else [1, 7, 42, 313])
            for seed in seeds:
                rows.append(dict(plant=plant, world=world, seed=seed, mode='guarded'))
                if 'bias' in world:
                    rows.append(dict(plant=plant, world=world, seed=seed, mode='control'))
        for world in ['north-bias', 'diagonal-bias']:
            rows.append(dict(plant=plant, world=world, seed=7, mode='expiry'))
            if not smoke:
                rows.extend(dict(plant=plant, world=world, seed=seed, mode='confirm') for seed in [809, 2027])
    return rows


def materialize(spec):
    world = spec['world']
    base = world.replace('map-bias-', 'map-')
    source, value = covariance_scenario(base)
    if world.startswith('map-bias'):
        dx, dy = (0., .8) if world.endswith('early') else (.8/math.sqrt(2.), -.8/math.sqrt(2.))
        value['gnss_bias_windows'] = [{'from': 8., 'until': 11., 'offset': {'x': dx, 'y': dy}}]
        value['name'] += ' + narrow-axis bias'
    if spec['mode'] != 'control':
        value['gnss_recovery'] = copy.deepcopy(POLICY)
    if spec['mode'] == 'expiry':
        value['gnss_bias_windows'][0]['until'] = 18.
        value['expected'] = 'fault'
    return source, value


def freeze(output, specs):
    manifest = {'policy': POLICY, 'episodes': []}
    for i, spec in enumerate(specs):
        source, value = materialize(spec)
        path = output/'frozen'/f'{i:03d}.json'
        content = json.dumps(value, indent=2)+'\n'
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            if path.read_text() != content:
                raise ValueError('frozen scenario was modified')
        else:
            with path.open('x', newline='\n') as stream:
                stream.write(content)
        manifest['episodes'].append(dict(spec, scenario=str(path.relative_to(output)), sha256=sha(path),
                                        source=source, source_sha256=sha(REPO/source)))
    path = output/'frozen-manifest.json'
    if path.exists():
        if json.loads(path.read_text()) != manifest:
            raise ValueError('frozen cohort changed')
    else:
        with path.open('x', newline='\n') as stream:
            stream.write(json.dumps(manifest, indent=2)+'\n')
    return manifest


def verify_ticks(config, ticks, measured_reset_stamp=None):
    """Unbounded time-ordered filter; only finally admitted fixes enter history.

    Recovery is folded from delivery events and independent complete proposals.
    No producer snapshots, guard state or covariance corrections are imported.
    Map registration observations remain the existing sensor-fit interface.
    """
    policy = config.get('gnss_recovery')
    motions, fixes, maps = [], [], []
    previous = observed = accepted_stamp = decision = nis = wheel = None
    accepted = rejected = streak = compensated = replayed_maps = replayed_gnss = 0
    since = last_candidate = last_distance = last_yaw = None
    regional = config.get('relocalization') or {}
    if regional.get('confirm_gnss_handoff') and measured_reset_stamp is not None:
        since = measured_reset_stamp
    grace = regional.get('map_only_grace_s')
    support = MapOnlyOracle(measured_reset_stamp, grace) if grace is not None and measured_reset_stamp is not None else None
    support_ticks, last_lidar, proposals = 0, None, []
    confirmations = []
    expired = False
    held = recovered = covered_ticks = 0
    maximum = maximum_prior = 0.
    oracle = MapFilter(config)
    map_config = config.get('localization_map') or {}
    for tick in ticks:
        inp, out = tick['input'], tick['expected']
        now = inp['time']
        wheel = inp.get('odometry') or wheel
        if previous is not None:
            motion = wheel
            if motion is None or now-motion['stamp'] > .15+1e-9 or now-previous > .25+1e-9:
                raise ValueError('this physical protocol requires healthy measured motion')
            motions.append((previous, now, motion))
            oracle.predict(motion, now-previous)
        if policy and since is not None:
            expired |= now-since > policy['max_hold_s']+1e-9
            if expired or (confirmations and now-last_candidate > .75+1e-9):
                confirmations = []
        fix = inp.get('gnss')
        if fix:
            if (observed is not None and fix['stamp'] <= observed) or not 0. <= now-fix['stamp'] <= .5+1e-9:
                raise ValueError('unexpected repeated or uncovered acquisition')
            prior = rebuild(config, motions, fixes, maps, fix['stamp'], 0)
            prior.correct(fix)
            observed, decision, nis = fix['stamp'], prior.decision, prior.nis
            proposal = rebuild(config, motions, fixes+[fix], maps, now) if decision == 'Accepted' else None
            proposal_distance = math.dist(oracle.x[:2], proposal.x[:2]) if proposal else None
            proposal_yaw = abs(angle(proposal.x[2]-oracle.x[2])) if proposal else None
            if decision == 'Accepted' and policy and since is not None:
                last_distance = math.dist(oracle.x[:2], proposal.x[:2])
                last_yaw = abs(angle(proposal.x[2]-oracle.x[2]))
                last_candidate = fix['stamp']
                if expired or last_distance > policy['max_position_correction_m'] or last_yaw > policy['max_yaw_correction_rad']:
                    confirmations = []
                else:
                    confirmations.append(fix['stamp'])
                if len(confirmations) < policy['confirmation_fixes']:
                    decision = 'RecoveryHeld'
                    held += 1
                else:
                    recovered += 1
                    since, confirmations = None, []
            if decision == 'Accepted':
                oracle = proposal
                fixes.append(fix)
                accepted, accepted_stamp, streak = accepted+1, fix['stamp'], 0
                compensated += now-fix['stamp'] > 1e-9
                replayed_maps += sum(t >= fix['stamp']-1e-9 for t, _, _ in maps)
            else:
                rejected, streak = rejected+1, streak+1
                if policy and decision == 'RejectedInnovation':
                    confirmations = []
                    if streak >= 2 and since is None:
                        since = now
            if measured_reset_stamp is not None:
                proposals.append(dict(delivery=now, acquisition=fix['stamp'], decision=decision,
                                      nis=nis, position_correction_m=proposal_distance, yaw_correction_rad=proposal_yaw,
                                      accepted_fixes=accepted, consecutive_candidates=len(confirmations)))
        diagnostic, scan = out.get('map_localization'), inp.get('lidar')
        if map_config and map_config.get('allow_delayed_scans') and scan and diagnostic['last_observed_stamp'] == scan['stamp'] and diagnostic.get('acquisition_prior') is not None:
            stamp = scan['stamp']
            if now-stamp > .2+1e-9:
                raise ValueError('expired map acquisition')
            prior = rebuild(config, motions, fixes, maps, stamp, 1)
            delta = pose_difference(diagnostic['acquisition_prior'], prior.x)
            maximum_prior = max(maximum_prior, delta)
            if delta > 1e-8 or abs(diagnostic['acquisition_delay_s']-(now-stamp)) > 1e-9:
                raise ValueError('map seed includes a held GNSS proposal')
            if diagnostic['matched_pose'] is not None:
                measurement = (stamp, diagnostic['matched_pose'], diagnostic['covariance'])
                admitted = prior.map_correct(measurement[1], measurement[2])
                candidate = None
                if admitted:
                    try:
                        candidate = rebuild(config, motions, fixes, maps+[measurement], now)
                    except Conflict:
                        admitted = False
                if admitted != (diagnostic['decision'] == 'Accepted'):
                    raise ValueError('map transaction gate or replay differs')
                if admitted:
                    oracle = candidate
                    maps.append(measurement)
                    replayed_gnss += sum(g['stamp'] > stamp+1e-9 for g in fixes)
        elif map_config and diagnostic['last_observed_stamp'] == now and diagnostic['matched_pose'] is not None:
            admitted = oracle.map_correct(diagnostic['matched_pose'], diagnostic['covariance'])
            if admitted != (diagnostic['decision'] == 'Accepted'):
                raise ValueError('synchronous map innovation gate differs')
            if admitted:
                if not scan or scan['stamp'] != now or inp['lidar_failed']:
                    raise ValueError('synchronous map correction lacks a fresh raw scan')
                if any(diagnostic['covariance'][i][i]+1e-12 < floor for i, floor in enumerate([.01,.01,1e-4])):
                    raise ValueError('synchronous map covariance bypassed conservative floors')
                maps.append((now, diagnostic['matched_pose'], diagnostic['covariance']))
        last_map = maps[-1][0] if maps else None
        covered = bool(map_config and accepted_stamp is not None and .75+1e-9 < now-accepted_stamp
                       <= map_config.get('max_gnss_outage_s', 10.)+1e-9 and diagnostic['decision'] == 'Accepted'
                       and last_map is not None and now-last_map <= .2+1e-9)
        covered_ticks += covered
        if map_config and (diagnostic['last_accepted_stamp'] != last_map or diagnostic['gnss_outage_covered'] != covered):
            raise ValueError('map clocks or outage coverage changed during a proposal')
        if scan and not inp['lidar_failed'] and (measured_reset_stamp is None or scan['stamp'] > measured_reset_stamp):
            last_lidar = scan['stamp']
        handoff_expired = measured_reset_stamp is not None and regional.get('confirm_gnss_handoff', False) and expired
        if ('GnssHandoffExpired' in out['health']) != handoff_expired:
            raise ValueError('measured handoff deadline differs from raw delivery time')
        map_only = support.verify(out, now, last_map, bool(diagnostic and diagnostic['decision'] == 'Accepted'),
                                  accepted_stamp, interrupted=rejected > 0 or handoff_expired,
                                  healthy=not handoff_expired and not inp['lidar_failed'] and last_lidar is not None and now-last_lidar <= .35+1e-9) if support else False
        if not support and (out.get('relocalization') or {}).get('map_only_support') is not None:
            raise ValueError('map-only support lacks a proved measured reset')
        support_ticks += map_only
        stale = not map_only and (accepted_stamp is None or (now-accepted_stamp > .75+1e-9 and not covered))
        if ('StaleGnss' in out['health']) != stale or ('GnssInnovationHold' in out['health']) != (streak >= 2):
            raise ValueError('held/rejected observations refreshed localization health')
        if (stale or streak >= 2 or handoff_expired) and out['command']['acceleration'] > -5.99:
            raise ValueError('recovery hold did not brake')
        delta = pose_difference(out['estimate']['pose'], oracle.x)
        maximum = max(maximum, delta)
        if delta > 1e-8 or abs(out['estimate']['speed']-oracle.speed) > 1e-8 or abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1]) > 1e-8:
            raise ValueError('held proposals changed reconstructed pose/covariance')
        verify_pose_covariance(out, oracle.p, config)
        for key, value in [('accepted_fixes', accepted), ('rejected_fixes', rejected), ('last_observed_stamp', observed),
                           ('last_accepted_stamp', accepted_stamp), ('last_decision', decision)]:
            if out['localization'][key] != value:
                raise ValueError('observation ownership differs: '+key)
        if nis is not None and not math.isclose(out['localization']['last_nis'], nis, abs_tol=1e-8, rel_tol=1e-9):
            raise ValueError('original acquisition NIS was overwritten')
        latency = out['gnss_latency']
        if (latency['compensated_fixes'] != compensated or latency.get('replayed_map_corrections', 0) != replayed_maps
                or latency.get('replayed_gnss_corrections', 0) != replayed_gnss or latency['history_unavailable_fixes']
                or latency.get('history_conflict_fixes', 0) or any(latency.get(k, 0) > 64 for k in
                    ['history_predictions', 'history_map_corrections', 'history_gnss_corrections'])):
            raise ValueError('uncommitted proposals entered retained event history or counters')
        if policy:
            phase = 'Expired' if expired else ('Ready' if since is None else ('Confirming' if confirmations else 'Holding'))
            expected = dict(phase=phase, hold_started_at=since, consecutive_candidates=len(confirmations),
                            last_candidate_stamp=last_candidate, last_position_correction_m=last_distance,
                            last_yaw_correction_rad=last_yaw, held_fixes=held, completed_recoveries=recovered)
            for key, value in expected.items():
                actual = out['gnss_recovery'][key]
                if isinstance(value, float):
                    if actual is None or abs(actual-value) > 1e-8:
                        raise ValueError('recovery proposal evidence differs: '+key)
                elif actual != value:
                    raise ValueError('recovery event fold differs: '+key)
        elif out.get('gnss_recovery') is not None:
            raise ValueError('default mode unexpectedly enabled recovery')
        previous = now
    result = dict(passed=True, verified_ticks=len(ticks), maximum_pose_difference=maximum,
                maximum_acquisition_prior_difference=maximum_prior, accepted_fixes=accepted, rejected_fixes=rejected,
                compensated_fixes=compensated, admitted_map_acquisitions=len(maps), replayed_map_corrections=replayed_maps,
                replayed_gnss_corrections=replayed_gnss, covered_outage_ticks=covered_ticks,
                held_proposals=held, completed_recoveries=recovered, expired=expired)
    if measured_reset_stamp is not None:
        result.update(map_only_support_ticks=support_ticks, gnss_proposals=proposals)
    return result


def run_case(output, spec, binary, replay_binary):
    folder = output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
    folder.mkdir(parents=True, exist_ok=False)
    source = output/spec['scenario']
    if sha(source) != spec['sha256']:
        raise ValueError('frozen scenario changed')
    frozen = json.loads(source.read_text())
    args = ['run', '--scenario', source] if spec['plant'] == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = dict(spec, expectation_met=False, recordings=str(folder.relative_to(output)))
    try:
        row['run_exit_code'] = execute(binary, args+['--seed', spec['seed'], '--output', folder/'run'], folder/'run-process')
        row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream:
            config = json.loads(next(stream))['header']['config']
            ticks = [r['tick'] for line in stream if 'tick' in (r := json.loads(line))]
        if config.get('gnss_recovery') != frozen.get('gnss_recovery'):
            raise ValueError('recovery calibration is not independent operational configuration')
        oracle = verify_ticks(config, ticks)
        physical_spec = dict(spec, calibrated=spec['mode'] in ['guarded', 'confirm'])
        row.update(physical(run, config, ticks, physical_spec, frozen, reconstruction=oracle))
        if spec['world'].startswith('map-bias') and spec['mode'] == 'guarded':
            stopped = min(f['truth']['speed'] for f in run['frames'] if 9.5 <= f['time'] < 11.4)
            healthy = any(t['input']['time'] > 13. and not t['expected']['health'] for t in ticks)
            row['fault'] = dict(minimum_actual_speed_m_s=stopped, healthy_gnss_recovery=healthy)
            if stopped > .05 or not healthy or oracle['completed_recoveries'] == 0 or oracle['rejected_fixes'] < 10:
                row['failed_gates'].append('fault_stop_recovery')
        if spec['mode'] == 'expiry':
            stopped = [f for f in run['frames'] if f['time'] >= 14.]
            if (not stopped or max(f['truth']['speed'] for f in stopped) > .05 or run['summary']['reached_goal']
                    or not oracle['expired'] or oracle['completed_recoveries'] or not any(t['input']['time'] > 19.
                        and t['expected']['localization']['last_decision'] == 'RecoveryHeld' for t in ticks)):
                row['failed_gates'].append('expired_hold_released')
        row['quality_passed'] = row['run_exit_code'] == 0 and not row['failed_gates']
        replayed = row['replay_exit_code'] == 0 and row['replay']['verified'] and row['replay']['ticks'] == len(ticks)
        if spec['mode'] == 'control':
            row['expectation_met'] = replayed and set(row['failed_gates']) <= {'source_acceptance'}
        else:
            row['expectation_met'] = replayed and row['quality_passed']
    except Exception as error:
        row['audit_error'] = repr(error)
    finally:
        row['storage'] = archive(folder)
    return row


def comparisons(rows):
    pairs = []
    for row in rows:
        if row['mode'] != 'guarded' or row['world'] not in ['north-bias', 'diagonal-bias']:
            continue
        control = next((r for r in rows if r['mode'] == 'control' and all(r[k] == row[k] for k in ['plant', 'world', 'seed'])), None)
        if control is None:
            continue
        reduction = 1.-row['maximum_error_m']/control['maximum_error_m'] if 'maximum_error_m' in row and control.get('maximum_error_m') else None
        pairs.append(dict(plant=row['plant'], world=row['world'], seed=row['seed'], maximum_error_reduction=reduction,
                          passed=bool(row.get('quality_passed') and control['expectation_met'] and reduction is not None and reduction >= .4)))
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args()
    output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:
        print('Frozen episodes:', len(manifest['episodes']))
        return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    rows = []
    for spec in manifest['episodes']:
        rows.append(run_case(output, spec, binaries[spec['plant']], args.binary.resolve()))
        pairs = comparisons(rows)
        report = dict(passed=all(r['expectation_met'] for r in rows) and all(p['passed'] for p in pairs),
                      complete=len(rows) == len(manifest['episodes']), binaries=provenance,
                      manifest_sha256=sha(output/'frozen-manifest.json'), episodes=rows, paired_controls=pairs)
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        row = rows[-1]
        print(row['plant'], row['world'], row['mode'], row['seed'], row['expectation_met'],
              row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['complete'] and report['passed'] else 1)


if __name__ == '__main__':
    main()
