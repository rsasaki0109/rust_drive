#!/usr/bin/env python3
"""Freeze paired motion-envelope trials and retain physical and coverage failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import os
from pathlib import Path

from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_hazards import check_motion_predictions, check_speed_profiles, prediction_radius_at
from check_lane_change_speeds import sweep
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_yield_relocalization import physical_yields, verify_yields

REPO = Path(__file__).resolve().parents[1]
BOUNDS = dict(position_error_m=.2, velocity_error_m_s=.3, acceleration_error_m_s2=.1)
WORLDS = ['mission', 'slow-crossing', 'traffic-lead-stop', 'intersection-crossing', 'blocked']
HORIZONS = [.2, 1., 2., 4., 8.]
GATES = dict(clearance_m=1., rmse_m=.3, maximum_error_m=.6, maximum_yaw_rad=.1,
             minimum_coverage=.99, minimum_samples_per_horizon=100, association_distance_m=2.,
             association_gap_m=.5, horizons_s=HORIZONS, bounds=BOUNDS,
             deadlines='unchanged source scenario duration',
             source_worlds='unchanged except forecast_envelope and evaluation-only full_rate_recording')


def specifications(plants, smoke=False):
    return [dict(plant=p, world=w, seed=s, mode=m) for p in plants for w in WORLDS
            for s in ([7] if smoke else [7, 42]) for m in ['legacy', 'envelope']]


def materialize(spec):
    source = f"scenarios/{spec['world']}.json"
    value = json.loads((REPO/source).read_text())
    value['full_rate_recording'] = True
    if spec['mode'] == 'envelope':
        value['forecast_envelope'] = copy.deepcopy(BOUNDS)
    return source, value


def freeze(output, specs):
    return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def coverage(run, ticks):
    """Evaluation-only associations to physical actors; never driver inputs.

    Use each acquired identity once. Ambiguous/unmatched acquisitions remain in
    the denominator. Report absence/end-of-record censoring explicitly. There
    is no nearest-future reselection after the acquisition's association.
    """
    frames = run['frames'];seen = set()
    rows = {str(h): dict(eligible=0, matched=0, covered=0, unmatched=0, actor_absent=0,
                        censored=0, maximum_shortfall_m=0.) for h in HORIZONS}
    def xy(p):return tuple(p[a] for a in ['x', 'y'])
    for index, tick in enumerate(ticks):
        out = tick['expected'];tracks = {t['id']:t for t in out['tracks']}
        for prediction in out['predictions']:
            track = tracks[prediction['id']];key = (track['id'], track['last_seen'])
            if key in seen:continue
            seen.add(key)
            acquired = round(track['last_seen']/.05)
            if acquired < 0 or acquired > index or abs(frames[acquired]['time']-track['last_seen']) > 1e-8:
                raise ValueError('coverage acquisition is not a recorded physical tick')
            candidates = sorted((math.dist(xy(track['position']), xy(o['position'])), o['id'])
                                for o in frames[acquired]['objects'])
            actor = candidates[0][1] if (candidates and candidates[0][0] <= 2.
                    and (len(candidates) == 1 or candidates[1][0]-candidates[0][0] >= .5)) else None
            for horizon in HORIZONS:
                row = rows[str(horizon)];future = index+round(horizon/.05)
                if future >= len(frames):row['censored'] += 1;continue
                row['eligible'] += 1
                if actor is None:row['unmatched'] += 1;continue
                actual = next((o for o in frames[future]['objects'] if o['id'] == actor), None)
                if actual is None:row['actor_absent'] += 1;continue
                row['matched'] += 1
                step = horizon/prediction['dt'];i = min(int(math.floor(step)), len(prediction['positions'])-2)
                f = step-i
                a, b = prediction['positions'][i:i+2]
                center = tuple((1-f)*a[k]+f*b[k] for k in ['x', 'y'])
                shortfall = math.dist(center, xy(actual['position']))+actual['radius']-prediction_radius_at(prediction, horizon)
                row['covered'] += shortfall <= 1e-8
                row['maximum_shortfall_m'] = max(row['maximum_shortfall_m'], shortfall)
    for row in rows.values():
        row['coverage_fraction'] = row['covered']/row['eligible'] if row['eligible'] else 0.
        row['passed'] = row['eligible'] >= 100 and row['coverage_fraction'] >= .99
    return dict(horizons=rows, passed=all(r['passed'] for r in rows.values()),
                acquired_identities=len(seen), evaluation_only=True)


def audit(run, config, ticks, spec, frozen):
    if not contains_declared(run['scenario'], frozen) or config.get('forecast_envelope') != frozen.get('forecast_envelope'):
        raise ValueError('frozen world or declared error budget changed')
    if {'truth', 'objects', 'world_motion', 'following', 'duration', 'sensor_timing', 'full_rate_recording'} & config.keys():
        raise ValueError('simulation truth or schedules entered the operational configuration')
    frames, summary = run['frames'], run['summary']
    if len(frames) != len(ticks) or len(ticks) != summary['steps']:
        raise ValueError('incomplete physical/control recording')
    errors, yaws = [], []
    for index, (frame, tick) in enumerate(zip(frames, ticks)):
        out = tick['expected']
        if abs(frame['time']-index*.05) > 1e-8 or out['time'] != frame['time'] or frame['estimate'] != out['estimate']:
            raise ValueError('physical/control clock or localization differs')
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x', 'y']],
                                [out['estimate']['pose']['position'][k] for k in ['x', 'y']]))
        yaws.append(abs(angle(frame['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
    rmse = math.sqrt(sum(e*e for e in errors)/len(errors));maximum = max(errors)
    if abs(rmse-summary['localization_rmse']) > 1e-10 or abs(maximum-summary['localization_max_error']) > 1e-10:
        raise ValueError('summary omitted physical accuracy failures')
    failed = []
    if not summary['passed']:failed.append('source_acceptance')
    if rmse > .3 or maximum > .6 or max(yaws) > .1:failed.append('localization_accuracy')
    if summary['simulated_seconds'] > frozen['duration']+1e-9:failed.append('source_deadline')
    physical = {}
    try:physical = sweep(run)
    except ValueError as error:physical = dict(passed=False, error=str(error));failed.append('swept_clearance')
    for key in ['collisions', 'road_violations', 'traffic_collisions', 'traffic_road_violations', 'intersection_violations']:
        if summary.get(key, 0):failed.append(key)
    folder = Path(spec['_folder'])
    forecasts = check_motion_predictions(folder/'run/sensors.jsonl')
    profiles = check_speed_profiles(folder/'run/sensors.jsonl')
    if config.get('yield_intersections'):
        zones = verify_yields(config, ticks)
        physical.update(priority=physical_yields(run, zones['records']))
    actual_coverage = coverage(run, ticks)
    driving_passed = not failed
    if not actual_coverage['passed']:failed.append('forecast_coverage')
    return dict(expected_run_exit_code=0, failed_gates=failed, driving_passed=driving_passed,
                motion_predictions=forecasts, speed_profiles=profiles, independent_clearance=physical,
                coverage=actual_coverage, rmse_m=rmse, maximum_error_m=maximum, maximum_yaw_rad=max(yaws),
                acquired_forecasts_verified=forecasts['verified'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--smoke', action='store_true');parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args();output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:return
    binaries = {'reference-2d':args.binary.resolve(), 'rne-dynamic':args.native_binary.resolve()}
    fingerprints = {p:dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    def one(spec):
        folder = output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output, spec, binaries[spec['plant']], args.binary.resolve(),
                        auditor=lambda run, config, ticks, row, frozen:audit(run, config, ticks, dict(row, _folder=str(folder)), frozen))
    rows = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one, manifest['episodes']):
            rows.append(row)
            report = dict(complete=len(rows)==len(manifest['episodes']), passed=all(r['passed'] for r in rows),
                          manifest_sha256=sha(output/'frozen-manifest.json'), binaries=fingerprints, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(row['plant'], row['world'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    if any(sha(binaries[p]) != fingerprints[p]['sha256'] for p in plants):raise ValueError('runtime changed during evaluation')
    if not report['passed']:raise SystemExit(1)


if __name__ == '__main__':main()
