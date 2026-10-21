#!/usr/bin/env python3
"""Qualify measured forecast braking while retaining every parent physical gate."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import json
import os
from pathlib import Path

from check_gnss_map_history import sha
from check_measurement_enclosure import audit as parent_audit, materialize as parent_input, specifications as parent_specs
from check_relocalization_map_only import freeze, run_case
from check_road_width import validate_inputs
from check_track_initialization import compare
from forecast_consistency_oracle import reconstruct

REPO = Path(__file__).resolve().parents[1]
PARENT = REPO/'assets/standalone-enclosure-results.json'
GATES = dict(minimum_recovery_acquisitions=3, minimum_recovery_span_s=.2, maximum_recovery_gap_s=.35,
             maximum_identities=128, references_per_identity=16, maximum_forecast_knots=64,
             physical_stop_deceleration_m_s2=3., physical_stop_settle_allowance_s=1.5,
             physical_hold_speed_m_s=.05, default_enabled=False, parent_results_sha256=sha(PARENT),
             physical_gates='All preceding source, accuracy, road, clearance, deadline and forecast coverage gates remain scored')


@lru_cache(maxsize=1)
def parent():return json.loads(PARENT.read_text())


def key(row):return tuple(row[k] for k in ['cohort', 'plant', 'world', 'seed', 'mode'])


def baseline(spec):return next(r for r in parent()['final_matrix']['episodes'] if key(r) == key(spec))


def specifications(plants):
    # Twenty parent conditions deliberately declare no forecast error envelope.
    # Preserve that unsupported configuration rather than adding calibration.
    return [s for s in parent_specs(plants, initialization='legacy') if parent_input(s)[1].get('forecast_envelope')]


def materialize(spec):
    source, value = parent_input(spec);old = baseline(spec)
    if value != parent()['frozen_inputs'][old['scenario']]['input']:raise ValueError('parent physical input changed')
    value['forecast_consistency_guard'] = True
    return source, value


def physical_holds(run, ticks):
    """Only the evaluation scorer sees physical speed. Short holds stay explicit."""
    intervals = [];start = None
    for i, tick in enumerate(ticks):
        blocked = 'InconsistentForecast' in tick['expected']['health']
        if blocked and start is None:start = i
        if start is not None and (not blocked or i == len(ticks)-1):
            end = i if blocked else i-1
            first, last = run['frames'][start], run['frames'][end]
            deadline = first['time']+GATES['physical_stop_settle_allowance_s']+first['truth']['speed']/GATES['physical_stop_deceleration_m_s2']
            samples = [f['truth']['speed'] for f in run['frames'][start:end+1] if f['time'] >= deadline-1e-9]
            intervals.append(dict(start_s=first['time'], end_s=last['time'], entry_speed_m_s=first['truth']['speed'],
                                  stop_deadline_s=deadline, scored_hold_samples=len(samples),
                                  maximum_actual_hold_speed_m_s=max(samples) if samples else None,
                                  stop_check_censored=not samples, passed=not samples or max(samples) <= GATES['physical_hold_speed_m_s']))
            start = None
    return dict(passed=all(r['passed'] for r in intervals), intervals=intervals,
                scored_intervals=sum(not r['stop_check_censored'] for r in intervals),
                censored_intervals=sum(r['stop_check_censored'] for r in intervals))


def audit(run, config, ticks, spec, frozen):
    if config.get('forecast_consistency_guard') is not True or frozen.get('forecast_consistency_guard') is not True:
        raise ValueError('declared forecast guard disappeared')
    result = parent_audit(run, config, ticks, spec, frozen)
    result['forecast_consistency'] = reconstruct(config, ticks)
    result['forecast_hold_physics'] = physical_holds(run, ticks)
    if not result['forecast_hold_physics']['passed']:result['failed_gates'].append('forecast_hold_physical_stop')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--only-index', type=int)
    parser.add_argument('--check-contract', action='store_true', help='Check monitor/brake correctness; retain strict physical and adoption failures in the report')
    suffix = '.exe' if os.name == 'nt' else ''
    parser.add_argument('--binary', type=Path, default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary', type=Path, default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args = parser.parse_args();output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants), factory=materialize, gates=GATES)
    if args.freeze_only:return
    binaries = {'reference-2d':args.binary.resolve(), 'rne-dynamic':args.native_binary.resolve()}
    fingerprints = {p:sha(binaries[p]) for p in plants};validate_inputs(output, manifest, args.binary.resolve())
    selected = list(enumerate(manifest['episodes']))
    if args.only_index is not None:selected = [selected[args.only_index]]
    def one(item):
        index, spec = item;folder = output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row = run_case(output, spec, binaries[spec['plant']], args.binary.resolve(),
                       auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s, _folder=str(folder)),f))
        row['comparison'] = compare(row, baseline(spec))
        receipts = output/'episode-receipts';receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row, indent=2)+'\n')
        return row
    report = dict(complete=False, passed=False, gates=GATES, binaries=fingerprints,
                  manifest_sha256=sha(output/'frozen-manifest.json'), episodes=[])
    report_path = output/('report.json' if args.only_index is None else f'report-{args.only_index}.json')
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one, selected):
            report['episodes'].append(row);report['complete'] = len(report['episodes']) == len(selected)
            report['contract_passed'] = all(not r.get('audit_error') and r.get('forecast_consistency',{}).get('passed') is True
                and r.get('forecast_hold_physics',{}).get('passed') is True and r.get('replay_exit_code') == 0
                and r.get('replay',{}).get('verified') is True and r['replay']['ticks'] == r['summary']['steps']
                and r['run_exit_code'] == (0 if r['summary']['passed'] else 1) for r in report['episodes'])
            report['regression_passed'] = all(r['comparison']['regression_passed'] for r in report['episodes'])
            report['strict_passes'] = sum(r['passed'] for r in report['episodes'])
            report['passed'] = report['complete'] and report['contract_passed'] and report['regression_passed']
            report['adoption_passed'] = report['passed'] and report['strict_passes'] == len(report['episodes'])
            report_path.write_text(json.dumps(report, indent=2)+'\n')
            print(row['plant'], row['world'], row['mode'], row['seed'], 'strict', row['passed'],
                  row.get('audit_error'), 'new failures', row['comparison']['new_failed_gates'], flush=True)
    if any(sha(binaries[p]) != h for p,h in fingerprints.items()):raise ValueError('runtime changed during qualification')
    raise SystemExit(0 if report['complete'] and (report['contract_passed'] if args.check_contract else report['passed']) else 1)


if __name__ == '__main__':main()
