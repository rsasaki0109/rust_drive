#!/usr/bin/env python3
"""Qualify sparse circular hypotheses without changing worlds or error budgets."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from functools import lru_cache
import json
import os
from pathlib import Path

from check_clear_sweep_recovery import audit as parent_audit
from check_gnss_map_history import sha
from check_relocalization_map_only import freeze, run_case
from check_road_width import validate_inputs
from check_track_initialization import compare
from sparse_circle_oracle import reconstruct

REPO = Path(__file__).resolve().parents[1]
PARENT = REPO/'assets/clear-sweep-recovery-results.json'
GATES = dict(parent_results_sha256=sha(PARENT), default_enabled=False,
             minimum_returns=3, maximum_sparse_returns=4, minimum_baseline_m=.5,
             minimum_triangle_altitude_m=.05, minimum_radius_m=.4, maximum_radius_m=2.5,
             maximum_radial_residual_m=.12, maximum_far_surface_projection_m=.12,
             unchanged='Original sensing/noise, worlds, goals/deadlines, motion budgets, clearance, road, accuracy and forecast coverage gates; all first acquisitions remain scored')


@lru_cache(maxsize=1)
def parent(): return json.loads(PARENT.read_text())


def baseline(spec):
    return next(r for r in parent()['final_matrix']['episodes'] if r['scenario'] == spec['parent_scenario'])


def specifications(plants):
    fields = ['cohort', 'plant', 'world', 'seed', 'mode', 'parent_mode', 'sweep_mode', 'family', 'heading', 'kind']
    return [dict({k:r[k] for k in fields if k in r}, mode=r['mode']+'-sparse', parent_scenario=r['scenario'])
            for r in parent()['final_matrix']['episodes'] if r['plant'] in plants and r['sweep_mode']=='recover']


def materialize(spec):
    old = baseline(spec); record = parent()['frozen_inputs'][old['scenario']]
    value = copy.deepcopy(record['input'])
    if value.get('fit_sparse_circles', False): raise ValueError('parent sparse policy changed')
    value['fit_sparse_circles'] = True
    return old['source'], value


def verify_policy(config, frozen, spec):
    if config.get('fit_sparse_circles') is not True or frozen.get('fit_sparse_circles') is not True:
        raise ValueError('declared sparse-circle fitting lost')
    old = baseline(spec); original = copy.deepcopy(frozen); original.pop('fit_sparse_circles')
    if original != parent()['frozen_inputs'][old['scenario']]['input']:
        raise ValueError('parent physical input or error budget changed')


def audit(run, config, ticks, spec, frozen):
    verify_policy(config, frozen, spec)
    result = parent_audit(run, config, ticks, spec, frozen)
    result['sparse_circles'] = reconstruct(config, ticks)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--freeze-only', action='store_true'); parser.add_argument('--only-index', type=int)
    suffix = '.exe' if os.name == 'nt' else ''
    parser.add_argument('--binary', type=Path, default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary', type=Path, default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args = parser.parse_args(); output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants), factory=materialize, gates=GATES)
    if args.freeze_only: return
    binaries = {'reference-2d':args.binary.resolve(), 'rne-dynamic':args.native_binary.resolve()}
    fingerprints = {p:sha(binaries[p]) for p in plants}; validate_inputs(output, manifest, args.binary.resolve())
    selected = list(enumerate(manifest['episodes']))
    if args.only_index is not None: selected = [selected[args.only_index]]
    def one(item):
        index, spec = item
        folder = output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row = run_case(output, spec, binaries[spec['plant']], args.binary.resolve(),
                       auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
        row['comparison'] = compare(row, baseline(spec))
        receipts = output/'episode-receipts'; receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2)+'\n')
        return row
    report = dict(complete=False, passed=False, gates=GATES, binaries=fingerprints,
                  manifest_sha256=sha(output/'frozen-manifest.json'), episodes=[])
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one, selected):
            report['episodes'].append(row); rows = report['episodes']
            report['complete'] = len(rows)==len(selected)
            report['contract_passed'] = all(not r.get('audit_error') and all(r.get(k,{}).get('passed') is True
                for k in ['sparse_circles','forecast_consistency','planar_sweeps','forecast_hold_physics'])
                and r.get('replay_exit_code')==0 and r.get('replay',{}).get('verified') is True
                and r['replay']['ticks']==r['summary']['steps']
                and r.get('run_exit_code')==(0 if r['summary']['passed'] else 1) for r in rows)
            report['regression_passed'] = all(r['comparison']['regression_passed'] for r in rows)
            report['strict_passes'] = sum(r['passed'] for r in rows)
            report['passed'] = report['complete'] and report['contract_passed'] and report['regression_passed']
            report['adoption_passed'] = report['passed'] and report['strict_passes']==len(rows)
            (output/('report.json' if args.only_index is None else f'report-{args.only_index}.json')).write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],'strict',row['passed'],row.get('audit_error'),
                  'new',row['comparison']['new_failed_gates'],flush=True)
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()): raise ValueError('runtime changed during qualification')
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
