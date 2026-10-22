#!/usr/bin/env python3
"""Qualify current-pose covariance occupancy without waiving earlier failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from functools import lru_cache
import json
import os
from pathlib import Path

from check_forecast_consistency import physical_holds
from check_forecast_envelope import audit as audit_fixed
from check_gnss_map_history import sha
from check_observed_detours import audit as audit_navigation
from check_relocalization_map_only import freeze,run_case
from check_road_width import road_margin,validate_inputs
from check_track_initialization import compare
from forecast_consistency_oracle import reconstruct as forecast_guard
from inspect_planning_progress import audit as progress
from measurement_enclosure_oracle import reconstruct as enclosure
from planar_sweep_oracle import audit_stream
from pose_forecast_oracle import reconstruct as pose_occupancy
from sparse_circle_oracle import reconstruct as sparse_geometry
from track_initialization_oracle import reconstruct as track_motion

REPO=Path(__file__).resolve().parents[1]
PARENTS=dict(detours=REPO/'assets/observed-detours-results.json',missions=REPO/'assets/planning-progress-results.json')
GATES=dict(default_enabled=False,sigma=3.,requires_absolute_heading=True,requires_motion_envelope=True,
    model='Current ENU pose ellipsoid propagated by the point-position Jacobian, plus exact-rotation Taylor remainder; conditional on localization covariance, no confidence calibration claim',
    require_all_original_gates=True,minimum_forecast_coverage=.99,minimum_coverage_samples=100,
    forecast_horizons_s=[.2,1.,2.,4.,8.],
    unchanged='Original complete world, sensing/noise, shape fits, motion budgets, goals/deadlines, clearance, road and accuracy gates; only forecast_pose_sigma is added')


@lru_cache(maxsize=2)
def parent(family):return json.loads(PARENTS[family].read_text())


def baseline(spec):
    return next(r for r in parent(spec['qualification_family'])['final_matrix']['episodes'] if r['scenario']==spec['baseline_scenario'])


def original(spec):
    p=parent(spec['qualification_family']);r=baseline(spec)
    if spec['qualification_family']=='detours':return next(v['contents'] for v in p['frozen_inputs']['episodes'] if v['scenario']==r['scenario'])
    return p['frozen_inputs'][r['scenario']]['input']


def specifications(plants):
    specs=[]
    fields=['plant','world','mode','seed','enabled','cohort','family','heading','kind','parent_mode','parent_scenario','baseline_scenario','sweep_mode']
    for family in PARENTS:
        for r in parent(family)['final_matrix']['episodes']:
            spec={k:r[k] for k in fields if k in r};spec.update(qualification_family=family,baseline_scenario=r['scenario'],mode=r['mode']+'-pose')
            if r['plant'] in plants and original(spec).get('absolute_heading_fusion'):specs.append(spec)
    return specs


def materialize(spec):
    value=copy.deepcopy(original(spec))
    if 'forecast_pose_sigma' in value:raise ValueError('preceding pose policy changed')
    value['forecast_pose_sigma']=GATES['sigma'];return baseline(spec)['source'],value


def verify_policy(config,frozen,spec):
    for value in [config,frozen]:
        if value.get('forecast_pose_sigma')!=GATES['sigma'] or type(value['forecast_pose_sigma']) not in [int,float]:
            raise ValueError('pose multiplier changed')
    value=copy.deepcopy(frozen);value.pop('forecast_pose_sigma')
    if value!=original(spec):raise ValueError('parent world, sensing, deadline or motion budget changed')
    if config.get('forecast_envelope')!=frozen['forecast_envelope']:raise ValueError('motion budgets changed')


def audit(run,config,ticks,spec,frozen):
    verify_policy(config,frozen,spec)
    return audit_dynamics(run,config,ticks,spec,frozen)


def audit_dynamics(run,config,ticks,spec,frozen):
    if spec['qualification_family']=='detours':result=audit_navigation(run,config,ticks,dict(spec,mode=baseline(spec)['mode']),frozen)
    else:
        result=audit_fixed(run,config,ticks,spec,frozen)
        result['expected_run_exit_code']=0 if run['summary']['passed'] else 1
        result.update(planning_progress=progress(config,ticks),forecast_consistency=forecast_guard(config,ticks),
            forecast_hold_physics=physical_holds(run,ticks),road_containment=road_margin(run),
            sparse_circles=sparse_geometry(config,ticks),track_motion=track_motion(config,ticks),
            measurement_enclosure=enclosure(config,ticks),planar_sweeps=audit_stream(config,ticks))
        if result['road_containment']['minimum_actual_road_margin_m']<.1:result['failed_gates'].append('physical_separation_road_reserve')
        if not result['forecast_hold_physics']['passed']:result['failed_gates'].append('forecast_hold_physical_stop')
    result['pose_occupancy']=pose_occupancy(config,ticks)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true');parser.add_argument('--only-index',type=int)
    parser.add_argument('--check-contract',action='store_true',help='verify covariance/occupancy provenance while retaining physical regression failures')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants),factory=materialize,gates=GATES)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};fingerprints={p:sha(binaries[p]) for p in plants}
    validate_inputs(output,manifest,args.binary.resolve());selected=list(enumerate(manifest['episodes']))
    if args.only_index is not None:selected=[selected[args.only_index]]
    def one(item):
        index,spec=item;folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
        row['comparison']=compare(row,baseline(spec));receipts=output/'episode-receipts';receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2)+'\n');return row
    report=dict(complete=False,passed=False,gates=GATES,binaries=fingerprints,manifest_sha256=sha(output/'frozen-manifest.json'),
        parent_results_sha256={family:sha(path) for family,path in PARENTS.items()},episodes=[])
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,selected):
            rows=report['episodes'];rows.append(row);report['complete']=len(rows)==len(selected)
            report['contract_passed']=all(not r.get('audit_error') and r.get('pose_occupancy',{}).get('passed') is True
                and r.get('replay_exit_code')==0 and r.get('replay',{}).get('verified') is True
                and r['replay']['ticks']==r['summary']['steps'] and r.get('run_exit_code')==(0 if r['summary']['passed'] else 1) for r in rows)
            report['regression_passed']=all(r['comparison']['regression_passed'] for r in rows)
            report['strict_passes']=sum(r['passed'] for r in rows)
            report['passed']=report['complete'] and report['contract_passed'] and report['regression_passed']
            report['adoption_passed']=report['passed'] and report['strict_passes']==len(rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],'goal',row.get('summary',{}).get('reached_goal'),
                row.get('audit_error',row.get('failed_gates')),'new',row['comparison']['new_failed_gates'],flush=True)
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()):raise ValueError('binary changed during qualification')
    raise SystemExit(0 if report['complete'] and report['contract_passed'] and (args.check_contract or report['passed']) else 1)


if __name__=='__main__':main()
