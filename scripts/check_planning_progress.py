#!/usr/bin/env python3
"""Qualify acquired progress status with byte-identical preceding driving behavior."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path

from check_gnss_map_history import sha
from check_recent_track_motion import audit as parent_audit
from check_relocalization_map_only import freeze, run_case
from check_road_width import validate_inputs
from inspect_planning_progress import audit as inspect, project_output
from comparison_control import Control

REPO=Path(__file__).resolve().parents[1]
PARENT=REPO/'assets/recent-track-motion-results.json'
GATES=dict(parent_results_sha256=sha(PARENT),default_enabled=False,
    minimum_acquisitions=3,minimum_acquired_dwell_s=2.,maximum_gap_s=.35,
    maximum_speed_m_s=.2,maximum_anchor_displacement_m=.2,maximum_route_progress_change_m=.2,
    maximum_prediction_identities=9,minimum_remaining_route_m=2.,
    require_no_available_complete_candidate=True,require_selected_stationary_checked_stop=True,
    require_same_sorted_blockers=True,require_nonpositive_acceleration=True,
    require_current_control_anchor_and_blockers=True,reconstruct_mapped_yield_ownership=True,
    require_identical_operational_output_bytes=True,require_identical_physical_frames=True,
    unchanged='Original worlds, sensors/noise, deadlines, motion/error budgets, clearance, road, accuracy and coverage gates')


@lru_cache(maxsize=1)
def parent():return json.loads(PARENT.read_text())


def baseline(spec):return next(r for r in parent()['final_matrix']['episodes'] if r['scenario']==spec['progress_baseline_scenario'])


def specifications(plants):
    fields=['cohort','plant','world','seed','parent_mode','sweep_mode','family','heading','kind','parent_scenario','baseline_scenario']
    return [dict({k:r[k] for k in fields if k in r},mode=r['mode']+'-progress',progress_baseline_scenario=r['scenario'])
            for r in parent()['final_matrix']['episodes'] if r['plant'] in plants]


def materialize(spec):
    old=baseline(spec);value=copy.deepcopy(parent()['frozen_inputs'][old['scenario']]['input'])
    if value.get('monitor_planning_progress') or value.get('record_planning_diagnostics'):
        raise ValueError('preceding evidence already contains the added status/capture policies')
    value.update(monitor_planning_progress=True,record_planning_diagnostics=True)
    return old['source'],value


def original_input(frozen):
    result=copy.deepcopy(frozen)
    for name in ['monitor_planning_progress','record_planning_diagnostics']:
        if result.pop(name,None) is not True:raise ValueError('declared progress/capture policy lost')
    return result


def verify_policy(config,frozen,spec):
    if any(config.get(k) is not True for k in ['monitor_planning_progress','record_planning_diagnostics']):
        raise ValueError('operational progress/capture policy lost')
    if original_input(frozen)!=parent()['frozen_inputs'][baseline(spec)['scenario']]['input']:
        raise ValueError('preceding world, deadline or budget changed')


def archive(control,old,name):
    row=control.episode(old,parent()['frozen_inputs'][old['scenario']]['input'])
    return control.archive(row,name)[0]


def audit(run,config,ticks,spec,frozen):
    verify_policy(config,frozen,spec)
    result=parent_audit(run,config,ticks,spec,original_input(frozen))
    result['planning_progress']=inspect(config,ticks)
    old=baseline(spec);folder=Path(spec['_folder']);control=spec['_control']
    preceding=json.loads(archive(control,old,'run/run.json'))
    digest=hashlib.sha256();count=0
    with (folder/'replay/outputs.jsonl').open('rb') as stream:
        for line in stream:digest.update(project_output(line));count+=1
    expected=hashlib.sha256(archive(control,old,'replay/outputs.jsonl')).hexdigest()
    if digest.hexdigest()!=expected or count!=preceding['summary']['steps']:
        raise ValueError('progress observer changed operational output bytes')
    if preceding['frames']!=run['frames'] or preceding['summary']!=run['summary']:
        raise ValueError('progress observer changed physical frames or complete outcomes')
    original=[json.loads(line)['tick']['input'] for line in archive(control,old,'run/sensors.jsonl').splitlines() if 'tick' in json.loads(line)]
    if original!=[t['input'] for t in ticks]:raise ValueError('progress observer changed acquired input values or clocks')
    blocked=[f for f,t in zip(run['frames'],ticks) if t['expected']['planning_progress']['state']=='StationaryForecastBlocked']
    maximum=max((abs(f['truth']['speed']) for f in blocked),default=0.)
    if maximum>.2+1e-9:raise ValueError('stationary status does not describe physical standstill in this cohort')
    result['progress_physics']=dict(passed=True,blocked_samples=len(blocked),maximum_actual_speed_m_s=maximum,evaluation_only=True)
    result['capture_compatibility']=dict(passed=True,operational_output_sha256=expected,
        projected_output_sha256=digest.hexdigest(),verified_output_ticks=count,
        identical_physical_frames=True,identical_summary=True,identical_sensor_inputs=True,
        control_binary_sha256=control.binaries[spec['plant']],
        historical_output_sha256=old['storage']['replay/outputs.jsonl']['sha256'])
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline-output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants),factory=materialize,gates=GATES)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    control=Control(args.baseline_output,{p:binaries[p] for p in plants})
    fingerprints={p:sha(binaries[p]) for p in plants};validate_inputs(output,manifest,args.binary.resolve())
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
            auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder),_control=control),f))
    report=dict(complete=False,passed=False,gates=GATES,binaries=fingerprints,
        manifest_sha256=sha(output/'frozen-manifest.json'),episodes=[])
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            rows=report['episodes'];rows.append(row);report['complete']=len(rows)==len(manifest['episodes'])
            report['passed']=report['complete'] and all(not r.get('audit_error') and
                all(r.get(k,{}).get('passed') is True for k in ['planning_progress','progress_physics','capture_compatibility'])
                and r.get('replay_exit_code')==0 and r.get('replay',{}).get('verified') is True
                and r['replay']['ticks']==r['summary']['steps']
                and r.get('run_exit_code')==(0 if r['summary']['passed'] else 1) for r in rows)
            report['strict_passes']=sum(r['passed'] for r in rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['seed'],'blocked',row.get('planning_progress',{}).get('blocked_control_ticks'),row.get('audit_error'),flush=True)
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()):raise ValueError('runtime changed during qualification')
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
