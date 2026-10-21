#!/usr/bin/env python3
"""Qualify bounded intermediate lateral proposals and retain inherited failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from functools import lru_cache
import json
import math
import os
from pathlib import Path

from check_absolute_heading import reconstruct as heading_oracle
from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_measurement_enclosure import audit as parent_audit, materialize as parent_input, specifications as parent_specs
from check_obstruction_stops import certify_trajectory
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_road_width import projector, road_margin, swept_clearance, validate_inputs, verify_targets

REPO=Path(__file__).resolve().parents[1]
PARENT=REPO/'assets/standalone-enclosure-results.json'
GATES=dict(minimum_physical_clearance_m=1.,minimum_road_reserve_m=.1,
           rmse_m=.3,maximum_error_m=.6,maximum_yaw_rad=.1,
           minimum_refined_forecast_clearance_m=1.,maximum_candidates=9,
           offset_fractions=[0.,1.,-1.,.875,-.875,.75,-.75,.5,-.5],
           certificate_cadence_ticks=10,original_mission_deadline_s=65.,new_corridor_deadline_s=45.,
           parent_results_sha256=sha(PARENT),forecast_budgets_unchanged=True,default_enabled=False)


@lru_cache(maxsize=1)
def parent():return json.loads(PARENT.read_text())


def key(spec):return tuple(spec[k] for k in ['plant','cohort','world','seed','mode'])


def baseline(spec):
    return next(r for r in parent()['final_matrix']['episodes'] if key(r)==key(spec))


def specifications(plants,suite='all'):
    rows=[]
    if suite in ['all','corridor']:
        rows += [dict(plant=p,world='corridor-'+w,seed=s,mode='refined' if enabled else 'three',
                      evaluation_family='corridor',refined=enabled,curve_amplitude=amplitude)
                 for p in plants for w,amplitude in [('straight',0.),('curve',2.)]
                 for s in [7,42] for enabled in [False,True]]
    if suite in ['all','regression']:
        rows += [dict(s,evaluation_family='regression',refined=True) for s in parent_specs(plants,initialization='legacy')
                 if parent_input(s)[1].get('lateral_candidate_margin_m') is not None]
    return rows


def materialize(spec):
    if spec['evaluation_family']=='regression':
        source,value=parent_input(spec)
        old=baseline(spec)
        if value!=parent()['frozen_inputs'][old['scenario']]['input']:
            raise ValueError('preceding world, sensing, calibration or deadlines changed')
    else:
        source='scenarios/blocked.json';value=json.loads((REPO/source).read_text())
        value.update(name='Intermediate lateral '+spec['world']+' '+spec['mode'],duration=45.,expected='goal',
                     road_length=100.,half_width=5.5,curve_amplitude=spec['curve_amplitude'],cruise_speed=4.,
                     full_rate_recording=True,local_route_geometry=True,lateral_candidate_margin_m=.35,
                     motion_limits=dict(max_acceleration_m_s2=2.,max_deceleration_m_s2=2.5,max_lateral_acceleration_m_s2=2.5),
                     absolute_heading_fusion=True,heading_sensor=dict(period_ticks=4,variance_rad2=1e-4),
                     confirm_track_initialization=True,enclose_track_measurements=True,
                     objects=[dict(s=45.,lateral=0.,radius=.5),dict(s=45.,lateral=7.1,radius=1.5),dict(s=45.,lateral=-7.1,radius=1.5)])
    value['refine_lateral_candidates']=spec['refined']
    return source,value


def verify_policy(actual,config,frozen):
    recorded=copy.deepcopy(actual);recorded.setdefault('refine_lateral_candidates',False)
    if not contains_declared(recorded,frozen):raise ValueError('physical input differs from frozen source')
    value=config.get('refine_lateral_candidates',False)
    if type(value) is not bool or value is not frozen['refine_lateral_candidates']:
        raise ValueError('intermediate-candidate policy changed or disappeared')
    if config.get('lateral_candidate_margin_m')!=frozen.get('lateral_candidate_margin_m'):
        raise ValueError('supplied road-width reserve changed')


def certificates(config,ticks):
    enabled=config.get('refine_lateral_candidates',False)
    checked=bounds=intermediate=refined_checked=0
    side=min(3.5,max(0.,config['route']['half_width']-config['vehicle']['radius']-config['lateral_candidate_margin_m']))
    locate=projector(config['route']['points'])
    for i,tick in enumerate(ticks):
        out=tick['expected'];target=out['trajectory']['lateral_target']
        if not math.isfinite(target):raise ValueError('nonfinite lateral target')
        refined=enabled and abs(target)>.1 and abs(abs(target)-side)>1e-10
        if refined:intermediate+=1
        if not all(math.isfinite(out['command'][k]) for k in ['acceleration','steering']):
            raise ValueError('nonfinite candidate command')
        if out['health'] or out['trajectory']['mode']=='Emergency':
            if out['command']['acceleration']>-5.99:raise ValueError('fault or emergency did not brake')
            continue
        if refined:
            here=locate(out['estimate']['pose']['position'])[1]
            span=locate(out['trajectory']['points'][-1]['position'])[1]-here
            required=min(40.,max(0.,config['route']['lengths'][-1]-here-1.))-.25
            if span<required:raise ValueError('refined path does not clear the complete planning horizon')
        if i%10:continue
        proof=certify_trajectory(out,config['vehicle']['radius'],minimum_clearance_m=1. if refined else 0.)
        refined_checked+=refined
        checked+=1;bounds+=proof['interval_bounds']
    return dict(verified_trajectories=checked,interval_bounds=bounds,minimum_refined_forecast_clearance_m=1.,
                verified_refined_trajectories=refined_checked,
                intermediate_control_ticks=intermediate,cadence_ticks=10)


def corridor_audit(run,config,ticks,spec,frozen):
    if {'heading_sensor','truth','objects','sensor_timing'} & config.keys():
        raise ValueError('simulation labels entered operational calibration')
    for field in ['confirm_track_initialization','enclose_track_measurements','absolute_heading_fusion','local_route_geometry']:
        if config.get(field) is not True:raise ValueError('declared sensor/geometry policy disappeared')
    if config.get('forecast_envelope'):raise ValueError('undeclared future-motion budget')
    frames,summary=run['frames'],run['summary']
    if len(frames)!=len(ticks) or len(ticks)!=summary['steps']:raise ValueError('incomplete full-rate physical evidence')
    errors=[];yaws=[]
    for i,(frame,tick) in enumerate(zip(frames,ticks)):
        inp,out=tick['input'],tick['expected'];t=i*.05
        if abs(frame['time']-t)>1e-8 or inp['time']!=frame['time'] or out['estimate']!=frame['estimate']:
            raise ValueError('physical and control clocks differ')
        sample=inp.get('absolute_heading')
        if bool(sample)!=(i%4==0):raise ValueError('declared heading acquisition changed')
        if sample and (sample['stamp']!=frame['time'] or sample['variance_rad2']!=1e-4
                       or abs(angle(sample['yaw_rad']-frame['truth']['pose']['yaw']))>math.sqrt(3e-4)+1e-12):
            raise ValueError('heading does not match the declared noisy acquisition')
        error=math.dist([frame['truth']['pose']['position'][k] for k in ['x','y']],[out['estimate']['pose']['position'][k] for k in ['x','y']])
        yaw=abs(angle(frame['truth']['pose']['yaw']-out['estimate']['pose']['yaw']))
        if not math.isfinite(error) or not math.isfinite(yaw):raise ValueError('nonfinite physical accuracy')
        errors.append(error);yaws.append(yaw)
    oracle=heading_oracle(config,ticks)
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors));maximum=max(errors)
    if abs(rmse-summary['localization_rmse'])>1e-10 or abs(maximum-summary['localization_max_error'])>1e-10:
        raise ValueError('complete localization errors were removed')
    clearance,road=swept_clearance(run),road_margin(run)
    failed=[]
    if not summary['passed']:failed.append('source_acceptance')
    if not summary['reached_goal'] or summary['simulated_seconds']>45.:failed.append('goal_deadline')
    if rmse>.3 or maximum>.6 or max(yaws)>.1:failed.append('localization_accuracy')
    if clearance['minimum_swept_clearance_m']<1. or summary['collisions']:failed.append('physical_clearance')
    if road['minimum_actual_road_margin_m']<.1 or summary['road_violations']:failed.append('road_reserve')
    if summary['reached_goal']:
        locate=projector(run['route']['points']);length=run['route']['lengths'][-1]
        final=frames[-1]
        if locate(final['truth']['pose']['position'])[1]<length-2. or final['truth']['speed']>.2:
            failed.append('independent_goal_stop')
    return dict(failed_gates=failed,oracle=oracle,rmse_m=rmse,maximum_error_m=maximum,maximum_yaw_rad=max(yaws),
                independent_clearance=clearance,road_containment=road)


def audit(run,config,ticks,spec,frozen):
    verify_policy(run['scenario'],config,frozen)
    result=(parent_audit if spec['evaluation_family']=='regression' else corridor_audit)(run,config,ticks,spec,frozen)
    result['targets']=verify_targets(config,ticks)
    result['candidate_certificates']=certificates(config,ticks)
    strict=list(result['failed_gates'])
    result['strict_failed_gates']=strict
    result['strict_passed']=not strict
    result['expected_run_exit_code']=0 if run['summary']['passed'] else 1
    if spec['evaluation_family']=='regression':
        old=baseline(spec)
        result['baseline_failed_gates']=old['failed_gates']
        result['new_failed_gates']=sorted(set(strict)-set(old['failed_gates']))
        result['failed_gates']=result['new_failed_gates']
    elif not spec['refined']:
        # Comparison processing is checked, while every actual failed gate remains above.
        result['failed_gates']=[]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--suite',choices=['all','corridor','regression'],default='all')
    parser.add_argument('--freeze-only',action='store_true')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve()
    plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze_inputs(output,specifications(plants,args.suite),factory=materialize,gates=GATES)
    if args.freeze_only:return
    validate_inputs(output,manifest,args.binary.resolve())
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    rows=[]
    report=dict(complete=False,gates=GATES,manifest_sha256=sha(output/'frozen-manifest.json'),episodes=rows,
                binaries={p:dict(path=str(binaries[p]),sha256=sha(binaries[p])) for p in plants})
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
                        auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            rows.append(row);report['complete']=len(rows)==len(manifest['episodes'])
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row['passed'],row.get('audit_error',row.get('strict_failed_gates')),flush=True)
    report.update(passed=all(r['passed'] for r in rows),
                  refined_corridor_strict_passes=sum(r.get('strict_passed',False) for r in rows if r['evaluation_family']=='corridor' and r['refined']),
                  original_mission_goals_reached=sum(r['summary']['reached_goal'] for r in rows if r['evaluation_family']=='regression' and r['world']=='mission'))
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if report['complete'] and report['passed'] else 1)


if __name__=='__main__':main()
