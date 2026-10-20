#!/usr/bin/env python3
"""Freeze obstruction-stop regressions, certify emitted paths and retain adoption failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import os
from pathlib import Path

from check_forecast_envelope import BOUNDS, GATES as FORECAST_GATES, WORLDS, audit as forecast_audit, materialize as forecast_world
from check_gnss_map_history import sha
from check_hazards import prediction_radius_at
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_speed_zones import progress

REPO=Path(__file__).resolve().parents[1]
GATES=dict(FORECAST_GATES, blocked_hold_window_s=[14.,20.], release_hold_window_s=[10.,11.5],
           actual_hold_speed_m_s=.05, minimum_held_ticks=10, release_motion_s=12.,
           latest_release_s=22., fault_hold_window_s=[13.5,14.95],
           full_candidate_progress_tolerance_m=.25, minimum_hold_command_m_s2=-.5)
GATES['settling_entry_speed_m_s']=.2
GATES['source_worlds']='five preceding authored worlds plus two release/fault follow-ups; paired policy is the only within-world difference'
GATES['parent_results_sha256']=sha(REPO/'assets/forecast-envelope-results.json')


def specifications(plants,smoke=False):
    seeds=[7] if smoke else [7,42]
    ordinary=[dict(plant=p,world=w,seed=s,mode=m,settle=True) for p in plants for w in WORLDS
              for s in seeds for m in ['legacy','envelope']]
    releases=[dict(plant=p,world=w,seed=s,mode=m,settle=m=='enabled') for p in plants
              for w in ['release','release-fault'] for s in seeds for m in ['disabled','enabled']]
    return ordinary+releases


def materialize(spec):
    if spec['world'] in WORLDS:
        source,value=forecast_world(spec)
    else:
        source='scenarios/blocked.json';value=json.loads((REPO/source).read_text())
        value.update(name='Obstruction stop '+spec['world'],duration=65.,expected='goal',
                     full_rate_recording=True,forecast_envelope=copy.deepcopy(BOUNDS))
        value['objects'][0].update(lateral_speed=2.,moving_from=12.)
        if spec['world']=='release-fault':value['gnss_dropout_windows']=[dict(**{'from':12.,'until':15.})]
    if spec['settle']:value['settle_obstruction_stops']=True
    return source,value


def freeze(output,specs):return freeze_inputs(output,specs,factory=materialize,gates=GATES)


def certify_trajectory(output,vehicle_radius):
    """Certify the accelerated path with midpoint/Lipschitz lower bounds.

    Split at every forecast knot. On each resulting interval the object velocity
    and radius slope are constant. Ego speed is linear, so endpoint maxima bound
    its magnitude. Recursion rejects both a contact witness and unresolved bounds.
    This checks the acceleration integral directly, without Rust's chord reserve.
    """
    points=output['trajectory']['points'];tested=0;minimum=math.inf
    if len(points)<2:raise ValueError('a path without a complete time interval cannot be certified')
    def xy(p):return (p['x'],p['y'])
    for a,b in zip(points,points[1:]):
        duration=b['time']-a['time'];start=xy(a['position']);end=xy(b['position']);distance=math.dist(start,end)
        acceleration=(b['speed']-a['speed'])/duration
        tangent=tuple((end[k]-start[k])/distance for k in range(2)) if distance>1e-6 else (math.cos(output['estimate']['pose']['yaw']),math.sin(output['estimate']['pose']['yaw']))
        def ego_at(t):
            elapsed=t-a['time'];f=(a['speed']*elapsed+acceleration*elapsed*elapsed/2)/distance if distance>1e-12 else 0.
            return tuple(start[k]+(end[k]-start[k])*f for k in range(2))
        for obj in output['predictions']:
            positions=obj['positions'];dt=obj['dt'];knots=[a['time'],b['time']]
            knots+= [i*dt for i in range(len(positions)) if a['time']<i*dt<b['time']]
            knots=sorted(set(knots))
            for lower,upper in zip(knots,knots[1:]):
                mid=(lower+upper)/2;index=min(int(mid/dt),len(positions)-1);nxt=min(index+1,len(positions)-1)
                velocity=tuple((positions[nxt][k]-positions[index][k])/dt for k in ['x','y'])
                speed=math.hypot(*velocity)
                transverse=abs(velocity[0]*tangent[1]-velocity[1]*tangent[0])
                reserve=max(.4*min(transverse,1.),.7*transverse/speed) if speed else 0.
                radius_slope=abs(prediction_radius_at(obj,upper)-prediction_radius_at(obj,lower))/(upper-lower)
                def gap(t):
                    f=(t-index*dt)/dt if index!=nxt else 0.
                    center=tuple(positions[index][k]+(positions[nxt][k]-positions[index][k])*f for k in ['x','y'])
                    return math.dist(ego_at(t),center)-vehicle_radius-prediction_radius_at(obj,t)-.3-.06*min(t,5.)-reserve
                stack=[(lower,upper,0)]
                while stack:
                    lo,hi,depth=stack.pop();center=(lo+hi)/2;value=gap(center);tested+=1
                    maximum_speed=max(a['speed']+acceleration*(lo-a['time']),a['speed']+acceleration*(hi-a['time']))
                    bound=value-(maximum_speed+speed+radius_slope+.06)*(hi-lo)/2
                    minimum=min(minimum,value)
                    if not math.isfinite(value) or value < -1e-8:raise ValueError('emitted trajectory contacts a predicted occupied envelope')
                    if bound>1e-9:continue
                    if depth>=24:raise ValueError('trajectory clearance could not be certified')
                    stack.extend([(lo,center,depth+1),(center,hi,depth+1)])
    return dict(certified=True,interval_bounds=tested,minimum_sampled_margin_m=minimum if math.isfinite(minimum) else None)


def audit_contract(config,ticks):
    """Independent post-emergency no-restart property on the straight fixtures.

    No producer latch/phase diagnostics seed the fold. A clear release must span
    the original full planning horizon and independently pass the collision check.
    """
    armed=False;settling=False;arms=[];releases=[];held=0;violations=[];certificates=0
    route=config['route']['points'];length=config['route']['lengths'][-1]
    for tick in ticks:
        out=tick['expected'];trajectory=out['trajectory'];path=trajectory['points'];now=out['time']
        if out['health']:
            if out['command']['acceleration']>-5.99:violations.append(dict(stamp=now,reason='health_fault_brake'))
            continue
        if trajectory['mode']=='Emergency':
            if out['predictions'] and not armed:armed=True;arms.append(now)
            if armed and out['estimate']['speed']<=.2:settling=True
            continue
        certificate=certify_trajectory(out,config['vehicle']['radius']);certificates+=certificate['interval_bounds']
        if not armed:continue
        here=progress(out['estimate']['pose']['position'],route);span=progress(path[-1]['position'],route)-here
        complete=span>=min(40.,max(0.,length-here-1.))-.25
        if complete:armed=False;settling=False;releases.append(now);continue
        settling=settling or out['estimate']['speed']<=.2
        if not settling:continue
        held+=1
        if (out['command']['acceleration']>-.5+1e-8 or path[-1]['speed']!=0.
                or len(path)<2 or abs(path[-1]['time']-path[-2]['time']-8.)>1e-8
                or any(b['speed']>a['speed']+1e-9 for a,b in zip(path,path[1:]))):
            violations.append(dict(stamp=now,reason='restarted_on_partial_blocked_candidate'))
    return dict(arms=arms,releases=releases,held_ticks=held,violations=violations,
                passed=not violations,certified_interval_bounds=certificates)


def audit(run,config,ticks,spec,frozen):
    if config.get('settle_obstruction_stops',False)!=frozen.get('settle_obstruction_stops',False):
        raise ValueError('recorded obstruction policy differs from frozen input')
    result=forecast_audit(run,config,ticks,spec,frozen);target=spec['world'] in ['blocked','release','release-fault'] and spec['mode']!='legacy'
    contract=audit_contract(config,ticks) if target else None
    failed=[];window=None;release=None
    if target:
        lo,hi=GATES['blocked_hold_window_s'] if spec['world']=='blocked' else GATES['release_hold_window_s']
        speeds=[f['truth']['speed'] for f in run['frames'] if lo-1e-9<=f['time']<=hi+1e-9]
        window=dict(start_s=lo,end_s=hi,maximum_actual_speed_m_s=max(speeds),samples=len(speeds))
        if spec['settle']:
            if not contract['passed'] or contract['held_ticks']<10:failed.append('post_collision_hold_contract')
            if window['maximum_actual_speed_m_s']>.05:failed.append('actual_standstill')
        if spec['world']!='blocked':
            release=next((t['expected']['time'] for t in ticks if t['expected']['time']>=12. and t['expected']['command']['acceleration']>0.),None)
            if spec['settle'] and (release is None or release>22. or not run['summary']['reached_goal']):failed.append('clear_path_release_goal')
            if spec['world']=='release-fault':
                maximum=max(f['truth']['speed'] for f in run['frames'] if 13.5-1e-9<=f['time']<=14.95+1e-9)
                window['fault_maximum_actual_speed_m_s']=maximum
                if spec['settle'] and maximum>.05:failed.append('fault_standstill')
    parent=None
    if spec['world'] in WORLDS:
        parent=next(r for r in json.loads((REPO/'assets/forecast-envelope-results.json').read_text())['physical_report']['episodes']
                    if all(r[k]==spec[k] for k in ['plant','world','mode','seed']))
        # Keep every source/calibration failure, including the four preceding
        # incomplete missions. Regression means no *new* failed acceptance gate.
        if parent['summary']['passed'] and not run['summary']['passed']:failed.append('new_source_acceptance_failure')
        if set(result['failed_gates'])-set(parent['failed_gates']):failed.append('new_physical_or_accuracy_failure')
    elif not run['summary']['passed']:failed.append('source_acceptance')
    if run['summary']['collisions'] or run['summary']['road_violations'] or result['independent_clearance'].get('passed') is False:
        failed.append('physical_separation')
    result.update(policy_contract=contract,actual_hold_window=window,release_command_s=release,
                  policy_regression_failures=failed,policy_regression_passed=not failed,
                  parent_source_passed=parent['summary']['passed'] if parent else None)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--smoke',action='store_true');parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--check-policy-regression',action='store_true',help='exit on policy regression, retaining separate complete-qualification failures')
    parser.add_argument('--binary',type=Path,default=REPO/'target/release'/('rustdriving.exe' if os.name=='nt' else 'rustdriving'))
    parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args=parser.parse_args();output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants,args.smoke))
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    fingerprints={p:dict(path=str(binaries[p]),sha256=sha(binaries[p])) for p in plants};rows=[]
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
                        auditor=lambda run,config,ticks,row,frozen:audit(run,config,ticks,dict(row,_folder=str(folder)),frozen))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            rows.append(row);enabled=[r for r in rows if r['settle']]
            regression=all(r.get('policy_regression_passed',False) and r.get('replay',{}).get('verified',False)
                           and r.get('replay_exit_code')==0 and r['replay']['ticks']==r['summary']['steps'] for r in enabled)
            report=dict(complete=len(rows)==len(manifest['episodes']),passed=all(r['passed'] for r in rows),
                        policy_regression_passed=regression,manifest_sha256=sha(output/'frozen-manifest.json'),binaries=fingerprints,episodes=rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row.get('policy_regression_passed'),row.get('audit_error',row.get('policy_regression_failures')),flush=True)
    assert all(sha(binaries[p])==fingerprints[p]['sha256'] for p in plants)
    if not (report['policy_regression_passed'] if args.check_policy_regression else report['passed']):raise SystemExit(1)


if __name__=='__main__':main()
