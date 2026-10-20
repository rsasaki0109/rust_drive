#!/usr/bin/env python3
"""Evaluate explicit synchronous heading sensing without replacing prior failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path

from check_forecast_envelope import coverage
from check_gnss_covariance import contains_declared
from check_gnss_latency import Filter, angle
from check_gnss_map_history import sha
from check_hazards import check_motion_predictions, check_speed_profiles
from check_obstruction_stops import certify_trajectory
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_road_width import (GATES as ROAD_GATES, materialize as road_input,
    specifications as road_specs, projector, road_margin, swept_clearance,
    validate_inputs, verify_targets)

REPO = Path(__file__).resolve().parents[1]
GATES = dict(ROAD_GATES, heading_nis=9., heading_age_s=.35,
             synthetic_variance_rad2=1e-4, heading_period_ticks=4,
             parent_heading_results_sha256=sha(REPO/'assets/fitted-chassis-recovery-results.json'))


def specifications(plants):
    rows = [dict(x, mode='heading', kind='base') for x in road_specs(plants) if x['mode']=='fitted']
    rows += [dict(plant=p, world=w, seed=131, mode='offset-'+name, kind='offset', offset_rad=sign*math.pi/60.)
             for p in plants for w in ['straight','curve','right'] for name,sign in [('minus',-1),('plus',1)]]
    rows += [dict(plant=p, world='straight', seed=171, mode=kind, kind=kind) for p in plants
             for kind in ['missing','jump','empty','gnss-loss','recover-missing','recover-jump','bias']]
    return rows


def materialize(spec):
    source, value = road_input(dict(spec, mode='fitted'))
    value['absolute_heading_fusion'] = True
    sensor = dict(period_ticks=4, variance_rad2=1e-4)
    kind = spec['kind']
    if kind=='offset': value['initial_heading_offset_rad'] = spec['offset_rad']
    if kind in ['missing','jump','empty','gnss-loss']:
        value.update(duration=12., expected='stop')
    if kind in ['missing','empty','recover-missing']:
        sensor['dropout_windows'] = [{'until':6. if kind=='recover-missing' else 12., 'from':0. if kind=='empty' else 4.}]
    if kind in ['jump','recover-jump','bias']:
        sensor['bias_windows'] = [{'from':0. if kind=='bias' else 4., 'until':6. if kind=='recover-jump' else value['duration'],
                                  'offset_rad':.2 if kind=='bias' else .8}]
    if kind=='gnss-loss': value['gnss_dropout_windows'] = [{'from':4.,'until':12.}]
    if kind=='empty':
        value['expected'] = 'fault'
    else:
        value['heading_sensor'] = sensor
    return source, value


def freeze(output, specs): return freeze_inputs(output, specs, factory=materialize, gates=GATES)


class HeadingFilter(Filter):
    """Dense Gaussian conditioning, independent of the Rust Joseph implementation."""
    def __init__(self, config):
        super().__init__(config)
        self.heading = dict(last_observed_stamp=None,last_accepted_stamp=None,last_decision=None,
                            last_nis=None,accepted_samples=0,rejected_samples=0)
        self.heading_fault = None

    def observe_heading(self, sample, now):
        h = self.heading
        if sample is not None:
            stamp,yaw,variance = (sample[k] for k in ['stamp','yaw_rad','variance_rad2'])
            valid = all(math.isfinite(x) for x in [stamp,yaw,variance]) and stamp>=0. and abs(yaw)<=math.pi and 0.<variance<=math.pi**2
            if not valid: self.heading_fault = (now,'InvalidAbsoluteHeading')
            elif h['last_observed_stamp'] is None or stamp>h['last_observed_stamp']:
                if abs(now-stamp)>1e-9: self.heading_fault = (now,'InvalidAbsoluteHeading')
                else:
                    residual = angle(yaw-self.x[2]); total = self.p[2][2]+variance; nis = residual*residual/total
                    h.update(last_observed_stamp=stamp,last_nis=nis)
                    if nis>9.:
                        h['last_decision']='RejectedInnovation'; h['rejected_samples']+=1
                        self.heading_fault=(now,'HeadingInnovationHold')
                    else:
                        column = [self.p[i][2] for i in range(3)]
                        self.x = [self.x[i]+column[i]/total*residual for i in range(3)]; self.x[2]=angle(self.x[2])
                        self.p = [[self.p[i][j]-column[i]*column[j]/total for j in range(3)] for i in range(3)]
                        h.update(last_decision='Accepted',last_accepted_stamp=stamp); h['accepted_samples']+=1
                        if self.heading_fault is None or stamp>self.heading_fault[0]: self.heading_fault=None
        issues = [] if self.heading_fault is None else [self.heading_fault[1]]
        if h['last_accepted_stamp'] is None or now-h['last_accepted_stamp']>.35+1e-9: issues.append('StaleAbsoluteHeading')
        return issues


def verify_covariance(actual, expected):
    if actual is None or len(actual)!=3 or any(len(row)!=3 for row in actual): raise ValueError('missing heading posterior')
    if any(not math.isfinite(actual[i][j]) or abs(actual[i][j]-actual[j][i])>1e-12 for i in range(3) for j in range(3)):
        raise ValueError('asymmetric or nonfinite heading posterior')
    a,b,c = actual[0]; _,d,e = actual[1]; _,_,f = actual[2]
    if a<=0. or a*d-b*b<=0. or a*(d*f-e*e)-b*(b*f-c*e)+c*(b*e-c*d)<=0.: raise ValueError('nonpositive heading posterior')
    difference=max(abs(actual[i][j]-expected[i][j]) for i in range(3) for j in range(3))
    if difference>1e-8: raise ValueError('heading posterior differs from independent conditioning')
    return difference


def verify_unactivated_heading_hold(run, ticks):
    """A missing required sensor must never emit an active driving profile."""
    if not ticks or len(run['frames'])!=len(ticks): raise ValueError('incomplete startup fault coverage')
    for frame,tick in zip(run['frames'],ticks):
        out=tick['expected']; command=out['command']; speed=frame['truth']['speed']
        if (tick['input'].get('absolute_heading') is not None or not out['emergency']
                or out['trajectory']['mode']!='Emergency' or 'StaleAbsoluteHeading' not in out['health']
                or not all(math.isfinite(command[k]) for k in ['acceleration','steering'])
                or command['acceleration']>-5.99 or command['steering']!=0.
                or not math.isfinite(speed) or abs(speed)>.05):
            raise ValueError('startup heading absence did not retain a complete emergency standstill')
    return dict(applicable=False,reason='No active trajectories; all-tick emergency standstill independently verified',
                active_trajectories=0,verified_emergency_ticks=len(ticks))


def reconstruct(config, ticks):
    if not config.get('absolute_heading_fusion') or config.get('gnss_latency_compensation') or config.get('relocalization') or config.get('localization_map'):
        raise ValueError('oracle requires synchronous single-epoch GNSS/heading input')
    oracle=HeadingFilter(config); previous=None; maximum=0.; health_counts={}
    for tick in ticks:
        inp,out=tick['input'],tick['expected']; now=inp['time']
        if previous is not None:
            if inp.get('odometry') is None: raise ValueError('uncovered motion in physical heading oracle')
            oracle.predict(inp['odometry'],now-previous)
        if inp.get('gnss') is not None: oracle.correct(inp['gnss'])
        issues=oracle.observe_heading(inp.get('absolute_heading'),now)
        actual=out['estimate']['pose']; delta=max(abs(actual['position'][k]-oracle.x[i]) for i,k in enumerate(['x','y']))
        delta=max(delta,abs(angle(actual['yaw']-oracle.x[2])),abs(out['estimate']['speed']-oracle.speed))
        maximum=max(maximum,delta)
        if delta>1e-8: raise ValueError(f'heading state differs at {now}: {delta}')
        verify_covariance(out.get('pose_covariance'),oracle.p)
        if abs(out['position_variance']-oracle.p[0][0]-oracle.p[1][1])>1e-8: raise ValueError('incorrect heading covariance trace')
        diagnostics=out.get('absolute_heading')
        if diagnostics is None: raise ValueError('missing independent heading diagnostics')
        for key,value in oracle.heading.items():
            if key=='last_nis' and value is not None:
                if diagnostics[key] is None or not math.isclose(diagnostics[key],value,rel_tol=1e-8,abs_tol=1e-8): raise ValueError('heading innovation differs')
            elif diagnostics[key]!=value: raise ValueError('heading diagnostic '+key+' differs')
        for key,value in [('last_observed_stamp',oracle.observed),('last_accepted_stamp',oracle.accepted_stamp),
                          ('last_decision',oracle.decision),('accepted_fixes',oracle.accepted),('rejected_fixes',oracle.rejected)]:
            if out['localization'][key]!=value: raise ValueError('heading changed GNSS diagnostic ownership')
        if oracle.nis is not None and not math.isclose(out['localization']['last_nis'],oracle.nis,rel_tol=1e-8,abs_tol=1e-8): raise ValueError('GNSS innovation differs')
        observed=[x for x in out['health'] if x in ['InvalidAbsoluteHeading','StaleAbsoluteHeading','HeadingInnovationHold']]
        if observed!=issues: raise ValueError('heading fault/age disagrees with acquisition history')
        for issue in issues: health_counts[issue]=health_counts.get(issue,0)+1
        if issues and out['command']['acceleration']>-5.99: raise ValueError('required heading fault did not brake')
        if oracle.accepted_stamp is None or now-oracle.accepted_stamp>.75+1e-9:
            if 'StaleGnss' not in out['health'] or out['command']['acceleration']>-5.99: raise ValueError('heading improperly refreshed GNSS')
        previous=now
    return dict(verified_ticks=len(ticks),maximum_state_difference=maximum,final_heading_diagnostics=oracle.heading,heading_health_counts=health_counts)


def audit(run,config,ticks,spec,frozen):
    if not contains_declared(run['scenario'],frozen) or not config.get('absolute_heading_fusion'):
        raise ValueError('physical scenario or declared heading mode differs')
    if {'heading_sensor','objects','truth','sensor_timing','full_rate_recording','duration','initial_heading_offset_rad'} & config.keys(): raise ValueError('simulator sensing parameters leaked to the driver')
    for key in ['forecast_envelope','lateral_candidate_margin_m','settle_obstruction_stops','local_route_geometry','motion_limits','initial_pose_covariance','gnss_covariance_shape','gnss_antenna_offset_m']:
        if config.get(key)!=frozen.get(key): raise ValueError('existing calibration/policy changed: '+key)
    first,second=config['route']['points'][:2]
    initial=config['initial_pose']
    if initial['position']!=first or abs(angle(initial['yaw']-math.atan2(second['y']-first['y'],second['x']-first['x'])))>1e-10:
        raise ValueError('physical initial yaw leaked into startup calibration')
    if spec['plant']=='rne-dynamic' and frozen.get('local_route_geometry') and config.get('rear_axle_offset_m',0.)<=0.:
        raise ValueError('native body/course reference missing')
    if run['route']!=config['route'] or run['vehicle']!=config['vehicle']: raise ValueError('different physical/operational geometry')
    if len(run['frames'])!=len(ticks) or len(ticks)!=run['summary']['steps']: raise ValueError('incomplete physical heading coverage')
    reconstruction=reconstruct(config,ticks); errors=[]; yaws=[]; certificates=0; certified=0; acquisitions=0
    sensor=frozen.get('heading_sensor')
    for i,(f,t) in enumerate(zip(run['frames'],ticks)):
        out=t['expected']; now=f['time']
        if abs(now-i*.05)>1e-8 or now!=out['time'] or out['estimate']!=f['estimate']: raise ValueError('physical/heading tick misalignment')
        errors.append(math.dist([f['truth']['pose']['position'][k] for k in ['x','y']],[out['estimate']['pose']['position'][k] for k in ['x','y']]))
        yaws.append(abs(angle(f['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
        sample=t['input'].get('absolute_heading'); available=sensor is not None and i%sensor['period_ticks']==0 and not any(now+1e-9>=w['from'] and now<w['until']-1e-9 for w in sensor.get('dropout_windows',[]))
        if (sample is not None)!=available: raise ValueError('heading acquisition/dropout schedule differs')
        if sample is not None:
            acquisitions+=1; bias=next((w['offset_rad'] for w in sensor.get('bias_windows',[]) if now+1e-9>=w['from'] and now<w['until']-1e-9),0.)
            if sample['stamp']!=now or sample['variance_rad2']!=sensor['variance_rad2'] or abs(angle(sample['yaw_rad']-f['truth']['pose']['yaw']-bias))>math.sqrt(3.*sensor['variance_rad2'])+1e-12:
                raise ValueError('heading observation is not the declared noisy acquisition')
        if i%10==0 and out['trajectory']['mode']!='Emergency': certificates+=certify_trajectory(out,config['vehicle']['radius'])['interval_bounds']; certified+=1
    summary=run['summary']; rmse=math.sqrt(sum(x*x for x in errors)/len(errors)); maximum=max(errors)
    if abs(rmse-summary['localization_rmse'])>1e-10 or abs(maximum-summary['localization_max_error'])>1e-10: raise ValueError('heading truth error omitted by summary')
    failed=[]
    if not summary['passed']: failed.append('source_acceptance')
    if rmse>.3 or maximum>.6 or max(yaws)>.1: failed.append('localization_accuracy')
    physical=swept_clearance(run); road=road_margin(run); target=verify_targets(config,ticks)
    if physical['minimum_swept_clearance_m']<1. or road['minimum_actual_road_margin_m']<.1 or any(summary.get(k,0) for k in ['collisions','road_violations','traffic_collisions','traffic_road_violations','intersection_violations']): failed.append('physical_separation_road_reserve')
    hold=None; passing=[]; locate=projector(run['route']['points']); kind=spec['kind']
    if spec['world']=='blocked' or kind in ['missing','jump','empty','gnss-loss']:
        start,end=(14.,20.) if spec['world']=='blocked' else (8.,12.)
        hold=max(f['truth']['speed'] for f in run['frames'] if start-1e-8<=f['time']<=end+1e-8)
        if hold>.05 or summary['reached_goal']: failed.append('required_fault_or_obstruction_hold')
    elif spec['world']!='mission':
        if not summary['reached_goal']: failed.append('goal_not_reached')
        passing=[locate(f['truth']['pose']['position'])[2] for f in run['frames'] if 33.<=locate(f['truth']['pose']['position'])[1]<=37.]
        if summary['reached_goal'] and (not passing or min(abs(x) for x in passing)<2.6 or (spec['world']=='right' and max(passing)>-2.6)): failed.append('incomplete_physical_side_pass')
    if kind.startswith('recover-'):
        hold=max(f['truth']['speed'] for f in run['frames'] if 5.5<=f['time']<=5.9+1e-9)
        if hold>.05: failed.append('missing_stop_before_heading_recovery')
    cov=coverage(run,ticks) if spec['world']=='mission' else None
    if cov and not cov['passed']: failed.append('forecast_coverage')
    folder=Path(spec['_folder'])
    profiles=verify_unactivated_heading_hold(run,ticks) if kind=='empty' else check_speed_profiles(folder/'run/sensors.jsonl')
    return dict(expected_run_exit_code=0,failed_gates=failed,heading_reconstruction=reconstruction,
        heading_acquisitions=acquisitions,independent_clearance=physical,road_containment=road,
        candidate_policy=target,rmse_m=rmse,maximum_error_m=maximum,maximum_yaw_rad=max(yaws),
        minimum_actual_passing_lateral_m=min(map(abs,passing),default=None),maximum_actual_hold_speed_m_s=hold,
        certified_planned_paths=certified,certified_interval_bounds=certificates,coverage=cov,
        motion_predictions=check_motion_predictions(folder/'run/sensors.jsonl'),speed_profiles=profiles)


def integration_passed(row):
    if row.get('audit_error') or not row.get('replay',{}).get('verified') or row.get('replay_exit_code')!=0: return False
    if row['replay'].get('ticks')!=row.get('summary',{}).get('steps'): return False
    if row.get('run_exit_code')!=(0 if row.get('summary',{}).get('passed') else 1): return False
    if row['kind']=='bias': return 'localization_accuracy' in row.get('failed_gates',[])
    if row['world']=='mission':
        parent=json.loads((REPO/'assets/fitted-chassis-recovery-results.json').read_text())['physical_matrix']['report']['episodes']
        previous=next(x for x in parent if x['plant']==row['plant'] and x['seed']==row['seed'] and x['world']=='mission' and x['mode']=='fitted')
        return not (set(row.get('failed_gates',[]))-set(previous['failed_gates']))
    return row.get('passed',False)


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',required=True,type=Path)
    p.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all'); p.add_argument('--freeze-only',action='store_true')
    p.add_argument('--check-integration-regression',action='store_true')
    p.add_argument('--binary',type=Path,default=REPO/'target/release/rustdriving'); p.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args=p.parse_args(); output=args.output.resolve(); plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants))
    if args.freeze_only: return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}; fingerprints={k:sha(binaries[k]) for k in plants}
    validate_inputs(output,manifest,args.binary.resolve()); rows=[]
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),auditor=lambda run,c,t,s,f:audit(run,c,t,dict(s,_folder=str(folder)),f))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            row['integration_regression_passed']=integration_passed(row); rows.append(row)
            report=dict(complete=len(rows)==len(manifest['episodes']),binaries=fingerprints,manifest_sha256=sha(output/'frozen-manifest.json'),episodes=rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row['integration_regression_passed'],row.get('audit_error',row.get('failed_gates')),flush=True)
    report['integration_regression_passed']=all(r['integration_regression_passed'] for r in rows)
    report['positive_qualification_passed']=all(r['passed'] for r in rows if r['kind']!='bias')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    assert all(sha(binaries[k])==fingerprints[k] for k in plants)
    if not (report['integration_regression_passed'] if args.check_integration_regression else report['positive_qualification_passed']): raise SystemExit(1)


if __name__=='__main__': main()
