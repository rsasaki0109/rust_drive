#!/usr/bin/env python3
"""Freeze paired road-width candidate trials and retain all qualification failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import subprocess

from check_forecast_envelope import BOUNDS, coverage
from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_hazards import check_motion_predictions, check_speed_profiles
from check_obstruction_stops import certify_trajectory
from check_relocalization_map_only import freeze as freeze_inputs, run_case

REPO = Path(__file__).resolve().parents[1]
GATES = dict(edge_reserve_m=.35, clearance_m=1., minimum_actual_road_margin_m=.1,
             rmse_m=.3, maximum_error_m=.6, maximum_yaw_rad=.1,
             minimum_passing_lateral_m=2.6, blocked_hold_window_s=[14., 20.],
             maximum_actual_hold_speed_m_s=.05, certificate_cadence_ticks=10,
             deadlines='45 s goal / 20 s blocked / unchanged 65 s inherited mission',
             parent_results_sha256=sha(REPO/'assets/obstruction-stop-results.json'))


def specifications(plants, smoke=False):
    rows = [dict(plant=p, world=w, seed=s, mode=m) for p in plants
            for w in ['straight', 'curve', 'right', 'blocked']
            for s in ([7] if smoke else [7, 42, 101]) for m in ['fixed', 'fitted']]
    rows += [dict(plant=p, world=w, seed=s, mode=m) for p in plants
             for w in ['wide', 'mission'] for s in ([7] if smoke else [7, 42])
             for m in ['fixed', 'fitted']]
    return rows


def materialize(spec):
    source = 'scenarios/mission.json' if spec['world']=='mission' else 'scenarios/blocked.json'
    value = json.loads((REPO/source).read_text())
    value.update(full_rate_recording=True, settle_obstruction_stops=True)
    if spec['world']=='mission':
        value['forecast_envelope'] = dict(BOUNDS)
    else:
        value.update(name='Road-width '+spec['world'], duration=45., expected='goal',
                     half_width=5.5 if spec['world']=='wide' else 4.5,
                     curve_amplitude=2. if spec['world']=='curve' else 0., cruise_speed=4.)
        value['objects'][0]['radius'] = .5
        if spec['world']!='wide':
            value['local_route_geometry'] = True
            value['motion_limits'] = dict(max_acceleration_m_s2=2.,max_deceleration_m_s2=2.5,max_lateral_acceleration_m_s2=2.5)
        if spec['world']=='right':
            value['objects'].append(dict(s=35., lateral=2.9, radius=.5))
        if spec['world']=='blocked':
            value.update(duration=20., expected='stop')
            value['objects'][0]['radius'] = 2.0
    if spec['mode']=='fitted': value['lateral_candidate_margin_m'] = GATES['edge_reserve_m']
    return source, value


def freeze(output, specs): return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def validate_inputs(output,manifest,binary):
    """Validate the entire cohort before any physical job starts."""
    folder=output/'validation'; folder.mkdir(exist_ok=False)
    rows=[]
    for i,spec in enumerate(manifest['episodes']):
        result=subprocess.run([str(binary),'validate-scenario','--scenario',str(output/spec['scenario'])],capture_output=True,text=True)
        (folder/f'{i:03d}.stdout').write_text(result.stdout); (folder/f'{i:03d}.stderr').write_text(result.stderr)
        rows.append(dict(index=i,exit_code=result.returncode,scenario=spec['scenario'],sha256=spec['sha256']))
    report=dict(complete=True,passed=all(r['exit_code']==0 for r in rows),episodes=rows)
    (output/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']: raise ValueError('scenario preflight failed; no physical trials started')


def projector(route):
    """Precompute only immutable supplied geometry; never reuse a producer projection."""
    if any(not math.isfinite(p[k]) for p in route for k in ['x','y']):
        raise ValueError('nonfinite road geometry')
    segments=[]; total=0.
    for a, b in zip(route, route[1:]):
        dx=b['x']-a['x']; dy=b['y']-a['y']; length=math.hypot(dx,dy)
        if length<=0.: raise ValueError('degenerate road reference')
        segments.append((a['x'],a['y'],dx,dy,length,total))
        total+=length
    if not segments: raise ValueError('missing road segments')
    def one(point):
        if not math.isfinite(point['x']) or not math.isfinite(point['y']):
            raise ValueError('nonfinite road position')
        best=None
        for ax,ay,dx,dy,length,start in segments:
            x=point['x']-ax; y=point['y']-ay
            fraction=max(0.,min(1.,(x*dx+y*dy)/(length*length)))
            distance=math.hypot(x-fraction*dx,y-fraction*dy)
            candidate=(distance,start+fraction*length,(dx*y-dy*x)/length)
            if best is None or candidate<best: best=candidate
        return best
    return one


def project(point, route):
    """Nearest finite segment, with independent distance and signed lateral coordinate."""
    return projector(route)(point)


def road_margin(run):
    route=run['route']; radius=run['vehicle']['radius']
    locate=projector(route['points'])
    margins=[route['half_width']-radius-locate(f['truth']['pose']['position'])[0]
             for f in run['frames']]
    return dict(minimum_actual_road_margin_m=min(margins),
                violations=sum(m < -1e-8 for m in margins), samples=len(margins))


def swept_clearance(run):
    """Score unsafe intervals too; a failed gate must not erase other diagnostics."""
    minimum=math.inf; contacts=0; previous=None
    xy=lambda p:(p['x'],p['y'])
    for frame in run['frames']:
        ego=xy(frame['truth']['pose']['position'])
        old={o['id']:o for o in previous['objects']} if previous else {}
        tick=math.inf
        for obj in frame['objects']:
            center=xy(obj['position']); distance=math.dist(ego,center)
            if obj['id'] in old:
                before=tuple(a-b for a,b in zip(xy(previous['truth']['pose']['position']),xy(old[obj['id']]['position'])))
                after=tuple(ego[k]-center[k] for k in range(2)); delta=tuple(after[k]-before[k] for k in range(2))
                norm=sum(v*v for v in delta)
                t=max(0.,min(1.,-sum(a*b for a,b in zip(before,delta))/norm)) if norm else 0.
                distance=min(distance,math.hypot(*(before[k]+t*delta[k] for k in range(2))))
            tick=min(tick,distance-run['vehicle']['radius']-obj['radius'])
        minimum=min(minimum,tick); contacts+=tick<0.; previous=frame
    if not math.isfinite(minimum) or abs(minimum-run['summary']['min_clearance'])>1e-8:
        raise ValueError('independent swept clearance differs from physical summary')
    return dict(full_tick_coverage=True,minimum_swept_clearance_m=minimum,contact_intervals=contacts)


def verify_targets(config,ticks):
    margin=config.get('lateral_candidate_margin_m')
    cap=4. if config.get('replan_checked_approaches',False) else 3.5
    side=3.5 if margin is None else max(0.,min(cap,config['route']['half_width']-config['vehicle']['radius']-margin))
    valid=[0.,side,-side] if side>.1 else [0.]
    refined=config.get('refine_lateral_candidates',False)
    if type(refined) is not bool or (refined and margin is None):
        raise ValueError('invalid intermediate-candidate policy')
    if refined:
        valid += [side*f for f in [.875,-.875,.75,-.75,.5,-.5] if abs(side*f)>.1]
    counts=0; locate=projector(config['route']['points'])
    for tick in ticks:
        out=tick['expected']; target=out['trajectory']['lateral_target']
        if not math.isfinite(target) or min(abs(target-v) for v in valid)>1e-10:
            raise ValueError('selected lateral offset does not fit the declared road-width policy')
        if abs(target)>.1: counts+=1
        for point in out['trajectory']['points']:
            distance=locate(point['position'])[0]
            if distance+config['vehicle']['radius']>config['route']['half_width']+1e-8:
                raise ValueError('emitted path leaves the unchanged circular road corridor')
    return dict(side_candidate_m=side, side_control_ticks=counts, verified_control_ticks=len(ticks))


def audit(run,config,ticks,spec,frozen):
    if not contains_declared(run['scenario'],frozen) or config.get('lateral_candidate_margin_m')!=frozen.get('lateral_candidate_margin_m'):
        raise ValueError('recorded world or candidate policy differs from frozen input')
    if config.get('forecast_envelope')!=frozen.get('forecast_envelope') or config.get('settle_obstruction_stops',False)!=frozen.get('settle_obstruction_stops',False):
        raise ValueError('preceding forecast or obstruction policy changed')
    if config.get('local_route_geometry',False)!=frozen.get('local_route_geometry',False):
        raise ValueError('declared local/chassis calibration mode changed')
    if spec['plant']=='rne-dynamic' and frozen.get('local_route_geometry') and not config.get('rear_axle_offset_m',0.)>0.:
        raise ValueError('native chassis reference is missing')
    if {'truth','objects','full_rate_recording','duration','world_motion','sensor_timing'} & config.keys():
        raise ValueError('simulator truth or schedules entered the driver')
    if run['route']!=config['route'] or run['vehicle']!=config['vehicle']:
        raise ValueError('operational and evaluated road/body differ')
    if len(run['frames'])!=len(ticks) or len(ticks)!=run['summary']['steps']:
        raise ValueError('incomplete physical/control coverage')
    errors=[]; yaws=[]; certificates=0; certified_paths=0
    for i,(frame,tick) in enumerate(zip(run['frames'],ticks)):
        out=tick['expected']
        if abs(frame['time']-i*.05)>1e-8 or out['time']!=frame['time'] or out['estimate']!=frame['estimate']:
            raise ValueError('physical/control alignment changed')
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x','y']],
                                [out['estimate']['pose']['position'][k] for k in ['x','y']]))
        yaws.append(abs(angle(frame['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
        if i%10==0 and out['trajectory']['mode']!='Emergency':
            certificates+=certify_trajectory(out,config['vehicle']['radius'])['interval_bounds']; certified_paths+=1
    summary=run['summary']; rmse=math.sqrt(sum(e*e for e in errors)/len(errors)); maximum=max(errors)
    if abs(rmse-summary['localization_rmse'])>1e-10 or abs(maximum-summary['localization_max_error'])>1e-10:
        raise ValueError('summary omitted localization errors')
    failed=[]; functional=[]
    if not summary['passed']: failed.append('source_acceptance')
    if rmse>.3 or maximum>.6 or max(yaws)>.1: failed.append('localization_accuracy')
    physical=swept_clearance(run); road=road_margin(run)
    if (physical['minimum_swept_clearance_m']<1. or road['minimum_actual_road_margin_m']<.1
            or any(summary.get(k,0) for k in ['collisions','road_violations','traffic_collisions','traffic_road_violations','intersection_violations'])):
        functional.append('physical_separation_road_reserve')
    if summary['simulated_seconds']>frozen['duration']+1e-8: functional.append('original_deadline')
    target=verify_targets(config,ticks); passing=[]; hold=None; locate=projector(run['route']['points'])
    if spec['world'] in ['straight','curve','right','wide']:
        if not summary['reached_goal']: functional.append('goal_not_reached')
        for f in run['frames']:
            _,s,lateral=locate(f['truth']['pose']['position'])
            if 33.<=s<=37.: passing.append(lateral)
        if summary['reached_goal']:
            if not passing or min(abs(v) for v in passing)<2.6: functional.append('missing_actual_side_pass')
            if spec['world']=='right' and max(passing)>-2.6: functional.append('occupied_left_refuge')
    elif spec['world']=='blocked':
        hold=max(f['truth']['speed'] for f in run['frames'] if 14.-1e-8<=f['time']<=20.+1e-8)
        if hold>.05 or summary['reached_goal']: functional.append('blocked_road_hold')
    else:
        previous=json.loads((REPO/'assets/obstruction-stop-results.json').read_text())['physical_report']['episodes']
        parent=next(r for r in previous if r['world']=='mission' and r['mode']=='envelope'
                    and r['plant']==spec['plant'] and r['seed']==spec['seed'])
        if summary['passed']!=parent['summary']['passed'] or summary['failures']!=parent['summary']['failures']:
            functional.append('changed_inherited_mission_outcome')
    actual_coverage=coverage(run,ticks) if spec['world']=='mission' else None
    if actual_coverage and not actual_coverage['passed']: failed.append('forecast_coverage')
    failed+=functional
    folder=Path(spec['_folder'])
    forecasts=check_motion_predictions(folder/'run/sensors.jsonl'); profiles=check_speed_profiles(folder/'run/sensors.jsonl')
    return dict(expected_run_exit_code=0, failed_gates=failed, functional_failures=functional,
        functional_passed=not functional, independent_clearance=physical, road_containment=road,
        candidate_policy=target, certified_planned_paths=certified_paths, certified_interval_bounds=certificates,
        minimum_actual_passing_lateral_m=min((abs(v) for v in passing),default=None),
        blocked_maximum_actual_speed_m_s=hold, rmse_m=rmse, maximum_error_m=maximum,
        maximum_yaw_rad=max(yaws), coverage=actual_coverage,
        motion_predictions=forecasts, speed_profiles=profiles)


def compare(rows):
    pairs=[]
    for fitted in [r for r in rows if r['mode']=='fitted']:
        fixed=next(r for r in rows if r['mode']=='fixed' and all(r[k]==fitted[k] for k in ['plant','world','seed']))
        failures=[]
        for row in [fixed,fitted]:
            if ('audit_error' in row or not row.get('replay',{}).get('verified') or row.get('replay_exit_code')!=0
                    or row.get('replay',{}).get('ticks')!=row.get('summary',{}).get('steps')
                    or row.get('run_exit_code')!=(0 if row.get('summary',{}).get('passed') else 1)):
                failures.append('invalid_physical_or_replay_evidence')
        if not fitted.get('functional_passed'): failures.append('fitted_functional_failure')
        if set(fitted.get('failed_gates',[]))-set(fixed.get('failed_gates',[])): failures.append('new_qualification_failure')
        identical=None
        if fitted['world'] in ['wide','mission']:
            identical=all(fitted['storage'][n]['sha256']==fixed['storage'][n]['sha256'] for n in ['replay/outputs.jsonl','run/summary.json'])
            if not identical: failures.append('changed_wide_road_outputs')
        pairs.append(dict(plant=fitted['plant'],world=fitted['world'],seed=fitted['seed'],
                          passed=not failures,failures=failures,wide_outputs_identical=identical))
    return pairs


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    p.add_argument('--smoke',action='store_true'); p.add_argument('--freeze-only',action='store_true')
    p.add_argument('--check-policy-regression',action='store_true')
    p.add_argument('--binary',type=Path,default=REPO/'target/release/rustdriving')
    p.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args=p.parse_args(); output=args.output.resolve(); plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants,args.smoke))
    if args.freeze_only: return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    fingerprints={k:sha(binaries[k]) for k in plants}; rows=[]
    validate_inputs(output,manifest,args.binary.resolve())
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
                        auditor=lambda run,config,ticks,spec,frozen:audit(run,config,ticks,dict(spec,_folder=str(folder)),frozen))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            rows.append(row)
            report=dict(complete=len(rows)==len(manifest['episodes']), manifest_sha256=sha(output/'frozen-manifest.json'),
                        binaries=fingerprints,episodes=rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row.get('functional_passed'),row.get('audit_error',row.get('failed_gates')),flush=True)
    report['pairs']=compare(rows); report['policy_regression_passed']=all(r['passed'] for r in report['pairs'])
    report['positive_qualification_passed']=all(r['passed'] for r in rows if r['mode']=='fitted')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    assert all(sha(binaries[k])==fingerprints[k] for k in plants)
    if not (report['policy_regression_passed'] if args.check_policy_regression else report['positive_qualification_passed']): raise SystemExit(1)


if __name__=='__main__': main()
