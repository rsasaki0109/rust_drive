#!/usr/bin/env python3
"""Independently audit live mission ownership through measured relocalization."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path

from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_gnss_recovery import POLICY as GNSS_POLICY
from check_lane_change_speeds import sweep
from check_lane_resume_speeds import interval_excess
from check_mid_edge import compare, reference
from check_relocalization import POLICY, REPO, sample, verify_epochs
from check_relocalization_map_only import freeze as freeze_inputs, run_case as drive_case

GATES = dict(duration_s=60., road_length_m=140., half_width_m=3.5, cruise_m_s=6.,
             valid_rmse_m=.2, valid_maximum_error_m=.4, valid_yaw_rad=.1,
             clearance_m=1., speed_excess_m_s=.15, post_loss_hold_delay_s=4.,
             post_reset_motion_window_s=[1.,3.8], minimum_post_reset_distance_m=3.,
             no_route_hold_after_s=32., minimum_reopen_hold_s=2.,
             navigation_freshness_s=1., handover_confirmations=3,
             reset_route_margin_m=.2, reset_yaw_rad=.35, gnss_policy=GNSS_POLICY)
LANDMARKS = [(6,12,.9),(13,-12,1.2),(22,11,.7),(30,-12,1.),(40,13,1.4),(47,-13,.8),
             (58,12,1.1),(69,-12,.9),(83,12,1.3),(96,-13,.7),(109,12,1.),(132,-12,.9)]


def specifications(plants, smoke=False):
    rows=[]
    for plant in plants:
        for world in ['straight','curve']:
            for seed in ([7] if smoke else [1,7,42]):
                rows.append(dict(plant=plant,world=world,seed=seed,mode='late-closure'))
            for mode in ['pre-switched','no-route','reopened','disabled','healthy']:
                rows.append(dict(plant=plant,world=world,seed=7,mode=mode))
    return rows


def materialize(spec):
    source=f"scenarios/gnss-delay-{spec['world']}.json"
    value=json.loads((REPO/source).read_text());mode=spec['mode']
    amplitude=0. if spec['world']=='straight' else 4.
    def point(x,detour=False):
        return dict(x=float(x),y=amplitude*math.sin(x/28.)+(4.*math.sin(math.pi*(x-70.)/40.)**4 if detour else 0.))
    nodes=[dict(id=name,position=point(x)) for name,x in [('start',0),('fork',70),('merge',110),('goal',140)]]
    edges=[dict(id=eid,**{'from':a,'to':b},points=[point(x,eid=='detour') for x in range(low,high+1)],half_width=3.5,speed_limit_m_s=cap)
           for eid,a,b,low,high,cap in [('head','start','fork',0,70,6.),('main','fork','merge',70,110,6.),('detour','fork','merge',70,110,3.),('tail','merge','goal',110,140,4.)]]
    nav=dict(network=dict(nodes=nodes,edges=edges),start='start',goal='goal',closed_edges=[])
    initial=reference(nav,[]);points=[(p['x'],p['y']) for p in initial['points']]
    centers=[(*sample(points,s,l),r) for s,l,r in LANDMARKS]
    value.update(name=f"Measured mission recovery {spec['world']} {mode}",duration=60.,road_length=140.,half_width=3.5,navigation=nav,
                 objects=[dict(s=s,lateral=l,radius=r) for s,l,r in LANDMARKS],
                 localization_map=dict(points=[dict(x=x+r*math.cos(i*math.tau/360),y=y+r*math.sin(i*math.tau/360)) for x,y,r in centers for i in range(360)]),
                 relocalization=dict(POLICY,map_only_grace_s=5.,confirm_gnss_handoff=True,preserve_navigation=True),
                 gnss_recovery=copy.deepcopy(GNSS_POLICY),expected='fault' if mode in ['no-route','disabled'] else 'goal')
    loss=16. if mode=='pre-switched' else 8.
    value['sensor_timing']['odometry_dropout_windows']=[] if mode=='healthy' else [dict(**{'from':loss,'until':loss+3.})]
    value['gnss_dropout_windows']=[] if mode=='healthy' else [dict(**{'from':loss,'until':loss+8.})]
    value.pop('gnss_dropout',None)
    closed=['main','detour'] if mode in ['no-route','reopened'] else ['main']
    value['navigation_updates']=[dict(stamp=3. if mode=='pre-switched' else 9.,revision=1,closed_edges=closed)]
    if mode=='reopened':value['navigation_updates'].append(dict(stamp=24.,revision=2,closed_edges=['main']))
    if mode=='disabled':value.pop('relocalization')
    return source,value


def freeze(output,specs):
    return freeze_inputs(output,specs,factory=materialize,gates=GATES)


def projection(points,position):
    """Segment projection reconstructed independently from map coordinates."""
    best=None;distance=0.
    for a,b in zip(points,points[1:]):
        dx,dy=b['x']-a['x'],b['y']-a['y'];length=math.hypot(dx,dy)
        qx,qy=position['x']-a['x'],position['y']-a['y'];u=max(0.,min(1.,(qx*dx+qy*dy)/(length*length)))
        gap=math.hypot(qx-u*dx,qy-u*dy)
        candidate=(gap,distance+u*length,(dx*qy-dy*qx)/length,math.atan2(dy,dx))
        if best is None or gap<best[0]:best=candidate
        distance+=length
    return best


def verify_navigation(config,ticks,reset_stamp=None,obstruction_observer=None,decision_auditor=None):
    nav=config['navigation'];radius=config['vehicle']['radius'];decel=(config.get('motion_limits') or {}).get('max_deceleration_m_s2',2.5)
    closed=list(nav.get('closed_edges',[]));active=reference(nav,closed);pending=None
    phase='Following';revision=switches=votes=0;stop=None;last_stamp=fault=watermark=wheel=None;blocked_hold=False
    reset_evidence=None;events=[];reset_ownership=None;handover_hold=False;held_ticks=0;local_exclusions=[]
    def effective_closed():return list(dict.fromkeys(closed+local_exclusions))
    def replan(ego):
        nonlocal phase,pending,stop
        excluded=effective_closed();wanted=reference(nav,excluded)
        if wanted is not None and wanted['edge_ids']==active['edge_ids']:
            phase='Following';stop=None;return
        cursor=0.;boundary=active['length_m'];common=0.
        for i,span in enumerate(active['traversals']):
            if span['edge'] in excluded:boundary=min(boundary,cursor)
            length=span['end_s_m']-span['start_s_m']
            if wanted is not None and wanted['edge_ids'][i:i+1]==[span['edge']] and abs(common-cursor)<1e-6:common+=length
            cursor+=length
        if wanted is not None:boundary=min(boundary,common)
        stop=max(0.,boundary-radius-2.);s=projection(active['points'],ego['pose']['position'])[1]
        reachable=stop-s>=max(0.,ego['speed'])**2/(1.6*decel)+1. or (abs(ego['speed'])<=.05 and s<=stop+.5)
        if wanted is not None and reachable and wanted['half_width']>radius and len(wanted['points'])<=5000 and wanted['length_m']<=1500.:
            phase='Braking';pending=wanted
        else:phase='Blocked'
    for tick in ticks:
        inp,out=tick['input'],tick['expected'];now=inp['time'];ego=out['estimate'];wheel=inp.get('odometry') or wheel
        if obstruction_observer is not None:
            wanted_exclusions=obstruction_observer.observe(tick,active,phase,revision,closed)
            obstruction_observer.audit_status(out['navigation'].get('observed_obstructions'))
            if wanted_exclusions!=local_exclusions:
                local_exclusions=list(wanted_exclusions);replan(ego)
        if reset_stamp is not None and now==reset_stamp:
            distance,s,lateral,yaw=projection(active['points'],ego['pose']['position']);yaw_error=abs(angle(ego['pose']['yaw']-yaw))
            admitted=abs(ego['speed'])<=.05 and distance+radius+.2<active['half_width'] and yaw_error<.35
            reset_evidence=dict(stamp=now,progress_s_m=s,lateral_m=lateral,distance_to_route_m=distance,yaw_error_rad=yaw_error,admitted=admitted)
            reset_ownership=dict(revision=revision,closed_edges=list(closed),active_edges=list(active['edge_ids']),switches=switches,phase_before=phase)
            votes=0;watermark=now;blocked_hold=False;handover_hold=False
            if not admitted:phase='Fault';fault=now;pending=None
            elif phase in ['Braking','Blocked']:pending=None;replan(ego)
        if reset_evidence is None or reset_evidence['admitted']:
            healthy=not out['health'];motion=wheel if healthy else None
            fresh=motion is not None and (watermark is None or motion['stamp']>watermark)
            stamp=now if motion is None else motion['stamp'];watermark=stamp if watermark is None else max(watermark,stamp)
            update=inp.get('navigation_update')
            if update is not None and update['revision']>revision and (last_stamp is None or update['stamp']>last_stamp):
                if not 0.<=now-update['stamp']<=1.+1e-9 or any(e not in {x['id'] for x in nav['network']['edges']} for e in update['closed_edges']):
                    phase='Fault';fault=now;pending=None;votes=0
                elif fault is None or update['stamp']>fault:
                    revision=update['revision'];closed=list(update['closed_edges']);last_stamp=update['stamp'];fault=None;votes=0
                    keep_pending=(obstruction_observer is not None and obstruction_observer.enabled and phase=='Braking'
                        and pending is not None and (wanted:=reference(nav,effective_closed())) is not None
                        and wanted['edge_ids']==pending['edge_ids'])
                    if not keep_pending:pending=None;replan(ego)
            if phase=='Fault':votes=0;watermark=max(watermark,now)
            else:
                if phase=='Following':blocked_hold=False;handover_hold=False
                _,s,lateral,yaw=projection(active['points'],ego['pose']['position'])
                if phase=='Blocked' and fresh and abs(motion['speed'])<=.05 and abs(ego['speed'])<=.05 and stop-s<=3.5:blocked_hold=True
                if phase=='Braking' and motion is not None and abs(motion['speed'])<=.05 and abs(ego['speed'])<=.05:
                    aligned=s<=stop+.5 and abs(lateral)+radius+.2<active['half_width'] and abs(angle(ego['pose']['yaw']-yaw))<.35
                    if not aligned:votes=0
                    elif fresh:
                        handover_hold=handover_hold or nav.get('hold_for_handover',False)
                        votes+=1
                        if votes>=3:
                            active=pending;pending=None;stop=None;switches+=1;phase='Following';blocked_hold=False;votes=0;handover_hold=False
                            events.append(dict(stamp=now,active_edges=list(active['edge_ids']),revision=revision,after_reset=reset_stamp is not None and now>reset_stamp))
                else:votes=0
        actual=out['navigation']
        if decision_auditor is not None:decision_auditor(tick,active,stop,phase)
        if actual.get('handover_hold',False)!=handover_hold:raise ValueError('unproved handover brake hold')
        held_ticks+=int(handover_hold)
        expected=dict(revision=revision,phase=phase,active_edges=active['edge_ids'],pending_edges=[] if pending is None else pending['edge_ids'],closed_edges=closed,stop_s=stop,switches=switches)
        for key,value in expected.items():
            if isinstance(value,float):
                if actual[key] is None or abs(actual[key]-value)>1e-8:raise ValueError('independent navigation geometry differs: '+key)
            elif actual[key]!=value:raise ValueError('independent navigation ownership differs: '+key)
        evidence=actual.get('localization_reset')
        if evidence is None or reset_evidence is None:
            if evidence!=reset_evidence:raise ValueError('unproved navigation reset evidence')
        else:
            for key,value in reset_evidence.items():
                if isinstance(value,bool):
                    if evidence[key]!=value:raise ValueError('route admission changed')
                elif abs(evidence[key]-value)>1e-8:raise ValueError('measured route admission differs: '+key)
        minimum_brake=-5.99 if phase=='Fault' else -.5
        if (phase=='Fault' or blocked_hold or handover_hold) and out['command']['acceleration']>minimum_brake+1e-9:raise ValueError('navigation hold failed to brake')
    return dict(passed=True,verified_ticks=len(ticks),reset_ownership=reset_ownership,switch_events=events,final_revision=revision,final_switches=switches,final_phase=phase,verified_handover_hold_ticks=held_ticks)


def scenario_matches(actual,frozen):
    recorded=copy.deepcopy(actual)
    recorded.setdefault('gnss_dropout_windows',[])
    recorded.setdefault('sensor_timing',{}).setdefault('odometry_dropout_windows',[])
    return contains_declared(recorded,frozen)


def audit(run,config,ticks,spec,frozen):
    if not scenario_matches(run['scenario'],frozen):raise ValueError('mission input differs from frozen scenario')
    if config['navigation']!=frozen['navigation'] or config.get('relocalization')!=frozen.get('relocalization'):raise ValueError('operational mission/policy changed')
    if config.get('speed_zones') or {'objects','sensor_timing','truth','navigation_updates'} & config.keys():raise ValueError('synthesis/policy ambiguity entered operational config')
    if config['localization_map']['points']!=frozen['localization_map']['points'] or config['gnss_recovery']!=frozen['gnss_recovery']:raise ValueError('map or GNSS policy changed')
    frames,summary=run['frames'],run['summary'];errors=[];yaws=[];updates=iter(frozen['navigation_updates']);next_update=next(updates,None)
    if len(frames)!=len(ticks) or len(ticks)!=summary['steps']:raise ValueError('full physical/control coverage required')
    loss=16. if spec['mode']=='pre-switched' else 8.;healthy=spec['mode']=='healthy';delay=frozen['sensor_timing']['gnss_delay_ticks']
    for i,(frame,tick) in enumerate(zip(frames,ticks)):
        inp,out=tick['input'],tick['expected'];now=inp['time']
        if abs(now-i*.05)>1e-8 or frame['time']!=now or frame['estimate']!=out['estimate'] or frame['navigation']!=out['navigation']:raise ValueError('control/physical alignment changed')
        dropped=not healthy and loss-1e-9<=now<loss+3.-1e-9
        if bool(inp.get('odometry'))==dropped or (inp.get('odometry') and inp['odometry']['stamp']!=now):raise ValueError('motion fault schedule changed')
        acq=(i-delay)*.05;gnss_due=i>=delay and (i-delay)%4==0 and (healthy or not loss-1e-9<=acq<loss+8.-1e-9)
        if bool(inp.get('gnss'))!=gnss_due or (gnss_due and abs(inp['gnss']['stamp']-acq)>1e-9):raise ValueError('GNSS acquisition/delivery changed')
        if bool(inp.get('lidar'))!=(i%2==0) or (inp.get('lidar') and inp['lidar']['stamp']!=now) or inp['lidar_failed']:raise ValueError('map scan acquisition changed')
        due=next_update is not None and next_update['stamp']<=now+1e-9
        if inp.get('navigation_update')!=(next_update if due else None):raise ValueError('external closure delivery changed')
        if due:next_update=next(updates,None)
        truth,pose=frame['truth']['pose'],out['estimate']['pose']
        errors.append(math.dist([truth['position'][k] for k in ['x','y']],[pose['position'][k] for k in ['x','y']]))
        yaws.append(abs(angle(truth['yaw']-pose['yaw'])))
    oracle=verify_epochs(config,ticks);valid=oracle.pop('valid_indices');reset=oracle['reset_stamp']
    nav=verify_navigation(config,ticks,reset)
    valid_rmse=math.sqrt(sum(errors[i]**2 for i in valid)/len(valid));valid_max=max(errors[i] for i in valid);valid_yaw=max(yaws[i] for i in valid)
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors));maximum=max(errors)
    if abs(summary['localization_rmse']-rmse)>1e-10 or abs(summary['localization_max_error']-maximum)>1e-10:raise ValueError('invalid-epoch errors were removed')
    explained=[f'localization max error {maximum:.3f} m exceeds 1 m'] if maximum>1. and valid_max<=1. else []
    failed=[]
    if summary['failures']!=explained or summary['passed']!=(not explained):failed.append('source_acceptance')
    if valid_rmse>.2 or valid_max>.4 or valid_yaw>.1:failed.append('valid_epoch_accuracy')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance']<1. or summary['simulated_seconds']>60.:failed.append('physical_clearance_deadline')
    clearance=sweep(run)
    initial=reference(config['navigation'],config['navigation'].get('closed_edges',[]));transitions=run['route_history'];plans=[]
    if not transitions or transitions[0]['time']!=0.:raise ValueError('missing original route history')
    for i,event in enumerate(transitions):
        closed=list(config['navigation'].get('closed_edges',[]))
        for u in frozen['navigation_updates']:
            if u['stamp']<=event['time']+1e-9:closed=list(u['closed_edges'])
        expected=reference(config['navigation'],closed);compare(event['plan'],expected);plans.append(expected)
        if i and (event['time']<=transitions[i-1]['time'] or max(abs(event[k]) for k in ['true_speed','estimated_speed'])>.05+1e-9):raise ValueError('route switch before actual standstill')
    if plans[0]['edge_ids']!=initial['edge_ids']:raise ValueError('initial mission changed')
    active=0;previous=None;speed_excess=0.;speed_violations=0;covered=set();closed=[];hold_start=None;reopen_hold=0.
    for frame in frames:
        now=frame['time'];truth=frame['truth'];plan=plans[active]
        excess,touched=interval_excess(plan['points'],plan['zones'],previous or truth,truth,run['vehicle']['radius']);covered.update((active,z) for z in touched)
        if active+1<len(plans) and now>=transitions[active+1]['time']-1e-9:
            active+=1;plan=plans[active];instant,touched=interval_excess(plan['points'],plan['zones'],truth,truth,run['vehicle']['radius']);excess=max(excess,instant);covered.update((active,z) for z in touched)
        speed_excess=max(speed_excess,excess);speed_violations+=excess>.15+1e-9
        state=frame['navigation']
        if state['active_edges']!=plan['edge_ids'] or state['switches']!=active:raise ValueError('active route timing differs from actual policy')
        for u in frozen['navigation_updates']:
            if u['stamp']<=now+1e-9:closed=u['closed_edges']
        s=projection(plan['points'],truth['pose']['position'])[1];cursor=0.
        for span in plan['traversals']:
            if span['edge'] in closed and s+run['vehicle']['radius']>=cursor:raise ValueError('physical body entered a closed edge')
            cursor+=span['end_s_m']-span['start_s_m']
        if spec['mode']=='reopened' and 9.<=now<24.:
            if abs(truth['speed'])<=.05:
                hold_start=now if hold_start is None else hold_start;reopen_hold=max(reopen_hold,now-hold_start)
            else:hold_start=None
        previous=truth
    if speed_violations:failed.append('original_edge_speed_caps')
    wanted_switches=0 if spec['mode'] in ['no-route','disabled'] else 1
    if len(transitions)-1!=wanted_switches or nav['final_switches']!=wanted_switches:failed.append('mission_switch_count')
    goal=spec['mode'] not in ['no-route','disabled']
    if summary['reached_goal']!=goal:failed.append('original_destination')
    reset_motion=None
    if healthy:
        if reset is not None or any(t['expected'].get('localization_epoch',0) for t in ticks):failed.append('healthy_no_reset')
    else:
        if min(f['truth']['speed'] for f in frames if loss+2.5<=f['time']<loss+3.2)>.05:failed.append('actual_stop_before_reset')
        if spec['mode']=='disabled':
            if reset is not None or max(f['truth']['speed'] for f in frames if f['time']>=loss+4.)>.05:failed.append('default_motion_loss_hold')
        elif reset is None or not loss+3.<=reset<=loss+7.:failed.append('measured_mission_recovery')
        else:
            window=[f for f in frames if reset+1.<=f['time']<=reset+3.8]
            reset_motion=sum(math.dist([a['truth']['pose']['position'][k] for k in ['x','y']],[b['truth']['pose']['position'][k] for k in ['x','y']]) for a,b in zip(window,window[1:]))
            if reset_motion<3.:failed.append('actual_resumed_mission_motion')
            owner=nav['reset_ownership']
            if owner['revision']!=1 or owner['closed_edges']!=(['main','detour'] if spec['mode'] in ['no-route','reopened'] else ['main']):failed.append('retained_closure_snapshot')
            if spec['mode']=='pre-switched':
                if owner['switches']!=1 or owner['active_edges']!=['head','detour','tail']:failed.append('retained_active_detour')
            elif owner['switches']!=0 or owner['active_edges']!=['head','main','tail']:failed.append('retained_active_initial_route')
    if spec['mode']=='no-route' and max(f['truth']['speed'] for f in frames if f['time']>=32.)>.05:failed.append('closed_mission_stop_hold')
    if spec['mode']=='reopened' and (reopen_hold<2. or nav['final_revision']!=2):failed.append('physical_reopen_hold')
    if goal and any((len(plans)-1,z) not in covered for z in range(len(plans[-1]['zones']))):failed.append('active_speed_intervals_not_exercised')
    return dict(oracle=oracle,navigation_oracle=nav,total_ticks=len(ticks),invalid_pose_ticks=len(ticks)-len(valid),
                global_rmse_m=rmse,global_maximum_error_m=maximum,valid_epoch_rmse_m=valid_rmse,
                valid_epoch_maximum_error_m=valid_max,valid_epoch_maximum_yaw_rad=valid_yaw,
                expected_run_exit_code=1 if explained else 0,source_accuracy_failures_retained=explained,
                independent_clearance=clearance,mapped_speed_maximum_excess_m_s=speed_excess,mapped_speed_violation_intervals=speed_violations,
                mapped_speed_intervals_exercised=sorted(covered),post_reset_distance_m=reset_motion,
                pre_reopening_physical_hold_s=reopen_hold,failed_gates=failed)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all');parser.add_argument('--smoke',action='store_true');parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--binary',type=Path,default=REPO/'target/release'/('rustdriving.exe' if os.name=='nt' else 'rustdriving'));parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne');args=parser.parse_args()
    output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant];manifest=freeze(output,specifications(plants,args.smoke))
    if args.freeze_only:print('Frozen navigation recovery episodes:',len(manifest['episodes']));return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};provenance={p:dict(path=str(binaries[p]),sha256=sha(binaries[p])) for p in plants};rows=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda spec:drive_case(output,spec,binaries[spec['plant']],args.binary.resolve(),auditor=audit),manifest['episodes']):
            rows.append(row);report=dict(passed=all(x['passed'] for x in rows),complete=len(rows)==len(manifest['episodes']),manifest_sha256=sha(output/'frozen-manifest.json'),binaries=provenance,gates=GATES,episodes=rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(row['plant'],row['world'],row['mode'],row['seed'],row['passed'],row.get('audit_error',row.get('failed_gates')),flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__=='__main__':main()
