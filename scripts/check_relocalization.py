#!/usr/bin/env python3
"""Freeze and audit measured regional recovery, retained loss and failed accuracy."""
import argparse
import copy
import json
import math
import os
from pathlib import Path

from check_gnss_antenna import archive
from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import inverse, sha, verify_ticks as verify_map_ticks
from check_lane_change_speeds import sweep
from check_speed_zones import execute
try:
    import numpy as np
except ImportError:
    np = None

REPO = Path(__file__).resolve().parents[1]
POLICY = dict(position_half_extent_m=4., yaw_half_extent_rad=.3, max_neighbor_checks=20000000,
              max_attempts=8, confirmations=3, retry_interval_s=.5)
GATES = dict(valid_rmse_m=.2, valid_max_error_m=.4, valid_max_yaw_rad=.1, clearance_m=1.,
             stop_window_s=[10.5,11.2], stop_speed_m_s=.05, recovery_window_s=[11.,15.], negative_hold_after_s=12.)
OBJECTS = [(6,5,.9),(13,-6,1.2),(22,4.5,.7),(30,-5,1.),(40,7,1.4),(47,-4.5,.8),
           (58,5.5,1.1),(69,-6.5,.9),(83,4.8,1.3),(96,-5.7,.7),(109,6.2,1.),(118,-4.8,.9)]


def specifications(plants, smoke=False):
    rows=[]
    for plant in plants:
        for world in ['straight','curve']:
            for seed in ([7] if smoke else [1,7,42]):
                rows.append(dict(plant=plant,world=world,seed=seed,mode='recover'))
            for mode in ['healthy','disabled']:
                rows.append(dict(plant=plant,world=world,seed=7,mode=mode))
        for mode in ['ambiguous','no-overlap','missing-motion','delayed-scan']:
            rows.append(dict(plant=plant,world='straight',seed=7,mode=mode))
    return rows


def sample(points, arc, lateral):
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length=math.hypot(dx,dy)
        if arc <= length:
            return (a[0]+dx*arc/length-dy*lateral/length,
                    a[1]+dy*arc/length+dx*lateral/length)
        arc-=length
    a,b=points[-2:];dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
    return b[0]-dy*lateral/length,b[1]+dx*lateral/length


def materialize(spec):
    source=f"scenarios/gnss-delay-{spec['world']}.json"
    value=json.loads((REPO/source).read_text())
    value['name']=f"Regional relocalization {spec['world']} {spec['mode']}"
    objects=OBJECTS if spec['mode']!='ambiguous' else [(s,l,.8) for s in range(0,121,4) for l in [-6,6]]
    value['objects']=[dict(s=s,lateral=l,radius=r) for s,l,r in objects]
    length=value['road_length'];n=math.ceil(length)
    points=[(length*i/n,value['curve_amplitude']*math.sin(length*i/n/28.)) for i in range(n+1)]
    centers=[(*sample(points,s,l),r) for s,l,r in objects]
    if spec['mode']=='ambiguous':
        centers=[(s,l,.8) for s in range(-20,145,4) for l in [-6,6]]
    mapped=[dict(x=x+r*math.cos(i*math.tau/360),y=y+r*math.sin(i*math.tau/360))
            for x,y,r in centers for i in range(360)]
    if spec['mode']=='no-overlap':
        for p in mapped:p['y']+=50.
    value['localization_map']=dict(points=mapped)
    if spec['mode']!='disabled':value['relocalization']=copy.deepcopy(POLICY)
    if spec['mode']=='ambiguous':value['relocalization']['max_neighbor_checks']=100000000
    if spec['mode']!='healthy':
        value['sensor_timing']['odometry_dropout_windows']=[{'from':8.,'until':45. if spec['mode']=='missing-motion' else 11.}]
    if spec['mode']=='delayed-scan':value['sensor_timing']['lidar_delay_ticks']=2
    value['expected']='goal' if spec['mode'] in ['recover','healthy'] else 'fault'
    return source,value


def freeze(output,specs):
    manifest=dict(gates=GATES,policy=POLICY,episodes=[])
    for index,spec in enumerate(specs):
        source,value=materialize(spec);path=output/'frozen'/f'{index:03d}.json';path.parent.mkdir(parents=True,exist_ok=True)
        text=json.dumps(value,indent=2)+'\n'
        if path.exists():
            if path.read_text()!=text:raise ValueError('frozen regional scenario changed')
        else:
            with path.open('x',newline='\n') as stream:stream.write(text)
        manifest['episodes'].append(dict(spec,scenario=str(path.relative_to(output)),sha256=sha(path),source=source,source_sha256=sha(REPO/source)))
    path=output/'frozen-manifest.json'
    if path.exists():
        if json.loads(path.read_text())!=manifest:raise ValueError('frozen regional cohort changed')
    else:
        with path.open('x',newline='\n') as stream:stream.write(json.dumps(manifest,indent=2)+'\n')
    return manifest


def wrap(value):return (value+math.pi)%math.tau-math.pi


def hypotheses(scan,mapped,center,policy):
    """Independent complete point-pair correlation, separate from the Rust grid."""
    radius=policy.get('position_half_extent_m',4.);cell=radius/8.;result=[]
    round_integer=lambda x:math.floor(x+.5) if x>=0 else math.ceil(x-.5)
    cx,cy=center['position']['x'],center['position']['y']
    arrays = np is not None and len(scan)*len(mapped)>10000
    if arrays:
        source=np.array([[p['x'],p['y']] for p in scan]);target=np.array([[p['x'],p['y']] for p in mapped])
    for index in range(-2,3):
        yaw=wrap(center['yaw']+index*policy.get('yaw_half_extent_rad',.3)/2.)
        c,s=math.cos(yaw),math.sin(yaw);votes={}
        if arrays:
            totals=np.zeros(289,dtype=np.int64)
            x=source[:,0]*c-source[:,1]*s;y=source[:,0]*s+source[:,1]*c
            for begin in range(0,len(scan),32):
                for offset in range(0,len(mapped),4096):
                    dx=target[None,offset:offset+4096,0]-x[begin:begin+32,None]-cx
                    dy=target[None,offset:offset+4096,1]-y[begin:begin+32,None]-cy
                    valid=(np.abs(dx)<=radius)&(np.abs(dy)<=radius)
                    a,b=dx[valid]/cell,dy[valid]/cell
                    a=np.where(a>=0,np.floor(a+.5),np.ceil(a-.5)).astype(np.int64)+8
                    b=np.where(b>=0,np.floor(b+.5),np.ceil(b-.5)).astype(np.int64)+8
                    totals+=np.bincount(a*17+b,minlength=289)
            votes={(i//17-8,i%17-8):int(v) for i,v in enumerate(totals) if v}
        else:
            for p in scan:
                x,y=p['x']*c-p['y']*s,p['x']*s+p['y']*c
                for target in mapped:
                    dx,dy=target['x']-x-cx,target['y']-y-cy
                    if abs(dx)<=radius and abs(dy)<=radius:
                        key=(round_integer(dx/cell),round_integer(dy/cell));votes[key]=votes.get(key,0)+1
        for (x,y),count in sorted(votes.items(),key=lambda row:(-row[1],row[0]))[:25]:
            result.append(dict(pose=dict(position=dict(x=cx+x*cell,y=cy+y*cell),yaw=yaw),votes=count))
    return result


def raw_step(scan,mapped,pose,config):
    """Exhaustive correspondences and a complex-number rigid least-squares step."""
    c,s=math.cos(pose['yaw']),math.sin(pose['yaw']);unique={}
    distance=config.get('max_correspondence_m',1.5)
    neighbors=[]
    if np is not None and len(scan)*len(mapped)>10000:
        targets=np.array([[p['x'],p['y']] for p in mapped]);source=np.array([[p['x'],p['y']] for p in scan])
        xs=source[:,0]*c-source[:,1]*s+pose['position']['x'];ys=source[:,0]*s+source[:,1]*c+pose['position']['y']
        for begin in range(0,len(scan),32):
            nearest=np.full(min(32,len(scan)-begin),np.inf);indices=np.zeros(len(nearest),dtype=np.int64)
            for offset in range(0,len(mapped),4096):
                distances=np.hypot(xs[begin:begin+32,None]-targets[None,offset:offset+4096,0],ys[begin:begin+32,None]-targets[None,offset:offset+4096,1])
                local=np.argmin(distances,axis=1);values=distances[np.arange(len(local)),local];better=values<nearest
                nearest[better]=values[better];indices[better]=offset+local[better]
            neighbors.extend(zip(nearest.tolist(),indices.tolist()))
    else:
        for p in scan:
            x,y=p['x']*c-p['y']*s+pose['position']['x'],p['x']*s+p['y']*c+pose['position']['y']
            neighbors.append(min((math.hypot(x-q['x'],y-q['y']),j) for j,q in enumerate(mapped)))
    for index,(d,j) in enumerate(neighbors):
        if d<=distance and (j not in unique or (d,index)<unique[j][:2]):unique[j]=(d,index,j)
    pairs=sorted(unique.values())
    pairs=pairs[:math.floor(len(pairs)*.8)]
    if len(pairs)<config.get('min_pairs',20) or len(pairs)/len(scan)<config.get('min_overlap',.5):
        raise ValueError('raw proposal lacks unique trimmed measured support')
    n=len(pairs);rms=math.sqrt(sum(d*d for d,_,_ in pairs)/n)
    sources=[complex(scan[i]['x'],scan[i]['y']) for _,i,_ in pairs]
    targets=[complex(mapped[j]['x'],mapped[j]['y']) for _,_,j in pairs]
    a,b=sum(sources)/n,sum(targets)/n
    cross=sum((p-a).conjugate()*(q-b) for p,q in zip(sources,targets))
    fitted_yaw=math.atan2(cross.imag,cross.real);rotation=complex(math.cos(fitted_yaw),math.sin(fitted_yaw));translation=b-rotation*a
    fitted=dict(position=dict(x=translation.real,y=translation.imag),yaw=fitted_yaw)
    return fitted,sources,rms


def verify_candidate(scan,mapped,seed,candidate,config,max_iterations=30):
    """Rebuild the actual bounded stopping rule from the selected raw seed.

    A small last step can change correspondences. Requiring another unexecuted
    step to be small imposed a different rule; no follow-up tolerance is waived.
    """
    pose=seed
    for _ in range(max_iterations):
        fitted,_,_=raw_step(scan,mapped,pose,config)
        delta=math.dist([fitted['position'][k] for k in ['x','y']],[pose['position'][k] for k in ['x','y']])
        yaw=abs(angle(fitted['yaw']-pose['yaw']))
        pose=fitted
        if delta<=1e-4 and yaw<=1e-4:break
    else:raise ValueError('raw regional fit did not satisfy the bounded stopping rule')
    if math.dist([pose['position'][k] for k in ['x','y']],[candidate['pose']['position'][k] for k in ['x','y']])>1e-8 or abs(angle(pose['yaw']-candidate['pose']['yaw']))>1e-8:
        raise ValueError('published pose differs from the raw seeded fit')
    return raw_fit(scan,mapped,candidate['pose'],config)


def raw_fit(scan,mapped,pose,config):
    """Independent final residuals and joint normal-matrix covariance."""
    _,sources,rms=raw_step(scan,mapped,pose,config)
    n=len(sources);c,s=math.cos(pose['yaw']),math.sin(pose['yaw'])
    normal=[[0.]*3 for _ in range(3)]
    for point in sources:
        rotated=point*complex(c,s)
        for row in [(1.,0.,-rotated.imag),(0.,1.,rotated.real)]:
            for i in range(3):
                for j in range(3):normal[i][j]+=row[i]*row[j]
    variance=max(1e-6,rms*rms*n/(2*n-3));inv=inverse(normal)
    covariance=[[v*variance for v in row] for row in inv]
    scale=[math.sqrt(max(1.,floor/covariance[i][i])) for i,floor in enumerate([.01,.01,.0001])]
    floored=[[covariance[i][j]*scale[i]*scale[j] for j in range(3)] for i in range(3)]
    return dict(rms_m=rms,inlier_count=n,covariance=floored)


def verify_search(search,inp,mapped,policy,map_config):
    raw=inp['lidar']['points'];count=min(len(raw),128)
    scan=[raw[i*len(raw)//count] for i in range(count)] if count else []
    if search['scan_points_used']!=count:raise ValueError('regional fixed input sampling changed')
    if search['neighbor_checks']>policy.get('max_neighbor_checks',20000000):
        raise ValueError('regional comparison budget exceeded')
    if search['error'] and 'budget exhausted' in search['error']:
        if search['proposed_pose'] is not None or search['neighbor_checks']!=policy.get('max_neighbor_checks',20000000):raise ValueError('incomplete search promoted a proposal or misstated exhaustion')
        return False
    expected=hypotheses(scan,mapped,search['center'],policy)
    if len(expected)!=len(search['hypotheses']) or search['seeds_tested']!=len(expected):
        raise ValueError('regional search skipped a selected hypothesis')
    for a,b in zip(expected,search['hypotheses']):
        if a['votes']!=b['votes'] or any(abs(a['pose']['position'][k]-b['pose']['position'][k])>1e-9 for k in ['x','y']) or abs(angle(a['pose']['yaw']-b['pose']['yaw']))>1e-10:
            raise ValueError('regional seed differs from complete raw correlation')
    if search['correlation_comparisons']!=5*len(scan)*len(mapped):
        raise ValueError('coarse point-pair comparison count changed')
    candidates=search['candidates']
    if not candidates:
        if not search['error'] or search['proposed_pose'] is not None:raise ValueError('empty regional search claimed a pose')
        return False
    best=min(candidates,key=lambda c:c['rms_m'])
    witnesses=[best]
    alternate=next((c for c in candidates if (math.dist([c['pose']['position'][k] for k in ['x','y']],[best['pose']['position'][k] for k in ['x','y']])>.25
                      or abs(angle(c['pose']['yaw']-best['pose']['yaw']))>.05) and c['rms_m']<=max(.02,best['rms_m'])*1.1),None)
    if alternate:witnesses.append(alternate)
    for candidate in witnesses:
        index=candidate['seed_index']
        if not 0<=index<len(search['hypotheses']):raise ValueError('regional candidate names an untested seed')
        proof=verify_candidate(scan,mapped,search['hypotheses'][index]['pose'],candidate,map_config)
        if abs(proof['rms_m']-candidate['rms_m'])>1e-8 or proof['inlier_count']!=candidate['inlier_count']:
            raise ValueError('regional candidate statistics differ from raw points')
    if bool(alternate)!=bool(search['error'] and 'ambiguous' in search['error']):
        raise ValueError('regional ambiguity decision changed')
    if search['error']:
        if search['proposed_pose'] is not None:raise ValueError('rejected search promoted a pose')
        return False
    if search['proposed_pose']!=best['pose']:raise ValueError('regional proposal is not its best accepted candidate')
    covariance=raw_fit(scan,mapped,best['pose'],map_config)['covariance']
    if max(abs(covariance[i][j]-search['proposed_covariance'][i][j]) for i in range(3) for j in range(3))>1e-8:
        raise ValueError('regional reset covariance differs from joint raw normal equations')
    return True


def verify_epochs(config,ticks):
    policy=config.get('relocalization');wheel=None;gap=None;previous=None
    gap_index=None
    for index,tick in enumerate(ticks):
        inp=tick['input'];now=inp['time'];wheel=inp.get('odometry') or wheel
        if previous is not None and (now-previous>.25 or wheel is None or now-wheel['stamp']>.15+1e-9):
            gap_index=index;gap=dict(last_covered_time=previous,detected_at=now,last_accepted_odometry_stamp=wheel['stamp'] if wheel else None,
                                   cause='ClockGap' if now-previous>.25 else 'MissingOdometry' if wheel is None else 'StaleOdometry');break
        previous=now
    if gap_index is None:
        if any(t['expected'].get('motion_discontinuity') or t['expected'].get('localization_epoch',0) for t in ticks):raise ValueError('covered motion claimed an epoch reset')
        return dict(passed=True,prefix=verify_map_ticks(config,ticks),reset_stamp=None,searches=0,valid_indices=list(range(len(ticks))))
    prefix=verify_map_ticks(config,ticks[:gap_index]);prior=ticks[gap_index-1]['expected']
    attempts=stopped=confirmed=0;last_motion=last_attempt=last_confirmation=None;last_pose=None;reset=None;searches=0
    for index in range(gap_index,len(ticks)):
        inp,out=ticks[index]['input'],ticks[index]['expected'];now=inp['time'];diagnostic=out.get('relocalization')
        if index>gap_index and policy:
            motion=inp.get('odometry')
            fresh=bool(motion and abs(motion['stamp']-now)<=1e-9 and motion['stamp']>gap['detected_at'] and abs(motion['speed'])<=.05 and abs(motion['yaw_rate'])<=.01 and (last_motion is None or motion['stamp']>last_motion))
            if not fresh:stopped=confirmed=0;last_confirmation=None;last_pose=None
            else:
                if last_motion is not None and now-last_motion>.075+1e-9:stopped=confirmed=0;last_confirmation=None;last_pose=None
                last_motion=now;stopped=min(3,stopped+1)
            if last_confirmation is not None and now-last_confirmation>.75+1e-9:confirmed=0;last_confirmation=None;last_pose=None
            scan=inp.get('lidar');valid=bool(scan and abs(scan['stamp']-now)<=1e-9 and scan['stamp']>gap['detected_at'] and not inp.get('lidar_failed',False))
            if scan and not valid:confirmed=0;last_confirmation=None;last_pose=None
            due=stopped>=3 and valid and attempts<policy['max_attempts'] and (last_attempt is None or now-last_attempt>=policy['retry_interval_s']-1e-9)
            search=diagnostic.get('search')
            if bool(search)!=bool(due):raise ValueError('regional search bypassed fresh standstill/cadence/budget admission')
            if search:
                attempts+=1;last_attempt=now;searches+=1
                if search['center']!=prior['estimate']['pose']:raise ValueError('search received an undeclared current pose prior')
                if verify_search(search,inp,config['localization_map']['points'],policy,config['localization_map']):
                    pose=search['proposed_pose']
                    consistent=last_pose is not None and math.dist([pose['position'][k] for k in ['x','y']],[last_pose['position'][k] for k in ['x','y']])<=.15 and abs(angle(pose['yaw']-last_pose['yaw']))<=.02
                    confirmed=confirmed+1 if consistent else 1;last_pose=pose;last_confirmation=now
                else:confirmed=0;last_confirmation=None;last_pose=None
            if diagnostic['attempts']!=attempts or diagnostic['stopped_samples']!=stopped:
                raise ValueError('regional observation counters differ from raw acquisitions')
            # Exhaustion explicitly cancels an incomplete confirmation chain.
            expected_confirmed=0 if attempts>=policy['max_attempts'] and confirmed<policy['confirmations'] else confirmed
            if diagnostic['confirmed_scans']!=expected_confirmed:raise ValueError('regional confirmation count changed')
            if confirmed>=policy['confirmations']:
                if out.get('localization_epoch')!=1 or out.get('motion_discontinuity') or out['estimate']['pose']!=last_pose:
                    raise ValueError('confirmed measured pose did not initialize exactly one new epoch')
                if out['estimate']['speed']!=0. or out['pose_covariance']!=search['proposed_covariance'] or out['tracks'] or out['predictions']:
                    raise ValueError('reset reused an old state, posterior or scene')
                if out['localization']['accepted_fixes'] or out['localization']['last_observed_stamp'] is not None or not out['emergency']:
                    raise ValueError('confirming frame reused an old acquisition or resumed')
                reset=(index,now,last_pose,search['proposed_covariance']);break
        if out.get('motion_discontinuity')!=gap or not out['emergency'] or out['command']!=dict(acceleration=-6.,steering=0.) or out.get('localization_epoch',0):
            raise ValueError('missing motion resumed without a measured reset')
        for key in ['estimate','position_variance','pose_covariance','tracking_frame','localization']:
            if out.get(key)!=prior.get(key):raise ValueError('invalid epoch changed held state/freshness')
        if out['tracks'] or out['predictions'] or out['gnss_latency']['history_predictions']:raise ValueError('scene/history bridged missing motion')
    valid=list(range(gap_index));post=None
    if reset:
        index,stamp,pose,covariance=reset;newconfig=copy.deepcopy(config);newconfig.update(initial_pose=pose,initial_pose_covariance=covariance)
        segment=copy.deepcopy(ticks[index:])
        for tick in segment:
            if tick['input'].get('gnss') and tick['input']['gnss']['stamp']<=stamp:tick['input']['gnss']=None
            if tick['expected'].get('localization_epoch')!=1 or tick['expected'].get('motion_discontinuity'):raise ValueError('reinitialized epoch silently changed')
        post=verify_map_ticks(newconfig,segment,measured_reset_stamp=stamp);valid+=list(range(index,len(ticks)))
    return dict(passed=True,prefix=prefix,post_epoch=post,reset_stamp=reset[1] if reset else None,searches=searches,valid_indices=valid)


def audit(run,config,ticks,spec,frozen):
    if not contains_declared(run['scenario'],frozen):raise ValueError('scenario differs from frozen regional inputs')
    if any(k in config for k in ['sensor_timing','objects','truth','odometry_dropout_windows']):raise ValueError('simulation fields entered the operational header')
    if config.get('relocalization')!=frozen.get('relocalization') or config['localization_map']['points']!=frozen['localization_map']['points']:raise ValueError('supplied recovery policy/map changed')
    frames,summary=run['frames'],run['summary'];errors=[];yaw=[]
    if len(frames)!=len(ticks) or len(ticks)!=summary['steps']:raise ValueError('missing full physical coverage')
    for i,(frame,tick) in enumerate(zip(frames,ticks)):
        inp,out=tick['input'],tick['expected']
        if abs(inp['time']-i*.05)>1e-8 or frame['time']!=inp['time'] or frame['estimate']!=out['estimate']:raise ValueError('physical/control time alignment changed')
        dropout=any(inp['time']+1e-9>=w['from'] and inp['time']<w['until']-1e-9 for w in frozen['sensor_timing'].get('odometry_dropout_windows',[]))
        if bool(inp.get('odometry'))==dropout or (inp.get('odometry') and inp['odometry']['stamp']!=inp['time']):raise ValueError('motion acquisition fault schedule changed')
        errors.append(math.dist([frame['truth']['pose']['position'][k] for k in ['x','y']],[out['estimate']['pose']['position'][k] for k in ['x','y']]))
        yaw.append(abs(angle(frame['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors));maximum=max(errors)
    if abs(rmse-summary['localization_rmse'])>1e-10 or abs(maximum-summary['localization_max_error'])>1e-10:raise ValueError('global invalid-epoch errors were removed')
    oracle=verify_epochs(config,ticks);valid=oracle.pop('valid_indices');valid_rmse=math.sqrt(sum(errors[i]**2 for i in valid)/len(valid));valid_max=max(errors[i] for i in valid);valid_yaw=max(yaw[i] for i in valid)
    clearance=sweep(run);failed=[]
    explained=[]
    if spec['mode']!='healthy' and maximum>1. and valid_max<=1.:explained=[f'localization max error {maximum:.3f} m exceeds 1 m']
    if summary['failures']!=explained or summary['passed']!=(not explained):failed.append('source_acceptance')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance']<1. or summary['simulated_seconds']>45:failed.append('physical_clearance_deadline')
    if spec['mode'] in ['healthy','recover'] and (not summary['reached_goal'] or valid_rmse>.2 or valid_max>.4 or valid_yaw>.1):failed.append('valid_epoch_goal_accuracy')
    if spec['mode']=='recover' and (oracle['reset_stamp'] is None or not 11.<=oracle['reset_stamp']<=15. or oracle['searches']<3):failed.append('measured_recovery')
    if spec['mode'] not in ['recover','healthy'] and (oracle['reset_stamp'] is not None or summary['reached_goal'] or max(f['truth']['speed'] for f in frames if f['time']>=12.)>.05):failed.append('negative_stop_hold')
    searches=[t['expected'].get('relocalization',{}).get('search') for t in ticks]
    if spec['mode']=='ambiguous' and not any(s and s['error'] and 'ambiguous' in s['error'] for s in searches):failed.append('ambiguity_witness')
    if spec['mode']=='no-overlap' and not any(s and s['error'] and 'no accepted' in s['error'] for s in searches):failed.append('no_fit_witness')
    if spec['mode'] in ['missing-motion','delayed-scan'] and oracle['searches']:failed.append('unsupported_acquisition_search')
    stop=None
    if spec['mode']!='healthy':
        stop=min(f['truth']['speed'] for f in frames if 10.5<=f['time']<11.2)
        if stop>.05:failed.append('physical_stop_before_recovery')
    return dict(oracle=oracle,global_rmse_m=rmse,global_maximum_error_m=maximum,valid_epoch_rmse_m=valid_rmse,valid_epoch_maximum_error_m=valid_max,valid_epoch_maximum_yaw_rad=valid_yaw,invalid_pose_ticks=len(ticks)-len(valid),source_accuracy_failures_retained=explained,expected_run_exit_code=1 if explained else 0,independent_clearance=clearance,minimum_actual_stop_speed_m_s=stop,failed_gates=failed)


def run_case(output,spec,binary,replay_binary):
    folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}";folder.mkdir(parents=True,exist_ok=False)
    source=output/spec['scenario'];assert sha(source)==spec['sha256'];row=dict(spec,passed=False,recordings=str(folder.relative_to(output)))
    args=['run','--scenario',source] if spec['plant']=='reference-2d' else ['--scenario',source,'--plant','dynamic']
    try:
        row['run_exit_code']=execute(binary,args+['--seed',spec['seed'],'--output',folder/'run'],folder/'run-process')
        row['replay_exit_code']=execute(replay_binary,['replay','--log',folder/'run/sensors.jsonl','--output',folder/'replay'],folder/'replay-process')
        run=json.loads((folder/'run/run.json').read_text());row['summary']=run['summary'];row['replay']=json.loads((folder/'replay/replay.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream:
            config=json.loads(next(stream))['header']['config'];ticks=[r['tick'] for line in stream if 'tick' in (r:=json.loads(line))]
        row.update(audit(run,config,ticks,spec,json.loads(source.read_text())))
        row['passed']=row['run_exit_code']==row['expected_run_exit_code'] and row['replay_exit_code']==0 and row['replay']['verified'] and row['replay']['ticks']==len(ticks) and not row['failed_gates']
    except Exception as error:row['audit_error']=repr(error)
    finally:row['storage']=archive(folder)
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all');parser.add_argument('--smoke',action='store_true');parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--binary',type=Path,default=REPO/'target/release'/('rustdriving.exe' if os.name=='nt' else 'rustdriving'));parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne');args=parser.parse_args()
    output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant];manifest=freeze(output,specifications(plants,args.smoke))
    if args.freeze_only:print('Frozen regional relocalization episodes:',len(manifest['episodes']));return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};provenance={p:dict(path=str(binaries[p]),sha256=sha(binaries[p])) for p in plants};rows=[]
    for spec in manifest['episodes']:
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve());rows.append(row)
        report=dict(passed=all(r['passed'] for r in rows),complete=len(rows)==len(manifest['episodes']),manifest_sha256=sha(output/'frozen-manifest.json'),binaries=provenance,gates=GATES,episodes=rows)
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(spec['plant'],spec['world'],spec['mode'],spec['seed'],row['passed'],row.get('audit_error',row.get('failed_gates')),flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__=='__main__':main()
