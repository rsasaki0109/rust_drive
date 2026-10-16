#!/usr/bin/env python3
"""Own-time RGB/depth rendering, causal history faults and independent warp equations."""
import argparse,copy,importlib.util
from pathlib import Path
import numpy as np
from projective_depth_common import Client,ROOT,audit_sequence,fresh,pose,require,sha,write
from projective_depth_raw import reconstruct_update
from projective_photo_raw import metadata
from projective_timing_raw import TimingHistory


def trajectory(elapsed):
    angle=.3*elapsed;t=np.eye(4);t[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
    t[:3,3]=[.5/.3*np.sin(angle),.5/.3*(1-np.cos(angle)),0.]
    return t


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=fresh(args.output);digest=sha(args.binary.read_bytes())
    spec=importlib.util.spec_from_file_location('timing_scene',ROOT/'scripts/check-projective-depth.py');scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    results={};systems=0;witness=None
    for name in ('nonplanar asynchronous','plane compensated','plane uncorrected','plane unavailable bootstrap'):
        plane=name.startswith('plane');enabled=name!='plane uncorrected';config=dict(photometric='keyframe',motion_model='constant_velocity')
        if enabled:config['timing']='previous_motion'
        client=Client(args.binary,scene.CAMERA,configuration=config);history=TimingHistory()
        model=weights=key_gray=key_rgb=last_rgb=None;key_phase=0.;rows=[];times=[];contexts=[]
        def render(elapsed):
            t=trajectory(elapsed);d=np.full((120,160),2.5) if plane else scene.render(t)
            return d,scene.render_gray(t,d)
        stamps=[1.+.03*i for i in range(10)] if not plane else [1.+.03*i for i in range(8)]
        if name=='plane unavailable bootstrap':stamps=stamps[:2]
        if not plane:stamps.insert(3,1.075)
        try:
            for i,stamp in enumerate(stamps):
                if name=='plane unavailable bootstrap':phase=.01
                elif plane:phase=(0.,0.,.014,-.014,.012,-.012,.014,-.014)[i]
                else:phase=(.012,-.009,.014,.005,.013,-.01,.014,-.009,.012,-.009,.014)[i]
                depth,_=render(stamp-1.);rgb=stamp+phase;_,gray=render(rgb-1.)
                if stamp==1.075:depth[:]=0.
                u=client.frame(stamp,depth,image=dict(timestamp=rgb,gray=gray.ravel().tolist()))
                fresh_rgb=last_rgb is None or rgb>last_rgb;last_rgb=rgb
                context=dict(target=key_gray,source=gray,source_timestamp=rgb,reference_timestamp=key_rgb,
                    qualified=key_gray is not None and abs(phase-key_phase)<=.005,fresh=fresh_rgb)
                if enabled:history.augment(context,stamp)
                initial=model is None
                model,weights,count=reconstruct_update(model,weights,depth,scene.CAMERA,u,image_context=context)
                systems+=count['iterations']+count['final_equations'];contexts.append(copy.deepcopy(context))
                if u['accepted'] and (initial or u['keyframe_replaced']):
                    key_gray=gray if fresh_rgb else None;key_rgb=rgb if fresh_rgb else None;key_phase=phase if fresh_rgb else 0.
                history.observe(u)
                row=dict(update=u)
                if u['accepted']:
                    error=np.linalg.inv(trajectory(stamp-1.))@pose(u['first_from_current'])
                    position=float(np.linalg.norm(error[:3,3]));rotation=float(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1)))
                    row.update(position_error_m=position,rotation_error_rad=rotation)
                    if enabled:require(position<.004 and rotation<.004,'own-time rendered motion 4mm/0.004rad: '+name)
                rows.append(row);times.append(stamp)
                if not plane and stamp==1.075:
                    require(not u['accepted'] and u['last_accepted_timestamp']==1.06,'failed acquisition cannot advance motion or reference rate')
                if not plane and stamp==1.09:
                    require(u['photometric']['timing']['source']['latest_timestamp']==1.06,'rejected pose excluded from time warp')
            if name=='plane uncorrected':
                depth,_=render(.231);_,gray=render(.231)
                expired=client.frame(1.231,depth,image=dict(timestamp=1.231,gray=gray.ravel().tolist()))
                recovery=client.frame(1.24,depth,image=dict(timestamp=1.24,gray=gray.ravel().tolist()))
                require(expired['lost'] and recovery['lost'] and not recovery['accepted'],'rejected weak information cannot renew accepted-depth expiry')
            client.finish()
        except BaseException:
            client.abort();raise
        audit_sequence(rows,times)
        from projective_information_raw import audit
        for r in rows:audit(r['update'])
        accepted=sum(r['update']['accepted'] for r in rows)
        if name=='nonplanar asynchronous':
            require(accepted==10 and any(r['update']['keyframe_replaced'] for r in rows),'own-time scene tracks through keyframe change')
            require(not rows[1]['update']['final_photometric']['used'],'nonzero-phase bootstrap withholds image before causal rate')
            require(rows[2]['update']['final_photometric']['used'],'initial reference backfilled after accepted depth interval')
            index=next(i for i,r in enumerate(rows) if i>2 and r['update'].get('final_photometric',{}).get('used',False) and r['update']['photometric']['timing']['reference']['latest_timestamp']>1.03)
            witness=(rows[index]['update'],contexts[index])
        elif name=='plane compensated':
            require(accepted==8 and all(r['update']['final_photometric']['used'] for r in rows[1:]),'time compensation retains plane observability')
        elif name=='plane uncorrected':
            require(accepted==2 and all('information_failure' in r['update'] for r in rows[2:]),'unobservable plane drift never commits pose')
            require(sum(r.get('position_error_m',1.)<.004 and r.get('rotation_error_rad',1.)<.004 for r in rows)<8,'uncorrected asynchronous plane fails full known-motion accuracy')
        else:require(accepted==1 and not rows[-1]['update'].get('final_photometric',{}).get('used',False),'asynchronous unobservable bootstrap cannot invent rate')
        accurate=sum(r.get('position_error_m',1.)<.004 and r.get('rotation_error_rad',1.)<.004 for r in rows)
        if name=='plane uncorrected':
            from projective_information_raw import audit
            failure=next(r['update'] for r in rows if 'information_failure' in r['update'])
            for label,change in (
                ('fabricated physical spectrum',lambda u:u['information_failure']['spectrum']['eigenvalues'].__setitem__(0,.5)),
                ('weak pose marked accepted',lambda u:u.update(accepted=True)),
                ('missing failure witness',lambda u:u.pop('information_failure'))):
                altered=copy.deepcopy(failure);change(altered)
                try:audit(altered)
                except (AssertionError,ValueError,KeyError,TypeError):pass
                else:raise ValueError('undetected observability corruption: '+label)
        results[name]=dict(accepted=accepted,accurate=accurate,frames=rows)
    require(witness is not None,'reference and source nonidentity compensation witness')
    corruptions=[]
    for name,change in (
        ('current observation substituted into motion history',lambda u:u['photometric']['timing']['source'].update(latest_timestamp=u['timestamp'])),
        ('wrong signed exposure transform',lambda u:u['photometric']['timing']['source']['image_from_depth']['translation'].__setitem__(0,.02)),
        ('newer rate substituted for frozen reference rate',lambda u:u['photometric']['timing']['reference'].update(latest_timestamp=u['timestamp'])),
        ('incorrect phase qualification',lambda u:u['photometric'].update(phase_qualified=False))):
        altered=copy.deepcopy(witness[0]);change(altered)
        try:metadata(altered,witness[1])
        except (ValueError,AssertionError,KeyError,TypeError):corruptions.append(name)
        else:raise ValueError('undetected timing corruption: '+name)
    require(sha(args.binary.read_bytes())==digest,'one fixed binary across timing controls')
    write(out/'controls.json',results)
    write(out/'summary.json',dict(pass_timing_controls=True,physical_guard=True,observability_corruptions_rejected=3,weak_pose_expiry_latches=True,raw_failure_witnesses=7,raw_geometry_systems=systems,raw_photometric_systems=systems,
        accepted={n:r['accepted'] for n,r in results.items()},accurate={n:r['accurate'] for n,r in results.items()},rejected_corruptions=corruptions,
        own_time_rendering=True,accepted_history_only=True,reference_rate_frozen=True,binary_sha256=digest,
        checker_sha256=sha(Path(__file__).read_bytes()),known_pose_role='static scene rendering at depth and RGB acquisition times and scoring only; never runtime inputs'))


if __name__=='__main__':main()
