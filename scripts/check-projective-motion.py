#!/usr/bin/env python3
"""Known screw motion, irregular cadence and accepted-history fault controls."""
import argparse,copy,importlib.util
from pathlib import Path
import numpy as np
from projective_depth_common import Client,ROOT,audit_sequence,fresh,pose,require,sha,write
from projective_depth_raw import reconstruct_update


def helix(elapsed,linear=.15,angular=.35):
    angle=elapsed*angular;t=np.eye(4);t[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
    t[:3,3]=[linear/angular*np.sin(angle),linear/angular*(1-np.cos(angle)),.03*elapsed]
    return t


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=fresh(args.output);digest=sha(args.binary.read_bytes())
    spec=importlib.util.spec_from_file_location('motion_scene',ROOT/'scripts/check-projective-depth.py');scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    client=Client(args.binary,scene.CAMERA,configuration=dict(motion_model='constant_velocity'));rows=[];times=[];faults={};checked=0;model=weights=None
    try:
        bad=client.frame(.99,np.empty((0,0)));require(not bad['accepted'] and bad['last_accepted_timestamp'] is None,'bad initialization cannot establish history');faults['malformed initial depth']=bad
        for stamp in (1.,1.03,1.05,1.06,1.075,1.09,1.10,1.16,1.19):
            expected=helix(stamp-1);depth=scene.render(expected)
            if stamp==1.06:depth=np.zeros_like(depth)
            if stamp==1.075:depth=np.empty((0,0))
            update=client.frame(stamp,depth);rows.append(dict(expected=expected.tolist(),update=update));times.append(stamp)
            if stamp in (1.06,1.075):
                require(not update['accepted'] and update['last_accepted_timestamp']==1.05,'failed observation cannot advance history')
                faults[str(stamp)]=update
                if stamp==1.075:
                    duplicate=client.frame(stamp,scene.render(expected));regression=client.frame(1.04,scene.render(expected))
                    for name,value in (('duplicate depth',duplicate),('regressing depth',regression)):
                        require(not value['accepted'] and value['last_accepted_timestamp']==1.05,'clock fault cannot advance history');faults[name]=value
            else:
                require(update['accepted'],'known irregular motion remains available')
                error=np.linalg.inv(expected)@pose(update['first_from_current']);position=float(np.linalg.norm(error[:3,3]));rotation=float(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1)))
                require(position<=.004 and rotation<=.004,'irregular helix 4mm/0.004rad gates')
                rows[-1].update(position_error_m=position,rotation_error_rad=rotation)
            if depth.shape==(120,160):
                model,weights,c=reconstruct_update(model,weights,depth,scene.CAMERA,update);checked+=c['iterations']+c['final_equations']
        audit_sequence(rows,times)
        after_fault=next(r['update'] for r in rows if r['update']['timestamp']==1.09)['prediction']
        require(after_fault['previous_timestamp']==1.03 and after_fault['latest_timestamp']==1.05 and after_fault['horizon_s']==1.09-1.05,'variable horizon excludes every rejected observation')
        expired=client.frame(1.391,scene.render(helix(.391)));recovery=client.frame(1.40,scene.render(helix(.40)))
        require(expired['lost'] and recovery['lost'] and not recovery['accepted'] and 'prediction' not in expired and 'prediction' not in recovery,'latched expiry never extrapolates');client.finish()
    except BaseException:
        client.abort();raise
    corruptions=[]
    def corrupt(name,change):
        altered=copy.deepcopy(rows);change(next(r['update'] for r in altered if r['update']['timestamp']==1.09))
        try:audit_sequence(altered,times)
        except (AssertionError,ValueError,KeyError,TypeError):corruptions.append(name)
        else:raise ValueError('undetected prediction corruption: '+name)
    corrupt('rejected acquisition inserted into history',lambda u:u['prediction'].update(latest_timestamp=1.075))
    corrupt('fixed frame interval substituted for elapsed time',lambda u:u['prediction'].update(horizon_s=.02))
    corrupt('wrong predicted root pose',lambda u:u['prediction']['first_from_prediction']['translation'].__setitem__(0,u['prediction']['first_from_prediction']['translation'][0]+.001))
    corrupt('future pose claimed as history',lambda u:u['prediction'].update(latest_timestamp=1.09))
    client=Client(args.binary,scene.CAMERA,configuration=dict(motion_model='constant_velocity'));bounded=[]
    try:
        for stamp,elapsed in ((2.,0.),(2.02,.02),(2.20,.02)):
            t=helix(elapsed,linear=.8,angular=1.);u=client.frame(stamp,scene.render(t));require(u['accepted'],'bounded fallback retains stopped-scene availability');bounded.append(dict(update=u))
        client.finish()
    except BaseException:
        client.abort();raise
    audit_sequence(bounded,[2.,2.02,2.20]);require(bounded[-1]['update']['prediction']['status']=='increment_limit','large prediction uses stationary fallback')
    require(sha(args.binary.read_bytes())==digest,'fixed binary throughout motion controls')
    write(out/'sequence.json',dict(frames=rows,faults=faults,expired=expired,recovery=recovery,bounded_fallback=bounded))
    write(out/'summary.json',dict(pass_motion_controls=True,valid_irregular_acquisitions=7,raw_geometry_systems=checked,
        rejected_faults=len(faults),rejected_corruptions=corruptions,bounded_fallback_pass=True,accepted_history_only=True,
        binary_sha256=digest,checker_sha256=sha(Path(__file__).read_bytes()),
        known_pose_role='analytic helix generates measurements and scores only; runtime sees depth and acquisition time'))


if __name__=='__main__':main()
