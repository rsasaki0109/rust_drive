#!/usr/bin/env python3
"""Ray-cast motion checks that exercise fixed-support image-boundary backtracking."""
import argparse,copy,importlib.util
from pathlib import Path
import numpy as np
from projective_depth_common import Client,ROOT,audit_sequence,exp_rotation,fresh,pose,require,sha,write
from projective_depth_raw import reconstruct_update


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=fresh(args.output);digest=sha(args.binary.read_bytes())
    spec=importlib.util.spec_from_file_location('step_scene',ROOT/'scripts/check-projective-depth.py');scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    first=scene.render(np.eye(4));first_gray=scene.render_gray(np.eye(4),first);rows=[];checked=trials=reduced=unavailable=0;tampered=[]
    for axis,angle,translation in ((0,.04,[.04,0,0]),(1,.08,[.06,.01,0]),(2,-.06,[.03,.01,0]),(0,0,[.09,0,0])):
        t=np.eye(4);v=np.zeros(3);v[axis]=angle;t[:3,:3]=exp_rotation(v);t[:3,3]=translation
        depth=scene.render(t);gray=scene.render_gray(t,depth);client=Client(args.binary,scene.CAMERA,configuration=dict(photometric='keyframe',step_control='fixed_support'))
        try:
            initial=client.frame(1.,first,image=dict(timestamp=1.001,gray=first_gray.ravel().tolist()))
            update=client.frame(1.03,depth,image=dict(timestamp=1.031,gray=gray.ravel().tolist()));client.finish()
        except BaseException:
            client.abort();raise
        require(initial['accepted'] and update['accepted'],'known motion remains available')
        audit_sequence([dict(update=initial),dict(update=update)],[1.,1.03])
        context=dict(target=first_gray,source=gray,source_timestamp=1.031,reference_timestamp=1.001,qualified=True,fresh=True)
        reconstruct_update(first,(first>.3).astype(np.uint8),depth,scene.CAMERA,update,image_context=context)
        error=np.linalg.inv(t)@pose(update['first_from_current']);position=float(np.linalg.norm(error[:3,3]));rotation=float(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1)))
        require(position<=.004 and rotation<=.004,'known-motion 4mm/0.004rad gates')
        controls=[s['step_control'] for s in update['iterations']];checked+=len(controls);trials+=sum(len(c['trial_costs_m2']) for c in controls)
        reduced+=sum(0<c['applied_scale']<1 for c in controls);unavailable+=sum(v is None for c in controls for v in c['trial_costs_m2'])
        rows.append(dict(expected=t.tolist(),initial=initial,update=update,position_error_m=position,rotation_error_rad=rotation))
        if not tampered:
            boundary=next((i for i,s in enumerate(update['iterations']) if s['step_control']['trial_costs_m2'][0] is None),None)
            if boundary is not None:
                altered=copy.deepcopy(update);altered['iterations'][boundary]['step_control']['trial_costs_m2'][0]=0.
                try:reconstruct_update(first,(first>.3).astype(np.uint8),depth,scene.CAMERA,altered,image_context=context)
                except (AssertionError,ValueError):tampered.append('discarded image pixels falsely produce a zero-cost trial')
                else:raise ValueError('undetected fabricated boundary trial')
    require(reduced>=3 and unavailable>=3 and len(tampered)==1,'exercise boundary backtracking and raw tamper rejection')
    require(sha(args.binary.read_bytes())==digest,'binary unchanged during controls')
    write(out/'controls.json',rows)
    write(out/'summary.json',dict(pass_step_control=True,motions=4,raw_step_reports=checked,raw_trial_costs=trials,
        reduced_steps=reduced,unavailable_trials=unavailable,rejected_corruptions=tampered,
        binary_sha256=digest,checker_sha256=sha(Path(__file__).read_bytes()),
        known_pose_role='synthetic measurement generation and scoring, never tracker input'))


if __name__=='__main__':main()
