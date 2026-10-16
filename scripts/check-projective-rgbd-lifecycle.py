#!/usr/bin/env python3
"""Known-scene control for asynchronous RGB/depth and color-reference ownership."""
import argparse,importlib.util
from pathlib import Path
import numpy as np
from projective_depth_common import Client,ROOT,audit_sequence,fresh,require,sha,write
from projective_depth_raw import reconstruct_update


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--step-control',choices=('disabled','fixed_support'),default='disabled')
    p.add_argument('--timing',choices=('disabled','previous_motion'),default='disabled')
    args=p.parse_args();out=fresh(args.output)
    from projective_timing_raw import TimingHistory
    history=TimingHistory()
    spec=importlib.util.spec_from_file_location('synthetic_rgbd',ROOT/'scripts/check-projective-depth.py')
    scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    client=Client(args.binary,scene.CAMERA,configuration=dict(photometric='keyframe',step_control=args.step_control,timing=args.timing,motion_model='constant_velocity' if args.timing=='previous_motion' else 'stationary'))
    model=weights=key_gray=key_rgb=last_rgb=None;key_phase=0.;rows=[];times=[];checked=0
    try:
        for stamp,x in ((1.,0.),(1.004,.03),(1.008,.06),(1.012,.09),(1.016,.12),
                        (1.04,.15),(1.07,.18),(1.10,.21),(1.13,.24),(1.16,.27)):
            transform=np.eye(4);transform[0,3]=x;raw=scene.render(transform)
            rgb_stamp=1.001 if stamp<=1.016 else stamp+.001
            image_pose=np.eye(4) if stamp<=1.016 else transform
            gray=scene.render_gray(image_pose,scene.render(image_pose))
            update=client.frame(stamp,raw,image=dict(timestamp=rgb_stamp,gray=gray.ravel().tolist()))
            require(update['accepted'],'new measured depth remains trackable')
            new_rgb=last_rgb is None or rgb_stamp>last_rgb;last_rgb=rgb_stamp;phase=rgb_stamp-stamp
            context=dict(target=key_gray,source=gray,source_timestamp=rgb_stamp,reference_timestamp=key_rgb,
                         qualified=key_gray is not None and abs(phase-key_phase)<=.005,fresh=new_rgb)
            if args.timing=='previous_motion':history.augment(context,stamp)
            initial=model is None
            model,weights,counts=reconstruct_update(model,weights,raw,scene.CAMERA,update,image_context=context)
            checked+=counts['iterations']+counts['final_equations']
            if initial or update['keyframe_replaced']:
                key_gray=gray if new_rgb else None;key_rgb=rgb_stamp if new_rgb else None;key_phase=phase if new_rgb else 0.
            history.observe(update)
            rows.append(dict(expected=transform.tolist(),update=update));times.append(stamp)
        require(any(r['update']['keyframe_replaced'] and not r['update']['photometric']['fresh'] for r in rows),'reference changes with a repeated image')
        require(any(r['update']['photometric']['reference_timestamp'] is None and r['update']['iterations'] for r in rows),'no stale image anchor in new view')
        require(rows[-1]['update']['final_photometric']['used'],'fresh image reference resumes intensity constraints')
        audit_sequence(rows,times)
        last=times[-1];depth=scene.render(transform);gray=scene.render_gray(transform,depth)
        def observe(stamp,rgb_stamp,pixels):
            image=None if rgb_stamp is None else dict(timestamp=rgb_stamp,gray=pixels)
            return client.frame(stamp,depth,image=image)
        repeated=observe(1.164,last+.001,gray.ravel().tolist())
        require(repeated['accepted'] and not repeated['photometric']['fresh'] and not repeated['final_photometric']['used'],'repeated RGB contributes no intensity')
        accepted=repeated['last_accepted_timestamp'];faults={}
        for name,stamp,rgb_stamp,pixels in (
            ('duplicate depth',1.164,last+.001,gray.ravel().tolist()),
            ('regressing RGB',1.17,1.16,gray.ravel().tolist()),
            ('missing RGB',1.18,None,[]),('malformed RGB size',1.19,1.191,[]),
            ('unassociated RGB',1.20,1.25,gray.ravel().tolist())):
            u=observe(stamp,rgb_stamp,pixels);require(not u['accepted'] and u['last_accepted_timestamp']==accepted,'fault does not refresh pose: '+name);faults[name]=u
        expired=observe(1.37,1.371,gray.ravel().tolist());recovery=observe(1.38,1.381,gray.ravel().tolist())
        require(expired['lost'] and recovery['lost'] and not recovery['accepted'],'RGB cannot hide accepted-depth expiry')
        client.finish()
    except BaseException:
        client.abort();raise
    write(out/'sequence.json',dict(frames=rows,repeated=repeated,faults=faults,expired=expired,recovery=recovery))
    write(out/'summary.json',dict(pass_lifecycle=True,measured_depth_frames=len(rows),raw_equations_checked=checked,step_control=args.step_control,timing=args.timing,
        repeated_rgb_never_contributes=True,color_anchor_cleared_and_resumed=True,faults_rejected=list(faults),
        no_implicit_reset=True,binary_sha256=sha(args.binary.read_bytes()),checker_sha256=sha(Path(__file__).read_bytes()),
        source='synthetic static scene; known poses generate measurements only'))


if __name__=='__main__':main()
