#!/usr/bin/env python3
"""Depth-range weighting: raw weak-plane witnesses and fixed noisy ray-cast controls."""
import argparse,copy,importlib.util,json
from pathlib import Path
import numpy as np
from projective_depth_common import ROOT,Client,audit_sequence,exp_rotation,fresh,pose,require,sha,write
from projective_depth_raw import reconstruct_update
from projective_range_raw import controls


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--residual',choices=('depth_variance_plane','depth_whitened_plane'),default='depth_variance_plane')
    args=parser.parse_args();out=fresh(args.output);binary=args.binary.absolute();binary_hash=sha(binary.read_bytes())
    scene_path=ROOT/'scripts/check-projective-depth.py'
    sources=(Path(__file__),scene_path,ROOT/'scripts/projective_depth_common.py',ROOT/'scripts/projective_depth_raw.py',
             ROOT/'scripts/projective_range_raw.py',ROOT/'scripts/projective_information_raw.py')
    sources=(*sources,ROOT/'scripts/projective_whitened_raw.py')
    pins={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources}
    write(out/'freeze.json',dict(binary_sha256=binary_hash,source_sha256=pins,residual=args.residual,noise_seed=1701,
        inverse_depth_std_per_m=.001,frames=12,position_gate_m=.01,rotation_gate_rad=.01,
        noise_role='explicit synthetic constant-disparity hypothesis, not evidence of physical sensor calibration'))
    spec=importlib.util.spec_from_file_location('scene',scene_path);scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    reference=np.full((120,160),4.);measured=np.full((120,160),4.005)
    client=Client(binary,scene.CAMERA,configuration=dict(residual=args.residual))
    try:
        initial=client.frame(1.,reference);failure=client.frame(1.03,measured)
        expired=client.frame(1.201,measured);latched=client.frame(1.21,reference);client.finish()
    except BaseException:client.abort();raise
    write(out/'plane.json',dict(frames=[initial,failure,expired,latched]))
    require(initial['accepted'] and not failure['accepted'] and not failure['lost']
            and failure['rejection']=='insufficient physical observability condition'
            and failure['last_accepted_timestamp']==1.,'weighting cannot create unmeasured planar directions')
    require(expired['lost'] and latched['lost'] and not latched['accepted'],'rejected weighted data cannot renew accepted clock')
    reconstruct_update(reference,(reference>.3).astype(np.uint8),measured,scene.CAMERA,failure,residual_policy=args.residual)
    audit_sequence([dict(update=x) for x in (initial,failure,expired,latched)],[1.,1.03,1.201,1.21])
    rejected=[]
    for name,policy,changed in (
        ('unweighted equation substituted','target_plane',failure),
        ('weight attenuated raw RMS',args.residual,copy.deepcopy(failure))):
        if name=='weight attenuated raw RMS':changed['information_failure']['equation']['rms_m']/=256
        try:reconstruct_update(reference,(reference>.3).astype(np.uint8),measured,scene.CAMERA,changed,residual_policy=policy)
        except (AssertionError,ValueError,KeyError,TypeError,IndexError):rejected.append(name)
        else:raise ValueError('undetected range-weight corruption: '+name)

    transforms=[];depths=[];rng=np.random.default_rng(1701)
    for i in range(12):
        transform=np.eye(4);transform[:3,:3]=exp_rotation(np.array([.001*i,-.0015*i,.0005*i]))
        transform[:3,3]=[.003*i,.002*np.sin(.4*i),.002*(np.cos(.4*i)-1)]
        exact=scene.render(transform)
        depths.append(1/(1/exact+rng.normal(0,.001,exact.shape)));transforms.append(transform)
    ledgers={}
    for mode in ('target_plane',args.residual):
        client=Client(binary,scene.CAMERA,configuration=dict(residual=mode));rows=[]
        try:
            for i,depth in enumerate(depths):rows.append(dict(update=client.frame(2.+.033*i,depth)))
            client.finish()
        except BaseException:client.abort();raise
        ledgers[mode]=rows;write(out/(mode+'-sensor-only.json'),dict(frames=rows,reference_pose_supplied_to_tracker=False))
    summaries={}
    for mode,rows in ledgers.items():
        audit_sequence(rows,[2.+.033*i for i in range(12)])
        model=weights=None;equations=0
        for depth,row in zip(depths,rows,strict=True):
            model,weights,checked=reconstruct_update(model,weights,depth,scene.CAMERA,row['update'],residual_policy=mode)
            equations+=checked['iterations']+checked['final_equations']
        errors=[]
        for transform,row in zip(transforms[1:],rows[1:],strict=True):
            u=row['update']
            if u['accepted']:
                estimate=pose(u['first_from_current'])
                position=float(np.linalg.norm(estimate[:3,3]-transform[:3,3]))
                rotation=float(np.arccos(np.clip((np.trace(transform[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
                errors.append(dict(position_error_m=position,rotation_error_rad=rotation))
        max_p=max((x['position_error_m'] for x in errors),default=None);max_r=max((x['rotation_error_rad'] for x in errors),default=None)
        summaries[mode]=dict(accepted_updates=len(errors),raw_equations=equations,max_position_error_m=max_p,max_rotation_error_rad=max_r,
            physical_control_pass=len(errors)==11 and max_p<=.01 and max_r<=.01,errors=errors)
    passed=all(x['physical_control_pass'] for x in summaries.values())
    require(sha(binary.read_bytes())==binary_hash and all(sha((ROOT/n).read_bytes())==h for n,h in pins.items()),'frozen control implementation')
    from projective_whitened_raw import controls as standardized_controls
    write(out/'summary.json',dict(pass_range_controls=passed,math_controls=controls(),standardized_math_controls=standardized_controls(),weak_plane_raw_checked=True,
        rejected_corruptions=rejected,noisy_synthetic=summaries,calibrated_physical_noise_claim=False,
        scope='conditional synthetic accuracy, measured-weight algebra, raw matrices and failure-state integrity'))
    print(json.dumps(summaries),flush=True)
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
