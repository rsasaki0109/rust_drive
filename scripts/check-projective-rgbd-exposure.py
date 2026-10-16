#!/usr/bin/env python3
"""Independent exposure/motion controls, including nuisance-only image texture."""
import argparse
from pathlib import Path
import numpy as np
from projective_depth_common import Client,audit_sequence,fresh,pose,require,sha,write
from projective_depth_raw import reconstruct_update,vertices

CAMERA=dict(width=160,height=120,fx=130.,fy=130.,cx=79.5,cy=59.5)


def texture(transform):
    p=vertices(np.full((120,160),2.5),CAMERA)@transform[:3,:3].T+transform[:3,3]
    return np.rint(128+30*np.sin(7*p[:,:,0])+25*np.cos(9*p[:,:,1])+20*np.sin(4*p[:,:,0]+5*p[:,:,1])).astype(np.uint8)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--step-control',choices=('disabled','fixed_support'),default='disabled')
    args=p.parse_args();out=fresh(args.output);binary_sha=sha(args.binary.read_bytes());results={};checked=0
    stationary=np.eye(4);moving=np.eye(4);a=.01;moving[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]];moving[:2,3]=[.015,-.01]
    for name,transform,alpha,beta,outliers in (
        ('stationary darkened',stationary,.75,25.,False),('moving darkened',moving,.75,25.,False),
        ('moving brightened',moving,1.1,5.,False),('moving exposed with outliers',moving,.75,25.,True),
        ('gain out of bounds',stationary,.25,20.,False),('offset out of bounds',stationary,1.,-90.,False),
        ('uniform image',stationary,1.,0.,False),('brightness explains image gradient',stationary,1.,15.,False)):
        depth=np.full((120,160),2.5);first=texture(stationary);current=texture(transform)
        if name=='uniform image':first[:]=100;current[:]=100
        if name=='brightness explains image gradient':
            first=np.tile(np.arange(160,dtype=np.uint8),(120,1))+40;current=first.copy()
        current=np.rint(alpha*current.astype(float)+beta).clip(0,255).astype(np.uint8)
        if outliers:current.ravel()[::19]=255
        expected_accept=name.startswith(('stationary darkened','moving '))
        client=Client(args.binary,CAMERA,configuration=dict(photometric='keyframe',step_control=args.step_control))
        try:
            initial=client.frame(1.,depth,image=dict(timestamp=1.001,gray=first.ravel().tolist()))
            update=client.frame(1.03,depth,image=dict(timestamp=1.031,gray=current.ravel().tolist()));client.finish()
        except BaseException:
            client.abort();raise
        require(initial['accepted'] and update['accepted']==expected_accept,'exposure/observability admission: '+name)
        audit_sequence([dict(update=initial),dict(update=update)],[1.,1.03])
        context=dict(target=first,source=current,source_timestamp=1.031,reference_timestamp=1.001,qualified=True,fresh=True)
        _,_,counts=reconstruct_update(depth,(depth>.3).astype(np.uint8),depth,CAMERA,update,image_context=context)
        checked+=counts['iterations']+counts['final_equations']
        result=dict(expected_accepted=expected_accept,initial=initial,update=update)
        if expected_accept:
            estimate=pose(update['first_from_current']);error=np.linalg.inv(transform)@estimate
            translation=float(np.linalg.norm(error[:3,3]));rotation=float(np.arccos(np.clip((np.trace(error[:3,:3])-1)/2,-1,1)))
            require(translation<.004 and rotation<.004,'known motion unaffected by exposure: '+name)
            exposure=update['final_photometric']['illumination']
            require(abs(exposure['gain']-1/alpha)<.03 and abs(exposure['offset_gray']+beta/alpha)<3,'measured exposure recovery')
            result.update(translation_error_m=translation,rotation_error_rad=rotation)
        else:
            require(update['last_accepted_timestamp']==1. and update['fusion'] is None,'unobservable/rejected image cannot renew pose')
        results[name]=result
    require(sha(args.binary.read_bytes())==binary_sha,'binary fixed throughout controls')
    write(out/'controls.json',results)
    write(out/'summary.json',dict(pass_exposure_controls=True,positive_controls=4,negative_controls=4,step_control=args.step_control,
        independently_reconstructed_systems=checked,binary_sha256=binary_sha,checker_sha256=sha(Path(__file__).read_bytes()),
        known_pose_role='Synthetic measurement generation and scoring; no pose input to tracker'))


if __name__=='__main__':main()
