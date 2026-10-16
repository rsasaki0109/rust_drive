#!/usr/bin/env python3
"""Reverse-frame equation checks on a ray-cast scene with unequal visibility."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np
from projective_depth_common import ROOT, Client, audit_sequence, exp_rotation, fresh, require, sha, write
from projective_depth_raw import normal_equation, pyramid, reconstruct_update


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--binary',type=Path,help='also check native plane rejection and atomic expiry')
    args=parser.parse_args();out=fresh(args.output)
    scene_path=ROOT/'scripts/check-projective-depth.py'
    spec=importlib.util.spec_from_file_location('scene',scene_path)
    scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    transform=np.eye(4)
    transform[:3,:3]=exp_rotation(np.array([.017,-.023,.011]))
    transform[:3,3]=[.024,-.016,.008]
    first=scene.render(np.eye(4));second=scene.render(transform)
    first[24:38,52:88]=0.;second[44:63,30:76]=0.
    a,b=pyramid(first,scene.CAMERA),pyramid(second,scene.CAMERA)
    inverse=np.linalg.inv(transform);r=inverse[:3,:3];x,y,z=inverse[:3,3]
    cross=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    adjoint=np.block([[r,cross@r],[np.zeros((3,3)),r]])
    rows=[]
    for scale in (4,2,1):
        forward=normal_equation(a[scale],b[scale],transform,scale,residual_policy='bidirectional_midpoint')
        reverse=normal_equation(b[scale],a[scale],inverse,scale,residual_policy='bidirectional_midpoint')
        counts=forward['direction_counts']
        require(min(counts)>=200 and counts[0]!=counts[1]
                and counts==reverse['direction_counts'][::-1], 'separate unequal directional support swaps')
        h=adjoint.T@reverse['h']@adjoint;g=-adjoint.T@reverse['g']
        np.testing.assert_allclose(forward['h'],h,atol=1e-9,rtol=1e-11)
        np.testing.assert_allclose(forward['g'],g,atol=1e-10,rtol=1e-11)
        require(abs(forward['rms_m']-reverse['rms_m'])<1e-12,'reversed scalar objective')
        rows.append(dict(scale=scale,direction_counts=counts,
            matrix_transport_max_error=float(np.max(abs(forward['h']-h))),
            gradient_transport_max_error=float(np.max(abs(forward['g']-g)))))
    planes=[]
    if args.binary is not None:
        depth=np.full((scene.CAMERA['height'],scene.CAMERA['width']),2.)
        for mode in ('symmetric_midpoint','bidirectional_midpoint'):
            client=Client(args.binary,scene.CAMERA,configuration=dict(residual=mode))
            try:
                initial=client.frame(1.,depth);failure=client.frame(1.03,depth)
                expired=client.frame(1.201,depth);latched=client.frame(1.21,depth);client.finish()
            except BaseException:client.abort();raise
            require(initial['accepted'] and not failure['accepted'] and not failure['lost']
                    and failure['rejection']=='insufficient physical observability condition'
                    and failure['last_accepted_timestamp']==1.,'two directions cannot create planar information')
            require(expired['lost'] and latched['lost'] and not latched['accepted'],'plane failures do not renew accepted clock')
            reconstruct_update(depth,(depth>.3).astype(np.uint8),depth,scene.CAMERA,failure,residual_policy=mode)
            audit_sequence([dict(update=u) for u in (initial,failure,expired,latched)],[1.,1.03,1.201,1.21])
            planes.append(dict(residual=mode,rejected_weak_information=True,raw_failure_witness_checked=True,expiry_latched=True))
            write(out/(mode+'-plane.json'),dict(frames=[initial,failure,expired,latched]))
    sources=(Path(__file__),scene_path,ROOT/'scripts/projective_depth_common.py',ROOT/'scripts/projective_depth_raw.py')
    report=dict(pass_algebraic_symmetry=True,controls=rows,native_plane_controls=planes,
        binary_sha256=sha(args.binary.read_bytes()) if args.binary else None,numeric_recorded_truth_decoded=False,
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources},
        scope='equation transport, reversed scalar objective and unequal visibility; native correctness is independently reconstructed from raw pixels')
    write(out/'summary.json',report);print(json.dumps(report),flush=True)


if __name__=='__main__':main()
