#!/usr/bin/env python3
"""Analytic pixels, raw matrix reconstruction, reference state and malformed CLI checks."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np
from multireference_rgbd_common import MultiClient, audit_sequence, compare_raw, raw_equation, raw_reference
from projective_depth_common import ROOT, exp_rotation, fresh, pose, require, sha, write

spec = importlib.util.spec_from_file_location('independent_rays',ROOT/'scripts/check-projective-depth.py')
rays = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rays)
CAMERA = rays.CAMERA


def texture(tx, ty):
    y,x = np.mgrid[:120,:160]
    x,y = (x-79.5)*2/130+tx,(y-59.5)*2/130+ty
    return np.rint(127+40*np.sin(8*x)+30*np.cos(6*y)+20*np.sin(3*x+2*y)).clip(0,255).astype(np.uint8)


def sequence(binary, mode, transforms, plane=False, phase=.001):
    client = MultiClient(binary,CAMERA,mode)
    rows,frames = [],[]
    raw = {}
    equations = 0
    try:
        for index,truth in enumerate(transforms):
            stamp = 1.+index*.03
            gray = None
            if plane:
                y,x = np.mgrid[:120,:160]
                direction = np.stack([(x-79.5)/130,(y-59.5)/130,np.ones_like(x)],axis=-1)@truth[:3,:3].T
                depth = (2.-truth[2,3])/direction[:,:,2]
                world = direction*depth[:,:,None]+truth[:3,3]
                x,y = world[:,:,0],world[:,:,1]
                gray = np.rint(127+40*np.sin(8*x)+30*np.cos(6*y)+20*np.sin(3*x+2*y)).clip(0,255).astype(np.uint8)
            else:
                depth = rays.render(truth)
            image = dict(timestamp=stamp+phase,gray=gray.ravel().tolist()) if gray is not None else None
            u = client.observation(stamp,depth,image)
            rows.append(dict(update=u))
            frames.append(dict(depth_timestamp=stamp,rgb_timestamp=stamp+phase))
            source = raw_reference(depth,gray,CAMERA,np.eye(4),phase)
            refs = [raw[t] for t in u['reference_timestamps']]
            for step in u['iterations']:
                expected = raw_equation(refs,source,pose(step['before']),step['scale'],u['photo_enabled'])
                compare_raw(step['equation'],expected)
                equations += 1
            require(u['accepted'],'analytic motion accepted: '+str(u['rejection']))
            estimate = pose(u['first_from_current'])
            if index:
                compare_raw(u['final_equation'],raw_equation(refs,source,estimate,1,u['photo_enabled']))
                equations += 1
            p = np.linalg.norm(estimate[:3,3]-truth[:3,3])
            a = np.arccos(np.clip((np.trace(truth[:3,:3].T@estimate[:3,:3])-1)/2,-1,1))
            require(p <= .004 and a <= .004,'analytic metric/direction accuracy')
            source['root'] = estimate
            raw[stamp] = source
        client.finish()
    except BaseException:
        client.abort()
        raise
    steps = audit_sequence(rows,frames,mode)
    return dict(frames=rows,metadata=frames,checked_raw_equations=equations,checked_normal_steps=steps)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = fresh(args.output)
    binary = args.binary.absolute()
    reports = []
    for axis,angle,translation in ((0,0.,[0.,0.,0.]),(0,-.02,[-.01,.02,.01]),
                                   (1,.02,[.02,-.01,.01]),(2,.02,[.01,.01,-.02])):
        truth = np.eye(4)
        v = np.zeros(3); v[axis] = angle
        truth[:3,:3],truth[:3,3] = exp_rotation(v),translation
        report = sequence(binary,'geometry',[np.eye(4),truth])
        reports.append(report)
        write(output/('motion-'+str(len(reports))+'.json'),report)
    truth = np.eye(4); truth[:3,3] = [.02,-.01,0.]
    plane = sequence(binary,'phase_gated_rgbd',[np.eye(4),truth],plane=True)
    reports.append(plane)
    write(output/'textured-plane.json',plane)
    # Enough translation to exercise eviction while the first map origin remains retained.
    path = []
    for i in range(18):
        truth = np.eye(4); truth[:3,3] = [.03*i,0.,0.]
        path.append(truth)
    multi = sequence(binary,'geometry',path)
    require(max(len(r['update']['reference_timestamps']) for r in multi['frames']) == 5,
            'exercise full reference budget')
    require(sum(r['update']['keyframe_replaced'] for r in multi['frames']) >= 4,'exercise oldest-key eviction')
    reports.append(multi)
    write(output/'reference-eviction.json',multi)
    # Nonidentity image-reference rotations expose derivative cell selection
    # hidden by a two-frame, identity-reference image test.
    path = []
    for i in range(8):
        truth = np.eye(4)
        truth[:3,:3] = exp_rotation(np.array([.001,-.002,.003])*i)
        truth[:3,3] = [.02*i,-.01*i,.001*i]
        path.append(truth)
    rotated = sequence(binary,'phase_gated_rgbd',path,plane=True)
    reports.append(rotated)
    write(output/'rotated-image-references.json',rotated)
    corruptions = []
    baseline = plane
    def corrupt(name, change):
        rows = copy.deepcopy(baseline['frames'])
        change(rows[1]['update'])
        try:
            audit_sequence(rows,baseline['metadata'],'phase_gated_rgbd')
        except (AssertionError,ValueError,KeyError,TypeError,IndexError):
            corruptions.append(name)
        else:
            raise ValueError('undetected corruption: '+name)
    corrupt('wrong combined solve',lambda u:u['iterations'][0]['delta'].__setitem__(0,.09))
    corrupt('wrong root pose',lambda u:u['first_from_current']['translation'].__setitem__(0,.2))
    corrupt('missing first reference',lambda u:u.update(reference_timestamps=[]))
    corrupt('forged accepted clock',lambda u:u.update(last_accepted_timestamp=1.))
    corrupt('lost latch',lambda u:u.update(lost=True))
    corrupt('excess final residual',lambda u:u['final_equation']['geometry'].update(rms_m=.1))
    corrupt('image count partition',lambda u:u['iterations'][0]['equation'].update(photometric_count=0))
    corrupt('key replacement',lambda u:u.update(keyframe_replaced=True))
    corrupt('RGB freshness',lambda u:u.update(photo_enabled=False))
    # Jointly scaled H/g preserves the solution; raw pixels must reject this corruption.
    u = plane['frames'][1]['update']; step = u['iterations'][0]
    first = raw_reference(np.full((120,160),2.),texture(0.,0.),CAMERA,np.eye(4),.001)
    source = raw_reference(np.full((120,160),2.),texture(.02,-.01),CAMERA,np.eye(4),.001)
    bad = copy.deepcopy(step['equation'])
    bad['h'] = (np.asarray(bad['h'])*2).tolist(); bad['g'] = (np.asarray(bad['g'])*2).tolist()
    try:
        compare_raw(bad,raw_equation([first],source,pose(step['before']),step['scale'],True))
    except AssertionError:
        corruptions.append('joint scaling hidden from solve-only audit')
    else:
        raise ValueError('undetected raw matrix scaling')
    config = json.dumps(dict(kind='Configure',camera=CAMERA,mode='geometry'))+'\n'
    invalid = {
        'missing configuration':'{"kind":"Frame","timestamp":1,"depth_m":[]}\n',
        'duplicate configuration':config+config,
        'unknown configuration field':json.dumps(dict(kind='Configure',camera=CAMERA,mode='geometry',truth=[]))+'\n',
        'invalid mode':json.dumps(dict(kind='Configure',camera=CAMERA,mode='truth'))+'\n',
        'oversized camera':json.dumps(dict(kind='Configure',camera=dict(CAMERA,width=644),mode='geometry'))+'\n',
        'invalid focal length':json.dumps(dict(kind='Configure',camera=dict(CAMERA,fx=0),mode='geometry'))+'\n',
        'unknown image field':config+'{"kind":"Frame","timestamp":1,"depth_m":[],"image":{"timestamp":1,"gray":[],"truth":[]}}\n',
        'out of range grayscale':config+'{"kind":"Frame","timestamp":1,"depth_m":[],"image":{"timestamp":1,"gray":[256]}}\n',
        'missing newline':config.rstrip('\n'),
        'no acquisition':config,
        'oversized line':' '*(8*1024*1024+1)+'\n',
        'nonfinite JSON':config+'{"kind":"Frame","timestamp":NaN,"depth_m":[]}\n',
        'frame cap':config+''.join(json.dumps(dict(kind='Frame',timestamp=1+i*.03,depth_m=[]))+'\n' for i in range(201)),
    }
    for name,body in invalid.items():
        p = subprocess.run([str(binary)],input=body,text=True,capture_output=True,timeout=30)
        require(p.returncode == 2 and bool(p.stderr),'transport rejection: '+name)
    write(output/'summary.json',dict(analytic_controls_pass=True,analytic_sequences=len(reports),
        checked_raw_equations=sum(r['checked_raw_equations'] for r in reports),
        checked_normal_steps=sum(r['checked_normal_steps'] for r in reports),
        translation_gate_m=.004,rotation_gate_rad=.004,rejected_report_corruptions=corruptions,
        rejected_transport_cases=list(invalid),binary_sha256=sha(binary.read_bytes()),
        checker_sha256=sha(Path(__file__).read_bytes()),
        scope='synthetic pixels, independent raw matrices and state; no physical or real-time claim'))


if __name__ == '__main__':
    main()
