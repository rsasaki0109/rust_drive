#!/usr/bin/env python3
"""Independent ray-cast motion controls and bounded transport checks for Rust depth fusion."""
import argparse
import copy
import json
from pathlib import Path
import subprocess

import numpy as np
from projective_depth_common import Client, audit_iterations, audit_sequence, exp_rotation, fresh, pose, require, sha, write

CAMERA = dict(width=160,height=120,fx=130.,fy=130.,cx=79.5,cy=59.5)


def render(transform):
    y,x = np.mgrid[:120,:160]
    rays = np.stack([(x-79.5)/130,(y-59.5)/130,np.ones_like(x)],axis=-1)
    direction = rays@transform[:3,:3].T
    origin = transform[:3,3]
    depth = (4-origin[2])/direction[:,:,2]
    for center,radius in (([-.5,-.2,2.2],.4),([.5,.3,2.8],.5),([.1,-.5,3.2],.4)):
        offset = origin-np.array(center)
        a = np.sum(direction*direction,axis=-1)
        b = 2*np.sum(direction*offset,axis=-1)
        discriminant = b*b-4*a*(offset@offset-radius*radius)
        intersection = (-b-np.sqrt(np.maximum(0,discriminant)))/(2*a)
        visible = (discriminant > 0)&(intersection > 0)&(intersection < depth)
        depth[visible] = intersection[visible]
    return depth


def render_gray(transform,depth):
    y,x=np.indices(depth.shape)
    points=np.stack([(x-CAMERA['cx'])*depth/CAMERA['fx'],(y-CAMERA['cy'])*depth/CAMERA['fy'],depth],axis=-1)
    world=points@transform[:3,:3].T+transform[:3,3]
    a,b,c=np.moveaxis(world,-1,0)
    return np.rint(np.clip(128+30*np.sin(3*a)+30*np.cos(4*b)+20*np.sin(2*a+3*b+c),0,255)).astype(np.uint8)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reprojection',choices=('point','tangent_plane'),default='point')
    parser.add_argument('--correspondence',choices=('forward','reciprocal','bilinear'),default='forward')
    parser.add_argument('--residual',choices=('target_plane','symmetric_planes','symmetric_midpoint','bidirectional_midpoint','depth_variance_plane','depth_whitened_plane'),default='target_plane')
    parser.add_argument('--normals',choices=('central_difference','inverse_depth_plane'),default='central_difference')
    parser.add_argument('--geometry',choices=('depth','vertices'),default='depth')
    parser.add_argument('--photometric',choices=('disabled','keyframe'),default='disabled')
    parser.add_argument('--step-control',choices=('disabled','fixed_support'),default='disabled')
    parser.add_argument('--motion-model',choices=('stationary','constant_velocity'),default='stationary')
    parser.add_argument('--fusion-sampling',choices=('nearest','visible_mean'),default='nearest')
    parser.add_argument('--visibility',choices=('disabled','free_space_decay'),default='disabled')
    args = parser.parse_args()
    output = fresh(args.output)
    binary = args.binary.absolute()
    cases = []
    corruption_names = []
    for axis,angle,translation in ((0,0.,[0.,0.,0.]),(0,-.02,[-.01,.02,.01]),
                                   (1,.02,[.02,-.01,.01]),(2,.02,[.01,.01,-.02])):
        transform = np.eye(4)
        vector = np.zeros(3)
        vector[axis] = angle
        transform[:3,:3],transform[:3,3] = exp_rotation(vector),translation
        client = Client(binary,CAMERA,configuration=dict(reprojection=args.reprojection,correspondence=args.correspondence,residual=args.residual,normals=args.normals,geometry=args.geometry,photometric=args.photometric,step_control=args.step_control,motion_model=args.motion_model,fusion_sampling=args.fusion_sampling,visibility=args.visibility))
        def frame(stamp,transform,depth):
            image=None if args.photometric=='disabled' else dict(timestamp=stamp+.001,gray=render_gray(transform,depth).ravel().tolist())
            return client.frame(stamp,depth,image=image)
        try:
            initial = frame(1.,np.eye(4),render(np.eye(4)))
            require(initial['accepted'],'analytic initialization')
            result = frame(1.03,transform,render(transform))
            require(result['accepted'],'analytic motion fit')
            require(all(('step_control' in s)==(args.step_control=='fixed_support') for s in result['iterations']),'explicit step policy')
            steps = audit_iterations(result)
            estimate = pose(result['first_from_current'])
            position = float(np.linalg.norm(estimate[:3,3]-transform[:3,3]))
            rotation = float(np.arccos(np.clip((np.trace(transform[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
            require(position <= .004 and rotation <= .004,'analytic direction/metre/rotation accuracy')
            duplicate = frame(1.03,transform,render(transform))
            require(not duplicate['accepted'] and duplicate['last_accepted_timestamp'] == 1.03,'duplicate cannot renew pose')
            failure = frame(1.1,transform,np.zeros((120,160)))
            require(not failure['accepted'] and failure['last_accepted_timestamp'] == 1.03,'failed fit cannot renew pose')
            expired = frame(1.231,transform,np.zeros((120,160)))
            recovery = frame(1.24,transform,render(transform))
            require(expired['lost'] and recovery['lost'] and not recovery['accepted'],'expired pose has no implicit reset')
            client.finish()
        except BaseException:
            client.abort()
            raise
        cases.append(dict(axis=axis,angle_rad=angle,known_first_from_current=transform.tolist(),
                          position_error_m=position,rotation_error_rad=rotation,checked_normal_steps=steps))
        write(output/('motion-'+str(len(cases))+'.json'),dict(result=result,expected=transform.tolist()))
        baseline = [dict(update=initial),dict(update=result)]
        audit_sequence(baseline,[1.,1.03])
        if angle == 0:
            def corrupt(name, change):
                modified = copy.deepcopy(baseline)
                change(modified[1]['update'])
                try:
                    audit_sequence(modified,[1.,1.03])
                except (ValueError,AssertionError,KeyError,IndexError,TypeError):
                    corruption_names.append(name)
                else:
                    raise ValueError('undetected corruption: '+name)
            corrupt('wrong normal solve',lambda u:u['iterations'][0]['delta'].__setitem__(0,.05))
            corrupt('wrong increment pose',lambda u:u['iterations'][0]['after']['translation'].__setitem__(0,.05))
            corrupt('altered root composition',lambda u:u['first_from_current']['translation'].__setitem__(0,.05))
            corrupt('missing support',lambda u:u['iterations'][0]['equation'].update(count=199))
            corrupt('returned-pose residual exceeds gate',lambda u:u['final_equation'].update(rms_m=.1))
            corrupt('lost fusion assignment',lambda u:u['fusion'].update(averaged=0))
            corrupt('wrong keyframe switch',lambda u:u.update(keyframe_replaced=True))
            corrupt('false accepted clock',lambda u:u.update(last_accepted_timestamp=1.))
            corrupt('wrong acquisition',lambda u:u.update(timestamp=1.04))
            corrupt('false loss state',lambda u:u.update(lost=True))
            if args.residual=='bidirectional_midpoint':
                corrupt('one unsupported direction hidden by sufficient total',lambda u:u['iterations'][0]['equation'].update(
                    direction_counts=[199,u['iterations'][0]['equation']['count']-199]))
                corrupt('one unsupported final direction hidden by sufficient total',lambda u:u['final_equation'].update(
                    direction_counts=[u['final_equation']['count']-199,199]))
    config = json.dumps(dict(kind='Configure',camera=CAMERA))+'\n'
    malformed = {
        'missing configuration':json.dumps(dict(kind='Frame',timestamp=1.,depth_m=[]))+'\n',
        'duplicate configuration':config+config,
        'unknown request field':json.dumps(dict(kind='Configure',camera=CAMERA,truth=[]))+'\n',
        'oversized camera':json.dumps(dict(kind='Configure',camera=dict(CAMERA,width=644)))+'\n',
        'invalid focal length':json.dumps(dict(kind='Configure',camera=dict(CAMERA,fx=0.)))+'\n',
        'unknown reprojection':json.dumps(dict(kind='Configure',camera=CAMERA,reprojection='unknown'))+'\n',
        'unknown correspondence':json.dumps(dict(kind='Configure',camera=CAMERA,correspondence='unknown'))+'\n',
        'unknown residual':json.dumps(dict(kind='Configure',camera=CAMERA,residual='unknown'))+'\n',
        'unknown normal estimation':json.dumps(dict(kind='Configure',camera=CAMERA,normals='unknown'))+'\n',
        'unknown geometry':json.dumps(dict(kind='Configure',camera=CAMERA,geometry='unknown'))+'\n',
        'unknown photometric mode':json.dumps(dict(kind='Configure',camera=CAMERA,photometric='unknown'))+'\n',
        'unknown step control':json.dumps(dict(kind='Configure',camera=CAMERA,step_control='unknown'))+'\n',
        'unknown timing mode':json.dumps(dict(kind='Configure',camera=CAMERA,timing='truth'))+'\n',
        'timing without photometric':json.dumps(dict(kind='Configure',camera=CAMERA,timing='previous_motion',motion_model='constant_velocity'))+'\n',
        'timing without motion history':json.dumps(dict(kind='Configure',camera=CAMERA,timing='previous_motion',photometric='keyframe'))+'\n',
        'timing with fixed support':json.dumps(dict(kind='Configure',camera=CAMERA,timing='previous_motion',photometric='keyframe',motion_model='constant_velocity',step_control='fixed_support'))+'\n',
        'unknown motion model':json.dumps(dict(kind='Configure',camera=CAMERA,motion_model='truth'))+'\n',
        'step control without RGB-D':json.dumps(dict(kind='Configure',camera=CAMERA,step_control='fixed_support'))+'\n',
        'missing final newline':config.rstrip('\n'),
        'no frame':config,
        'oversized request':' '*(8*1024*1024+1)+'\n',
        'nonfinite JSON':config+'{"kind":"Frame","timestamp":NaN,"depth_m":[]}\n',
        'frame cap':config+''.join(json.dumps(dict(kind='Frame',timestamp=1.+i*.03,depth_m=[]))+'\n' for i in range(201)),
    }
    malformed.update({'unsupported vertex '+key:json.dumps(dict(kind='Configure',camera=CAMERA,geometry='vertices',**{key:value}))+'\n'
                      for key,value in (('reprojection','tangent_plane'),('correspondence','reciprocal'),
                                        ('residual','symmetric_planes'),('normals','inverse_depth_plane'))})
    malformed.update({'unsupported photometric '+key:json.dumps(dict(kind='Configure',camera=CAMERA,photometric='keyframe',**{key:value}))+'\n'
                      for key,value in (('geometry','vertices'),('reprojection','tangent_plane'),('correspondence','reciprocal'),
                                        ('residual','symmetric_planes'),('normals','inverse_depth_plane'))})
    malformed['invalid RGB byte']=config+json.dumps(dict(kind='Frame',timestamp=1.,depth_m=[],image=dict(timestamp=1.,gray=[256])))+'\n'
    malformed.update({'unsupported midpoint '+key:json.dumps(dict(kind='Configure',camera=CAMERA,residual='symmetric_midpoint',**{key:value}))+'\n'
                      for key,value in (('geometry','vertices'),('reprojection','tangent_plane'),('correspondence','bilinear'),
                                        ('normals','inverse_depth_plane'),('photometric','keyframe'))})
    malformed.update({'unsupported bidirectional '+key:json.dumps(dict(kind='Configure',camera=CAMERA,residual='bidirectional_midpoint',**{key:value}))+'\n'
                      for key,value in (('geometry','vertices'),('reprojection','tangent_plane'),('correspondence','reciprocal'),
                                        ('normals','inverse_depth_plane'),('photometric','keyframe'))})
    malformed['RGB label injection']=config+json.dumps(dict(kind='Frame',timestamp=1.,depth_m=[],image=dict(timestamp=1.,gray=[],truth=[])))+'\n'
    malformed.update({'unsupported depth variance '+key:json.dumps(dict(kind='Configure',camera=CAMERA,residual='depth_variance_plane',**{key:value}))+'\n'
                      for key,value in (('geometry','vertices'),('reprojection','tangent_plane'),('correspondence','bilinear'),
                                        ('normals','inverse_depth_plane'),('photometric','keyframe'))})
    malformed.update({'unsupported standardized depth '+key:json.dumps(dict(kind='Configure',camera=CAMERA,residual='depth_whitened_plane',**{key:value}))+'\n'
                      for key,value in (('geometry','vertices'),('reprojection','tangent_plane'),('correspondence','reciprocal'),
                                        ('normals','inverse_depth_plane'),('photometric','keyframe'))})
    malformed['unknown fusion sampling']=json.dumps(dict(kind='Configure',camera=CAMERA,fusion_sampling='truth'))+'\n'
    malformed.update({'unsupported visible mean '+key:json.dumps(dict(kind='Configure',camera=CAMERA,fusion_sampling='visible_mean',**{key:value}))+'\n'
                      for key,value in (('geometry','vertices'),('reprojection','tangent_plane'),('correspondence','reciprocal'),
                                        ('residual','depth_whitened_plane'),('normals','inverse_depth_plane'),('photometric','keyframe'),
                                        ('motion_model','constant_velocity'),('step_control','fixed_support'),('timing','previous_motion'))})
    malformed['unknown visibility']=json.dumps(dict(kind='Configure',camera=CAMERA,visibility='truth'))+'\n'
    malformed.update({'unsupported visibility '+key:json.dumps(dict(kind='Configure',camera=CAMERA,visibility='free_space_decay',**{key:value}))+'\n'
                      for key,value in (('fusion_sampling','visible_mean'),('geometry','vertices'),('reprojection','tangent_plane'),
                                        ('correspondence','bilinear'),('residual','symmetric_planes'),('normals','inverse_depth_plane'),
                                        ('photometric','keyframe'),('motion_model','constant_velocity'),
                                        ('step_control','fixed_support'),('timing','previous_motion'))})
    for name,body in malformed.items():
        result = subprocess.run([str(binary)],input=body,text=True,capture_output=True,timeout=30)
        require(result.returncode == 2 and bool(result.stderr),'transport rejection: '+name)
        # Exit 2 at EOF is also returned for a valid Configure with no frames.
        # Invalid configuration must reject before acknowledging it; otherwise
        # that unrelated EOF error would make these controls pass incorrectly.
        acknowledgments = {'duplicate configuration','no frame','nonfinite JSON','frame cap','invalid RGB byte','RGB label injection'}
        if name not in acknowledgments:
            require(result.stdout == '', 'rejected before any acknowledgment: '+name)
        else:
            lines = result.stdout.splitlines()
            require(len(lines) == (201 if name=='frame cap' else 1)
                    and json.loads(lines[0]) == dict(configured=True,schema_version=1),
                    'exact acknowledged prefix before rejection: '+name)
    write(output/'summary.json',dict(analytic_controls_pass=True,translation_gate_m=.004,rotation_gate_rad=.004,
        motions=cases,reprojection=args.reprojection,correspondence=args.correspondence,residual=args.residual,normals=args.normals,geometry=args.geometry,photometric=args.photometric,step_control=args.step_control,motion_model=args.motion_model,rejected_transport_cases=list(malformed),binary_sha256=sha(binary.read_bytes()),
        rejected_report_corruptions=corruption_names,
        checker_sha256=sha(Path(__file__).read_bytes()),
        scope='analytic geometry and independent linear algebra; no physical accuracy or real-time claim',fusion_sampling=args.fusion_sampling,visibility=args.visibility))


if __name__ == '__main__':
    main()
