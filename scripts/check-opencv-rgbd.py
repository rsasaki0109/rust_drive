#!/usr/bin/env python3
"""Independently score and retain every reference outcome; does not audit OpenCV's solver."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CASES = ('tum-fr1-room-temporal', 'tum-fr1-desk2-independent')


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(body):
    return hashlib.sha256(body).hexdigest()


def read(path):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 8*1024*1024,
            'bounded real input required')
    return path.read_bytes()


def document(path):
    def invalid(value):
        raise ValueError('nonfinite JSON: '+value)
    return json.loads(read(path), parse_constant=invalid)


def rotation(q):
    x,y,z,w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def truth_at(rows, stamp):
    right = int(np.searchsorted(rows[:,0],stamp))
    require(right < len(rows), 'no reference extrapolation')
    if rows[right,0] == stamp:
        xyz, q = rows[right,1:4], rows[right,4:]
    else:
        require(right > 0, 'no reference extrapolation')
        a,b = rows[right-1],rows[right]
        require(b[0]-a[0] <= .02, 'reference bracket exceeds20ms')
        alpha = (stamp-a[0])/(b[0]-a[0])
        xyz = a[1:4]+alpha*(b[1:4]-a[1:4])
        qa,qb = a[4:]/np.linalg.norm(a[4:]), b[4:]/np.linalg.norm(b[4:])
        if qa@qb < 0:
            qb = -qb
        perpendicular = qb-(qa@qb)*qa
        length = np.linalg.norm(perpendicular)
        if length < 1e-12:
            q = qa
        else:
            angle = math.atan2(length,float(qa@qb))
            q = math.cos(alpha*angle)*qa+math.sin(alpha*angle)*perpendicular/length
    pose = np.eye(4)
    pose[:3,:3],pose[:3,3] = rotation(q),xyz
    return pose


def controls():
    # Independently hand-computable midpoint: half a metre and 45 degrees.
    rows = np.array([[0,0,0,0,0,0,0,1],
                     [.01,1,0,0,0,0,math.sqrt(.5),math.sqrt(.5)]],dtype=float)
    expected = np.array([[math.sqrt(.5),-math.sqrt(.5),0,.5],
                         [math.sqrt(.5),math.sqrt(.5),0,0],
                         [0,0,1,0],[0,0,0,1]])
    np.testing.assert_allclose(truth_at(rows,.005),expected,atol=1e-12,rtol=0)
    rows[1,4:] *= -1
    np.testing.assert_allclose(truth_at(rows,.005),expected,atol=1e-12,rtol=0)
    rejected = 0
    for stamp in (-.001,.02):
        try:
            truth_at(rows,stamp)
        except ValueError:
            rejected += 1
    rows[1,0] = .03
    try:
        truth_at(rows,.005)
    except ValueError:
        rejected += 1
    require(rejected == 3, 'reference timing controls')
    return dict(hand_calculated_half_metre_45_degree_pose=True, quaternion_sign_invariance=True,
                extrapolation_and_bracket_rejections=3)


def context(case, folder):
    base = ROOT/'data'/case
    body = read(base/'manifest.json')
    manifest = json.loads(body)
    freeze = document(folder/'freeze.json')
    require(freeze['manifest_sha256'] == digest(body), 'manifest source binding')
    require(freeze['evaluator_sha256'] == digest(read(ROOT/'scripts/compare-opencv-rgbd.py')),
            'evaluator source binding')
    require(freeze['opencv_wheel_version'] == '4.13.0.92' and freeze['opencv_version'] == '4.13.0',
            'reference version pin')
    require(freeze['selected_frames'] == 180 and freeze['updates'] == 179, 'complete selection')
    require(freeze['camera'] == [[525.,0.,319.5],[0.,525.,239.5],[0.,0.,1.]], 'fixed registered grid')
    expected = {v['file']:dict(bytes=v['bytes'],sha256=v['sha256']) for v in manifest['files']}
    require(len(expected) == len(manifest['files']) and freeze['sources'] == expected, 'complete inventory binding')
    for name,pin in expected.items():
        require(Path(name).name == name, 'plain source name')
        raw = read(base/'raw'/name)
        require(len(raw) == pin['bytes'] and digest(raw) == pin['sha256'], 'raw source binding')
    import io
    rows = np.loadtxt(io.BytesIO(read(base/'raw/groundtruth.txt')),comments='#')
    require(rows.ndim == 2 and rows.shape[1] == 8 and len(rows) <= 30000
            and np.isfinite(rows).all() and np.all(np.diff(rows[:,0]) > 0)
            and np.all(np.linalg.norm(rows[:,4:],axis=1) > .5), 'whole chronological reference')
    frames = manifest['frames']
    require(len(frames) == 180, 'complete manifest frames')
    origin = truth_at(rows,frames[0]['depth_timestamp'])
    expected_poses = []
    for frame in frames[1:]:
        pose = truth_at(rows,frame['depth_timestamp'])
        root = np.eye(4)
        root[:3,:3] = origin[:3,:3].T@pose[:3,:3]
        root[:3,3] = origin[:3,:3].T@(pose[:3,3]-origin[:3,3])
        expected_poses.append(root)
    return frames,expected_poses,freeze


def audit(case, bundle, ctx):
    frames,truths,freeze = ctx
    report,sensors = bundle['results'],bundle['sensors']
    protocol = report['protocol']
    require(report['dataset'] == case and protocol == freeze['protocol'], 'fixed protocol identity')
    for name,value in dict(ground_truth_operational=False,pose_resets=0,accepted_pose_max_age_s=.2,
                           root_translation_gate_m=.1,root_rotation_gate_rad=.1).items():
        require(protocol[name] == value, 'unchanged gate: '+name)
    outcomes = report['frames']
    require(len(outcomes) == 179 and len(sensors['frames']) == 179 and sensors['ground_truth_parsed'] is False,
            'all outcomes and pre-label ledger retained')
    last = frames[0]['depth_timestamp']
    lost = False
    positions,angles = [],[]
    accurate = 0
    for before,frame,result,truth,sensor in zip(frames,frames[1:],outcomes,truths,sensors['frames']):
        stamp = frame['depth_timestamp']
        require(stamp > before['depth_timestamp'] and frame['rgb_timestamp'] >= before['rgb_timestamp']
                and abs(stamp-frame['rgb_timestamp']) <= .02, 'qualified sensor clocks')
        require(result['source_index'] == frame['source_index'] and result['depth_timestamp'] == stamp,
                'selected acquisition identity')
        operational = {k:v for k,v in result.items() if k not in ('position_error_m','rotation_error_rad','accurate')}
        require(operational == sensor, 'unaltered pre-label outcome')
        lost |= stamp-last > .2
        require(type(result['accepted']) is bool and not (lost and result['accepted']), 'expiry latch')
        if not result['accepted']:
            require('first_camera_from_current' not in result and 'accurate' not in result
                    and isinstance(result['rejection'],str), 'rejection retained')
            continue
        last = stamp
        estimate = np.asarray(result['first_camera_from_current'])
        require(estimate.shape == (4,4) and np.isfinite(estimate).all()
                and np.allclose(estimate[3],[0,0,0,1],atol=1e-9,rtol=0)
                and np.allclose(estimate[:3,:3].T@estimate[:3,:3],np.eye(3),atol=1e-6,rtol=0)
                and abs(np.linalg.det(estimate[:3,:3])-1) <= 1e-6, 'proper rigid pose')
        position = float(np.linalg.norm(estimate[:3,3]-truth[:3,3]))
        delta = truth[:3,:3].T@estimate[:3,:3]
        sine = np.linalg.norm([delta[2,1]-delta[1,2],delta[0,2]-delta[2,0],delta[1,0]-delta[0,1]])/2
        angle = float(math.atan2(float(sine),float((np.trace(delta)-1)/2)))
        require(abs(position-result['position_error_m']) <= 1e-7
                and abs(angle-result['rotation_error_rad']) <= 1e-7, 'independent root errors')
        good = position <= .1 and angle <= .1
        require(type(result['accurate']) is bool and result['accurate'] == good, 'unchanged root gate')
        positions.append(position)
        angles.append(angle)
        accurate += good
    summary = dict(total_updates=179,accepted=len(positions),root_accurate=accurate,lost=lost,
                   max_position_error_m=max(positions,default=None),max_rotation_error_rad=max(angles,default=None),
                   complete_accuracy_pass=accurate == 179 and not lost)
    for key,value in summary.items():
        actual = report['summary'][key]
        require(abs(value-actual) <= 1e-7 if type(value) is float else actual == value,
                'independent summary: '+key)
    require(sensors['lost'] == lost, 'sensor ledger loss state')
    return summary


def corruptions(case, bundle, ctx):
    probes = []
    def probe(name, change):
        modified = copy.deepcopy(bundle)
        change(modified)
        try:
            audit(case,modified,ctx)
        except (ValueError,KeyError,TypeError,IndexError):
            probes.append(name)
        else:
            raise ValueError('undetected corruption: '+name)
    probe('dropped outcome',lambda b:b['results']['frames'].pop())
    probe('changed error',lambda b:b['results']['frames'][0].update(position_error_m=100.))
    probe('changed gate',lambda b:b['results']['protocol'].update(root_translation_gate_m=100.))
    probe('wrong acquisition',lambda b:b['results']['frames'][0].update(depth_timestamp=0.))
    probe('relabelled accuracy',lambda b:b['results']['frames'][-1].update(
        accurate=not b['results']['frames'][-1]['accurate']))
    probe('altered pre-label pose',lambda b:b['sensors']['frames'][0]['first_camera_from_current'][0].__setitem__(3,100.))
    def nonrigid(b):
        for kind in ('results','sensors'):
            b[kind]['frames'][0]['first_camera_from_current'][0][0] = 2.
    probe('nonrigid pose in both ledgers',nonrigid)
    probe('false summary',lambda b:b['results']['summary'].update(
        root_accurate=b['results']['summary']['root_accurate']+1))
    probe('false loss state',lambda b:b['sensors'].update(lost=not b['sensors']['lost']))
    return probes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    require(not args.output.exists() and not args.output.is_symlink(), 'fresh audit output')
    results = {}
    checked_controls = controls()
    combined = document(args.comparison/'comparison.json')
    for case in CASES:
        folder = args.comparison/case
        ctx = context(case,folder)
        bundle = dict(results=document(folder/'results.json'),sensors=document(folder/'sensor-only.json'))
        summary = audit(case,bundle,ctx)
        require(combined['summaries'][case] == bundle['results']['summary'], 'combined complete outcomes')
        results[case] = dict(summary=summary,rejected_corruptions=corruptions(case,bundle,ctx),
                            input_sha256={name:digest(read(folder/name))
                                          for name in ('freeze.json','results.json','sensor-only.json')})
    passed = all(v['summary']['complete_accuracy_pass'] for v in results.values())
    require(combined['complete_accuracy_pass'] == passed and combined['exit_code'] == (0 if passed else 1),
            'accuracy failure exit retained')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(dict(integrity_pass=True,complete_accuracy_pass=passed,cases=results,controls=checked_controls,
                       auditor_sha256=digest(read(Path(__file__))),
                       limitation='audits source bindings, retention, clocks and root scoring; not OpenCV internals'),
                  stream,indent=2,allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
