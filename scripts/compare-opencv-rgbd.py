#!/usr/bin/env python3
"""Fixed OpenCV RGB-D reference on viewed TUM windows; not a Rust dependency."""
import argparse
import hashlib
import importlib.metadata
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

CAMERA = np.array([[525., 0., 319.5], [0., 525., 239.5], [0., 0., 1.]])
PROTOCOL = dict(method='OpenCV RgbdICPOdometry', min_depth_m=.3, max_depth_m=5.,
                max_depth_difference_m=.07, random_seed=0, threads=1,
                source_to_destination='current_to_last_accepted', root_origin='first_depth_camera',
                primary_clock='strictly new depth; associated nondecreasing RGB within20ms',
                accepted_pose_max_age_s=.2, pose_resets=0,
                root_translation_gate_m=.1, root_rotation_gate_rad=.1,
                ground_truth_operational=False, data_role='already viewed development/regression',
                timing_claim=False)


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(body):
    return hashlib.sha256(body).hexdigest()


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def fresh(path):
    path = path.absolute()
    require(not path.exists() and not path.is_symlink() and
            not any(p.is_symlink() for p in path.parents), 'fresh real output directory required')
    path.mkdir(parents=True)
    return path


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion_matrix(q):
    q = q/np.linalg.norm(q)
    cross = skew(q[:3])
    return np.eye(3)+2*q[3]*cross+2*cross@cross


def reference(rows, stamp):
    index = np.searchsorted(rows[:, 0], stamp)
    if index < len(rows) and rows[index, 0] == stamp:
        value = rows[index]
        result = np.eye(4)
        result[:3, :3] = quaternion_matrix(value[4:])
        result[:3, 3] = value[1:4]
        return result
    require(0 < index < len(rows), 'reference extrapolation is prohibited')
    a, b = rows[index-1], rows[index]
    require(b[0]-a[0] <= .02, 'reference bracket exceeds20ms')
    alpha = (stamp-a[0])/(b[0]-a[0])
    qa, qb = a[4:]/np.linalg.norm(a[4:]), b[4:]/np.linalg.norm(b[4:])
    dot = qa@qb
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > .9995:
        q = (1-alpha)*qa+alpha*qb
    else:
        angle = np.arccos(np.clip(dot, -1., 1.))
        q = (np.sin((1-alpha)*angle)*qa+np.sin(alpha*angle)*qb)/np.sin(angle)
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(q)
    result[:3, 3] = (1-alpha)*a[1:4]+alpha*b[1:4]
    return result


def controls():
    # Independent one-ray unit/frame witness before touching recorded pixels.
    k = np.array([[2.,0.,0.],[0.,4.,0.],[0.,0.,1.]])
    depth = np.full((3, 4), 4., dtype=np.float32)
    actual = cv2.rgbd.depthTo3d(depth, k)
    np.testing.assert_allclose(actual[2, 1], [2., 2., 4.], atol=1e-7, rtol=0)
    y, x = np.mgrid[:120, :160]
    image = ((x*37+y*71+x*y*3) % 256).astype(np.uint8)
    depth = (1.+.1*np.sin(x/10)+.1*np.cos(y/11)).astype(np.float32)
    k = np.array([[130.,0.,79.5],[0.,130.,59.5],[0.,0.,1.]])
    method = cv2.rgbd.RgbdICPOdometry_create(k, .3, 5., .07)
    success, transform = method.compute(image, depth, None, image, depth, None)
    require(success, 'stationary analytic reference fit failed')
    np.testing.assert_allclose(transform, np.eye(4), atol=1e-7, rtol=0)
    # Analytic ray/sphere intersections give independently known, nonzero poses.
    # This checks transform direction as well as metres and rotation composition.
    y, x = np.mgrid[:240, :320]
    k = np.array([[260.,0.,159.5],[0.,260.,119.5],[0.,0.,1.]])
    rays = np.stack([(x-159.5)/260, (y-119.5)/260, np.ones_like(x)], axis=-1)

    def render(pose):
        direction = rays@pose[:3,:3].T
        origin = pose[:3,3]
        depth = (4-origin[2])/direction[:,:,2]
        for center, radius in (([-.5,-.2,2.2],.4),([.5,.3,2.8],.5),([.1,-.5,3.2],.4)):
            offset = origin-np.asarray(center)
            a = np.sum(direction*direction,axis=-1)
            b = 2*np.sum(direction*offset,axis=-1)
            discriminant = b*b-4*a*(offset@offset-radius*radius)
            distance = (-b-np.sqrt(np.maximum(0,discriminant)))/(2*a)
            visible = (discriminant>0)&(distance>0)&(distance<depth)
            depth[visible] = distance[visible]
        point = origin+direction*depth[:,:,None]
        image = np.clip(127+50*np.sin(8*point[:,:,0])+30*np.cos(6*point[:,:,1])
                        +20*np.sin(5*point[:,:,2]),0,255).astype(np.uint8)
        return image, depth.astype(np.float32)

    image0, depth0 = render(np.eye(4))
    motions = []
    for axis, angle, translation in ((1,.02,[.02,-.01,.01]),
                                      (0,-.02,[-.01,.02,.01]),
                                      (2,.02,[.01,.01,-.02])):
        vector = np.zeros(3)
        vector[axis] = np.sin(angle/2)
        pose = np.eye(4)
        pose[:3,:3] = quaternion_matrix(np.r_[vector,np.cos(angle/2)])
        pose[:3,3] = translation
        image1, depth1 = render(pose)
        cv2.setRNGSeed(0)
        method = cv2.rgbd.RgbdICPOdometry_create(k,.3,5.,.07)
        success, transform = method.compute(image1,depth1,None,image0,depth0,None)
        require(success, 'analytic moving reference fit failed')
        position = float(np.linalg.norm(transform[:3,3]-pose[:3,3]))
        rotation = float(np.arccos(np.clip((np.trace(pose[:3,:3].T@transform[:3,:3])-1)/2,-1,1)))
        require(position <= .002 and rotation <= .002, 'nonzero direction/unit control failed')
        motions.append(dict(axis=axis, known_first_from_current=pose.tolist(),
                            position_error_m=position, rotation_error_rad=rotation))
    return dict(one_ray_metre_coordinates=True, stationary_analytic_pose=True,
                nonzero_motion_translation_gate_m=.002, nonzero_motion_rotation_gate_rad=.002,
                nonzero_motion_controls=motions)


def evaluate(dataset, output):
    base = Path('data')/dataset
    raw = base/'raw'
    manifest_bytes = (base/'manifest.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    require(dataset in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'), 'fixed viewed records only')
    frames = manifest['frames']
    require(len(frames) == 180, 'retain every selected frame')
    inventory = {}
    for item in manifest['files']:
        require(Path(item['file']).name == item['file'] and item['file'] not in inventory, 'plain unique input file')
        require(type(item['bytes']) is int and 0 < item['bytes'] <= 4*1024*1024, 'bounded raw input')
        inventory[item['file']] = item

    def verified(name):
        pin = inventory[name]
        path = raw/name
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 4*1024*1024, 'regular bounded source')
        body = path.read_bytes()
        require(len(body) == pin['bytes'] and sha(body) == pin['sha256'], 'source pin mismatch: '+name)
        return body

    for frame in frames:
        require(math.isfinite(frame['depth_timestamp']) and math.isfinite(frame['rgb_timestamp'])
                and abs(frame['rgb_timestamp']-frame['depth_timestamp']) <= .02, 'qualified acquisition stamps')
        for name in (frame['depth_file'],frame['rgb_file']):
            verified(name)
    require(all(b['depth_timestamp'] > a['depth_timestamp'] and b['rgb_timestamp'] >= a['rgb_timestamp']
                for a,b in zip(frames,frames[1:])), 'strict depth and nondecreasing RGB source clocks')
    verified('groundtruth.txt')  # Encoded hash only, no pose parsing here.
    method = cv2.rgbd.RgbdICPOdometry_create(CAMERA, .3, 5., .07)
    write(output/'freeze.json', dict(protocol=PROTOCOL, manifest_sha256=sha(manifest_bytes),
        sources={name:dict(bytes=item['bytes'],sha256=item['sha256']) for name,item in inventory.items()},
        camera=CAMERA.tolist(), selected_frames=180, updates=179,
        opencv_wheel_version=importlib.metadata.version('opencv-contrib-python-headless'),
        opencv_version=cv2.__version__, numpy_version=np.__version__,
        opencv_build_sha256=sha(cv2.getBuildInformation().encode()),
        opencv_library_sha256=sha((Path(cv2.__file__).parent/'cv2.abi3.so').read_bytes()),
        opencv_licenses={name:sha((Path(cv2.__file__).parent/name).read_bytes())
                         for name in ('LICENSE.txt','LICENSE-3RD-PARTY.txt')},
        evaluator_sha256=sha(Path(__file__).read_bytes()),
        resolved_defaults=dict(iterations=method.getIterationCounts().ravel().tolist(),
            min_gradient_magnitudes=method.getMinGradientMagnitudes().ravel().tolist(),
            max_points_part=method.getMaxPointsPart(), max_rotation=method.getMaxRotation(),
            max_translation=method.getMaxTranslation())))

    def image_pair(frame):
        import io
        image = np.asarray(Image.open(io.BytesIO(verified(frame['rgb_file']))), dtype=np.uint32)
        require(image.shape == (480,640,3), 'RGB layout')
        gray = ((77*image[:,:,0]+150*image[:,:,1]+29*image[:,:,2]) >> 8).astype(np.uint8)
        depth = np.asarray(Image.open(io.BytesIO(verified(frame['depth_file']))), dtype=np.float32)/5000.
        require(depth.shape == (480,640), 'depth layout')
        depth[depth == 0] = np.nan
        return gray, depth

    previous = image_pair(frames[0])
    root, last_accepted, lost = np.eye(4), frames[0]['depth_timestamp'], False
    outcomes = []
    for frame in frames[1:]:
        stamp = frame['depth_timestamp']
        result = dict(source_index=frame['source_index'], depth_timestamp=stamp, accepted=False)
        if stamp-last_accepted > .2:
            lost = True
        try:
            require(not lost, 'accepted-pose expiry latched; no implicit reset')
            current = image_pair(frame)
            success, transform = method.compute(current[0], current[1], None, previous[0], previous[1], None)
            require(success and transform.shape == (4,4) and np.isfinite(transform).all(),
                    'reference solver rejected the frame')
            require(np.allclose(transform[3], [0.,0.,0.,1.], atol=1e-9, rtol=0)
                    and np.allclose(transform[:3,:3].T@transform[:3,:3], np.eye(3),atol=1e-6,rtol=0)
                    and abs(np.linalg.det(transform[:3,:3])-1) <= 1e-6, 'nonrigid reference result')
            root = root@transform
            previous, last_accepted = current, stamp
            result.update(accepted=True, first_camera_from_current=root.tolist())
        except (ValueError, cv2.error) as error:
            result['rejection'] = str(error)
        outcomes.append(result)
    write(output/'sensor-only.json', dict(frames=outcomes, lost=lost, ground_truth_parsed=False))
    # Numeric labels are parsed only after every operational update.
    import io
    labels = np.loadtxt(io.BytesIO(verified('groundtruth.txt')), comments='#')
    require(labels.ndim == 2 and labels.shape[1] == 8 and len(labels) <= 30000
            and np.isfinite(labels).all() and np.all(np.diff(labels[:,0]) > 0), 'complete chronological reference')
    origin = np.linalg.inv(reference(labels, frames[0]['depth_timestamp']))
    for result in outcomes:
        truth = origin@reference(labels, result['depth_timestamp'])
        if result['accepted']:
            estimate = np.asarray(result['first_camera_from_current'])
            position = np.linalg.norm(estimate[:3,3]-truth[:3,3])
            rotation = np.arccos(np.clip((np.trace(truth[:3,:3].T@estimate[:3,:3])-1)/2,-1.,1.))
            result.update(position_error_m=float(position),rotation_error_rad=float(rotation),
                          accurate=bool(position <= .1 and rotation <= .1))
    summary = dict(total_updates=179, accepted=sum(v['accepted'] for v in outcomes),
                   root_accurate=sum(v.get('accurate',False) for v in outcomes), lost=lost,
                   max_position_error_m=max((v['position_error_m'] for v in outcomes if v['accepted']),default=None),
                   max_rotation_error_rad=max((v['rotation_error_rad'] for v in outcomes if v['accepted']),default=None))
    summary['complete_accuracy_pass'] = summary['root_accurate'] == 179 and not lost
    write(output/'results.json', dict(schema_version=1,dataset=dataset,protocol=PROTOCOL,summary=summary,frames=outcomes))
    print(dataset,json.dumps(summary),flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--controls-only',action='store_true',help='analytic controls without recorded data')
    args = parser.parse_args()
    output = fresh(args.output)
    require(importlib.metadata.version('opencv-contrib-python-headless') == '4.13.0.92', 'pinned reference wheel')
    cv2.setNumThreads(1)
    cv2.setRNGSeed(0)
    write(output/'controls.json',controls())
    if args.controls_only:
        return 0
    summaries = {}
    for dataset in ('tum-fr1-room-temporal','tum-fr1-desk2-independent'):
        cv2.setRNGSeed(0)
        folder = output/dataset
        folder.mkdir()
        summaries[dataset] = evaluate(dataset,folder)
    passed = all(v['complete_accuracy_pass'] for v in summaries.values())
    write(output/'comparison.json',dict(protocol=PROTOCOL, summaries=summaries,
                                      complete_accuracy_pass=passed, exit_code=0 if passed else 1))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
