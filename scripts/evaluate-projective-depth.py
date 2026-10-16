#!/usr/bin/env python3
"""Source-frozen Rust projective-depth comparison on already viewed TUM recordings."""
import argparse
import gzip
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image
from projective_depth_common import CAMERA, ROOT, Client, audit_sequence, fresh, pose, require, sha, write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
VIEWED_CASES = (*CASES,'tum-fr1-rpy-heldout')


def evaluate(case, binary, output, reprojection='point', correspondence='forward', residual='target_plane', normals='central_difference', geometry='depth', photometric='disabled', step_control='disabled',motion_model='stationary',timing='disabled',fusion_sampling='nearest',visibility='disabled'):
    base = ROOT/'data'/case
    manifest_body = (base/'manifest.json').read_bytes()
    manifest = json.loads(manifest_body)
    frames = manifest['frames']
    require(len(frames) == 180,'retain all selected acquisitions')
    inventory = {}
    for item in manifest['files']:
        name = item['file']
        require(Path(name).name == name and name not in inventory and 0 < item['bytes'] <= 4*1024*1024,
                'bounded unique source inventory')
        inventory[name] = item
    def verified(name):
        path = base/'raw'/name
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 4*1024*1024,'bounded source')
        body = path.read_bytes()
        require(len(body) == inventory[name]['bytes'] and sha(body) == inventory[name]['sha256'],'source digest: '+name)
        return body
    for name in inventory:
        verified(name)  # Encoded reference bytes only; numeric labels are deferred.
    require(all(b['depth_timestamp'] > a['depth_timestamp'] for a,b in zip(frames,frames[1:])),
            'strict acquisition clock')
    sources = [Path(__file__),ROOT/'scripts/projective_depth_common.py',ROOT/'scripts/check-opencv-rgbd.py',
               ROOT/'assets/projective-depth-rust-v1/design.json',
               ROOT/'scripts/projective_depth_raw.py',
               ROOT/'scripts/projective_vertex_raw.py',
               ROOT/'scripts/projective_photo_raw.py',
               ROOT/'scripts/projective_step_raw.py',
               ROOT/'scripts/projective_motion_raw.py',
               ROOT/'scripts/projective_timing_raw.py',
               ROOT/'scripts/projective_information_raw.py',
               ROOT/'scripts/projective_range_raw.py',
               ROOT/'scripts/projective_whitened_raw.py',
               ROOT/'scripts/projective_fusion_raw.py',
               ROOT/'scripts/projective_visibility_raw.py',
               *sorted((ROOT/'integrations/projective-depth/src').rglob('*.rs')),
               ROOT/'integrations/projective-depth/Cargo.toml',ROOT/'integrations/projective-depth/Cargo.lock']
    write(output/'freeze.json',dict(schema_version=1,dataset=case,data_role='already viewed port regression',
        manifest_sha256=sha(manifest_body),binary_sha256=sha(binary.read_bytes()),
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in sources},
        inputs={name:dict(bytes=v['bytes'],sha256=v['sha256']) for name,v in inventory.items()},
        information_guard='physical_spectrum_v1',camera=CAMERA,reprojection=reprojection,correspondence=correspondence,residual=residual,normals=normals,geometry=geometry,photometric=photometric,step_control=step_control,motion_model=motion_model,timing=timing,primary_clock='depth; associated RGB decoded only in keyframe photometric mode',root_translation_gate_m=.1,
        root_rotation_gate_rad=.1,reference_bracket_max_s=.02,pose_resets=0,ground_truth_operational=False,
        selected_frames=180,updates=179,numpy_version=np.__version__,fusion_sampling=fusion_sampling,visibility=visibility))
    client = Client(binary,CAMERA,configuration=dict(reprojection=reprojection,correspondence=correspondence,residual=residual,normals=normals,geometry=geometry,photometric=photometric,step_control=step_control,motion_model=motion_model,timing=timing,fusion_sampling=fusion_sampling,visibility=visibility))
    outcomes = []
    try:
        for frame in frames:
            image = Image.open(io.BytesIO(verified(frame['depth_file'])))
            require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original grayscale depth')
            depth = np.asarray(image,dtype=float)/5000.
            rgb=None
            if photometric=='keyframe':
                color=Image.open(io.BytesIO(verified(frame['rgb_file'])))
                require(color.size==(640,480) and color.mode=='RGB','associated measured RGB')
                color=np.asarray(color,dtype=np.uint32)
                gray=((77*color[:,:,0]+150*color[:,:,1]+29*color[:,:,2])>>8).astype(np.uint8)
                rgb=dict(timestamp=frame['rgb_timestamp'],gray=gray.ravel().tolist())
            update = client.frame(frame['depth_timestamp'],depth,image=rgb)
            require(update['timestamp'] == frame['depth_timestamp'],'acquisition identity')
            outcomes.append(dict(source_index=frame['source_index'],update=update))
        client.finish()
    except BaseException:
        client.abort()
        raise
    write(output/'sensor-only.json',dict(frames=outcomes,ground_truth_parsed=False))

    # All operational work has finished before numeric truth or prototype scores.
    spec = importlib.util.spec_from_file_location('independent_reference',ROOT/'scripts/check-opencv-rgbd.py')
    independent = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(independent)
    independent.controls()
    labels = np.loadtxt(io.BytesIO(verified('groundtruth.txt')),comments='#')
    require(labels.ndim == 2 and labels.shape[1] == 8 and len(labels) <= 30000
            and np.isfinite(labels).all() and np.all(np.diff(labels[:,0]) > 0)
            and np.all(np.linalg.norm(labels[:,4:],axis=1) > .5),'whole chronological reference')
    origin = np.linalg.inv(independent.truth_at(labels,frames[0]['depth_timestamp']))
    prototype_path = ROOT/'assets/projective-depth-diagnostics-v1'/(case+'-fused-v6.json.gz')
    prototype = json.loads(gzip.decompress(prototype_path.read_bytes()))['frames'] if case in CASES else None
    accepted = accurate = checked_steps = 0
    errors, parity = [],[]
    last_accepted = None
    lost = False
    for index,(frame,row) in enumerate(zip(frames,outcomes)):
        update = row['update']
        if last_accepted is not None:
            lost |= frame['depth_timestamp']-last_accepted > .2
        require(update['lost'] == lost and not (lost and update['accepted']),'expiry latch')
        if update['accepted']:
            last_accepted = frame['depth_timestamp']
        require(update['last_accepted_timestamp'] == last_accepted,'rejection cannot renew accepted clock')
        if index == 0:
            require(update['accepted'] and not update['iterations'],'one initialization outside update denominator')
            np.testing.assert_allclose(pose(update['first_from_current']),np.eye(4),atol=0,rtol=0)
            continue
        other = prototype[index-1] if prototype is not None else None
        comparison = dict(source_index=frame['source_index'])
        if other is not None:comparison['same_acceptance']=other['accepted'] == update['accepted']
        if update['accepted']:
            estimate = pose(update['first_from_current'])
            truth = origin@independent.truth_at(labels,frame['depth_timestamp'])
            position = float(np.linalg.norm(estimate[:3,3]-truth[:3,3]))
            rotation = float(np.arccos(np.clip((np.trace(truth[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
            good = position <= .1 and rotation <= .1
            row.update(position_error_m=position,rotation_error_rad=rotation,accurate=good)
            accepted += 1
            accurate += good
            errors.append((position,rotation))
            if other is not None and other['accepted']:
                previous = np.asarray(other['root'])
                comparison.update(translation_disagreement_m=float(np.linalg.norm(estimate[:3,3]-previous[:3,3])),
                    rotation_disagreement_rad=float(np.arccos(np.clip((np.trace(previous[:3,:3].T@estimate[:3,:3])-1)/2,-1,1))))
        else:
            require(update['first_from_current'] is None and update['fusion'] is None,'no rejected-state commit')
        parity.append(comparison)
    from projective_information_raw import audit
    for row in outcomes:audit(row['update'])
    checked_steps = audit_sequence(outcomes,[f['depth_timestamp'] for f in frames])
    summary = dict(total_updates=179,accepted=accepted,root_accurate=accurate,lost=lost,
        max_position_error_m=max((v[0] for v in errors),default=None),
        max_rotation_error_rad=max((v[1] for v in errors),default=None),
        complete_accuracy_pass=accurate == 179 and not lost,independently_checked_normal_steps=checked_steps)
    write(output/'results.json',dict(schema_version=1,dataset=case,summary=summary,frames=outcomes,
        comparison_to_viewed_prototype=parity,prototype_report_sha256=sha(prototype_path.read_bytes()) if prototype is not None else None,
        audit_scope='independent normal solves, increments, state accounting, root scores and full prototype trajectory comparison; not all raw correspondence matrices'))
    print(case,json.dumps(summary),flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',action='append',choices=VIEWED_CASES,help='viewed recording; default retains the original two-recording comparison')
    parser.add_argument('--reprojection',choices=('point','tangent_plane'),default='point')
    parser.add_argument('--correspondence',choices=('forward','reciprocal','bilinear'),default='forward')
    parser.add_argument('--residual',choices=('target_plane','symmetric_planes','symmetric_midpoint','bidirectional_midpoint','depth_variance_plane','depth_whitened_plane'),default='target_plane')
    parser.add_argument('--normals',choices=('central_difference','inverse_depth_plane'),default='central_difference')
    parser.add_argument('--geometry',choices=('depth','vertices'),default='depth')
    parser.add_argument('--photometric',choices=('disabled','keyframe'),default='disabled')
    parser.add_argument('--step-control',choices=('disabled','fixed_support'),default='disabled')
    parser.add_argument('--motion-model',choices=('stationary','constant_velocity'),default='stationary')
    parser.add_argument('--timing',choices=('disabled','previous_motion'),default='disabled')
    parser.add_argument('--fusion-sampling',choices=('nearest','visible_mean'),default='nearest')
    parser.add_argument('--visibility',choices=('disabled','free_space_decay'),default='disabled')
    args = parser.parse_args()
    output = fresh(args.output)
    summaries = {}
    binary_digest = sha(args.binary.read_bytes())
    selected=args.case or CASES
    require(len(set(selected))==len(selected),'unique recording selection')
    for case in selected:
        require(sha(args.binary.read_bytes())==binary_digest,'one unchanged native binary across recordings')
        folder = output/case
        folder.mkdir()
        summaries[case] = evaluate(case,args.binary.absolute(),folder,args.reprojection,args.correspondence,args.residual,args.normals,args.geometry,args.photometric,args.step_control,args.motion_model,args.timing,args.fusion_sampling,args.visibility)
    require(sha(args.binary.read_bytes())==binary_digest,'native binary unchanged through evaluation')
    passed = all(v['complete_accuracy_pass'] for v in summaries.values())
    write(output/'comparison.json',dict(summaries=summaries,complete_accuracy_pass=passed,exit_code=0 if passed else 1))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
