#!/usr/bin/env python3
"""Reconstruct raw depth, all ICP equations and fused models without decoding pose truth."""
import argparse
import copy
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image
from projective_depth_common import CAMERA, ROOT, audit_iterations, audit_sequence, fresh, require, sha, write
from projective_depth_raw import compare_equation, fuse, inverse_depth_normals, normal_equation, pyramid, raster, reconstruct_update

SOURCES = (Path(__file__),ROOT/'scripts/projective_depth_raw.py',ROOT/'scripts/projective_depth_common.py',
           ROOT/'scripts/projective_fusion_raw.py',
           ROOT/'scripts/projective_visibility_raw.py',
           ROOT/'scripts/projective_range_raw.py',
           ROOT/'scripts/projective_whitened_raw.py',
           ROOT/'scripts/projective_vertex_raw.py',
           ROOT/'scripts/projective_photo_raw.py',
           ROOT/'scripts/projective_step_raw.py',
           ROOT/'scripts/projective_motion_raw.py',ROOT/'scripts/projective_timing_raw.py',ROOT/'scripts/projective_information_raw.py',
           ROOT/'scripts/check-projective-depth.py',ROOT/'scripts/requirements-visual.txt',
           ROOT/'assets/projective-depth-raw-audit-v1/design.json')
SOURCE_PINS = {str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in SOURCES}


def read(path, limit=8*1024*1024):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= limit,'bounded regular input')
    return path.read_bytes()


def document(path):
    body = read(path,64*1024*1024 if path.suffix=='.json' else 8*1024*1024)
    if path.suffix == '.gz':
        with gzip.GzipFile(fileobj=io.BytesIO(body)) as stream:
            body = stream.read(64*1024*1024+1)
        require(len(body) <= 64*1024*1024,'bounded expanded report')
    def invalid(value):
        raise ValueError('nonfinite JSON: '+value)
    return json.loads(body,parse_constant=invalid)


def verify_prior(path):
    packet = ROOT/'assets/projective-depth-rust-v1'
    pin = document(packet/'files.json')[str(path.relative_to(packet))]
    body = read(path)
    require(len(body) == pin['bytes'] and sha(body) == pin['sha256'],'immutable prior evidence binding')


def hand_controls():
    c = dict(width=8,height=8,fx=2.,fy=4.,cx=0.,cy=0.)
    layer = pyramid(np.full((8,8),4.),c)
    np.testing.assert_allclose(layer[1]['vertices'][2,1],[2.,2.,4.],atol=0,rtol=0)
    np.testing.assert_allclose(layer[1]['normals'][2,1],[0.,0.,1.],atol=0,rtol=0)
    require(layer[2]['camera']['cx'] == -.25 and layer[2]['camera']['cy'] == -.25,
            'half-pixel pyramid center convention')
    raw = np.ones((8,8))
    raw[0,0] = 0.
    raw[0,2] = 1.2
    raw[0:2,4:6] = [[1.,1.02],[1.,1.02]]
    reduced = pyramid(raw,c)[2]['depth']
    np.testing.assert_allclose(reduced[0,:3],[0.,0.,1.01],atol=1e-15,rtol=0)
    c = dict(width=32,height=32,fx=32.,fy=32.,cx=15.5,cy=15.5)
    raw = np.zeros((32,32));weight = np.zeros((32,32),dtype=np.uint8)
    raw[16,16:18] = 1.
    weight[16,16:18] = [2,7]
    transform = np.eye(4);transform[2,3] = 1.
    output,w,stats = raster(raw,weight,transform,c)
    require(stats == dict(source_valid=2,projected=2,selected=1)
            and output[16,16] == 2. and w[16,16] == 2,'exact index tie witness')
    raw[17,16],weight[17,16] = .5,4
    output,w,stats = raster(raw,weight,transform,c)
    require(stats == dict(source_valid=3,projected=3,selected=1)
            and output[16,16] == 1.5 and w[16,16] == 4,'nearer Z wins independent of index')
    model = np.full((32,32),2.);weight = np.ones((32,32),dtype=np.uint8)
    raw = np.zeros((32,32));raw[0,:4] = [2.01,1.,3.,1.5]
    model[0,3],weight[0,3] = 0.,0
    output,w,stats = fuse(model,weight,raw,np.eye(4),c)
    np.testing.assert_allclose(output[0,:4],[2.005,1.,2.,1.5],atol=1e-15,rtol=0)
    require([stats[k] for k in ('filled','averaged','replaced','ignored')] == [1,1,1,1],
            'four hand-calculated fusion partitions')
    model[:],weight[:],raw[:] = 2.,16,2.01
    output,w,_ = fuse(model,weight,raw,np.eye(4),c)
    np.testing.assert_allclose(output,(16*2.+2.01)/17,atol=1e-15,rtol=0)
    require(np.all(w == 16),'fusion weight cap still updates depth')
    layer = pyramid(np.ones((32,32)),c)[1]
    transform = np.eye(4);transform[2,3] = .01
    e = normal_equation(layer,layer,transform,1)
    require(e['count'] == 225,'hand-calculated planar sample count')
    np.testing.assert_allclose([e['h'][2,2],e['g'][2],e['rms_m']],[225.,2.25,.01],atol=1e-10,rtol=0)
    y,x = np.indices((32,32))
    expected = np.array([.1,-.15,1.]);expected /= np.linalg.norm(expected)
    depth = 2/(expected[0]*(x-c['cx'])/c['fx']+expected[1]*(y-c['cy'])/c['fy']+expected[2])
    fitted = inverse_depth_normals(depth,c)
    np.testing.assert_allclose(fitted[2:-2,2:-2],np.broadcast_to(expected,(28,28,3)),atol=2e-13,rtol=0)
    for bad in (0.,.3,5.,2.1):
        depth = np.full((32,32),2.);depth[16,14] = bad
        fitted = inverse_depth_normals(depth,c)
        np.testing.assert_array_equal(fitted[16,16],np.zeros(3))
        np.testing.assert_allclose(fitted[16,17],[0,0,1],atol=2e-13,rtol=0)
    require(not np.any(fitted[:2]) and not np.any(fitted[-2:]),'complete five-by-five support')
    depth = np.full((32,32),2.)
    depth[16,17],depth[16,15],depth[17,16],depth[15,16] = 2.02,1.98,2.02,1.98
    require(not np.any(inverse_depth_normals(depth,c)[16,16]),'nonplanar residual rejected')
    central,planes = pyramid(depth,c),pyramid(depth,c,'inverse_depth_plane')
    for scale in (2,4):
        np.testing.assert_array_equal(planes[scale]['normals'],central[scale]['normals'])
    np.testing.assert_array_equal(planes[1]['normals'][16,16],central[1]['normals'][16,16])
    return dict(metre_axis_normal_witness=True,half_pixel_centers=True,invalid_discontinuous_reduction=True,
                exact_raster_tie_and_occlusion=True,four_fusion_partitions=True,
                capped_weight_arithmetic=True,hand_calculated_225_point_plane_equation=True,
                inverse_depth_plane_camera_normal=True,missing_discontinuous_plane_support_rejected=True,
                nonplanar_and_coarse_normal_support_preserved=True)


def analytic(output, reports, reprojection='point', correspondence='forward', residual='target_plane', normals='central_difference', geometry='depth', photometric='disabled',fusion_sampling='nearest',visibility='disabled'):
    spec = importlib.util.spec_from_file_location('independent_scene',ROOT/'scripts/check-projective-depth.py')
    scene = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scene)
    first = scene.render(np.eye(4))
    summaries = []
    witnesses = []
    for index in range(1,5):
        path = reports/('motion-'+str(index)+'.json') if reports else \
            ROOT/'assets/projective-depth-rust-v1'/('motion-'+str(index)+'.json.gz')
        if reports is None:
            verify_prior(path)
        record = document(path)
        raw = scene.render(np.asarray(record['expected']))
        update = record['result']
        audit_iterations(update)
        context=None
        if photometric=='keyframe':
            context=dict(target=scene.render_gray(np.eye(4),first),source=scene.render_gray(np.asarray(record['expected']),raw),
                         source_timestamp=1.03+.001,reference_timestamp=1.+.001,qualified=True,fresh=True)
        _,_,checked = reconstruct_update(first,(first > .3).astype(np.uint8),raw,scene.CAMERA,update,reprojection,correspondence,residual,normals,geometry,context,fusion_sampling,visibility)
        summaries.append(dict(motion=index,report_sha256=sha(read(path)),**checked))
        witnesses.append((first,raw,scene.CAMERA,update,context))
    model,raw,c,original,context = witnesses[2]
    rejected = []
    def corruption(name, change):
        altered = copy.deepcopy(original)
        change(altered)
        try:
            reconstruct_update(model,(model > .3).astype(np.uint8),raw,c,altered,reprojection,correspondence,residual,normals,geometry,context,fusion_sampling,visibility)
        except (AssertionError,ValueError,KeyError,TypeError,IndexError):
            rejected.append(name)
        else:
            raise ValueError('undetected raw audit corruption: '+name)
    def rescale_normal(u):
        e = u['iterations'][0]['equation']
        e['h'] = (2*np.asarray(e['h'])).tolist()
        e['g'] = (2*np.asarray(e['g'])).tolist()
        if 'photometric' in u['iterations'][0]:
            p=u['iterations'][0]['photometric'];p['h']=(2*np.asarray(p['h'])).tolist();p['g']=(2*np.asarray(p['g'])).tolist()
    rescaled = copy.deepcopy(original)
    rescale_normal(rescaled)
    audit_iterations(rescaled)  # A valid solve alone cannot detect joint H/g scaling.
    corruption('joint H/g rescaling with unchanged valid solve',rescale_normal)
    corruption('false correspondence count',lambda u:u['iterations'][0]['equation'].update(
        count=u['iterations'][0]['equation']['count']+1))
    corruption('false per-iteration RMS',lambda u:u['iterations'][0]['equation'].update(
        rms_m=u['iterations'][0]['equation']['rms_m']+.001))
    corruption('false final support',lambda u:u['final_equation'].update(count=u['final_equation']['count']+1))
    corruption('false returned-pose RMS',lambda u:u['final_equation'].update(rms_m=.02))
    if residual=='bidirectional_midpoint':
        def wrong_directions(u):
            e=u['iterations'][0]['equation'];a,b=e['direction_counts'];e['direction_counts']=[a-1,b+1]
        corruption('changed direction partition with unchanged total and solve',wrong_directions)
        corruption('missing independent direction counts',lambda u:u['final_equation'].pop('direction_counts'))
        try:
            reconstruct_update(model,(model>.3).astype(np.uint8),raw,c,original,
                               residual_policy='symmetric_midpoint')
        except (AssertionError,ValueError,KeyError,TypeError,IndexError):
            rejected.append('one-way correspondence objective substituted for bidirectional matching')
        else:raise ValueError('one-way objective substitution undetected')
    def move_fusion(u):
        require(u['fusion']['averaged'] > 0,'nonempty analytic fusion witness')
        u['fusion']['averaged'] -= 1
        u['fusion']['ignored'] += 1
    corruption('fusion category moved while total remains valid',move_fusion)
    if photometric=='keyframe':
        corruption('altered image gradient',lambda u:u['iterations'][0]['photometric']['g'].__setitem__(0,u['iterations'][0]['photometric']['g'][0]+100.))
        corruption('repeated image falsely marked fresh',lambda u:u['photometric'].update(fresh=False))
        if 'illumination' in original['iterations'][0]['photometric']:
            corruption('altered exposure gain',lambda u:u['iterations'][0]['photometric']['illumination'].update(
                gain=u['iterations'][0]['photometric']['illumination']['gain']+.01))
            corruption('altered exposure fitting support',lambda u:u['iterations'][0]['photometric']['illumination'].update(
                candidate_count=u['iterations'][0]['photometric']['illumination']['candidate_count']+1))
        if 'step_control' in original['iterations'][0]:
            corruption('altered fixed-support initial cost',lambda u:u['iterations'][0]['step_control'].update(
                before_cost_m2=u['iterations'][0]['step_control']['before_cost_m2']+.001))
            corruption('altered fixed-support trial cost',lambda u:u['iterations'][0]['step_control']['trial_costs_m2'].__setitem__(0,.1))
            corruption('altered fixed-support geometry count',lambda u:u['iterations'][0]['step_control'].update(
                geometric_count=u['iterations'][0]['step_control']['geometric_count']+1))
    vertex_controls=None
    fusion_controls=None
    visibility_controls=None
    if visibility=='free_space_decay':
        from projective_visibility_raw import controls
        visibility_controls=controls()
        corruption('false free-space visibility count',lambda u:u['fusion']['visibility'].update(
            free_space=u['fusion']['visibility']['free_space']+1))
        corruption('false visibility removal count',lambda u:u['fusion']['visibility'].update(
            removed=u['fusion']['visibility']['removed']+1))
        corruption('missing visibility evidence receipt',lambda u:u['fusion'].pop('visibility'))
    if fusion_sampling=='visible_mean':
        from projective_fusion_raw import controls
        fusion_controls=controls()
        try:
            reconstruct_update(model,(model>.3).astype(np.uint8),raw,c,original)
        except (AssertionError,ValueError,KeyError,TypeError,IndexError):
            rejected.append('nearest-Z sampling substituted for visible mean')
        else:
            raise ValueError('fusion sampling substitution undetected')
        corruption('altered visible surface merge count',lambda u:u['fusion']['measurement'].update(
            surface_merged=u['fusion']['measurement'].get('surface_merged',0)+1))
    if geometry=='vertices':
        from projective_vertex_raw import controls
        vertex_controls=controls()
        try:
            reconstruct_update(model,(model>.3).astype(np.uint8),raw,c,original)
        except (AssertionError,ValueError,KeyError,TypeError,IndexError):
            rejected.append('legacy depth geometry substituted for vertex geometry')
        else:
            raise ValueError('vertex geometry substitution undetected')
    write(output/'analytic-audit.json',dict(hand_controls=hand_controls(),motions=summaries,geometry=geometry,vertex_controls=vertex_controls,
        rejected_corruptions=rejected,prior_solve_only_auditor_accepts_rescaled_Hg=True,
        fusion_sampling=fusion_sampling,fusion_controls=fusion_controls,visibility=visibility,visibility_controls=visibility_controls))


def recorded(case, output, reports=None):
    source = ROOT/'data'/case
    prior = reports/case if reports else ROOT/'assets/projective-depth-rust-v1'/case
    freeze_path = prior/'freeze.json' if reports else prior/'final-freeze.json'
    if reports is None:verify_prior(freeze_path)
    freeze = document(freeze_path)
    reprojection = freeze.get('reprojection','point')
    correspondence = freeze.get('correspondence','forward')
    residual = freeze.get('residual','target_plane')
    normals = freeze.get('normals','central_difference')
    geometry = freeze.get('geometry','depth')
    fusion_sampling = freeze.get('fusion_sampling','nearest')
    require(fusion_sampling in ('nearest','visible_mean'),'declared fusion sampling policy')
    visibility = freeze.get('visibility','disabled')
    require(visibility in ('disabled','free_space_decay'),'declared visibility policy')
    photometric = freeze.get('photometric','disabled')
    step_policy=freeze.get('step_control')
    motion_model=freeze.get('motion_model','stationary')
    information_guard=freeze.get('information_guard')
    require(information_guard in (None,'physical_spectrum_v1'),'declared physical information guard')
    timing=freeze.get('timing','disabled')
    require(timing in ('disabled','previous_motion'),'declared timing policy')
    if timing=='previous_motion':require(photometric=='keyframe' and motion_model=='constant_velocity' and step_policy=='disabled','qualified temporal policies')
    require(motion_model in ('stationary','constant_velocity'),'declared motion model')
    require(step_policy in (None,'disabled','fixed_support'),'declared step policy')
    require(photometric in ('disabled','keyframe'),'declared image policy')
    require(geometry in ('depth','vertices'),'declared geometry policy')
    require(normals in ('central_difference','inverse_depth_plane'),'declared normal policy')
    require(residual in ('target_plane','symmetric_planes','symmetric_midpoint','bidirectional_midpoint','depth_variance_plane','depth_whitened_plane'),'declared residual policy')
    require(correspondence in ('forward','reciprocal','bilinear'),'declared correspondence policy')
    require(reprojection in ('point','tangent_plane'),'declared reprojection policy')
    if visibility=='free_space_decay':require((geometry,reprojection,correspondence,residual,normals,fusion_sampling,photometric,motion_model,timing)==('depth','point','forward','target_plane','central_difference','nearest','disabled','stationary','disabled') and step_policy in (None,'disabled'),'qualified complete visibility policies')
    ledger_path = prior/'sensor-only.json' if reports else prior/'sensor-only.json.gz'
    if reports is None:verify_prior(ledger_path)
    ledger = document(ledger_path)
    require(ledger['ground_truth_parsed'] is False,'original pre-label operational ledger')
    manifest_body = read(source/'manifest.json')
    require(sha(manifest_body) == freeze['manifest_sha256'],'frozen manifest binding')
    manifest = json.loads(manifest_body)
    frames = manifest['frames']
    outcomes = ledger['frames']
    require(len(frames) == len(outcomes) == 180 and freeze['camera'] == CAMERA,'complete fixed selection and camera')
    require([f['source_index'] for f in frames] == list(range(100,280)),'unchanged acquisition indices')
    def verified(frame):
        name = frame['depth_file']
        require(Path(name).name == name,'plain source filename')
        body = read(source/'raw'/name,4*1024*1024)
        pin = freeze['inputs'][name]
        require(len(body) == pin['bytes'] and sha(body) == pin['sha256'],'raw depth source binding')
        return body
    for frame in frames:
        verified(frame)  # Hashes only; no pixel or reference label decoding yet.
    require(all(sha(read(ROOT/n))==d for n,d in SOURCE_PINS.items()),'audit sources unchanged since process start')
    write(output/'freeze.json',dict(dataset=case,source_sha256=SOURCE_PINS,
        manifest_sha256=sha(manifest_body),original_freeze_sha256=sha(read(freeze_path)),reprojection=reprojection,correspondence=correspondence,residual=residual,normals=normals,geometry=geometry,photometric=photometric,step_control=step_policy,motion_model=motion_model,timing=timing,information_guard=information_guard,
        original_sensor_ledger_sha256=sha(read(ledger_path,64*1024*1024)),depth_source_count=180,
        ground_truth_decoded=False,raw_color_decoded=photometric=='keyframe'))
    if information_guard:
        from projective_information_raw import audit
        for row in outcomes:audit(row['update'])
    steps = audit_sequence(outcomes,[f['depth_timestamp'] for f in frames])
    model = weights = None
    key_gray=key_rgb=last_rgb=None;key_phase=0.
    from projective_timing_raw import TimingHistory
    timing_history=TimingHistory()
    checked_frames = []
    for frame,row in zip(frames,outcomes):
        try:
            require(row['source_index'] == frame['source_index'],'original frame identity')
            image = Image.open(io.BytesIO(verified(frame)))
            require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original 16-bit depth layout')
            raw = np.asarray(image,dtype=float)/5000.
            context=None;fresh_rgb=False;current_gray=None
            if photometric=='keyframe' and not row['update']['lost']:
                name=frame['rgb_file'];require(Path(name).name==name,'plain RGB source')
                body=read(source/'raw'/name,4*1024*1024);pin=freeze['inputs'][name]
                require(len(body)==pin['bytes'] and sha(body)==pin['sha256'],'raw RGB source binding')
                color=Image.open(io.BytesIO(body));require(color.size==(640,480) and color.mode=='RGB','measured RGB layout')
                color=np.asarray(color,dtype=np.uint32);current_gray=((77*color[:,:,0]+150*color[:,:,1]+29*color[:,:,2])>>8).astype(np.uint8)
                rgb=frame['rgb_timestamp'];require(last_rgb is None or rgb>=last_rgb,'monotonic associated RGB')
                fresh_rgb=last_rgb is None or rgb>last_rgb;last_rgb=rgb
                phase=rgb-frame['depth_timestamp'];require(abs(phase)<=.02,'RGB association')
                context=dict(target=key_gray,source=current_gray,source_timestamp=rgb,reference_timestamp=key_rgb,
                             qualified=key_gray is not None and abs(phase-key_phase)<=.005,fresh=fresh_rgb)
                if timing=='previous_motion':timing_history.augment(context,frame['depth_timestamp'])
            initial=model is None
            if row['update']['iterations']:
                require(('prediction' in row['update'])==(motion_model=='constant_velocity'),'configured prediction provenance')
            if step_policy is not None:
                require(all(('step_control' in s)==(step_policy=='fixed_support') for s in row['update']['iterations']),'complete configured step reports')
            model,weights,checked = reconstruct_update(model,weights,raw,CAMERA,row['update'],reprojection,correspondence,residual,normals,geometry,context,fusion_sampling,visibility)
            if photometric=='keyframe':
                checked['photometric_equations']=sum('photometric' in s for s in row['update']['iterations'])+int('final_photometric' in row['update'])
                checked['used_photometric_solves']=sum(s.get('photometric',{}).get('used',False) for s in row['update']['iterations'])
                controls=[s['step_control'] for s in row['update']['iterations'] if 'step_control' in s]
                checked.update(step_controls=len(controls),step_trials=sum(len(c['trial_costs_m2']) for c in controls),
                    reduced_steps=sum(0<c['applied_scale']<1 for c in controls),halted_levels=sum(c['applied_scale']==0 for c in controls))
            if photometric=='keyframe' and row['update']['accepted'] and (initial or row['update']['keyframe_replaced']):
                key_gray=current_gray if fresh_rgb else None;key_rgb=rgb if fresh_rgb else None;key_phase=phase if fresh_rgb else 0.
            timing_history.observe(row['update'])
            checked_frames.append(dict(source_index=frame['source_index'],**checked,
                supported_model_pixels=int(np.sum(weights > 0)),maximum_model_weight=int(weights.max())))
        except (ValueError,AssertionError,KeyError,TypeError,IndexError) as error:
            write(output/'first-failure.json',dict(source_index=frame['source_index'],
                completed_frames=len(checked_frames),error_type=type(error).__name__,error=str(error),
                physical_accuracy_gates_changed=False))
            raise
        if len(checked_frames)%30 == 0:
            print(case,'raw-audited frames',len(checked_frames),flush=True)
    require(sum(v['iterations'] for v in checked_frames) == steps,'every reported solve raw-reconstructed')
    require(all(sha(read(ROOT/n))==d for n,d in SOURCE_PINS.items()),'audit sources unchanged through reconstruction')
    summary = dict(frames=180,updates=179,raw_iteration_equations=steps,
        returned_pose_equations=sum(v['final_equations'] for v in checked_frames),
        reference_view_warps=sum(v['model_replaced'] for v in checked_frames),
        fused_model_generations=179,exact_discrete_counts=True,physical_information_checked=information_guard is not None,ground_truth_decoded=False,
        physical_accuracy_claim=False)
    if photometric=='keyframe':
        summary.update(raw_photometric_equations=sum(v['photometric_equations'] for v in checked_frames),
                       used_photometric_solves=sum(v['used_photometric_solves'] for v in checked_frames),raw_color_decoded=True)
        for key in ('step_controls','step_trials','reduced_steps','halted_levels'):
            summary[key]=sum(v[key] for v in checked_frames)
    write(output/'audit.json',dict(pass_raw_mathematics=True,summary=summary,frames=checked_frames))
    print(case,json.dumps(summary),flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--analytic-only',action='store_true')
    parser.add_argument('--analytic-reports',type=Path,help='fresh reports from check-projective-depth.py; defaults to frozen originals')
    parser.add_argument('--recorded-reports',type=Path,help='fresh evaluator reports; defaults to frozen originals')
    parser.add_argument('--case',action='append',choices=('tum-fr1-room-temporal','tum-fr1-desk2-independent','tum-fr1-rpy-heldout'),help='viewed recording; default retains the original two-recording audit')
    parser.add_argument('--reprojection',choices=('point','tangent_plane'),default='point',help='policy for fresh analytic reports')
    parser.add_argument('--correspondence',choices=('forward','reciprocal','bilinear'),default='forward')
    parser.add_argument('--residual',choices=('target_plane','symmetric_planes','symmetric_midpoint','bidirectional_midpoint','depth_variance_plane','depth_whitened_plane'),default='target_plane')
    parser.add_argument('--normals',choices=('central_difference','inverse_depth_plane'),default='central_difference')
    parser.add_argument('--geometry',choices=('depth','vertices'),default='depth')
    parser.add_argument('--photometric',choices=('disabled','keyframe'),default='disabled')
    parser.add_argument('--fusion-sampling',choices=('nearest','visible_mean'),default='nearest')
    parser.add_argument('--visibility',choices=('disabled','free_space_decay'),default='disabled')
    args = parser.parse_args()
    output = fresh(args.output)
    require(args.analytic_reports is not None or (args.reprojection=='point' and args.correspondence=='forward' and args.residual=='target_plane' and args.normals=='central_difference' and args.geometry=='depth' and args.photometric=='disabled' and args.fusion_sampling=='nearest' and args.visibility=='disabled'),'frozen analytic reports use original policies')
    analytic(output,args.analytic_reports,args.reprojection,args.correspondence,args.residual,args.normals,args.geometry,args.photometric,args.fusion_sampling,args.visibility)
    cases = {}
    if not args.analytic_only:
        selected=args.case or ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
        require(len(set(selected))==len(selected),'unique recorded audit selection')
        require('tum-fr1-rpy-heldout' not in selected or args.recorded_reports is not None,'RPY requires explicit viewed reports')
        for case in selected:
            folder = output/case
            folder.mkdir()
            cases[case] = recorded(case,folder,args.recorded_reports)
    write(output/'summary.json',dict(pass_raw_mathematics=True,cases=cases,
        ground_truth_decoded=False,physical_accuracy_improvement_claim=False,subjective_maturity_percent=20))


if __name__ == '__main__':
    main()
