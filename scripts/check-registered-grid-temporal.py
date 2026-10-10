#!/usr/bin/env python3
"""Independent source-bound registered-grid camera and continuous RGB-D audit.

The source manifest and operational pixel model are separate immutable inputs.
The earlier temporal/pair mathematics are byte verified and executed unchanged
in an isolated dependency namespace; no original module globals are overridden.
"""
import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import types

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
TEMPORAL_SHA = '231d6c5fd9a56d9925d7a52ad5a1affb36170275914fb275f199fcf385040e4e'
PAIR_SHA = '78e9466da3c30605b1f25a8037a6c51fcfb2198c8caacbacde85a6eb9ccb3737'
MAX_BYTES = 64*1024*1024
REGISTERED_CAMERA = dict(width=640,height=480,fx=525.,fy=525.,cx=319.5,cy=239.5,
                         units_per_metre=5000.,invalid_depth=0)
CAMERA_MODEL_SHA = '511cff92d365983efe40fdb90c9336ac1c122ebef6269594a07081b4eda8ecf3'
DESIGN_SHA = 'cfd56dd22e2aadd1f3fdaba8b3162a58d19a783d62438d4d5694bbca3ab41ac3'
PRIOR_FREEZE_SHA = '1f419ab2ee40bc619322c1ed27c301f07dc3b95486bd85b41563bf07c673ac93'
MODEL_PATH = ROOT/'assets/registered-grid-temporal-v1/camera-model.json'
DESIGN_PATH = ROOT/'assets/registered-grid-temporal-v1/design.json'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def bounded(path, limit=MAX_BYTES):
    path = Path(path)
    require(path.is_file() and path.stat().st_size <= limit, 'missing/oversized input: '+str(path))
    with path.open('rb') as source:
        raw = source.read(limit+1)
    require(len(raw) <= limit, 'input grew beyond byte bound')
    return raw


def require_new_output(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(),
            'output already exists or is a symlink: '+str(path))
    require(not any(parent.is_symlink() for parent in path.parents),
            'output parent is a symlink: '+str(path))


def write_new(path, result):
    require_new_output(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    require_new_output(path)
    with path.open('x', encoding='utf8') as destination:
        destination.write(json.dumps(result, indent=2, allow_nan=False)+'\n')


def load_math():
    pair_path = ROOT/'scripts/check-multiscale-features.py'
    require(digest(bounded(pair_path)) == PAIR_SHA, 'original pair oracle changed')
    spec = importlib.util.spec_from_file_location('immutable_registered_grid_pair_math', pair_path)
    pair = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pair)
    temporal_path = ROOT/'scripts/check-multiscale-temporal.py'
    raw = bounded(temporal_path)
    require(digest(raw) == TEMPORAL_SHA, 'original temporal oracle changed')
    names = {'pose_json','independent_fit','block_reason','initial_state','reject_mutant',
        'exact','verified_raw_loader','validate_acquisition','initialization_geometry',
        'check_state','empty_measurement','score_root','check_initialization',
        'check_frontend_subset','checked_sensor','check_sequence_sensor','physical_audit',
        'temporal_mutations','self_tests','audit_control','control_mutations',
        'validate_design','source_hashes'}
    tree = ast.parse(raw, filename=str(temporal_path))
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    require({node.name for node in functions} == names, 'immutable helper definition inventory')
    # Execute byte-exact definitions in OUR namespace, supplying dependencies
    # up front. The original temporal module is never imported or modified.
    namespace = dict(M=pair,copy=copy,math=math,json=json,np=np,Path=Path,
        require=require,digest=digest,bounded=bounded,FIT_CACHE={},SENSOR_AUDIT_CACHE={},
        ROOT=ROOT,DESIGN_PATH=ROOT/'assets/multiscale-temporal-v1/design.json',
        DESIGN_SHA='d0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df',
        BASE_FREEZE_SHA='c9b9cc2be0d8168dfc1e9e3aa81eeb1b5b4c45a0c160d0139774c2ade8ee0a5c',
        IDENTITY=([0.,0.,0.],[1.,0.,0.,0.]),
        LOST='visual odometry lost; explicit new origin required',
        EXPIRED='visual accepted-pose age exceeded; localization lost',
        DUPLICATE='duplicate or stale RGB acquisition; no pose permission renewal',
        GAP='sensor association gap exceeds .02s',
        PAIR_FIELDS={'accepted','matches','correspondences','refinement_observations',
                     'coarse_fit','coarse_pose','refinement','relative_estimate','rejection'})
    definitions = '\n\n'.join(ast.get_source_segment(raw.decode('utf8'), node) for node in functions)
    table=next(node for node in tree.body if isinstance(node,ast.Assign) and
        any(isinstance(target,ast.Name) and target.id=='CONTROL_SPECS' for target in node.targets))
    exec(compile(ast.get_source_segment(raw.decode('utf8'),table),str(temporal_path),'exec'),namespace)
    exec(compile(definitions,str(temporal_path),'exec'),namespace)
    return pair,types.SimpleNamespace(**namespace)


def effective_manifest(source_manifest, effective_camera):
    # A new mathematical view, never a rewrite of the source manifest or its
    # provenance. Pixel/depth inputs and numeric reference poses remain unchanged.
    result = copy.deepcopy(source_manifest)
    result['depth_calibration'] = copy.deepcopy(effective_camera)
    return result


def audit_geometry_and_labels(report, source_manifest, raw_root, effective_camera, math_helpers):
    inventory, raw = math_helpers.verified_raw_loader(source_manifest,raw_root)
    math_helpers.exact(report['inventory_integrity'],dict(all_files_verified=True,
        files_verified=len(inventory),bytes_verified=sum(pin['bytes'] for pin in inventory.values()),
        no_pixels_decoded=True,no_numeric_labels_parsed=True),'opaque full-source inventory')
    view = effective_manifest(source_manifest,effective_camera)
    poses,states,evidence = math_helpers.check_sequence_sensor(report,view,inventory,raw)
    # Both native/multiscale complete measured histories are audited above.
    labels=[f['file'] for f in source_manifest['files'] if f['role']=='evaluation_only_mocap_ground_truth']
    require(len(labels)==1,'physical label source count')
    gt,error=None,None
    try:
        gt=math_helpers.M.normalized_truth(raw(labels[0]))
    except ValueError as failure:
        error=str(failure)
    math_helpers.exact(report['evaluation_label_failure'],error,'physical label failure')
    counts,maxima=math_helpers.physical_audit(report,view,poses,states,gt,error)
    return dict(kind='registered_grid_viewed_continuous_regression',summary=counts,
        maximum_accepted_root_errors=maxima,frames=evidence,
        numeric_truth_checked_after_all_sensor_fits=True,one_initial_origin=True,
        no_reset_or_loss_recovery=True,all_acquisitions_retained=True,
        source_manifest_preserved=True,effective_camera=effective_camera)


def analytic_math_controls(pair, helpers):
    """Original mathematical controls plus explicit registered axial-Z rays."""
    prior=helpers.self_tests()
    depth=np.full((480,640),10000,dtype=np.uint16)
    feature=dict(x=500.5,y=350.5)
    point,error=pair.depth_point(feature,depth,REGISTERED_CAMERA)
    require(error is None,'registered-grid analytic ray rejected')
    expected=[(500.5-319.5)*2./525.,(350.5-239.5)*2./525.,2.]
    pair.compare_value(point,expected,'registered-grid analytic axial-Z point')
    require(abs(pair.V.r.norm(point)-2.)>.05,'axial-Z control did not distinguish ray range')
    # Changing the effective K changes physical backprojection, with identical
    # image/descriptor/depth bytes. This control asserts an actual numerical
    # difference and does not claim which model explains recorded drift.
    legacy=dict(REGISTERED_CAMERA,fx=517.306408,fy=516.469215,cx=318.643040,cy=255.313989)
    legacy_point,error=pair.depth_point(feature,depth,legacy)
    require(error is None and pair.V.r.norm([x-y for x,y in zip(point,legacy_point)])>.02,
            'cross-model control was a no-op')
    baseline=dict(point=point,units_per_metre=5000.,additional_depth_scale=1.,
                  additional_undistortion=False,pixel_domain='original registered VGA grid')
    def validate(data):
        pair.compare_value(data['point'],expected,'corrupted registered axial-Z ray')
        helpers.exact({key:data[key] for key in baseline if key!='point'},
            {key:baseline[key] for key in baseline if key!='point'},'registered model policy')
    def legacy_ray(data):
        data['point']=legacy_point
    def double_scale(data):
        data['point']=[x*1.035 for x in point]
    def range_as_z(data):
        length=pair.V.r.norm(point)
        data['point']=[x*2./length for x in point]
    def round_ray(data):
        data['point']=[(501.-319.5)*2./525.,(351.-239.5)*2./525.,2.]
    operations={
        'legacy_intrinsics_as_registered':legacy_ray,'double_applied_1_035_scale':double_scale,
        'range_instead_of_axial_z':range_as_z,'rounded_fractional_feature_ray':round_ray,
        'extra_undistortion':lambda data:data.__setitem__('additional_undistortion',True),
        'changed_pixel_domain':lambda data:data.__setitem__('pixel_domain','undistorted RGB grid'),
        'changed_depth_units':lambda data:data.__setitem__('units_per_metre',1000.)}
    validate(baseline)
    for name,mutate in operations.items():
        helpers.reject_mutant(baseline,mutate,validate)
    return dict(kind='registered_grid_analytic_math',prior_state_controls=prior,
        registered_point_m=point,legacy_point_m=legacy_point,
        registered_axial_z_m=2.,registered_ray_range_m=pair.V.r.norm(point),
        non_noop_model_corruptions_rejected=list(operations),recorded_data_read=False,
        drift_causality_claim=False)


def fixed_documents(helpers):
    model_raw=bounded(MODEL_PATH,512*1024)
    design_raw=bounded(DESIGN_PATH,512*1024)
    require(digest(model_raw)==CAMERA_MODEL_SHA and digest(design_raw)==DESIGN_SHA,
            'fixed registered-grid model/design changed')
    model,design=json.loads(model_raw),json.loads(design_raw)
    expected=dict(REGISTERED_CAMERA,depth_quantity='axial_z',depth_scale_multiplier=1.,
        additional_undistortion=False,pixel_domain='original_registered_rgb_depth_grid',
        lens_model='pinhole_on_registered_grid')
    helpers.exact(model['operational'],expected,'closed registered-grid operational model')
    require(model['profile_id']=='tum-fr1-registered-ros-default-v1' and
        model['source_rgb_provenance']['role']=='documentation_only_not_operational_pixel_model',
        'registered model/source roles')
    require(design['registered_grid_camera_model_sha256']==CAMERA_MODEL_SHA and
            design['no_retuning_after_first_outcome'] is True,'model design binding')
    return model,design


def compiled_sources(helpers):
    prior_raw=bounded(ROOT/'assets/multiscale-temporal-v1/room-freeze.json',512*1024)
    require(digest(prior_raw)==PRIOR_FREEZE_SHA,'prior temporal freeze changed')
    prior=helpers.source_hashes()
    helpers.exact(prior,json.loads(prior_raw)['sources'],'prior18 original compiled sources')
    paths=dict(registered_binary='integrations/rgbd/src/bin/rustdriving-rgbd-registered-grid-temporal.rs',
        registered_support='integrations/rgbd/src/registered_grid_temporal_support.rs',
        registered_grid='integrations/rgbd/src/registered_grid.rs',
        registered_design='assets/registered-grid-temporal-v1/design.json',
        camera_model='assets/registered-grid-temporal-v1/camera-model.json')
    current=dict(prior,**{key:digest(bounded(ROOT/path)) for key,path in paths.items()})
    require(len(prior)==18 and len(current)==23,'compiled source binding counts')
    return current,prior


def source_calibration(source_manifest,model,helpers):
    camera=source_manifest['depth_calibration']
    result={key:camera[key] for key in REGISTERED_CAMERA}
    legacy=dict(REGISTERED_CAMERA,fx=517.306408,fy=516.469215,cx=318.64304,cy=255.313989)
    helpers.exact(result,legacy,'original source RGB calibration')
    source=source_manifest['calibration_source'];provenance=model['source_rgb_provenance']
    for key in ('repository','revision','source_path','bytes','sha256'):
        helpers.exact(source[key],provenance[key],'source RGB calibration provenance '+key)
    for key in ('fx','fy','cx','cy'):
        helpers.exact(result[key],provenance[key],'source RGB intrinsics documentation '+key)
    if 'source_rgb_distortion' in camera:
        helpers.exact(camera['source_rgb_distortion'],provenance['distortion'],'source D documentation')
    return result


def protocol_check(report,manifest_bytes,freeze,helpers):
    model,design=fixed_documents(helpers)
    manifest=json.loads(manifest_bytes);name=manifest['dataset']
    require(name in design['datasets'] and digest(manifest_bytes)==design['datasets'][name]['manifest_sha256'],
            'original fixed source manifest changed')
    source=source_calibration(manifest,model,helpers)
    current,prior=compiled_sources(helpers)
    expected=dict(schema_version=1,algorithm='registered_grid_multiscale_continuous_viewed_comparison',
        dataset=name,manifest_sha256=digest(manifest_bytes),design_sha256=DESIGN_SHA,design=design,
        sources=current,prior_sources=prior,camera_model_sha256=CAMERA_MODEL_SHA,
        camera_model=model,effective_calibration=model['operational'],source_calibration=source,
        source_reuse='Measurement/frontend/state/scoring functions copied byte-for-byte from immutable multiscale temporal prototype. Only explicit effective registered-grid calibration derivation, provenance and new controls differ; prior18 source bindings retained separately.',
        indices='ordinal0..179; original source_index100..279 separately retained',no_ground_truth_operational=True)
    helpers.exact(freeze,expected,'registered-grid external source/profile freeze')
    helpers.exact(report['freeze'],freeze,'report external freeze')
    for key in ('schema_version','algorithm','dataset','manifest_sha256','camera_model_sha256',
                'camera_model','effective_calibration','source_calibration'):
        helpers.exact(report[key],expected[key],'registered report '+key)
    helpers.exact(report['ground_truth_operational'],False,'operational GT prohibited')
    helpers.exact(report['raw_redistributed'],False,'raw redistribution prohibited')
    require(report['input_failure'] is None,'source-invalid sensor report requires separate input audit')
    require(len(manifest['frames'])==180 and
        [f['source_index'] for f in manifest['frames']]==list(range(100,280)),'fixed180frame original window')
    return manifest,model


def audit_control(report,pair,helpers):
    model,_=fixed_documents(helpers)
    current,prior=compiled_sources(helpers)
    require(set(report)=={'schema_version','kind','sources','prior_sources','design_sha256',
        'camera_model_sha256','calibration','texture','cases','registered_grid_control'},'model control schema')
    require(report['schema_version']==1 and report['kind']=='analytic_continuous_measured_controls'
        and report['design_sha256']==DESIGN_SHA and report['camera_model_sha256']==CAMERA_MODEL_SHA,
        'model control source identity')
    helpers.exact(report['sources'],current,'control23 sources')
    helpers.exact(report['prior_sources'],prior,'control original18 sources')
    # Explicit metadata view for the unchanged synthetic camera fixtures. No
    # algorithm globals, source files or operational registered K are modified.
    legacy={key:report[key] for key in ('schema_version','kind','calibration','texture','cases')}
    legacy.update(sources=prior,design_sha256='d0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df')
    legacy_proof=helpers.audit_control(legacy)
    registered=report['registered_grid_control']
    require(set(registered)=={'camera_model','effective_calibration','rays','case'},'registered control fields')
    helpers.exact(registered['camera_model'],model,'registered control camera model')
    helpers.exact(registered['effective_calibration'],model['operational'],'registered control effective camera')
    ray_specs=[([319.5,239.5],5000),([0.,0.],10000),([638.5,478.5],7500),([187.25,201.75],23456)]
    require(len(registered['rays'])==len(ray_specs),'omitted registered ray')
    points=[]
    for witness,(pixel,raw_depth) in zip(registered['rays'],ray_specs):
        require(set(witness)=={'pixel','raw_depth','point','reprojected_pixel'},'registered ray schema')
        helpers.exact(witness['pixel'],pixel,'registered ray pixel')
        helpers.exact(witness['raw_depth'],raw_depth,'registered axial raw depth')
        z=raw_depth/5000.
        point=[(pixel[0]-319.5)*z/525.,(pixel[1]-239.5)*z/525.,z]
        pair.compare_value(witness['point'],point,'registered nominal ray')
        reprojected=[525.*point[0]/point[2]+319.5,525.*point[1]/point[2]+239.5]
        pair.compare_value(witness['reprojected_pixel'],reprojected,'registered nominal projection')
        points.append(point)
    case=registered['case'];specs=helpers.CONTROL_SPECS['healthy']
    require(case['kind']=='healthy' and len(case['frames'])==4,'registered measured control kind/length')
    manifest=dict(depth_calibration=model['operational'],frames=[])
    rendered=[]
    base=pair.texture(640,480)
    for i,(stamp,rgb_stamp,shift,raw_depth,blank) in enumerate(specs):
        manifest['frames'].append(dict(source_index=i,depth_timestamp=stamp,rgb_timestamp=rgb_stamp))
        helpers.exact(case['frames'][i]['render'],dict(shift_pixels=shift,raw_depth=raw_depth,blank=blank),
            'registered measured control render')
        dx,dy=shift;gray=np.zeros_like(base)
        gray[dy:,dx:]=base[:480-dy,:640-dx]
        rendered.append((gray,np.full((480,640),raw_depth,dtype=np.uint16)))
    poses,states,evidence=helpers.check_sequence_sensor(case,manifest,{},None,rendered)
    # Analytic camera roots enter scoring after all measured native/multi fits.
    truths=[([-shift[0]*1.5/525.,-shift[1]*1.5/525.,0.],[1.,0.,0.,0.])
            for _,_,shift,_,_ in specs]
    counts,maxima=helpers.physical_audit(case,manifest,poses,states,analytic_roots=truths)
    helpers.exact(case['expected_accepted'],[True]*4,'registered healthy availability')
    require(case['expected_behavior_passed'] is True and
            all(v['all_updates_passed'] for v in counts.values()),'registered measured healthy control failed')
    return dict(kind='registered_grid_analytic_sensor_controls',legacy_state_controls=legacy_proof,
        registered_control=dict(summary=counts,maximum_accepted_root_errors=maxima,frames=evidence,
            rays=points),effective_camera=model['operational'],
        numeric_truth_checked_after_all_sensor_fits=True,original_globals_overridden=False)


def model_mutations(report,validate,helpers,control=False):
    def profile(data):
        return data['registered_grid_control']['camera_model'] if control else data['camera_model']
    def effective(data):
        return data['registered_grid_control']['effective_calibration'] if control else data['effective_calibration']
    def legacy_k(data):
        effective(data)['fx']=517.306408
    def cross_profile(data):
        profile(data)['profile_id']='tum-fr2-registered-ros-default-v1'
    def warp(data):
        effective(data)['additional_undistortion']=True
    def distortion(data):
        effective(data)['k1']=.262383
    def double_scale(data):
        effective(data)['depth_scale_multiplier']=1.035
    def quantity(data):
        effective(data)['depth_quantity']='euclidean_range'
    def domain(data):
        effective(data)['pixel_domain']='undistorted_rgb_grid'
    def ray(data):
        if control:
            data['registered_grid_control']['rays'][1]['point'][2]*=1.035
        else:
            row=next(r for r in data['frames'][1:] if r['multiscale']['accepted'])
            row['multiscale']['correspondences'][0]['previous'][2]*=1.035
    operations=dict(legacy_k_as_registered=legacy_k,cross_profile_identity=cross_profile,
        extra_undistortion=warp,unknown_brown_distortion=distortion,double_depth_scale=double_scale,
        range_instead_of_axial_z=quantity,changed_pixel_domain=domain,double_scaled_measured_point=ray)
    if not control:
        def false_source(data):
            data['source_calibration']['fy']=525.
        operations['source_calibration_rewritten']=false_source
    for name,mutate in operations.items():
        helpers.reject_mutant(report,mutate,validate)
    return list(operations)


def audit_recorded(report,manifest_bytes,raw_root,freeze,helpers):
    source_manifest,model=protocol_check(report,manifest_bytes,freeze,helpers)
    # Pinned RGB calibration documentation is a separate source asset, never
    # an operational K/D update. Verify it opaquely as well as manifest inventory.
    source=source_manifest['calibration_source']
    calibration_raw=bounded(raw_root/source['file'],4*1024*1024)
    require(len(calibration_raw)==source['bytes'] and digest(calibration_raw)==source['sha256'],
            'original RGB calibration documentation bytes')
    proof=audit_geometry_and_labels(report,source_manifest,raw_root,model['operational'],helpers)
    proof.update(camera_model_sha256=CAMERA_MODEL_SHA,source_calibration=report['source_calibration'],
        source_calibration_document_verified=True,drift_causality_claim=False)
    return proof


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--manifest',type=Path)
    parser.add_argument('--raw',type=Path)
    parser.add_argument('--freeze',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    require_new_output(args.output)
    pair,helpers=load_math()
    if args.self_test:
        require(args.report is None,'self-test takes no sensor report')
        fixed_documents(helpers)
        proof=analytic_math_controls(pair,helpers)
    else:
        require(args.report is not None,'report required')
        report=json.loads(bounded(args.report))
        control=report.get('kind')=='analytic_continuous_measured_controls'
        if control:
            validate=lambda data:audit_control(data,pair,helpers)
        else:
            require(args.manifest is not None and args.raw is not None and args.freeze is not None,
                'recorded audit requires --manifest, --raw and --freeze')
            manifest_bytes=bounded(args.manifest,512*1024)
            freeze=json.loads(bounded(args.freeze,512*1024))
            validate=lambda data:audit_recorded(data,manifest_bytes,args.raw,freeze,helpers)
        proof=validate(report)
        proof['non_noop_model_corruptions_rejected']=model_mutations(report,validate,helpers,control)
        if control:
            # All original meaningful state mutations operate on their original
            # 320px fixtures and unchanged mathematical/state contract.
            legacy={key:report[key] for key in ('schema_version','kind','calibration','texture','cases')}
            legacy.update(sources=report['prior_sources'],
                design_sha256='d0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df')
            proof['non_noop_state_corruptions_rejected']=helpers.control_mutations(legacy,helpers.audit_control)
        else:
            proof['non_noop_state_corruptions_rejected']=helpers.temporal_mutations(report,validate)
    current,prior=compiled_sources(helpers)
    proof.update(schema='rustdriving-registered-grid-continuous-oracle-v1',
        auditor_sha256=digest(bounded(Path(__file__))),imported_temporal_oracle_sha256=TEMPORAL_SHA,
        imported_pair_oracle_sha256=PAIR_SHA,original_math_sha256=pair.PINNED_MATH,
        compiled_sources=current,prior_compiled_sources=prior,design_sha256=DESIGN_SHA,
        camera_model_sha256=CAMERA_MODEL_SHA,imported_globals_overridden=False)
    write_new(args.output,proof)
    print(json.dumps({key:proof[key] for key in ('kind','summary','maximum_accepted_root_errors',
        'non_noop_model_corruptions_rejected','non_noop_state_corruptions_rejected') if key in proof}))


if __name__=='__main__':
    main()
