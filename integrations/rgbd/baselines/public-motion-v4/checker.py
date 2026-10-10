#!/usr/bin/env python3
"""Independent frozen recorded-motion geometry, physical pose and uncertainty audit."""
import argparse
import copy
import json
import math
from pathlib import Path
import importlib.util
_spec = importlib.util.spec_from_file_location("rgbd_oracle", Path(__file__).with_name("check-recorded-rgbd.py"))
r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(r)
ROOT=Path(__file__).resolve().parent.parent

def compose(a,b):
    return ([x+y for x,y in zip(a[0],r.rotate(a[1],b[0]))],r.qnorm(r.multiply(a[1],b[1])))

def timed_pose(gt,stamp):
    try:return r.interpolate(gt,stamp)
    except ValueError:return None

def physical_relative(a,b):
    return r.relative(a,b) if a is not None and b is not None else None

def audit(report, manifest, clouds, gt):
    r.require(report['ground_truth_operational'] is False,'truth used operationally')
    r.require(len(report['frames'])==11,'omitted rejected frames')
    c=report['freeze']['registration_config']; u=report['freeze']['uncertainty']
    counters={k:0 for k in ['pair_accepted','pair_accurate','map_accepted','map_accurate','pair_reference_valid','map_reference_valid','continuous_odometry_frames']}
    records=[]; identity=([0.,0.,0.],[1.,0.,0.,0.]);trajectory=identity;map_prior=identity
    origin=timed_pose(gt,manifest['frames'][0]['timestamp'])
    for i,row in enumerate(report['frames'],1):
        a,b=manifest['frames'][i-1:i+1]
        for k,v in [('previous_file',a['file']),('current_file',b['file']),('previous_timestamp',a['timestamp']),('current_timestamp',b['timestamp']),('source_index',b['source_index'])]:r.require(row[k]==v,'changed selection/timestamp')
        r.require(row['scan_points']==len(clouds[i]) and row['pair_map_points']==len(clouds[i-1]) and row['fixed_map_points']==len(clouds[0]),'cloud geometry count')
        prev=timed_pose(gt,a['timestamp']);cur=timed_pose(gt,b['timestamp']);map_truth=physical_relative(origin,cur)
        evidence={'index':b['source_index']}
        for mode,map_index,truth,key in [('pair',i-1,physical_relative(prev,cur),'pair'),('fixed_map',0,map_truth,'map')]:
            fit=row[mode];expected_prior=identity if mode=='pair' else map_prior;prior=r.pose(row[mode+'_initial_pose'])
            for x,y in zip(prior[0],expected_prior[0]):r.close(x,y,'truth-based or invented initialization')
            r.close(r.quaternion_error(prior[1],expected_prior[1]),0,'initial rotation',1e-7)
            r.require(fit['reference_valid']==(truth is not None),'invented physical reference availability')
            if truth is not None:
                counters[key+'_reference_valid']+=1;t,q=r.pose(fit['evaluation_only_truth'])
                for x,y in zip(t,truth[0]):r.close(x,y,'invented mocap translation')
                r.close(r.quaternion_error(q,truth[1]),0,'invented mocap rotation',1e-7)
            else:
                r.require(fit.get('reference_rejection') and fit['within_accuracy_gates'] is False,'unscorable fit concealed')
                r.require(not any(k in fit for k in ['evaluation_only_truth','translation_error_m','rotation_error_rad']),'invented unscorable label/error')
            r.require(type(fit['accepted']) is bool,'invalid acceptance')
            if fit['accepted']:
                counters[key+'_accepted']+=1;estimate=r.pose(fit['estimate'])
                r.require(r.norm([x-y for x,y in zip(estimate[0],prior[0])])<=c['max_translation_jump_m'] and r.quaternion_error(estimate[1],prior[1])<=c['max_rotation_jump_rad'],'unsafe registration jump')
                if mode=='fixed_map':map_prior=estimate
                n,rms=r.correspondences(clouds[i],clouds[map_index],estimate,c)
                r.close(fit['rms_m'],rms,'invented residual',1e-7);r.close(fit['inlier_fraction'],n/len(clouds[i]),'invented overlap')
                r.require(rms<=c['max_rms_m'] and n>=c['min_pairs'] and n/len(clouds[i])>=c['min_overlap'],'invalid geometry acceptance')
                r.require(fit['ambiguity_probes']==12 and fit['neighbor_checks']<=20_000_000,'unsafe budget/ambiguity shortcut')
                conditional=fit['conditional_covariance_xyz_rotation'];provisional=fit['provisional_covariance_xyz_rotation']
                for j in range(6):
                    for k in range(6):r.close(provisional[j][k],conditional[j][k]+((u['position_std_floor_m'] if j<3 else u['rotation_std_floor_rad'])**2 if j==k else 0),'covariance allowance differs from freeze')
                r.covariance_nees(conditional,[0.]*6);r.covariance_nees(provisional,[0.]*6)
                if truth is not None:
                    delta=[x-y for x,y in zip(estimate[0],truth[0])];te=r.norm(delta);ae=r.quaternion_error(estimate[1],truth[1])
                    r.close(fit['translation_error_m'],te,'invented translation accuracy');r.close(fit['rotation_error_rad'],ae,'invented rotation accuracy')
                    ok=te<=.1 and ae<=.1;r.require(fit['within_accuracy_gates']==ok,'lowered pose gates');counters[key+'_accurate']+=int(ok)
                    dq=r.qnorm(r.multiply(estimate[1],r.conjugate(truth[1])));norm=r.norm(dq[1:]);rv=[x*ae/norm for x in dq[1:]] if norm>1e-12 else [0.]*3
                    nees=r.covariance_nees(provisional,delta+rv);raw_nees=r.covariance_nees(conditional,delta+rv)
                    evidence[mode]={'translation_error_m':te,'rotation_error_rad':ae,'conditional_nees':raw_nees,'provisional_nees':nees,'inside_nominal_95_reference':nees<=12.591587}
                else:evidence[mode]={'accepted_sensor_fit':True,'reference_rejection':fit['reference_rejection']}
            else:
                r.require(fit['within_accuracy_gates'] is False and fit.get('rejection'),'hidden rejection')
                r.require(not any(k in fit for k in ['estimate','translation_error_m','provisional_covariance_xyz_rotation']),'invented rejected estimate')
                evidence[mode]={'rejection':fit['rejection'],'reference_valid':truth is not None}
        pair=row['pair'];trajectory=compose(trajectory,r.pose(pair['estimate'])) if pair['accepted'] and trajectory is not None else None
        odo=row['continuous_odometry'];r.require(odo['valid']==(trajectory is not None),'invented odometry bridge')
        if trajectory is not None:
            counters['continuous_odometry_frames']+=1;actual=r.pose(odo['estimate'])
            for x,y in zip(actual[0],trajectory[0]):r.close(x,y,'odometry composition')
            r.close(r.quaternion_error(actual[1],trajectory[1]),0,'odometry quaternion composition',1e-7)
            r.require(odo['reference_valid']==(map_truth is not None),'invented odometry truth')
            if map_truth is not None:
                r.close(odo['translation_error_m'],r.norm([x-y for x,y in zip(actual[0],map_truth[0])]),'odometry position error');r.close(odo['rotation_error_rad'],r.quaternion_error(actual[1],map_truth[1]),'odometry angle error')
            else:r.require(odo.get('reference_rejection') and 'translation_error_m' not in odo,'invented odometry score')
        records.append(evidence)
    r.require(report['summary']==dict(pairs=11,**counters),'summary omitted failures')
    return records

def main():
    p=argparse.ArgumentParser();p.add_argument('--report',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--raw',type=Path,required=True);p.add_argument('--freeze',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    report=json.loads(r.bounded(a.report));manifest=json.loads(r.bounded(a.manifest));freeze_raw=r.bounded(a.freeze);freeze=json.loads(freeze_raw)
    r.require(report['freeze']==freeze and report['freeze_sha256']==r.digest(freeze_raw),'external freeze mismatch')
    paths={'matcher_source_sha256':'crates/localization/src/registration3d.rs','evaluator_source_sha256':'integrations/rgbd/src/main.rs','motion_source_sha256':'integrations/rgbd/src/motion.rs','cargo_lock_sha256':'integrations/rgbd/Cargo.lock','independent_checker_sha256':'scripts/check-recorded-motion.py','geometry_checker_sha256':'scripts/check-recorded-rgbd.py'}
    for key,path in paths.items():r.require(freeze[key]==r.digest(r.bounded(ROOT/path)),'frozen source changed '+path)
    r.require(freeze['manifest_sha256']==r.digest(r.bounded(a.manifest)),'manifest changed')
    r.require(freeze['registration_config_sha256']==r.canonical(freeze['registration_config']),'configuration freeze mismatch')
    raw={}
    for f in manifest['files']:
        b=r.bounded(a.raw/f['file']);r.require(len(b)==f['bytes'] and r.digest(b)==f['sha256'],'raw hash mismatch');raw[f['file']]=b
    index=[line.split() for line in raw['depth.txt'].decode().splitlines() if line and not line.startswith('#')]
    clouds=[]
    for f in manifest['frames']:
        r.require(float(index[f['source_index']][0])==f['timestamp'] and Path(index[f['source_index']][1]).name==f['file'],'source-index selection mismatch')
        coarse={}
        for point in r.depth_geometry(raw[f['file']]):coarse.setdefault(tuple(math.floor(x/.06) for x in point),point)
        clouds.append([coarse[k] for k in sorted(coarse)])
    gt=r.gt_rows(raw['groundtruth.txt']);records=audit(report,manifest,clouds,gt)
    mutations=[]
    tests=[('omit_failure',lambda x:x['frames'].pop()),('invent_summary',lambda x:x['summary'].__setitem__('pair_accurate',99)),('truth_operational',lambda x:x.__setitem__('ground_truth_operational',True)),('timestamp',lambda x:x['frames'][0].__setitem__('current_timestamp',0.))]
    for i,row in enumerate(report['frames']):
        for mode in ['pair','fixed_map']:
            if row[mode]['accepted']:
                tests.extend([(mode+'_position',lambda x,i=i,mode=mode:x['frames'][i][mode]['estimate']['translation_m'].__setitem__(0,100.)),(mode+'_covariance',lambda x,i=i,mode=mode:x['frames'][i][mode]['provisional_covariance_xyz_rotation'][0].__setitem__(0,1e-12))]);break
        if len(tests)>4:break
    first_map=next((i for i,x in enumerate(report['frames']) if x['fixed_map']['accepted']),None)
    if first_map is not None:
        tests.extend([('fixed_map_position',lambda x,i=first_map:x['frames'][i]['fixed_map']['estimate']['translation_m'].__setitem__(1,100.)),('fixed_map_covariance',lambda x,i=first_map:x['frames'][i]['fixed_map']['provisional_covariance_xyz_rotation'][3].__setitem__(3,-1.)),('ground_truth_initialization',lambda x,i=first_map:x['frames'][i]['fixed_map_initial_pose']['translation_m'].__setitem__(2,10.))])
    first_odo=next((i for i,x in enumerate(report['frames']) if x['continuous_odometry']['valid']),None)
    if first_odo is not None:tests.append(('odometry_composition',lambda x,i=first_odo:x['frames'][i]['continuous_odometry']['estimate']['translation_m'].__setitem__(0,10.)))
    rejected=next(((i,m) for i,x in enumerate(report['frames']) for m in ['pair','fixed_map'] if not x[m]['accepted']),None)
    if rejected is not None:
        i,m=rejected;tests.append(('invent_rejected_estimate',lambda x,i=i,m=m:x['frames'][i][m].__setitem__('estimate',{'translation_m':[0,0,0],'quaternion_wxyz':[1,0,0,0]})))
    for name,change in tests:
        altered=copy.deepcopy(report);change(altered)
        try:audit(altered,manifest,clouds,gt)
        except (ValueError,KeyError,IndexError):mutations.append(name)
        else:raise ValueError('mutation accepted '+name)
    result={'schema_version':1,'passed_integrity':True,'summary':report['summary'],'frames':records,'mutations_rejected':mutations,'raw_sha256_verified':True,'source_freeze_verified':True,'physical_pose_independently_scored':True,'optimizer_independently_rerun':False,'confidence_coverage_claim':False}
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result['summary']))
if __name__=='__main__':main()
