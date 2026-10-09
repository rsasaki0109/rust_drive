#!/usr/bin/env python3
"""Independently verify one actual opt-in adaptive-terrain RNE lead stop.

The unchanged 22-second seed-7 fixture, native rays, measured AABBs and
200 Hz motion are checked against analytic geometry and actor speed bounds.
Frozen adaptive parameters stay opt-in; all 441 sensor-only ticks are replayed.
This single authored positive does not establish real-data generalization;
the separate fresh Autzen failures must remain disclosed. No body physics,
semantic recognition, perfect ground removal or unseen accuracy is claimed.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('terrain_ground_oracle', ROOT/'scripts/check-ground-scenes.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
SCENARIO = ROOT/'scenarios/native-ground-traffic-stop.json'
SCENE = ROOT/'scenes/ground-moving-traffic.json'
EXPECTED_SCENARIO_SHA = '70b304b4b2afe305a28f70932c1137e4d2ce87e6df0be88db23017264fdfad25'
EXPECTED_SCENE_SHA = 'f1c2e9c9493404647a23cec64a4ccefae911bdcf4a5ce9a6be3513bede515b32'
MAX_RAW_BYTES = 384*1024*1024
MAX_ARCHIVE_BYTES = 128*1024*1024


def verify_geometry(p):
    root = ROOT
    r=json.load((p/'run.json').open());e=json.load((p/'scene.json').open());s=json.load((root/'scenes/ground-moving-traffic.json').open());header,ticks=g.rays.load_log(p/'sensors.jsonl');cal=header['config']['lidar3d'];beams=g.directions(cal)
    g.require(cal['azimuth_columns']==720 and cal['elevation_rings']==16
              and cal['collision_bottom_m']==.15 and cal['collision_top_m']==1.65, 'experimental acquisition/research height changed')
    g.require(e['lidar3d']==cal and e['perception3d']==header['config']['perception3d'],'native/replay calibration mismatch')
    g.require(cal.get('ground') is None and e['operating_mode']=='lidar3d_adaptive_terrain_objects','mode/ground calibration mismatch')
    fixed_terrain={'cell_size_m':1.,'initial_height_m':.15,'max_height_m':2.5,'max_slope':.3,
        'window_radii_cells':[1,2,4,8,16],'min_support_neighbors':3,'min_supported_cells':12,
        'min_supported_fraction':.5,'max_points':20000,'max_cells':20000,
        'max_candidate_work':8000000,'coordinate_bound_m':10000000.}
    fixed_objects={'tolerance_m':.6,'voxel_size_m':.6,'min_points':3,'max_points':20000,
        'max_cluster_points':20000,'max_clusters':1000,'max_candidate_work':1000000,'coordinate_bound_m':10000000.}
    fixed_adaptive={'cell_size_m':1.,'support_radius_m':16.,'max_slope':.45,'max_residual_m':.18,
        'min_support_neighbors':6,'max_support_neighbors':32,'min_supported_cells':12,
        'min_supported_fraction':.5,'max_points':20000,'max_cells':20000,
        'max_candidate_work':8000000,'coordinate_bound_m':10000000.}
    g.require(header['config']['perception3d']=={'terrain':fixed_terrain,'objects':fixed_objects,'adaptive_terrain':fixed_adaptive},
              'frozen adaptive/Euclidean profile or resource budget changed')
    physical=g.check_capsule(r,s,e,ticks)
    counts={'acquisitions':len(e['acquisitions']),'ray_grid_entries':0,'xyz_returns':0,'confident_acquisitions':0,'aabbs':0,'aabb_points':0,'actor_returns':0,'ground_returns':0,'actor_points_in_aabbs':0,'ground_points_in_aabbs':0,'maximum_range_residual_m':0.,'collision_relevant_actor_points':0,'collision_relevant_ground_points':0,'ground_only_collision_relevant_aabbs':0,'actor_collision_relevant_aabbs':0,'mixed_aabbs':0,'collision_relevant_mixed_aabbs':0,'maximum_mixed_aabb_extent_m':{'x':0.,'y':0.,'z':0.}}
    minimum_supported_fraction=1.
    minimum_supported_cells=math.inf
    maximum_candidate_work=0
    for i,a in enumerate(e['acquisitions']):
     tick=ticks[2*i];cloud=tick['input']['lidar3d'];diag=tick['expected']['perception3d'];g.require(a['cloud_3d']['returns']==cloud['returns'],'native queries not delivered as XYZ')
     g.require(a['time']==cloud['stamp']==diag['stamp'],'acquisition clocks mismatch')
     g.require(diag['confident'] or tick['expected']['emergency'],'unsupported perception did not brake')
     terrain=diag['terrain'];adaptive=diag['adaptive_terrain'];ground=diag['ground_return_indices'];n=len(cloud['returns'])
     g.require(n<=fixed_adaptive['max_points'], 'adaptive return resource budget exceeded')
     g.require(terrain['occupied_cells']==adaptive['occupied_cells']
               and terrain['supported_cells']==adaptive['supported_cells']
               and terrain['candidate_ground_cells']==adaptive['supported_cells']
               and terrain['rejected_cells']==adaptive['occupied_cells']-adaptive['supported_cells']
               and terrain['candidate_work']==adaptive['candidate_work']
               and terrain['supported_fraction']==adaptive['supported_fraction']
               and terrain['confident']==adaptive['confident'], 'adaptive/summary diagnostics mismatch')
     g.require(0<=adaptive['rejected_low_cells']<=adaptive['occupied_cells']
               and 0<=adaptive['rejected_elevated_cells']<=adaptive['occupied_cells'], 'invalid adaptive rejection counters')
     if not diag['confident']:
      g.require(tick['expected']['emergency'] and tick['expected']['command']['acceleration']==-6
                and 'InvalidLidar' in tick['expected']['health'] and not diag['objects']
                and diag['projected_points']==0, 'unsupported adaptive perception did not fail closed')
     g.require(diag['measured_points']==n and len(ground)==diag['ground_points']
               and ground==sorted(set(ground)) and all(0<=j<n for j in ground)
               and diag['ground_points']+diag['non_ground_points']==n,'invalid measured point partition')
     occupied=terrain['occupied_cells'];supported=terrain['supported_cells']
     fraction=terrain['supported_fraction']
     g.require(0<occupied<=fixed_adaptive['max_cells'] and 0<=supported<=terrain['candidate_ground_cells']<=occupied
               and terrain['candidate_ground_cells']+terrain['rejected_cells']==occupied
               and math.isfinite(fraction) and abs(fraction-supported/occupied)<1e-12
               and 0<=terrain['candidate_work']<=fixed_adaptive['max_candidate_work'], 'invalid finite ground support diagnostics')
     g.require(terrain['confident']==(supported>=fixed_adaptive['min_supported_cells'] and fraction>=fixed_adaptive['min_supported_fraction'])
               and diag['confident']==terrain['confident'],'ground confidence does not match fixed support contract')
     g.require(sum(o['point_count'] for o in diag['objects'])+diag['noise_points']==diag['non_ground_points']
               and diag['object_candidate_work']<=fixed_objects['max_candidate_work'],'object partition/resource diagnostics mismatch')
     minimum_supported_fraction=min(minimum_supported_fraction,fraction)
     minimum_supported_cells=min(minimum_supported_cells,supported)
     maximum_candidate_work=max(maximum_candidate_work,terrain['candidate_work'])
     counts['confident_acquisitions']+=diag['confident'];origin=(*g.scenes.xy(a['pose']['position']),cal['mount_height_m']);yaw=a['pose']['yaw'];c,sn=math.cos(yaw),math.sin(yaw);nonnull=[]
     for ordinal,value in enumerate(a['cloud_3d']['ranges_m']):
      dx,dy,dz=beams[ordinal];direction=(c*dx-sn*dy,sn*dx+c*dy,dz)
      candidates=[(z,'ground') for box in s['ground_cuboids'] if (z:=g.rays.box_ray(origin,direction,box)) is not None]
      candidates += [(z,'actor') for obj in a['objects'] if (z:=g.rays.capsule_ray(origin,direction,obj)) is not None]
      nearest,role=min(candidates,default=(math.inf,None));ambiguous=None
      if (value is not None)!=(cal['min_range_m']<=nearest<=cal['max_range_m']) or value is not None and abs(value-nearest)>.06:
       ambiguous=g.capsule_grazing(origin,direction,value,a['objects'],s['ground_cuboids'])
       if ambiguous:nearest,role=value,'actor'
      g.require(ambiguous is not None or (value is not None)==(cal['min_range_m']<=nearest<=cal['max_range_m']),'ray presence mismatch')
      counts['ray_grid_entries']+=1
      if value is not None:
       g.require(abs(value-nearest)<=.06,'nearest physical ray mismatch');counts['maximum_range_residual_m']=max(counts['maximum_range_residual_m'],abs(value-nearest));nonnull.append((ordinal,value,role))
     g.require(len(nonnull)==len(cloud['returns']),'XYZ count mismatch')
     roles=[]
     for measured,(ordinal,value,role) in zip(cloud['returns'],nonnull):
      point=measured['point'];dx,dy,dz=beams[ordinal]
      g.require(measured['ray_index']==ordinal and math.dist((point['x'],point['y'],point['z']),(value*dx,value*dy,cal['mount_height_m']+value*dz))<1e-7,'body XYZ mismatch')
      counts['xyz_returns']+=1;counts[role+'_returns']+=1;roles.append(role)
     object_indices=set()
     for obj in diag['objects']:
      indices=obj['return_indices'];g.require(indices and all(0<=j<n for j in indices)
          and not(set(indices)&set(ground)) and not(set(indices)&object_indices), 'AABB partition duplicates or removes ground');object_indices.update(indices);points=[cloud['returns'][j]['point'] for j in indices];g.require(len(indices)==obj['point_count'] and len(indices)==len(set(indices)),'AABB count/index mismatch')
      for axis in ['x','y','z']:
       lo=min(x[axis] for x in points);hi=max(x[axis] for x in points)
       g.require(abs(obj['min'][axis]-lo)<1e-9 and abs(obj['max'][axis]-hi)<1e-9 and abs(obj['center'][axis]-(lo+hi)/2)<1e-9,'AABB not derived from actual measured XYZ')
      g.require(obj['collision_relevant']==(obj['max']['z']>=cal['collision_bottom_m'] and obj['min']['z']<=cal['collision_top_m']),'measured height gate mismatch')
      counts['aabbs']+=1;counts['aabb_points']+=len(indices)
      component_roles={roles[j] for j in indices}
      if obj['collision_relevant']:
       counts['ground_only_collision_relevant_aabbs']+=component_roles=={'ground'}
       counts['actor_collision_relevant_aabbs']+='actor' in component_roles
      if len(component_roles)>1:
       counts['mixed_aabbs']+=1
       counts['collision_relevant_mixed_aabbs']+=obj['collision_relevant']
       for axis in ['x','y','z']:counts['maximum_mixed_aabb_extent_m'][axis]=max(counts['maximum_mixed_aabb_extent_m'][axis],obj['max'][axis]-obj['min'][axis])
      for j in indices:
       counts[roles[j]+'_points_in_aabbs']+=1
       if obj['collision_relevant']:counts['collision_relevant_'+roles[j]+'_points']+=1
     # Simulator-role labels score this evidence only; recursively verify absence from operational config/input.
    for value in [header['config']]+[tick['input'] for tick in ticks]:
     def keys(x):
      if isinstance(x,dict):return set(x).union(*(keys(v) for v in x.values()))
      if isinstance(x,list):return set().union(*(keys(v) for v in x))
      return set()
     g.require(not(keys(value)&{'truth','ground_cuboids','scene','hit_role','hit_id','acquisitions'}),'truth/role leakage')
     if value is not header['config']:g.require('objects' not in keys(value),'actor truth entered delivered sensors')
     else:g.require(set(value['perception3d']['objects'])=={'tolerance_m','voxel_size_m','min_points','max_points','max_cluster_points','max_clusters','max_candidate_work','coordinate_bound_m'},'object calibration contains actor truth')
    replay=json.load((p/'replay/replay.json').open());g.require(replay['verified'] and replay['ticks']==r['summary']['steps'],'full replay mismatch')
    def supplied_fields(actual, expected):
        if isinstance(expected, dict):
            return isinstance(actual, dict) and all(k in actual and supplied_fields(actual[k],v) for k,v in expected.items())
        if isinstance(expected, list):
            return isinstance(actual, list) and len(actual)==len(expected) and all(supplied_fields(a,b) for a,b in zip(actual,expected))
        return actual==expected
    g.require(supplied_fields(r['scenario'],json.loads(SCENARIO.read_text())), 'scenario/deadline altered')
    actor_spec=r['scenario']['objects'][0]
    g.require(all(actor_spec[k]==0 for k in ['lateral_speed','active_from','moving_from'])
              and all(actor_spec['following'][k]==v for k,v in {'minimum_gap_m':3.0,'time_headway_s':1.5,
                  'max_acceleration_m_s2':2.0,'comfortable_deceleration_m_s2':2.0,'max_deceleration_m_s2':4.0,'sensor_range_m':45.0}.items()),
              'serialized native actor default motion calibration changed')
    g.require(r['summary']['passed'] and r['summary']['simulated_seconds']==22 and r['summary']['steps']==441
              and r['summary']['collisions']==r['summary']['road_violations']==0
              and r['summary']['min_clearance']>=1 and r['summary']['progress']>=23
              and r['summary']['final_speed']<=.2, 'unchanged physical blocked-road acceptance failed')
    g.require(counts['actor_collision_relevant_aabbs']>0 and counts['ground_only_collision_relevant_aabbs']==0,
              'actual actor is unclustered or a ground-only component entered collision objects')
    # A previous 10 Hz actor position plus the known 2 m/s maximum speed bounds
    # its possible circle center at each 200 Hz native ego sample. Labels/positions
    # here belong solely to the independent scoring oracle.
    minimum_actor_reserve=math.inf
    acquisitions=e['acquisitions'];cursor=0
    for sample in e['motion_samples']:
        while cursor+1<len(acquisitions) and acquisitions[cursor+1]['time']<=sample['time']+1e-9:cursor+=1
        acquisition=acquisitions[cursor];elapsed=max(0,sample['time']-acquisition['time'])
        for actor in acquisition['objects']:
            reserve=math.dist(sample['position'],g.scenes.xy(actor['position']))-r['vehicle']['radius']-actor['radius']-2*elapsed
            minimum_actor_reserve=min(minimum_actor_reserve,reserve)
    g.require(minimum_actor_reserve>=1, 'independent bounded 200 Hz actor circle clearance below 1 m')
    counts['minimum_200hz_actor_clearance_bound_m']=minimum_actor_reserve
    counts['ground_support']={'minimum_supported_fraction':minimum_supported_fraction,'minimum_supported_cells':minimum_supported_cells,'maximum_candidate_work':maximum_candidate_work}
    counts['historical_zero_ground_member_assertion_passed']=counts['collision_relevant_ground_points']==0
    counts['confident_fraction']=counts['confident_acquisitions']/counts['acquisitions']
    g.require(0<=counts['confident_fraction']<=1 and math.isfinite(minimum_actor_reserve), 'non-finite measured diagnostics')
    return r, replay, physical, counts


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/adaptive-native')
    parser.add_argument('--verify-existing',type=Path,help='verify an existing current-profile capture instead of generating one')
    parser.add_argument('--compact',action='store_true',help='delete verified raw capture only after archive/member SHA/gzip CRC verification')
    parser.add_argument('--report',type=Path,help='also write the compact verified report at this path')
    args=parser.parse_args()
    require=g.require;sha=g.rays.sha
    require(sha(SCENARIO)==EXPECTED_SCENARIO_SHA and sha(SCENE)==EXPECTED_SCENE_SHA,'physical fixture or deadline changed')
    cli=ROOT/'target/release/rustdrive';native=ROOT/'integrations/rne/target/release/rustdrive-rne'
    require(cli.is_file() and (args.verify_existing or native.is_file()),'build locked reference/native release binaries first')
    pin=(ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    checkout=subprocess.run(['git','-C',str(ROOT.parent/'RobotNativeEngine'),'rev-parse','HEAD'],capture_output=True,text=True,check=True).stdout.strip()
    require(checkout==pin,'native checkout differs from committed RNE pin')
    dependencies=[Path(__file__),ROOT/'scripts/check-terrain-objects.py',ROOT/'scripts/check-ground-scenes.py',ROOT/'scripts/check-lidar-3d.py',ROOT/'scripts/check-native-scenes.py',ROOT/'scripts/check_hazards.py']
    checker_hashes={str(p.relative_to(ROOT)):sha(p) for p in dependencies}
    report={'schema_version':1,'source_fingerprint_sha256':g.hazards.source_fingerprint(),
        'checker_dependency_sha256':checker_hashes,'fixture_sha256':{'scenario':sha(SCENARIO),'scene':sha(SCENE)},
        'rne_revision':pin,'passed':False,'body_physics':False,'semantic_recognition':False,
        'scope':'One authored native adaptive-terrain lead-stop episode; observed-surface AABBs. Fresh measured Autzen generalization failed separately.',
        'real_data_generalization_claim':False,
        'frozen_adaptive_source_sha256':'a5e20db00507cb53de3ba264e948fc64a84d00ddb2febb40037f29c5b074b8d9'}
    args.output.mkdir(parents=True,exist_ok=True);report_path=args.output/'report.json';report_path.unlink(missing_ok=True)
    require(sha(ROOT/'crates/perception/src/terrain_adaptive.rs')==report['frozen_adaptive_source_sha256'],
            'adaptive algorithm changed after parameter freeze')
    raw=args.verify_existing if args.verify_existing else args.output/'run'
    if not args.verify_existing:
        raw.mkdir(parents=True,exist_ok=True)
        for name in ['run.json','summary.json','scene.json','sensors.jsonl','replay/replay.json','replay/outputs.jsonl']:(raw/name).unlink(missing_ok=True)
        code,error=g.hazards.invoke([native,'--plant','dynamic','--scenario',SCENARIO,'--scene',SCENE,
            '--lidar-3d','--terrain-objects','--adaptive-terrain','--seed',7,'--output',raw])
        report.update(exit_code=code,diagnostics=error)
        report_path.write_text(json.dumps(report,indent=2)+'\n')
        require(code==0,'native physical episode failed; retained raw evidence is not a positive')
    total=sum(p.stat().st_size for p in raw.rglob('*') if p.is_file())
    require(total<=MAX_RAW_BYTES,'capture exceeds the 384 MiB raw evidence budget')
    replay_code,replay_error=g.hazards.invoke([cli,'replay','--log',raw/'sensors.jsonl','--output',raw/'replay'])
    require(replay_code==0,'full raw sensor-only replay failed')
    run,replay,physical,counts=verify_geometry(raw)
    report.update(summary=run['summary'],replay=replay,physical=physical,measured_xyz_aabb_provenance=counts,
        sensor_only_boundary_verified=True,raw_sha256={name:sha(raw/name) for name in ['run.json','scene.json','sensors.jsonl']},
        speed_profiles=g.hazards.check_speed_profiles(raw/'sensors.jsonl'),
        motion_predictions=g.hazards.check_motion_predictions(raw/'sensors.jsonl'),
        control_metrics=g.hazards.control_metrics(raw/'sensors.jsonl'),
        gnss_integrity=g.hazards.check_gnss_innovation_hold(raw/'sensors.jsonl'))
    require(report['control_metrics']['max_normal_commanded_steering_rate_rad_s']<=.7+1e-8,'steering rate exceeded calibration')
    require(report['source_fingerprint_sha256']==g.hazards.source_fingerprint()
            and checker_hashes=={str(p.relative_to(ROOT)):sha(p) for p in dependencies},'source/checker changed during proof')
    archive=g.rays.compact_case(raw,report,True)
    require(archive['bytes']<=MAX_ARCHIVE_BYTES,'compressed evidence exceeds 128 MiB budget; retained raw/verified archive')
    target=args.output/'native-adaptive-terrain-seed-7.tar.gz'
    if Path(archive['path']).resolve()!=target.resolve():shutil.copyfile(archive['path'],target)
    require(sha(target)==archive['sha256'],'published archive copy hash differs')
    archive['path']=str(target);report['archive']=archive
    if args.compact:
        shutil.rmtree(raw);archive['raw_preserved']=False
    report.update(passed=True,complete=True)
    report_path.write_text(json.dumps(report,indent=2)+'\n')
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(f'{report_path}: actual 22 s native adaptive-terrain/AABB lead stop, 441 tick exact replay; passed=True')
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except (OSError,ValueError,KeyError,subprocess.CalledProcessError) as error:
        print(f'adaptive terrain/object check: {error}',file=sys.stderr);sys.exit(2)
