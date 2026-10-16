from pathlib import Path
import importlib.util,io,json,sys
import numpy as np
from PIL import Image
root=Path('/workspace/rust_drive');sys.path.insert(0,str(root/'scripts'))
import reselected_center_refinement as kernel
from projective_depth_common import sha
from boundary_depth_support import mask

def load(name,file):
 spec=importlib.util.spec_from_file_location(name,root/'scripts'/file);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
source=json.loads((root/'assets/reselected-center-refinement-v1/summary.json').read_text());case='tum-fr1-desk2-independent';saved=source['recorded'][case];sample=next(s for s in saved['samples'] if s['source_index']==200);batch=sample['refinement']['rounds'][1]['batch']
common=load('support_preflight_inputs','evaluate-reselected-center-refinement.py').common
body=common.archived(root/'assets/reciprocal-keyframe-depth-v1','first/'+case+'-retain/freeze.json');freeze=json.loads(body);assert sha(body)==saved['input_freeze_sha256']
base=root/'data'/case;body=(base/'manifest.json').read_bytes();assert sha(body)==freeze['manifest_sha256'];frames={f['depth_timestamp']:f for f in json.loads(body)['frames']}
audit=load('support_preflight_raw','check-reciprocal-keyframes.py').auditor();check=load('support_preflight_audit','check-fitted-center-refinement.py');nodes=[];other=[]
for stamp,pose in zip(sample['node_timestamps'],batch['initial_poses'],strict=True):
 name=frames[stamp]['depth_file'];body=(base/'raw'/name).read_bytes();assert sha(body)==freeze['input_sha256'][name]['sha256'];image=Image.open(io.BytesIO(body));assert image.size==(640,480);depth=np.asarray(image,dtype=float)/5000
 nodes+=kernel.fixed.prepare([dict(timestamp=stamp,root=np.asarray(pose),surface=kernel.fixed.base.pyramid(depth,freeze['camera']))])
 other+=check.prepare([dict(timestamp=stamp,root=np.asarray(pose),surface=audit.pyramid(depth,freeze['camera']))])
check.compare_prepared(nodes,other);checked=check.verify(batch,other,np.asarray(batch['initial_poses']),sample['prior_before'],audit)
population=kernel.fixed.population(nodes,np.asarray(batch['initial_poses']),1);check.check_metadata(kernel.fixed.metadata(population),check.population(other,np.asarray(batch['initial_poses']),1))
initial=np.asarray(batch['initial_poses']);poses=np.asarray(batch['final_poses']);delta=np.asarray(batch['final_correction']['delta']);trials=[]

def points(row,p):
 a,b=row['target'],row['source'];w=row['points']@p[b,:3,:3].T+p[b,:3,3];return (w-p[a,:3,3])@p[a,:3,:3]
def uv(q,c):return np.column_stack([c['fx']*q[:,0]/q[:,2]+c['cx'],c['fy']*q[:,1]/q[:,2]+c['cy']])
for trial in batch['failed_search']['trials']:
 alpha=trial['alpha'];candidate=poses.copy()
 for i,d in enumerate(delta,1):candidate[i]=audit.increment(poses[i],alpha*d)
 rows=[]
 for item in population:
  a,b=item['target'],item['source'];layer=nodes[a]['surface'][1];c=layer['camera'];query=points(item,candidate);ids=kernel.fixed.sample(layer,query)[0]
  relative=np.linalg.solve(candidate[a],candidate[b]);homogeneous=np.column_stack([item['points'],np.ones(len(query))])@relative.T;ind=check.independent_sample(other[a]['surface'][1],homogeneous[:,:3])[0];np.testing.assert_array_equal(ids,ind)
  lost=np.setdiff1d(np.arange(len(query)),ids);q_uv=uv(query,c);at_start=uv(points(item,initial),c);at_end=uv(points(item,poses),c)
  for index in lost:
   p=q_uv[index];x,y=np.floor(p).astype(int);cell=bool(layer['boundary_cells'][y,x]);bad=np.argwhere(~layer['boundary_cells']);distance=np.max(np.maximum(np.maximum(bad[:,::-1]-at_start[index],at_start[index]-(bad[:,::-1]+1)),0),axis=1)
   rows.append(dict(target=a,source=b,source_pixel=int(item['source_ids'][index]),initial_uv=at_start[index].tolist(),terminal_uv=at_end[index].tolist(),candidate_uv=p.tolist(),initial_to_terminal_pixels=(at_end[index]-at_start[index]).tolist(),distance_from_initial_to_unsupported_cell_linf_pixels=float(distance.min()),candidate_cell=[int(x),int(y)],candidate_cell_supported=cell,boundary_mask_pass=bool(mask(layer,query[index:index+1])[0]),patch=layer['boundary_cells'][y-1:y+2,x-1:x+2].tolist()))
 trials.append(dict(alpha=alpha,lost_sources=rows))
import interior_center_refinement as interior
identified=next(r for r in population if (r['target'],r['source'])==(3,4));index=int(np.flatnonzero(identified['source_ids']==279616)[0])
assert kernel.fixed.sampling_mask(identified,nodes,initial,1)[index] and not interior.sampling_mask(identified,nodes,initial,1)[index]
value=dict(checked_saved_systems=checked,applied_pose_updates=0,trials=trials,identified_source_initial_one_pixel_pass=True,identified_source_initial_two_pixel_reject=True)
Path('/tmp/rustdrive-support-preflight.json').write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(dict(checked_saved_systems=checked,trial_lost_counts=[len(t['lost_sources']) for t in trials],smallest_trial=trials[-1])))
