#!/usr/bin/env python3
"""Independently audit saved terminal depth objectives without applying updates."""
import argparse
import copy
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import depth_objective_bridge as diagnostic
from depth_correspondence_oracle import independent_matches
from projective_depth_common import ROOT,fresh,require,sha,write
from reciprocal_depth_oracle import cross_matrices

kernel = diagnostic.kernel
CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


verifier = load('objective_bridge_continuous_audit','check-slope-admission-refinement.py')
fixed_audit = load('objective_bridge_fixed_audit','check-fixed-depth-refinement.py')
objective = load('objective_bridge_discrete_objective','check-descent-keyframes.py')
common = load('objective_bridge_inputs','evaluate-alternating-depth-refinement.py')


def original_population(nodes,poses):
    result = []
    for b in range(1,len(nodes)):
        for a in range(b):
            pair = independent_matches([dict(nodes[a],root=poses[a])],nodes[b],poses[b],1,False)
            if pair['count']>=200:result.append(dict(pair,target=a,source=b))
    return result


def continuous_rows(matches,nodes,poses):
    result = []
    for item in matches:
        a,b = item['target'],item['source'];relative = np.linalg.solve(poses[a],poses[b])
        q = np.column_stack([item['points'],np.ones(len(item['points']))])@relative.T
        ids,f,df,_ = verifier.independent_sample(nodes[a]['surface'][1],q[:,:3])
        np.testing.assert_array_equal(ids,np.arange(len(q)))
        df = df/item['normalizer'][:,None]
        world = item['points']@poses[b][:3,:3].T+poses[b][:3,3]
        differential = -np.einsum('ij,njk->nik',poses[a][:3,:3].T,cross_matrices(world))
        j = np.concatenate([df@poses[a][:3,:3].T,np.einsum('ni,nij->nj',df,differential)],axis=1)
        result.append(dict(target=a,source=b,source_ids=item['source_ids'],points=item['points'],
            normals=item['normals'],r=f/item['normalizer'],j=j))
    return result


def discrete_rows(matches,poses):
    result = []
    for item in matches:
        a,b = item['target'],item['source'];pt,nt,ps,ns = item['points'];relative = np.linalg.solve(poses[a],poses[b])
        q = ps@relative[:3,:3].T+relative[:3,3];sn = ns@relative[:3,:3].T
        summed = nt+sn;length = np.linalg.norm(summed,axis=1);n = summed/length[:,None]
        difference = q-pt;r = np.einsum('ni,ni->n',n,difference)
        world = ps@poses[b][:3,:3].T+poses[b][:3,3];wn = ns@poses[b][:3,:3].T
        dq = -np.einsum('ij,njk->nik',poses[a][:3,:3].T,cross_matrices(world))
        dn = -np.einsum('ij,njk->nik',poses[a][:3,:3].T,cross_matrices(wn))
        projector = np.eye(3)[None,:,:]-np.einsum('ni,nj->nij',n,n)
        dn = np.einsum('nij,njk->nik',projector,dn)/length[:,None,None]
        j = np.concatenate([n@poses[a][:3,:3].T,np.einsum('ni,nij->nj',n,dq)+np.einsum('ni,nij->nj',difference,dn)],axis=1)
        result.append(dict(target=a,source=b,source_ids=item['pixel_ids'][:,1],target_ids=item['pixel_ids'][:,0],
            points=ps,normals=ns,r=r,j=j))
    return result


def flatten(rows,current):
    result = {}
    for row in rows:
        a,b = row['target'],row['source'];incidence = np.array([int(i==b)-int(i==a) for i in range(1,current+1)])
        jacobian = np.einsum('i,nj->nij',incidence,row['j']).reshape(len(row['r']),6*current)
        for i,s in enumerate(row['source_ids']):
            key = (a,b,int(s));require(key not in result,'independent unique source occurrence')
            result[key] = (row['r'][i],jacobian[i],row['points'][i],row['normals'][i],
                None if 'target_ids' not in row else int(row['target_ids'][i]))
    return result


def check_bridge(actual,fixed,original,new,current):
    c,o,f = [flatten(rows,current) for rows in (fixed,original,new)]
    shared = sorted(c.keys()&f.keys());lost = c.keys()-f.keys();added = f.keys()-c.keys()
    for key in c:
        require(key in o,'independent original source ownership')
        np.testing.assert_array_equal(c[key][2],o[key][2]);np.testing.assert_array_equal(c[key][3],o[key][3])
    for key in shared:
        np.testing.assert_array_equal(c[key][2],f[key][2]);np.testing.assert_array_equal(c[key][3],f[key][3])
    def score(jrows,rrows,keys):
        if not keys:return np.zeros(6*current)
        return np.array([jrows[k][1] for k in keys]).T@np.array([min(.01,max(-.01,rrows[k][0])) for k in keys])
    stages = [score(c,c,sorted(c)),score(c,c,shared),score(c,o,shared),score(o,o,shared),score(f,f,shared),score(f,f,sorted(f))]
    expected = {key:stages[i+1]-stages[i] for i,key in enumerate(diagnostic.TERMS)}
    for key,value in expected.items():np.testing.assert_allclose(actual['terms'][key],value,atol=2e-9,rtol=2e-8,err_msg=key)
    for key,value in (('fixed_raw_gradient',stages[0]),('fresh_raw_gradient',stages[-1]),('gradient_difference',stages[-1]-stages[0])):
        np.testing.assert_allclose(actual[key],value,atol=2e-9,rtol=2e-8,err_msg=key)
    pairs = []
    for a,b in sorted({k[:2] for k in c.keys()|f.keys()}):
        group = lambda keys:sorted(k[2] for k in keys if k[:2]==(a,b))
        ci,fi,si,li,ai = map(group,(c,f,shared,lost,added))
        pairs.append(dict(target=a,source=b,fixed_count=len(ci),fresh_count=len(fi),shared_count=len(si),
            removed_count=len(li),added_count=len(ai),retargeted_shared_count=sum(o[a,b,s][4]!=f[a,b,s][4] for s in si),
            shared_source_sha256=sha(np.array(si,dtype='<i8').tobytes()),
            removed_source_sha256=sha(np.array(li,dtype='<i8').tobytes()),added_source_sha256=sha(np.array(ai,dtype='<i8').tobytes())))
    require(actual['pairs']==pairs,'independent source partitions and identity digests')
    residual = sum(np.array(actual['terms'][k]) for k in diagnostic.TERMS)-np.array(actual['gradient_difference'])
    np.testing.assert_allclose(residual,0.,atol=2e-9,rtol=0)
    np.testing.assert_allclose(actual['max_partition_error'],max(abs(residual)),atol=2e-12,rtol=0)
    return len(shared),len(lost),len(added)


def check_scaled(actual,report,h):
    scale = np.reciprocal(np.sqrt(h.diagonal()));np.testing.assert_allclose(actual['coordinate_scale'],scale,atol=1e-14,rtol=1e-12)
    difference = scale*np.asarray(report['gradient_difference']);length = np.linalg.norm(difference)
    np.testing.assert_allclose(actual['scaled_difference_norm'],length,atol=1e-12,rtol=1e-10)
    for key in diagnostic.TERMS:
        value = scale*np.asarray(report['terms'][key]);entry = actual['terms'][key]
        np.testing.assert_allclose(entry['scaled_norm'],np.linalg.norm(value),atol=1e-12,rtol=1e-10)
        if length==0:require(entry['signed_projection_fraction'] is None,'zero difference has no projection fraction')
        else:np.testing.assert_allclose(entry['signed_projection_fraction'],np.dot(value,difference)/length**2,atol=1e-12,rtol=1e-10)


def compare_rows(actual,wanted):
    require(len(actual)==len(wanted),'independent complete factor rows')
    for row,other in zip(actual,wanted,strict=True):
        for key in ('target','source'):require(row[key]==other[key],'independent factor endpoints')
        np.testing.assert_array_equal(row['source_ids'],other['source_ids'])
        if 'target_ids' in row:np.testing.assert_array_equal(row['target_ids'],other['target_ids'])
        for key in ('r','j','points','normals'):np.testing.assert_allclose(row[key],other[key],atol=2e-10,rtol=2e-8)


def correction(e,current,audit):
    actual = kernel.discrete.correction(e,current);fixed_audit.check_correction(actual,e,current,audit)
    delta = actual['delta']
    return dict(actual,max_translation_m=None if delta is None else max(float(np.linalg.norm(d[:3])) for d in delta),
        max_rotation_rad=None if delta is None else max(float(np.linalg.norm(d[3:])) for d in delta))


def measure(nodes,other,initial,poses,op_prior,prior,audit,report=None):
    fixed = kernel.population(nodes,initial,1);ind_fixed = verifier.population(other,initial,1)
    verifier.check_metadata(kernel.metadata(fixed),ind_fixed)
    original = kernel.discrete.population(nodes,initial,1);ind_original = original_population(other,initial)
    new = kernel.discrete.population(nodes,poses,1);ind_new = original_population(other,poses)
    op_rows = (diagnostic.continuous_rows(fixed,nodes,poses),diagnostic.discrete_rows(original,poses),diagnostic.discrete_rows(new,poses))
    ind_rows = (continuous_rows(ind_fixed,other,poses),discrete_rows(ind_original,poses),discrete_rows(ind_new,poses))
    for actual,expected in zip(op_rows,ind_rows,strict=True):compare_rows(actual,expected)
    bridge = diagnostic.bridge(*op_rows,len(nodes)-1);check_bridge(bridge,*ind_rows,len(nodes)-1)
    first = kernel.equation(fixed,nodes,poses,op_prior,1);last = kernel.discrete.equation(new,nodes,poses,op_prior)
    ind_first = verifier.system(ind_fixed,other,poses,prior,1,audit)
    ind_last = fixed_audit.independent_system(ind_new,other,poses,prior,audit,objective)
    for e,wanted in ((first,ind_first),(last,ind_last)):audit.compare_equation(kernel.base.encode_equation(e),wanted)
    # Also bind fresh selection to the unchanged native/discrete equation.
    audit.compare_equation(kernel.base.encode_equation(last),audit.independent_equation(other,poses,1,prior))
    if report is not None:
        audit.compare_equation(report['final_equation'],ind_first);audit.compare_equation(report['final_discrete_equation'],ind_last)
    prior_gradient = kernel.base.prior_system(op_prior,nodes,poses)[1]
    np.testing.assert_allclose(first['g'],prior_gradient+bridge['fixed_raw_gradient'],atol=2e-9,rtol=2e-8)
    np.testing.assert_allclose(last['g'],prior_gradient+bridge['fresh_raw_gradient'],atol=2e-9,rtol=2e-8)
    np.testing.assert_allclose(last['g']-first['g'],bridge['gradient_difference'],atol=2e-9,rtol=2e-8)
    reselected = kernel.population(nodes,poses,1);ind_reselected = verifier.population(other,poses,1)
    verifier.check_metadata(kernel.metadata(reselected),ind_reselected)
    re = kernel.equation(reselected,nodes,poses,op_prior,1);ind_re = verifier.system(ind_reselected,other,poses,prior,1,audit)
    audit.compare_equation(kernel.base.encode_equation(re),ind_re)
    scaled = diagnostic.scaled_summary(bridge,last['h']);check_scaled(scaled,bridge,ind_last['h'])
    return dict(bridge=bridge,scaled=scaled,prior_gradient=prior_gradient.tolist(),
        fixed_continuous=dict(source_occurrences=first['count'],correction=correction(first,len(nodes)-1,audit)),
        fresh_discrete=dict(source_occurrences=last['count'],correction=correction(last,len(nodes)-1,audit)),
        reselected_continuous=dict(source_occurrences=re['count'],correction=correction(re,len(nodes)-1,audit)),
        endpoint_systems_independently_checked=3,archived_endpoint_systems_rechecked=0 if report is None else 2)


def controls():
    # Authored rows force every bridge term, Huber clipping, source loss/addition,
    # retargeting and opposite incidence on two moving endpoints.
    def rows(ids,r,j,targets):
        ids = np.array(ids,dtype=int);n = len(ids)
        return [dict(target=1,source=2,source_ids=ids,target_ids=np.array(targets,dtype=int),r=np.array(r),j=np.array(j,dtype=float).reshape(n,6),
            points=np.column_stack([ids,ids*.1,np.ones(n)*2]),normals=np.tile([0.,0.,1.],(n,1)))]
    c = rows([10,20,30],[.002,-.02,.006],np.arange(18).reshape(3,6)/20,[11,21,31])
    o = rows([10,20,30],[.003,-.004,.03],np.arange(18).reshape(3,6)/17+.1,[11,21,31])
    f = rows([20,30,40],[-.004,.007,-.02],[o[0]['j'][1],np.arange(6)/9,np.arange(6)/7],[21,32,41])
    report = diagnostic.bridge(c,o,f,2);check_bridge(report,c,o,f,2)
    require(all(np.linalg.norm(report['terms'][k])>0 for k in diagnostic.TERMS),'authored nonzero five-term decomposition')
    scaled = diagnostic.scaled_summary(report,np.diag(np.arange(1,13)));check_scaled(scaled,report,np.diag(np.arange(1,13)))
    same = diagnostic.bridge(c,c,c,2);check_bridge(same,c,c,c,2)
    check_scaled(diagnostic.scaled_summary(same,np.eye(12)),same,np.eye(12))
    empty = diagnostic.bridge([],[],[],2);check_bridge(empty,[],[],[],2)
    rejected = []
    changes = [('wrong gradient term',lambda v:v['terms']['shared_jacobian'].__setitem__(0,v['terms']['shared_jacobian'][0]+1)),
        ('wrong source partition',lambda v:v['pairs'][0].update(shared_count=3)),
        ('wrong source digest',lambda v:v['pairs'][0].update(added_source_sha256='0'*64)),
        ('wrong endpoint gradient',lambda v:v['fresh_raw_gradient'].__setitem__(0,1.))]
    for name,change in changes:
        bad = copy.deepcopy(report);change(bad)
        try:check_bridge(bad,c,o,f,2)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected bridge corruption '+name)
    guards = []
    for name,change in (('duplicate source',lambda v:v[0]['source_ids'].__setitem__(1,10)),
        ('missing original source',lambda v:v[1]['source_ids'].__setitem__(0,99)),
        ('changed source geometry',lambda v:v[2]['points'].__setitem__((0,0),99.)),
        ('nonfinite differential',lambda v:v[0]['j'].__setitem__((0,0),np.nan))):
        sets = copy.deepcopy((c,o,f));change([sets[0][0],sets[1][0],sets[2][0]])
        try:diagnostic.bridge(*sets,2)
        except (ValueError,AssertionError):guards.append(name)
        else:raise ValueError('unguarded bridge input '+name)
    audit = load('objective_bridge_rendered','check-reciprocal-keyframes.py').auditor();camera = audit.rays.CAMERA
    truth = np.array([np.eye(4),audit.increment(np.eye(4),np.array([.04,0,.002,0,.0015,0]))])
    depth = [audit.rays.render(p) for p in truth];initial = np.array([truth[0],truth[1],truth[1]])
    initial[1] = audit.increment(initial[1],np.array([.0007,-.0003,.0002,.0001,-.00014,.00009]))
    initial[2] = audit.increment(initial[2],np.array([-.0004,.0002,-.0001,-.0001,.00008,.00017]))
    depths = depth+[depth[1]]
    nodes = kernel.prepare([dict(timestamp=1.+i*.03,root=p.copy(),surface=kernel.base.pyramid(z,camera)) for i,(p,z) in enumerate(zip(initial,depths,strict=True))])
    other = verifier.prepare([dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,depths,strict=True)])
    op_prior = kernel.base.Prior(1.,[1.03],[np.eye(4)],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1);prior = op_prior.encode()
    poses = initial.copy();poses[2] = audit.increment(poses[2],np.array([.0001,-.0002,.0001,.0001,0,0]))
    actual = measure(nodes,other,initial,poses,op_prior,prior,audit)
    fixed = kernel.population(nodes,initial,1);new = kernel.discrete.population(nodes,poses,1)
    axes = 0
    for matches,energy in ((fixed,lambda p:kernel.energy(fixed,nodes,p,op_prior,1)),(new,lambda p:kernel.discrete.energy(new,nodes,p,op_prior))):
        e = kernel.equation(matches,nodes,poses,op_prior,1) if matches is fixed else kernel.discrete.equation(matches,nodes,poses,op_prior)
        numeric = []
        for node in (1,2):
            for axis in range(6):
                d = np.zeros(6);d[axis]=1e-7;plus,minus = poses.copy(),poses.copy()
                plus[node]=audit.increment(poses[node],d);minus[node]=audit.increment(poses[node],-d)
                numeric.append((energy(plus)-energy(minus))/2e-7);axes += 1
        np.testing.assert_allclose(numeric,e['g'],atol=4e-7,rtol=3e-6)
    return dict(analytic_pass=True,authored_five_terms_nonzero=True,identity_and_empty_zero_differences=True,
        corruptions_rejected=rejected,input_guards=guards,rendered_endpoint_systems=actual['endpoint_systems_independently_checked'],
        nonzero_prior_gradient_axes=axes,rendered=actual)


def statistics(samples):
    result = dict(samples=len(samples),fixed_converged=0,fresh_discrete_converged=0,reselected_continuous_converged=0,
        fixed_source_occurrences=0,fresh_source_occurrences=0,shared_source_occurrences=0,removed_source_occurrences=0,
        added_source_occurrences=0,retargeted_shared_source_occurrences=0,endpoint_systems_independently_checked=0,archived_endpoint_systems_rechecked=0)
    for sample in samples:
        for field,out in (('fixed_continuous','fixed_converged'),('fresh_discrete','fresh_discrete_converged'),('reselected_continuous','reselected_continuous_converged')):
            result[out] += sample[field]['correction']['converged']
        for out,field in (('fixed_source_occurrences','fixed_count'),('fresh_source_occurrences','fresh_count'),('shared_source_occurrences','shared_count'),
            ('removed_source_occurrences','removed_count'),('added_source_occurrences','added_count'),('retargeted_shared_source_occurrences','retargeted_shared_count')):
            result[out] += sum(p[field] for p in sample['bridge']['pairs'])
        for field in ('endpoint_systems_independently_checked','archived_endpoint_systems_rechecked'):result[field] += sample[field]
    return result


def recorded(case,source):
    saved = source['recorded'][case];require([s['source_index'] for s in saved['samples']]==list(SAMPLES),'same viewed terminal windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body);require(sha(body)==saved['input_freeze_sha256'],'original input freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256']==inventory[prefix+'sensor-only.json.gz']['sha256'],'original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']}
    audit = load('objective_bridge_raw_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor();surfaces = {};samples = []
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];receipt = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0<path.stat().st_size<=4*1024*1024,'bounded original depth PNG')
            body = path.read_bytes();require(sha(body)==receipt['sha256'] and len(body)==receipt['bytes'],'original PNG digest')
            image = Image.open(io.BytesIO(body));require(image.size==(640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth image')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.prepare([dict(surface=kernel.base.pyramid(depth,freeze['camera']))])[0]['surface'],
                verifier.prepare([dict(surface=audit.pyramid(depth,freeze['camera']))])[0]['surface'])
        return surfaces[stamp]
    for sample in saved['samples']:
        report = sample['refinement'];initial = np.asarray(report['initial_poses']);poses = np.asarray(report['final_poses']);prior = sample['prior_before']
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes];verifier.compare_prepared(nodes,other)
        before = json.dumps(sample,sort_keys=True);value = measure(nodes,other,initial,poses,common.prior_object(prior),prior,audit,report)
        require(before==json.dumps(sample,sort_keys=True),'archived input report unmodified')
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        value.update(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],
            initial_poses_sha256=sha(np.asarray(initial,dtype='<f8').tobytes()),terminal_poses_sha256=sha(np.asarray(poses,dtype='<f8').tobytes()),
            prior_sha256=sha(json.dumps(prior,sort_keys=True).encode()))
        samples.append(value);surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'fixed / fresh / reselected',value['fixed_continuous']['correction']['converged'],
            value['fresh_discrete']['correction']['converged'],value['reselected_continuous']['correction']['converged'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],samples=samples,statistics=statistics(samples))


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--analytic-only',action='store_true')
    args = parser.parse_args();out = fresh(args.output);analytic = controls();write(out/'analytic.json',analytic)
    if args.analytic_only:return
    packet = ROOT/'assets/depth-objective-bridge-v1';freeze = json.loads((packet/'source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'frozen source '+name)
    body = common.archived(ROOT/'assets/slope-admission-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free saved endpoints')
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'unchanged source '+name)
    write(out/'summary.json',dict(data_role='already viewed read-only terminal objective diagnosis',sample_indices=list(SAMPLES),
        input_slope_admission_sha256=sha(body),new_local_pose_fitting=False,new_trajectory_fitting=False,numeric_ground_truth_parsed=False,
        physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,recorded=reports,source_sha256=freeze))


if __name__=='__main__':main()
