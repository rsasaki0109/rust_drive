#!/usr/bin/env python3
"""Independent legacy/C1 center union, frozen metrics and unchanged solver audit."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import fitted_center_refinement as kernel
from depth_correspondence_oracle import independent_matches
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


previous = load('fitted_center_original_oracle','check-boundary-slope-refinement.py')
private = load('fitted_center_private_oracle','check-boundary-slope-refinement.py')
prepare,compare_prepared,independent_sample = private.prepare,private.compare_prepared,private.independent_sample
center = private.previous.center
sampling_mask = private.private.sampling_mask


def center_seed(item,nodes,poses,scale):
    a,b = item['target'],item['source'];points,normals = item['points'][2:];relative = np.linalg.solve(poses[a],poses[b])
    q = np.column_stack([points,np.ones(len(points))])@relative.T;normal = normals@relative[:3,:3].T;bound = .12 if scale==4 else .08
    layer = nodes[a]['surface'][scale]
    ids,_,df,target = center.interpolation.independent_sample(layer,q[:,:3]);length = np.sqrt(np.einsum('ij,ij->i',df,df))
    require(np.isfinite(length).all() and np.all(length>1e-12),'finite legacy center gradient')
    geometry = np.sqrt(np.sum((q[ids,:3]-target)**2,axis=1))<bound
    alignment = np.einsum('ij,ij->i',normal[ids],df/length[:,None])>.8
    old_ids = ids[geometry&alignment]
    ci,_,cd,ct = independent_sample(layer,q[:,:3]);size = np.sqrt(np.einsum('ij,ij->i',cd,cd))
    require(np.isfinite(size).all() and np.all(size>1e-12),'finite fitted center gradient')
    cgeometry = np.sqrt(np.sum((q[ci,:3]-ct)**2,axis=1))<bound
    calignment = np.einsum('ij,ij->i',normal[ci],cd/size[:,None])>.8
    fitted_ids = ci[cgeometry&calignment];new_ids = np.setdiff1d(fitted_ids,old_ids,assume_unique=True)
    selected = np.union1d(old_ids,new_ids);legacy = np.isin(selected,old_ids)
    old_metric = {int(i):float(value) for i,value in zip(ids,length,strict=True)}
    fitted_metric = {int(i):float(value) for i,value in zip(ci,size,strict=True)}
    metrics = np.array([old_metric[int(i)] if is_old else fitted_metric[int(i)] for i,is_old in zip(selected,legacy,strict=True)])
    row = dict(target=a,source=b,points=points[selected],normals=normals[selected],source_ids=item['pixel_ids'][selected,1],normalizer=metrics)
    stages = dict(target=a,source=b,current_source=b==len(nodes)-1,raw_count=len(points),bilinear_corner_support=len(ids),
        bilinear_geometry=int(geometry.sum()),legacy_center_count=len(old_ids),fitted_corner_support=len(ci),
        fitted_geometry=int(cgeometry.sum()),fitted_center_count=len(fitted_ids),fitted_only_center_count=len(new_ids),union_center_count=len(selected))
    return row,legacy,stages


def selection(nodes,poses,scale):
    result = [];stages = []
    for b in range(1,len(nodes)):
        for a in range(b):
            item = independent_matches([dict(nodes[a],root=poses[a])],nodes[b],poses[b],scale,False)
            if item['count']<200:continue
            item = dict(item,target=a,source=b);row,legacy,stage = center_seed(item,nodes,poses,scale)
            keep = sampling_mask(row,nodes,poses,scale);old = int(np.sum(keep&legacy));total = int(np.sum(keep))
            stage.update(legacy_after_probes=old,union_after_probes=total,admitted_legacy_count=old if old>=200 else 0,
                admitted_union_count=total if total>=200 else 0,admitted_fitted_only_count=int(np.sum(keep&~legacy)) if total>=200 else 0,
                fitted_only_source_sha256=sha(np.asarray(row['source_ids'][keep&~legacy] if total>=200 else [],dtype='<i8').tobytes()))
            stages.append(stage)
            if total>=200:result.append(dict(row,**{key:row[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    return result,stages


def population(nodes,poses,scale):return selection(nodes,poses,scale)[0]


private.verify.__globals__['population'] = population
private.verify_checkpoint.__globals__['population'] = population
verify,verify_checkpoint,check_metadata,system,objective,reference = (
    private.verify,private.verify_checkpoint,private.check_metadata,private.system,private.objective,private.reference)


def source_proof():
    for name in ('values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(getattr(kernel,name) is getattr(kernel.private,name),'unchanged private boundary solver '+name)
    require(kernel.refine.__globals__['population'] is kernel.population and kernel.snapshot.__globals__['population'] is kernel.population,'one fixed union population')
    require(kernel.sample is kernel.previous.sample and kernel.prepare is kernel.previous.prepare,'unchanged fitted field/domain')
    require(kernel.previous.refine.__globals__['population'] is kernel.previous.population,'old operational selection remains isolated')
    require(verify.__globals__['population'] is population and verify_checkpoint.__globals__['population'] is population,'private independent union binding')
    require(previous.verify.__globals__['population'] is previous.population,'old independent selection remains isolated')
    return True


def admission_controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32));rho = .5+.016*abs(xx-12);z = 1/rho
    xyz = np.stack([(xx-c['cx'])*z/c['fx'],(yy-c['cy'])*z/c['fy'],z],axis=2)
    layer = dict(camera=c,depth=z,vertices=xyz,normals=np.tile([0.,0.,1.],(32,32,1)))
    point = np.array([[(12.05-c['cx'])*2/c['fx'],(12.3-c['cy'])*2/c['fy'],2.]])
    poses = np.array([np.eye(4),np.eye(4)]);raw = dict(target=0,source=1,count=1,pixel_ids=np.array([[88,77]]),points=(point,np.array([[0.,0.,1.]]),point,np.array([[0.,0.,1.]])))
    def nodes(value):return [dict(timestamp=1.+i*.03,root=np.eye(4),surface={1:value}) for i in range(2)]
    op,ind = kernel.prepare(nodes(layer)),prepare(nodes(layer));row,legacy,stage = kernel.center_seed(raw,op,poses,1);want,old,wstage = center_seed(raw,ind,poses,1)
    require(stage==wstage and stage['legacy_center_count']==0 and stage['fitted_only_center_count']==1,'C1-only agreement restores an authored source')
    check_metadata(kernel.metadata([row]),[want]);np.testing.assert_array_equal(legacy,old)
    require(kernel.sampling_mask(row,op,poses,1)[0] and sampling_mask(want,ind,poses,1)[0],'new center retains all nine model-domain probes')
    # Raw reciprocal matching and measured-normal eligibility are checked by the
    # real rendered/recorded paths. These authored rows test center gates only.
    for name,normal,query in (('legacy_only',np.array([[np.sin(1.05),0.,np.cos(1.05)]]),point),
        ('both_reject',np.array([[1.,0.,0.]]),point),('too_far',np.array([[0.,0.,1.]]),point*np.array([1.,1.,1.1]))):
        item = dict(raw,points=(point,raw['points'][1],query,normal));op_case,ind_case = op,ind
        if name=='legacy_only':
            tilted = dict(layer,normals=np.tile(normal[0],(32,32,1)))
            op_case,ind_case = kernel.prepare(nodes(tilted)),prepare(nodes(tilted))
        actual,old,counts = kernel.center_seed(item,op_case,poses,1)
        expected,eold,ecounts = center_seed(item,ind_case,poses,1);require(counts==ecounts,'independent '+name+' counts');check_metadata(kernel.metadata([actual]),[expected])
        if name=='legacy_only':require(counts['legacy_center_count']==1 and counts['fitted_center_count']==0 and len(actual['source_ids'])==1,'legacy path is retained when C1 agreement fails')
        else:require(not len(actual['source_ids']),'agreement/distance guard remains mandatory')
    bad = dict(layer,depth=layer['depth'].copy());bad['depth'][12,12]=0.
    actual,_,counts = kernel.center_seed(raw,kernel.prepare(nodes(bad)),poses,1);require(not len(actual['source_ids']),'missing measured corner depth stays missing')
    no_normal = dict(layer,normals=layer['normals'].copy());no_normal['normals'][13,13]=0.
    actual,_,counts = kernel.center_seed(raw,kernel.prepare(nodes(no_normal)),poses,1)
    require(counts['bilinear_corner_support']==0 and counts['fitted_center_count']==1,'C1 branch does not invent or require unused corner normals')
    return dict(fitted_only_center_added=True,legacy_only_center_retained=True,both_alignment_reject=True,distance_reject=True,
        missing_depth_reject=True,unused_target_corner_normal_not_filled=True,point_and_metric_audited=True)


def controls():
    proof = source_proof();inherited = previous.controls();admission = admission_controls()
    audit = load('fitted_center_rendered','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source);require(warm['accepted'],'measured rendered warm start')
    initial = np.asarray(warm['revised_roots']);nodes = kernel.prepare([dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(z,camera)) for t,p,z in zip((1.,1.03),initial,(depth,source),strict=True)])
    other = prepare([dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,(depth,source),strict=True)]);compare_prepared(nodes,other)
    actual,stages = kernel.selection(nodes,initial,1);expected,counts = selection(other,initial,1);require(stages==counts,'independent rendered stage counts');check_metadata(kernel.metadata(actual),expected)
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    endpoints = sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,initial,None,audit,cap) for cap in (30,60,90))
    rejected = []
    for name,change in (('false convergence',lambda r:r.update(continuous_converged=not r['continuous_converged'])),
        ('wrong source identities',lambda r:r['frozen_population'][0].update(source_pixel_sha256='0'*64)),
        ('wrong normalizer digest',lambda r:r['frozen_population'][0].update(normalizer_sha256='0'*64)),
        ('wrong final matrix',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist()))):
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected fitted-center corruption '+name)
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.)),('identity',depth),('quantized_identity',np.round(depth/.002)*.002)):
        raw = kernel.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)])
        ind = prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw]);poses = np.array([identity,identity]);q = kernel.refine(raw,poses,None)
        checked += verify(q,ind,poses,None,audit);require(not q['trace'],'identity/degenerate data apply no updates')
        if name in ('identity','quantized_identity'):require(q['continuous_converged'],'identity remains independently stationary')
        else:require(q['guard_failure'] is not None,'degenerate data keep guards')
        guards[name]=q['guard_failure']
    third = dict(timestamp=1.06,root=initial[1].copy(),surface=nodes[1]['surface']);raw3 = nodes+[third];ind3 = other+[dict(third,surface=other[1]['surface'])]
    poses3 = np.array([identity,initial[1],initial[1]])
    poses3[1] = audit.increment(poses3[1],np.array([.00073,-.00031,.00021,.00012,-.00014,.00009]));poses3[2] = audit.increment(poses3[2],np.array([-.00041,.00023,-.00011,-.00011,.00008,.00017]))
    prior = kernel.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    matches = kernel.population(raw3,poses3,1);wanted = population(ind3,poses3,1);e = kernel.equation(matches,raw3,poses3,prior,1)
    audit.compare_equation(kernel.base.encode_equation(e),system(wanted,ind3,poses3,prior.encode(),1,audit));numeric = []
    for node in (1,2):
        for axis in range(6):
            d = np.zeros(6);d[axis]=1e-7;plus,minus = poses3.copy(),poses3.copy();plus[node]=audit.increment(poses3[node],d);minus[node]=audit.increment(poses3[node],-d)
            numeric.append((objective(wanted,ind3,plus,prior.encode(),1,audit)-objective(wanted,ind3,minus,prior.encode(),1,audit))/2e-7)
    np.testing.assert_allclose(numeric,e['g'],atol=4e-7,rtol=3e-6)
    return dict(analytic_pass=True,source_proof=proof,inherited_boundary_controls_pass=inherited['analytic_pass'],inherited_historical_identity_controls=inherited['historical_identity_controls'],
        admission=admission,reported_systems=checked,checkpoint_systems=endpoints,prior_system_checks=1,objective_gradient_axes=12,
        rendered_sources=sum(len(r['source_ids']) for r in actual),rendered_old_sources=sum(len(r['source_ids']) for r in kernel.previous.population(nodes,initial,1)),
        rendered_continuous_converged=report['continuous_converged'],rendered_applied_corrections=report['applied_iterations'],rejected_corruptions=rejected,guards=guards)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('fitted_center_refinement.py','check-fitted-center-refinement.py')}
    write(out/'summary.json',report)


if __name__=='__main__':main()
