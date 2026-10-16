#!/usr/bin/env python3
"""Independent interval-cell support and unchanged fixed-objective verification."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import boundary_slope_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


previous = load('boundary_original_oracle','check-slope-admission-refinement.py')
private = load('boundary_private_oracle','check-slope-admission-refinement.py')
original_sample = previous.independent_sample
EPSILON_PIXELS = 1e-9


def cells(layer):
    z = np.lib.stride_tricks.sliding_window_view(layer['depth'],(2,2));slopes = layer['depth_slopes']
    supported = np.lib.stride_tricks.sliding_window_view(slopes['valid'],(2,2)).all(axis=(-1,-2))
    for key in ('du','dv'):supported &= np.isfinite(np.lib.stride_tricks.sliding_window_view(slopes[key],(2,2))).all(axis=(-1,-2))
    supported &= ((z>.3)&(z<5)).all(axis=(-1,-2))
    supported &= np.ptp(z,axis=(-1,-2))<=.05+.01*np.min(z,axis=(-1,-2))
    result = np.zeros(layer['depth'].shape,dtype=bool);result[:-1,:-1] = supported
    result[:1,:] = False;result[:,:1] = False;result[-2:,:] = False;result[:,-2:] = False
    return result


def prepare(nodes):
    return [dict(n,surface={s:dict(layer,boundary_cells=cells(layer)) for s,layer in n['surface'].items()}) for n in previous.prepare(nodes)]


def mask(layer,points):
    c = layer['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    selected = np.flatnonzero(np.isfinite(points).all(axis=1)&(points[:,2]>.3));q = points[selected]
    with np.errstate(over='ignore',invalid='ignore'):
        image = q@k.T;uv = image[:,:2]/image[:,2,None]
    bounded = np.isfinite(uv).all(axis=1)&(uv[:,0]>0)&(uv[:,0]<c['width'])&(uv[:,1]>0)&(uv[:,1]<c['height'])
    selected,uv = selected[bounded],uv[bounded]
    lower = np.floor(uv-EPSILON_PIXELS).astype(int);upper = np.floor(uv+EPSILON_PIXELS).astype(int)
    supported = np.ones(len(selected),dtype=bool)
    for xend,yend in ((0,0),(0,1),(1,0),(1,1)):
        x = (lower,upper)[xend][:,0];y = (lower,upper)[yend][:,1]
        good = (x>=1)&(x<c['width']-2)&(y>=1)&(y<c['height']-2)
        found = np.zeros(len(selected),dtype=bool);found[good] = layer['boundary_cells'][y[good],x[good]];supported &= found
    result = np.zeros(len(points),dtype=bool);result[selected[supported]] = True;return result


def independent_sample(layer,points):
    selected = np.flatnonzero(mask(layer,points));ids,r,df,target = original_sample(layer,points[selected])
    return selected[ids],r,df,target


# All dictionaries below belong only to the separately loaded private audit.
# Keep its original center selector and the original control audit isolated.
private.independent_sample = independent_sample
private.system.__globals__['values'].__globals__['independent_sample'] = independent_sample
prepare_original = previous.prepare
population,check_metadata,system,objective = private.population,private.check_metadata,private.system,private.objective
verify,verify_checkpoint,reference = private.verify,private.verify_checkpoint,private.reference


def compare_prepared(nodes,other):
    result = previous.compare_prepared(nodes,other)
    for node,ind in zip(nodes,other,strict=True):
        for scale in node['surface']:
            np.testing.assert_array_equal(node['surface'][scale]['boundary_cells'],ind['surface'][scale]['boundary_cells'])
    return result


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    for name in ('sampling_mask','population'):
        require(body('boundary_slope_refinement.py',name)==body('slope_admission_refinement.py',name),'unchanged initial selection body '+name)
    for name in ('values','evaluate','equation','energy','search'):
        require(getattr(kernel,name) is getattr(kernel.local,name),'unchanged private objective '+name)
    for name in ('refine','snapshot','checkpoint'):
        require(getattr(kernel,name) is getattr(kernel.driver,name),'unchanged private driver '+name)
    require(kernel.local.values.__globals__['sample'] is kernel.sample,'private objective boundary sampler')
    require(kernel.refine.__globals__['population'] is kernel.population,'private initial selection binding')
    require(verify.__globals__['population'] is population and private.sampling_mask.__globals__['independent_sample'] is independent_sample,'private independent selection binding')
    require(verify_checkpoint.__globals__['population'] is population,'private checkpoint population binding')
    require(system.__globals__['values'].__globals__['independent_sample'] is independent_sample,'private independent residual sampler')
    require(previous.system.__globals__['values'].__globals__['independent_sample'] is original_sample,'original control residual remains unchanged')
    require(previous.sampling_mask.__globals__['independent_sample'] is original_sample,'original control selection remains unchanged')
    return True


def interpolation_controls():
    from boundary_depth_support import mask as operational_mask,sample as operational_sample,cells as operational_cells
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32));rho = .5+.0001*(xx-15)**2+.00008*(yy-16)**2+.00001*xx*yy
    def layer(depth):
        xyz = np.stack([(xx-c['cx'])*depth/c['fx'],(yy-c['cy'])*depth/c['fy'],depth],axis=2)
        return dict(camera=c,depth=depth,normals=np.tile([0.,0.,1.],(32,32,1)),vertices=xyz)
    def points(uv):return np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(len(uv),2.)])
    raw = layer(1/rho);op = kernel.prepare([dict(surface={1:raw})])[0]['surface'][1];ind = prepare([dict(surface={1:raw})])[0]['surface'][1]
    np.testing.assert_array_equal(op['boundary_cells'],ind['boundary_cells'])
    uv = np.array([[12.,12.3],[12.-.25e-9,12.3],[12.+.25e-9,12.3],[12.,13.],[12.2,13.4],[12.+2e-9,12.3]])
    q = points(uv);a = operational_sample(op,q);b = independent_sample(ind,q)
    for x,y in zip(a,b,strict=True):np.testing.assert_allclose(x,y,atol=3e-13,rtol=0)
    np.testing.assert_array_equal(a[0],np.arange(len(q)))
    original = kernel.previous.sample(op,q)
    for x,y in zip(a,original,strict=True):np.testing.assert_array_equal(x,y)
    gradients = 0
    for axis in range(3):
        delta = np.zeros_like(q);delta[:,axis]=1e-7
        positive,negative = operational_sample(op,q+delta),operational_sample(op,q-delta)
        np.testing.assert_allclose((positive[1]-negative[1])/2e-7,a[2][:,axis],atol=2e-8,rtol=2e-6);gradients += len(q)
    bad = layer(np.full((32,32),2.));bad['depth'][12,11] = 0.
    bop = kernel.prepare([dict(surface={1:bad})])[0]['surface'][1];bind = prepare([dict(surface={1:bad})])[0]['surface'][1]
    np.testing.assert_array_equal(operational_cells(bop),cells(bind))
    edge = points(np.array([[12.-2e-9,12.3],[12.-.25e-9,12.3],[12.,12.3],[12.+.25e-9,12.3],[12.+2e-9,12.3]]))
    np.testing.assert_array_equal(operational_mask(bop,edge),[False,False,False,False,True])
    np.testing.assert_array_equal(operational_mask(bop,edge),mask(bind,edge))
    for x,y in zip(operational_sample(bop,edge),independent_sample(bind,edge),strict=True):np.testing.assert_allclose(x,y,atol=3e-13,rtol=0)
    invalid = np.array([[np.nan,0.,2.],[0.,0.,.3],[np.inf,0.,1.],[1e300,0.,1.]])
    np.testing.assert_array_equal(operational_mask(op,invalid),False);np.testing.assert_array_equal(mask(ind,invalid),False)
    # A corner needs all four incident cells; reject even when the original
    # upper-right cell is supported and a diagonally incident cell is absent.
    corner = layer(np.full((32,32),2.));corner['depth'][11,11]=0.
    cop = kernel.prepare([dict(surface={1:corner})])[0]['surface'][1];cind = prepare([dict(surface={1:corner})])[0]['surface'][1]
    point = points(np.array([[12.,12.]]))
    require(not operational_mask(cop,point)[0] and not mask(cind,point)[0],'corner needs every incident supported cell')
    return dict(all_incident_cell_controls=True,raw_values_unchanged=True,no_coordinate_snap=True,finite_difference_axes=gradients,
        one_sided_boundary_rejected=True,missing_depth_remains_missing=True,corner_control=True,invalid_queries_rejected=True)


def controls():
    proof = source_proof();interpolation = interpolation_controls()
    audit = load('boundary_rendered','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    historical = {};checked = endpoints = 0
    for name,z in (('unquantized',depth),('quantized_2mm',np.round(depth/.002)*.002)):
        nodes = kernel.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)])
        other = prepare([dict(n,surface=audit.pyramid(z,camera)) for n in nodes]);compare_prepared(nodes,other)
        for node,ind in zip(nodes,other,strict=True):
            for scale in node['surface']:np.testing.assert_array_equal(node['surface'][scale]['boundary_cells'],ind['surface'][scale]['boundary_cells'])
        poses = np.array([identity,identity]);before = kernel.previous.population(nodes,poses,1);old_ind = previous.population(other,poses,1)
        after = kernel.population(nodes,poses,1);wanted = population(other,poses,1);check_metadata(kernel.metadata(after),wanted)
        old_ids = set(map(int,before[0]['source_ids']));ind_ids = set(map(int,old_ind[0]['source_ids']));new_ids = set(map(int,after[0]['source_ids']))
        require(old_ids!=ind_ids and new_ids<=old_ids&ind_ids,'old disagreement resolved by incident-cell support without adding sources')
        report = kernel.refine(nodes,poses,None);checked += verify(report,other,poses,None,audit)
        require(report['continuous_converged'] and not report['trace'],'independently verified identity stationarity')
        endpoints += sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,poses,None,audit,cap) for cap in (30,60,90))
        historical[name] = dict(old_operational_sources=len(old_ids),old_independent_sources=len(ind_ids),new_agreed_sources=len(new_ids),
            removed_from_operational=len(old_ids-new_ids),removed_from_independent=len(ind_ids-new_ids),source_sha256=sha(np.array(sorted(new_ids),dtype='<i8').tobytes()),
            old_only_operational=sorted(old_ids-ind_ids),old_only_independent=sorted(ind_ids-old_ids),resolved=True,identity_converged=True,applied_corrections=0)
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source);require(warm['accepted'],'measured rendered warm start')
    initial = np.asarray(warm['revised_roots']);nodes = kernel.prepare([dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(z,camera)) for t,p,z in zip((1.,1.03),initial,(depth,source),strict=True)])
    other = prepare([dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,(depth,source),strict=True)])
    report = kernel.refine(nodes,initial,None);checked += verify(report,other,initial,None,audit)
    endpoints += sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,initial,None,audit,cap) for cap in (30,60,90))
    rejected = []
    for name,change in (('false convergence',lambda r:r.update(continuous_converged=not r['continuous_converged'])),
        ('wrong source identities',lambda r:r['frozen_population'][0].update(source_pixel_sha256='0'*64)),
        ('wrong normalizer digest',lambda r:r['frozen_population'][0].update(normalizer_sha256='0'*64)),
        ('wrong final system',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist()))):
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected boundary report corruption '+name)
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))):
        raw = kernel.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)])
        ind = prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw]);poses = np.array([identity,identity]);q = kernel.refine(raw,poses,None)
        checked += verify(q,ind,poses,None,audit);require(q['guard_failure'] is not None and not q['trace'],'degenerate data keep guards');guards[name]=q['guard_failure']
    # Reuse the original independent nonzero-prior fixture, while binding only
    # the sampler and selector in this private verifier/objective.
    third = dict(timestamp=1.06,root=initial[1].copy(),surface=nodes[1]['surface']);raw3 = nodes+[third];ind3 = other+[dict(third,surface=other[1]['surface'])]
    poses3 = np.array([identity,initial[1],initial[1]])
    poses3[1] = audit.increment(poses3[1],np.array([.00073,-.00031,.00021,.00012,-.00014,.00009]))
    poses3[2] = audit.increment(poses3[2],np.array([-.00041,.00023,-.00011,-.00011,.00008,.00017]))
    prior = kernel.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    matches = kernel.population(raw3,poses3,1);expected = population(ind3,poses3,1);e = kernel.equation(matches,raw3,poses3,prior,1)
    audit.compare_equation(kernel.base.encode_equation(e),system(expected,ind3,poses3,prior.encode(),1,audit))
    numeric = []
    for node in (1,2):
        for axis in range(6):
            d = np.zeros(6);d[axis]=1e-7;plus,minus = poses3.copy(),poses3.copy()
            plus[node]=audit.increment(poses3[node],d);minus[node]=audit.increment(poses3[node],-d)
            numeric.append((objective(expected,ind3,plus,prior.encode(),1,audit)-objective(expected,ind3,minus,prior.encode(),1,audit))/2e-7)
    np.testing.assert_allclose(numeric,e['g'],atol=3e-7,rtol=2e-6)
    return dict(analytic_pass=True,source_proof=proof,interpolation=interpolation,historical_identity_controls=historical,
        reported_systems=checked,checkpoint_systems=endpoints,prior_system_checks=1,objective_gradient_axes=12,
        rendered_continuous_converged=report['continuous_converged'],rendered_applied_corrections=report['applied_iterations'],
        rejected_corruptions=rejected,guards=guards,original_solver_and_audit_bindings_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('boundary_depth_support.py','boundary_slope_refinement.py','check-boundary-slope-refinement.py')}
    write(out/'summary.json',report)


if __name__=='__main__':main()
