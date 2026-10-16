#!/usr/bin/env python3
"""Independent turnover accounting, source selection parity and corruption controls."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import depth_correspondence_stability as operational
import depth_correspondence_oracle as oracle
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def compare(actual,expected):
    require(set(actual) == set(expected),'complete turnover schema')
    for key,wanted in expected.items():
        value = actual[key]
        if isinstance(wanted,(float,np.floating)):
            np.testing.assert_allclose(value,wanted,atol=1e-10,rtol=1e-9,err_msg=key)
        elif isinstance(wanted,bool):require(type(value) is bool and value == wanted,'turnover boolean '+key)
        else:require(value == wanted,'turnover field '+key)


def source_proof():
    rows = []
    for old,new,function in [('frozen_depth_objective.py','depth_correspondence_stability.py','select'),
                              ('check-descent-keyframes.py','depth_correspondence_oracle.py','independent_matches')]:
        def body(name):
            source = (ROOT/'scripts'/name).read_text()
            node = next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name == function)
            return ast.get_source_segment(source,node)
        require(body(new) == body(old).replace('pixel_sha256=sha(indices.tobytes()))',
            'pixel_sha256=sha(indices.tobytes()),pixel_ids=indices)'),'only pixel IDs added to frozen selector')
        rows.append(dict(parent=old,child=new,function=function,only_pixel_ids_added=True))
    return rows


def fixture(ids):
    ids = np.asarray(ids,dtype='<i8').reshape(-1,2);n = len(ids)
    pt = np.zeros((n,3));pt[:,2] = np.where(ids[:,0] >= 300,2.002,2.)
    ps = np.zeros((n,3));ps[:,2] = 2.004
    normal = np.tile([0.,0.,1.],(n,1))
    return dict(points=(pt,normal,ps,normal),count=n,pixel_ids=ids,pixel_sha256=sha(ids.tobytes()))


def controls():
    proof = source_proof();audit = load('turnover_render_audit','check-reciprocal-keyframes.py').auditor()
    frozen = load('turnover_frozen_objective','check-descent-keyframes.py')
    before = np.eye(4);after = before.copy();after[2,3] = .001
    a = fixture([(i,i) for i in range(200)])
    b = fixture([(i,i) for i in range(197)]+[(300,197),(400,400),(401,401)])
    args = (a,b,before,before,before,after)
    actual = operational.transition(*args);expected = oracle.transition(*args);compare(actual,expected)
    require((actual['stable_pairs'],actual['retargeted_sources'],actual['disappeared_sources'],actual['appeared_sources']) == (197,1,2,2),'hand-counted source classification')
    require(actual['removed_pairs'] == actual['added_pairs'] == 3,'equal population can hide changed pairs')
    np.testing.assert_allclose(actual['association_delta'],-.000024,atol=1e-14,rtol=0)
    c = fixture([(i,i) for i in range(199)])
    gate = operational.transition(c,a,*([before]*4));compare(gate,oracle.transition(c,a,*([before]*4)))
    require(gate['stable_pairs'] == 199 and not gate['before_admitted'] and gate['after_admitted']
        and gate['admitted_stable_pairs'] == 0 and gate['admitted_added_pairs'] == 200,'199-to-200 gate charges entire new factor')
    np.testing.assert_allclose(gate['association_delta'],.0016,atol=1e-14,rtol=0)
    reverse = operational.transition(a,c,*([before]*4));compare(reverse,oracle.transition(a,c,*([before]*4)))
    require(reverse['admitted_removed_pairs'] == 200 and reverse['admitted_added_pairs'] == 0,'200-to-199 removes entire factor')
    empty = fixture([])
    for left,right in ((empty,empty),(empty,c),(empty,a),(a,empty)):
        compare(operational.transition(left,right,*([before]*4)),oracle.transition(left,right,*([before]*4)))
    # Physical rendered pixels exercise the augmented matchers, with SVD normals
    # separately reconstructed. The manual ID fixture above tests bookkeeping.
    camera = audit.rays.CAMERA;depth = audit.rays.render(np.eye(4))
    target = dict(root=before,surface=operational_pyramid(depth,camera))
    source = dict(root=before,surface=operational_pyramid(depth,camera))
    ta = dict(target,surface=audit.pyramid(depth,camera));sb = dict(source,surface=audit.pyramid(depth,camera))
    layers = [];changed = 0
    for scale in (4,2,1):
        old = operational.select(target,source,before,before,scale)
        control = frozen.independent_matches([ta],sb,before,scale,False)
        require({k:v for k,v in old.items() if k not in ('points','pixel_ids')} == {k:v for k,v in control.items() if k != 'points'},'unchanged matcher count and hash')
        for shift in (0.,.02):
            moved = before.copy();moved[0,3] = shift
            new = operational.select(target,source,before,moved,scale)
            oa = oracle.independent_matches([ta],sb,before,scale,False)
            ob = oracle.independent_matches([ta],sb,moved,scale,False)
            result = operational.transition(old,new,before,before,before,moved)
            compare(result,oracle.transition(oa,ob,before,before,before,moved))
            if shift == 0:require(result['stable_pairs'] == old['count'] and not result['retargeted_sources'],'identity preserves every pair')
            else:changed += result['removed_pairs']+result['added_pairs']
            layers.append(dict(scale=scale,translation_m=shift,before_count=result['before_count'],after_count=result['after_count'],stable_pairs=result['stable_pairs'],retargeted_sources=result['retargeted_sources']))
    require(changed > 0,'rendered movement changes pixel-pair identities')
    summary = operational.aggregate([actual,gate],-.01,.02)
    compare(summary,oracle.aggregate([expected,gate],-.01,.02))
    rejected = []
    changes = [('wrong stable count',lambda r:r.update(stable_pairs=198)),
        ('lost retargeting',lambda r:r.update(retargeted_sources=0)),
        ('wrong sample denominator',lambda r:r.update(before_count=201)),
        ('ignored gate transition',lambda r:r.update(after_admitted=False)),
        ('wrong removed cost sign',lambda r:r.update(removed_cost_at_after=-r['removed_cost_at_after'])),
        ('wrong reassociation delta',lambda r:r.update(association_delta=0.)),
        ('wrong pixel digest',lambda r:r.update(after_pixel_sha256='0'*64))]
    for name,change in changes:
        bad = copy.deepcopy(actual);change(bad)
        try:compare(bad,expected)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected turnover corruption: '+name)
    bad = copy.deepcopy(summary);bad['prior_after'] += .1
    try:compare(bad,oracle.aggregate([expected,gate],-.01,.02))
    except (ValueError,AssertionError):rejected.append('wrong prior cost')
    else:raise ValueError('undetected prior-cost corruption')
    duplicate = copy.deepcopy(a);duplicate['pixel_ids'][1] = duplicate['pixel_ids'][0]
    duplicate['pixel_sha256'] = sha(duplicate['pixel_ids'].tobytes())
    for function in (operational.transition,oracle.transition):
        try:function(duplicate,a,*([before]*4))
        except ValueError:pass
        else:raise ValueError('duplicate reciprocal pixel accepted')
    return dict(analytic_pass=True,source_proof=proof,manual_equal_population_counts=dict(stable=197,retargeted=1,disappeared=2,appeared=2),
        admitted_factor_birth_cost=.0016,admitted_factor_death_cost=-.0016,rendered_layers=layers,
        raw_aggregation_with_prior_checked=True,duplicate_pixel_ids_rejected_by_both=True,rejected_corruptions=rejected)


def operational_pyramid(depth,camera):
    from patch_depth_normals import pyramid
    return pyramid(depth,camera)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in
        ('check-depth-correspondence.py','depth_correspondence_stability.py','depth_correspondence_oracle.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
