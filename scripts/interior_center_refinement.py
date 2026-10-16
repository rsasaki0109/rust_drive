"""Require a two-pixel sampling interior in the unchanged reselection loop."""
import importlib.util

import numpy as np

import reselected_center_refinement as previous
from projective_depth_common import ROOT,require


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


RADIUS_PIXELS = 2
fixed = load('interior_center_private_fixed','fitted_center_refinement.py')


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];t,s = poses[a],poses[b]
    root = item['points']@s[:3,:3].T+s[:3,3];local = (root-t[:3,3])@t[:3,:3]
    camera = nodes[a]['surface'][scale]['camera'];keep = np.ones(len(local),dtype=bool)
    for dv in range(-RADIUS_PIXELS,RADIUS_PIXELS+1):
        for du in range(-RADIUS_PIXELS,RADIUS_PIXELS+1):
            q = local.copy();q[:,0] += du*q[:,2]/camera['fx'];q[:,1] += dv*q[:,2]/camera['fy']
            ids,_,df,_ = fixed.sample(nodes[a]['surface'][scale],q)
            length = np.linalg.norm(df,axis=1)
            require(np.isfinite(length).all() and np.all(length>1e-12),'finite interior sampling gradient')
            supported = np.zeros(len(local),dtype=bool);supported[ids] = True;keep &= supported
    return keep


def selection(nodes,poses,scale):
    seed,stages = previous.fixed.selection(nodes,poses,scale);rows = [];counts = {}
    for item in seed:
        keep = sampling_mask(item,nodes,poses,scale);count = int(keep.sum());counts[item['target'],item['source']] = count
        if count>=200:rows.append(dict(item,**{key:item[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    stages = [dict(s,interior_after_probes=counts.get((s['target'],s['source']),0),
        admitted_interior_count=counts.get((s['target'],s['source']),0) if counts.get((s['target'],s['source']),0)>=200 else 0) for s in stages]
    return rows,stages


def population(nodes,poses,scale):return selection(nodes,poses,scale)[0]


fixed.population,fixed.selection = population,selection
fixed.private.driver.population = population
private = load('interior_center_private_outer','reselected_slope_refinement.py')
private.fixed = fixed
ROUNDS,INNER_CAP = private.ROUNDS,private.INNER_CAP
transition,stop,refine = private.transition,private.stop,private.refine
