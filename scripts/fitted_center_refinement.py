"""Retain the legacy center seed and add independently supported C1 centers."""
import importlib.util

import numpy as np

import boundary_slope_refinement as previous
from projective_depth_common import ROOT,require,sha

spec = importlib.util.spec_from_file_location('fitted_center_private_solver',ROOT/'scripts/boundary_slope_refinement.py')
private = importlib.util.module_from_spec(spec);spec.loader.exec_module(private)
base,discrete,metadata,prepare,sample = private.base,private.discrete,private.metadata,private.prepare,private.sample
sampling_mask = private.sampling_mask


def center_seed(item,nodes,poses,scale):
    a,b = item['target'],item['source'];points,normals = item['points'][2:];t,s = poses[a],poses[b]
    world = points@s[:3,:3].T+s[:3,3];local = (world-t[:3,3])@t[:3,:3];source_normals = normals@s[:3,:3].T
    layer = nodes[a]['surface'][scale];bound = .12 if scale==4 else .08
    ids,_,df,target = previous.center.sample(layer,local);length = np.linalg.norm(df,axis=1)
    require(np.isfinite(length).all() and np.all(length>1e-12),'finite legacy center gradient')
    geometry = np.linalg.norm(local[ids]-target,axis=1)<bound
    agreement = np.sum(source_normals[ids]*(df/length[:,None]@t[:3,:3].T),axis=1)>.8
    legacy = np.zeros(len(points),dtype=bool);legacy[ids[geometry&agreement]] = True
    normalizer = np.zeros(len(points));normalizer[ids] = length
    ci,_,cd,ct = sample(layer,local);size = np.linalg.norm(cd,axis=1)
    require(np.isfinite(size).all() and np.all(size>1e-12),'finite fitted center gradient')
    cgeometry = np.linalg.norm(local[ci]-ct,axis=1)<bound
    calignment = np.sum(source_normals[ci]*(cd/size[:,None]@t[:3,:3].T),axis=1)>.8
    fitted = np.zeros(len(points),dtype=bool);fitted[ci[cgeometry&calignment]] = True
    new = fitted&~legacy
    # Legacy normalizers win on the intersection; only new C1-only sources get
    # the C1 gradient-length metric, frozen at these initial poses.
    normalizer[ci[new[ci]]] = size[new[ci]]
    keep = legacy|new
    row = dict(target=a,source=b,points=points[keep],normals=normals[keep],
        source_ids=item['pixel_ids'][keep,1],normalizer=normalizer[keep])
    stages = dict(target=a,source=b,current_source=b==len(nodes)-1,raw_count=len(points),
        bilinear_corner_support=len(ids),bilinear_geometry=int(geometry.sum()),legacy_center_count=int(legacy.sum()),
        fitted_corner_support=len(ci),fitted_geometry=int(cgeometry.sum()),fitted_center_count=int(fitted.sum()),
        fitted_only_center_count=int(new.sum()),union_center_count=int(keep.sum()))
    return row,legacy[keep],stages


def selection(nodes,poses,scale):
    rows = [];stages = []
    for item in discrete.population(nodes,poses,scale):
        row,legacy,stage = center_seed(item,nodes,poses,scale);keep = sampling_mask(row,nodes,poses,scale)
        old_count = int(np.count_nonzero(keep&legacy));new_count = int(keep.sum())
        stage.update(legacy_after_probes=old_count,union_after_probes=new_count,
            admitted_legacy_count=old_count if old_count>=200 else 0,admitted_union_count=new_count if new_count>=200 else 0,
            admitted_fitted_only_count=int(np.count_nonzero(keep&~legacy)) if new_count>=200 else 0,
            fitted_only_source_sha256=sha(np.asarray(row['source_ids'][keep&~legacy] if new_count>=200 else [],dtype='<i8').tobytes()))
        stages.append(stage)
        if new_count>=200:rows.append(dict(row,**{key:row[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    return rows,stages


def population(nodes,poses,scale):return selection(nodes,poses,scale)[0]


private.driver.population = population
values,evaluate,equation,energy,search,refine,snapshot,checkpoint = (
    private.values,private.evaluate,private.equation,private.energy,private.search,private.refine,private.snapshot,private.checkpoint)
