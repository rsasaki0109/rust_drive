"""Pixel-pair turnover and frozen/re-associated Huber cost accounting."""
import numpy as np
from projective_depth_common import require,sha


def select(target,source,target_pose,source_pose,scale):
    stride = {4:1,2:2,1:4}[scale]
    a,b = target['surface'][scale],source['surface'][scale]
    xyz = b['vertices'][::stride,::stride].reshape(-1,3)
    normal = b['normals'][::stride,::stride].reshape(-1,3)
    y,x = np.indices(b['depth'].shape)
    pixels = np.column_stack([x[::stride,::stride].ravel(),y[::stride,::stride].ravel()])
    keep = np.sum(normal*normal,axis=1) > .25
    xyz,normal,pixels = xyz[keep],normal[keep],pixels[keep]
    source_normals = normal.copy()
    q = xyz@source_pose[:3,:3].T+source_pose[:3,3]
    normal = normal@source_pose[:3,:3].T
    local = (q-target_pose[:3,3])@target_pose[:3,:3]
    ids = np.flatnonzero(local[:,2] > .3)
    c = a['camera']
    u = np.rint(c['fx']*local[ids,0]/local[ids,2]+c['cx'])
    v = np.rint(c['fy']*local[ids,1]/local[ids,2]+c['cy'])
    inside = (u >= 1)&(u < c['width']-1)&(v >= 1)&(v < c['height']-1)
    ids,u,v = ids[inside],u[inside].astype(int),v[inside].astype(int)
    n = a['normals'][v,u];nr = n@target_pose[:3,:3].T
    valid = ((np.sum(n*n,axis=1) > .25)
             &(np.linalg.norm(local[ids]-a['vertices'][v,u],axis=1) < (.12 if scale == 4 else .08))
             &(np.sum(normal[ids]*nr,axis=1) > .8))
    ids,nr,u,v = ids[valid],nr[valid],u[valid],v[valid]
    point = a['vertices'][v,u]@target_pose[:3,:3].T+target_pose[:3,3]
    back = (point-source_pose[:3,3])@source_pose[:3,:3]
    front = np.flatnonzero(back[:,2] > .3);camera = b['camera']
    returned = np.column_stack([np.rint(camera['fx']*back[front,0]/back[front,2]+camera['cx']),
                                np.rint(camera['fy']*back[front,1]/back[front,2]+camera['cy'])])
    selected = front[np.all(returned == pixels[ids[front]],axis=1)]
    source_ids = pixels[ids[selected],1]*b['camera']['width']+pixels[ids[selected],0]
    target_ids = v[selected]*a['camera']['width']+u[selected]
    indices = np.column_stack([target_ids,source_ids]).astype('<i8')
    return dict(points=(a['vertices'][v[selected],u[selected]],a['normals'][v[selected],u[selected]],
        xyz[ids[selected]],source_normals[ids[selected]]),count=len(selected),pixel_sha256=sha(indices.tobytes()),pixel_ids=indices)


def losses(item,target,source):
    pt,nt,ps,ns = item['points']
    difference = ps@source[:3,:3].T+source[:3,3]-pt@target[:3,:3].T-target[:3,3]
    normal = nt@target[:3,:3].T+ns@source[:3,:3].T
    length = np.linalg.norm(normal,axis=1)
    require(np.all(length > 1e-12),'nondegenerate diagnostic normals')
    residual = np.sum(normal*difference,axis=1)/length
    a = abs(residual)
    return np.where(a <= .01,.5*residual**2,.01*(a-.005))


def valid_ids(item):
    ids = item['pixel_ids']
    require(ids.shape == (item['count'],2) and ids.dtype.kind in 'iu','pixel-pair array shape/type')
    require(np.all(ids >= 0),'nonnegative pixel IDs')
    require(all(len(np.unique(ids[:,i])) == len(ids) for i in (0,1)),'reciprocal pixels are one-to-one')
    require(item['pixel_sha256'] == sha(ids.astype('<i8').tobytes()),'honest pixel-pair digest')


def transition(before,after,old_target,old_source,new_target,new_source):
    for item in (before,after):valid_ids(item)
    old = {tuple(row):i for i,row in enumerate(before['pixel_ids'])}
    new = {tuple(row):i for i,row in enumerate(after['pixel_ids'])}
    common = old.keys()&new.keys()
    old_sources = {s:t for t,s in old};new_sources = {s:t for t,s in new}
    shared_sources = old_sources.keys()&new_sources.keys()
    stable = len(common)
    retargeted = int(sum(old_sources[s] != new_sources[s] for s in shared_sources))
    old_admitted,new_admitted = before['count'] >= 200,after['count'] >= 200
    admitted_common = common if old_admitted and new_admitted else set()
    moved = losses(before,new_target,new_source);fresh = losses(after,new_target,new_source)
    old_loss = losses(before,old_target,old_source)
    removed = [old[k] for k in old.keys()-admitted_common] if old_admitted else []
    added = [new[k] for k in new.keys()-admitted_common] if new_admitted else []
    # Equal IDs name the same measured point/normal tuple, even after movement.
    for k in common:
        require(abs(moved[old[k]]-fresh[new[k]]) <= 1e-12,'stable pair has the same moved loss')
    raw_before = float(old_loss.sum()) if old_admitted else 0.
    raw_fixed = float(moved.sum()) if old_admitted else 0.
    raw_fresh = float(fresh.sum()) if new_admitted else 0.
    dropped = float(moved[removed].sum());entered = float(fresh[added].sum())
    require(abs((raw_fresh-raw_fixed)-(entered-dropped)) <= 1e-10,'association-cost partition')
    return dict(before_count=before['count'],after_count=after['count'],stable_pairs=stable,
        retargeted_sources=retargeted,disappeared_sources=len(old_sources.keys()-new_sources.keys()),
        appeared_sources=len(new_sources.keys()-old_sources.keys()),
        removed_pairs=before['count']-stable,added_pairs=after['count']-stable,
        before_admitted=old_admitted,after_admitted=new_admitted,
        admitted_stable_pairs=len(admitted_common),admitted_removed_pairs=len(removed),admitted_added_pairs=len(added),
        before_pixel_sha256=before['pixel_sha256'],after_pixel_sha256=after['pixel_sha256'],
        raw_before=raw_before,raw_fixed_after=raw_fixed,raw_reassociated_after=raw_fresh,
        removed_cost_at_after=dropped,added_cost_at_after=entered,
        association_delta=raw_fresh-raw_fixed)


def aggregate(edges,prior_before,prior_after):
    counts = ('before_count','after_count','stable_pairs','retargeted_sources','disappeared_sources','appeared_sources',
        'removed_pairs','added_pairs','admitted_stable_pairs','admitted_removed_pairs','admitted_added_pairs')
    result = {k:sum(e[k] for e in edges) for k in counts}
    for key in ('raw_before','raw_fixed_after','raw_reassociated_after','removed_cost_at_after','added_cost_at_after'):
        result[key] = sum(e[key] for e in edges)
    result.update(prior_before=prior_before,prior_after=prior_after,
        factor_gate_changes=sum(e['before_admitted'] != e['after_admitted'] for e in edges))
    old = result['raw_before']+prior_before;fixed = result['raw_fixed_after']+prior_after
    fresh = result['raw_reassociated_after']+prior_after
    result.update(objective_before=old,objective_fixed_after=fixed,objective_reassociated_after=fresh,
        fixed_step_delta=fixed-old,association_delta=fresh-fixed,total_delta=fresh-old)
    return result
