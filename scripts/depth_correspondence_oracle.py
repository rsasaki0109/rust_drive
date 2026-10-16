"""Separate relative-camera selection and array-set turnover oracle."""
import numpy as np
from projective_depth_common import require,sha


def independent_matches(references,source,pose,scale,enabled):
    require(len(references) == 1 and not enabled,'one geometric pair without RGB')
    ref = references[0];a,b = ref['surface'][scale],source['surface'][scale]
    stride = {4:1,2:2,1:4}[scale];pixels = np.arange(b['depth'].size).reshape(b['depth'].shape)[::stride,::stride].ravel()
    xyz,normals = b['vertices'].reshape(-1,3)[pixels],b['normals'].reshape(-1,3)[pixels]
    ok = np.linalg.norm(normals,axis=1) > .5;xyz,normals,pixels = xyz[ok],normals[ok],pixels[ok]
    relative = np.linalg.solve(ref['root'],pose)
    q = xyz@relative[:3,:3].T+relative[:3,3]
    sn = normals@relative[:3,:3].T
    ids = np.flatnonzero(q[:,2] > .3);c = a['camera']
    k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    image = q[ids]@k.T;uv = np.round(image[:,:2]/image[:,2,None])
    inside = ((uv >= 1).all(axis=1)&(uv[:,0] < c['width']-1)&(uv[:,1] < c['height']-1))
    ids,uv = ids[inside],uv[inside].astype(int);u,v = uv.T
    point,tn = a['vertices'][v,u],a['normals'][v,u]
    ok = (np.linalg.norm(tn,axis=1) > .5)
    ok &= np.sqrt(np.sum((q[ids]-point)**2,axis=1)) < (.12 if scale == 4 else .08)
    ok &= np.einsum('ij,ij->i',sn[ids],tn) > .8
    ids,point,tn = ids[ok],point[ok],tn[ok]
    forward_count = len(ids)
    inverse = np.linalg.inv(relative)
    back = point@inverse[:3,:3].T+inverse[:3,3]
    front = np.flatnonzero(back[:,2] > .3)
    camera = b['camera'];matrix = np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1.]])
    projected = back[front]@matrix.T
    returned = np.round(projected[:,:2]/projected[:,2,None])
    expected = np.column_stack([pixels[ids[front]]%camera['width'],pixels[ids[front]]//camera['width']])
    chosen = front[np.logical_and.reduce(returned == expected,axis=1)]
    ids,point,tn = ids[chosen],point[chosen],tn[chosen]
    chosen_uv = uv[ok][chosen]
    target_ids = chosen_uv[:,1]*a['camera']['width']+chosen_uv[:,0]
    indices = np.column_stack([target_ids,pixels[ids]]).astype('<i8')
    return dict(points=(point,tn,xyz[ids],normals[ids]),count=len(ids),pixel_sha256=sha(indices.tobytes()),pixel_ids=indices)


def losses(item,target,source):
    pt,nt,ps,ns = item['points']
    relative = np.linalg.solve(target,source)
    difference = ps@relative[:3,:3].T+relative[:3,3]-pt
    summed = nt+ns@relative[:3,:3].T
    r = np.einsum('ij,ij->i',difference,summed)/np.sqrt(np.einsum('ij,ij->i',summed,summed))
    result = np.empty(len(r));quadratic = abs(r) <= .01
    result[quadratic] = .5*r[quadratic]*r[quadratic]
    result[~quadratic] = .01*abs(r[~quadratic])-.00005
    return result


def transition(before,after,old_target,old_source,new_target,new_source):
    structured = []
    for item in (before,after):
        ids = np.asarray(item['pixel_ids'])
        require(ids.shape == (item['count'],2) and ids.dtype.kind in ('i','u') and np.all(ids >= 0),'oracle pixel shape/type/range')
        require(np.unique(ids[:,0]).size == len(ids) == np.unique(ids[:,1]).size,'oracle reciprocal uniqueness')
        require(sha(ids.astype('<i8').tobytes()) == item['pixel_sha256'],'oracle pixel digest')
        structured.append(np.ascontiguousarray(ids,dtype='<i8').view([('target','<i8'),('source','<i8')]).ravel())
    common,i,j = np.intersect1d(*structured,assume_unique=True,return_indices=True)
    source_common = len(np.intersect1d(before['pixel_ids'][:,1],after['pixel_ids'][:,1],assume_unique=True))
    n,m = before['count'],after['count'];old_ok,new_ok = n >= 200,m >= 200
    old_loss = losses(before,old_target,old_source);moved = losses(before,new_target,new_source)
    fresh = losses(after,new_target,new_source)
    np.testing.assert_allclose(moved[i],fresh[j],atol=1e-12,rtol=0)
    old_mask,new_mask = np.ones(n,dtype=bool),np.ones(m,dtype=bool)
    if old_ok and new_ok:old_mask[i] = False;new_mask[j] = False
    if not old_ok:old_mask[:] = False
    if not new_ok:new_mask[:] = False
    a = float(old_loss.sum())*old_ok;b = float(moved.sum())*old_ok;c = float(fresh.sum())*new_ok
    return dict(before_count=n,after_count=m,stable_pairs=len(common),retargeted_sources=source_common-len(common),
        disappeared_sources=n-source_common,appeared_sources=m-source_common,removed_pairs=n-len(common),added_pairs=m-len(common),
        before_admitted=old_ok,after_admitted=new_ok,admitted_stable_pairs=len(common) if old_ok and new_ok else 0,
        admitted_removed_pairs=int(old_mask.sum()),admitted_added_pairs=int(new_mask.sum()),
        before_pixel_sha256=before['pixel_sha256'],after_pixel_sha256=after['pixel_sha256'],
        raw_before=a,raw_fixed_after=b,raw_reassociated_after=c,
        removed_cost_at_after=float(moved[old_mask].sum()),added_cost_at_after=float(fresh[new_mask].sum()),association_delta=c-b)


def prior_value(prior,nodes,poses,audit):
    if prior is None:return 0.
    timestamps = [n['timestamp'] for n in nodes]
    d = np.hstack([audit.independent_chart(poses[timestamps.index(t)],np.asarray(anchor))[0]
        for t,anchor in zip(prior['ids'],prior['anchors'],strict=True)])
    return float(np.einsum('i,ij,j',d,np.asarray(prior['h']),d)/2+np.dot(d,prior['g']))


def aggregate(edges,prior_before,prior_after):
    count_keys = ('before_count','after_count','stable_pairs','retargeted_sources','disappeared_sources','appeared_sources',
        'removed_pairs','added_pairs','admitted_stable_pairs','admitted_removed_pairs','admitted_added_pairs')
    values = {k:int(np.sum([e[k] for e in edges])) for k in count_keys}
    for key in ('raw_before','raw_fixed_after','raw_reassociated_after','removed_cost_at_after','added_cost_at_after'):
        values[key] = float(np.sum([e[key] for e in edges]))
    values['prior_before'],values['prior_after'] = prior_before,prior_after
    values['factor_gate_changes'] = sum(int(e['before_admitted']) != int(e['after_admitted']) for e in edges)
    values['objective_before'] = values['raw_before']+prior_before
    values['objective_fixed_after'] = values['raw_fixed_after']+prior_after
    values['objective_reassociated_after'] = values['raw_reassociated_after']+prior_after
    values['fixed_step_delta'] = values['objective_fixed_after']-values['objective_before']
    values['association_delta'] = values['raw_reassociated_after']-values['raw_fixed_after']
    values['total_delta'] = values['objective_reassociated_after']-values['objective_before']
    return values
