"""Frozen reciprocal correspondences and bounded Armijo pose updates."""
import numpy as np

import reciprocal_keyframe_depth as base
from projective_depth_common import exp_rotation, require, sha


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
        xyz[ids[selected]],source_normals[ids[selected]]),count=len(selected),pixel_sha256=sha(indices.tobytes()))


def frozen(nodes,poses,scale):
    result = []
    for source in range(1,len(nodes)):
        for target in range(source):
            data = select(nodes[target],nodes[source],poses[target],poses[source],scale)
            if data['count'] >= 200:result.append(dict(data,target=target,source=source))
    return result


def energy(matches,nodes,poses,prior):
    cost = 0.
    for item in matches:
        t,s = poses[item['target']],poses[item['source']]
        pt,nt,ps,ns = item['points']
        difference = ps@s[:3,:3].T+s[:3,3]-pt@t[:3,:3].T-t[:3,3]
        normal = nt@t[:3,:3].T+ns@s[:3,:3].T
        length = np.linalg.norm(normal,axis=1)
        require(np.all(length > 1e-12),'frozen normal sum')
        r = np.einsum('ij,ij->i',difference,normal)/length
        a = abs(r);cost += float(np.where(a <= .01,.5*r*r,.01*(a-.005)).sum())
    if prior is not None:
        ids = [n['timestamp'] for n in nodes]
        delta = np.concatenate([base.chart(poses[ids.index(stamp)],anchor)[0]
            for stamp,anchor in zip(prior.ids,prior.anchors,strict=True)])
        cost += float(delta@prior.h@delta/2+prior.g@delta)
    require(np.isfinite(cost),'finite frozen objective')
    return cost


def search(nodes,poses,scale,prior,delta,gradient):
    matches = frozen(nodes,poses,scale);before = energy(matches,nodes,poses,prior)
    directional = float(np.asarray(gradient).ravel()@delta.ravel())
    require(np.isfinite(directional) and directional <= 0,'nonascending linear proposal')
    record = dict(before_cost=before,directional_derivative=directional,
        matches=[{k:v for k,v in x.items() if k != 'points'} for x in matches],trials=[])
    for trial in range(8):
        alpha = 2.**(-trial);candidate = [p.copy() for p in poses]
        for node,d in enumerate(delta,1):
            inc = np.eye(4);inc[:3,:3],inc[:3,3] = exp_rotation(alpha*d[3:]),alpha*d[:3]
            candidate[node] = inc@candidate[node];base.valid_pose(candidate[node])
        cost = energy(matches,nodes,candidate,prior)
        limit = before+1e-4*alpha*directional+1e-12*max(1.,abs(before))
        record['trials'].append(dict(alpha=alpha,cost=cost,limit=limit))
        if cost <= limit:
            record['accepted_alpha'] = alpha
            return candidate,record
    raise ValueError('line search')
