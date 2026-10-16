"""Optional NumPy joint-keyframe pose research. No driving or Rust default integration."""
import numpy as np
from projective_depth_common import exp_rotation, require


def pair(target, source, target_pose, source_pose, scale):
    stride = {4:1,2:2,1:4}[scale]
    a,b = target['surface'][scale],source['surface'][scale]
    xyz = b['vertices'][::stride,::stride].reshape(-1,3)
    normal = b['normals'][::stride,::stride].reshape(-1,3)
    keep = np.sum(normal*normal,axis=1) > .25
    xyz,normal = xyz[keep],normal[keep]
    q = xyz@source_pose[:3,:3].T+source_pose[:3,3]
    normal = normal@source_pose[:3,:3].T
    local = (q-target_pose[:3,3])@target_pose[:3,:3]
    ids = np.flatnonzero(local[:,2] > .3)
    c = a['camera']
    u = np.rint(c['fx']*local[ids,0]/local[ids,2]+c['cx'])
    v = np.rint(c['fy']*local[ids,1]/local[ids,2]+c['cy'])
    inside = (u >= 1)&(u < c['width']-1)&(v >= 1)&(v < c['height']-1)
    ids,u,v = ids[inside],u[inside].astype(int),v[inside].astype(int)
    n = a['normals'][v,u]; nr = n@target_pose[:3,:3].T
    valid = ((np.sum(n*n,axis=1) > .25)
             &(np.linalg.norm(local[ids]-a['vertices'][v,u],axis=1) < (.12 if scale == 4 else .08))
             &(np.sum(normal[ids]*nr,axis=1) > .8))
    ids,n,nr,u,v = ids[valid],n[valid],nr[valid],u[valid],v[valid]
    r = np.sum(n*(local[ids]-a['vertices'][v,u]),axis=1)
    j = np.column_stack([nr,np.cross(q[ids],nr)])
    w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
    return len(r),float(r@r),j.T@(w[:,None]*j),j.T@(w*r)


def equation(nodes, poses, scale, mode):
    variable = list(range(1,len(nodes))) if mode == 'joint' else [len(nodes)-1]
    slots = {node:slice(6*k,6*k+6) for k,node in enumerate(variable)}
    h,g = np.zeros((len(variable)*6,len(variable)*6)),np.zeros(len(variable)*6)
    edges = []; squared = 0.; count = 0; graph = [set() for _ in nodes]
    for source in range(1,len(nodes)):
        for target in range(source):
            n,ss,hh,gg = pair(nodes[target],nodes[source],poses[target],poses[source],scale)
            if n < 200:
                continue
            edges.append(dict(target=target,source=source,count=n,rms_m=float(np.sqrt(ss/n))))
            graph[target].add(source);graph[source].add(target)
            count += n;squared += ss
            a,b = slots.get(target),slots.get(source)
            if a is not None:h[a,a] += hh;g[a] -= gg
            if b is not None:h[b,b] += hh;g[b] += gg
            if a is not None and b is not None:h[a,b] -= hh;h[b,a] -= hh
    connected = {0}
    while True:
        updated = connected.union(*(graph[k] for k in connected))
        if updated == connected:break
        connected = updated
    if len(connected) != len(nodes):raise ValueError('disconnected measured constraint graph')
    return h,g,edges,float(np.sqrt(squared/count)),variable


def fit(nodes, mode, trace=None):
    require(mode in ('fixed','joint') and 2 <= len(nodes) <= 5,'bounded, explicitly selected optimizer')
    poses = [np.asarray(n['root'],dtype=float).copy() for n in nodes]
    for pose in poses:
        require(pose.shape == (4,4) and np.isfinite(pose).all(),'finite 4x4 optical pose')
        require(np.max(abs(pose[3]-[0,0,0,1])) <= 1e-12,'homogeneous pose')
        require(np.max(abs(pose[:3,:3].T@pose[:3,:3]-np.eye(3))) <= 1e-8
                and abs(np.linalg.det(pose[:3,:3])-1) <= 1e-8,'proper rigid pose')
    require(np.array_equal(poses[0],np.eye(4)),'first optical pose fixed exactly')
    require(all(np.isfinite(n['timestamp']) and 0 <= n['timestamp'] <= 1e12 for n in nodes),'finite bounded node clocks')
    require(all(b['timestamp'] > a['timestamp'] for a,b in zip(nodes,nodes[1:])),'unique chronological node clocks')
    if trace is None:trace = []
    for scale,cap in [(4,10),(2,7),(1,5)]:
        for _ in range(cap):
            h,g,edges,rms,variable = equation(nodes,poses,scale,mode)
            diagonal = np.sqrt(np.diag(h))
            if not np.isfinite(diagonal).all() or np.any(diagonal <= 0):raise ValueError('zero diagonal')
            a = h/diagonal[:,None]/diagonal[None,:];eig = np.linalg.eigvalsh(a)
            if eig[0] <= 0 or eig[-1]/eig[0] > 1e6:raise ValueError('condition')
            d = (np.linalg.solve(a,-g/diagonal)/diagonal).reshape(-1,6)
            if any(np.linalg.norm(x[:3]) > .1 or np.linalg.norm(x[3:]) > .15 for x in d):raise ValueError('increment')
            before = [p.tolist() for p in poses]
            for node,delta in zip(variable,d):
                inc = np.eye(4);inc[:3,:3],inc[:3,3] = exp_rotation(delta[3:]),delta[:3]
                poses[node] = inc@poses[node]
            trace.append(dict(scale=scale,edges=edges,rms_m=rms,h=h.tolist(),g=g.tolist(),condition=float(eig[-1]/eig[0]),
                variable_nodes=variable,before=before,delta=d.tolist(),after=[p.tolist() for p in poses]))
            if all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d):break
    _,_,edges,rms,_ = equation(nodes,poses,1,mode)
    if rms > .03:raise ValueError('returned-pose global residual')
    return poses,trace,dict(edges=edges,rms_m=rms)


