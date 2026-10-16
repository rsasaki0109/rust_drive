"""Bounded NumPy depth research with derived marginal factors, outside driving."""
from dataclasses import dataclass
import json

import numpy as np
from subpixel_depth_factors import pair
from projective_depth_common import exp_rotation, require, sha
from projective_depth_raw import pyramid


def skew(v):
    x,y,z = v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def valid_pose(p):
    require(p.shape == (4,4) and np.isfinite(p).all(),'finite homogeneous pose')
    require(np.max(abs(p[3]-[0,0,0,1])) <= 1e-12
            and np.max(abs(p[:3,:3].T@p[:3,:3]-np.eye(3))) <= 1e-8
            and abs(np.linalg.det(p[:3,:3])-1) <= 1e-8,'proper optical pose')


def chart(pose, anchor):
    """Exact coordinates/Jacobian for Exp(w)*anchor + direct left translation."""
    rotation = pose[:3,:3]@anchor[:3,:3].T
    sine_vector = np.array([rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],
                            rotation[1,0]-rotation[0,1]])/2
    sine = np.linalg.norm(sine_vector)
    cosine = np.clip((np.trace(rotation)-1)/2,-1.,1.)
    angle = np.arctan2(sine,cosine)
    require(angle < np.pi-1e-6,'marginal rotation chart boundary')
    w = sine_vector*(angle/sine) if sine > 1e-14 else sine_vector
    v = pose[:3,3]-rotation@anchor[:3,3]
    k = skew(w)
    coefficient = (1-angle/(2*np.tan(angle/2)))/(angle*angle) if angle > 1e-4 else 1/12+angle*angle/720
    jacobian = np.zeros((6,6))
    jacobian[:3,:3] = np.eye(3)
    jacobian[:3,3:] = -skew(v)
    jacobian[3:,3:] = np.eye(3)-k/2+coefficient*(k@k)
    return np.r_[v,w],jacobian


@dataclass
class Prior:
    origin: float
    ids: list
    anchors: list
    h: np.ndarray
    g: np.ndarray
    consumed: list
    generation: int

    def encode(self):
        return dict(origin=self.origin,ids=self.ids,anchors=[a.tolist() for a in self.anchors],
                    h=self.h.tolist(),g=self.g.tolist(),consumed_pairs=[list(x) for x in self.consumed],
                    generation=self.generation)


def prior_digest(prior):
    return sha(json.dumps(None if prior is None else prior.encode(),sort_keys=True,
                          separators=(',',':'),allow_nan=False).encode())


def prior_system(prior,nodes,poses):
    size = 6*(len(nodes)-1)
    h,g = np.zeros((size,size)),np.zeros(size)
    if prior is None:return h,g,[]
    ids = [n['timestamp'] for n in nodes]
    require(prior.origin == ids[0] and len(prior.ids) <= 3 and len(set(prior.ids)) == len(prior.ids),
            'bounded prior in the same first-camera origin')
    require(all(t in ids[1:] for t in prior.ids),'marginal prior references retained nodes')
    size_prior = len(prior.ids)*6
    require(prior.h.shape == (size_prior,size_prior) and prior.g.shape == (size_prior,)
            and np.isfinite(prior.h).all() and np.isfinite(prior.g).all(),'finite marginal system')
    residual = [];j = np.zeros((size_prior,size))
    for k,(stamp,anchor) in enumerate(zip(prior.ids,prior.anchors,strict=True)):
        node = ids.index(stamp)
        delta,jacobian = chart(poses[node],anchor)
        residual.extend(delta)
        j[k*6:k*6+6,(node-1)*6:node*6] = jacobian
    delta = np.asarray(residual)
    h = j.T@prior.h@j
    g = j.T@(prior.h@delta+prior.g)
    return h,g,delta.tolist()


def add_factor(h,g,source,target,hh,gg):
    b = slice((source-1)*6,source*6)
    h[b,b] += hh;g[b] += gg
    if target:
        a = slice((target-1)*6,target*6)
        h[a,a] += hh;g[a] -= gg
        h[a,b] -= hh;h[b,a] -= hh


def equation(nodes,poses,scale,prior,incident=None):
    h,g,delta = prior_system(prior,nodes,poses)
    edges = [];count = 0;squared = 0.
    history = set() if prior is None else set(prior.consumed)
    for source in range(1,len(nodes)):
        for target in range(source):
            ids = (nodes[target]['timestamp'],nodes[source]['timestamp'])
            if incident is not None and not set(ids)&incident:continue
            n,ss,hh,gg = pair(nodes[target],nodes[source],poses[target],poses[source],scale)
            if n < 200:continue
            require(ids not in history,'marginalized raw factor cannot be counted again')
            edges.append(dict(target=target,source=source,count=n,rms_m=float(np.sqrt(ss/n))))
            count += n;squared += ss
            add_factor(h,g,source,target,hh,gg)
    return dict(h=h,g=g,edges=edges,rms_m=float(np.sqrt(squared/count)) if count else 0.,
                count=count,prior_delta=delta)


def solve(h,g):
    diagonal = np.sqrt(np.diag(h))
    require(np.isfinite(h).all() and np.isfinite(g).all() and np.all(diagonal > 0),'zero/nonfinite diagonal')
    scaled = h/diagonal[:,None]/diagonal[None,:]
    eigen = np.linalg.eigvalsh(scaled)
    require(eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6,'condition')
    factor = np.linalg.cholesky(scaled)
    right = -g/diagonal
    result = np.linalg.solve(factor.T,np.linalg.solve(factor,right))/diagonal
    return result,float(eigen[-1]/eigen[0])


def encode_equation(e):
    return {k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in e.items()}


def fit(nodes,prior,trace):
    require(2 <= len(nodes) <= 5,'bounded pose window')
    poses = [n['root'].copy() for n in nodes]
    for p in poses:valid_pose(p)
    require(np.array_equal(poses[0],np.eye(4)),'fixed first-camera origin')
    require(prior is None or nodes[-1]['timestamp'] not in prior.ids,'no prior on an unobserved current frame')
    for scale,cap in [(4,10),(2,7),(1,5)]:
        for _ in range(cap):
            e = equation(nodes,poses,scale,prior)
            require(any(x['source'] == len(nodes)-1 for x in e['edges']),'new frame needs measured support')
            d,condition = solve(e['h'],e['g']);d = d.reshape(-1,6)
            require(all(np.linalg.norm(x[:3]) <= .1 and np.linalg.norm(x[3:]) <= .15 for x in d),'increment')
            before = [p.tolist() for p in poses]
            for node,delta in enumerate(d,1):
                inc = np.eye(4);inc[:3,:3],inc[:3,3] = exp_rotation(delta[3:]),delta[:3]
                poses[node] = inc@poses[node]
                valid_pose(poses[node])
            trace.append(dict(scale=scale,before=before,equation=encode_equation(e),delta=d.tolist(),
                              condition=condition,after=[p.tolist() for p in poses]))
            if all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d):break
    final = equation(nodes,poses,1,prior)
    require(any(x['source'] == len(nodes)-1 for x in final['edges']),'new frame needs measured support')
    require(final['rms_m'] <= .03,'returned-pose global residual')
    _,condition = solve(final['h'],final['g'])
    final['condition'] = condition
    return poses,encode_equation(final)


def schur(h,g,removed,kept):
    """Eliminate only the removed principal block, without matrix inversion."""
    ee,ek = h[np.ix_(removed,removed)],h[np.ix_(removed,kept)]
    diagonal = np.sqrt(np.diag(ee))
    require(np.all(diagonal > 0),'marginal zero diagonal')
    a = ee/diagonal[:,None]/diagonal[None,:]
    eigen = np.linalg.eigvalsh(a)
    require(eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6,'marginal condition')
    factor = np.linalg.cholesky(a)
    rhs = np.column_stack([ek,g[removed]])/diagonal[:,None]
    solved = np.linalg.solve(factor.T,np.linalg.solve(factor,rhs))/diagonal[:,None]
    output_h = h[np.ix_(kept,kept)]-ek.T@solved[:,:-1]
    output_g = g[kept]-ek.T@solved[:,-1]
    output_h = (output_h+output_h.T)/2
    eig = np.linalg.eigvalsh(output_h)
    require(eig[0] >= -1e-10*max(1.,abs(eig[-1])),'marginal PSD failure')
    require(np.isfinite(output_h).all() and np.isfinite(output_g).all(),'finite marginal result')
    return output_h,output_g,float(eigen[-1]/eigen[0])


def marginalize(nodes,poses,keep_ids,prior,receipt):
    ids = [n['timestamp'] for n in nodes]
    require(ids[0] in keep_ids and ids[-1] in keep_ids,'retain first and current')
    removed_ids = [t for t in ids if t not in keep_ids]
    receipt.update(eliminated_ids=removed_ids,kept_ids=keep_ids)
    if not removed_ids:
        receipt['consumed_pairs'] = []
        return prior
    e = equation(nodes,poses,1,prior,set(removed_ids))
    removed = [k for node,t in enumerate(ids[1:]) if t in removed_ids for k in range(6*node,6*node+6)]
    kept = [k for node,t in enumerate(ids[1:]) if t not in removed_ids for k in range(6*node,6*node+6)]
    new_pairs = [(ids[x['target']],ids[x['source']]) for x in e['edges']]
    history = ([] if prior is None else prior.consumed)+new_pairs
    require(len(set(history)) == len(history) <= 2000,'unique bounded marginalized factors')
    receipt.update(input_equation=encode_equation(e),removed_columns=removed,kept_columns=kept,
                   consumed_pairs=[list(x) for x in new_pairs])
    h,g,condition = schur(e['h'],e['g'],removed,kept)
    result_ids = [t for t in ids[1:] if t not in removed_ids]
    anchors = [p.copy() for t,p in zip(ids,poses) if t in result_ids]
    require(len(result_ids) <= 3,'bounded retained prior')
    receipt['eliminated_condition'] = condition
    return Prior(ids[0],result_ids,anchors,h,g,history,1 if prior is None else prior.generation+1)


class Tracker:
    def __init__(self,camera,mode):
        require(mode in ('discard','retain'),'explicit marginalization mode')
        require(32 <= camera['width'] <= 640 and 32 <= camera['height'] <= 480
                and camera['width']%4 == camera['height']%4 == 0,'bounded camera shape')
        require(all(np.isfinite(camera[k]) for k in ('fx','fy','cx','cy'))
                and 20 <= camera['fx'] <= 5000 and 20 <= camera['fy'] <= 5000
                and 0 <= camera['cx'] < camera['width'] and 0 <= camera['cy'] < camera['height'],'pinhole intrinsics')
        self.camera = dict(camera);self.mode = mode
        self.cache = {};self.keys = [];self.latest = None;self.prior = None
        self.observed = None;self.count = 0;self.lost = False

    def step(self,stamp,depth):
        require(self.count < 200,'at most200 acquisitions')
        self.count += 1
        clock_ok = isinstance(stamp,(int,float)) and not isinstance(stamp,bool) and np.isfinite(stamp) and 0 <= stamp <= 1e12
        out = dict(timestamp=stamp if clock_ok else None,accepted=False,trace=[],stage='clock',
                   prior_before_sha256=prior_digest(self.prior))
        try:
            require(clock_ok and (self.observed is None or stamp > self.observed),'invalid/duplicate depth clock')
            self.observed = stamp
            if self.latest is not None and stamp-self.latest > .2:self.lost = True
            require(not self.lost,'accepted pose expiry latched')
            out['stage'] = 'depth'
            require(isinstance(depth,np.ndarray) and depth.shape == (self.camera['height'],self.camera['width'])
                    and depth.dtype.kind in 'fiu'
                    and np.isfinite(depth).all() and np.all((depth >= 0)&(depth <= 100)),'bounded measured depth')
            current = dict(timestamp=stamp,surface=pyramid(depth,self.camera),
                           root=np.eye(4) if self.latest is None else self.cache[self.latest]['root'].copy())
            if self.latest is None:
                require(np.sum(np.sum(current['surface'][4]['normals']**2,axis=-1) > .25) >= 200,'initial surface support')
                self.cache = {stamp:current};self.keys = [stamp];self.latest = stamp
                out.update(accepted=True,nodes=[stamp],root=np.eye(4).tolist(),prior_commit=None,stage='initialized')
            else:
                ids = self.keys+([] if self.latest in self.keys else [self.latest])+[stamp]
                nodes = [self.cache[t] for t in ids[:-1]]+[current]
                out.update(nodes=ids,stage='fit')
                poses,final = fit(nodes,self.prior,out['trace'])
                out.update(final_equation=final,attempted_roots=[p.tolist() for p in poses],stage='displacement')
                require(np.linalg.norm((np.linalg.inv(self.cache[self.latest]['root'])@poses[-1])[:3,3]) <= .5,'accepted displacement')
                relative = np.linalg.inv(poses[ids.index(self.keys[-1])])@poses[-1]
                angle = np.arccos(np.clip((np.trace(relative[:3,:3])-1)/2,-1,1))
                replace = np.linalg.norm(relative[:3,3]) > .1 or angle > .1
                keys = [self.keys[0],*self.keys[1:][-1:],stamp] if replace else list(self.keys)
                keep = keys+([] if stamp in keys else [stamp])
                next_prior = None
                if self.mode == 'retain':
                    out.update(stage='marginalization',marginalization={})
                    next_prior = marginalize(nodes,poses,keep,self.prior,out['marginalization'])
                # Commit only after fitting, final admission and elimination all succeed.
                updated = {t:dict(n,root=p.copy()) for t,n,p in zip(ids,nodes,poses) if t in keep}
                self.cache,self.keys,self.latest,self.prior = updated,keys,stamp,next_prior
                out.update(accepted=True,stage='committed',root=poses[-1].tolist(),revised_roots=[p.tolist() for p in poses],
                           keyframe_replaced=bool(replace),prior_commit=None if self.prior is None else self.prior.encode())
        except ValueError as error:
            out['rejection'] = str(error)
        out.update(lost=self.lost,last_accepted_timestamp=self.latest,prior_after_sha256=prior_digest(self.prior))
        return out
