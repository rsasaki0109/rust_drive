#!/usr/bin/env python3
"""Independent QR/profile, quaternion-chart and raw depth checks for marginal factors."""
import argparse
import copy
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import marginalized_keyframe_depth as kernel
from multireference_rgbd_common import raw_equation
from projective_depth_common import fresh, require, sha, write
from projective_depth_raw import pyramid


def imported(name,path):
    spec = importlib.util.spec_from_file_location(name,path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

rays = imported('rays',ROOT/'scripts/check-projective-depth.py')


def cross_matrix(v):
    return np.cross(np.tile(v,(3,1)),np.eye(3)).T


def increment(pose,d):
    # Skew-square Rodrigues form, separate from the operational outer-product helper.
    w = d[3:];angle = np.linalg.norm(w);k = cross_matrix(w)
    a = np.sin(angle)/angle if angle > 1e-12 else 1.
    b = (1-np.cos(angle))/(angle*angle) if angle > 1e-8 else .5-angle*angle/24
    e = np.eye(4);e[:3,:3] = np.eye(3)+a*k+b*(k@k);e[:3,3] = d[:3]
    return e@pose


def quaternion(rotation):
    # Largest-component construction avoids the operational skew-vector log.
    candidates = np.array([1+np.trace(rotation),1+rotation[0,0]-rotation[1,1]-rotation[2,2],
        1-rotation[0,0]+rotation[1,1]-rotation[2,2],1-rotation[0,0]-rotation[1,1]+rotation[2,2]])
    k = int(np.argmax(candidates));q = np.zeros(4);q[k] = np.sqrt(max(0.,candidates[k]))/2
    s = 4*q[k]
    if k == 0:q[1:] = [rotation[2,1]-rotation[1,2],rotation[0,2]-rotation[2,0],rotation[1,0]-rotation[0,1]];q[1:] /= s
    else:
        i = k-1;j = (i+1)%3;l = (i+2)%3
        q[0] = (rotation[l,j]-rotation[j,l])/s
        q[j+1] = (rotation[j,i]+rotation[i,j])/s
        q[l+1] = (rotation[l,i]+rotation[i,l])/s
    q /= np.linalg.norm(q)
    if q[0] < 0:q = -q
    return q


def independent_chart(pose,anchor):
    relative = pose[:3,:3]@anchor[:3,:3].T
    q = quaternion(relative);v = q[1:];s = np.linalg.norm(v)
    angle = 2*np.arctan2(s,q[0])
    require(angle < np.pi-1e-6,'chart boundary')
    rotation_vector = angle*v/s if s > 1e-14 else 2*v
    # Quaternion rotation of the anchor translation, without the operational R*t expression.
    t0 = anchor[:3,3]
    rotated = t0+2*q[0]*np.cross(v,t0)+2*np.cross(v,np.cross(v,t0))
    translation = pose[:3,3]-rotated
    j = np.zeros((6,6));j[:3,:3] = np.eye(3);j[:3,3:] = -cross_matrix(translation)
    if s > 1e-10:
        u = v/s
        dv = (angle/s)*(np.eye(3)-np.outer(u,u))+2*q[0]*np.outer(u,u)
        j[3:,3:] = dv@(.5*(q[0]*np.eye(3)-cross_matrix(v)))+np.outer(v,v)
    else:
        j[3:,3:] = np.eye(3)-cross_matrix(rotation_vector)/2
    return np.r_[translation,rotation_vector],j


def independent_prior(prior,nodes,poses):
    size = (len(nodes)-1)*6
    if prior is None:return np.zeros((size,size)),np.zeros(size),[]
    ids = [n['timestamp'] for n in nodes]
    require(prior['origin'] == ids[0] and len(set(prior['ids'])) == len(prior['ids']) <= 3,'same bounded prior origin')
    require(len(set(map(tuple,prior['consumed_pairs']))) == len(prior['consumed_pairs']),'unique factor history')
    coordinates = [];blocks = []
    for stamp,anchor in zip(prior['ids'],prior['anchors'],strict=True):
        require(stamp in ids[1:],'prior node retained')
        d,j = independent_chart(poses[ids.index(stamp)],np.asarray(anchor));coordinates.extend(d)
        selection = np.zeros((1,len(nodes)-1));selection[0,ids.index(stamp)-1] = 1
        blocks.append(j@np.kron(selection,np.eye(6)))
    matrix = np.vstack(blocks);d = np.asarray(coordinates)
    return matrix.T@np.asarray(prior['h'])@matrix,matrix.T@(np.asarray(prior['h'])@d+prior['g']),d.tolist()


def independent_equation(nodes,poses,scale,prior,incident=None):
    h,g,delta = independent_prior(prior,nodes,poses)
    edges = [];count = 0;squared = 0.
    history = set() if prior is None else set(map(tuple,prior['consumed_pairs']))
    for b in range(1,len(nodes)):
        for a in range(b):
            pair_id = (nodes[a]['timestamp'],nodes[b]['timestamp'])
            if incident is not None and not set(pair_id)&incident:continue
            target = dict(nodes[a],root=poses[a],gray=None,phase=0.)
            source = dict(nodes[b],gray=None,phase=0.)
            e = raw_equation([target],source,poses[b],scale,False)['geometry']
            if e['count'] < 200:continue
            require(pair_id not in history,'no duplicated marginal/active factor')
            incidence = np.array([int(i == b)-int(i == a) for i in range(1,len(nodes))])
            matrix = np.kron(incidence[None,:],np.eye(6))
            h += matrix.T@e['h']@matrix;g += matrix.T@e['g']
            edges.append(dict(target=a,source=b,count=e['count'],rms_m=e['rms_m']))
            count += e['count'];squared += e['count']*e['rms_m']**2
    return dict(h=h,g=g,edges=edges,count=count,rms_m=float(np.sqrt(squared/count)) if count else 0.,prior_delta=delta)


def compare_equation(actual,expected):
    require(actual['count'] == expected['count'],'raw support count')
    require([(e['target'],e['source'],e['count']) for e in actual['edges']]
            == [(e['target'],e['source'],e['count']) for e in expected['edges']],'raw factor identity/support')
    np.testing.assert_allclose(actual['h'],expected['h'],atol=1e-6,rtol=1e-8)
    np.testing.assert_allclose(actual['g'],expected['g'],atol=1e-8,rtol=1e-8)
    np.testing.assert_allclose(actual['rms_m'],expected['rms_m'],atol=1e-10,rtol=0)
    np.testing.assert_allclose(actual['prior_delta'],expected['prior_delta'],atol=1e-10,rtol=0)


def independent_solve(h,g):
    scale = np.sqrt(np.diag(h));a = h/scale[:,None]/scale[None,:]
    eigen,vec = np.linalg.eigh(a)
    require(np.all(scale > 0) and eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6,'supported combined system')
    return (vec@((vec.T@(-g/scale))/eigen)/scale).reshape(-1,6),float(eigen[-1]/eigen[0])


def independent_schur(h,g,removed,kept):
    scale = np.sqrt(np.diag(h)[removed]);ee = h[np.ix_(removed,removed)]/scale[:,None]/scale[None,:]
    q,r = np.linalg.qr(ee)
    rhs = np.column_stack([h[np.ix_(removed,kept)],g[removed]])/scale[:,None]
    solved = np.linalg.solve(r,q.T@rhs)/scale[:,None]
    return (h[np.ix_(kept,kept)]-h[np.ix_(kept,removed)]@solved[:,:-1],
            g[kept]-h[np.ix_(kept,removed)]@solved[:,-1])


def audit_step(nodes,prior,step):
    before,after = np.asarray(step['before']),np.asarray(step['after'])
    expected = independent_equation(nodes,before,step['scale'],prior)
    compare_equation(step['equation'],expected)
    d,condition = independent_solve(expected['h'],expected['g'])
    np.testing.assert_allclose(step['condition'],condition,atol=1e-6,rtol=1e-8)
    np.testing.assert_allclose(step['delta'],d,atol=1e-9,rtol=1e-7)
    wanted = before.copy()
    for node,delta in enumerate(np.asarray(step['delta']),1):
        require(np.linalg.norm(delta[:3]) <= .1 and np.linalg.norm(delta[3:]) <= .15,'bounded node update')
        wanted[node] = increment(before[node],delta)
    np.testing.assert_allclose(after,wanted,atol=1e-11,rtol=0)
    np.testing.assert_array_equal(after[0],np.eye(4))
    return after


def audit_marginalization(nodes,poses,prior,receipt,output):
    ids = [n['timestamp'] for n in nodes];removed_ids = receipt['eliminated_ids'];keep_ids = receipt['kept_ids']
    require(removed_ids == [t for t in ids if t not in keep_ids] and ids[0] in keep_ids and ids[-1] in keep_ids,
            'only expired window nodes eliminated')
    if not removed_ids:
        require(receipt['consumed_pairs'] == [] and output == prior,'no elimination duplicates no factors')
        return
    e = independent_equation(nodes,poses,1,prior,set(removed_ids))
    compare_equation(receipt['input_equation'],e)
    removed = [i for k,t in enumerate(ids[1:]) if t in removed_ids for i in range(6*k,6*k+6)]
    kept = [i for k,t in enumerate(ids[1:]) if t not in removed_ids for i in range(6*k,6*k+6)]
    require(receipt['removed_columns'] == removed and receipt['kept_columns'] == kept,'exact eliminated blocks')
    h,g = independent_schur(e['h'],e['g'],removed,kept)
    scale = np.sqrt(np.diag(e['h'])[removed])
    eig = np.linalg.eigvalsh(e['h'][np.ix_(removed,removed)]/scale[:,None]/scale[None,:])
    np.testing.assert_allclose(receipt['eliminated_condition'],eig[-1]/eig[0],atol=1e-6,rtol=1e-8)
    np.testing.assert_allclose(output['h'],h,atol=1e-6,rtol=1e-8)
    np.testing.assert_allclose(output['g'],g,atol=1e-8,rtol=1e-8)
    pair_ids = [[ids[x['target']],ids[x['source']]] for x in e['edges']]
    require(receipt['consumed_pairs'] == pair_ids,'only incident raw factors consumed')
    history = ([] if prior is None else prior['consumed_pairs'])+pair_ids
    require(output['consumed_pairs'] == history and len(set(map(tuple,history))) == len(history),'factor history without double counting')
    require(output['origin'] == ids[0] and output['ids'] == [t for t in ids[1:] if t in keep_ids],
            'same fixed origin and retained prior identities')
    np.testing.assert_allclose(output['anchors'],[p for t,p in zip(ids,poses) if t in output['ids']],atol=0,rtol=0)
    require(output['generation'] == (1 if prior is None else prior['generation']+1),'one marginal generation')


def digest(prior):
    return sha(json.dumps(prior,sort_keys=True,separators=(',',':'),allow_nan=False).encode())


def next_level(trace):
    if not trace:return 4
    levels = [4,2,1];caps = {4:10,2:7,1:5};last = trace[-1];level = last['scale']
    d = np.asarray(last['delta'])
    done = (sum(s['scale'] == level for s in trace) == caps[level]
            or all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d))
    return (levels[levels.index(level)+1] if level != 1 else None) if done else level


def verify_failed_fit(nodes,poses,prior,row):
    level = next_level(row['trace']);reason = row['rejection']
    if reason == 'marginal rotation chart boundary':
        try:independent_prior(prior,nodes,poses)
        except ValueError:return
        raise ValueError('false chart-domain rejection')
    e = independent_equation(nodes,poses,level or 1,prior)
    if reason == 'new frame needs measured support':
        require(not any(x['source'] == len(nodes)-1 for x in e['edges']),'reproduce absent current support')
    elif reason in ('zero/nonfinite diagonal','condition'):
        h = e['h'];diagonal = np.diag(h)
        valid = np.isfinite(h).all() and np.isfinite(e['g']).all() and np.all(diagonal > 0)
        if valid:
            s = np.sqrt(diagonal);eig = np.linalg.eigvalsh(h/s[:,None]/s[None,:])
            valid = eig[0] > 0 and eig[-1]/eig[0] <= 1e6
        require(not valid,'reproduce rejected combined observability')
    elif reason == 'increment':
        d,_ = independent_solve(e['h'],e['g'])
        require(any(np.linalg.norm(x[:3]) > .1 or np.linalg.norm(x[3:]) > .15 for x in d),'reproduce oversized increment')
    elif reason == 'returned-pose global residual':
        require(level is None and e['rms_m'] > .03,'reproduce final raw residual rejection')
    else:raise ValueError('unexpected fit rejection: '+reason)


def recorded(rows,frames,verified,mode,camera):
    import io
    from PIL import Image
    require(len(rows) == len(frames) == 180,'complete selected acquisitions')
    cache = {};keys = [];latest = None;prior = None;lost = False
    systems = accepted = rejected = generations = 0;reasons = {};consumed = 0
    for frame,entry in zip(frames,rows):
        stamp = frame['depth_timestamp'];u = entry['update']
        require(entry['source_index'] == frame['source_index'] and u['timestamp'] == stamp,'source acquisition identity')
        require(u['prior_before_sha256'] == digest(prior),'unmodified prior between acquisitions')
        if latest is not None:lost |= stamp-latest > .2
        require(u['lost'] == lost and not (lost and u['accepted']),'terminal expiry')
        if lost:
            require(not u['trace'] and 'nodes' not in u and 'root' not in u
                    and u['rejection'] == 'accepted pose expiry latched','expired tracker does no synthetic work')
        else:
            depth = np.asarray(Image.open(io.BytesIO(verified(frame['depth_file']))),dtype=float)/5000
            current = dict(timestamp=stamp,surface=pyramid(depth,camera),root=np.eye(4) if latest is None else cache[latest]['root'].copy())
            if latest is None:
                require(u['accepted'] and not u['trace'] and u['nodes'] == [stamp],'one initialization')
                require(np.sum(np.sum(current['surface'][4]['normals']**2,axis=-1) > .25) >= 200,'initial measured support')
                np.testing.assert_array_equal(u['root'],np.eye(4))
                require(u['prior_commit'] is None,'no authored initial prior')
                cache = {stamp:current};keys = [stamp];latest = stamp
            else:
                ids = keys+([] if latest in keys else [latest])+[stamp]
                require(u['nodes'] == ids and len(ids) <= 5,'bounded deduplicated raw window')
                require(prior is None or stamp not in prior['ids'],'no prior on the new observation')
                nodes = [cache[t] for t in ids[:-1]]+[current]
                poses = np.asarray([n['root'] for n in nodes])
                levels = [s['scale'] for s in u['trace']]
                require(len(levels) <= 22 and levels == sorted(levels,reverse=True)
                        and all(s in (4,2,1) for s in levels)
                        and all(levels.count(s) <= c for s,c in [(4,10),(2,7),(1,5)]),'bounded pyramid work')
                for step in u['trace']:
                    np.testing.assert_allclose(step['before'],poses,atol=1e-12,rtol=0)
                    poses = audit_step(nodes,prior,step);systems += 1
                if u['accepted'] or u['stage'] != 'fit':
                    compare_equation(u['final_equation'],independent_equation(nodes,poses,1,prior));systems += 1
                    _,condition = independent_solve(np.asarray(u['final_equation']['h']),np.asarray(u['final_equation']['g']))
                    np.testing.assert_allclose(u['final_equation']['condition'],condition,atol=1e-6,rtol=1e-8)
                    require(u['final_equation']['rms_m'] <= .03,'accepted returned-pose raw residual')
                    np.testing.assert_allclose(u['attempted_roots'],poses,atol=1e-12,rtol=0)
                if u['accepted']:
                    require(bool(u['trace']) and np.linalg.norm((np.linalg.inv(cache[latest]['root'])@poses[-1])[:3,3]) <= .5,
                            'accepted measured displacement')
                    np.testing.assert_allclose(u['root'],poses[-1],atol=1e-12,rtol=0)
                    np.testing.assert_allclose(u['revised_roots'],poses,atol=1e-12,rtol=0)
                    delta = np.linalg.inv(poses[ids.index(keys[-1])])@poses[-1]
                    angle = np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
                    replace = np.linalg.norm(delta[:3,3]) > .1 or angle > .1
                    require(u['keyframe_replaced'] == replace,'motion key selection at revised poses')
                    keep_keys = [keys[0],*keys[1:][-1:],stamp] if replace else list(keys)
                    keep = keep_keys+([] if stamp in keep_keys else [stamp])
                    if mode == 'retain':
                        require(u['marginalization']['kept_ids'] == keep,'same prescribed eviction policy')
                        audit_marginalization(nodes,poses,prior,u['marginalization'],u['prior_commit'])
                        if u['marginalization']['eliminated_ids']:
                            generations += 1;systems += 1;consumed += len(u['marginalization']['consumed_pairs'])
                    else:require(u['prior_commit'] is None and 'marginalization' not in u,'discard control has no retained information')
                    cache = {t:dict(n,root=p.copy()) for t,n,p in zip(ids,nodes,poses) if t in keep}
                    keys,latest,prior = keep_keys,stamp,u['prior_commit'];accepted += 1
                else:
                    require('root' not in u and 'revised_roots' not in u and 'prior_commit' not in u,'failed attempt cannot commit')
                    if u['stage'] == 'fit':verify_failed_fit(nodes,poses,prior,u)
                    elif u['stage'] == 'displacement':
                        require(np.linalg.norm((np.linalg.inv(cache[latest]['root'])@poses[-1])[:3,3]) > .5,'reproduce displacement rejection')
                    elif u['stage'] == 'marginalization':
                        receipt = u['marginalization'];removed_ids = set(receipt['eliminated_ids'])
                        e = independent_equation(nodes,poses,1,prior,removed_ids)
                        compare_equation(receipt['input_equation'],e);systems += 1
                        rr,kk = receipt['removed_columns'],receipt['kept_columns']
                        ee = e['h'][np.ix_(rr,rr)];diag = np.diag(ee);valid = np.all(diag > 0)
                        if valid:
                            scale = np.sqrt(diag);eigen = np.linalg.eigvalsh(ee/scale[:,None]/scale[None,:])
                            valid = eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6
                        if u['rejection'] in ('marginal zero diagonal','marginal condition'):require(not valid,'reproduce rejected elimination')
                        elif u['rejection'] == 'marginal PSD failure':
                            h,_ = independent_schur(e['h'],e['g'],rr,kk);eigen = np.linalg.eigvalsh((h+h.T)/2)
                            require(eigen[0] < -1e-10*max(1.,abs(eigen[-1])),'reproduce rejected Schur PSD')
                        else:raise ValueError('unexpected marginal rejection: '+u['rejection'])
                    else:raise ValueError('unexpected acquisition stage: '+u['stage'])
                    rejected += 1
        if not u['accepted']:reasons[u['rejection']] = reasons.get(u['rejection'],0)+1
        require(u['last_accepted_timestamp'] == latest and u['prior_after_sha256'] == digest(prior),'atomic clock/prior commit')
    return dict(pass_integrity=True,checked_reported_systems=systems,accepted_updates=accepted,
                reproduced_fit_rejections=rejected,marginal_generations=generations,consumed_unique_raw_factors=consumed,
                rejections=reasons,lost=lost,ground_truth_parsed=False)


def analytic():
    rng = np.random.default_rng(8041)
    a = rng.normal(size=(80,18));a[:,6:12] *= .01;b = rng.normal(size=80)
    removed = list(range(6,12));kept = [i for i in range(18) if i not in removed]
    h,g = a.T@a,a.T@b
    hh,gg,_ = kernel.schur(h,g,removed,kept)
    q,_ = np.linalg.qr(a[:,removed],mode='complete');projection = q[:,len(removed):].T
    ak,bp = projection@a[:,kept],projection@b
    np.testing.assert_allclose(hh,ak.T@ak,atol=1e-10,rtol=1e-10)
    np.testing.assert_allclose(gg,ak.T@bp,atol=1e-10,rtol=1e-10)
    def profile(x):
        d = np.linalg.lstsq(a[:,removed],-b-a[:,kept]@x,rcond=None)[0]
        residual = a[:,removed]@d+a[:,kept]@x+b
        return residual@residual/2
    for _ in range(5):
        x = rng.normal(size=len(kept))
        np.testing.assert_allclose(profile(x)-profile(np.zeros(len(kept))),x@hh@x/2+gg@x,atol=1e-9,rtol=1e-10)
    anchor = increment(np.eye(4),np.array([2.,-1.,.5,.2,-.1,.15]))
    chart_cases = []
    for angle in [1e-9,1e-4,.3,1.2]:
        delta = np.array([.02,-.04,.03,angle,angle/3,-angle/4]);pose = increment(anchor,delta)
        actual,j = kernel.chart(pose,anchor);expected,independent_j = independent_chart(pose,anchor)
        np.testing.assert_allclose(actual,delta,atol=1e-12,rtol=0)
        np.testing.assert_allclose(actual,expected,atol=1e-12,rtol=0)
        np.testing.assert_allclose(j,independent_j,atol=1e-10,rtol=0)
        numeric = []
        for k in range(6):
            e = np.zeros(6);e[k] = 1e-6
            plus = independent_chart(increment(pose,e),anchor)[0]
            minus = independent_chart(increment(pose,-e),anchor)[0]
            numeric.append((plus-minus)/2e-6)
        np.testing.assert_allclose(j,np.asarray(numeric).T,atol=2e-9,rtol=0)
        chart_cases.append(angle)
    reports = {};frames = 15;captured = None;captured_step = None
    for mode in ['discard','retain']:
        tracker = kernel.Tracker(rays.CAMERA,mode);cache = {};prior = None;checked = generations = 0;errors = []
        for i in range(frames):
            stamp = 1.+i*.03;truth = increment(np.eye(4),np.array([.04*i,0.,.002*i,0.,.0015*i,0.]))
            depth = rays.render(truth);u = tracker.step(stamp,depth)
            require(u['accepted'],'analytic measured motion: '+str(u.get('rejection')))
            current = dict(timestamp=stamp,surface=pyramid(depth,rays.CAMERA),root=np.eye(4) if not cache else cache[max(cache)]['root'].copy())
            if i:
                nodes = [cache[t] for t in u['nodes'][:-1]]+[current]
                previous = np.asarray([n['root'] for n in nodes])
                for step in u['trace']:
                    np.testing.assert_allclose(step['before'],previous,atol=1e-12,rtol=0)
                    previous = audit_step(nodes,prior,step);checked += 1
                compare_equation(u['final_equation'],independent_equation(nodes,previous,1,prior));checked += 1
                if mode == 'retain':
                    audit_marginalization(nodes,previous,prior,u['marginalization'],u['prior_commit'])
                    generations += bool(u['marginalization']['eliminated_ids'])
                    if u['marginalization']['eliminated_ids']:
                        captured = copy.deepcopy((nodes,previous,prior,u['marginalization'],u['prior_commit']))
                        captured_step = copy.deepcopy((nodes,prior,u['trace'][0]))
                for node,p in zip(nodes,previous):node['root'] = p.copy()
                cache[stamp] = current
            else:cache[stamp] = current
            prior = u['prior_commit']
            cache = {t:cache[t] for t in tracker.cache}
            error = np.linalg.norm(np.asarray(u['root'])[:3,3]-truth[:3,3]);errors.append(float(error))
            angle = np.arccos(np.clip((np.trace(truth[:3,:3].T@np.asarray(u['root'])[:3,:3])-1)/2,-1,1))
            require(error <= .004 and angle <= .004,'analytic optical motion accuracy')
        reports[mode] = dict(checked_reported_equations=checked,marginal_generations=generations,max_position_error_m=max(errors))
        require(mode == 'discard' or generations >= 8,'exercise repeated elimination and key eviction')
        before = kernel.prior_digest(tracker.prior);last = tracker.latest
        bad = tracker.step(last+.03,np.zeros((120,160)))
        require(not bad['accepted'] and bad['last_accepted_timestamp'] == last
                and bad['prior_after_sha256'] == before,'rejected new measurement cannot refresh prior/clock')
        expired = tracker.step(last+.201,np.zeros((120,160)))
        later = tracker.step(last+.21,depth)
        require(expired['lost'] and later['lost'] and not later['accepted'] and not later['trace'],'terminal expiry without reset')
    flat = np.full((120,160),2.)
    t = kernel.Tracker(rays.CAMERA,'retain');require(t.step(1.,flat)['accepted'],'plane initialization')
    require(not t.step(1.03,flat)['accepted'],'prior cannot create unobserved planar motion')
    corruptions = []
    def corrupt(name,change):
        nodes,poses,prior,receipt,result = copy.deepcopy(captured)
        change(nodes,poses,prior,receipt,result)
        try:audit_marginalization(nodes,poses,prior,receipt,result)
        except (ValueError,AssertionError):corruptions.append(name)
        else:raise ValueError('undetected marginal corruption: '+name)
    corrupt('wrong Schur gradient',lambda n,p,b,r,a:a['g'].__setitem__(0,a['g'][0]+1.))
    corrupt('coherently scaled Schur H/g',lambda n,p,b,r,a:a.update(h=(np.asarray(a['h'])*2).tolist(),g=(np.asarray(a['g'])*2).tolist()))
    corrupt('duplicate factor history',lambda n,p,b,r,a:a['consumed_pairs'].append(a['consumed_pairs'][0]))
    corrupt('wrong fixed origin',lambda n,p,b,r,a:a.update(origin=a['origin']+1))
    corrupt('wrong linearization anchor',lambda n,p,b,r,a:a['anchors'][0][0].__setitem__(3,.5))
    def double_count(nodes,poses,before,receipt,result):
        full = independent_equation(nodes,poses,1,before)
        receipt['input_equation'] = kernel.encode_equation(full)
        h,g = independent_schur(full['h'],full['g'],receipt['removed_columns'],receipt['kept_columns'])
        result['h'],result['g'] = h.tolist(),g.tolist()
    corrupt('retained raw factors also marginalized',double_count)
    for name,change in [('normal system scaled without changing its solve',lambda s:s['equation'].update(
        h=(np.asarray(s['equation']['h'])*2).tolist(),g=(np.asarray(s['equation']['g'])*2).tolist())),
        ('moved first origin',lambda s:s['after'][0][0].__setitem__(3,.1)),
        ('wrong normal increment',lambda s:s['delta'][0].__setitem__(0,.09))]:
        nodes,prior,step = copy.deepcopy(captured_step);change(step)
        try:audit_step(nodes,prior,step)
        except (ValueError,AssertionError):corruptions.append(name)
        else:raise ValueError('undetected trace corruption: '+name)
    # Late elimination failure must be as atomic as an early measurement failure.
    subject,clean = kernel.Tracker(rays.CAMERA,'retain'),kernel.Tracker(rays.CAMERA,'retain')
    for i in range(2):
        truth = increment(np.eye(4),np.array([.04*i,0.,0.,0.,0.,0.]));depth = rays.render(truth)
        require(subject.step(1.+i*.03,depth)['accepted'] and clean.step(1.+i*.03,depth)['accepted'],'fault prefix')
    old_prior = kernel.prior_digest(subject.prior);old_roots = {t:n['root'].copy() for t,n in subject.cache.items()}
    saved = kernel.schur
    def injected_failure(*args):raise ValueError('injected elimination failure')
    kernel.schur = injected_failure
    try:
        truth = increment(np.eye(4),np.array([.08,0.,0.,0.,0.,0.]))
        failed = subject.step(1.06,rays.render(truth))
    finally:kernel.schur = saved
    require(not failed['accepted'] and failed['stage'] == 'marginalization' and bool(failed['trace'])
            and failed['last_accepted_timestamp'] == 1.03 and failed['prior_after_sha256'] == old_prior,
            'elimination failure preserves accepted state')
    for t,p in old_roots.items():np.testing.assert_array_equal(subject.cache[t]['root'],p)
    truth = increment(np.eye(4),np.array([.12,0.,0.,0.,0.,0.]));depth = rays.render(truth)
    actual,expected = subject.step(1.09,depth),clean.step(1.09,depth)
    require(actual['accepted'] and expected['accepted'],'fresh recovery after rejected elimination')
    np.testing.assert_allclose(actual['root'],expected['root'],atol=1e-12,rtol=0)
    require(actual['prior_after_sha256'] == expected['prior_after_sha256'],'clean prior parity after omission')
    for stamp in [1.09,1.08,float('nan')]:
        bad = subject.step(stamp,depth)
        require(not bad['accepted'] and bad['last_accepted_timestamp'] == 1.09
                and bad['prior_before_sha256'] == bad['prior_after_sha256'],'bad clock cannot mutate prior')
    try:kernel.schur(np.zeros((12,12)),np.zeros(12),list(range(6)),list(range(6,12)))
    except ValueError:pass
    else:raise ValueError('rank-deficient removed block received fabricated information')
    for bad_depth in [np.full((120,160),'bad',dtype=object),np.full((120,160),complex(2,1))]:
        blank = kernel.Tracker(rays.CAMERA,'retain');bad = blank.step(1.,bad_depth)
        require(not bad['accepted'] and bad['last_accepted_timestamp'] is None,'invalid depth type rejected atomically')
    return dict(analytic_pass=True,profile_equivalence_cases=5,chart_angles_rad=chart_cases,reports=reports,
                discarded_information_control=True,no_fabricated_new_support=True,terminal_expiry=True,
                rejected_corruptions=corruptions,late_elimination_failure_atomic=True,clean_recovery_prior_parity=True,
                malformed_clock_and_depth_rejected=True,singular_elimination_rejected=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    report = analytic()
    report.update(checker_sha256=sha(Path(__file__).read_bytes()),kernel_sha256=sha((ROOT/'scripts/marginalized_keyframe_depth.py').read_bytes()))
    write(out/'summary.json',report)


if __name__ == '__main__':main()
