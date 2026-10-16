"""Independent fixed-population Huber objective, prior and line-search audit."""
import numpy as np
from projective_depth_common import ROOT, fresh, require, sha, write
import argparse
import copy
import importlib.util
from pathlib import Path
import descent_keyframe_depth as kernel


def cross_matrices(v):
    # Each matrix maps w to v cross w, independent of the factor's cross-product J.
    result = np.zeros((len(v),3,3))
    result[:,0,1],result[:,0,2] = -v[:,2],v[:,1]
    result[:,1,0],result[:,1,2] = v[:,2],-v[:,0]
    result[:,2,0],result[:,2,1] = -v[:,1],v[:,0]
    return result


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
    return dict(points=(point,tn,xyz[ids],normals[ids]),count=len(ids),pixel_sha256=sha(indices.tobytes()))


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def selection(nodes,poses,scale):
    matches = []
    for source in range(1,len(nodes)):
        for target in range(source):
            a = dict(nodes[target],root=poses[target])
            data = independent_matches([a],nodes[source],poses[source],scale,False)
            if data['count'] >= 200:matches.append(dict(data,target=target,source=source))
    return matches


def objective(matches,nodes,poses,prior,audit):
    cost = 0.
    for item in matches:
        pt,nt,ps,ns = item['points']
        relative = np.linalg.solve(poses[item['target']],poses[item['source']])
        q = ps@relative[:3,:3].T+relative[:3,3];normal = ns@relative[:3,:3].T+nt
        length = np.sqrt(np.sum(normal*normal,axis=1));require(np.all(length > 1e-12),'independent normal sum')
        r = np.sum((q-pt)*normal,axis=1)/length
        quadratic = abs(r) <= .01
        cost += float(np.dot(r[quadratic],r[quadratic])/2+.01*np.sum(abs(r[~quadratic])-.005))
    if prior is not None:
        ids = [n['timestamp'] for n in nodes]
        delta = np.hstack([audit.independent_chart(poses[ids.index(stamp)],np.asarray(anchor))[0]
            for stamp,anchor in zip(prior['ids'],prior['anchors'],strict=True)])
        cost += float(.5*np.einsum('i,ij,j',delta,np.asarray(prior['h']),delta)+np.dot(delta,prior['g']))
    require(np.isfinite(cost),'independent finite objective')
    return cost


def expected_search(nodes,poses,scale,prior,d,g,audit):
    matches = selection(nodes,poses,scale);before = objective(matches,nodes,poses,prior,audit)
    directional = float(np.dot(g,d.ravel()));require(np.isfinite(directional) and directional <= 0,'independent nonascending proposal')
    result = dict(before_cost=before,directional_derivative=directional,
        matches=[{k:v for k,v in x.items() if k != 'points'} for x in matches],trials=[])
    for exponent in range(8):
        alpha = 1/(2**exponent);candidate = np.asarray(poses).copy()
        for node,delta in enumerate(d,1):candidate[node] = audit.increment(poses[node],delta*alpha)
        cost = objective(matches,nodes,candidate,prior,audit)
        limit = before+.0001*alpha*directional+1e-12*max(1,abs(before))
        result['trials'].append(dict(alpha=alpha,cost=cost,limit=limit))
        if cost <= limit:
            result['accepted_alpha'] = alpha
            return candidate,result
    return None,result


def compare_search(actual,wanted):
    require(actual['matches'] == wanted['matches'],'fixed pair identities and sample hashes')
    require(actual['accepted_alpha'] == wanted['accepted_alpha'],'first independently admissible step length')
    require(len(actual['trials']) == len(wanted['trials']),'complete attempted step lengths')
    for field in ('before_cost','directional_derivative'):
        np.testing.assert_allclose(actual[field],wanted[field],atol=1e-10,rtol=1e-8)
    for a,b in zip(actual['trials'],wanted['trials'],strict=True):
        require(a['alpha'] == b['alpha'],'prescribed halving sequence')
        np.testing.assert_allclose([a['cost'],a['limit']],[b['cost'],b['limit']],atol=1e-10,rtol=1e-8)


def auditor():
    audit = load('descent_reciprocal_audit','check-reciprocal-keyframes.py').auditor();audit.kernel = kernel
    original_step,original_failure = audit.audit_step,audit.verify_failed_fit
    def step(nodes,prior,item):
        if 'skip_reason' in item:
            require('line_search' not in item,'skipped coarse system has no applied line search')
            return original_step(nodes,prior,item)
        poses = np.asarray(item['before']);e = audit.independent_equation(nodes,poses,item['scale'],prior)
        audit.compare_equation(item['equation'],e)
        d,condition = audit.independent_solve(e['h'],e['g'])
        require(all(np.linalg.norm(x[:3]) <= .1 and np.linalg.norm(x[3:]) <= .15 for x in d),'unchanged full proposal guard')
        np.testing.assert_allclose(item['delta'],d,atol=1e-9,rtol=1e-7)
        np.testing.assert_allclose(item['condition'],condition,atol=1e-6,rtol=1e-8)
        after,wanted = expected_search(nodes,poses,item['scale'],prior,d,e['g'],audit)
        require(after is not None,'an applied update has an admissible step length')
        compare_search(item['line_search'],wanted)
        np.testing.assert_allclose(item['after'],after,atol=1e-10,rtol=0)
        np.testing.assert_array_equal(item['after'][0],np.eye(4))
        return np.asarray(item['after']).copy()
    def failed(nodes,poses,prior,u):
        if u['rejection'] not in ('line search','nonascending linear proposal'):return original_failure(nodes,poses,prior,u)
        scale = audit.next_level(u['trace']) or 1;e = audit.independent_equation(nodes,poses,scale,prior)
        guard = load('descent_failure_guards','check-selective-keyframes.py')
        reason,d,_ = guard.failure(audit,e,len(nodes)-1);require(reason is None,'line-search rejection follows admissible original guards')
        if u['rejection'] == 'nonascending linear proposal':require(float(e['g']@d.ravel()) > 0,'reproduce nonascending-proposal guard')
        else:
            after,_ = expected_search(nodes,poses,scale,prior,d,e['g'],audit)
            require(after is None,'all eight line-search candidates independently fail')
    audit.audit_step = step;audit.verify_failed_fit = failed
    return audit


def controls(audit):
    import frozen_depth_objective as operational
    camera = audit.rays.CAMERA;poses = np.array([np.eye(4)]*3)
    depth = audit.rays.render(np.eye(4))
    nodes = [dict(timestamp=1.+i*.03,root=p.copy(),surface=operational.base.pyramid(depth,camera)) for i,p in enumerate(poses)]
    independent = [dict(n,surface=audit.pyramid(depth,camera)) for n in nodes]
    prior = kernel.Prior(1.,[1.03],[np.eye(4)],np.eye(6)*1e6,np.array([-10000.,0,0,0,0,0]),[],1)
    data = operational.frozen(nodes,poses,1);expected = selection(independent,poses,1)
    require([{k:v for k,v in x.items() if k != 'points'} for x in data] == [{k:v for k,v in x.items() if k != 'points'} for x in expected],'independent frozen pixel population')
    d = np.zeros((2,6));d[0,0] = .08;g = np.r_[prior.g,np.zeros(6)]
    after,receipt = operational.search(nodes,poses,1,prior,d,g)
    wanted,independent_receipt = expected_search(independent,poses,1,prior.encode(),d,g,audit)
    compare_search(receipt,independent_receipt);np.testing.assert_allclose(after,wanted,atol=1e-12,rtol=0)
    require(receipt['accepted_alpha'] == .125 and len(receipt['trials']) == 4,'real frozen objective exercises three rejected lengths before acceptance')
    exhausted = copy.deepcopy(prior);exhausted.g[0] = -.01
    g = np.r_[exhausted.g,np.zeros(6)]
    try:operational.search(nodes,poses,1,exhausted,d,g)
    except ValueError as error:require(str(error) == 'line search','explicit exhausted-search rejection')
    else:raise ValueError('exhausted search was accepted')
    rejected_after,exhausted_receipt = expected_search(independent,poses,1,exhausted.encode(),d,g,audit)
    require(rejected_after is None and len(exhausted_receipt['trials']) == 8,'independently reproduce all eight rejected lengths')
    # Full objective, including nonlinear prior coordinates, must differentiate
    # to the original raw+prior gradient at the frozen population.
    poses[1] = audit.increment(poses[1],np.array([.013,-.007,.004,.008,-.006,.009]))
    poses[2] = audit.increment(poses[2],np.array([.019,.005,-.003,-.005,.004,-.007]))
    gradient_prior = kernel.Prior(1.,[1.03],[np.eye(4)],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    data = operational.frozen(nodes,poses,1);expected = selection(independent,poses,1)
    e = kernel.equation(nodes,poses,1,gradient_prior)
    audit.compare_equation(kernel.encode_equation(e),audit.independent_equation(independent,poses,1,gradient_prior.encode()))
    numeric = []
    for node in (1,2):
        for axis in range(6):
            perturb = np.zeros(6);perturb[axis] = 1e-6
            plus,minus = poses.copy(),poses.copy()
            plus[node] = audit.increment(poses[node],perturb);minus[node] = audit.increment(poses[node],-perturb)
            vp = objective(expected,independent,plus,gradient_prior.encode(),audit)
            vm = objective(expected,independent,minus,gradient_prior.encode(),audit)
            numeric.append((vp-vm)/2e-6)
    np.testing.assert_allclose(numeric,e['g'],atol=3e-7,rtol=2e-6)
    np.testing.assert_allclose(operational.energy(data,nodes,poses,gradient_prior),objective(expected,independent,poses,gradient_prior.encode(),audit),atol=1e-12,rtol=0)
    # Use an actual fitted rendered update for report-corruption controls.
    subject = kernel.Tracker(camera,'retain');subject.step(1.,depth)
    truth = audit.increment(np.eye(4),np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    update = subject.step(1.03,source);require(update['accepted'],'analytic descent fit accepted')
    raw_nodes = [dict(timestamp=t,root=np.eye(4),surface=audit.pyramid(z,camera)) for t,z in [(1.,depth),(1.03,source)]]
    captured = update['trace'][0];audit.audit_step(raw_nodes,None,captured)
    corruptions = []
    changes = [('wrong step length',lambda s:s['line_search'].update(accepted_alpha=.5)),
        ('wrong objective',lambda s:s['line_search']['trials'][0].update(cost=100.)),
        ('wrong pixel population',lambda s:s['line_search']['matches'][0].update(pixel_sha256='0'*64)),
        ('missing candidate receipt',lambda s:s['line_search']['trials'].pop()),
        ('wrong directional derivative',lambda s:s['line_search'].update(directional_derivative=10.)),
        ('wrong applied pose',lambda s:s['after'][1][0].__setitem__(3,.09))]
    for name,change in changes:
        bad = copy.deepcopy(captured);change(bad)
        try:audit.audit_step(raw_nodes,None,bad)
        except (ValueError,AssertionError):corruptions.append(name)
        else:raise ValueError('undetected search corruption: '+name)
    parent = (ROOT/'scripts/reciprocal_keyframe_depth.py').read_text();child = (ROOT/'scripts/descent_keyframe_depth.py').read_text()
    child = child.replace('from frozen_depth_objective import search\n','')
    for position in (0,1):
        p = parent.split('def fit(',1);c = child.split('def fit(',1)
        require((p[0] == c[0]) if position == 0 else p[1].split('def schur(',1)[1] == c[1].split('def schur(',1)[1],'only fitter and objective import changed')
    return dict(fixed_pixel_population_checked=True,backtracking_control_alpha=receipt['accepted_alpha'],
        backtracking_trials=4,exhausted_search_trials=8,objective_gradient_axes=12,
        gradient_includes_transported_prior=True,rejected_search_corruptions=corruptions,
        only_fitter_and_objective_import_changed=True,
        controlled_prior_is_engineered_not_calibrated=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);audit = auditor()
    extra = controls(audit);report = audit.analytic()
    report.update(descent_controls=extra,source_sha256={'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in
        ('check-descent-keyframes.py','frozen_depth_objective.py','descent_keyframe_depth.py')})
    write(out/'summary.json',report)


if __name__ == '__main__':main()
