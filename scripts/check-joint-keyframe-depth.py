#!/usr/bin/env python3
"""Separate Jacobian, incidence-matrix and analytic-motion audit for the first trial."""
import argparse
import copy
import importlib.util
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from projective_depth_common import exp_rotation, fresh, require, sha, write
from multireference_rgbd_common import raw_equation
from projective_depth_raw import pyramid

def imported(name,path):
    spec = importlib.util.spec_from_file_location(name,path)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
kernel = imported('kernel',ROOT/'scripts/joint_keyframe_depth.py')
rays = imported('rays',ROOT/'scripts/check-projective-depth.py')

def increment(pose,d):
    inc = np.eye(4);inc[:3,:3],inc[:3,3] = exp_rotation(d[3:]),d[:3]
    return inc@pose

def independent(nodes,poses,scale,mode):
    variable = list(range(1,len(nodes))) if mode == 'joint' else [len(nodes)-1]
    size = len(variable)*6;h,g = np.zeros((size,size)),np.zeros(size);edges = [];ss = 0.;count = 0
    for b in range(1,len(nodes)):
        for a in range(b):
            target = dict(nodes[a],gray=None,root=poses[a],phase=0.)
            source = dict(nodes[b],gray=None,phase=0.)
            e = raw_equation([target],source,poses[b],scale,False)['geometry']
            if e['count'] < 200:continue
            incidence = np.array([int(k == b)-int(k == a) for k in variable])
            matrix = np.kron(incidence[None,:],np.eye(6))
            h += matrix.T@e['h']@matrix;g += matrix.T@e['g']
            edges.append(dict(target=a,source=b,count=e['count'],rms_m=e['rms_m']))
            count += e['count'];ss += e['count']*e['rms_m']**2
    return h,g,edges,float(np.sqrt(ss/count)) if count else 0.,variable

def audit(nodes,mode,trace,final,returned):
    require(len(trace) <= 22,'bounded iteration count')
    levels = [s['scale'] for s in trace]
    require(levels == sorted(levels,reverse=True)
            and all(s in (4,2,1) for s in levels)
            and all(levels.count(s) <= cap for s,cap in [(4,10),(2,7),(1,5)]),'fixed pyramid work caps')
    previous = [n['root'] for n in nodes]
    for step in trace:
        before,after = np.asarray(step['before']),np.asarray(step['after'])
        np.testing.assert_allclose(before,previous,atol=1e-12,rtol=0)
        h,g,edges,rms,variable = independent(nodes,before,step['scale'],mode)
        actual_h,actual_g = np.asarray(step['h']),np.asarray(step['g'])
        np.testing.assert_allclose(actual_h,h,atol=1e-7,rtol=1e-10)
        np.testing.assert_allclose(actual_g,g,atol=1e-9,rtol=1e-10)
        require(step['variable_nodes'] == variable,'fixed origin and selected variables')
        require([(x['target'],x['source'],x['count']) for x in edges]
            == [(x['target'],x['source'],x['count']) for x in step['edges']],'raw edge identity/support')
        np.testing.assert_allclose(rms,step['rms_m'],atol=1e-10,rtol=0)
        diagonal = np.sqrt(np.diag(h));scaled = h/diagonal[:,None]/diagonal[None,:]
        eig,vec = np.linalg.eigh(scaled)
        require(eig[0] > 0 and eig[-1]/eig[0] <= 1e6,'anchored SPD conditioning')
        np.testing.assert_allclose(step['condition'],eig[-1]/eig[0],atol=1e-7,rtol=1e-8)
        # Spectral solve, independent of the kernel's direct solve.
        expected = (vec@((vec.T@(-g/diagonal))/eig)/diagonal).reshape(-1,6)
        np.testing.assert_allclose(step['delta'],expected,atol=1e-10,rtol=1e-8)
        changed = before.copy()
        for node,d in zip(variable,np.asarray(step['delta'])):
            require(np.linalg.norm(d[:3]) <= .1 and np.linalg.norm(d[3:]) <= .15,'bounded node increments')
            changed[node] = increment(before[node],d)
        np.testing.assert_allclose(after,changed,atol=1e-11,rtol=0)
        np.testing.assert_allclose(after[0],np.eye(4),atol=0,rtol=0)
        previous = after
    np.testing.assert_allclose(returned,previous,atol=1e-12,rtol=0)
    if final is None:
        return len(trace)
    _,_,edges,rms,_ = independent(nodes,returned,1,mode)
    require(rms <= .03,'returned-pose residual')
    np.testing.assert_allclose(final['rms_m'],rms,atol=1e-10,rtol=0)
    require([(x['target'],x['source'],x['count']) for x in edges]
        == [(x['target'],x['source'],x['count']) for x in final['edges']],'final edge support')
    return len(trace)+1


def connected(edges,size):
    # Transitive closure, separate from the operational set traversal.
    adjacency = np.eye(size,dtype=bool)
    for e in edges:adjacency[e['target'],e['source']] = adjacency[e['source'],e['target']] = True
    for k in range(size):adjacency |= adjacency[:,k,None]&adjacency[k,None,:]
    return bool(adjacency[0].all())


def verify_rejection(nodes,mode,row,attempt,prior):
    trace = row['trace']
    reason = row['rejection']
    levels = [4,2,1];caps = {4:10,2:7,1:5}
    candidate = 4
    if trace:
        last = trace[-1];candidate = last['scale'];d = np.asarray(last['delta'])
        finished = (sum(s['scale'] == candidate for s in trace) == caps[candidate]
                    or all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d))
        if finished:candidate = levels[levels.index(candidate)+1] if candidate != 1 else None
    h,g,edges,rms,_ = independent(nodes,attempt,candidate or 1,mode)
    if reason == 'disconnected measured constraint graph':
        require(not connected(edges,len(nodes)),'reproduce rejected disconnected graph')
    elif reason in ('zero diagonal','condition'):
        diagonal = np.sqrt(np.diag(h))
        valid = np.isfinite(diagonal).all() and np.all(diagonal > 0)
        if valid:
            eig = np.linalg.eigvalsh(h/diagonal[:,None]/diagonal[None,:])
            valid = eig[0] > 0 and eig[-1]/eig[0] <= 1e6
        require(not valid,'reproduce rejected normal-system observability')
    elif reason == 'increment':
        diagonal = np.sqrt(np.diag(h));scaled = h/diagonal[:,None]/diagonal[None,:]
        eig,vec = np.linalg.eigh(scaled)
        d = (vec@((vec.T@(-g/diagonal))/eig)/diagonal).reshape(-1,6)
        require(any(np.linalg.norm(x[:3]) > .1 or np.linalg.norm(x[3:]) > .15 for x in d),
                'reproduce rejected increment')
    elif reason == 'returned-pose global residual':
        require(candidate is None and rms > .03,'reproduce returned-pose residual rejection')
    elif reason == 'accepted displacement':
        require(candidate is None and np.linalg.norm((np.linalg.inv(prior)@attempt[-1])[:3,3]) > .5,
                'reproduce accepted-pose displacement rejection')
    else:
        raise ValueError('unexpected unverified rejection: '+reason)


def recorded(rows,frames,verified,mode,camera):
    import io
    from PIL import Image
    require(len(rows) == len(frames) == 180,'all selected acquisitions retained')
    keys = [];cache = {};latest = None;lost = False
    equations = rejected = accepted = 0
    rejection_reasons = {}
    for frame,row in zip(frames,rows):
        stamp = frame['depth_timestamp']
        require(row['timestamp'] == stamp and row['source_index'] == frame['source_index'],'source acquisition identity')
        if latest is not None:lost |= stamp-latest > .2
        require(row['lost'] == lost and not (lost and row['accepted']),'expiry is terminal')
        if lost:
            require(not row.get('trace') and 'root' not in row and row['last_accepted_timestamp'] == latest,
                    'lost tracking does not fit, reset or refresh')
            rejection_reasons[row['rejection']] = rejection_reasons.get(row['rejection'],0)+1
            continue
        depth = np.asarray(Image.open(io.BytesIO(verified(frame['depth_file']))),dtype=float)/5000
        current = dict(timestamp=stamp,surface=pyramid(depth,camera),
                       root=np.eye(4) if latest is None else cache[latest]['root'].copy())
        if latest is None:
            require(row['accepted'] and not row['trace'] and row['nodes'] == [stamp],'one measured initialization')
            require(np.sum(np.sum(current['surface'][4]['normals']**2,axis=-1) > .25) >= 200,'independent initial support')
            np.testing.assert_array_equal(row['root'],np.eye(4))
            keys = [stamp];cache[stamp] = current;latest = stamp
        else:
            ids = [*keys,*([latest] if latest not in keys else []),stamp]
            require(row['nodes'] == ids and len(ids) <= 5,'deduplicated bounded measured window')
            nodes = [cache[t] for t in ids[:-1]]+[current]
            trace = row['trace']
            attempt = np.asarray(trace[-1]['after']) if trace else np.asarray([n['root'] for n in nodes])
            final = row.get('final_equation') if row['accepted'] else None
            equations += audit(nodes,mode,trace,final,attempt)
            if row['accepted']:
                require(bool(trace),'accepted pose requires fitted measurements')
                np.testing.assert_allclose(row['root'],attempt[-1],atol=1e-12,rtol=0)
                np.testing.assert_allclose(row['revised_roots'],attempt,atol=1e-12,rtol=0)
                require(np.linalg.norm((np.linalg.inv(cache[latest]['root'])@attempt[-1])[:3,3]) <= .5,
                        'accepted displacement bound')
                delta = np.linalg.inv(attempt[ids.index(keys[-1])])@attempt[-1]
                angle = np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
                replace = np.linalg.norm(delta[:3,3]) > .1 or angle > .1
                require(row['keyframe_replaced'] == replace,'key selection from refined reference poses')
                for node,root in zip(nodes,attempt):node['root'] = root.copy()
                cache[stamp] = current
                if replace:keys = [keys[0],*keys[1:][-1:],stamp]
                latest = stamp
                cache = {t:cache[t] for t in {*keys,latest}}
                accepted += 1
            else:
                require('root' not in row and 'revised_roots' not in row and not row.get('keyframe_replaced'),
                        'rejection cannot commit poses or references')
                verify_rejection(nodes,mode,row,attempt,cache[latest]['root'])
                rejection_reasons[row['rejection']] = rejection_reasons.get(row['rejection'],0)+1
                rejected += 1
        require(row['last_accepted_timestamp'] == latest,'accepted clock changes only on success')
    return dict(pass_integrity=True,raw_reconstructed_equations=equations,accepted_updates=accepted,
                independently_reproduced_fit_rejections=rejected,rejections=rejection_reasons,lost=lost,
                ground_truth_parsed=False)

def finite_differences():
    source = increment(np.eye(4),np.array([.08,-.03,.02,.03,-.02,.01]))
    target = increment(np.eye(4),np.array([-.04,.06,.01,-.01,.04,.02]))
    point = np.array([.3,-.2,1.7]);surface = np.array([.2,-.1,1.6]);n = np.array([.2,.1,.9]);n /= np.linalg.norm(n)
    q = source[:3,:3]@point+source[:3,3];nr = target[:3,:3]@n
    source_j = np.r_[nr,np.cross(q,nr)]
    def residual(a,b):
        local = b[:3,:3].T@(a[:3,:3]@point+a[:3,3]-b[:3,3])
        return n@(local-surface)
    derivatives = []
    for which in ['source','target']:
        numeric = []
        for k in range(6):
            d = np.zeros(6);d[k] = 1e-6
            if which == 'source':numeric.append((residual(increment(source,d),target)-residual(increment(source,-d),target))/2e-6)
            else:numeric.append((residual(source,increment(target,d))-residual(source,increment(target,-d)))/2e-6)
        np.testing.assert_allclose(numeric,source_j if which == 'source' else -source_j,atol=1e-9,rtol=0)
        derivatives.append(numeric)
    # Any common left perturbation is a gauge direction before first-pose anchoring.
    j = np.r_[-source_j,source_j][None,:]
    np.testing.assert_allclose((j.T@j)@np.tile(np.eye(6),(2,1)),np.zeros((12,6)),atol=1e-14,rtol=0)
    return dict(source_jacobian=derivatives[0],target_jacobian=derivatives[1],central_difference_step=1e-6)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    output = fresh(args.output)
    derivatives = finite_differences();nodes = [];truth = []
    for i in range(5):
        pose = increment(np.eye(4),np.array([.06*i,.005*i,.002*i,0.,.006*i,.003*i]))
        truth.append(pose)
        guess = pose.copy() if i == 0 else increment(pose,np.array([-.003*i,.008,.001,.002,-.003,.002]))
        nodes.append(dict(timestamp=1.+.03*i,root=guess,surface=pyramid(rays.render(pose),rays.CAMERA)))
    before = [n['root'].copy() for n in nodes];reports = {};corruptions = []
    for mode in ['fixed','joint']:
        poses,trace,final = kernel.fit(nodes,mode)
        checked = audit(nodes,mode,trace,final,poses)
        errors = [float(np.linalg.norm(p[:3,3]-t[:3,3])) for p,t in zip(poses,truth)]
        angles = [float(np.arccos(np.clip((np.trace(t[:3,:3].T@p[:3,:3])-1)/2,-1,1))) for p,t in zip(poses,truth)]
        if mode == 'joint':require(max(errors) <= .004 and max(angles) <= .004,'joint refinement recovers all node poses')
        for a,b in zip(before,[n['root'] for n in nodes]):np.testing.assert_array_equal(a,b)
        reports[mode] = dict(translation_error_m=errors,rotation_error_rad=angles,checked_equations=checked,
            trace=trace,final=final,returned=[p.tolist() for p in poses])
        if mode == 'joint':
            for name,change in [('wrong target/source sign',lambda x:x[0]['delta'][0].__setitem__(0,.09)),
                ('moved first origin',lambda x:x[0]['after'][0][0].__setitem__(3,.1)),
                ('missing reference constraint',lambda x:x[0]['edges'].pop()),
                ('wrong variable set',lambda x:x[0].update(variable_nodes=[4])),
                ('wrong condition',lambda x:x[0].update(condition=1.)),
                ('coherently scaled normal system',lambda x:x[0].update(
                    h=(np.asarray(x[0]['h'])*2).tolist(),g=(np.asarray(x[0]['g'])*2).tolist()))]:
                modified = copy.deepcopy(trace);change(modified)
                try:audit(nodes,mode,modified,final,poses)
                except (ValueError,AssertionError):corruptions.append(name)
                else:raise ValueError('undetected corruption '+name)
    flat = dict(timestamp=2.,root=np.eye(4),surface=pyramid(np.full((120,160),2.),rays.CAMERA))
    for mode in ['fixed','joint']:
        try:kernel.fit([flat,dict(flat,timestamp=2.03)],mode)
        except ValueError:pass
        else:raise ValueError('unobservable plane accepted')
    empty = dict(timestamp=2.03,root=np.eye(4),surface=pyramid(np.zeros((120,160)),rays.CAMERA))
    try:kernel.fit([nodes[0],empty],'joint')
    except ValueError:pass
    else:raise ValueError('disconnected graph accepted')
    invalid_nodes = []
    for name,change in [('unknown mode',lambda x:None),('moved origin',lambda x:x[0]['root'].__setitem__((0,3),.1)),
        ('duplicate clock',lambda x:x[1].update(timestamp=x[0]['timestamp'])),
        ('nonfinite clock',lambda x:x[1].update(timestamp=float('nan'))),
        ('nonrigid pose',lambda x:x[1]['root'].__setitem__((0,0),2.))]:
        bad = [dict(n,root=n['root'].copy()) for n in nodes];change(bad)
        try:kernel.fit(bad,'invalid' if name == 'unknown mode' else 'joint')
        except ValueError:invalid_nodes.append(name)
        else:raise ValueError('invalid node set accepted: '+name)
    write(output/'summary.json',dict(checker_sha256=sha(Path(__file__).read_bytes()),kernel_sha256=sha((ROOT/'scripts/joint_keyframe_depth.py').read_bytes()),analytic_integrity_pass=True,derivatives=derivatives,
        reports=reports,rejected_corruptions=corruptions,unobservable_plane_rejected=True,disconnected_graph_rejected=True,
        source_input_poses_unchanged=True,rejected_invalid_inputs=invalid_nodes,translation_gate_m=.004,rotation_gate_rad=.004))

if __name__ == '__main__':main()
