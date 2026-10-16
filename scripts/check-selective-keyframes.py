#!/usr/bin/env python3
"""Independent proof of coarse skips and strict finest-level retained tracking."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import selective_keyframe_depth as kernel
from projective_depth_common import ROOT, fresh, require, sha, write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def failure(audit,e,current):
    """Spectral solve, independent of the operational Cholesky decision."""
    if not any(x['source'] == current for x in e['edges']):return 'new frame needs measured support',None,None
    diag = np.diag(e['h'])
    if not (np.isfinite(e['h']).all() and np.isfinite(e['g']).all() and np.all(diag > 0)):
        return 'zero/nonfinite diagonal',None,None
    scale = np.sqrt(diag);eigen = np.linalg.eigvalsh(e['h']/scale[:,None]/scale[None,:])
    if eigen[0] <= 0 or eigen[-1]/eigen[0] > 1e6:return 'condition',None,None
    d,condition = audit.independent_solve(e['h'],e['g'])
    if any(np.linalg.norm(x[:3]) > .1 or np.linalg.norm(x[3:]) > .15 for x in d):return 'increment',d,condition
    return None,d,condition


def auditor():
    patch = load('selective_patch_audit','check-patch-keyframes.py');audit = patch.auditor()
    audit.kernel = kernel
    original_step,original_recorded = audit.audit_step,audit.recorded
    def step(nodes,prior,item):
        if 'skip_reason' not in item:return original_step(nodes,prior,item)
        require(item['scale'] in (4,2),'finest level cannot be skipped')
        before = np.asarray(item['before'])
        expected = audit.independent_equation(nodes,before,item['scale'],prior)
        audit.compare_equation(item['equation'],expected)
        reason,d,condition = failure(audit,expected,len(nodes)-1)
        require(reason is not None and item['skip_reason'] == reason,'independently reproduced coarse skip')
        np.testing.assert_array_equal(item['after'],before)
        np.testing.assert_array_equal(item['delta'],np.zeros((len(nodes)-1,6)))
        if d is None:
            require(item['proposed_delta'] is None and item['condition'] is None,'no fabricated coarse proposal')
        else:
            np.testing.assert_allclose(item['proposed_delta'],d,atol=1e-9,rtol=1e-7)
            np.testing.assert_allclose(item['condition'],condition,atol=1e-6,rtol=1e-8)
        return before.copy()
    def schedule(trace,complete=False):
        prefix = []
        for item in trace:
            require(item['scale'] == audit.next_level(prefix),'prescribed next pyramid level')
            require('skip_reason' not in item or item['scale'] in (4,2),'no finest skip')
            prefix.append(item)
        if complete:require(bool(trace) and trace[-1]['scale'] == 1 and audit.next_level(trace) is None,'complete finest iteration budget/convergence')
    def recorded(rows,frames,verified,mode,camera,progress_label=None):
        for row in rows:
            u = row['update']
            schedule(u['trace'],bool(u['trace']) and (u['accepted'] or u['stage'] != 'fit'))
        checked = 0;step_callback = audit.audit_step
        def progress(nodes,prior,item):
            nonlocal checked
            result = step_callback(nodes,prior,item);checked += 1
            if progress_label is not None and checked%1000 == 0:
                print(progress_label,'independent trace systems',checked,flush=True)
            return result
        audit.audit_step = progress
        try:result = original_recorded(rows,frames,verified,mode,camera)
        finally:audit.audit_step = step_callback
        skipped = [s for r in rows for s in r['update']['trace'] if 'skip_reason' in s]
        result['checked_coarse_skips'] = len(skipped)
        result['coarse_skip_reasons'] = {reason:sum(s['skip_reason'] == reason for s in skipped) for reason in sorted({s['skip_reason'] for s in skipped})}
        return result
    audit.audit_step = step;audit.recorded = recorded;audit.schedule = schedule
    return audit


def scheduling_controls(audit):
    patch = load('selective_normal_controls','check-patch-keyframes.py')
    camera = audit.rays.CAMERA
    truth = audit.increment(np.eye(4),np.array([.02,0,.001,0,.001,0]))
    target_depth,source_depth = audit.rays.render(np.eye(4)),audit.rays.render(truth)
    perforated = source_depth.copy();perforated[1::2,1::2] = 0
    subject = kernel.Tracker(camera,'retain')
    import patch_keyframe_depth as parent
    control = parent.Tracker(camera,'retain')
    require(subject.step(1.,target_depth)['accepted'] and control.step(1.,target_depth)['accepted'],'measured initialization')
    u = subject.step(1.03,perforated);old = control.step(1.03,perforated)
    require(u['accepted'] and not old['accepted'] and old['rejection'] == 'new frame needs measured support','fine support rescues actual depth-hole fixture')
    a = dict(timestamp=1.,root=np.eye(4),surface=patch.independent_pyramid(target_depth,camera))
    b = dict(timestamp=1.03,root=np.eye(4),surface=patch.independent_pyramid(perforated,camera))
    nodes = [a,b];poses = np.array([np.eye(4),np.eye(4)])
    for item in u['trace']:
        np.testing.assert_array_equal(item['before'],poses);poses = audit.audit_step(nodes,None,item)
    audit.compare_equation(u['final_equation'],audit.independent_equation(nodes,poses,1,None))
    audit.schedule(u['trace'],True)
    require([s['scale'] for s in u['trace'] if 'skip_reason' in s] == [4,2],'both empty coarse levels skipped exactly once')
    error = float(np.linalg.norm(poses[-1,:3,3]-truth[:3,3]))
    require(error < .004,'perforated measured-motion accuracy')
    # Explicit layer fixtures isolate conditioning and increment guards; these
    # are injected surfaces, not claimed to be outputs of an image decoder.
    normal = np.array([.2,.3,1.]);normal /= np.linalg.norm(normal)
    flat_nodes = [dict(timestamp=t,root=np.eye(4),surface=patch.independent_pyramid(d,camera)) for t,d in [(1.,target_depth),(1.03,source_depth)]]
    for n in flat_nodes:
        for scale in (4,2):n['surface'][scale]['normals'][:] = normal
    trace = [];fitted,final = kernel.fit(flat_nodes,None,trace)
    require([s['skip_reason'] for s in trace if 'skip_reason' in s] == ['condition','condition'],'rank-deficient tilted-normal layers skipped')
    for item in trace:audit.audit_step(flat_nodes,None,item)
    audit.schedule(trace,True);audit.compare_equation(final,audit.independent_equation(flat_nodes,fitted,1,None))
    shifted = [dict(timestamp=t,root=np.eye(4),surface=patch.independent_pyramid(target_depth,camera)) for t in (1.,1.03)]
    for scale in (4,2):shifted[1]['surface'][scale]['vertices'][:,:,2] += .11
    increment_trace = [];fit,final = kernel.fit(shifted,None,increment_trace)
    require(any(s.get('skip_reason') == 'increment' for s in increment_trace),'oversized coarse proposal exercised')
    for item in increment_trace:audit.audit_step(shifted,None,item)
    audit.schedule(increment_trace,True)
    np.testing.assert_allclose(fit[-1],np.eye(4),atol=1e-10,rtol=0)
    audit.compare_equation(final,audit.independent_equation(shifted,fit,1,None))
    rejected = []
    def corrupt(name,nodes,item,change):
        bad = copy.deepcopy(item);change(bad)
        try:audit.audit_step(nodes,None,bad)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected skip corruption: '+name)
    skip = u['trace'][0]
    corrupt('pose moved during coarse skip',nodes,skip,lambda s:s['after'][1][0].__setitem__(3,.2))
    corrupt('skip has nonzero applied increment',nodes,skip,lambda s:s['delta'][0].__setitem__(0,.2))
    corrupt('wrong skip reason',nodes,skip,lambda s:s.update(skip_reason='increment'))
    corrupt('finest skip',nodes,skip,lambda s:s.update(scale=1))
    regular = next(s for s in trace if 'skip_reason' not in s)
    corrupt('skip on admissible measured system',flat_nodes,regular,lambda s:s.update(skip_reason='condition',proposed_delta=None))
    bad = copy.deepcopy(u['trace']);bad[0]['scale'] = 2
    try:audit.schedule(bad)
    except ValueError:rejected.append('unjustified level jump')
    else:raise ValueError('undetected level jump')
    # A finest-level zero-support measurement must remain a failure even when
    # both coarse stages are skipped. Every guard remains active at scale 1.
    zero = [dict(timestamp=t,root=np.eye(4),surface=patch.independent_pyramid(target_depth,camera)) for t in (1.,1.03)]
    for layer in zero[1]['surface'].values():layer['normals'][:] = 0
    ztrace = []
    try:kernel.fit(zero,None,ztrace)
    except ValueError as e:require(str(e) == 'new frame needs measured support','fine support still required')
    else:raise ValueError('finest no-support frame accepted')
    require(len(ztrace) == 2 and all('skip_reason' in s for s in ztrace),'bounded coarse skips before terminal fine rejection')
    # Full-source proof: all retained information, admission and commit code
    # outside fit() is byte-identical to the preceding frozen engine.
    old_text = (ROOT/'scripts/patch_keyframe_depth.py').read_text();new_text = (ROOT/'scripts/selective_keyframe_depth.py').read_text()
    require(old_text.split('def fit(')[0] == new_text.split('def fit(')[0]
        and old_text.split('\ndef schur(')[1] == new_text.split('\ndef schur(')[1],'only fit scheduling changes')
    return dict(perforated_depth_rescued=True,perforated_position_error_m=error,
        controlled_tilted_normal_condition_skips=2,controlled_increment_skips=sum(s.get('skip_reason') == 'increment' for s in increment_trace),
        controlled_surface_equations_checked=len(u['trace'])+len(trace)+len(increment_trace)+3,
        fine_zero_support_still_rejected=True,rejected_corruptions=rejected,parent_only_fit_changed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);audit = auditor()
    controls = scheduling_controls(audit)
    patch = load('shared_patch_controls','check-patch-keyframes.py');normals = patch.normal_controls()
    symmetric = load('shared_symmetric_controls','check-symmetric-keyframes.py');factor = symmetric.factor_controls(audit)
    report = audit.analytic()
    report.update(level_scheduling=controls,patch_normals=normals,symmetric_factor=factor,
        source_sha256={str(p.relative_to(ROOT)):sha(p.read_bytes()) for p in [Path(__file__),ROOT/'scripts/selective_keyframe_depth.py']})
    write(out/'summary.json',report)


if __name__ == '__main__':main()
