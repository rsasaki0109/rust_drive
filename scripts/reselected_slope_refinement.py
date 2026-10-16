"""Three bounded fixed-source fits with explicit between-round reselection."""
import numpy as np

import slope_admission_refinement as fixed
from projective_depth_common import require,sha

ROUNDS,INNER_CAP = 3,30


def transition(before,after):
    old = {(x['target'],x['source']):x for x in before};new = {(x['target'],x['source']):x for x in after};rows = []
    for pair in sorted(old.keys()|new.keys()):
        a,b = old.get(pair),new.get(pair)
        x = np.array([],dtype=int) if a is None else a['source_ids'];y = np.array([],dtype=int) if b is None else b['source_ids']
        common,i,j = np.intersect1d(x,y,assume_unique=True,return_indices=True)
        ratios = np.array([]);changed = 0
        if len(common):
            for key in ('points','normals'):np.testing.assert_array_equal(a[key][i],b[key][j])
            require(np.all(a['normalizer'][i]>0) and np.all(b['normalizer'][j]>0),'positive metric normalizers')
            ratios = b['normalizer'][j]/a['normalizer'][i];changed = int(np.count_nonzero(b['normalizer'][j]!=a['normalizer'][i]))
        rows.append(dict(target=pair[0],source=pair[1],old_count=len(x),new_count=len(y),shared_count=len(common),
            removed_count=len(x)-len(common),added_count=len(y)-len(common),
            shared_source_sha256=sha(np.asarray(common,dtype='<i8').tobytes()),
            changed_normalizers_on_shared=changed,normalizer_ratio_min=None if not len(ratios) else float(ratios.min()),
            normalizer_ratio_median=None if not len(ratios) else float(np.median(ratios)),
            normalizer_ratio_max=None if not len(ratios) else float(ratios.max()),source_values_preserved=True))
    return rows


def stop(batch,reselected,round_number):
    require(type(round_number) is int and 1<=round_number<=ROUNDS,'bounded association round')
    if batch['guard_failure'] is not None:return 'inner_guard',batch['guard_failure']
    if not batch['continuous_converged']:return 'inner_budget',None
    if reselected['guard_failure'] is not None:return 'selection_guard',reselected['guard_failure']
    if reselected['converged']:return 'converged',None
    if round_number==ROUNDS:return 'round_budget',None
    return None,None


def refine(nodes,initial,prior,scale=1):
    require(2<=len(nodes)<=5 and scale in (4,2,1),'bounded reselected continuous window')
    poses = np.asarray(initial).copy();original = poses.copy();rounds = [];termination = failure = None
    require(poses.shape==(len(nodes),4,4),'one pose per acquisition')
    for pose in poses:fixed.base.valid_pose(pose)
    require(np.array_equal(poses[0],np.eye(4)),'fixed optical origin')
    require(prior is None or nodes[-1]['timestamp'] not in prior.ids,'no prior on new acquisition')
    for round_number in range(1,ROUNDS+1):
        before = fixed.population(nodes,poses,scale)
        batch = fixed.refine(nodes,poses,prior,scale,cap=INNER_CAP)
        require(batch['frozen_population']==fixed.metadata(before),'inner population selected once at the round start')
        poses = np.asarray(batch['final_poses']);after = fixed.population(nodes,poses,scale)
        e = fixed.equation(after,nodes,poses,prior,scale);correction = fixed.discrete.correction(e,len(nodes)-1)
        selection = dict(population=fixed.metadata(after),equation=fixed.base.encode_equation(e),correction=correction,
            cost=fixed.energy(after,nodes,poses,prior,scale))
        rounds.append(dict(number=round_number,batch=batch,reselected=selection,transition=transition(before,after)))
        termination,failure = stop(batch,correction,round_number)
        if termination is not None:break
    return dict(scale=scale,max_rounds=ROUNDS,inner_cap=INNER_CAP,max_corrections=ROUNDS*INNER_CAP,
        initial_poses=original.tolist(),rounds=rounds,association_rounds=len(rounds),
        applied_iterations=sum(r['batch']['applied_iterations'] for r in rounds),termination=termination,guard_failure=failure,
        selection_converged=termination=='converged',final_fixed_converged=batch['continuous_converged'],
        final_poses=poses.tolist(),final_reselected_correction=correction,
        final_original_discrete_equation=batch['final_discrete_equation'],final_original_discrete_correction=batch['final_discrete_correction'])
