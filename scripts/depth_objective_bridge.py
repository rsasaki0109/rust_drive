"""Read-only source and differential accounting between two depth objectives."""
import numpy as np

import slope_admission_refinement as kernel
from projective_depth_common import require,sha
from symmetric_depth_factors import residual

TERMS = ('removed_fixed_sources','shared_residual_influence','shared_jacobian',
    'shared_target_reassignment','fresh_only_sources')


def continuous_rows(matches,nodes,poses,scale=1):
    result = []
    for item in matches:
        r,j = kernel.evaluate(item,nodes,poses,scale)
        result.append(dict(target=item['target'],source=item['source'],source_ids=item['source_ids'],
            points=item['points'],normals=item['normals'],r=r,j=j))
    return result


def discrete_rows(matches,poses):
    result = []
    for item in matches:
        a,b = item['target'],item['source'];t,s = poses[a],poses[b];pt,nt,ps,ns = item['points']
        r,j = residual(pt@t[:3,:3].T+t[:3,3],nt@t[:3,:3].T,ps@s[:3,:3].T+s[:3,3],ns@s[:3,:3].T)
        result.append(dict(target=a,source=b,source_ids=item['pixel_ids'][:,1],target_ids=item['pixel_ids'][:,0],
            points=ps,normals=ns,r=r,j=j))
    return result


def indexed(rows,current):
    result = {}
    for row in rows:
        a,b = row['target'],row['source'];ids = np.asarray(row['source_ids']);n = len(ids)
        require(0<=a<b<=current and (a,b) not in result,'unique bounded factor endpoints')
        require(ids.dtype.kind in 'iu' and np.all(ids>=0) and len(np.unique(ids))==n,'unique source identities')
        require(row['r'].shape==(n,) and row['j'].shape==(n,6),'one residual/differential per source')
        require(row['points'].shape==row['normals'].shape==(n,3),'source geometry shape')
        require(all(np.isfinite(row[k]).all() for k in ('r','j','points','normals')),'finite bridge rows')
        if 'target_ids' in row:
            require(row['target_ids'].shape==(n,) and row['target_ids'].dtype.kind in 'iu','target identity shape/type')
        result[a,b] = row
    return result


def add(vector,local,pair):
    a,b = pair
    vector[6*(b-1):6*b] += local
    if a:vector[6*(a-1):6*a] -= local


def selected(row,ids):
    lookup = {int(value):i for i,value in enumerate(row['source_ids'])}
    require(all(int(value) in lookup for value in ids),'initial measured pair for every fixed source')
    return np.array([lookup[int(value)] for value in ids],dtype=int)


def bridge(fixed,original,fresh,current):
    """No solve, pruning, threshold override, or objective change is performed."""
    fixed,original,fresh = [indexed(rows,current) for rows in (fixed,original,fresh)]
    size = 6*current;terms = {key:np.zeros(size) for key in TERMS}
    first,last = np.zeros(size),np.zeros(size);pairs = []
    for pair in sorted(fixed.keys()|fresh.keys()):
        c,f = fixed.get(pair),fresh.get(pair)
        ci = np.array([],dtype=int) if c is None else c['source_ids']
        fi = np.array([],dtype=int) if f is None else f['source_ids']
        common,i,j = np.intersect1d(ci,fi,assume_unique=True,return_indices=True)
        lost = ~np.isin(ci,common);new = ~np.isin(fi,common)
        local = {key:np.zeros(6) for key in TERMS};retargeted = 0
        if c is not None:
            require(pair in original,'original pair owns the fixed sources')
            o = original[pair];all_old = selected(o,ci)
            np.testing.assert_array_equal(c['points'],o['points'][all_old])
            np.testing.assert_array_equal(c['normals'],o['normals'][all_old])
            add(first,c['j'].T@np.clip(c['r'],-.01,.01),pair)
            local['removed_fixed_sources'] = -c['j'][lost].T@np.clip(c['r'][lost],-.01,.01)
        if f is not None:
            add(last,f['j'].T@np.clip(f['r'],-.01,.01),pair)
            local['fresh_only_sources'] = f['j'][new].T@np.clip(f['r'][new],-.01,.01)
        if len(common):
            k = selected(o,common)
            np.testing.assert_array_equal(c['points'][i],f['points'][j])
            np.testing.assert_array_equal(c['normals'][i],f['normals'][j])
            cr,orr,fr = [np.clip(row['r'][ids],-.01,.01) for row,ids in ((c,i),(o,k),(f,j))]
            cj,oj,fj = c['j'][i],o['j'][k],f['j'][j]
            local['shared_residual_influence'] = cj.T@(orr-cr)
            local['shared_jacobian'] = (oj-cj).T@orr
            local['shared_target_reassignment'] = fj.T@fr-oj.T@orr
            changed = o['target_ids'][k]!=f['target_ids'][j];retargeted = int(changed.sum())
            np.testing.assert_array_equal(o['r'][k][~changed],f['r'][j][~changed])
            np.testing.assert_array_equal(o['j'][k][~changed],f['j'][j][~changed])
        for key,value in local.items():add(terms[key],value,pair)
        pairs.append(dict(target=pair[0],source=pair[1],fixed_count=len(ci),fresh_count=len(fi),shared_count=len(common),
            removed_count=int(lost.sum()),added_count=int(new.sum()),retargeted_shared_count=retargeted,
            shared_source_sha256=sha(np.asarray(common,dtype='<i8').tobytes()),
            removed_source_sha256=sha(np.asarray(ci[lost],dtype='<i8').tobytes()),
            added_source_sha256=sha(np.asarray(fi[new],dtype='<i8').tobytes())))
    difference = last-first;error = sum(terms.values())-difference
    np.testing.assert_allclose(error,0.,atol=2e-10,rtol=0)
    return dict(pairs=pairs,fixed_raw_gradient=first.tolist(),fresh_raw_gradient=last.tolist(),
        gradient_difference=difference.tolist(),terms={key:value.tolist() for key,value in terms.items()},
        max_partition_error=float(np.max(abs(error))))


def scaled_summary(report,fresh_h):
    diagonal = np.diag(fresh_h)
    require(np.isfinite(diagonal).all() and np.all(diagonal>0),'positive common diagnostic coordinate scale')
    scale = 1/np.sqrt(diagonal);difference = scale*np.asarray(report['gradient_difference'])
    squared = float(difference@difference)
    values = {}
    for key in TERMS:
        term = scale*np.asarray(report['terms'][key])
        values[key] = dict(scaled_norm=float(np.linalg.norm(term)),
            signed_projection_fraction=None if squared==0 else float(term@difference/squared))
    return dict(coordinate_scale=scale.tolist(),scaled_difference_norm=float(np.sqrt(squared)),terms=values)
