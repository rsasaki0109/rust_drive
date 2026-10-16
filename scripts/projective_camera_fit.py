"""Offline two-parameter camera probes; no operational tracking or pose labels here."""
import numpy as np

from projective_depth_common import require

BOUND = .05
HUBER = .01


def evaluate(sample, parameters):
    a, b = np.asarray(parameters, dtype=float)
    require(np.isfinite([a, b]).all() and max(abs(a), abs(b)) <= BOUND+1e-12, 'bounded camera parameters')
    axial = np.exp(b); lateral = np.array([np.exp(-a), np.exp(-a), 1.])
    ps, pt = axial*sample['source']*lateral, axial*sample['target']*lateral
    n = sample['normal']*np.array([np.exp(a), np.exp(a), 1.])
    n /= np.linalg.norm(n, axis=1)[:, None]
    transform = sample['transform']; rotated = ps@transform[:3, :3].T
    q = rotated+transform[:3, 3]; difference = q-pt
    residual = np.einsum('ij,ij->i', n, difference)
    # The normal moves under the inverse transpose of the focal ray change.
    bn = n*np.array([1., 1., 0.])
    dn = bn-n*np.einsum('ij,ij->i', n, bn)[:, None]
    dps, dpt = ps*np.array([-1., -1., 0.]), pt*np.array([-1., -1., 0.])
    da = np.einsum('ij,ij->i', dn, difference)+np.einsum('ij,ij->i', n, dps@transform[:3, :3].T-dpt)
    db = np.einsum('ij,ij->i', n, rotated-pt)
    # Express fitting errors in the original nominal raw metres. Otherwise
    # shrinking the depth multiplier also shrinks measurement noise, creating
    # artificial scale information even when reference translation is zero.
    fit_jacobian = np.column_stack([da/axial, -(n@transform[:3, 3])/axial])
    return dict(residual=residual, jacobian=np.column_stack([da, db]), normal=n, query=q,
                fit_residual=residual/axial, fit_jacobian=fit_jacobian)


def loss(residual):
    magnitude = abs(residual)
    return np.where(magnitude <= HUBER, .5*residual**2, HUBER*(magnitude-.5*HUBER))


def system(samples, parameters):
    require(len(samples) > 0, 'nonempty camera batches')
    h = np.zeros((2, 2)); g = np.zeros(2); cost = 0.
    for sample in samples:
        values = evaluate(sample, parameters); residual, j = values['fit_residual'], values['fit_jacobian']
        require(len(residual) >= 200 and np.isfinite(j).all() and np.isfinite(residual).all(), 'finite supported camera batch')
        weight = np.minimum(1., HUBER/np.maximum(abs(residual), 1e-15))
        normalization = 1/(len(samples)*len(residual))
        h += normalization*(j.T@(weight[:, None]*j)); g += normalization*(j.T@(weight*residual))
        cost += normalization*float(np.sum(loss(residual)))
    eigen = np.linalg.eigvalsh(h)
    condition = float(eigen[-1]/eigen[0]) if eigen[0] > 0 else None
    observable = bool(eigen[-1] >= 1e-12 and condition is not None and condition <= 10000.)
    return dict(h=h, g=g, cost_m2=cost, eigenvalues_m2=eigen, condition=condition, observable=observable)


def serial(values):
    return {key: value.tolist() if isinstance(value, np.ndarray) else value for key, value in values.items()}


def fit(samples):
    parameters = np.zeros(2); trace = []
    for _ in range(12):
        equation = system(samples, parameters)
        row = dict(before=parameters.tolist(), equation=serial(equation), trials=[]); trace.append(row)
        if not equation['observable']:
            status = 'unobservable'; break
        step = np.linalg.solve(equation['h'], -equation['g'])
        row['unclipped_step'] = step.tolist()
        if max(abs(step)) < 1e-7:
            status = 'at_bound' if max(abs(parameters)) >= BOUND-1e-10 else 'converged'; break
        step *= min(1., .02/max(abs(step)))
        accepted = False
        for factor in (1., .5, .25, .125, .0625):
            trial = np.clip(parameters+factor*step, -BOUND, BOUND)
            candidate = system(samples, trial)
            row['trials'].append(dict(factor=factor, parameters=trial.tolist(), cost_m2=candidate['cost_m2']))
            if candidate['cost_m2'] < equation['cost_m2']:
                parameters = trial; accepted = True; row['after'] = parameters.tolist(); break
        if not accepted:
            status = 'at_bound' if max(abs(parameters)) >= BOUND-1e-10 else 'line_search_failure'; break
    else:
        status = 'iteration_limit'
    if max(abs(parameters)) >= BOUND-1e-10:
        status = 'at_bound'
    return dict(status=status, parameters=parameters.tolist(), focal_multiplier=float(np.exp(parameters[0])),
                depth_multiplier=float(np.exp(parameters[1])), final_system=serial(system(samples, parameters)), trace=trace)


def validation(samples, parameters):
    rows = []
    for sample in samples:
        values = evaluate(sample, parameters); r, n, q = values['residual'], values['normal'], values['query']
        j = np.column_stack([n, np.cross(q, n)])
        w = np.minimum(1., HUBER/np.maximum(abs(r), 1e-15))
        h, g = j.T@(w[:, None]*j), j.T@(w*r)
        eigen = np.linalg.eigvalsh(h)
        observable = eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6
        correction = np.linalg.solve(h, -g) if observable else None
        rows.append(dict(samples=len(r), robust_cost_m2=float(np.mean(loss(r))), rms_m=float(np.sqrt(np.mean(r*r))),
            pose_equation=dict(h=h.tolist(), g=g.tolist()), correction=None if correction is None else correction.tolist()))
    complete = all(row['correction'] is not None for row in rows)
    corrections = np.array([row['correction'] for row in rows]) if complete else None
    summary = dict(pairs=len(rows), all_pose_probes_observable=complete,
        mean_robust_cost_m2=float(np.mean([row['robust_cost_m2'] for row in rows])),
        rms_m=float(np.sqrt(np.mean([row['rms_m']**2 for row in rows]))),
        median_position_correction_m=float(np.median(np.linalg.norm(corrections[:, :3], axis=1))) if complete else None,
        median_rotation_correction_rad=float(np.median(np.linalg.norm(corrections[:, 3:], axis=1))) if complete else None)
    return dict(summary=summary, pairs=rows)
