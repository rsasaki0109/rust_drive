#!/usr/bin/env python3
"""Independent triangle-normal/finite-difference camera checks and raw audit."""
import argparse
import copy
import importlib.util
import io
import json
from pathlib import Path
import tarfile

import numpy as np
from PIL import Image

from projective_depth_common import CAMERA, ROOT, exp_rotation, fresh, require, sha, write


def basis(normals):
    axis = np.eye(3)[np.argmin(abs(normals), axis=1)]
    u = np.cross(normals, axis); u /= np.linalg.norm(u, axis=1)[:, None]
    return u, np.cross(normals, u)


def independent(sample, parameters):
    a, b = parameters; scale = np.array([np.exp(b-a), np.exp(b-a), np.exp(b)])
    u, v = basis(sample['normal'])
    n = np.cross(u*scale, v*scale); n /= np.linalg.norm(n, axis=1)[:, None]
    query = (sample['source']*scale)@sample['transform'][:3, :3].T+sample['transform'][:3, 3]
    residual = np.sum(n*(query-sample['target']*scale), axis=1)
    return residual, n, query


def fit_residual(sample, parameters, objective):
    residual = independent(sample, parameters)[0]
    require(objective in ('physical','raw_scaled'), 'declared fitted residual units')
    return residual if objective == 'physical' else residual/np.exp(parameters[1])


def finite_system(samples, parameters, objective='raw_scaled'):
    h = np.zeros((2, 2)); g = np.zeros(2); cost = 0.
    for sample in samples:
        residual = fit_residual(sample, parameters, objective); columns = []
        for axis in range(2):
            d = np.zeros(2); d[axis] = 1e-6
            columns.append((fit_residual(sample, np.asarray(parameters)+d, objective)-fit_residual(sample, np.asarray(parameters)-d, objective))/2e-6)
        j = np.column_stack(columns); w = np.minimum(1., .01/np.maximum(abs(residual), 1e-15))
        factor = 1/(len(samples)*len(residual))
        h += factor*j.T@(w[:, None]*j); g += factor*j.T@(w*residual)
        cost += factor*float(np.sum(np.where(abs(residual) <= .01, residual*residual/2, .01*abs(residual)-.00005)))
    return dict(h=h, g=g, cost_m2=cost)


def independent_cost(samples, parameters, objective='raw_scaled'):
    total = 0.
    for sample in samples:
        residual = fit_residual(sample, parameters, objective)
        total += float(np.mean(np.where(abs(residual) <= .01, residual*residual/2, .01*abs(residual)-.00005)))/len(samples)
    return total


def compare(actual, expected):
    np.testing.assert_allclose(actual['h'], expected['h'], atol=2e-10, rtol=2e-6, err_msg='independent camera normal matrix')
    np.testing.assert_allclose(actual['g'], expected['g'], atol=2e-11, rtol=2e-6, err_msg='independent camera gradient')
    require(abs(actual['cost_m2']-expected['cost_m2']) <= 1e-12, 'independent camera robust cost')
    if 'eigenvalues_m2' in actual:
        eigen = np.linalg.eigvalsh(np.asarray(actual['h']))
        np.testing.assert_allclose(actual['eigenvalues_m2'], eigen, atol=1e-14, rtol=1e-10)
        condition = float(eigen[-1]/eigen[0]) if eigen[0] > 0 else None
        require(condition == actual['condition'], 'fixed-unit camera sensitivity condition')
        require(actual['observable'] == (eigen[-1] >= 1e-12 and condition is not None and condition <= 10000.), 'camera observability admission')


def synthetic(parameters, translation=True):
    rng = np.random.default_rng(1704); samples = []
    for i in range(6):
        target = rng.uniform([-.7, -.5, 1.1], [.7, .5, 4.5], (600, 3))
        normals = rng.normal(0., .4, (600, 3)); normals[:, 2] += 1.5
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        r = exp_rotation(np.array([.03*(i-2), -.04*(i%3-1), .02*(i-1)]))
        t = np.array([.03*(i-2), .02*(i%3-1), .03*(i+1)]) if translation else np.zeros(3)
        source = (target-t)@r
        # Construct the raw measurement coordinates and triangle normals from
        # physical points. The checked implementation uses a dual normal map.
        a, b = parameters; inverse = np.array([np.exp(a-b), np.exp(a-b), np.exp(-b)])
        u, v = basis(normals); raw_n = np.cross(u*inverse, v*inverse)
        raw_n /= np.linalg.norm(raw_n, axis=1)[:, None]
        transform = np.eye(4); transform[:3, :3] = r; transform[:3, 3] = t
        samples.append(dict(source=source*inverse, target=target*inverse, normal=raw_n, transform=transform))
    return samples


def controls():
    from projective_camera_fit import evaluate, fit, system
    recovered = []
    for truth in ([.018, -.023], [-.025, .016]):
        samples = synthetic(truth); result = fit(samples)
        require(result['status'] == 'converged', 'excited synthetic camera fit converges')
        np.testing.assert_allclose(result['parameters'], truth, atol=1e-7, rtol=0)
        recovered.append(result['parameters'])
        audit_fit(json.loads(json.dumps(result)), samples)
    samples = synthetic([.018, -.023]); parameters = np.array([-.01, .012])
    expected = finite_system(samples, parameters); compare(system(samples, parameters), expected)
    sample = samples[0]; observed = evaluate(sample, parameters); columns = []
    for axis in range(2):
        d = np.zeros(2); d[axis] = 1e-6
        columns.append((independent(sample, parameters+d)[0]-independent(sample, parameters-d)[0])/2e-6)
    np.testing.assert_allclose(observed['jacobian'], np.column_stack(columns), atol=3e-8, rtol=2e-6)
    fitted_columns = []
    for axis in range(2):
        d = np.zeros(2); d[axis] = 1e-6
        fitted_columns.append((fit_residual(sample, parameters+d, 'raw_scaled')-fit_residual(sample, parameters-d, 'raw_scaled'))/2e-6)
    np.testing.assert_allclose(observed['fit_jacobian'], np.column_stack(fitted_columns), atol=3e-8, rtol=2e-6)
    # A pure-rotation control makes uniform depth scale unobservable.
    require(not system(synthetic([.018, -.023], False), [.018, -.023])['observable'], 'pure rotation cannot calibrate depth scale')
    noisy = synthetic([.018, -.023], False); rng = np.random.default_rng(1705)
    for s in noisy:
        s['source'] += rng.normal(0., .003, s['source'].shape)
        s['target'] += rng.normal(0., .003, s['target'].shape)
    gauge = system(noisy, [.018, -.023]); smaller = system(noisy, [.018, -.033])
    require(not gauge['observable'] and abs(gauge['cost_m2']-smaller['cost_m2']) < 1e-12,
            'noise cannot create metric scale information under pure rotation')
    require(all(np.all(evaluate(s, [.018, -.023])['fit_jacobian'][:, 1] == 0.) for s in noisy), 'zero translation has exactly zero scale sensitivity')
    require(fit(noisy)['status'] == 'unobservable', 'noisy gauge rejects before inventing calibration')
    identity = copy.deepcopy(samples)
    for s in identity:s['source'] = s['target'].copy(); s['transform'] = np.eye(4)
    require(fit(identity)['status'] == 'unobservable', 'unexcited data cannot export a calibration')
    altered = copy.deepcopy(system(samples, parameters)); altered['h'] *= 2; altered['g'] *= 2
    try:compare(altered, expected)
    except (AssertionError, ValueError):pass
    else:raise ValueError('joint H/g rescaling escaped independent finite differences')
    return dict(recovered_profiles=recovered, focal_and_depth_derivatives_checked=True, triangle_normals_independent=True,
        pure_rotation_scale_rejected=True, noisy_rotation_scale_rejected=True, normalized_scale_invariant=True,
        unexcited_fit_rejected=True, joint_Hg_rescaling_rejected=True)


def points(depth, ids):
    z = depth.ravel()[ids]; y, x = ids//640, ids%640
    return np.column_stack([(x-319.5)*z/525., (y-239.5)*z/525., z])


def normals(depth, ids):
    y, x = ids//640, ids%640; inside = (x > 0)&(x < 639)&(y > 0)&(y < 479)
    result = np.zeros((len(ids), 3)); safe = ids[inside]; z = depth.ravel()[safe]
    valid = (z > .3)&(z < 5)
    for offset in (1, -1, 640, -640):
        neighbour = depth.ravel()[safe+offset]
        valid &= (neighbour > .3)&(abs(neighbour-z) < .05+.01*z)
    cross = np.cross(points(depth, safe+1)-points(depth, safe-1), points(depth, safe+640)-points(depth, safe-640))
    length = np.linalg.norm(cross, axis=1); valid &= length > 1e-10
    selected = np.flatnonzero(inside)[valid]; result[selected] = cross[valid]/length[valid, None]
    return result


def load(case, reports, reference):
    base = ROOT/'data'/case; folder = reports/case
    freeze = json.loads((folder/'freeze.json').read_text()); body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'] and freeze['camera'] == CAMERA, 'fixed camera and original manifest')
    manifest = json.loads(body)
    def read(name):
        path = base/'raw'/name; require(path.is_file() and not path.is_symlink(), 'regular original source')
        body = path.read_bytes(); pin = freeze['inputs'][name]
        require(len(body) == pin['bytes'] and sha(body) == pin['sha256'], 'qualified original source')
        return body
    labels = np.loadtxt(io.BytesIO(read('groundtruth.txt')), comments='#'); batches = dict(train=[], validation=[])
    receipts = json.loads((folder/'samples.json').read_text())['pairs']
    for receipt in receipts:
        a, b = receipt['offsets']; frames = [manifest['frames'][i] for i in (a, b)]
        depths = []
        for frame in frames:
            image = Image.open(io.BytesIO(read(frame['depth_file'])))
            require(image.size == (640, 480), 'original depth dimensions')
            depths.append(np.asarray(image, dtype=float)/5000.)
        old, new = depths; transform = np.linalg.inv(reference.truth_at(labels, frames[0]['depth_timestamp']))@reference.truth_at(labels, frames[1]['depth_timestamp'])
        ids = np.arange(480*640).reshape(480, 640)[::2, ::2].ravel()
        ps, ns = points(new, ids), normals(new, ids); good = np.sum(ns*ns, axis=1) > .25
        ids, ps, ns = ids[good], ps[good], ns[good]
        q = ps@transform[:3, :3].T+transform[:3, 3]; ns = ns@transform[:3, :3].T
        good = np.isfinite(q).all(axis=1)&(q[:, 2] > .3)
        ids, ps, q, ns = ids[good], ps[good], q[good], ns[good]
        x = np.rint(525*q[:, 0]/q[:, 2]+319.5); y = np.rint(525*q[:, 1]/q[:, 2]+239.5)
        good = (x >= 1)&(x < 639)&(y >= 1)&(y < 479)
        ids, ps, q, ns = ids[good], ps[good], q[good], ns[good]
        cells = y[good].astype(np.int64)*640+x[good].astype(np.int64)
        pt, nt = points(old, cells), normals(old, cells)
        good = (np.sum(nt*nt, axis=1) > .25)&(np.linalg.norm(q-pt, axis=1) < .08)&(np.sum(ns*nt, axis=1) > .8)
        require(int(good.sum()) == receipt['original_matches'], 'independent original full match population')
        expected = dict(source=ps[good][::8], target=pt[good][::8], normal=nt[good][::8], transform=transform,
                        source_pixels=ids[good][::8], target_pixels=cells[good][::8])
        path = folder/receipt['cache_file']; require(sha(path.read_bytes()) == receipt['cache_sha256'], 'derived cache byte binding')
        with np.load(path, allow_pickle=False) as cache:
            require(set(cache.files) == set(expected), 'complete source-cache schema')
            for name, value in expected.items():
                if name.endswith('_pixels'):np.testing.assert_array_equal(cache[name], value)
                else:np.testing.assert_allclose(cache[name], value, atol=1e-12, rtol=1e-12)
        batches[receipt['partition']].append(expected)
    return batches


def audit_fit(result, samples, objective='raw_scaled'):
    checked = 0; parameters = np.zeros(2)
    for step in result['trace']:
        np.testing.assert_array_equal(step['before'], parameters)
        compare(step['equation'], finite_system(samples, step['before'], objective)); checked += 1
        if 'unclipped_step' in step:
            proposal = np.linalg.solve(step['equation']['h'], -np.asarray(step['equation']['g']))
            np.testing.assert_allclose(step['unclipped_step'], proposal, atol=1e-12, rtol=1e-10)
            proposal *= min(1., .02/max(abs(proposal))) if max(abs(proposal)) else 1.
        accepted = None
        for i, trial in enumerate(step['trials']):
            require(trial['factor'] == (1., .5, .25, .125, .0625)[i], 'fixed line-search factors')
            np.testing.assert_array_equal(trial['parameters'], np.clip(parameters+trial['factor']*proposal, -.05, .05))
            cost = independent_cost(samples, trial['parameters'], objective)
            require(abs(trial['cost_m2']-cost) <= 1e-12, 'independent line-search trial cost')
            if trial['cost_m2'] < step['equation']['cost_m2']:
                require(i == len(step['trials'])-1, 'stop after first improving trial'); accepted = trial['parameters']
        require(('after' in step) == (accepted is not None), 'accepted camera step is backed by an improving trial')
        if accepted is not None:
            np.testing.assert_array_equal(step['after'], accepted); parameters = np.asarray(accepted)
    np.testing.assert_array_equal(result['parameters'], parameters)
    compare(result['final_system'], finite_system(samples, result['parameters'], objective))
    if result['status'] == 'converged':
        require(result['final_system']['observable'] and max(abs(parameters)) < .05-1e-10, 'qualified fit is observable and interior')
        require(max(abs(np.asarray(result['trace'][-1]['unclipped_step']))) < 1e-7, 'actual stationary camera fit')
    return checked+1


def audit(reports, output, reference, source_archive=None):
    freeze = json.loads((reports/'freeze.json').read_text())
    objective = freeze.get('fit_objective','physical')
    require(not freeze['native_binary_executed'] and freeze['original_scoring_clocks_unchanged'], 'offline-only calibration role')
    archived = None
    if source_archive is not None:
        with tarfile.open(source_archive) as stream:
            archived = {m.name: stream.extractfile(m).read() for m in stream.getmembers() if m.isfile()}
    for name, digest in freeze['source_sha256'].items():
        require(sha((ROOT/name).read_bytes() if archived is None else archived[name]) == digest, 'fixed diagnostic source epoch')
    batches = {case: load(case, reports, reference) for case in freeze['validation_records']}
    checked = 0; all_train = []; fits = {}; folds = {}
    for case in freeze['training_records']:
        samples = batches[case]['train']; all_train.extend(samples)
        result = json.loads((reports/(case+'-fit.json')).read_text()); fits[case] = result['full']; folds[case] = result['leave_one_pair_out']
        checked += audit_fit(result['full'], samples, objective)
        require(len(result['leave_one_pair_out']) == len(samples) == 15, 'all fixed leave-one-out folds retained')
        for i, f in enumerate(result['leave_one_pair_out']):checked += audit_fit(f, samples[:i]+samples[i+1:], objective)
    joint = json.loads((reports/'joint-fit.json').read_text()); checked += audit_fit(joint, all_train, objective)
    validation_checks = {}
    for case in freeze['validation_records']:
        result = json.loads((reports/(case+'-validation.json')).read_text())
        for label, parameters in (('original', [0., 0.]), ('candidate', joint['parameters'])):
            rows = result[label]['pairs']; require(len(rows) == len(batches[case]['validation']) == 15, 'all validation pairs retained')
            corrections = []
            for row, sample in zip(rows, batches[case]['validation'], strict=True):
                residual, n, q = independent(sample, parameters); j = np.column_stack([n, np.cross(q, n)])
                w = np.minimum(1., .01/np.maximum(abs(residual), 1e-15)); h, g = j.T@(w[:, None]*j), j.T@(w*residual)
                np.testing.assert_allclose(row['pose_equation']['h'], h, atol=1e-7, rtol=1e-10)
                np.testing.assert_allclose(row['pose_equation']['g'], g, atol=1e-9, rtol=1e-10)
                require(abs(row['rms_m']-float(np.sqrt(np.mean(residual**2)))) <= 1e-12, 'independent validation raw RMS')
                cost = float(np.mean(np.where(abs(residual) <= .01, residual*residual/2, .01*abs(residual)-.00005)))
                require(abs(row['robust_cost_m2']-cost) <= 1e-12 and row['samples'] == len(residual), 'independent validation robust cost and population')
                eigen = np.linalg.eigvalsh(h); observable = eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6
                require((row['correction'] is not None) == observable, 'complete validation pose observability')
                if observable:
                    correction = np.linalg.solve(h, -g); corrections.append(correction)
                    np.testing.assert_allclose(row['correction'], correction, atol=1e-10, rtol=1e-8)
            summary = result[label]['summary']; complete = len(corrections) == 15
            require(summary['pairs'] == 15 and summary['all_pose_probes_observable'] == complete, 'validation denominator unchanged')
            require(abs(summary['mean_robust_cost_m2']-float(np.mean([row['robust_cost_m2'] for row in rows]))) <= 1e-12, 'validation robust cost summary')
            require(abs(summary['rms_m']-float(np.sqrt(np.mean([row['rms_m']**2 for row in rows])))) <= 1e-12, 'validation RMS summary')
            if complete:
                corrections = np.asarray(corrections)
                require(abs(summary['median_position_correction_m']-float(np.median(np.linalg.norm(corrections[:, :3], axis=1)))) <= 1e-10, 'validation translation summary')
                require(abs(summary['median_rotation_correction_rad']-float(np.median(np.linalg.norm(corrections[:, 3:], axis=1)))) <= 1e-10, 'validation rotation summary')
        original, candidate = result['original']['summary'], result['candidate']['summary']
        flags = dict(complete_pose_probes=original['all_pose_probes_observable'] and candidate['all_pose_probes_observable'])
        for metric in ('mean_robust_cost_m2','rms_m','median_position_correction_m','median_rotation_correction_rad'):
            a, b = original[metric], candidate[metric]; flags[metric+'_nonincreasing'] = a is not None and b is not None and b <= a
        require(flags == result['checks'], 'all validation gates recomputed'); validation_checks[case] = flags
    summary = json.loads((reports/'summary.json').read_text())
    records = freeze['training_records']
    agreement = float(np.max(abs(np.asarray(fits[records[0]]['parameters'])-fits[records[1]]['parameters'])))
    deviations = {c: max(float(np.max(abs(np.asarray(f['parameters'])-fits[c]['parameters']))) for f in folds[c]) for c in records}
    checks = dict(all_fits_converged_without_bound=joint['status'] == 'converged' and all(fits[c]['status'] == 'converged' and all(f['status'] == 'converged' for f in folds[c]) for c in records),
        cross_record_agreement=agreement <= .01, leave_one_pair_out_stable=all(v <= .01 for v in deviations.values()),
        validation_nonincreasing=all(all(row.values()) for row in validation_checks.values()))
    require(summary['checks'] == checks and summary['trial_eligible'] == all(checks.values()), 'unchanged eligibility decision')
    require((reports/'trial-profile.json').exists() == summary['trial_eligible'], 'unqualified profile never exported')
    write(output/'raw-audit.json', dict(pass_independent_camera_audit=True, finite_difference_systems=checked,
        raw_pairs=75, validation_pose_systems=90, raw_samples_reconstructed=True, all_leave_one_out_folds_checked=True,
        trial_eligible=summary['trial_eligible'], physical_calibration_certified=False, fit_objective=objective))


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--output', type=Path, required=True)
    p.add_argument('--reports', type=Path); p.add_argument('--source-archive', type=Path)
    args = p.parse_args(); output = fresh(args.output)
    require(args.source_archive is None or args.reports is not None, 'source archive belongs to a recorded diagnostic')
    sources = [Path(__file__), ROOT/'scripts/projective_camera_fit.py', ROOT/'scripts/projective_depth_common.py', ROOT/'scripts/check-opencv-rgbd.py']
    pins = {str(f.relative_to(ROOT)): sha(f.read_bytes()) for f in sources}
    write(output/'freeze.json', dict(source_sha256=pins, reports=None if args.reports is None else str(args.reports),
        diagnostic_source_archive_sha256=None if args.source_archive is None else sha(args.source_archive.read_bytes())))
    write(output/'controls.json', controls())
    if args.reports is not None:
        spec = importlib.util.spec_from_file_location('ref', ROOT/'scripts/check-opencv-rgbd.py')
        ref = importlib.util.module_from_spec(spec); spec.loader.exec_module(ref); ref.controls(); audit(args.reports.absolute(), output, ref, args.source_archive)
    require(all(sha((ROOT/n).read_bytes()) == h for n, h in pins.items()), 'checker sources unchanged')
    write(output/'summary.json', dict(pass_camera_checks=True, raw_audit_completed=args.reports is not None))


if __name__ == '__main__':main()
