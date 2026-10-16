#!/usr/bin/env python3
"""Offline focal/depth-scale consistency study on previously viewed raw pairs."""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

from projective_depth_common import CAMERA, ROOT, fresh, require, sha, write
from projective_depth_raw import compare_equation, normal_equation, pyramid
from projective_range_raw import pairs, equation
from projective_camera_fit import fit, validation

CASES = ('tum-fr1-room-temporal', 'tum-fr1-desk2-independent', 'tum-fr1-rpy-heldout')
TRAIN = [(i, i+6) for i in range(0, 180, 12)]
VALIDATE = [(i, i+6) for i in range(3, 174, 12)]


def prepare(case, output, reference):
    base = ROOT/'data'/case; body = (base/'manifest.json').read_bytes(); manifest = json.loads(body)
    frames = manifest['frames']; require(len(frames) == 180, 'original complete window')
    require([f['source_index'] for f in frames] == list(range(100, 280)), 'original source indices')
    require(all(b['depth_timestamp'] > a['depth_timestamp'] for a, b in zip(frames, frames[1:])), 'chronological depth clock')
    require(not ({i for pair in TRAIN for i in pair}&{i for pair in VALIDATE for i in pair}), 'disjoint training/validation frame endpoints')
    inventory = {item['file']: item for item in manifest['files']}
    def read(name):
        path = base/'raw'/name; require(path.is_file() and not path.is_symlink(), 'regular qualified original source')
        data = path.read_bytes(); pin = inventory[name]
        require(len(data) == pin['bytes'] and sha(data) == pin['sha256'], 'original bytes: '+name)
        return data
    folder = output/case; folder.mkdir()
    write(folder/'freeze.json', dict(manifest_sha256=sha(body), camera=CAMERA, depth_units_per_metre=5000,
        inputs={k: dict(bytes=v['bytes'], sha256=v['sha256']) for k, v in inventory.items()},
        training_pairs=TRAIN if case != CASES[2] else [], validation_pairs=VALIDATE,
        role='viewed development calibration diagnosis; numeric reference allowed only offline'))
    labels = np.loadtxt(io.BytesIO(read('groundtruth.txt')), comments='#')
    require(labels.ndim == 2 and labels.shape[1] == 8 and np.isfinite(labels).all()
            and np.all(np.diff(labels[:, 0]) > 0), 'finite chronological offline reference')
    samples = {}; receipts = []
    def depth(index):
        image = Image.open(io.BytesIO(read(frames[index]['depth_file'])))
        require(image.size == (640, 480) and image.mode in ('I;16', 'I;16B', 'I'), 'original measured depth')
        return pyramid(np.asarray(image, dtype=float)/5000., CAMERA)[1]
    for partition, selected in (('train', TRAIN if case != CASES[2] else []), ('validation', VALIDATE)):
        samples[partition] = []
        for index, (a, b) in enumerate(selected):
            transform = np.linalg.inv(reference.truth_at(labels, frames[a]['depth_timestamp']))@reference.truth_at(labels, frames[b]['depth_timestamp'])
            target, source = depth(a), depth(b); sample = pairs(target, source, transform, 1)
            compare_equation(equation(sample, False), normal_equation(target, source, transform, 1))
            frozen = {k: sample[k][::8].copy() for k in ('source', 'target', 'normal')}
            require(len(frozen['source']) >= 200, 'each fixed pair retains at least200 calibration samples')
            frozen['transform'] = transform
            def pixels(points):
                x = np.rint(CAMERA['fx']*points[:, 0]/points[:, 2]+CAMERA['cx']).astype(np.int64)
                y = np.rint(CAMERA['fy']*points[:, 1]/points[:, 2]+CAMERA['cy']).astype(np.int64)
                return y*640+x
            frozen['source_pixels'] = pixels(frozen['source']); frozen['target_pixels'] = pixels(frozen['target'])
            name = partition+'-'+str(index)+'.npz'
            with (folder/name).open('xb') as stream:
                np.savez_compressed(stream, **frozen)
            receipts.append(dict(partition=partition, index=index, offsets=[a, b], source_indices=[100+a, 100+b],
                timestamps=[frames[a]['depth_timestamp'], frames[b]['depth_timestamp']], original_matches=len(sample['source']),
                frozen_samples=len(frozen['source']), cache_file=name, cache_sha256=sha((folder/name).read_bytes())))
            samples[partition].append(frozen)
    write(folder/'samples.json', dict(pairs=receipts, native_pose_fitter_executed=False))
    return samples


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--design', type=Path, action='append', help='optional fixed designs to bind in the freeze')
    args = p.parse_args(); out = fresh(args.output)
    sources = [Path(__file__), ROOT/'scripts/projective_camera_fit.py', ROOT/'scripts/check-projective-camera.py',
        ROOT/'scripts/projective_depth_common.py', ROOT/'scripts/projective_depth_raw.py', ROOT/'scripts/projective_range_raw.py',
        ROOT/'scripts/check-opencv-rgbd.py', ROOT/'scripts/requirements-visual.txt', *(args.design or [])]
    pins = {str(f.absolute().relative_to(ROOT)): sha(f.read_bytes()) for f in sources}
    write(out/'freeze.json', dict(source_sha256=pins, training_records=CASES[:2], validation_records=CASES,
        parameters=['log_shared_focal_multiplier', 'log_depth_multiplier'], numpy_version=np.__version__,
        numeric_reference_role='offline fixed-motion calibration probes', original_scoring_clocks_unchanged=True,
        native_binary_executed=False, default_changed=False, fit_objective='raw_scaled'))
    spec = importlib.util.spec_from_file_location('reference', ROOT/'scripts/check-opencv-rgbd.py')
    reference = importlib.util.module_from_spec(spec); spec.loader.exec_module(reference); reference.controls()
    batches = {case: prepare(case, out, reference) for case in CASES}
    fits = {}; folds = {}; all_train = []
    for case in CASES[:2]:
        samples = batches[case]['train']; all_train.extend(samples)
        fits[case] = fit(samples)
        folds[case] = [fit(samples[:i]+samples[i+1:]) for i in range(len(samples))]
        write(out/(case+'-fit.json'), dict(full=fits[case], leave_one_pair_out=folds[case]))
        print(case, fits[case]['status'], fits[case]['parameters'], flush=True)
    joint = fit(all_train); write(out/'joint-fit.json', joint)
    results = {}; validation_flags = {}
    for case in CASES:
        original = validation(batches[case]['validation'], np.zeros(2))
        candidate = validation(batches[case]['validation'], joint['parameters'])
        flags = dict(complete_pose_probes=original['summary']['all_pose_probes_observable'] and candidate['summary']['all_pose_probes_observable'])
        for metric in ('mean_robust_cost_m2', 'rms_m', 'median_position_correction_m', 'median_rotation_correction_rad'):
            a, b = original['summary'][metric], candidate['summary'][metric]
            flags[metric+'_nonincreasing'] = a is not None and b is not None and b <= a
        results[case] = dict(original=original, candidate=candidate, checks=flags)
        validation_flags[case] = flags
        write(out/(case+'-validation.json'), results[case]); print(case, json.dumps(flags), flush=True)
    agreement = float(np.max(abs(np.asarray(fits[CASES[0]]['parameters'])-fits[CASES[1]]['parameters'])))
    deviations = {case: max(float(np.max(abs(np.asarray(f['parameters'])-fits[case]['parameters']))) for f in folds[case]) for case in CASES[:2]}
    converged = joint['status'] == 'converged' and all(fits[c]['status'] == 'converged'
        and all(f['status'] == 'converged' for f in folds[c]) for c in CASES[:2])
    checks = dict(all_fits_converged_without_bound=converged, cross_record_agreement=agreement <= .01,
        leave_one_pair_out_stable=all(v <= .01 for v in deviations.values()), validation_nonincreasing=all(all(v.values()) for v in validation_flags.values()))
    eligible = all(checks.values())
    summary = dict(trial_eligible=eligible, checks=checks, joint_fit_status=joint['status'], diagnostic_parameters=joint['parameters'],
        maximum_between_record_log_difference=agreement, maximum_leave_one_pair_out_log_deviation=deviations,
        validation={c: dict(original=v['original']['summary'], candidate=v['candidate']['summary'], checks=v['checks']) for c, v in results.items()},
        exported_trial_profile=eligible, physical_calibration_certified=False, default_changed=False,
        heldout_accuracy_claim=False, subjective_maturity_percent=20, fit_objective='raw_scaled')
    if eligible:
        a, b = joint['parameters']; camera = dict(CAMERA); camera['fx'] *= float(np.exp(a)); camera['fy'] *= float(np.exp(a))
        write(out/'trial-profile.json', dict(schema_version=1, camera=camera, depth_multiplier=float(np.exp(b)),
            qualification='eligible for a frozen viewed trajectory trial only', original_scoring_clocks_unchanged=True,
            derivation_source_sha256=pins, default_adoption=False))
    require(all(sha((ROOT/n).read_bytes()) == h for n, h in pins.items()), 'unchanged calibration source epoch')
    write(out/'summary.json', summary)
    return 0 if eligible else 1


if __name__ == '__main__':
    sys.exit(main())
