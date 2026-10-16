#!/usr/bin/env python3
"""Ghost-surface removal, unknown/occluded depth and accepted-state controls."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

from projective_depth_common import ROOT, Client, audit_sequence, exp_rotation, fresh, pose, require, sha, write
from projective_depth_raw import reconstruct_update
from projective_visibility_raw import controls


def collect(binary, camera, depths, policy, inject=False):
    client = Client(binary, camera, configuration=dict(visibility=policy)); rows = []; inputs = []
    try:
        for i, raw in enumerate(depths):
            if inject and i == 5:
                invalid = np.zeros(raw.shape)
                rows.append(dict(update=client.frame(2.+.033*i-.005, invalid))); inputs.append(invalid)
            rows.append(dict(update=client.frame(2.+.033*i, raw))); inputs.append(raw)
        client.finish()
    except BaseException:
        client.abort(); raise
    return rows, inputs


def reconstruct(rows, inputs, camera, policy):
    audit_sequence(rows, [r['update']['timestamp'] for r in rows])
    model = weights = None; snapshots = []; equations = 0
    for raw, row in zip(inputs, rows, strict=True):
        model, weights, checked = reconstruct_update(model, weights, raw, camera, row['update'], visibility=policy)
        equations += checked['iterations']+checked['final_equations']
        snapshots.append((model.copy(), weights.copy()))
    return snapshots, equations


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); out = fresh(args.output); binary = args.binary.absolute()
    sources = [Path(__file__), ROOT/'scripts/projective_depth_common.py', ROOT/'scripts/check-projective-depth.py',
               *sorted((ROOT/'scripts').glob('projective_*_raw.py'))]
    pins = {str(f.relative_to(ROOT)): sha(f.read_bytes()) for f in sources}; binary_hash = sha(binary.read_bytes())
    write(out/'freeze.json', dict(source_sha256=pins, binary_sha256=binary_hash, dynamic_frames=9,
        patch_rows=[12, 24], patch_columns=[12, 24], foreground_depth_m=1.5, original_support=3,
        unknown_offset=4, expected_first_removal_offset=6, motion_frames=12, axial_noise_std_m=.002,
        noise_seed=1703, position_gate_m=.01, rotation_gate_rad=.01, reference_pose_supplied_to_tracker=False))
    spec = importlib.util.spec_from_file_location('scene', ROOT/'scripts/check-projective-depth.py')
    scene = importlib.util.module_from_spec(spec); spec.loader.exec_module(scene)
    patch = (slice(12, 24), slice(12, 24)); plain = scene.render(np.eye(4))
    depths = []
    for i in range(9):
        raw = plain.copy()
        if i in (0, 1, 2, 7):raw[patch] = 1.5
        if i == 4:raw[patch] = 0.
        depths.append(raw)
    runs = {}; inputs = {}
    for name, policy, inject in (('legacy', 'disabled', False), ('decay', 'free_space_decay', False),
                                 ('decay_rejected', 'free_space_decay', True)):
        rows, stream = collect(binary, scene.CAMERA, depths, policy, inject)
        runs[name] = rows; inputs[name] = stream
        write(out/(name+'-sensor-only.json'), dict(frames=rows, reference_pose_supplied_to_tracker=False))
    summaries = {}; snapshots = {}
    for name, rows in runs.items():
        policy = 'disabled' if name == 'legacy' else 'free_space_decay'
        snapshots[name], equations = reconstruct(rows, inputs[name], scene.CAMERA, policy)
        valid = [row for row in rows if row['update']['accepted']]
        require(len(valid) == 9 and not any(row['update']['lost'] for row in rows), 'complete valid dynamic-scene tracking')
        for row in valid:
            np.testing.assert_allclose(pose(row['update']['first_from_current']), np.eye(4), atol=1e-10, rtol=0)
        summaries[name] = dict(valid_acquisitions=9, raw_equations=equations,
            stale_foreground_cells_after_offset6=int((snapshots[name][6 if name != 'decay_rejected' else 7][0][patch] < 2.).sum()))
    require(summaries['legacy']['stale_foreground_cells_after_offset6'] == 144, 'legacy retains the demonstrated stale foreground')
    require(summaries['decay']['stale_foreground_cells_after_offset6'] == 0, 'decay removes all 144 contradicted foreground cells')
    visibility = [row['update']['fusion']['visibility'] for row in runs['decay'][1:]]
    for offset in (3, 5):
        require(visibility[offset-1]['free_space'] == 144 and visibility[offset-1]['removed'] == 0, 'one credit per measured acquisition')
    require(visibility[3]['free_space'] == 0 and visibility[3]['removed'] == 0, 'unknown patch spends no support credit')
    require(visibility[5]['removed'] == 144 and runs['decay'][6]['update']['fusion']['filled'] == 144, 'third observation clears and refills the surface')
    require(visibility[6]['occluded'] >= 100 and visibility[6]['removed'] == 0, 'near occluder is distinct from measured free space')
    require(visibility[7]['removed'] == 144, 'new one-credit foreground clears on its first contradiction')
    good = [r for r in runs['decay_rejected'] if r['update']['accepted']]
    require(good == runs['decay'], 'injected rejected depth leaves all nine valid outcomes exactly identical')
    failure = runs['decay_rejected'][5]['update']
    require(not failure['accepted'] and failure['fusion'] is None and failure['last_accepted_timestamp'] == 2.+.033*4,
            'rejected acquisition never spends visibility evidence or renews pose')
    wrong_model, wrong_weights = snapshots['legacy'][6]
    try:
        reconstruct_update(wrong_model, wrong_weights, depths[7], scene.CAMERA, runs['decay'][7]['update'], visibility='free_space_decay')
    except (AssertionError, ValueError, KeyError, TypeError, IndexError):
        rejected = ['retained ghost model with valid removal receipt']
    else:
        raise ValueError('subsequent equations failed to detect a retained stale surface')
    # A moving static scene separately exercises carving before a keyframe warp.
    rng = np.random.default_rng(1703); transforms = []; motion_depths = []
    for i in range(12):
        t = np.eye(4); t[:3, :3] = exp_rotation(np.array([.001*i, -.0015*i, .0005*i]))
        t[:3, 3] = [.012*i, .002*np.sin(.4*i), .002*(np.cos(.4*i)-1)]
        exact = scene.render(t); motion_depths.append(exact+rng.normal(0., .002, exact.shape)); transforms.append(t)
    motion, stream = collect(binary, scene.CAMERA, motion_depths, 'free_space_decay')
    write(out/'motion-sensor-only.json', dict(frames=motion, reference_pose_supplied_to_tracker=False))
    _, equations = reconstruct(motion, stream, scene.CAMERA, 'free_space_decay')
    require(all(r['update']['accepted'] for r in motion), 'all twelve moving-scene acquisitions accepted')
    warps = sum(r['update']['keyframe_replaced'] for r in motion)
    require(warps >= 1, 'moving control exercises a stored-map reference change')
    errors = []
    for t, row in zip(transforms[1:], motion[1:], strict=True):
        estimate = pose(row['update']['first_from_current'])
        errors.append(dict(position_error_m=float(np.linalg.norm(estimate[:3, 3]-t[:3, 3])),
            rotation_error_rad=float(np.arccos(np.clip((np.trace(t[:3, :3].T@estimate[:3, :3])-1)/2, -1, 1)))))
    max_p = max(v['position_error_m'] for v in errors); max_r = max(v['rotation_error_rad'] for v in errors)
    require(max_p <= .01 and max_r <= .01, 'unchanged known-motion accuracy gates')
    require(sha(binary.read_bytes()) == binary_hash and all(sha((ROOT/n).read_bytes()) == h for n, h in pins.items()), 'control binary and sources unchanged')
    write(out/'summary.json', dict(pass_visibility_controls=True, hand_controls=controls(), dynamic=summaries,
        identical_valid_outcomes_after_rejection=9, rejected_corruptions=rejected,
        motion=dict(raw_equations=equations, reference_warps=warps, max_position_error_m=max_p, max_rotation_error_rad=max_r, errors=errors),
        physical_sensor_accuracy_claim=False))
    print(json.dumps(dict(dynamic=summaries, motion_equations=equations, maximum_position_m=max_p, maximum_rotation_rad=max_r)), flush=True)


if __name__ == '__main__':
    main()
