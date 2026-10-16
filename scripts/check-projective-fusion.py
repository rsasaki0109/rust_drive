#!/usr/bin/env python3
"""Known-scene fusion, keyframe warps and rejected-update atomicity controls."""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np

from projective_depth_common import ROOT, Client, audit_sequence, exp_rotation, fresh, pose, require, sha, write
from projective_depth_raw import reconstruct_update
from projective_fusion_raw import controls


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); out = fresh(args.output); binary = args.binary.absolute()
    sources = [Path(__file__), ROOT/'scripts/projective_depth_common.py', ROOT/'scripts/projective_depth_raw.py',
               ROOT/'scripts/projective_fusion_raw.py', ROOT/'scripts/check-projective-depth.py']
    pins = {str(f.relative_to(ROOT)): sha(f.read_bytes()) for f in sources}
    binary_hash = sha(binary.read_bytes())
    write(out/'freeze.json', dict(binary_sha256=binary_hash, source_sha256=pins,
        fusion_sampling='visible_mean', frames=12, axial_noise_std_m=.002, noise_seed=1703,
        position_gate_m=.01, rotation_gate_rad=.01, failure_injection_before_offset=5,
        operational_inputs='depth and acquisition timestamp only', scope='synthetic regression'))
    spec = importlib.util.spec_from_file_location('scene', ROOT/'scripts/check-projective-depth.py')
    scene = importlib.util.module_from_spec(spec); spec.loader.exec_module(scene)
    depths = []; transforms = []; rng = np.random.default_rng(1703)
    for i in range(12):
        t = np.eye(4); t[:3, :3] = exp_rotation(np.array([.001*i, -.0015*i, .0005*i]))
        t[:3, 3] = [.012*i, .002*np.sin(.4*i), .002*(np.cos(.4*i)-1)]
        exact = scene.render(t)
        depths.append(exact+rng.normal(0., .002, exact.shape)); transforms.append(t)
    ledgers = {}; streams = {}
    for name in ('clean', 'with_rejection'):
        client = Client(binary, scene.CAMERA, configuration=dict(fusion_sampling='visible_mean'))
        rows = []; inputs = []
        try:
            for i, raw in enumerate(depths):
                if name == 'with_rejection' and i == 5:
                    invalid = np.zeros(raw.shape); stamp = 2.+.033*i-.005
                    rows.append(dict(update=client.frame(stamp, invalid))); inputs.append(invalid)
                rows.append(dict(update=client.frame(2.+.033*i, raw))); inputs.append(raw)
            client.finish()
        except BaseException:
            client.abort(); raise
        ledgers[name] = rows; streams[name] = inputs
        write(out/(name+'-sensor-only.json'), dict(frames=rows, reference_pose_supplied_to_tracker=False))
    # All native outcomes are saved before using the prescribed scene poses.
    summaries = {}
    for name, rows in ledgers.items():
        audit_sequence(rows, [r['update']['timestamp'] for r in rows])
        model = weights = None; equations = 0; changes = 0
        for raw, row in zip(streams[name], rows, strict=True):
            model, weights, checked = reconstruct_update(model, weights, raw, scene.CAMERA, row['update'],
                fusion_sampling='visible_mean')
            equations += checked['iterations']+checked['final_equations']; changes += checked['model_replaced']
        require(changes >= 1, 'the sequence actually exercises stored-model view changes')
        accepted = [r['update'] for r in rows if r['update']['accepted']]
        require(len(accepted) == 12, 'all twelve valid acquisitions accepted')
        errors = []
        for t, update in zip(transforms[1:], accepted[1:], strict=True):
            estimate = pose(update['first_from_current'])
            errors.append(dict(position_error_m=float(np.linalg.norm(estimate[:3, 3]-t[:3, 3])),
                rotation_error_rad=float(np.arccos(np.clip((np.trace(t[:3, :3].T@estimate[:3, :3])-1)/2, -1, 1)))))
        max_p = max(v['position_error_m'] for v in errors); max_r = max(v['rotation_error_rad'] for v in errors)
        require(max_p <= .01 and max_r <= .01, 'fixed known-scene accuracy gates')
        summaries[name] = dict(accepted_updates=11, raw_equations=equations, reference_warps=changes,
            max_position_error_m=max_p, max_rotation_error_rad=max_r, errors=errors)
    good = [r for r in ledgers['with_rejection'] if r['update']['accepted']]
    require(good == ledgers['clean'], 'rejected depth leaves every subsequent accepted outcome identical')
    failure = ledgers['with_rejection'][5]['update']
    require(not failure['accepted'] and not failure['lost'] and failure['fusion'] is None
            and failure['last_accepted_timestamp'] == 2.+.033*4, 'failure cannot commit fusion or renew pose')
    require(sha(binary.read_bytes()) == binary_hash and all(sha((ROOT/n).read_bytes()) == h for n, h in pins.items()),
            'unchanged binary and control sources')
    write(out/'summary.json', dict(pass_fusion_controls=True, hand_controls=controls(), cases=summaries,
        accepted_outcomes_identical_after_rejection=12, physical_sensor_accuracy_claim=False))
    print(json.dumps(summaries), flush=True)


if __name__ == '__main__':
    main()
