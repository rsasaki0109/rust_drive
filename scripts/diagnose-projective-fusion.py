#!/usr/bin/env python3
"""Offline fusion decomposition; reference poses never enter the tracker."""
import argparse
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from projective_depth_common import CAMERA, ROOT, fresh, pose, require, sha, write
import projective_depth_raw as original
import projective_fusion_raw as mean

CASES = ('tum-fr1-room-temporal', 'tum-fr1-desk2-independent', 'tum-fr1-rpy-heldout')
PROBES = range(1, 180, 12)


def distribution(values):
    require(len(values) > 0, 'nonempty diagnostic population')
    median = float(np.median(values))
    return dict(count=len(values), mean_m=float(np.mean(values)), median_m=median,
                rms_m=float(np.sqrt(np.mean(values*values))),
                mad_m=float(np.median(abs(values-median)))*1.4826)


def analytic():
    c = dict(width=160, height=120, fx=140., fy=140., cx=79.5, cy=59.5)
    transform = np.eye(4); transform[2, 3] = .1
    rng = np.random.default_rng(1702)
    errors = {'nearest': [], 'visible_mean': []}
    collisions = {'nearest': [], 'visible_mean': []}
    for _ in range(40):
        raw = 2.+rng.normal(0., .01, (120, 160)); weight = np.ones(raw.shape, dtype=np.uint8)
        old, ow, _ = original.raster(raw, weight, transform, c)
        new, nw, _ = mean.raster(raw, weight, transform, c)
        np.testing.assert_array_equal(ow > 0, nw > 0)
        # Count projecting rays independently of either selector.
        y, x = np.indices(raw.shape)
        tx = np.rint((x-c['cx'])*raw/(raw+.1)+c['cx']).astype(int)
        ty = np.rint((y-c['cy'])*raw/(raw+.1)+c['cy']).astype(int)
        hits = np.bincount((ty*160+tx).ravel(), minlength=raw.size).reshape(raw.shape)
        for label, depth in (('nearest', old), ('visible_mean', new)):
            errors[label].append(depth[ow > 0]-2.1)
            collisions[label].append(depth[hits > 1]-2.1)
    results = {label: dict(all_cells=distribution(np.concatenate(errors[label])),
                          collision_cells=distribution(np.concatenate(collisions[label])))
               for label in errors}
    require(results['nearest']['collision_cells']['mean_m'] < -.003,
            'signed nearest-Z selection bias is visible in fixed control')
    require(abs(results['visible_mean']['collision_cells']['mean_m']) <
            abs(results['nearest']['collision_cells']['mean_m'])/3,
            'surface averaging reduces this conditional signed bias')
    return dict(hand_controls=mean.controls(), prescribed_plane=results,
                physical_noise_or_trajectory_validation=False)


def probe(target_depth, raw, transform):
    equation = original.normal_equation(original.pyramid(target_depth, CAMERA)[1],
        original.pyramid(raw, CAMERA)[1], transform, 1)
    eigen = np.linalg.eigvalsh(equation['h'])
    usable = equation['count'] >= 200 and eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6
    correction = np.linalg.solve(equation['h'], -equation['g']) if usable else None
    return dict(count=equation['count'], rms_m=equation['rms_m'],
        physical_condition=float(eigen[-1]/eigen[0]) if eigen[0] > 0 else None,
        correction_at_reference_pose=None if correction is None else correction.tolist())


def recording(case, output, reference):
    base = ROOT/'data'/case
    body = (base/'manifest.json').read_bytes(); manifest = json.loads(body)
    inventory = {item['file']: item for item in manifest['files']}
    def read(name):
        path = base/'raw'/name
        require(path.is_file() and not path.is_symlink(), 'regular qualified input')
        b = path.read_bytes(); p = inventory[name]
        require(len(b) == p['bytes'] and sha(b) == p['sha256'], 'qualified original bytes')
        return b
    ledger_path = ROOT/'assets/projective-surface-v1/recorded/first-default'/case/'sensor-only.json.gz'
    ledger_body = ledger_path.read_bytes()
    pin = json.loads((ROOT/'assets/projective-surface-v1/files.json').read_text())[str(ledger_path.relative_to(ROOT/'assets/projective-surface-v1'))]
    require(len(ledger_body) == pin['bytes'] and sha(ledger_body) == pin['sha256'], 'canonical default ledger')
    ledger = json.loads(gzip.decompress(ledger_body))
    require(not ledger['ground_truth_parsed'] and len(ledger['frames']) == 180, 'pre-label archived sensor outputs')
    labels = np.loadtxt(io.BytesIO(read('groundtruth.txt')), comments='#')
    require(labels.ndim == 2 and labels.shape[1] == 8 and np.isfinite(labels).all()
            and np.all(np.diff(labels[:, 0]) > 0), 'finite chronological offline reference')
    truth = [reference.truth_at(labels, f['depth_timestamp']) for f in manifest['frames']]
    write(output/(case+'-freeze.json'), dict(manifest_sha256=sha(body), ledger_sha256=sha(ledger_body),
        raw_input_sha256={k: v['sha256'] for k, v in inventory.items()},
        role='offline prescribed-transform decomposition, not runtime reference input'))
    maps = {}; weights = {}; key = 0; rows = []; fusion_counts = []
    for i, (frame, row) in enumerate(zip(manifest['frames'], ledger['frames'], strict=True)):
        require(row['source_index'] == frame['source_index'], 'complete indexed chronology')
        update = row['update']; require(update['accepted'] and not update['lost'], 'fixed accepted baseline schedule')
        image = Image.open(io.BytesIO(read(frame['depth_file'])))
        require(image.size == (640, 480) and image.mode in ('I;16', 'I;16B', 'I'), 'measured axial depth')
        raw = np.asarray(image, dtype=float)/5000.
        if i == 0:
            for mode in ('estimated_nearest', 'reference_nearest', 'reference_mean'):
                maps[mode] = raw.copy(); weights[mode] = (raw > .3).astype(np.uint8)
            anchor = raw.copy()
            continue
        expected = np.linalg.inv(truth[key])@truth[i]
        if i in PROBES:
            channels = {'raw_anchor': anchor, **maps}
            rows.append(dict(source_index=frame['source_index'], physical_keyframe_source_index=manifest['frames'][key]['source_index'],
                channels={name: probe(depth, raw, expected) for name, depth in channels.items()}))
        estimated = pose(update['iterations'][-1]['after'])
        counts = {}
        for mode in maps:
            method = mean if mode == 'reference_mean' else original
            transform = estimated if mode == 'estimated_nearest' else expected
            warp = None
            if update['keyframe_replaced']:
                maps[mode], weights[mode], warp = method.raster(maps[mode], weights[mode], np.linalg.inv(transform), CAMERA)
                transform = np.eye(4)
            maps[mode], weights[mode], count = method.fuse(maps[mode], weights[mode], raw, transform, CAMERA)
            count['warp'] = warp
            if mode == 'estimated_nearest':
                require(count == update['fusion'], 'exact legacy fusion replay partitions')
            counts[mode] = count
        fusion_counts.append(dict(source_index=frame['source_index'], channels=counts))
        if update['keyframe_replaced']:
            key = i; anchor = raw.copy()
        if i % 30 == 0:
            print(case, 'offline model updates', i, flush=True)
    summary = {}
    for mode in rows[0]['channels']:
        equations = [r['channels'][mode] for r in rows]
        corrections = np.array([e['correction_at_reference_pose'] for e in equations if e['correction_at_reference_pose'] is not None])
        require(len(corrections) > 0, 'observable offline probe equations')
        summary[mode] = dict(probes=len(equations), observable=len(corrections),
            median_rms_m=float(np.median([e['rms_m'] for e in equations])),
            median_position_correction_m=float(np.median(np.linalg.norm(corrections[:, :3], axis=1))),
            median_rotation_correction_rad=float(np.median(np.linalg.norm(corrections[:, 3:], axis=1))))
    write(output/(case+'.json'), dict(summary=summary, probes=rows, model_updates=fusion_counts,
        reference_fusion_is_counterfactual=True, same_keyframe_schedule=True, operational_accuracy_claim=False))
    print(case, json.dumps(summary), flush=True)
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--controls-only', action='store_true')
    p.add_argument('--design', type=Path, help='optional fixed experiment design to bind in the freeze')
    args = p.parse_args(); output = fresh(args.output)
    sources = [Path(__file__), ROOT/'scripts/projective_fusion_raw.py', ROOT/'scripts/projective_depth_raw.py',
        ROOT/'scripts/projective_depth_common.py', ROOT/'scripts/check-opencv-rgbd.py']
    if args.design is not None:
        sources.append(args.design.absolute())
    pins = {str(path.relative_to(ROOT)): sha(path.read_bytes()) for path in sources}
    write(output/'freeze.json', dict(source_sha256=pins, fixed_probe_offsets=list(PROBES),
        offline_only=True, numeric_reference_allowed_only_in_diagnostic=True))
    controls = analytic(); write(output/'controls.json', controls)
    cases = {}
    if not args.controls_only:
        spec = importlib.util.spec_from_file_location('reference', ROOT/'scripts/check-opencv-rgbd.py')
        reference = importlib.util.module_from_spec(spec); spec.loader.exec_module(reference); reference.controls()
        for case in CASES:
            cases[case] = recording(case, output, reference)
    require(all(sha((ROOT/name).read_bytes()) == h for name, h in pins.items()), 'frozen diagnostic source')
    write(output/'summary.json', dict(pass_diagnostic_integrity=True, controls=controls, cases=cases,
        root_accuracy_improvement_established=False, physical_noise_calibration_established=False))


if __name__ == '__main__':
    main()
