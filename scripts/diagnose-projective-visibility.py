#!/usr/bin/env python3
"""Fixed-support map-shift probes and raw-view visibility on viewed recordings."""
import argparse
import gzip
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from projective_depth_common import CAMERA, ROOT, fresh, pose, require, sha, write
from projective_depth_raw import compare_equation, fuse, normal_equation, pyramid, raster
from projective_visibility_raw import controls, observations

CASES = ('tum-fr1-room-temporal', 'tum-fr1-desk2-independent', 'tum-fr1-rpy-heldout')


def pairs(target, source, transform):
    ids = np.arange(source['depth'].size).reshape(source['depth'].shape)[::2, ::2].ravel()
    ps, ns = (source[k].reshape(-1, 3)[ids] for k in ('vertices', 'normals'))
    valid = np.sum(ns*ns, axis=1) > .25
    ids, ps, ns = ids[valid], ps[valid], ns[valid]
    q = ps@transform[:3, :3].T+transform[:3, 3]; ns = ns@transform[:3, :3].T
    valid = np.isfinite(q).all(axis=1)&(q[:, 2] > .3)
    ids, q, ns = ids[valid], q[valid], ns[valid]
    c = target['camera']
    x = np.rint(c['fx']*q[:, 0]/q[:, 2]+c['cx']); y = np.rint(c['fy']*q[:, 1]/q[:, 2]+c['cy'])
    valid = (x >= 1)&(x < c['width']-1)&(y >= 1)&(y < c['height']-1)
    ids, q, ns = ids[valid], q[valid], ns[valid]
    cells = y[valid].astype(np.int64)*c['width']+x[valid].astype(np.int64)
    point, n = (target[k].reshape(-1, 3)[cells] for k in ('vertices', 'normals'))
    valid = (np.sum(n*n, axis=1) > .25)&(np.linalg.norm(q-point, axis=1) < .08)
    valid &= np.einsum('ij,ij->i', n, ns) > .8
    return dict(ids=ids[valid], cells=cells[valid], query=q[valid], target=point[valid], normal=n[valid])


def equation(j, residual, weights=None):
    w = np.minimum(1., .01/np.maximum(abs(residual), 1e-15)) if weights is None else weights
    return dict(count=len(residual), rms_m=float(np.sqrt(np.mean(residual**2))), h=j.T@(w[:, None]*j), g=j.T@(w*residual))


def linear(e):
    eigen = np.linalg.eigvalsh(e['h'])
    require(e['count'] >= 200 and eigen[0] > 0 and eigen[-1]/eigen[0] <= 1e6, 'observable paired snapshot')
    return np.linalg.solve(e['h'], -e['g'])


def serial(e):
    return {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in e.items()}


def paired_probe(anchor, model, source, transform):
    a, m = pairs(anchor, source, transform), pairs(model, source, transform)
    full = {}
    for label, sample, target in (('raw_anchor', a, anchor), ('reference_fused', m, model)):
        n, q = sample['normal'], sample['query']
        e = equation(np.column_stack([n, np.cross(q, n)]), np.einsum('ij,ij->i', n, q-sample['target']))
        compare_equation(e, normal_equation(target, source, transform, 1)); full[label] = serial(e)
    ids, ia, im = np.intersect1d(a['ids'], m['ids'], return_indices=True)
    require(len(ids) >= 200, 'nonempty original common support')
    np.testing.assert_array_equal(a['cells'][ia], m['cells'][im])
    np.testing.assert_array_equal(a['query'][ia], m['query'][im])
    q, p0, p1, n = a['query'][ia], a['target'][ia], m['target'][im], a['normal'][ia]
    j = np.column_stack([n, np.cross(q, n)])
    r0 = np.einsum('ij,ij->i', n, q-p0); dr = np.einsum('ij,ij->i', n, p0-p1)
    w = np.minimum(1., .01/np.maximum(abs(r0), 1e-15))
    e0, shift, frozen = equation(j, r0, w), equation(j, dr, w), equation(j, r0+dr, w)
    d0, ds, df = linear(e0), linear(shift), linear(frozen)
    np.testing.assert_allclose(df, d0+ds, atol=1e-12, rtol=1e-10)
    nf = m['normal'][im]; jf = np.column_stack([nf, np.cross(q, nf)])
    changed = equation(jf, np.einsum('ij,ij->i', nf, q-p1)); dc = linear(changed)
    angles = np.arccos(np.clip(np.einsum('ij,ij->i', n, nf), -1, 1))
    return dict(full_support_equations=full, common_pairs=len(ids),
        frozen_raw_normal_equations={k: serial(v) for k, v in (('anchor', e0), ('map_shift', shift), ('fused', frozen))},
        correction={k: v.tolist() for k, v in (('anchor', d0), ('map_shift', ds), ('fused_fixed_normal', df), ('fused_updated_normal', dc))},
        common_fused_updated_normal_equation=serial(changed),
        axial_map_shift_mean_m=float(np.mean(p1[:, 2]-p0[:, 2])),
        plane_map_shift_rms_m=float(np.sqrt(np.mean(dr*dr))), normal_change_median_rad=float(np.median(angles)),
        same_pixel_support_and_additive_fixed_system_verified=True)


def math_controls():
    spec = importlib.util.spec_from_file_location('scene', ROOT/'scripts/check-projective-depth.py')
    scene = importlib.util.module_from_spec(spec); spec.loader.exec_module(scene)
    anchor = pyramid(scene.render(np.eye(4)), scene.CAMERA)[1]
    for axis in range(6):
        delta = np.zeros(6); delta[axis] = .001
        model = {k: v.copy() if isinstance(v, np.ndarray) else v for k, v in anchor.items()}
        model['vertices'] = anchor['vertices']+delta[:3]+np.cross(delta[3:], anchor['vertices'])
        report = paired_probe(anchor, model, anchor, np.eye(4))
        np.testing.assert_allclose(report['correction']['anchor'], np.zeros(6), atol=1e-12, rtol=0)
        np.testing.assert_allclose(report['correction']['map_shift'], delta, atol=1e-12, rtol=0)
    return dict(known_linearized_axes=6, fixed_pixel_support=True, fixed_normal_system_additive=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--controls-only', action='store_true')
    parser.add_argument('--design', type=Path, help='optional fixed design to bind in the freeze')
    args = parser.parse_args(); out = fresh(args.output)
    sources = [Path(__file__), ROOT/'scripts/projective_visibility_raw.py', ROOT/'scripts/projective_depth_raw.py',
               ROOT/'scripts/projective_depth_common.py', ROOT/'scripts/check-opencv-rgbd.py',
               ROOT/'scripts/check-projective-depth.py']
    if args.design is not None:sources.append(args.design.absolute())
    pins = {str(f.relative_to(ROOT)): sha(f.read_bytes()) for f in sources}
    write(out/'freeze.json', dict(source_sha256=pins, offline_reference_only=True, probe_offsets=list(range(1, 180, 12))))
    write(out/'controls.json', controls())
    write(out/'math-controls.json', math_controls())
    spec = importlib.util.spec_from_file_location('ref', ROOT/'scripts/check-opencv-rgbd.py')
    ref = importlib.util.module_from_spec(spec); spec.loader.exec_module(ref); ref.controls()
    summaries = {}
    for case in (() if args.controls_only else CASES):
        base = ROOT/'data'/case; manifest_body = (base/'manifest.json').read_bytes(); manifest = json.loads(manifest_body)
        inventory = {f['file']: f for f in manifest['files']}
        def read(name):
            path = base/'raw'/name; require(path.is_file() and not path.is_symlink(), 'regular qualified original input')
            body = path.read_bytes(); pin = inventory[name]
            require(len(body) == pin['bytes'] and sha(body) == pin['sha256'], 'qualified original bytes')
            return body
        ledger_path = ROOT/'assets/projective-surface-v1/recorded/first-default'/case/'sensor-only.json.gz'
        ledger_body = ledger_path.read_bytes()
        pin = json.loads((ROOT/'assets/projective-surface-v1/files.json').read_text())[str(ledger_path.relative_to(ROOT/'assets/projective-surface-v1'))]
        require(len(ledger_body) == pin['bytes'] and sha(ledger_body) == pin['sha256'], 'canonical baseline sensor ledger')
        ledger = json.loads(gzip.decompress(ledger_body)); require(not ledger['ground_truth_parsed'], 'pre-label native outputs')
        labels = np.loadtxt(io.BytesIO(read('groundtruth.txt')), comments='#')
        require(labels.ndim == 2 and labels.shape[1] == 8 and np.isfinite(labels).all() and np.all(np.diff(labels[:, 0]) > 0), 'chronological offline reference')
        truth = [ref.truth_at(labels, f['depth_timestamp']) for f in manifest['frames']]
        write(out/(case+'-freeze.json'), dict(manifest_sha256=sha(manifest_body), sensor_ledger_sha256=sha(ledger_body),
              raw_input_sha256={k: v['sha256'] for k, v in inventory.items()}))
        maps = {}; weights = {}; key = 0; probes = []; rows = []
        for i, (frame, saved) in enumerate(zip(manifest['frames'], ledger['frames'], strict=True)):
            u = saved['update']; require(u['accepted'] and not u['lost'] and saved['source_index'] == frame['source_index'], 'fixed complete accepted chronology')
            image = Image.open(io.BytesIO(read(frame['depth_file'])))
            require(image.size == (640, 480) and image.mode in ('I;16', 'I;16B', 'I'), 'original axial depth')
            raw = np.asarray(image, dtype=float)/5000.
            if i == 0:
                for mode in ('estimated', 'reference'):
                    maps[mode] = raw.copy(); weights[mode] = (raw > .3).astype(np.uint8)
                anchor = raw.copy(); continue
            source = pyramid(raw, CAMERA)[1]
            expected = np.linalg.inv(truth[key])@truth[i]; estimated = pose(u['iterations'][-1]['after'])
            if i in range(1, 180, 12):
                probe = paired_probe(pyramid(anchor, CAMERA)[1], pyramid(maps['reference'], CAMERA)[1], source, expected)
                probes.append(dict(source_index=frame['source_index'], keyframe_source_index=manifest['frames'][key]['source_index'], **probe))
            row = dict(source_index=frame['source_index'], visibility={})
            for mode, transform in (('estimated', estimated), ('reference', expected)):
                free, stats = observations(maps[mode], weights[mode], raw, source['normals'], np.linalg.inv(transform), CAMERA)
                credits = weights[mode].ravel()[free]
                row['visibility'][mode] = dict(**stats, free_support_one=int((credits == 1).sum()),
                                              free_support_sixteen=int((credits == 16).sum()))
                warp = None
                if u['keyframe_replaced']:
                    maps[mode], weights[mode], warp = raster(maps[mode], weights[mode], np.linalg.inv(transform), CAMERA)
                    transform = np.eye(4)
                maps[mode], weights[mode], counts = fuse(maps[mode], weights[mode], raw, transform, CAMERA)
                counts['warp'] = warp
                if mode == 'estimated':
                    require(counts == u['fusion'], 'exact baseline fused-map replay')
            rows.append(row)
            if u['keyframe_replaced']:
                key = i; anchor = raw.copy()
            if i % 30 == 0:
                print(case, 'diagnosed frames', i, flush=True)
        summary = dict(probes=len(probes), paired_common_matches=sum(v['common_pairs'] for v in probes), visibility={})
        for mode in ('estimated', 'reference'):
            sums = {k: sum(row['visibility'][mode][k] for row in rows) for k in ('tested', 'consistent', 'occluded', 'free_space', 'free_support_one', 'free_support_sixteen')}
            sums['free_space_fraction'] = sums['free_space']/sums['tested']; summary['visibility'][mode] = sums
        for channel in ('anchor', 'map_shift', 'fused_fixed_normal', 'fused_updated_normal'):
            d = np.array([v['correction'][channel] for v in probes])
            summary[channel] = dict(median_position_response_m=float(np.median(np.linalg.norm(d[:, :3], axis=1))),
                                    median_rotation_response_rad=float(np.median(np.linalg.norm(d[:, 3:], axis=1))))
        summary['median_plane_map_shift_rms_m'] = float(np.median([v['plane_map_shift_rms_m'] for v in probes]))
        write(out/(case+'.json'), dict(summary=summary, probes=probes, visibility=rows, operational_reference_pose_input=False))
        summaries[case] = summary; print(case, json.dumps(summary), flush=True)
    require(all(sha((ROOT/n).read_bytes()) == h for n, h in pins.items()), 'diagnostic sources unchanged')
    write(out/'summary.json', dict(pass_diagnostic_integrity=True, cases=summaries, physical_error_cause_identified=False,
        physical_noise_calibration=False, ground_truth_operational=False))


if __name__ == '__main__':
    main()
