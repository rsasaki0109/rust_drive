#!/usr/bin/env python3
"""Additive diagnosis of the frozen first AV1.1 trial, preserving its failures."""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--first', type=Path, required=True)
    parser.add_argument('--projection-binary', type=Path, required=True)
    parser.add_argument('--depth-binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('first_av_checker', Path(__file__).with_name('check-argoverse-sparse-depth.py'))
    original = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(original)
    require, write, digest = original.require, original.write, original.digest
    output = args.output.absolute()
    require(not output.exists() and not output.is_symlink() and
            not any(p.is_symlink() for p in output.parents), 'fresh output directory required')
    output.mkdir(parents=True)
    first = args.first.absolute()
    freeze = json.loads((first/'freeze.json').read_text())
    first_result = json.loads((first/'results.json').read_text())
    # Bind the complete first result, source checker and binaries before diagnosis.
    for name, expected in freeze['sources_sha256'].items():
        require(digest((original.ROOT/name).read_bytes()) == expected, 'first source changed')
    binaries = [args.projection_binary.absolute(), args.depth_binary.absolute()]
    for path in binaries:
        require(digest(path.read_bytes()) == freeze['binaries_sha256'][str(path)], 'first binary changed')
    write(output/'freeze.json', dict(first_results_sha256=digest((first/'results.json').read_bytes()),
        first_freeze_sha256=digest((first/'freeze.json').read_bytes()),
        diagnostic_source_sha256=digest(Path(__file__).read_bytes()),
        policy='Keep original all-pixel failure. Independently audit in-frame pixels and all raster selections with the same frozen tolerances; report original off-canvas violations.'))
    raw = args.raw.absolute()
    calibration_item = freeze['acquisition']['calibration']
    calibration_bytes = (raw/calibration_item['file']).read_bytes()
    require(digest(calibration_bytes) == calibration_item['sha256'], 'original calibration changed')
    k, _, _, q, position = original.camera_model(json.loads(calibration_bytes))
    outcomes, journal = [], []
    for source in freeze['acquisition']['points']:
        source_bytes = (raw/source['file']).read_bytes()
        require(digest(source_bytes) == source['sha256'], 'original point source changed')
        points = original.read_ply(source_bytes)
        camera = original.inverse_quaternion_points(points, q, position)
        positive = camera[:, 2] > 0
        expected_pixels = np.full((len(points), 2), np.nan)
        h = camera[positive]@k.T
        expected_pixels[positive] = h[:, :2]/h[:, 2, None]
        inside = positive & (expected_pixels[:, 0] >= 0) & (expected_pixels[:, 0] < 1920) & (expected_pixels[:, 1] >= 0) & (expected_pixels[:, 1] < 1200)
        case = first/Path(source['file']).stem
        projection_bytes = (case/'first-projection.json').read_bytes()
        depth_bytes = (case/'first-depth.json').read_bytes()
        projection, depth = json.loads(projection_bytes), json.loads(depth_bytes)['depth']
        require(len(projection['rows']) == len(points), 'all original projected rows')
        actual_camera = np.array([row['camera_xyz_m'] for row in projection['rows']])
        np.testing.assert_allclose(actual_camera, camera, atol=1e-10, rtol=1e-12)
        actual_pixels = np.array([row['pixel'] for row in projection['rows'] if row['pixel'] is not None])
        differences = np.abs(actual_pixels-expected_pixels[positive])
        violations = np.any(differences > 1e-8+1e-12*np.abs(expected_pixels[positive]), axis=1)
        original_bad_ids = np.flatnonzero(positive)[violations]
        require(not inside[original_bad_ids].any(), 'original tolerance violation affects an in-frame point')
        np.testing.assert_allclose(actual_pixels[inside[positive]], expected_pixels[inside], atol=1e-8, rtol=1e-12)
        require([row['status'] for row in projection['rows']] ==
                ['inframe' if inside[i] else 'outside' if positive[i] else 'behind' for i in range(len(points))],
                'all original statuses match independently')

        ids = np.flatnonzero(inside)
        xy = np.floor(expected_pixels[inside]).astype(int)
        order = np.lexsort((ids, camera[ids, 2], xy[:, 0], xy[:, 1]))
        xy, ids = xy[order], ids[order]
        starts = np.flatnonzero(np.r_[True, np.any(xy[1:] != xy[:-1], axis=1)])
        counts = np.diff(np.r_[starts, len(ids)])
        winners = ids[starts]
        actual = depth['pixels']
        np.testing.assert_array_equal([cell['pixel'] for cell in actual], xy[starts])
        np.testing.assert_array_equal([cell['point_index'] for cell in actual], winners)
        np.testing.assert_array_equal([cell['support_count'] for cell in actual], counts)
        np.testing.assert_allclose([cell['depth_m'] for cell in actual], camera[winners, 2], atol=1e-10, rtol=1e-12)
        expected_winner = np.full(len(points), -1)
        expected_winner[ids] = np.repeat(winners, counts)
        selected = np.zeros(len(points), dtype=bool)
        selected[winners] = True
        require(len(depth['assignments']) == len(points), 'all original source assignments retained')
        for i, assignment in enumerate(depth['assignments']):
            status = 'selected' if selected[i] else 'occluded' if inside[i] else 'outside' if positive[i] else 'behind'
            require(assignment['point_index'] == i and assignment['status'] == status, 'all assignments match')
            require(assignment['selected_point_index'] == (int(expected_winner[i]) if inside[i] else None), 'all winning-row links match')
            require(assignment['pixel'] == (np.floor(expected_pixels[i]).astype(int).tolist() if inside[i] else None), 'all assignment pixel links match')
        summary = dict(points=len(points), inframe_points=int(inside.sum()), occupied_pixels=len(winners),
                       occluded_points=int(inside.sum())-len(winners), outside=int((positive & ~inside).sum()),
                       behind=int((~positive).sum()))
        require(depth['summary'] == summary, 'complete original summary')
        repeat_projection, repeat_depth = case/'repeat-projection.json', case/'repeat-depth.json'
        if not repeat_projection.exists() or not repeat_depth.exists():
            repeat = output/Path(source['file']).stem
            repeat.mkdir()
            repeat_projection, repeat_depth = repeat/'projection.json', repeat/'depth.json'
            commands = [[binaries[0], '--calib-cam', first/'camera.txt', '--calib-velo', first/'ego-to-camera.txt',
                         '--calib-imu', first/'unused-imu.txt', '--points', case/'points.bin', '--camera', '0', '--output', repeat_projection],
                        [binaries[1], '--projection', repeat_projection, '--output', repeat_depth]]
            for command in commands:
                result = subprocess.run([str(v) for v in command], text=True, capture_output=True, timeout=120)
                journal.append(dict(command=[str(v) for v in command], exit_code=result.returncode,
                                    stdout=result.stdout, stderr=result.stderr))
                require(result.returncode == 0, 'repeat CLI failure: '+result.stderr)
        require(repeat_projection.read_bytes() == projection_bytes and repeat_depth.read_bytes() == depth_bytes,
                'original projection/depth reproduce byte for byte')
        outcomes.append(dict(file=source['file'], **summary, density=len(winners)/(1920*1200),
            all_raster_cells_and_assignments_match=True, byte_identical_repeat=True,
            max_camera_coordinate_error_m=float(np.max(np.abs(actual_camera-camera))),
            max_inframe_pixel_error_px=float(np.max(differences[inside[positive]])),
            original_pixel_tolerance_violating_points=len(original_bad_ids),
            original_violations_all_outside=True,
            violating_camera_depth_range_m=[float(camera[original_bad_ids, 2].min()), float(camera[original_bad_ids, 2].max())] if len(original_bad_ids) else None,
            max_positive_depth_pixel_error_px=float(differences.max())))
    proof = dict(schema_version=1, dataset='Argoverse 1.1 official tracking sample', scans=outcomes,
                 original_all_pixel_trial_passed=first_result['all_integrity_checks_passed'],
                 original_scans_passing=sum(case['integrity_passed'] for case in first_result['scans']),
                 inframe_projection_and_all_raster_assignments_match=True,
                 original_failure_preserved=True, pixel_tolerance_changed=False,
                 all_selected_points=sum(case['points'] for case in outcomes),
                 actual_kitti_data=False, independent_physical_reference_available=False,
                 measured_depth_rmse_m=None, measured_depth_mae_m=None)
    write(output/'journal.json', journal)
    write(output/'results.json', proof)
    print(json.dumps(proof))


if __name__ == '__main__':
    main()
