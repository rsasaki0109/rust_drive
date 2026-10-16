#!/usr/bin/env python3
"""Independent analytic projection-to-sparse-depth checks; no KITTI accuracy claim."""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def expected_depth(size, rows):
    """Group the independently projected rows, then sort each cell by Z/ID."""
    groups = {}
    for row in rows:
        if row['status'] == 'inframe':
            u, v = row['pixel']
            groups.setdefault((math.floor(v), math.floor(u)), []).append(row)
    pixels, winners = [], {}
    for (y, x), members in sorted(groups.items()):
        ordered = sorted(members, key=lambda row: (row['camera_xyz_m'][2], row['index']))
        winner = ordered[0]
        pixels.append(dict(pixel=[x, y], depth_m=winner['camera_xyz_m'][2],
                           point_index=winner['index'], support_count=len(members)))
        for row in members:
            winners[row['index']] = (winner['index'], [x, y])
    assignments = []
    for row in rows:
        winner, pixel = winners.get(row['index'], (None, None))
        status = row['status'] if winner is None else (
            'selected' if winner == row['index'] else 'occluded')
        assignments.append(dict(point_index=row['index'], status=status, pixel=pixel,
                                selected_point_index=winner))
    inframe = sum(row['status'] == 'inframe' for row in rows)
    return dict(rectified_size=size, pixels=pixels, assignments=assignments,
                summary=dict(points=len(rows), inframe_points=inframe, occupied_pixels=len(pixels),
                             occluded_points=inframe-len(pixels),
                             outside=sum(row['status'] == 'outside' for row in rows),
                             behind=sum(row['status'] == 'behind' for row in rows)))


def hand_report():
    # Deliberately preprojected rows test raster rules, independently of calibration.
    values = [([0., 0., 5.], [0.9, 0.9], 'inframe'),
              ([20., 0., 4.], [0.1, 0.1], 'inframe'),
              ([0., 0., 4.], [0., 0.], 'inframe'),
              ([0., 0., 2.], [1., 0.], 'inframe'),
              ([0., 0., 1.], [3.99, 2.99], 'inframe'),
              ([0., 0., 1.], [4., 0.], 'outside'),
              ([0., 0., 1.], [0., 3.], 'outside'),
              ([0., 0., 1.], [-0.01, 0.], 'outside'),
              ([0., 0., 0.], None, 'behind'),
              ([0., 0., -1.], None, 'behind')]
    rows = [dict(index=i, velodyne_xyz_m=[0., 0., 0.], reflectance=0.5,
                 camera_xyz_m=xyz, pixel=pixel, status=status)
            for i, (xyz, pixel, status) in enumerate(values)]
    return dict(schema_version=1, kind='kitti_processed_projection_analytic_tool', camera_index=0,
                effective_model=dict(camera_index=0, rectified_size=[4, 3]), rows=rows,
                summary=dict(points=10, inframe=5, outside=3, behind=2))


def check_report(actual, expected, source, camera, binary_source, rasterizer_source):
    require(actual['schema_version'] == 1 and actual['kind'] == 'kitti_sparse_axial_depth',
            'depth schema')
    require(actual['camera_index'] == camera, 'selected camera')
    require(actual['input_projection'] == dict(bytes=source.stat().st_size, sha256=digest(source)),
            'exact input receipt')
    require(actual['sources'] == dict(binary=digest(binary_source), rasterizer=digest(rasterizer_source),
                cargo_manifest=digest(ROOT/'integrations/kitti-depth/Cargo.toml'),
                cargo_lock=digest(ROOT/'integrations/kitti-depth/Cargo.lock')),
            'compiled source receipt')
    depth = actual['depth']
    require(depth['rectified_size'] == expected['rectified_size'], 'canvas')
    require(depth['summary'] == expected['summary'], 'complete counts')
    require(depth['assignments'] == expected['assignments'], 'all row assignments and winner links')
    require(len(depth['pixels']) == len(expected['pixels']), 'complete occupied cells')
    for actual_pixel, expected_pixel in zip(depth['pixels'], expected['pixels']):
        require(set(actual_pixel) == set(expected_pixel), 'pixel schema')
        require(all(actual_pixel[key] == expected_pixel[key]
                    for key in ('pixel', 'point_index', 'support_count')), 'cell order/support/winner')
        require(type(actual_pixel['depth_m']) in (int, float)
                and math.isfinite(actual_pixel['depth_m'])
                and abs(actual_pixel['depth_m']-expected_pixel['depth_m']) <= 1e-8,
                'independent axial depth (absolute tolerance 1e-8 m)')


def audit(args):
    output = args.output.absolute()
    require(not output.exists() and not output.is_symlink(), 'fresh output directory required')
    require(not any(parent.is_symlink() for parent in output.parents), 'real output parents required')
    projection_binary = args.projection_binary.absolute()
    depth_binary = args.depth_binary.absolute()
    for binary in (projection_binary, depth_binary):
        require(binary.is_file() and not binary.is_symlink(), 'regular executable required')
    spec = importlib.util.spec_from_file_location('projection_oracle', ROOT/'scripts/check-kitti-calibration.py')
    oracle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(oracle)
    require(oracle.np.__version__ == '2.3.5', 'pinned NumPy 2.3.5 required')
    output.mkdir(parents=True)
    source_path = ROOT/'integrations/kitti-depth/src/main.rs'
    module_path = ROOT/'integrations/kitti-depth/src/depth.rs'
    write(output/'inputs.json', dict(projection_binary_sha256=digest(projection_binary),
          depth_binary_sha256=digest(depth_binary), checker_sha256=digest(Path(__file__)),
          sources=dict(binary=digest(source_path), rasterizer=digest(module_path)),
          pixel_policy='floor; exact half-open bounds', depth_absolute_tolerance_m=1e-8,
          actual_kitti_data=False, measured_accuracy=False))
    fixture = output/'fixture'
    oracle.prepare_fixture(fixture)
    oracle.verify_fixture(fixture)
    journal = []

    def invoke(command, expected_code):
        result = subprocess.run([str(part) for part in command], capture_output=True, text=True, timeout=30)
        journal.append(dict(command=[str(part) for part in command], returncode=result.returncode,
                            stdout=result.stdout, stderr=result.stderr))
        require(result.returncode == expected_code, f'expected exit {expected_code}: {result.stderr}')
        return result

    def depth_run(name, source, expected, camera=0):
        target = output/(name+'.json')
        invoke([depth_binary, '--projection', source, '--output', target], 0)
        actual = json.loads(target.read_bytes())
        check_report(actual, expected, source, camera, source_path, module_path)
        repeat = output/(name+'-repeat.json')
        invoke([depth_binary, '--projection', source, '--output', repeat], 0)
        require(target.read_bytes() == repeat.read_bytes(), 'byte-identical depth replay')
        return actual

    summaries = []
    for camera in range(4):
        projection = output/f'projection-{camera}.json'
        invoke([projection_binary, '--calib-cam', fixture/'calib_cam_to_cam.txt',
                '--calib-velo', fixture/'calib_velo_to_cam.txt',
                '--calib-imu', fixture/'calib_imu_to_velo.txt', '--points', fixture/'points.bin',
                '--camera', camera, '--output', projection], 0)
        independent = oracle.independently_project(fixture, camera)
        expected = expected_depth(independent['rectified_size'], independent['rows'])
        report = depth_run(f'camera-{camera}-depth', projection, expected, camera)
        summaries.append(report['depth']['summary'])

    hand = hand_report()
    expected = expected_depth([4, 3], hand['rows'])
    # Hand witnesses verify the checker before it judges Rust output.
    require(expected['pixels'] == [
        dict(pixel=[0, 0], depth_m=4., point_index=1, support_count=3),
        dict(pixel=[1, 0], depth_m=2., point_index=3, support_count=1),
        dict(pixel=[3, 2], depth_m=1., point_index=4, support_count=1)], 'hand cell witness')
    require([row['status'] for row in expected['assignments']] ==
            ['occluded', 'selected', 'occluded', 'selected', 'selected',
             'outside', 'outside', 'outside', 'behind', 'behind'], 'hand assignments')
    source = output/'hand-projection.json'
    write(source, hand)
    depth_run('hand-depth', source, expected)

    closer = copy.deepcopy(hand)
    closer['rows'][2]['camera_xyz_m'][2] = math.nextafter(4., 0.)
    source = output/'near-tie-projection.json'
    write(source, closer)
    expected = expected_depth([4, 3], closer['rows'])
    require(expected['pixels'][0]['point_index'] == 2, 'non-epsilon near-tie witness')
    depth_run('near-tie-depth', source, expected)

    empty = copy.deepcopy(hand)
    for row in empty['rows']:
        row.update(camera_xyz_m=[0., 0., -1.], pixel=None, status='behind')
    empty['summary'] = dict(points=10, inframe=0, outside=0, behind=10)
    source = output/'empty-projection.json'
    write(source, empty)
    depth_run('empty-depth', source, expected_depth([4, 3], empty['rows']))

    # Corrupt both Rust inputs and produced reports; the latter exercise this checker.
    mutations = {
        'wrong_version': lambda p: p.update(schema_version=2),
        'wrong_kind': lambda p: p.update(kind='other'),
        'wrong_camera': lambda p: p.update(camera_index=4),
        'mismatched_camera': lambda p: p['effective_model'].update(camera_index=1),
        'empty_rows': lambda p: p.update(rows=[]),
        'bad_index': lambda p: p['rows'][0].update(index=1),
        'negative_index': lambda p: p['rows'][0].update(index=-1),
        'missing_pixel': lambda p: p['rows'][0].update(pixel=None),
        'negative_z': lambda p: p['rows'][0].update(camera_xyz_m=[0, 0, -1]),
        'behind_with_pixel': lambda p: p['rows'][8].update(pixel=[0, 0]),
        'wrong_status': lambda p: p['rows'][0].update(status='outside'),
        'right_boundary': lambda p: p['rows'][0].update(pixel=[4, 0]),
        'bottom_boundary': lambda p: p['rows'][0].update(pixel=[0, 3]),
        'negative_pixel': lambda p: p['rows'][0].update(pixel=[-0.01, 0]),
        'wrong_total': lambda p: p['summary'].update(points=9),
        'wrong_inframe': lambda p: p['summary'].update(inframe=6),
        'wrong_behind': lambda p: p['summary'].update(behind=3),
        'zero_width': lambda p: p['effective_model'].update(rectified_size=[0, 3]),
        'capacity': lambda p: p['effective_model'].update(rectified_size=[8192, 8192]),
        'dimension_bound': lambda p: p['effective_model'].update(rectified_size=[8193, 1]),
    }
    for name, mutate in mutations.items():
        invalid = copy.deepcopy(hand)
        mutate(invalid)
        source = output/(name+'-input.json')
        write(source, invalid)
        target = output/(name+'-rejected.json')
        invoke([depth_binary, '--projection', source, '--output', target], 2)
        require(target.read_bytes() == b'', 'failed input must not emit a partial depth report')
    malformed = ['{', source.read_text()+' trailing',
                 json.dumps(hand).replace('0.5', '1e999', 1),
                 json.dumps(hand).replace('0.5', 'NaN', 1),
                 json.dumps(hand).replace('"schema_version": 1',
                                          '"schema_version": 1, "schema_version": 1', 1)]
    for index, text in enumerate(malformed):
        source = output/f'malformed-{index}.json'
        source.write_text(text)
        invoke([depth_binary, '--projection', source, '--output', output/f'malformed-{index}-out.json'], 2)
    source = output/'oversized.json'
    with source.open('xb') as stream:
        stream.truncate(64*1024*1024+1)
    invoke([depth_binary, '--projection', source, '--output', output/'oversized-out.json'], 2)
    source.unlink()

    valid_source = output/'hand-projection.json'
    existing = output/'preserved.json'
    existing.write_bytes(b'preserve existing bytes')
    targets = [existing, output, output/'nested/../escape.json', output/'raw-dot/./escape.json']
    # Path normalizes '.', so retain literal spelling for that CLI control.
    targets[-1] = str(output)+'/raw-dot/./escape.json'
    links = []
    try:
        if sys.platform != 'win32':
            link = output/'linked-parent'
            link.symlink_to(output, target_is_directory=True)
            links.append(link)
            dangling = output/'dangling.json'
            dangling.symlink_to(output/'missing.json')
            links.append(dangling)
            targets += [link/'escape.json', dangling]
        for target in targets:
            result = invoke([depth_binary, '--projection', output/'missing-input.json', '--output', target], 2)
            require('output' in result.stderr, 'output admission must reject before input reads')
    finally:
        for path in links:
            path.unlink()
    require(existing.read_bytes() == b'preserve existing bytes', 'existing bytes preserved')
    require(not (output/'escape.json').exists(), 'no escaped output')
    invoke([depth_binary, '--projection', valid_source, '--projection', valid_source], 2)
    invoke([depth_binary, '--unknown', valid_source, '--output', output/'unknown.json'], 2)
    invoke([depth_binary, '--projection', valid_source], 2)
    baseline = json.loads((output/'hand-depth.json').read_bytes())
    corrupted = [
        lambda p: p['depth']['pixels'][0].update(point_index=0),
        lambda p: p['depth']['pixels'][0].update(depth_m=5.),
        lambda p: p['depth']['pixels'][0].update(support_count=2),
        lambda p: p['depth']['assignments'][0].update(selected_point_index=0),
        lambda p: p['depth']['assignments'].pop(),
        lambda p: p['depth']['summary'].update(points=9),
        lambda p: p['input_projection'].update(sha256='0'*64),
    ]
    for mutate in corrupted:
        report = copy.deepcopy(baseline)
        mutate(report)
        try:
            check_report(report, expected_depth([4, 3], hand['rows']), valid_source, 0, source_path, module_path)
        except ValueError:
            pass
        else:
            raise ValueError('independent checker accepted a corrupted depth report')
    write(output/'journal.json', journal)
    proof = dict(schema_version=1, analytic_geometry_and_contracts_passed=True,
                 camera_summaries=summaries, independently_projected_rows=72,
                 byte_identical_depth_replays=7,
                 positive_cli_runs=sum(row['returncode'] == 0 for row in journal),
                 rejected_cli_runs=sum(row['returncode'] == 2 for row in journal),
                 checker_corruptions_rejected=len(corrupted), actual_kitti_data=False,
                 measured_accuracy=False, output_directory=str(output))
    write(output/'results.json', proof)
    print(json.dumps(proof))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--projection-binary', type=Path, required=True)
    parser.add_argument('--depth-binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    audit(parser.parse_args())


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, AssertionError, subprocess.SubprocessError) as error:
        print('Independent KITTI sparse depth: '+str(error), file=sys.stderr)
        sys.exit(2)
