#!/usr/bin/env python3
"""Research-only AV1.1 measured-cloud projection/raster consistency experiment.

The original KITTI-format CLI is exercised via an explicit coordinate/schema
bridge. No KITTI capture, matched camera image, or independent depth GT is used.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def write(path, document):
    with path.open('x') as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
        stream.write('\n')


def read_ply(raw):
    header, body = raw.split(b'end_header\n', 1)
    lines = header.decode('ascii').splitlines()
    require(lines[:2] == ['ply', 'format binary_little_endian 1.0'], 'bounded binary little-endian PLY')
    require(len(lines) == 8 and lines[2].startswith('element vertex '), 'exact vertex-only PLY header')
    require(lines[3:] == ['property float x', 'property float y', 'property float z',
                         'property uchar intensity', 'property ushort laser_number'], 'fixed measured PLY fields')
    count = int(lines[2].split()[-1])
    require(0 < count <= 200000 and len(body) == count*15, 'complete bounded packed PLY rows')
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', 'u1'), ('laser', '<u2')])
    records = np.frombuffer(body, dtype=dtype)
    points = np.column_stack([records[key] for key in ('x', 'y', 'z', 'i')]).astype('<f4')
    require(np.isfinite(points).all(), 'finite measured points/intensity')
    return points


def camera_model(config):
    camera = next(item['value'] for item in config['camera_data_']
                  if item['key'] == 'image_raw_ring_front_center')
    q = np.asarray(camera['vehicle_SE3_camera_']['rotation']['coefficients'], dtype=float)
    require(q.shape == (4,) and np.isfinite(q).all() and np.linalg.norm(q) > 1e-12, 'valid scalar-first quaternion')
    q /= np.linalg.norm(q)
    w, x, y, z = q
    ego_from_camera = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    position = np.asarray(camera['vehicle_SE3_camera_']['translation'], dtype=float)
    rotation, translation = ego_from_camera.T, -ego_from_camera.T@position
    k = np.array([[camera['focal_length_x_px_'], camera['skew_'], camera['focal_center_x_px_']],
                  [0., camera['focal_length_y_px_'], camera['focal_center_y_px_']], [0., 0., 1.]])
    return k, rotation, translation, q, position


def inverse_quaternion_points(points, q, position):
    # Independent vector rotation of inverse(camera->ego), without the matrix
    # expression used by the format bridge and the Rust calibration loader.
    vector = points[:, :3].astype(float)-position
    imaginary = -q[1:]
    return vector+2*np.cross(imaginary, np.cross(imaginary, vector)+q[0]*vector)


def controls():
    q = np.array([2**-.5, 0., 0., 2**-.5])
    actual = inverse_quaternion_points(np.array([[1., 3., 3., 0.]]), q, np.array([1., 2., 3.]))
    np.testing.assert_allclose(actual, [[1., 0., 0.]], atol=1e-14, rtol=0)
    body = np.array([(1., -2., 3., 127, 63)], dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('i','u1'),('l','<u2')]).tobytes()
    header = b'ply\nformat binary_little_endian 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nproperty uchar intensity\nproperty ushort laser_number\nend_header\n'
    np.testing.assert_array_equal(read_ply(header+body), [[1., -2., 3., 127.]])
    return dict(inverse_quaternion_hand_control=True, packed_ply_hand_control=True)


def bridge(directory, k, rotation, translation):
    """The input frame is Argoverse ego; the selected virtual cam0 is its real
    undistorted front-center camera. Extra schema cameras/native/IMU fields are
    nonoperational placeholders, explicitly not KITTI factory calibration.
    """
    line = lambda key, values: key+': '+' '.join(format(float(v), '.17g') for v in np.asarray(values).ravel())
    text = ['calib_time: AV1.1 schema bridge; not a KITTI calibration or acquisition']
    projection = np.column_stack((k, np.zeros(3)))
    for i in range(4):
        for key, values in [('S', [1920, 1200]), ('K', k), ('D', [0]*5), ('R', np.eye(3)),
                            ('T', [0]*3), ('S_rect', [1920, 1200]), ('R_rect', np.eye(3)), ('P_rect', projection)]:
            text.append(line(f'{key}_{i:02}', values))
    (directory/'camera.txt').write_text('\n'.join(text)+'\n')
    (directory/'ego-to-camera.txt').write_text(line('R', rotation)+'\n'+line('T', translation)+'\n')
    (directory/'unused-imu.txt').write_text(line('R', np.eye(3))+'\n'+line('T', [0]*3)+'\n')


def independently_check(points, model, projection, depth):
    k, _, _, q, position = model
    camera = inverse_quaternion_points(points, q, position)
    positive = camera[:, 2] > 0
    pixels = np.full((len(points), 2), np.nan)
    homogeneous = camera[positive]@k.T
    pixels[positive] = homogeneous[:, :2]/homogeneous[:, 2, None]
    inside = positive & (pixels[:, 0] >= 0) & (pixels[:, 0] < 1920) & (pixels[:, 1] >= 0) & (pixels[:, 1] < 1200)
    rows = projection['rows']
    require(len(rows) == len(points), 'all measured source rows retained')
    actual_camera = np.array([row['camera_xyz_m'] for row in rows])
    np.testing.assert_allclose(actual_camera, camera, atol=1e-10, rtol=1e-12)
    require(all(row['index'] == i for i, row in enumerate(rows)), 'source row indices preserved')
    np.testing.assert_array_equal([row['velodyne_xyz_m'] for row in rows], points[:, :3].astype(float))
    np.testing.assert_array_equal([row['reflectance'] for row in rows], points[:, 3].astype(float))
    actual_pixels = np.array([row['pixel'] for row in rows if row['pixel'] is not None])
    np.testing.assert_allclose(actual_pixels, pixels[positive], atol=1e-8, rtol=1e-12)
    require([row['status'] for row in rows] ==
            ['inframe' if inside[i] else 'outside' if positive[i] else 'behind' for i in range(len(points))],
            'exact half-open projection statuses')
    indices = np.flatnonzero(inside)
    cells = np.floor(pixels[inside]).astype(int)
    ordered = np.lexsort((indices, camera[indices, 2], cells[:, 0], cells[:, 1]))
    ordered_cells, ordered_ids = cells[ordered], indices[ordered]
    beginnings = np.r_[True, np.any(ordered_cells[1:] != ordered_cells[:-1], axis=1)]
    starts = np.flatnonzero(beginnings)
    winner_ids, winner_cells = ordered_ids[starts], ordered_cells[starts]
    support = np.diff(np.r_[starts, len(ordered_ids)])
    actual = depth['depth']
    require(actual['rectified_size'] == [1920, 1200], 'unchanged full canvas')
    actual_cells = actual['pixels']
    np.testing.assert_array_equal([cell['pixel'] for cell in actual_cells], winner_cells)
    np.testing.assert_array_equal([cell['point_index'] for cell in actual_cells], winner_ids)
    np.testing.assert_array_equal([cell['support_count'] for cell in actual_cells], support)
    np.testing.assert_allclose([cell['depth_m'] for cell in actual_cells], camera[winner_ids, 2], atol=1e-10, rtol=1e-12)
    winning_for_row = np.full(len(points), -1)
    winning_for_row[ordered_ids] = np.repeat(winner_ids, support)
    is_winner = np.zeros(len(points), dtype=bool)
    is_winner[winner_ids] = True
    require(len(actual['assignments']) == len(points), 'complete assignment denominator')
    for i, assignment in enumerate(actual['assignments']):
        status = 'selected' if is_winner[i] else 'occluded' if inside[i] else 'outside' if positive[i] else 'behind'
        require(assignment['point_index'] == i and assignment['status'] == status, 'every point assignment')
        require(assignment['selected_point_index'] == (int(winning_for_row[i]) if inside[i] else None), 'every winner link')
        require(assignment['pixel'] == (np.floor(pixels[i]).astype(int).tolist() if inside[i] else None), 'every cell link')
    expected = dict(points=len(points), inframe_points=int(inside.sum()), occupied_pixels=len(winner_ids),
                    occluded_points=int(inside.sum())-len(winner_ids),
                    outside=int((positive & ~inside).sum()), behind=int((~positive).sum()))
    require(actual['summary'] == expected, 'complete summary counts')
    return dict(**expected, density=len(winner_ids)/(1920*1200),
        max_camera_coordinate_error_m=float(np.max(np.abs(actual_camera-camera))),
        max_inframe_pixel_error_px=float(np.max(np.abs(np.array([row['pixel'] for row in rows if row['status']=='inframe'])-pixels[inside]))),
        scope='Independent mathematical consistency on recorded points/calibration; not physical depth error')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--projection-binary', type=Path, required=True)
    parser.add_argument('--depth-binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.absolute()
    require(not output.exists() and not output.is_symlink() and
            not any(p.is_symlink() for p in output.parents), 'fresh real output directory required')
    output.mkdir(parents=True)
    proof_controls = controls()
    raw = args.raw.absolute()
    acquisition = json.loads((raw/'acquisition.json').read_text())
    items = acquisition['points']+[acquisition['calibration']]
    for item in items:
        require(Path(item['file']).name == item['file'], 'plain raw file name')
        path = raw/item['file']
        require(path.is_file() and not path.is_symlink() and path.stat().st_size <= 5*1024*1024, 'bounded regular raw file')
        body = path.read_bytes()
        require(len(body) == item['bytes'] and digest(body) == item['sha256'], 'selected raw payload identity')
    binaries = [args.projection_binary.absolute(), args.depth_binary.absolute()]
    write(output/'freeze.json', dict(acquisition=acquisition, sources_sha256={
        str(path.relative_to(ROOT)):digest(path.read_bytes()) for path in [Path(__file__), ROOT/'scripts/fetch-argoverse-depth-sample.py']},
        binaries_sha256={str(path):digest(path.read_bytes()) for path in binaries},
        camera='image_raw_ring_front_center', canvas=[1920,1200],
        coordinate_bridge='Argoverse ego XYZ -> source undistorted front-center camera via inverse vehicle_SE3_camera; virtual KITTI cam0; shared rectification identity',
        native_extra_camera_and_imu_fields='nonoperational schema placeholders; not physical KITTI calibration',
        geometric_absolute_tolerance_m=1e-10, pixel_absolute_tolerance_px=1e-8, relative_tolerance=1e-12,
        raster_indices_winners_counts_tolerance=0, references='independent quaternion/vector projection and NumPy grouped sorting; no physical GT',
        controls=proof_controls, measured_kitti_accuracy=False, source_license='CC BY-NC-SA 4.0; © 2018-2019 Argo AI, LLC'))
    calibration = json.loads((raw/'vehicle_calibration_info.json').read_text())
    model = camera_model(calibration)
    bridge(output, *model[:3])
    outcomes, journal = [], []
    for item in acquisition['points']:
        points = read_ply((raw/item['file']).read_bytes())
        name = Path(item['file']).stem
        directory = output/name
        directory.mkdir()
        point_file = directory/'points.bin'
        point_file.write_bytes(points.tobytes())
        outcome = dict(file=item['file'], source_points=len(points), integrity_passed=False)
        outcomes.append(outcome)
        try:
            for suffix in ('first','repeat'):
                projection_path, depth_path = directory/(suffix+'-projection.json'), directory/(suffix+'-depth.json')
                commands = [[binaries[0], '--calib-cam', output/'camera.txt', '--calib-velo', output/'ego-to-camera.txt',
                             '--calib-imu', output/'unused-imu.txt', '--points', point_file, '--camera', '0', '--output', projection_path],
                            [binaries[1], '--projection', projection_path, '--output', depth_path]]
                for command in commands:
                    result = subprocess.run([str(v) for v in command], text=True, capture_output=True, timeout=120)
                    journal.append(dict(command=[str(v) for v in command], exit_code=result.returncode,
                                        stdout=result.stdout, stderr=result.stderr))
                    require(result.returncode == 0, 'actual CLI rejection: '+result.stderr)
                outcome[suffix] = independently_check(points, model, json.loads(projection_path.read_bytes()), json.loads(depth_path.read_bytes()))
            for kind in ('projection','depth'):
                require((directory/('first-'+kind+'.json')).read_bytes() == (directory/('repeat-'+kind+'.json')).read_bytes(), 'byte-identical '+kind+' replay')
            outcome.update(integrity_passed=True, byte_identical_projection_and_depth=True)
        except (ValueError, KeyError, AssertionError, OSError, subprocess.SubprocessError) as error:
            outcome['failure'] = str(error)
        write(directory/'check.json', outcome)
    proof = dict(schema_version=1, dataset='Argoverse 1.1 official tracking sample',
        source_archive_etag=acquisition['archive_etag'], source_log=acquisition['log'],
        selected_scans=len(outcomes), retained_source_points=sum(item['source_points'] for item in outcomes),
        all_integrity_checks_passed=all(item['integrity_passed'] for item in outcomes), scans=outcomes,
        actual_kitti_data=False, independent_physical_reference_available=False,
        measured_depth_rmse_m=None, measured_depth_mae_m=None,
        scope='Recorded points/calibration, explicit format bridge, mathematical projection/raster consistency only')
    write(output/'journal.json', journal)
    write(output/'results.json', proof)
    print(json.dumps(proof))
    return 0 if proof['all_integrity_checks_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
