#!/usr/bin/env python3
"""Independent authored KITTI-format geometry controls; no recorded KITTI accuracy.

NumPy 2.3.5 homogeneous products/inverses are independent of the Rust parser/math.
Fixed absolute tolerances, declared before the first CLI trial: matrices 1e-10,
metric points and pixels 1e-8. Half-open pixel inclusion is exact, without epsilon.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MATRIX_ATOL = 1e-10
POINT_PIXEL_ATOL = 1e-8
PAPER_SHA = 'af54d6fd1042661cd7dc01aeece435d20fc418078f6343246779f9f1b6a523f5'
FIXTURE_PINS = {
    'calib_cam_to_cam.txt': (1271, 'd43d84b23551dac2c3e8b083a70847b1eccdf510eb9c8c463539a6a63aa84fcb'),
    'calib_velo_to_cam.txt': (124, 'bd2194e3b0244c626f28c841bbf3741f9ab862a529d0bdacd397903415bdaaee'),
    'calib_imu_to_velo.txt': (100, 'ba3ffc60e382d95cb115dec9b930c224718a74b013b84ed0d143f4637118058f'),
    'points.bin': (288, 'ac2c44b8a0434728f327dc3e7f53c97288666c0e017c37451e80e147deaed1e3'),
}
SOURCE_PATHS = {
    'binary': 'integrations/rgbd/src/bin/rustdriving-kitti-project.rs',
    'calibration': 'integrations/rgbd/src/kitti_calibration.rs',
    'cargo_manifest': 'integrations/rgbd/Cargo.toml',
    'cargo_lock': 'integrations/rgbd/Cargo.lock',
    'rust_toolchain': 'rust-toolchain.toml',
    'design': 'assets/kitti-projection-v1/design.json',
    'source_authority': 'assets/kitti-projection-v1/source-authority.json',
}
DESIGN_PATH = ROOT / 'assets/kitti-projection-v1/design.json'
DESIGN_SHA = 'f19310ee577bf9e77b8b2969763d5ec58f7e0cb1884671ac076f8d987a52832c'
AUTHORITY_PATH = ROOT / 'assets/kitti-projection-v1/source-authority.json'
AUTHORITY_SHA = '2eff9f3660d23430c8639f68abac8da1161d53334a3459a2c4018d6043db5fee'
COMPILED_SOURCE_PINS = {
    'binary': '960470e075ea3a8772d753881ce8a959b2128c2067ba2bfd56230ca037176a5d',
    'calibration': '4871dd098459b18e95dccd6d7d574826cba092ede3a10850fc47e061bb5a622d',
    'design': 'f19310ee577bf9e77b8b2969763d5ec58f7e0cb1884671ac076f8d987a52832c',
    'source_authority': '2eff9f3660d23430c8639f68abac8da1161d53334a3459a2c4018d6043db5fee',
    'cargo_manifest': '5a49c3a2c04951ffbf3e154d28fe1e13d2dda92b410b15ec849bfe33377e0d5f',
    'cargo_lock': '67845fb7e4a53e15e4e3c70bba72142aff666893cf1d0c96c4654cdc16b7197b',
    'rust_toolchain': '5597a94fc41b6f56313d195195fbeb63df9eb694cfe9fa5d0a8011c999e335ed',
}
IDENTITY = np.eye(3)
RX = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
RY = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
RZ = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def fresh(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(), 'fresh output required')
    require(not any(p.is_symlink() for p in path.parents), 'symlink output parent')
    return path


def write(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def homogeneous(rotation, translation=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def fixture():
    sizes = [[640, 480], [672, 496], [640, 480], [704, 520]]
    intrinsics = [np.array(value, dtype=float) for value in (
        [[512, 16, 320], [0, 512, 240], [0, 0, 1]],
        [[480, -8, 336], [0, 544, 248], [0, 0, 1]],
        [[576, 24, 300], [0, 512, 220], [0, 0, 1]],
        [[512, -16, 352], [0, 480, 260], [0, 0, 1]])]
    offsets = [[.125, -.0625, .25], [-.5, .125, .125],
               [.25, .25, -.0625], [-.75, -.125, .5]]
    rectifications = [RY, RZ, RX, RZ @ RZ]
    native_rotations = [IDENTITY, RZ, RX, RY]
    text = ['calib_time: authored analytic fixture; not measured KITTI', 'corner_dist: 0.1']
    def line(key, values):
        return key + ': ' + ' '.join(format(float(v), '.17g') for v in np.asarray(values).ravel())
    for index in range(4):
        suffix = f'{index:02d}'
        native = [[700 + 11 * index, 0, 660 + index],
                  [0, 720 + 13 * index, 250 + index], [0, 0, 1]]
        projection = intrinsics[index] @ np.column_stack((IDENTITY, offsets[index]))
        for key, values in [
            ('S', [1392, 512]), ('K', native),
            ('D', [.1 + .01 * index, -.2, .003, -.004, .05]),
            ('R', native_rotations[index]), ('T', [index * .125, 0, index * -.0625]),
            ('S_rect', sizes[index]), ('R_rect', rectifications[index]), ('P_rect', projection)]:
            text.append(line(key + '_' + suffix, values))
    cam = '\n'.join(text) + '\n'
    velo = '\n'.join(['calib_time: authored analytic fixture; not measured KITTI',
        line('R', RZ), line('T', [.25, -.5, .75]), line('delta_f', [0, 0]),
        line('delta_c', [0, 0])]) + '\n'
    imu = '\n'.join(['calib_time: authored analytic fixture; not measured KITTI',
        line('R', RX), line('T', [-.125, .375, .5])]) + '\n'
    transform = homogeneous(IDENTITY, offsets[0]) @ homogeneous(RY) @ homogeneous(RZ, [.25, -.5, .75])
    target_pixels = [(320, 240, 4), (0, 240, 4), (640, 240, 4), (320, 0, 4),
        (320, 480, 4), (.125, 240, 4), (-.125, 240, 4), (639.875, 240, 4),
        (640.125, 240, 4), (320, .125, 4), (320, -.125, 4), (320, 479.875, 4),
        (320, 480.125, 4), (320, 240, -4), (0, 0, 0), (320, 240, 2 ** -12),
        (112, 128, 8), (576, 448, 2)]
    inverse = np.linalg.inv(transform)
    raw = bytearray()
    for index, (u, v, z) in enumerate(target_pixels):
        metric = np.linalg.inv(intrinsics[0]) @ np.array([u * z, v * z, z])
        xyz = (inverse @ np.append(metric, 1.))[:3]
        raw.extend(struct.pack('<ffff', *xyz, (index % 5) / 4))
    return {'calib_cam_to_cam.txt': cam.encode(), 'calib_velo_to_cam.txt': velo.encode(),
            'calib_imu_to_velo.txt': imu.encode(), 'points.bin': bytes(raw)}


def prepare_fixture(output):
    output = fresh(output)
    raw = fixture()
    output.mkdir(parents=True, exist_ok=False)
    for name, body in raw.items():
        with (output / name).open('xb') as stream:
            stream.write(body)
    manifest = dict(schema_version=1, role='authored_generalized_kitti_format_geometry_controls',
        actual_kitti_data=False, actual_images=False, actual_ground_truth=False,
        calibration_physically_measured=False, authored_point_frame_stamp_seconds=0.,
        stamp_role='explicit authored fixture metadata, not a measured acquisition timestamp',
        binary_encoding='little-endian float32 x,y,z,reflectance;16bytes/point', points=18,
        files={name: dict(bytes=len(body), sha256=hashlib.sha256(body).hexdigest())
               for name, body in raw.items()}, matrix_absolute_tolerance=MATRIX_ATOL,
        point_pixel_absolute_tolerance=POINT_PIXEL_ATOL, relative_tolerance=0,
        pixel_inclusion='exact0<=u<width and0<=v<height; positive target-camera axialZ',
        source_reference_paper_sha256=PAPER_SHA, auditor_sha256=digest(Path(__file__)))
    write(output / 'fixture.json', manifest)
    return manifest


def source_fields(path):
    body = Path(path).read_bytes()
    require(len(body) <= 65536, 'bounded calibration source')
    result = {}
    for line in body.decode('ascii').splitlines():
        key, value = line.split(':', 1)
        require(key not in result, 'duplicate calibration key')
        if key == 'calib_time':
            result[key] = value.strip()
        else:
            result[key] = np.array([float(v) for v in value.split()])
            require(np.isfinite(result[key]).all(), 'nonfinite calibration source')
    return result


def independently_project(directory, camera_index):
    directory = Path(directory)
    camera = source_fields(directory / 'calib_cam_to_cam.txt')
    velo = source_fields(directory / 'calib_velo_to_cam.txt')
    imu = source_fields(directory / 'calib_imu_to_velo.txt')
    cam0_from_velo = homogeneous(velo['R'].reshape(3, 3), velo['T'])
    velo_from_imu = homogeneous(imu['R'].reshape(3, 3), imu['T'])
    shared = homogeneous(camera['R_rect_00'].reshape(3, 3))
    suffix = f'{camera_index:02d}'
    projection = camera['P_rect_' + suffix].reshape(3, 4)
    intrinsic = projection[:, :3]
    offset = np.linalg.inv(intrinsic) @ projection[:, 3]
    target_from_velo = homogeneous(IDENTITY, offset) @ shared @ cam0_from_velo
    target_from_imu = target_from_velo @ velo_from_imu
    raw = (directory / 'points.bin').read_bytes()
    require(len(raw) > 0 and len(raw) % 16 == 0 and len(raw) <= 16 * 200_000, 'bounded16-byte records')
    points = np.frombuffer(raw, dtype='<f4').reshape(-1, 4).astype(float)
    require(np.isfinite(points).all(), 'finite actual fixture records')
    size = camera['S_rect_' + suffix].astype(int)
    rows = []
    for index, point in enumerate(points):
        xyz = np.append(point[:3], 1.)
        metric = (target_from_velo @ xyz)[:3]
        pixel_h = projection @ shared @ cam0_from_velo @ xyz
        imu_xyz = (np.linalg.inv(velo_from_imu) @ xyz)[:3]
        # Independently verify inverse-chain round trip, including translations.
        np.testing.assert_allclose(target_from_imu @ np.append(imu_xyz, 1.),
                                   np.append(metric, 1.), atol=MATRIX_ATOL, rtol=0)
        pixel = None if metric[2] <= 0 else (pixel_h[:2] / pixel_h[2]).tolist()
        status = 'behind' if pixel is None else ('inframe' if
            0 <= pixel[0] < size[0] and 0 <= pixel[1] < size[1] else 'outside')
        rows.append(dict(index=index, velodyne_xyz_m=point[:3].tolist(), reflectance=point[3],
                         camera_xyz_m=metric.tolist(), pixel=pixel, status=status,
                         independent_imu_xyz_m=imu_xyz.tolist()))
    return dict(camera_index=camera_index, rectified_size=size.tolist(), k=intrinsic.tolist(),
        translation_offset_m=offset.tolist(), physical_camera_center_in_shared_rect0_m=(-offset).tolist(),
        shared_r_rect_00=shared[:3, :3].tolist(), p_rect_selected=projection.tolist(),
        rectified_camera_from_velo=target_from_velo.tolist(),
        rectified_camera_from_imu=target_from_imu.tolist(), rows=rows,
        summary={key: sum(row['status'] == key for row in rows) for key in ('inframe', 'outside', 'behind')})


def near(expected, actual, tolerance=MATRIX_ATOL, label='$'):
    if isinstance(expected, np.ndarray):
        expected = expected.tolist()
    if type(expected) is float:
        require(type(actual) in (int, float) and math.isfinite(actual)
                and math.isfinite(expected) and abs(expected - actual) <= tolerance,
                'independent numeric witness: ' + label)
    elif type(expected) is list:
        require(type(actual) is list and len(actual) == len(expected), 'witness row inventory: ' + label)
        for index, (left, right) in enumerate(zip(expected, actual)):
            near(left, right, tolerance, label + '[' + str(index) + ']')
    elif type(expected) is dict:
        require(type(actual) is dict and expected.keys() == actual.keys(), 'witness fields: ' + label)
        for key in expected:
            near(expected[key], actual[key], tolerance, label + '.' + key)
    else:
        require(type(expected) is type(actual) and expected == actual, 'exact witness: ' + label)


def rigid_json(matrix):
    return dict(rotation=matrix[:3, :3].tolist(), translation_m=matrix[:3, 3].tolist())


def calibration_witness(directory):
    directory = Path(directory)
    cam = source_fields(directory / 'calib_cam_to_cam.txt')
    velo = source_fields(directory / 'calib_velo_to_cam.txt')
    imu = source_fields(directory / 'calib_imu_to_velo.txt')
    cameras = []
    for index in range(4):
        suffix = f'{index:02d}'
        cameras.append(dict(native_size=cam['S_' + suffix].astype(int).tolist(),
            native_k=cam['K_' + suffix].reshape(3, 3).tolist(), native_d=cam['D_' + suffix].tolist(),
            native_cam_from_cam0=rigid_json(homogeneous(cam['R_' + suffix].reshape(3, 3), cam['T_' + suffix])),
            rectified_size=cam['S_rect_' + suffix].astype(int).tolist(),
            declared_rectification=cam['R_rect_' + suffix].reshape(3, 3).tolist(),
            projection=cam['P_rect_' + suffix].reshape(3, 4).tolist()))
    return dict(cameras=cameras,
        cam0_from_velo=rigid_json(homogeneous(velo['R'].reshape(3, 3), velo['T'])),
        velo_from_imu=rigid_json(homogeneous(imu['R'].reshape(3, 3), imu['T'])),
        calibration_times=[cam['calib_time'], velo['calib_time'], imu['calib_time']],
        corner_distance_m=float(cam['corner_dist'][0]),
        velo_delta_f=velo['delta_f'].tolist(), velo_delta_c=velo['delta_c'].tolist())


def validate_report(report, directory, camera_index, source_hashes):
    require(set(report) == {'schema_version', 'kind', 'camera_index', 'protocol_sha256',
        'protocol', 'source_authority_sha256', 'source_authority', 'source_hashes', 'sources',
        'calibration', 'effective_model', 'rows', 'summary', 'projection_chain', 'camera_axes',
        'velodyne_and_imu_axes', 'translation_units', 'native_camera_parameters',
        'compatibility_metadata', 'camera_images_loaded', 'labels_loaded',
        'ground_truth_operational', 'data_downloaded', 'limits'}, 'exact report field inventory')
    require(report['schema_version'] == 1 and type(report['schema_version']) is int
            and report['kind'] == 'kitti_processed_projection_analytic_tool', 'tool report schema')
    require(report['camera_index'] == camera_index and type(report['camera_index']) is int,
            'selected camera identity')
    require(report['sources'] == source_hashes, 'compiled source identities')
    require(report['protocol_sha256'] == DESIGN_SHA and report['source_authority_sha256'] == AUTHORITY_SHA,
            'fixed protocol/authority identity')
    require(report['protocol'] == json.loads(DESIGN_PATH.read_bytes())
            and report['source_authority'] == json.loads(AUTHORITY_PATH.read_bytes()),
            'complete fixed protocol/authority source objects')
    names = dict(calib_cam_to_cam='calib_cam_to_cam.txt', calib_velo_to_cam='calib_velo_to_cam.txt',
                 calib_imu_to_velo='calib_imu_to_velo.txt', velodyne_points='points.bin')
    require(report['source_hashes'] == {key: dict(bytes=FIXTURE_PINS[name][0],
                sha256=FIXTURE_PINS[name][1]) for key, name in names.items()}, 'exact four source/bin byte pins')
    for flag in ('camera_images_loaded', 'labels_loaded', 'ground_truth_operational', 'data_downloaded'):
        require(report[flag] is False, 'no imagery/labels/acquisition: ' + flag)
    require(report['projection_chain'] == 'P_rect_selected * embed(R_rect_00) * T_cam0_from_velo * X_velo'
            and report['camera_axes'] == 'right,down,forward'
            and report['velodyne_and_imu_axes'] == 'forward,left,up'
            and report['translation_units'] == 'metres', 'coordinate/domain contract')
    require(report['native_camera_parameters'] ==
            'provenance only; never applied to already processed/rectified pixels', 'native K/D role')
    require(report['compatibility_metadata'] ==
            'calib_time/corner_dist/delta_f/delta_c accepted as metadata; not used in projection',
            'compatibility metadata role')
    require(report['limits'] == [
        'Requires user-provided authorized raw calibration/Velodyne files; no dataset acquisition or accuracy claim.',
        'Projection only; no motion compensation, deskew, timestamps, pose evaluation, tracking, uncertainty or driving integration.',
        'Input hashes identify supplied bytes; they do not authenticate factory calibration, a common acquisition day, or archive membership. Calibration processing times may differ.',
        'Reflectance is retained, finite checked, and unused in geometry.',
        'All input rows retained; nonpositive camera depth is behind, without a pixel.'], 'honest adapter scope')
    near(calibration_witness(directory), report['calibration'], label='calibration_all_four')
    expected = independently_project(directory, camera_index)
    model = dict(camera_index=camera_index, rectified_size=expected['rectified_size'], k=expected['k'],
        rectified_baseline_m=expected['translation_offset_m'],
        rectified_camera_center_in_rect0_m=expected['physical_camera_center_in_shared_rect0_m'],
        shared_r_rect_00=expected['shared_r_rect_00'], p_rect_selected=expected['p_rect_selected'],
        rectified_camera_from_velo=rigid_json(np.asarray(expected['rectified_camera_from_velo'])),
        rectified_camera_from_imu=rigid_json(np.asarray(expected['rectified_camera_from_imu'])))
    near(model, report['effective_model'], label='effective_matrix_model')
    rows = [{key: row[key] for key in ('index', 'velodyne_xyz_m', 'reflectance',
                                     'camera_xyz_m', 'pixel', 'status')} for row in expected['rows']]
    near(rows, report['rows'], POINT_PIXEL_ATOL, 'every_actual_binary_point')
    near(dict(points=18, **expected['summary']), report['summary'], label='all_rows_metric_status')
    return expected


def report_corruptions(report, validate, camera_index):
    def geometry(data):
        data['rows'][0]['camera_xyz_m'][0] += .001
    def pixel(data):
        data['rows'][0]['pixel'][0] += .001
    def imu_inverse(data):
        data['effective_model']['rectified_camera_from_imu']['translation_m'][0] += .01
    def transform_direction(data):
        data['effective_model']['rectified_camera_from_velo']['rotation'][0][0] += .1
    def zero_yz(data):
        data['effective_model']['rectified_baseline_m'][1:] = [0., 0.]
    def center_sign(data):
        data['effective_model']['rectified_camera_center_in_rect0_m'][0] *= -1
    def native_warp(data):
        data['effective_model']['k'] = copy.deepcopy(data['calibration']['cameras'][camera_index]['native_k'])
    def metadata(data):
        data['calibration']['cameras'][(camera_index + 1) % 4]['native_d'][0] += .01
    def point_bin(data):
        data['rows'][0]['velodyne_xyz_m'][0] += .001
    def source(data):
        data['source_hashes']['velodyne_points']['sha256'] = '0' * 64
    def selected(data):
        data['camera_index'] = (camera_index + 1) % 4
    def behind(data):
        row = next(row for row in data['rows'] if row['status'] == 'behind')
        row['pixel'] = [0., 0.]
    def category(data):
        row = next(row for row in data['rows'] if row['status'] == 'outside')
        row['status'] = 'inframe'
    operations = dict(metric_geometry=geometry, pixel_projection=pixel, inverse_imu_chain=imu_inverse,
        reversed_or_wrong_transform=transform_direction, discarded_p4_yz=zero_yz,
        physical_center_sign=center_sign, applied_native_k=native_warp,
        unselected_native_provenance=metadata, binary_record=point_bin, source_hash=source,
        selected_camera_identity=selected, behind_has_pixel=behind, outside_as_inframe=category,
        omitted_point=lambda d: d['rows'].pop(),
        invented_total=lambda d: d['summary'].__setitem__('points', 19),
        labels_operational=lambda d: d.__setitem__('ground_truth_operational', True),
        wrong_metric_units=lambda d: d.__setitem__('translation_units', 'centimetres'),
        wrong_camera_axes=lambda d: d.__setitem__('camera_axes', 'forward,left,up'),
        altered_reflectance=lambda d: d['rows'][0].__setitem__('reflectance', .001),
        altered_protocol=lambda d: d['protocol']['geometry'].__setitem__('units', 'centimetres'),
        invented_factory_authentication=lambda d: d['source_authority'].__setitem__('provenance_limit', 'authenticated'),
        unknown_report_field=lambda d: d.__setitem__('invented_trajectory', []),
        unknown_model_field=lambda d: d['effective_model'].__setitem__('extra_undistortion', True),
        boolean_record_identity=lambda d: d['rows'][0].__setitem__('index', False))
    if camera_index != 0:
        def wrong_rectification(data):
            data['effective_model']['shared_r_rect_00'] = copy.deepcopy(
                data['calibration']['cameras'][camera_index]['declared_rectification'])
        operations['selected_r_rect_instead_of_shared00'] = wrong_rectification
    rejected = []
    for name, mutate in operations.items():
        changed = copy.deepcopy(report)
        mutate(changed)
        require(json.dumps(changed, sort_keys=True) != json.dumps(report, sort_keys=True),
                'no-op geometry corruption: ' + name)
        try:
            validate(changed)
        except (ValueError, AssertionError):
            rejected.append(name)
        else:
            raise ValueError('independent auditor accepted geometry corruption: ' + name)
    return rejected


def verify_fixture(directory):
    directory = Path(directory).absolute()
    generated = fixture()
    for name, (size, sha) in FIXTURE_PINS.items():
        path = directory / name
        require(not path.is_symlink() and not any(parent.is_symlink() for parent in path.parents),
                'regular authored fixture inputs')
        require(path.stat().st_size == size and digest(path) == sha, 'authored fixture byte identity: ' + name)
        require(hashlib.sha256(generated[name]).hexdigest() == sha, 'independent generator changed: ' + name)
    return directory


def analytic_controls(directory):
    """Hand-derived matrices/edge pixels, independent of reported Rust outputs."""
    proofs = [independently_project(directory, index) for index in range(4)]
    near([[0., 0., 1., .875], [1., 0., 0., -.5625], [0., 1., 0., 0.], [0., 0., 0., 1.]],
         proofs[0]['rectified_camera_from_velo'])
    near([[0., 1., 0., 1.375], [1., 0., 0., -.6875], [0., 0., -1., .375], [0., 0., 0., 1.]],
         proofs[0]['rectified_camera_from_imu'])
    near([.5625, 4., -.875], proofs[0]['rows'][0]['velodyne_xyz_m'])
    near([0., 0., 4.], proofs[0]['rows'][0]['camera_xyz_m'])
    for index, pixel, status in [(0, [320., 240.], 'inframe'), (1, [0., 240.], 'inframe'),
        (2, [640., 240.], 'outside'), (3, [320., 0.], 'inframe'), (4, [320., 480.], 'outside'),
        (5, [.125, 240.], 'inframe'), (6, [-.125, 240.], 'outside'),
        (7, [639.875, 240.], 'inframe'), (8, [640.125, 240.], 'outside'),
        (9, [320., .125], 'inframe'), (10, [320., -.125], 'outside'),
        (11, [320., 479.875], 'inframe'), (12, [320., 480.125], 'outside'),
        (13, None, 'behind'), (14, None, 'behind'), (15, [320., 240.], 'inframe')]:
        near(pixel, proofs[0]['rows'][index]['pixel'], POINT_PIXEL_ATOL)
        require(proofs[0]['rows'][index]['status'] == status, 'exact hand pixel boundary')
    require([p['summary'] for p in proofs] == [
        dict(inframe=10, outside=6, behind=2), dict(inframe=8, outside=7, behind=3),
        dict(inframe=5, outside=10, behind=3), dict(inframe=12, outside=5, behind=1)], 'analytic all-four status counts')
    # Algebraically wrong alternative chains must actually differ on this fixture.
    cam = source_fields(directory / 'calib_cam_to_cam.txt')
    velo = source_fields(directory / 'calib_velo_to_cam.txt')
    vtransform = homogeneous(velo['R'].reshape(3, 3), velo['T'])
    point = np.append(proofs[0]['rows'][0]['velodyne_xyz_m'], 1.)
    correct = np.asarray(proofs[0]['rows'][0]['camera_xyz_m'])
    offset = np.asarray(proofs[0]['translation_offset_m'])
    alternatives = {
        'reversed_velo_transform': (homogeneous(IDENTITY, offset) @ homogeneous(RY)
                                    @ np.linalg.inv(vtransform) @ point)[:3],
        'wrong_multiplication_order': (vtransform @ homogeneous(RY)
                                     @ homogeneous(IDENTITY, offset) @ point)[:3],
        'missing_shared_rectification': (homogeneous(IDENTITY, offset) @ vtransform @ point)[:3],
        'baseline_sign': (homogeneous(IDENTITY, -offset) @ homogeneous(RY) @ vtransform @ point)[:3],
        'discarded_yz_baseline': (homogeneous(IDENTITY, [offset[0], 0., 0.])
                                @ homogeneous(RY) @ vtransform @ point)[:3],
    }
    for name, wrong in alternatives.items():
        require(np.linalg.norm(wrong - correct) > .01, 'non-discriminating authored chain: ' + name)
    for camera_index in range(1, 4):
        wrong = (homogeneous(IDENTITY, proofs[camera_index]['translation_offset_m'])
            @ homogeneous(cam[f'R_rect_{camera_index:02d}'].reshape(3, 3)) @ vtransform @ point)[:3]
        require(np.linalg.norm(wrong - proofs[camera_index]['rows'][0]['camera_xyz_m']) > .01,
                'per-camera rectification corruption must alter actual geometry')
    return dict(kind='authored_analytic_matrix_and_half_open_pixel_controls', actual_rust_cli_runs=0,
        hand_matrices_checked=2, hand_binary_point_checked=True, hand_pixel_cases_checked=16,
        inverse_imu_round_trips_checked=72, summaries=[p['summary'] for p in proofs],
        non_noop_wrong_chain_controls=list(alternatives) + ['selected_r_rect_01', 'selected_r_rect_02', 'selected_r_rect_03'])


def command_for(binary, directory, camera, output):
    return [str(binary), '--calib-cam', str(directory / 'calib_cam_to_cam.txt'),
            '--calib-velo', str(directory / 'calib_velo_to_cam.txt'),
            '--calib-imu', str(directory / 'calib_imu_to_velo.txt'),
            '--points', str(directory / 'points.bin'), '--camera', str(camera), '--output', str(output)]


def execute(command, log):
    with Path(log).open('x') as stream:
        actual = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=60)
    return dict(command=command, actual_exit_code=actual.returncode, log_sha256=digest(log))


def malformed_cli_controls(binary, directory, camera, output):
    """Actual strict-loader negative trials, with unchanged original fixture bytes."""
    output.mkdir()
    raw = {name: (directory / name).read_bytes() for name in FIXTURE_PINS}
    suffix = f'{camera:02d}'
    def camera_line(body, key, values=None, omit=False):
        lines = body.decode('ascii').splitlines()
        position = next(index for index, line in enumerate(lines) if line.startswith(key + ':'))
        if omit:
            lines.pop(position)
        else:
            lines[position] = key + ': ' + values
        return ('\n'.join(lines) + '\n').encode('ascii')
    operations = {
        'duplicate_camera_key': ('calib_cam_to_cam.txt', lambda b: b + f'K_{suffix}: 1 0 0 0 1 0 0 0 1\n'.encode()),
        'missing_selected_projection': ('calib_cam_to_cam.txt', lambda b: camera_line(b, 'P_rect_' + suffix, omit=True)),
        'projection_wrong_cardinality': ('calib_cam_to_cam.txt', lambda b: camera_line(b, 'P_rect_' + suffix, '1 0 0')),
        'nonfinite_projection': ('calib_cam_to_cam.txt', lambda b: camera_line(b, 'P_rect_' + suffix, 'NaN 0 0 0 0 1 0 0 0 0 1 0')),
        'nonrigid_rectification': ('calib_cam_to_cam.txt', lambda b: camera_line(b, 'R_rect_' + suffix, '.5 0 0 0 1 0 0 0 1')),
        'singular_projection': ('calib_cam_to_cam.txt', lambda b: camera_line(b, 'P_rect_' + suffix, '0 0 0 0 0 1 0 0 0 0 1 0')),
        'missing_native_distortion': ('calib_cam_to_cam.txt', lambda b: camera_line(b, 'D_' + suffix, omit=True)),
        'unknown_geometry_field': ('calib_cam_to_cam.txt', lambda b: b + b'unknown_geometry: 0\n'),
        'improper_velo_rotation': ('calib_velo_to_cam.txt', lambda b: camera_line(b, 'R', '-1 0 0 0 1 0 0 0 1')),
        'nonfinite_imu_translation': ('calib_imu_to_velo.txt', lambda b: camera_line(b, 'T', '0 NaN 0')),
        'truncated_point_record': ('points.bin', lambda b: b[:-1]),
        'empty_point_file': ('points.bin', lambda b: b''),
        'nonfinite_binary_coordinate': ('points.bin', lambda b: struct.pack('<f', float('nan')) + b[4:]),
        'nonfinite_reflectance': ('points.bin', lambda b: b[:12] + struct.pack('<f', float('inf')) + b[16:]),
    }
    trials = []
    for name, (file, mutate) in operations.items():
        changed = mutate(raw[file])
        require(changed != raw[file], 'no-op actual source corruption: ' + name)
        case = output / name; case.mkdir()
        for original, body in raw.items():
            with (case / original).open('xb') as stream:
                stream.write(changed if original == file else body)
        trial = execute(command_for(binary, case, camera, case / 'rejected.json'), case / 'actual.log')
        write(case / 'actual-journal.json', trial)
        require(trial['actual_exit_code'] == 2, 'loader accepted malformed source: ' + name)
        require(not (case / 'rejected.json').exists() or (case / 'rejected.json').stat().st_size == 0,
                'malformed source produced successful geometry evidence')
        trials.append(dict(name=name, **trial, changed_file=file,
                           changed_source_sha256=hashlib.sha256(changed).hexdigest()))
    return trials


def actual_output_guards(binary, directory, camera, output):
    output.mkdir()
    old = output / 'existing.json'; old.write_bytes(b'preserved evidence')
    link = output / 'link'; link.symlink_to(old)
    dangling = output / 'dangling'; dangling.symlink_to(output / 'not-here')
    parent_link = output / 'parent-link'; parent_link.symlink_to(output, target_is_directory=True)
    existing_directory = output / 'existing-directory'; existing_directory.mkdir()
    trials = []
    for name, target in [('existing_file', old), ('existing_directory', existing_directory),
                         ('valid_symlink', link), ('dangling_symlink', dangling),
                         ('symlink_parent', parent_link / 'fresh.json'),
                         ('dot_component', str(output) + '/./fresh-dot.json'),
                         ('parent_component', str(output) + '/../fresh-parent.json')]:
        # Deliberately nonexistent sources prove output admission precedes reads.
        trial = execute(command_for(binary, output / 'no-inputs', camera, target), output / (name + '.log'))
        write(output / (name + '-journal.json'), trial)
        require(trial['actual_exit_code'] == 2, 'output hazard accepted: ' + name)
        text = (output / (name + '.log')).read_text()
        require('output' in text and 'input metadata' not in text, 'output admission did not precede reads')
        trials.append(dict(name=name, **trial, admission_precedes_source_reads=True))
    require(old.read_bytes() == b'preserved evidence' and link.is_symlink()
            and dangling.is_symlink() and not (output / 'fresh.json').exists(), 'overwritten previous evidence')
    return trials


def python_output_guards():
    rejected = []
    with tempfile.TemporaryDirectory(prefix='kitti-oracle-output-') as temporary:
        parent = Path(temporary)
        old = parent / 'existing'; old.write_bytes(b'original evidence')
        directory = parent / 'directory'; directory.mkdir()
        link = parent / 'link'; link.symlink_to(old)
        dangling = parent / 'dangling'; dangling.symlink_to(parent / 'missing')
        parent_link = parent / 'linked-parent'; parent_link.symlink_to(directory, target_is_directory=True)
        for name, target in [('existing_file', old), ('existing_directory', directory),
                             ('valid_symlink', link), ('dangling_symlink', dangling),
                             ('symlink_parent', parent_link / 'fresh')]:
            try:
                fresh(target)
            except ValueError:
                rejected.append(name)
            else:
                raise ValueError('Python guard accepted ' + name)
        require(old.read_bytes() == b'original evidence', 'Python guard overwrote evidence')
    return rejected


def actual_audit(args, output, directory):
    require(digest(DESIGN_PATH) == DESIGN_SHA and digest(AUTHORITY_PATH) == AUTHORITY_SHA,
            'fixed design/source authority changed')
    design = json.loads(DESIGN_PATH.read_bytes())
    require(design['authored_fixture']['files'] == {
        key: dict(bytes=size, sha256=sha) for key, (size, sha) in FIXTURE_PINS.items()}, 'fixed fixture contract')
    require(design['authored_fixture']['fixed_independent_tolerances'] == dict(
        homogeneous_matrix_absolute=MATRIX_ATOL, point_and_pixel_absolute=POINT_PIXEL_ATOL), 'fixed tolerances')
    require(set(COMPILED_SOURCE_PINS) == set(SOURCE_PATHS), 'compiled source freeze not finalized')
    for key, path in SOURCE_PATHS.items():
        require(digest(ROOT / path) == COMPILED_SOURCE_PINS[key], 'frozen compiled source changed: ' + key)
    require(args.binary is not None, 'actual geometry audit requires --binary')
    binary = args.binary.absolute()
    require(not binary.is_symlink() and binary.is_file(), 'regular actual binary required')
    binary_sha = digest(binary)
    output.mkdir(parents=True, exist_ok=False)
    # Concrete source/fixture/executable receipts persist BEFORE the first call.
    write(output / 'freeze.json', dict(schema_version=1,
        prepared_before_any_authored_fixture_cli=True, design_sha256=DESIGN_SHA,
        source_authority_sha256=AUTHORITY_SHA, compiled_sources=COMPILED_SOURCE_PINS,
        binary_sha256=binary_sha, auditor_sha256=digest(Path(__file__)),
        fixture_files={key: dict(bytes=size, sha256=sha) for key, (size, sha) in FIXTURE_PINS.items()},
        matrix_absolute_tolerance=MATRIX_ATOL, point_pixel_absolute_tolerance=POINT_PIXEL_ATOL,
        relative_tolerance=0, authored_point_frame_stamp_seconds=0., actual_kitti_data=False))
    cameras, journals = [], []
    for index in range(4):
        target = output / f'camera-{index}.json'
        journal = execute(command_for(binary, directory, index, target), output / f'camera-{index}.log')
        write(output / f'camera-{index}-journal.json', journal)
        require(journal['actual_exit_code'] == 0, 'actual positive CLI failure; preserved first result/log')
        require(target.stat().st_size <= 64 * 1024 * 1024, 'bounded actual report')
        report = json.loads(target.read_bytes())
        validate = lambda data: validate_report(data, directory, index, COMPILED_SOURCE_PINS)
        expected = validate(report)
        mutations = report_corruptions(report, validate, index)
        malformed = malformed_cli_controls(binary, directory, index, output / f'camera-{index}-source-negatives')
        guards = actual_output_guards(binary, directory, index, output / f'camera-{index}-output-guards')
        cameras.append(dict(camera_index=index, summary=report['summary'], every_row_checked=len(expected['rows']),
            independent_geometry=expected, actual_report_sha256=digest(target),
            non_noop_report_corruptions_rejected=mutations,
            actual_malformed_source_trials=malformed, actual_output_guard_trials=guards))
        journals.append(journal)
    verify_fixture(directory)
    require(digest(binary) == binary_sha and digest(DESIGN_PATH) == DESIGN_SHA, 'binary/design changed during proof')
    require(all(digest(ROOT / path) == COMPILED_SOURCE_PINS[key] for key, path in SOURCE_PATHS.items()),
            'compiled sources changed during proof')
    proof = dict(schema='independent-authored-kitti-calibration-oracle-v1',
        integrity_and_actual_geometry_passed=True, actual_positive_cli_runs=4,
        actual_malformed_source_cli_runs=sum(len(c['actual_malformed_source_trials']) for c in cameras),
        actual_output_guard_cli_runs=sum(len(c['actual_output_guard_trials']) for c in cameras),
        cameras=cameras, journals=journals, binary_sha256=binary_sha, compiled_sources=COMPILED_SOURCE_PINS,
        design_sha256=DESIGN_SHA, source_authority_sha256=AUTHORITY_SHA,
        auditor_sha256=digest(Path(__file__)),
        fixture_files={key: dict(bytes=size, sha256=sha) for key, (size, sha) in FIXTURE_PINS.items()},
        authored_point_frame_stamp_seconds=0., actual_kitti_data=False, camera_images=False,
        ground_truth_labels=False, calibration_physically_measured=False, accuracy_claim=False,
        primary_reference_paper_sha256=PAPER_SHA,
        matrix_absolute_tolerance=MATRIX_ATOL, point_pixel_absolute_tolerance=POINT_PIXEL_ATOL,
        relative_tolerance=0, no_global_math_overrides=True,
        calibration_times_are_source_metadata_not_authenticated_common_acquisition=True,
        scope='Generalized authored KITTI-format math/schema/CLI controls, not measured KITTI validation.',
        analytic_controls=analytic_controls(directory), python_output_guards=python_output_guards())
    write(output / 'results.json', proof)
    return proof


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-fixture', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--fixture', type=Path,
        default=ROOT / 'artifacts/goal30-kitti-preparation/authored-geometry-fixture-v1')
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = fresh(args.output)  # Before fixture/source reads or any actual CLI.
    require(np.__version__ == '2.3.5', 'pinned NumPy2.3.5 required')
    require(not (args.prepare_fixture and args.self_test), 'choose preparation or self-test')
    if args.prepare_fixture:
        proof = prepare_fixture(output)
        print(json.dumps(proof))
    elif args.self_test:
        directory = verify_fixture(args.fixture)
        proof = analytic_controls(directory)
        proof.update(python_output_guards=python_output_guards(), auditor_sha256=digest(Path(__file__)))
        output.mkdir(parents=True, exist_ok=False)
        write(output / 'results.json', proof)
        print(json.dumps(proof))
    else:
        directory = verify_fixture(args.fixture)
        proof = actual_audit(args, output, directory)
        print(json.dumps(dict(integrity_and_actual_geometry_passed=True,
                              actual_positive_cli_runs=4,
                              summaries=[camera['summary'] for camera in proof['cameras']])))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, AssertionError, subprocess.SubprocessError) as error:
        print('Independent KITTI-format geometry: ' + str(error), file=sys.stderr)
        sys.exit(2)
