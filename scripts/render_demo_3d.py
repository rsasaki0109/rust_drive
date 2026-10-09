"""Render verified RNE telemetry with Blender Cycles CPU and package a 3D GIF."""
import argparse
import hashlib
import json
import math
import shlex
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFilter
from render_demo import font

ROOT = Path(__file__).resolve().parent.parent
GROUND_MODES = ('lidar3d_ground', 'lidar3d_ground_body')


def verify_recorded_lidar3d(evidence, calibration):
    """Audit recorded XYZ/ordinal consistency; physical ray acceptance is separate."""
    acquisitions = evidence.get('acquisitions')
    if not isinstance(acquisitions, list) or not acquisitions:
        raise ValueError('Inclined LiDAR evidence requires recorded acquisitions')
    columns, rings = calibration['azimuth_columns'], calibration['elevation_rings']
    ray_count = columns*rings
    total, failed = 0, 0
    for acquisition in acquisitions:
        cloud = acquisition.get('cloud_3d')
        if cloud is None and acquisition.get('cloud_3d_failed') is True:
            failed += 1
            continue
        if not isinstance(cloud, dict):
            raise ValueError('Inclined LiDAR acquisition has neither recorded cloud nor explicit failure')
        ranges, returns = cloud.get('ranges_m'), cloud.get('returns')
        if (not isinstance(ranges, list) or len(ranges) != ray_count or
                not isinstance(returns, list) or len(returns) > ray_count):
            raise ValueError('Inclined LiDAR cloud dimensions differ from calibrated firing ordinals')
        slots = set()
        for measured in returns:
            index, point = measured['ray_index'], measured['point']
            if type(index) is not int or not 0 <= index < ray_count or index in slots:
                raise ValueError('Inclined LiDAR return has an invalid or duplicate firing ordinal')
            slots.add(index)
            coordinates = [point['x'], point['y'], point['z']]
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in coordinates):
                raise ValueError('Inclined LiDAR XYZ coordinates must be finite')
            vector = [coordinates[0], coordinates[1], coordinates[2]-calibration['mount_height_m']]
            distance = math.sqrt(sum(v*v for v in vector))
            observed = ranges[index]
            if (type(observed) not in (int, float) or not math.isfinite(observed) or
                    not calibration['min_range_m']-1e-7 <= distance <= calibration['max_range_m']+1e-7 or
                    abs(observed-distance) > 1e-7):
                raise ValueError('Inclined LiDAR XYZ range differs from its recorded ray range')
            azimuth = -math.pi+2*math.pi*(index//rings)/(columns-1)
            elevation = calibration['min_elevation_rad']+(calibration['max_elevation_rad']-
                calibration['min_elevation_rad'])*(index % rings)/(rings-1)
            expected = [math.cos(elevation)*math.cos(azimuth),
                        -math.cos(elevation)*math.sin(azimuth), math.sin(elevation)]
            if any(abs(actual/distance-wanted) > 1e-4 for actual, wanted in zip(vector, expected)):
                raise ValueError('Inclined LiDAR return direction differs from its calibrated beam')
        if {index for index, value in enumerate(ranges) if value is not None} != slots:
            raise ValueError('Inclined LiDAR firing ranges and XYZ return identities differ')
        total += len(returns)
    return {'acquisitions': len(acquisitions), 'failed_acquisitions': failed,
            'returns_verified': total, 'ray_ordinals_per_acquisition': ray_count}


def verify_ground_diagnostics(evidence):
    """Check recorded fit metadata consistency, not independent fit reconstruction."""
    config = evidence['lidar3d'].get('ground')
    fields = {'reference_height_m', 'max_slope', 'max_height_offset_m',
              'residual_threshold_m', 'fit_radius_m', 'min_inliers',
              'min_sector_inliers', 'min_cell_inliers'}
    if not isinstance(config, dict) or set(config) != fields:
        raise ValueError('Ground mode requires complete measured-fit calibration')
    bounds = {'reference_height_m': (-1, 1), 'max_slope': (.001, .1),
              'max_height_offset_m': (.001, .04), 'residual_threshold_m': (.002, .03),
              'fit_radius_m': (3, 12), 'min_inliers': (32, 10000),
              'min_sector_inliers': (3, config['min_inliers']/8), 'min_cell_inliers': (3, 256)}
    for field, (lo, hi) in bounds.items():
        value = config[field]
        if (type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi or
                (field.startswith('min_') and type(value) is not int)):
            raise ValueError('Invalid measured-ground fit calibration')
    if evidence['lidar3d']['mount_height_m']-config['reference_height_m'] < .1:
        raise ValueError('Ground reference must lie below the calibrated LiDAR mount')
    acquisitions = {round(a['time'], 9): a for a in evidence['acquisitions']}
    diagnostics = evidence.get('ground_observations')
    if not isinstance(diagnostics, list) or not diagnostics:
        raise ValueError('Ground mode requires measured fit diagnostics')
    removed, preserved, confident, previous = 0, 0, 0, -math.inf
    for fit in diagnostics:
        stamp = fit['stamp']
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= previous:
            raise ValueError('Ground diagnostics require increasing acquisition stamps')
        previous = stamp
        acquisition = acquisitions.get(round(stamp, 9))
        if acquisition is None or abs(acquisition['time']-stamp) > 1e-8 or 'cloud_3d' not in acquisition:
            raise ValueError('Ground diagnostics do not match a recorded raw cloud')
        count = len(acquisition['cloud_3d']['returns'])
        for field in ['candidate_points', 'inliers', 'removed_points', 'preserved_points', 'supported_cells']:
            if type(fit[field]) is not int or not 0 <= fit[field] <= count:
                raise ValueError('Ground diagnostic counts exceed measured cloud')
        sectors = fit['sector_inliers']
        if (not isinstance(sectors, list) or len(sectors) != 8 or
                any(type(v) is not int or v < 0 for v in sectors) or sum(sectors) != fit['inliers'] or
                fit['inliers'] > fit['candidate_points'] or fit['removed_points']+fit['preserved_points'] != count or
                type(fit['confidence']) is not bool):
            raise ValueError('Ground fit diagnostic support/counts are inconsistent')
        for field in ['max_inlier_residual_m', 'rms_residual_m']:
            if type(fit[field]) not in (int, float) or not math.isfinite(fit[field]) or not 0 <= fit[field] <= config['residual_threshold_m']+1e-9:
                raise ValueError('Ground residual diagnostics exceed the configured fit band')
        if fit['confidence']:
            plane = fit['plane']
            if (not isinstance(plane, dict) or set(plane) != {'a','b','c'} or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in plane.values()) or
                    math.hypot(plane['a'], plane['b']) > config['max_slope']+1e-9 or
                    abs(plane['c']-config['reference_height_m']) > config['max_height_offset_m']+1e-9 or
                    fit['inliers'] < config['min_inliers'] or min(sectors) < config['min_sector_inliers']):
                raise ValueError('Confident ground fit lacks bounded measured support')
            confident += 1
        removed += fit['removed_points']
        preserved += fit['preserved_points']
    return {'observations_verified': len(diagnostics), 'confident_fits': confident,
            'removed_returns': removed, 'preserved_returns': preserved,
            'audit_scope': 'recorded support/count/plane consistency; independent ground oracle is separate'}


def body_box(calibration, pose):
    offset = calibration['center_offset_body_m']
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    return {'id': 'research-body-envelope',
            'center_m': [pose['position']['x']+c*offset[0]-s*offset[1],
                         pose['position']['y']+s*offset[0]+c*offset[1],
                         calibration['bottom_m']+calibration['height_m']/2],
            'half_extents_m': [calibration['length_m']/2, calibration['width_m']/2, calibration['height_m']/2],
            'yaw_rad': pose['yaw']}


def verify_body_guard(guard, run):
    if guard.get('schema_version') != 1 or guard.get('summary', {}).get('passed') is not True:
        raise ValueError('Body envelope evidence must pass independent motion guards')
    calibration = guard.get('calibration', {})
    expected = {'length_m': 4.2, 'width_m': 1.8, 'height_m': 1.5, 'bottom_m': .15,
                'speed_bound_m_s': 12, 'clearance_floor_m': 1}
    for field, wanted in expected.items():
        value = calibration.get(field)
        if type(value) not in (int, float) or not math.isfinite(value) or abs(value-wanted) > 1e-9:
            raise ValueError('Body wire overlay requires the supported research calibration')
    if (calibration.get('center_offset_body_m') != [0,0] or
            calibration.get('calibration_kind') != 'authored_research_dimensions' or
            calibration.get('pose_reference') != 'native_planar_plant_reference'):
        raise ValueError('Body calibration/reference differs from supported research geometry')
    samples = {round(sample['time'], 9): sample for sample in guard['motion_samples']}
    for frame in run['frames']:
        sample = samples.get(round(frame['time'], 9))
        expected_pose = frame['truth']['pose']
        if (sample is None or abs(sample['time']-frame['time']) > 1e-8 or
                math.dist([sample['pose']['position']['x'],sample['pose']['position']['y']],
                          [expected_pose['position']['x'],expected_pose['position']['y']]) > 1e-8 or
                abs(sample['pose']['yaw']-expected_pose['yaw']) > 1e-8):
            raise ValueError('Research body guard poses differ from displayed recording')


def load_native_scene(path, run):
    """Validate a successful simulator sidecar against the displayed timeline."""
    evidence = json.loads(path.read_text())
    if (evidence.get('schema_version') != 1 or
            evidence.get('backend') != run['backend'] or
            evidence.get('scenario') != run['scenario']['name'] or
            evidence.get('summary', {}).get('passed') is not True):
        raise ValueError('Native scene evidence must pass and match the recording backend/scenario')
    if (type(evidence.get('seed')) is not int or not 0 <= evidence['seed'] < 2**64 or
            evidence['seed'] != run['summary']['seed']):
        raise ValueError('Native scene evidence requires a u64 seed matching the recording summary')
    if 'operating_mode' in evidence:
        mode = evidence['operating_mode']
        if mode not in ['multi_height_lidar', 'lidar3d', *GROUND_MODES]:
            raise ValueError('Unsupported native scene operating mode')
        calibration = evidence.get('multi_height_lidar' if mode == 'multi_height_lidar' else 'lidar3d', {})
        if mode == 'multi_height_lidar':
            if not isinstance(calibration, dict) or set(calibration) != {
                    'heights_m', 'collision_bottom_m', 'collision_top_m'}:
                raise ValueError('Native multi-height evidence requires complete calibrated sensor metadata')
            heights = calibration['heights_m']
            if (not isinstance(heights, list) or len(heights) != 3 or
                    any(type(h) not in (int, float) or not math.isfinite(h) for h in heights) or
                    any(abs(actual-expected) > 1e-9 for actual, expected in
                        zip(sorted(heights), [0.15, 0.6, 3.7]))):
                raise ValueError('Native multi-height evidence must use the three measured calibrated planes')
        else:
            expected = {'azimuth_columns': calibration.get('azimuth_columns') if mode in GROUND_MODES else 720, 'elevation_rings': 16,
                        'min_elevation_rad': -math.pi/12, 'max_elevation_rad': math.pi/12,
                        'mount_height_m': 0.6, 'min_range_m': 0.2, 'max_range_m': 45.0}
            if not isinstance(calibration, dict) or set(calibration) != set(expected) | {
                    'collision_bottom_m', 'collision_top_m'} | ({'ground'} if mode in GROUND_MODES else set()):
                raise ValueError('Native inclined LiDAR evidence requires complete calibrated sensor metadata')
            if mode in GROUND_MODES and calibration['azimuth_columns'] not in (180, 360):
                raise ValueError('Ground replay requires the supported 180/360-column native calibration')
            for field, wanted in expected.items():
                value = calibration[field]
                if (type(value) not in (int, float) or not math.isfinite(value) or
                        abs(value-wanted) > 1e-9 or
                        (field in ['azimuth_columns', 'elevation_rings'] and type(value) is not int)):
                    raise ValueError('Inclined LiDAR metadata differs from the supported native beam calibration')
        radius = run['vehicle']['radius']
        body_mode = mode in GROUND_MODES and 'body_guard' in evidence
        height_window = [('collision_bottom_m', -0.85), ('collision_top_m', 2.65)] if body_mode else [
            ('collision_bottom_m', 0.1-radius), ('collision_top_m', 1.1+radius)]
        for field, expected in height_window:
            value = calibration[field]
            if type(value) not in (int, float) or not math.isfinite(value) or abs(value-expected) > 1e-9:
                raise ValueError('Native height gate must match the recorded ego envelope and clearance reserve')
        if mode in ['lidar3d', *GROUND_MODES]:
            verify_recorded_lidar3d(evidence, calibration)
        if mode in GROUND_MODES:
            verify_ground_diagnostics(evidence)
    scene = evidence.get('scene', {})
    boxes = scene.get('static_cuboids')
    ground_mode = evidence.get('operating_mode') in GROUND_MODES
    ground_boxes = scene.get('ground_cuboids', []) if ground_mode else []
    if (scene.get('schema_version') != (2 if ground_mode else 1) or not isinstance(scene.get('name'), str) or
            not scene['name'].strip() or not isinstance(boxes, list) or
            not isinstance(ground_boxes, list) or not 1 <= len(boxes)+len(ground_boxes) <= 128 or
            (ground_mode and not ground_boxes)):
        raise ValueError('A bounded schema-1 native cuboid scene is required')
    ids = set()
    for box in boxes+ground_boxes:
        if not isinstance(box.get('id'), str) or not box['id'].strip() or box['id'] in ids:
            raise ValueError('Native cuboid IDs must be nonempty and unique')
        ids.add(box['id'])
        for field in ['center_m', 'half_extents_m']:
            vector = box.get(field)
            if (not isinstance(vector, list) or len(vector) != 3 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) or abs(v) > 2000 or
                        (field == 'half_extents_m' and v <= 0) for v in vector)):
                raise ValueError('Native cuboid coordinates must be finite ENU meters; extents positive')
        yaw = box.get('yaw_rad')
        if isinstance(yaw, bool) or not isinstance(yaw, (int, float)) or not math.isfinite(yaw) or abs(yaw) > math.pi:
            raise ValueError('Native cuboid yaw must be finite within [-pi, pi]')
    if ground_mode and any(abs(box['center_m'][2]+box['half_extents_m'][2]) > 1e-9 for box in ground_boxes):
        raise ValueError('Physical road tops must match the flat road datum at zero meters')
    if ground_mode and ('body_guard' in evidence) != (evidence['operating_mode'] == 'lidar3d_ground_body'):
        raise ValueError('Ground operating mode and physical body evidence disagree')
    if ground_mode and 'body_guard' in evidence:
        verify_body_guard(evidence['body_guard'], run)
    observations = evidence.get('observations')
    if not isinstance(observations, list) or not observations:
        raise ValueError('Native scene evidence requires recorded body poses')
    poses = {}
    previous = -math.inf
    for observation in observations:
        time = observation['time']
        pose = observation['pose']
        values = [time, pose['position']['x'], pose['position']['y'], pose['yaw']]
        if any(not math.isfinite(v) for v in values) or time <= previous:
            raise ValueError('Native scene observations must have finite poses and increasing timestamps')
        previous = time
        poses[round(time, 9)] = observation
    for frame in run['frames']:
        observation = poses.get(round(frame['time'], 9))
        pose = frame['truth']['pose']
        if (observation is None or abs(observation['time']-frame['time']) > 1e-8 or
                abs(observation['pose']['position']['x']-pose['position']['x']) > 1e-8 or
                abs(observation['pose']['position']['y']-pose['position']['y']) > 1e-8 or
                abs(observation['pose']['yaw']-pose['yaw']) > 1e-8):
            raise ValueError('Native scene body pose/time differs from the displayed recording')
    return evidence


def verify_native_cuboids(rendered, scene, field='static_cuboids'):
    """Check actual world-space mesh corners, including height and yaw."""
    expected = {box['id']: box for box in scene[field]}
    if len(rendered) != len(expected) or {box['id'] for box in rendered} != set(expected):
        raise ValueError('Rendered native cuboid identities/count differ from the physical scene')
    for actual in rendered:
        box = expected[actual['id']]
        c, s = math.cos(box['yaw_rad']), math.sin(box['yaw_rad'])
        corners = []
        for sx in [-1, 1]:
            for sy in [-1, 1]:
                for sz in [-1, 1]:
                    x, y, z = [sign*half for sign, half in zip([sx, sy, sz], box['half_extents_m'])]
                    corners.append([box['center_m'][0]+c*x-s*y, box['center_m'][1]+s*x+c*y,
                                    box['center_m'][2]+z])
        if len(actual['world_corners_m']) != 8 or any(
                len(corner) != 3 or not all(math.isfinite(v) for v in corner)
                for corner in actual['world_corners_m']) or any(
                min(math.dist(corner, wanted) for wanted in corners) > 1e-4
                for corner in actual['world_corners_m']) or any(
                min(math.dist(wanted, corner) for corner in actual['world_corners_m']) > 1e-4
                for wanted in corners):
            raise ValueError('Rendered native cuboid geometry differs from the physical scene')


def encode_gif(images, output, durations):
    """Use a shared palette and mild spatial noise reduction for README size."""
    prepared = []
    for image in images:
        image = image.copy()
        image.paste(image.crop((0, 54, 960, 594)).filter(ImageFilter.MedianFilter(3)), (0, 54))
        prepared.append(image)
    samples = prepared[::max(1, len(prepared)//16)]
    palette_sample = Image.new('RGB', (480, 160*len(samples)))
    saturated = []
    for i, sample in enumerate(samples):
        thumb = sample.resize((240, 160))
        palette_sample.paste(thumb, (0, 160*i))
        saturated.extend(rgb for rgb in thumb.getdata() if max(rgb)-min(rgb) > .45*max(rgb))
    # Reserve color capacity for small cars, path overlays and HUD text instead
    # of letting the large ground/asphalt regions dominate the shared palette.
    if saturated:
        swatches = Image.new('RGB', (240, 160*len(samples)))
        swatches.putdata([saturated[i % len(saturated)] for i in range(swatches.width*swatches.height)])
        palette_sample.paste(swatches, (240, 0))
    palette = palette_sample.quantize(colors=192)
    encoded = [image.quantize(palette=palette, dither=Image.Dither.NONE) for image in prepared]
    encoded[0].save(output, save_all=True, append_images=encoded[1:], duration=durations, loop=0, optimize=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, default=Path('assets/rne-3d-demo.gif'))
    parser.add_argument('--scene-output', type=Path, help='Save an editable Blender scene at the last rendered frame')
    parser.add_argument('--native-scene', type=Path, help='Successful native scene.json evidence; opt in to rendering physical cuboids')
    parser.add_argument('--traffic-models', nargs='+', choices=['hatchback','sedan','van','pickup'], default=['hatchback'], help='Display models assigned in stable actor appearance order')
    parser.add_argument('--camera', choices=['ego','traffic'], default='ego', help='Follow ego or frame ego and active reactive vehicles')
    parser.add_argument('--preview-time', type=float, help='Render one PNG instead of the complete GIF')
    parser.add_argument('--samples', type=int, default=16)
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.samples <= 128 or not 1 <= args.threads <= 64:
        parser.error('samples must be 1–128 and threads 1–64')
    if args.preview_time is not None and not math.isfinite(args.preview_time):
        parser.error('preview time must be finite')
    if args.scene_output:
        if args.scene_output.suffix != '.blend':
            parser.error('scene output must use the .blend extension')
        args.scene_output.parent.mkdir(parents=True, exist_ok=True)
    run = json.loads(args.run.read_text())
    if run.get('schema_version') != 1 or not run.get('backend', '').startswith('rne-') or not run['summary']['passed']:
        raise SystemExit('A successful schema-1 actual RNE recording is required')
    frames = run['frames']
    native_evidence = load_native_scene(args.native_scene, run) if args.native_scene else None
    indices = [0]
    for i in range(1, len(frames)):
        if frames[i]['time']-frames[indices[-1]]['time'] >= .3-1e-9:
            indices.append(i)
    if indices[-1] != len(frames)-1:
        indices.append(len(frames)-1)
    if args.preview_time is not None:
        indices = [min(range(len(frames)), key=lambda i: abs(frames[i]['time']-args.preview_time))]
    cache = ROOT/'artifacts/3d'
    cache.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='capture-', dir=cache) as temporary:
        directory = Path(temporary)
        request = {'run': str(args.run.resolve()), 'indices': indices,
                   'frames_directory': temporary, 'samples': args.samples,
                   'scene_output': str(args.scene_output.resolve()) if args.scene_output else None,
                   'traffic_models': args.traffic_models, 'camera': args.camera}
        if native_evidence:
            request['native_scene'] = native_evidence['scene']
            if native_evidence.get('operating_mode') in GROUND_MODES:
                request['ground_mode'] = True
                if 'body_guard' in native_evidence:
                    request['body_calibration'] = native_evidence['body_guard']['calibration']
        request_file = directory/'request.json'
        request_file.write_text(json.dumps(request))
        command = ['blender', '--background', '--factory-startup', '--threads', str(args.threads),
                   '--python-exit-code', '2', '--python', str(ROOT/'scripts/blender_scene.py'), '--', str(request_file)]
        with (directory/'blender.log').open('w') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise SystemExit((directory/'blender.log').read_text()[-5000:])
        scene_info = json.loads((directory/'scene-info.json').read_text())
        audit = json.loads((directory/'audit.json').read_text())
        if native_evidence:
            verify_native_cuboids(scene_info['native_cuboids'], native_evidence['scene'])
            if native_evidence.get('operating_mode') in GROUND_MODES:
                verify_native_cuboids(scene_info['native_ground_cuboids'], native_evidence['scene'], 'ground_cuboids')
                if 'body_guard' in native_evidence:
                    verify_native_cuboids([scene_info['body_envelope']], {'static_cuboids': [body_box(
                        native_evidence['body_guard']['calibration'], frames[indices[-1]]['truth']['pose'])]})
        images = []
        for number, index in enumerate(indices):
            frame, record = frames[index], audit[number]
            expected_pose = frame['truth']['pose']
            pose_error = math.hypot(record['ego_pose']['position']['x']-expected_pose['position']['x'], record['ego_pose']['position']['y']-expected_pose['position']['y'])
            expected_objects = {a['id']: a for a in frame['objects']}
            objects_match = len(record['objects']) == len(expected_objects) and all(
                a['id'] in expected_objects and math.hypot(a['position']['x']-expected_objects[a['id']]['position']['x'], a['position']['y']-expected_objects[a['id']]['position']['y']) <= 1e-4 for a in record['objects'])
            if record['time'] != frame['time'] or pose_error > 1e-4 or abs(record['ego_pose']['yaw']-expected_pose['yaw']) > 1e-5 or not objects_match:
                raise SystemExit('Rendered scene state differs from the recorded simulation')
            if native_evidence:
                verify_native_cuboids(record['native_cuboids'], native_evidence['scene'])
                if native_evidence.get('operating_mode') in GROUND_MODES:
                    verify_native_cuboids(record['native_ground_cuboids'], native_evidence['scene'], 'ground_cuboids')
                    if 'body_guard' in native_evidence:
                        verify_native_cuboids([record['body_envelope']], {'static_cuboids': [body_box(native_evidence['body_guard']['calibration'], expected_pose)]})
            with Image.open(directory/f'{number:04d}.png') as rendered:
                image = Image.new('RGB', (960, 640), '#0a1220')
                image.paste(rendered, (0, 54))
            draw = ImageDraw.Draw(image)
            draw.text((22, 10), 'RustDrive', font=font(28, True), fill='#edf4ff')
            draw.text((204, 20), 'RNE NATIVE DYNAMICS  /  BLENDER 3D REPLAY', font=font(12, True), fill='#46e3c2')
            if native_evidence:
                label = {'multi_height_lidar': 'MULTI-HEIGHT / PLANAR EGO',
                         'lidar3d': 'INCLINED LIDAR / PLANAR EGO',
                         'lidar3d_ground': 'MEASURED GROUND / PLANAR EGO',
                         'lidar3d_ground_body': 'GROUND + BODY / PLANAR EGO'}.get(
                             native_evidence.get('operating_mode'), 'PHYSICAL CUBOIDS / PLANAR EGO')
                draw.text((610, 36), label, font=font(11, True), fill='#ffb46e')
            phase = (frame.get('navigation') or {}).get('phase', frame['trajectory']['mode'])
            draw.text((22, 606), f"{phase.upper()}   |   {frame['truth']['speed']*3.6:.1f} km/h   |   t = {frame['time']:.1f} s", font=font(15, True), fill='#edf4ff')
            draw.text((610, 608), 'BLUE ego   TRAFFIC actors   TEAL plan   /   3x', font=font(12), fill='#8698b3')
            images.append(image)
        if args.preview_time is not None:
            images[0].save(args.output.with_suffix('.png'))
            print(args.output.with_suffix('.png'))
            if args.scene_output:
                print(f'Editable scene: {args.scene_output}')
            return
        durations = [100]*(len(images)-1)+[1400]
        encode_gif(images, args.output, durations)
        images[min(len(images)-1, 50)].save(args.output.with_suffix('.png'))
        with Image.open(args.output) as gif:
            count, duration = gif.n_frames, 0
            for index in range(count):
                gif.seek(index)
                duration += gif.info['duration']
            assert gif.size == (960, 640) and duration == sum(durations)
        renderer_hash = hashlib.sha256()
        for source in ['blender_scene.py', 'blender_assets.py', 'render_demo_3d.py']:
            renderer_hash.update(source.encode())
            renderer_hash.update((ROOT/'scripts'/source).read_bytes())
        provenance = {'schema_version': 1, 'backend': run['backend'],
                      'renderer': 'Blender Cycles CPU', 'blender_version': subprocess.check_output(['blender', '--version'], text=True).splitlines()[0],
                      'input_trace': str(args.run), 'input_sha256': hashlib.sha256(args.run.read_bytes()).hexdigest(),
                      'rne_expected_revision': (ROOT/'integrations/rne/rne-revision.txt').read_text().strip(),
                      'scenario': run['scenario'], 'summary': run['summary'], 'gif_frames': count,
                      'sampled_frames': len(images), 'playback_speed': 3, 'scene_states_verified': len(audit),
                      'samples': args.samples, 'physics_domain': 'planar', 'scene': scene_info,
                      'renderer_source_sha256': renderer_hash.hexdigest(),
                      'gif_palette_colors': 192, 'gif_dither': False, 'spatial_filter': '3x3 median, viewport only',
                      'renderer_command': shlex.join(['python3','scripts/render_demo_3d.py',str(args.run),'--output',str(args.output),
                                                     '--samples',str(args.samples),'--threads',str(args.threads),'--camera',args.camera,
                                                     '--traffic-models',*args.traffic_models]+(['--scene-output',str(args.scene_output)] if args.scene_output else [])+
                                                    (['--native-scene',str(args.native_scene)] if args.native_scene else []))}
        if native_evidence:
            provenance['native_scene'] = {
                'input': str(args.native_scene),
                'input_sha256': hashlib.sha256(args.native_scene.read_bytes()).hexdigest(),
                'scene': native_evidence['scene'], 'summary': native_evidence['summary'],
                'seed': native_evidence['seed'],
                'body_pose_states_verified': len(frames),
                'cuboid_mesh_states_verified': len(audit),
                'operational_lidar_height_m': 0.6,
                'diagnostic_lidar_heights_m': [0.15, 3.7],
                'contact_response': False}
            if native_evidence.get('operating_mode') == 'multi_height_lidar':
                calibration = native_evidence['multi_height_lidar']
                details = provenance['native_scene']
                del details['operational_lidar_height_m']
                del details['diagnostic_lidar_heights_m']
                details.update({
                    'operating_mode': 'multi_height_lidar',
                    'measured_lidar_heights_m': calibration['heights_m'],
                    'projected_lidar_heights_m': sorted(h for h in calibration['heights_m']
                        if calibration['collision_bottom_m'] <= h <= calibration['collision_top_m']),
                    'collision_height_interval_m': [calibration['collision_bottom_m'], calibration['collision_top_m']],
                    'projection': 'calibrated height gate, then 5 cm body-XY first-point voxels'})
            elif native_evidence.get('operating_mode') in ['lidar3d', *GROUND_MODES]:
                calibration = native_evidence['lidar3d']
                details = provenance['native_scene']
                del details['operational_lidar_height_m']
                del details['diagnostic_lidar_heights_m']
                details.update({
                    'operating_mode': native_evidence['operating_mode'], 'lidar3d_calibration': calibration,
                    'cloud_frame': 'body forward/left, road-datum up',
                    'recorded_cloud_audit': verify_recorded_lidar3d(native_evidence, calibration),
                    'diagnostic_horizontal_heights_m': [0.6, 0.15, 3.7],
                    'collision_height_interval_m': [calibration['collision_bottom_m'], calibration['collision_top_m']],
                    'projection': 'measured Z gate, firing-ordinal order, then 5 cm body-XY first-return voxels',
                    'instantaneous_scan': True, 'deskew': False})
                if native_evidence['operating_mode'] in GROUND_MODES:
                    details.update({'ground_diagnostics_audit': verify_ground_diagnostics(native_evidence),
                                    'ground_mesh_states_verified': len(audit),
                                    'physical_road_top_m': 0.0,
                                    'projection': 'measured supported ground removal, measured Z gate, ordinal-order 5 cm body-XY voxels',
                                    'ground_contact_response': False,
                                    'gif_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
                                    'encoded_duration_ms': duration})
                    if 'body_guard' in native_evidence:
                        details['body_guard'] = {'calibration': native_evidence['body_guard']['calibration'],
                                                 'summary': native_evidence['body_guard']['summary'],
                                                 'mesh_states_verified': len(audit),
                                                 'visualization': 'separate wire envelope; cosmetic car dimensions are not calibrated',
                                                 'guard_scope': 'upright body versus static obstacle cuboids; explicit road excluded',
                                                 'traffic_geometry': 'native capsule proxies; circumscribed circular planar clearance'}
        args.output.with_suffix('.json').write_text(json.dumps(provenance, indent=2)+'\n')
        print(f'{args.output}: {count} frames, 960x640, {args.output.stat().st_size:,} bytes')


if __name__ == '__main__':
    main()
