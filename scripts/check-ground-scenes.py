#!/usr/bin/env python3
"""Independent road raycasts, measured ground diagnostics and physical acceptance.

Scene roles are consulted only by this offline evaluator. Driving inputs contain
unlabelled measured XYZ returns and generic sensor/ground calibration.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('inclined_checks', ROOT/'scripts/check-lidar-3d.py')
rays = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rays)
scenes, hazards, require, sha = rays.scenes, rays.hazards, rays.require, rays.sha
CASES = [
    ('ground-midbeam', 'native-ground-stop', 'sensed_stop'),
    ('ground-clear', 'native-ground-goal', 'healthy_goal'),
    ('ground-low-slab', 'native-ground-stop', 'sensed_stop'),
    ('ground-overhead', 'native-ground-goal', 'healthy_goal'),
    ('ground-moving-traffic', 'native-ground-traffic-stop', 'moving_traffic_stop'),
    ('ground-narrow-support', 'native-ground-support-fault', 'confidence_stop'),
]
GROUND = {'reference_height_m': 0.0, 'max_slope': 0.05, 'max_height_offset_m': 0.03,
          'residual_threshold_m': 0.02, 'fit_radius_m': 8.0, 'min_inliers': 200,
          'min_sector_inliers': 20, 'min_cell_inliers': 6}
BODY = {'length_m': 4.2, 'width_m': 1.8, 'height_m': 1.5, 'bottom_m': 0.15,
        'center_offset_body_m': [0.0, 0.0], 'pose_reference': 'native_planar_plant_reference',
        'speed_bound_m_s': 12.0, 'clearance_floor_m': 1.0,
        'calibration_kind': 'authored_research_dimensions'}
BODY_CASES = [(case, scenario.replace('native-ground-', 'native-body-'), category)
              for case, scenario, category in CASES if category != 'confidence_stop']
CAPSULE_BOUNDARY_BAND_M = 0.0001  # Empirical pinned f32/Parry GJK grazing band, not a numerical guarantee.


def capsule_axis_gap(point, actor):
    vertical = max(0.1-point[2], point[2]-1.1, 0.0)
    return math.hypot(math.dist(point[:2], scenes.xy(actor['position'])), vertical)-actor['radius']


def capsule_grazing(origin, vector, value, actors, boxes):
    """Certify a measured grazing hit in a bounded numerical ambiguity band.

    Native f32 support-map raycasts can report a tangent hit when an exact
    analytic capsule misses by tens of micrometers. This is counted explicitly;
    the checker makes no exact presence claim for these beams.
    """
    if value is None:
        return None
    measured_point = tuple(p+value*d for p, d in zip(origin, vector))
    certain = [distance for box in boxes if (distance := rays.box_ray(origin, vector, box)) is not None]
    for actor in actors:
        contracted = dict(actor, radius=actor['radius']-CAPSULE_BOUNDARY_BAND_M)
        if (distance := rays.capsule_ray(origin, vector, contracted)) is not None:
            certain.append(distance)
    if min(certain, default=math.inf) < value-rays.RANGE_TOLERANCE_M:
        return None
    for actor in actors:
        delta = (origin[0]-actor['position']['x'], origin[1]-actor['position']['y'])
        horizontal = vector[0]**2+vector[1]**2
        times = [0.0]
        if horizontal > 1e-14:
            times.append(max(0.0, -sum(a*b for a, b in zip(delta, vector))/horizontal))
        for height in [0.1, 1.1]:
            offset = (*delta, origin[2]-height)
            times.append(max(0.0, -sum(a*b for a, b in zip(offset, vector))))
            if abs(vector[2]) > 1e-14:
                times.append(max(0.0, (height-origin[2])/vector[2]))
        closest = min(times, key=lambda time: capsule_axis_gap(tuple(p+time*d for p, d in zip(origin, vector)), actor))
        beam_gap = capsule_axis_gap(tuple(p+closest*d for p, d in zip(origin, vector)), actor)
        measured_gap = capsule_axis_gap(measured_point, actor)
        expanded = dict(actor, radius=actor['radius']+CAPSULE_BOUNDARY_BAND_M)
        possible = rays.capsule_ray(origin, vector, expanded)
        if abs(beam_gap) <= CAPSULE_BOUNDARY_BAND_M and abs(measured_gap) <= CAPSULE_BOUNDARY_BAND_M \
                and 0.1 <= origin[2]+closest*vector[2] <= 1.1 and 0.1 <= measured_point[2] <= 1.1 \
                and abs(value-closest) <= rays.RANGE_TOLERANCE_M and possible is not None \
                and abs(value-possible) <= rays.RANGE_TOLERANCE_M:
            return {'beam_surface_gap_m': abs(beam_gap), 'measured_surface_gap_m': abs(measured_gap)}
    return None


def verify_poses(run, evidence, ticks, positions):
    observations = evidence['observations']
    require(len(observations) == len(ticks) and abs(evidence['motion_samples'][0]['time']) < 1e-10
            and abs(evidence['motion_samples'][-1]['time']-run['summary']['simulated_seconds']) < 1e-7,
            'native pose history does not cover all control clocks')
    for index, observation in enumerate(observations):
        require(abs(observation['time']-ticks[index]['input']['time']) < 1e-8
                and math.dist(scenes.xy(observation['pose']['position']), positions[index*10]) < 1e-7,
                'native observation differs from 200 Hz translation')
    for frame in run['frames']:
        observation = observations[round(frame['time']/0.05)]
        require(math.dist(scenes.xy(observation['pose']['position']), scenes.xy(frame['truth']['pose']['position'])) < 1e-7
                and abs(observation['pose']['yaw']-frame['truth']['pose']['yaw']) < 1e-8,
                'native acquisition pose differs from recorded driving truth')


def rectangle_vertices(center, half, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(center[0]+c*x-s*y, center[1]+s*x+c*y)
            for x, y in [(-half[0], -half[1]), (half[0], -half[1]),
                         (half[0], half[1]), (-half[0], half[1])]]


def rectangle_distance(first, second):
    # Edge crossings and point containment; no native separating-axis helper.
    def inside(point, polygon):
        signs = [(b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0])
                 for a, b in zip(polygon, polygon[1:]+polygon[:1])]
        return all(value >= 0 for value in signs) or all(value <= 0 for value in signs)
    if inside(first[0], second) or inside(second[0], first):
        return 0.0
    best = math.inf
    for a, b in zip(first, first[1:]+first[:1]):
        for c, d in zip(second, second[1:]+second[:1]):
            if scenes.intersect_segments(a, b, c, d):
                return 0.0
            best = min(best, scenes.point_segment_distance(a, c, d), scenes.point_segment_distance(b, c, d),
                       scenes.point_segment_distance(c, a, b), scenes.point_segment_distance(d, a, b))
    return best


def body_clearance(pose, box):
    car = rectangle_vertices(scenes.xy(pose['position']), (2.1, 0.9), pose['yaw'])
    obstacle = rectangle_vertices(box['center_m'][:2], box['half_extents_m'][:2], box['yaw_rad'])
    vertical = max(box['center_m'][2]-box['half_extents_m'][2]-1.65,
                   0.15-box['center_m'][2]-box['half_extents_m'][2], 0.0)
    return math.hypot(rectangle_distance(car, obstacle), vertical)


def check_body(run, scene, evidence, ticks):
    require(evidence['schema_version'] == 1 and evidence['scene'] == scene
            and evidence['backend'] == run['backend'] and evidence['seed'] == run['summary']['seed']
            and evidence['scenario'] == run['scenario']['name'], 'body scene capture identity differs')
    capture = evidence['body_guard']
    require(capture['schema_version'] == 1 and capture['calibration'] == BODY
            and abs(run['vehicle']['radius']-math.hypot(2.1, 0.9)) < 1e-12
            and 'ego_capsule' not in evidence, 'physical car dimensions or acceptance mode changed')
    samples = capture['motion_samples']
    require(len(samples) == (len(ticks)-1)*10+1 and len(evidence['motion_samples']) == len(samples),
            'body guard omits 200 Hz yaw poses')
    positions = [scenes.xy(point['pose']['position']) for point in samples]
    require(all(abs(point['time']-legacy['time']) < 1e-8
                and math.dist(position, legacy['position']) < 1e-7
                for point, position, legacy in zip(samples, positions, evidence['motion_samples'])),
            'body capture differs from actual native translation history')
    verify_poses(run, evidence, ticks, positions)
    minimum, overlaps = math.inf, 0
    for index, (a, b) in enumerate(zip(samples, samples[1:])):
        dt = b['time']-a['time']
        angle = abs(math.remainder(b['pose']['yaw']-a['pose']['yaw'], 2*math.pi))
        require(abs(dt-0.005) < 1e-8 and math.dist(positions[index], positions[index+1])/dt <= 12+1e-8
                and angle <= 0.1+1e-9, 'body substep exceeds calibrated translation/rotation bounds')
        inflation = 12*dt/2+math.hypot(2.1, 0.9)*angle/2
        clearances = [max(min(body_clearance(a['pose'], box), body_clearance(b['pose'], box))-inflation, 0.0)
                      for box in scene['static_cuboids']]
        minimum = min([minimum]+clearances)
        overlaps += any(value <= 0 for value in clearances)
    witnesses = capture['rapier_sensor_witnesses']
    require(len(witnesses) == len(samples), 'native Rapier body witness omitted a substep')
    native_overlaps = 0
    for sample, witness in zip(samples, witnesses):
        expected = sorted(box['id'] for box in scene['static_cuboids'] if body_clearance(sample['pose'], box) <= 1e-7)
        require(abs(witness['time']-sample['time']) < 1e-8 and witness['force_free'] is True
                and witness['obstacle_ids'] == expected,
                'actual Rapier sensor overlap differs from independent upright-box geometry')
        native_overlaps += bool(expected)
        for box in scene['static_cuboids']:
            minimum = min(minimum, body_clearance(sample['pose'], box))
    minimum = None if math.isinf(minimum) else minimum
    summary = capture['summary']
    require(evidence['summary'] == summary and summary['checks'] == (2*len(samples)-1)*len(scene['static_cuboids'])
            and summary['guard_overlap_intervals'] == overlaps and summary['native_overlap_samples'] == native_overlaps
            and ((minimum is None and summary['min_clearance_m'] is None)
                 or (minimum is not None and abs(summary['min_clearance_m']-minimum) < 1e-7)),
            'body guard summary differs from independent polygon-distance motion audit')
    passed = (minimum is None or minimum >= 1.0) and overlaps == 0 and native_overlaps == 0
    require(summary['passed'] == passed, 'body acceptance differs from fixed physical floor')
    return {'passed': passed, 'min_clearance_m': minimum, 'guard_overlap_intervals': overlaps,
            'native_overlap_samples': native_overlaps, 'motion_samples': len(samples),
            'body_geometry_verified': True, 'rapier_sensor_witnesses_verified': True,
            'scope': 'upright static cuboids; moving traffic uses conservative circumscribed-circle sweeps'}


def sector(point):
    return min(7, math.floor((math.atan2(point['y'], point['x'])+math.pi)/(2*math.pi)*8))


def cell(point):
    return sector(point), math.floor(math.hypot(point['x'], point['y'])/5)


def least_squares(points):
    """Centered 2x2 covariance fit, independent of the driver's 3x3 elimination."""
    require(len(points) >= 3, 'reported ground plane lacks three independent measurements')
    n = len(points)
    means = [math.fsum(p[key] for p in points)/n for key in ['x', 'y', 'z']]
    xx = math.fsum((p['x']-means[0])**2 for p in points)
    yy = math.fsum((p['y']-means[1])**2 for p in points)
    xy = math.fsum((p['x']-means[0])*(p['y']-means[1]) for p in points)
    xz = math.fsum((p['x']-means[0])*(p['z']-means[2]) for p in points)
    yz = math.fsum((p['y']-means[1])*(p['z']-means[2]) for p in points)
    determinant = xx*yy-xy*xy
    require(determinant > 1e-9, 'ground fit has degenerate measured spatial support')
    a, b = (xz*yy-yz*xy)/determinant, (yz*xx-xz*xy)/determinant
    return a, b, means[2]-a*means[0]-b*means[1]


def check_plane(scan, diagnostic):
    points = [measured['point'] for measured in scan['returns']]
    require(diagnostic is not None and abs(diagnostic['stamp']-scan['stamp']) < 1e-8,
            'fresh XYZ acquisition lacks same-clock measured ground diagnostics')
    candidates = [p for p in points if 0.5 <= math.hypot(p['x'], p['y']) <= GROUND['fit_radius_m']
                  and abs(p['z']-GROUND['reference_height_m']) <= GROUND['max_height_offset_m']
                  +GROUND['max_slope']*math.hypot(p['x'], p['y'])+GROUND['residual_threshold_m']]
    require(diagnostic['candidate_points'] == len(candidates), 'ground candidate count differs from measured XYZ')
    plane = diagnostic['plane']
    removed = [False]*len(points)
    reconstruction_error = 0.0
    if plane is None:
        require(not diagnostic['confidence'] and diagnostic['inliers'] == 0
                and diagnostic['sector_inliers'] == [0]*8 and diagnostic['supported_cells'] == 0,
                'missing ground plane claims measured support')
    else:
        a, b, c = plane['a'], plane['b'], plane['c']
        require(all(math.isfinite(v) for v in [a, b, c]) and math.hypot(a, b) <= GROUND['max_slope']
                and abs(c-GROUND['reference_height_m']) <= GROUND['max_height_offset_m'],
                'fitted plane exceeds bounded sensor calibration')
        def residual(p):
            return abs(p['z']-(a*p['x']+b*p['y']+c))
        inliers = [p for p in candidates if residual(p) <= GROUND['residual_threshold_m']]
        sectors = [sum(sector(p) == index for p in inliers) for index in range(8)]
        require(diagnostic['inliers'] == len(inliers) and diagnostic['sector_inliers'] == sectors,
                'fitted-plane inlier or sector counts differ from raw measurements')
        if inliers:
            maximum = max(residual(p) for p in inliers)
            rms = math.sqrt(math.fsum(residual(p)**2 for p in inliers)/len(inliers))
            require(abs(diagnostic['max_inlier_residual_m']-maximum) < 1e-7
                    and abs(diagnostic['rms_residual_m']-rms) < 1e-7,
                    'reported plane residual statistics differ from measured inliers')
            fit_a, fit_b, fit_c = least_squares(inliers)
            reconstruction_error = math.hypot(a-fit_a, b-fit_b)*GROUND['fit_radius_m']+abs(c-fit_c)
            require(reconstruction_error <= 1e-4,
                    'reported plane is not the measured-inlier least-squares fit')
        confidence = len(inliers) >= GROUND['min_inliers'] and min(sectors) >= GROUND['min_sector_inliers']
        require(diagnostic['confidence'] == confidence, 'ground confidence differs from measured spatial support')
        if confidence:
            support = {}
            for point in points:
                if residual(point) <= GROUND['residual_threshold_m']:
                    support[cell(point)] = support.get(cell(point), 0)+1
            supported = {key for key, value in support.items() if value >= GROUND['min_cell_inliers']}
            require(diagnostic['supported_cells'] == len(supported), 'ground support-cell count is inconsistent')
            removed = [residual(point) <= GROUND['residual_threshold_m'] and cell(point) in supported for point in points]
        else:
            require(diagnostic['supported_cells'] == 0, 'unconfident plane removed unsupported ground cells')
    require(diagnostic['removed_points'] == sum(removed)
            and diagnostic['preserved_points'] == len(points)-sum(removed),
            'ground removal/preservation counts differ from supported measured residuals')
    return removed, reconstruction_error


def check_capsule(run, scene, evidence, ticks):
    if scene['static_cuboids']:
        return rays.check_guard(run, scene, evidence, ticks)
    require(evidence['schema_version'] == 1 and evidence['scene'] == scene
            and evidence['backend'] == run['backend'] and evidence['seed'] == run['summary']['seed']
            and evidence['scenario'] == run['scenario']['name'], 'empty-obstacle scene capture identity differs')
    samples = evidence['motion_samples']
    require(evidence['ego_capsule'] == {'axis_bottom_m': 0.1, 'axis_top_m': 1.1,
            'radius_m': run['vehicle']['radius'], 'speed_bound_m_s': 12.0, 'clearance_floor_m': 1.0},
            'native capsule calibration changed')
    require(len(samples) == (len(ticks)-1)*10+1, 'native scene omits 200 Hz motion')
    for a, b in zip(samples, samples[1:]):
        dt = b['time']-a['time']
        require(abs(dt-0.005) < 1e-8 and math.dist(a['position'], b['position'])/dt <= 12+1e-8,
                'native scene translation interval or speed bound changed')
    require(evidence['summary']['passed'] and evidence['summary']['min_clearance_m'] is None
            and evidence['summary']['checks'] == 0 and evidence['summary']['guard_overlap_intervals'] == 0,
            'authored ground was incorrectly counted as a physical obstacle')
    verify_poses(run, evidence, ticks, [sample['position'] for sample in samples])
    for frame in run['frames']:
        sample = samples[round(frame['time']/0.005)]
        require(math.dist(sample['position'], scenes.xy(frame['truth']['pose']['position'])) < 1e-7,
                'empty-obstacle motion evidence differs from recorded truth')
    return {'passed': True, 'min_clearance_m': None, 'guard_overlap_intervals': 0,
            'motion_samples': len(samples), 'static_obstacle_checks': 0}


def directions(config):
    result = []
    for column in range(config['azimuth_columns']):
        azimuth = -math.pi+2*math.pi*column/(config['azimuth_columns']-1)
        for ring in range(config['elevation_rings']):
            elevation = config['min_elevation_rad']+(config['max_elevation_rad']-config['min_elevation_rad'])*ring/(config['elevation_rings']-1)
            result.append((math.cos(elevation)*math.cos(azimuth), -math.cos(elevation)*math.sin(azimuth), math.sin(elevation)))
    return result


def check_ground(run, scene, evidence, log):
    header, ticks = rays.load_log(log)
    calibration = rays.calibration(run['vehicle']['radius'])
    calibration['azimuth_columns'] = 180
    calibration['ground'] = GROUND
    body = evidence['operating_mode'] == 'lidar3d_ground_body'
    if body:
        calibration.update(collision_bottom_m=-0.85, collision_top_m=2.65)
    require(evidence['operating_mode'] in ['lidar3d_ground', 'lidar3d_ground_body'] and evidence['lidar3d'] == calibration
            and header['config'].get('lidar3d') == calibration, 'ground/beam calibration differs between native sensing and replay')
    require(scene['schema_version'] == 2 and scene['ground_cuboids'], 'ground fixture lacks explicit offline ground geometry')
    require(len(ticks) == run['summary']['steps'] and header['config']['vehicle'] == run['vehicle'],
            'ground run sensor count or vehicle calibration changed')
    forbidden = ['scene', 'static_cuboids', 'ground_cuboids', 'center_m', 'half_extents_m', 'acquisitions', 'motion_samples']
    require(not any(f'"{field}"' in json.dumps(header['config']) for field in forbidden),
            'authored road/obstacle truth leaked into operational ground calibration')
    measured = check_body(run, scene, evidence, ticks) if body else check_capsule(run, scene, evidence, ticks)
    beams = directions(calibration)
    acquisitions = evidence['acquisitions']
    recorded_frames = {round(frame['time']/0.05): frame for frame in run['frames']}
    require(len(acquisitions) == (len(ticks)+1)//2, 'ground scan capture omits native firing clocks')
    totals = {'ray_grid_entries_verified': 0, 'xyz_returns_verified': 0, 'ground_removed_points': 0,
              'preserved_points': 0, 'height_eligible_preserved_points': 0, 'expected_5cm_voxel_returns': 0,
              'confident_acquisitions': 0, 'unconfident_acquisitions': 0,
              'actual_ground_returns': 0, 'removed_actual_ground_returns': 0,
              'actual_static_returns': 0, 'removed_actual_static_returns': 0,
              'height_eligible_static_returns': 0,
              'actual_actor_returns': 0, 'preserved_actual_actor_returns': 0}
    totals['empirical_capsule_boundary_hits'] = 0
    maximum_boundary_gap = 0.0
    maximum_error, maximum_fit_error = 0.0, 0.0
    for acquisition_index, acquisition in enumerate(acquisitions):
        tick_index = acquisition_index*2
        tick = ticks[tick_index]
        observation = evidence['observations'][tick_index]
        require(abs(acquisition['time']-tick['input']['time']) < 1e-8 and acquisition['pose'] == observation['pose'],
                'ground ray pose or clock differs from acquisition truth')
        if tick_index in recorded_frames:
            require(acquisition['objects'] == recorded_frames[tick_index]['objects'],
                    'ground actor ray geometry differs from recorded physical traffic')
        scan = tick['input'].get('lidar3d')
        require(scan is not None and abs(scan['stamp']-acquisition['time']) < 1e-8
                and tick['input'].get('lidar') is None and tick['input'].get('multi_height_lidar') is None,
                'ground sensing mode mixes raw inputs or loses acquisition clocks')
        cloud = acquisition['cloud_3d']
        require(cloud['returns'] == scan['returns'] and len(cloud['ranges_m']) == len(beams),
                'ground driving returns differ from complete actual native firing grid')
        removed, fit_error = check_plane(scan, tick['expected'].get('ground'))
        maximum_fit_error = max(maximum_fit_error, fit_error)
        confidence = tick['expected']['ground']['confidence']
        totals['confident_acquisitions' if confidence else 'unconfident_acquisitions'] += 1
        require(confidence or tick['expected']['emergency'], 'unconfident measured ground did not request emergency braking')
        origin = (*scenes.xy(acquisition['pose']['position']), calibration['mount_height_m'])
        yaw = acquisition['pose']['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        nonnull = []
        for ordinal, value in enumerate(cloud['ranges_m']):
            dx, dy, dz = beams[ordinal]
            vector = c*dx-s*dy, s*dx+c*dy, dz
            candidates = [(distance, 'ground') for box in scene['ground_cuboids']
                          if (distance := rays.box_ray(origin, vector, box)) is not None]
            candidates += [(distance, 'static') for box in scene['static_cuboids']
                           if (distance := rays.box_ray(origin, vector, box)) is not None]
            candidates += [(distance, 'actor') for obj in acquisition['objects']
                           if (distance := rays.capsule_ray(origin, vector, obj)) is not None]
            nearest, role = min(candidates, default=(math.inf, None))
            ambiguous = None
            if (value is not None) != (0.2 <= nearest <= 45) or value is not None and abs(value-nearest) > rays.RANGE_TOLERANCE_M:
                ambiguous = capsule_grazing(origin, vector, value, acquisition['objects'],
                                             scene['ground_cuboids']+scene['static_cuboids'])
                if ambiguous:
                    totals['empirical_capsule_boundary_hits'] += 1
                    maximum_boundary_gap = max(maximum_boundary_gap, *ambiguous.values())
                    nearest, role = value, 'actor'
            require(ambiguous is not None or (value is not None) == (0.2 <= nearest <= 45),
                    f'ground/obstacle nearest ray presence differs at t={acquisition["time"]}, ordinal={ordinal}')
            totals['ray_grid_entries_verified'] += 1
            if value is not None:
                require(math.isfinite(value) and 0.2-1e-9 <= value <= 45+1e-9
                        and abs(value-nearest) <= rays.RANGE_TOLERANCE_M,
                        f'ground-mode range differs from actual nearest 3D physical intersection at t={acquisition["time"]}, ordinal={ordinal}, measured={value}, expected={nearest}, role={role}')
                maximum_error = max(maximum_error, abs(value-nearest))
                nonnull.append((ordinal, value, role))
        require(len(nonnull) == len(scan['returns']), 'ground raw XYZ count differs from native firing returns')
        voxels = set()
        for index, (measured_return, (ordinal, value, role)) in enumerate(zip(scan['returns'], nonnull)):
            point = measured_return['point']
            dx, dy, dz = beams[ordinal]
            require(measured_return['ray_index'] == ordinal
                    and math.dist((point['x'], point['y'], point['z']),
                                  (value*dx, value*dy, calibration['mount_height_m']+value*dz)) < 1e-7,
                    'ground-mode body XYZ or ordinal differs from native inclined ray')
            totals[f'actual_{role}_returns'] += 1
            if role == 'static' and calibration['collision_bottom_m'] <= point['z'] <= calibration['collision_top_m']:
                totals['height_eligible_static_returns'] += 1
            if removed[index] and role in ['ground', 'static']:
                totals[f'removed_actual_{role}_returns'] += 1
            if role == 'actor' and not removed[index]:
                totals['preserved_actual_actor_returns'] += 1
            if not removed[index] and calibration['collision_bottom_m'] <= point['z'] <= calibration['collision_top_m']:
                totals['height_eligible_preserved_points'] += 1
                voxels.add((math.floor(point['x']/0.05), math.floor(point['y']/0.05)))
        totals['xyz_returns_verified'] += len(nonnull)
        totals['ground_removed_points'] += sum(removed)
        totals['preserved_points'] += len(removed)-sum(removed)
        totals['expected_5cm_voxel_returns'] += len(voxels)
    for index, tick in enumerate(ticks):
        if index % 2:
            require(tick['expected'].get('ground') is None and tick['input'].get('lidar3d') is None,
                    'ground diagnostics or acquisition persisted between native firing clocks')
    measured.update(totals)
    measured.update({'acquisitions': len(acquisitions), 'maximum_range_residual_m': maximum_error,
                     'maximum_fitted_plane_reconstruction_error_m': maximum_fit_error,
                     'range_tolerance_m': rays.RANGE_TOLERANCE_M,
                     'empirical_capsule_boundary_band_m': CAPSULE_BOUNDARY_BAND_M,
                     'maximum_empirical_boundary_surface_gap_m': maximum_boundary_gap,
                     'exact_presence_beams_verified': totals['ray_grid_entries_verified']-totals['empirical_capsule_boundary_hits'],
                     'capsule_boundary_scope': 'empirical pinned f32/GJK ambiguity, not a certified floating-point error bound',
                     'raw_ground_geometry_verified': True, 'ground_classification_verified': True,
                     'sensor_only_boundary_verified': True})
    return measured


def mutations(run, scene, evidence, log, cli):
    records = [json.loads(line) for line in log.read_text().splitlines()]
    first = next(index for index, record in enumerate(records) if record.get('kind') == 'tick'
                 and record['tick']['expected'].get('ground') is not None)
    changed = log.parent/'mutation-sensors.jsonl'
    replay_output = log.parent/'mutation-replay'
    results = []
    try:
        for name in ['changed-fit-plane', 'changed-removed-count', 'changed-confidence', 'changed-ground-prior', 'changed-raw-xyz']:
            altered = copy.deepcopy(records)
            diagnostic = altered[first]['tick']['expected']['ground']
            if name == 'changed-fit-plane':
                if diagnostic['plane']:
                    diagnostic['plane']['c'] += 0.01
                else:
                    diagnostic['plane'] = {'a': 0.0, 'b': 0.0, 'c': 0.01}
            elif name == 'changed-removed-count':
                diagnostic['removed_points'] += 1
            elif name == 'changed-confidence':
                diagnostic['confidence'] = not diagnostic['confidence']
            elif name == 'changed-ground-prior':
                # Invalid mount/prior gap must fail even when the original scan
                # was already unconfident and its brake command is unchanged.
                altered[0]['header']['config']['lidar3d']['ground']['reference_height_m'] = 0.55
            else:
                altered[first]['tick']['input']['lidar3d']['returns'][0]['point']['z'] += 0.1
            changed.write_text(''.join(json.dumps(record, separators=(',', ':'))+'\n' for record in altered))
            try:
                check_ground(run, scene, evidence, changed)
            except ValueError as error:
                reason = str(error)
            else:
                raise ValueError(f'measured ground oracle accepted mutation {name}')
            (replay_output/'replay.json').unlink(missing_ok=True)
            code, stderr = hazards.invoke([cli, 'replay', '--log', changed, '--output', replay_output])
            require(code != 0, f'Rust replay accepted changed measured-ground evidence {name}')
            results.append({'mutation': name, 'rejected': True, 'oracle_reason': reason,
                            'replay_exit_code': code, 'mutated_log_sha256': sha(changed), 'replay_stderr': stderr[:1000]})
        if 'body_guard' in evidence:
            _, ticks = rays.load_log(log)
            for name in ['changed-body-width', 'missing-native-witness', 'changed-body-yaw', 'changed-body-clearance']:
                altered = copy.deepcopy(evidence)
                if name == 'changed-body-width':
                    altered['body_guard']['calibration']['width_m'] += 0.1
                elif name == 'missing-native-witness':
                    altered['body_guard']['rapier_sensor_witnesses'].pop()
                elif name == 'changed-body-yaw':
                    altered['body_guard']['motion_samples'][1]['pose']['yaw'] += 0.2
                else:
                    altered['body_guard']['summary']['min_clearance_m'] = 123.0
                try:
                    check_body(run, scene, altered, ticks)
                except ValueError as error:
                    results.append({'mutation': name, 'rejected': True, 'oracle_reason': str(error),
                                    'scope': 'offline physical capture; no driver ground-truth input'})
                else:
                    raise ValueError(f'physical body oracle accepted mutation {name}')
        altered_scene, altered_capture = copy.deepcopy(scene), copy.deepcopy(evidence)
        altered_scene['ground_cuboids'][0]['center_m'][2] += 0.1
        altered_capture['scene'] = altered_scene
        try:
            check_ground(run, altered_scene, altered_capture, log)
        except ValueError as error:
            results.append({'mutation': 'changed-actual-road-height', 'rejected': True,
                            'oracle_reason': str(error), 'scope': 'offline actual-ray geometry'})
        else:
            raise ValueError('ground oracle accepted altered physical road height')
        actor = {'position': {'x': 10.0, 'y': 0.0}, 'radius': 1.0}
        require(capsule_grazing((0.0, 1.00005, 0.6), (1.0, 0.0, 0.0), 10.0, [actor], []) is not None,
                'empirical grazing classifier cannot reconstruct a within-band tangent')
        blocking_box = {'center_m': [5.0, 1.0, 0.6], 'half_extents_m': [0.5, 0.2, 0.1], 'yaw_rad': 0.0}
        for name, origin, value, boxes in [
                ('outside-capsule-boundary-band', (0.0, 1.0002, 0.6), 10.0, []),
                ('outside-tangent-range-bound', (0.0, 1.00005, 0.6), 10.1, []),
                ('grazing-behind-certain-geometry', (0.0, 1.00005, 0.6), 10.0, [blocking_box])]:
            require(capsule_grazing(origin, (1.0, 0.0, 0.0), value, [actor], boxes) is None,
                    f'empirical boundary classifier accepted corrupt query {name}')
            results.append({'mutation': name, 'rejected': True, 'scope': 'independent empirical grazing classifier'})
    finally:
        changed.unlink(missing_ok=True)
        for name in ['replay.json', 'outputs.jsonl']:
            (replay_output/name).unlink(missing_ok=True)
        if replay_output.exists():
            replay_output.rmdir()
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/ground-scenes')
    parser.add_argument('--compact', action='store_true')
    parser.add_argument('--plants', nargs='+', choices=['dynamic', 'kinematic'], default=['dynamic', 'kinematic'])
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--modes', nargs='+', choices=['lidar3d_ground', 'lidar3d_ground_body'], default=['lidar3d_ground', 'lidar3d_ground_body'])
    parser.add_argument('--cases', nargs='+', choices=[case for case, _, _ in CASES])
    args = parser.parse_args()
    require(len(set(args.plants)) == len(args.plants) and len(set(args.modes)) == len(args.modes)
            and len(set(args.seeds)) == len(args.seeds)
            and all(0 <= seed < 2**64 for seed in args.seeds), 'plants/seeds must be distinct with u64 seeds')
    cli, native = ROOT/'target/release/rustdrive', ROOT/'integrations/rne/target/release/rustdrive-rne'
    require(cli.is_file() and native.is_file(), 'build locked reference and native release binaries first')
    pin = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    head = subprocess.run(['git', '-C', ROOT.parent/'RobotNativeEngine', 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    require(head == pin, 'RNE checkout differs from the committed revision')
    args.output.mkdir(parents=True, exist_ok=True)
    report_file = args.output/'report.json'
    report_file.unlink(missing_ok=True)
    proof_file = ROOT/'scripts/fixtures/native-capsule-grazing-proof.json'
    dependencies = [Path(__file__), ROOT/'scripts/check-lidar-3d.py', ROOT/'scripts/check-native-scenes.py', ROOT/'scripts/check_hazards.py', proof_file]
    inputs = [ROOT/'scenes'/f'{case}.json' for case, _, _ in CASES]+[ROOT/'scenarios'/f'{name}.json' for name in sorted({name for _, name, _ in CASES+BODY_CASES})]
    report = {'schema_version': 1, 'source_fingerprint_sha256': hazards.source_fingerprint(),
              'checker_sha256': sha(Path(__file__)), 'checker_dependency_sha256': {str(path.relative_to(ROOT)): sha(path) for path in dependencies},
              'scene_inputs_sha256': {str(path.relative_to(ROOT)): sha(path) for path in inputs}, 'rne_revision': pin,
              'plants': args.plants, 'seeds': args.seeds, 'modes': args.modes,
              'runs': [], 'passed': False, 'compact': args.compact,
              'separate_native_rejection_test': 'body::tests::ignoring_native_braking_cannot_hide_actual_cuboid_overlap'}
    report['native_grazing_reproduction'] = json.loads(proof_file.read_text())
    for mode in args.modes:
        for plant in args.plants:
            for case, scenario_name, category in (BODY_CASES if mode == 'lidar3d_ground_body' else CASES):
                if args.cases and case not in args.cases:
                    continue
                scene = json.loads((ROOT/'scenes'/f'{case}.json').read_text())
                for seed in args.seeds:
                    output = args.output/mode/plant/case/f'seed-{seed}'
                    output.mkdir(parents=True, exist_ok=True)
                    for name in ['run.json', 'scene.json', 'summary.json', 'sensors.jsonl', 'replay/replay.json']:
                        (output/name).unlink(missing_ok=True)
                    code, stderr = hazards.invoke([native, '--scene', ROOT/'scenes'/f'{case}.json',
                        '--scenario', ROOT/'scenarios'/f'{scenario_name}.json', '--plant', plant, '--seed', seed,
                        '--output', output, '--lidar-3d', '--ground-segmentation']+(['--vehicle-body'] if mode == 'lidar3d_ground_body' else []))
                    run = json.loads((output/'run.json').read_text())
                    evidence = json.loads((output/'scene.json').read_text())
                    replay_code, replay_stderr = hazards.invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                    replay = json.loads((output/'replay/replay.json').read_text())
                    require(code == 0 and run['summary']['passed'] and replay_code == 0 and replay['verified']
                            and replay['ticks'] == run['summary']['steps'], 'ground episode does not pass physical acceptance and exact sensor replay')
                    measured = check_ground(run, scene, evidence, output/'sensors.jsonl')
                    require(measured['passed'] and run['summary']['collisions'] == 0 and run['summary']['road_violations'] == 0,
                            'ground road/static obstacle clearance regressed')
                    if category == 'confidence_stop':
                        require(measured['unconfident_acquisitions'] == measured['acquisitions']
                                and run['summary']['final_speed'] <= 0.2 and run['summary']['emergency_steps'] > 0
                                and not run['summary']['reached_goal'], 'unsupported-ground case did not demonstrate confidence braking')
                    else:
                        require(measured['confident_acquisitions'] == measured['acquisitions']
                                and measured['removed_actual_ground_returns'] > 0, 'healthy case lacks measured supported-ground removal')
                        if category == 'healthy_goal':
                            require(run['summary']['reached_goal'], 'ground/overhead goal not reached')
                            if case == 'ground-overhead':
                                require(measured['actual_static_returns'] > 0 and measured['height_eligible_static_returns'] == 0,
                                        'overhead goal lacks measured elevated returns excluded by calibrated body window')
                        else:
                            require(not run['summary']['reached_goal'] and run['summary']['final_speed'] <= 0.2
                                    and measured['height_eligible_preserved_points'] > 0, 'measured obstacle/traffic did not cause a stop')
                        if case in ['ground-low-slab', 'ground-midbeam']:
                            require(measured['actual_static_returns'] > measured['removed_actual_static_returns']
                                    and measured['height_eligible_static_returns'] > 0,
                                    'ground separation preserved no measured height-eligible obstacle returns')
                    controls = hazards.control_metrics(output/'sensors.jsonl')
                    require(controls['max_normal_commanded_steering_rate_rad_s'] <= 0.7+1e-8, 'ground mode exceeds steering-rate calibration')
                    row = {'operating_mode': mode, 'plant': plant, 'scene': case, 'scenario': scenario_name, 'category': category, 'seed': seed,
                           'output': str(output), 'exit_code': code, 'summary': run['summary'], 'scene_summary': evidence['summary'],
                           'independent_scene': measured, 'replay': replay, 'control_metrics': controls,
                           'motion_predictions': hazards.check_motion_predictions(output/'sensors.jsonl'),
                           'passed': True, 'raw_sha256': {name: sha(output/name) for name in ['run.json', 'scene.json', 'sensors.jsonl']}}
                    if category != 'confidence_stop':
                        row['speed_profiles'] = hazards.check_speed_profiles(output/'sensors.jsonl')
                    if case == 'ground-moving-traffic':
                        require(measured['actual_actor_returns'] > 0 and measured['preserved_actual_actor_returns'] > 0
                                and run['summary']['min_clearance'] >= 1.0, 'moving ground traffic lacks measured preserved actor returns or clearance')
                        row['traffic'] = hazards.check_traffic(run, output/'sensors.jsonl', case)
                        require(any(abs(actor['speed_m_s']) > 0.5 for frame in run['frames'] for actor in frame['traffic']),
                                'moving-ground fixture never actually moved')
                    if stderr:
                        row['stderr'] = stderr
                    if replay_stderr:
                        row['replay_stderr'] = replay_stderr
                    if seed == args.seeds[0]:
                        row['mutation_rejections'] = mutations(run, scene, evidence, output/'sensors.jsonl', cli)
                    if args.compact:
                        preserve = plant == 'dynamic' and case in ['ground-midbeam', 'ground-moving-traffic'] and seed == 7
                        row['archive'] = rays.compact_case(output, row, preserve)
                    report['runs'].append(row)
                    report_file.write_text(json.dumps(report, indent=2)+'\n')
                    print(f'{plant:10s} {case:22s} seed {seed:3d}: PASS ({category}); ground removals {measured["ground_removed_points"]}', flush=True)
    require(report['runs'], 'selected ground matrix contains no episodes')
    require(report['source_fingerprint_sha256'] == hazards.source_fingerprint()
            and report['checker_dependency_sha256'] == {str(path.relative_to(ROOT)): sha(path) for path in dependencies}
            and report['scene_inputs_sha256'] == {str(path.relative_to(ROOT)): sha(path) for path in inputs},
            'ground source, checker or physical fixtures changed during acceptance')
    report['accepted_episodes'] = len(report['runs'])
    report['healthy_accepted_episodes'] = sum(row['category'] != 'confidence_stop' for row in report['runs'])
    report['confidence_stop_episodes'] = sum(row['category'] == 'confidence_stop' for row in report['runs'])
    report['body_accepted_episodes'] = sum(row['operating_mode'] == 'lidar3d_ground_body' for row in report['runs'])
    report['mutation_rejections'] = sum(len(row.get('mutation_rejections', [])) for row in report['runs'])
    report['empirical_capsule_boundary_hits'] = sum(row['independent_scene']['empirical_capsule_boundary_hits'] for row in report['runs'])
    report['maximum_empirical_boundary_surface_gap_m'] = max(row['independent_scene']['maximum_empirical_boundary_surface_gap_m'] for row in report['runs'])
    report['complete'] = True
    report['passed'] = True
    report['categories'] = {category: sum(row['category'] == category for row in report['runs']) for _, _, category in CASES}
    report_file.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_file}: {len(report["runs"])} accepted episodes; categories={report["categories"]}; passed=True')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'ground-scene check: {error}', file=sys.stderr)
        sys.exit(2)
