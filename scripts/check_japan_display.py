"""Independent RustDriving Japanese display geometry audit, not a driving test.

Run from the repository with Blender:
blender --background --factory-startup --threads 2 --python-exit-code 1 \
  --python scripts/check_japan_display.py -- \
  --output artifacts/japan-display-audit.json

SPDX-License-Identifier: Apache-2.0
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector

SCRIPT_DIRECTORY = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIRECTORY))
import blender_japan_assets as japan
from blender_assets import car

TOLERANCE_M = 2e-5
JUNCTIONS = ((15., 3.4), (36., 1.35))
JUNCTION_ONE_WAY = (False, True)
MAIN_LANES = (-8., 0., 16.)
WORKER_POSES = ((0, 0), (.19, .6), (.37, .7), (.71, 1.3), (1.3, .5))


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def source_hashes():
    names = (Path(__file__).name, 'blender_japan_assets.py',
             'blender_city_assets.py', 'blender_vru_assets.py', 'blender_assets.py')
    return {name: hashlib.sha256((SCRIPT_DIRECTORY/name).read_bytes()).hexdigest()
            for name in names}


def evaluated_points(root):
    graph = bpy.context.evaluated_depsgraph_get()
    inverse = root.matrix_world.inverted()
    points = []
    for obj in root.children_recursive:
        if obj.type not in ('MESH', 'CURVE', 'FONT'):
            continue
        evaluated = obj.evaluated_get(graph)
        mesh = evaluated.to_mesh()
        try:
            transform = inverse @ evaluated.matrix_world
            points.extend(transform @ vertex.co for vertex in mesh.vertices)
        finally:
            evaluated.to_mesh_clear()
    require(bool(points), 'Expected evaluated display geometry')
    return points


def root_pose(root):
    return tuple(root.location), tuple(root.rotation_euler), tuple(root.scale)


def check_workers():
    cases = []
    previous = None
    for radius in (.4, 1.8):
        root = japan.worker('RustDriving worker', radius)
        root.location, root.rotation_euler.z = (17, -9, 0), .72
        before = root_pose(root)
        require(root['declared_circle_radius'] == radius, 'Physical radius metadata changed')
        measurements = []
        for time, speed in WORKER_POSES:
            japan.animate_worker(root, time, speed)
            bpy.context.view_layer.update()
            vertices = evaluated_points(root)
            require(root_pose(root) == before and tuple(root.scale) == (1, 1, 1),
                    'Cosmetic worker animation changed the root pose or unit scale')
            require(max(math.hypot(v.x, v.y) for v in vertices) <= root['display_radius_m']+TOLERANCE_M,
                    'Worker exceeds horizontal camera bound')
            require(max(v.z for v in vertices) <= root['display_height_m']+TOLERANCE_M,
                    'Worker exceeds camera height bound')
            require(min(v.z for v in vertices) >= -TOLERANCE_M,
                    'Worker geometry penetrates flat datum')
            rig = japan._WORKERS[root.as_pointer()]
            inverse = root.matrix_world.inverted()
            hand = inverse @ rig['limbs'][-1]['hand'].matrix_world.translation
            target = inverse @ rig['model'].matrix_world @ Vector((.34, -.27, .99))
            error = (hand-target).length
            require(error < TOLERANCE_M, 'Actual worker hand misses the shovel grip')
            measurements.append({'time': time,
                                 'min': [min(v[i] for v in vertices) for i in range(3)],
                                 'max': [max(v[i] for v in vertices) for i in range(3)],
                                 'hand_error_m': error})
        if previous is not None:
            require(measurements == previous,
                    'Physical radius changes cosmetic worker geometry or contact')
        previous = measurements
        cases.append({'radius': radius, 'poses': measurements, 'root_unchanged': True})
    return cases


def check_junction_sidewalks():
    edges = [{'points': [{'x': float(x), 'y': lane} for x in range(0, 121, 2)],
              'half_width': 1.8} for lane in MAIN_LANES]
    counts = japan.japan_environment(edges)
    require(counts['junction_centers_x_m'] == [15., 36.] and
            counts['cross_street_half_widths_m'] == [3.4, 1.35] and
            counts['cross_street_one_way_directions'] == ['two-way', '+Y'],
            'Japan junction metadata differs from the independent geometry contract')
    require(counts['buildings'] > 0 and counts['storefronts'] > 0,
            'Japanese streetscape unexpectedly empty')
    bpy.context.view_layer.update()
    checked = 0
    for obj in bpy.data.objects:
        if not obj.name.startswith(('Curb', 'Sidewalk')):
            continue
        for face in obj.data.polygons:
            vertices = [obj.matrix_world @ obj.data.vertices[i].co for i in face.vertices]
            if max(v.z for v in vertices) < .05:
                continue
            x0, x1 = min(v.x for v in vertices), max(v.x for v in vertices)
            y0, y1 = min(v.y for v in vertices), max(v.y for v in vertices)
            for center, width in JUNCTIONS:
                for lane in MAIN_LANES:
                    overlaps = (max(x0, center-width-.4)+1e-5 < min(x1, center+width+.4)
                                and max(y0, lane-1.8)+1e-5 < min(y1, lane+1.8))
                    require(not overlaps,
                            f'Raised sidewalk crosses a live junction: {obj.name}, x={center}, lane={lane}')
            checked += 1
    require(checked > 0, 'No raised sidewalk faces were checked')
    return counts, checked


def check_cross_streets():
    results = []
    for (center, width), one_way in zip(JUNCTIONS, JUNCTION_ONE_WAY):
        root = japan.cross_street('Original cross street', center, half_width=width, one_way=one_way)
        bpy.context.view_layer.update()
        require(root['one_way_direction'] == ('+Y' if one_way else 'two-way'),
                'Cross street has incorrect traffic-direction metadata')
        center_paint = [obj for obj in root.children if 'center paint' in obj.name]
        arrows = [obj for obj in root.children if obj.get('display_arrow_direction')]
        require(bool(center_paint) != one_way, 'One-way street must omit the amber center divider')
        arrow_checks = []
        if one_way:
            require(bool(arrows), 'One-way street has no direction arrows')
            for arrow in arrows:
                vertices = [arrow.matrix_world @ vertex.co for vertex in arrow.data.vertices]
                min_y, max_y = min(v.y for v in vertices), max(v.y for v in vertices)
                tip = [v for v in vertices if abs(v.y-max_y) < TOLERANCE_M]
                tail = [v for v in vertices if abs(v.y-min_y) < TOLERANCE_M]
                require(len(tip) == 1 and len(tail) == 2 and max_y-min_y > 1.0 and
                        abs(tip[0].x-center) < TOLERANCE_M,
            'Actual one-way arrow geometry does not point toward +Y')
                require(all(abs(v.x-center) <= width+TOLERANCE_M for v in vertices),
                        'Direction arrow extends outside its street')
                require(all(max(min_y, lane-1.8)+TOLERANCE_M >= min(max_y, lane+1.8)
                            for lane in (-8., 0.)),
                        'Direction arrow crosses a live main-lane junction')
                arrow_checks.append({'tip_m': list(tip[0]), 'tail_y_m': min_y})
        zebras = [obj for obj in root.children if 'main-road zebra' in obj.name]
        require(bool(zebras), 'Cross street has no main-road zebra geometry')
        zebra_bounds = []
        checked_faces = 0
        for zebra in zebras:
            vertices = [zebra.matrix_world @ vertex.co for vertex in zebra.data.vertices]
            for face in zebra.data.polygons:
                face_points = [vertices[index] for index in face.vertices]
                require(min(v.x for v in face_points) >= center-width-TOLERANCE_M and
                        max(v.x for v in face_points) <= center+width+TOLERANCE_M,
                        'A main-road zebra face extends outside the validated junction envelope')
                checked_faces += 1
            zebra_bounds.append({'min_x_m': min(v.x for v in vertices),
                                 'max_x_m': max(v.x for v in vertices),
                                 'min_y_m': min(v.y for v in vertices),
                                 'max_y_m': max(v.y for v in vertices)})
        results.append({'center_x_m': center, 'half_width_m': width,
                        'one_way_direction': root['one_way_direction'],
                        'center_divider_count': len(center_paint), 'arrows': arrow_checks,
                        'main_road_zebra_faces_checked': checked_faces,
                        'main_road_zebra_object_bounds': zebra_bounds})
    # Measure the complete crossing hatchback at its +Y orientation, including
    # mirrors and wheels. This checks display fit, not the native capsule proxy.
    materials = [japan.material('Crossing-car audit '+name, color) for name, color in
                 [('paint', (.1,.3,.6)), ('glass', (.1,.2,.3)),
                  ('tire', (.02,.02,.02)), ('headlight', (.8,.8,.7))]]
    center, half_width = JUNCTIONS[1]
    vehicle = car('Actual preset crossing hatchback', 1.0, *materials, variant='hatchback')
    vehicle.location, vehicle.rotation_euler.z = (center, 10, 0), math.pi/2
    bpy.context.view_layer.update()
    vertices = [vehicle.matrix_world @ point for point in evaluated_points(vehicle)]
    min_x, max_x = min(v.x for v in vertices), max(v.x for v in vertices)
    require(min_x >= center-half_width-TOLERANCE_M and max_x <= center+half_width+TOLERANCE_M,
            'Full-size crossing hatchback does not fit the illustrated one-way street')
    results[1]['crossing_hatchback_full_width_m'] = max_x-min_x
    results[1]['minimum_display_lateral_margin_m'] = min(min_x-center+half_width, center+half_width-max_x)
    return results


def check_stationary_paint_nonoverlap(path, cross_streets):
    """Set-containment proof using a separately accepted native physical report.

    This does not rerun native samples. The report's independently checked
    200 Hz stationary-body exclusion from each street implies exclusion from
    the measured zebra faces contained in that street. The report is SHA-bound.
    """
    require(0 < path.stat().st_size <= 16*1024*1024, 'Physical report exceeds size limit')
    raw = path.read_bytes()
    report = json.loads(raw)
    repository = SCRIPT_DIRECTORY.parent
    fixtures = {'scenario': repository/'scenarios/native-japan-city-demo.json',
                'scene': repository/'scenes/japan-city-construction.json'}
    require(report.get('passed') is True and report.get('complete') is True,
            'Stationary paint proof requires an accepted complete native report')
    require(report.get('fixture_sha256') ==
            {key: hashlib.sha256(file.read_bytes()).hexdigest() for key, file in fixtures.items()},
            'Physical report does not describe the current recorded fixtures')
    checker = repository/'scripts/check-japan-city-demo.py'
    require(report.get('checker_dependency_sha256', {}).get('scripts/check-japan-city-demo.py') ==
            hashlib.sha256(checker.read_bytes()).hexdigest(),
            'Physical report stationary-body checker does not match its frozen source')
    cases = report.get('cases', [])
    require(len(cases) == 3 and {case.get('seed') for case in cases} == {1, 7, 42},
            'Stationary paint proof requires all three accepted native seeds')
    checked = []
    for case in cases:
        require(case['summary'].get('passed') is True, 'Native case acceptance failed')
        evidence = case['junctions_and_construction']
        junctions = evidence['intersections']
        require(len(junctions) == len(cross_streets), 'Physical/display junction counts differ')
        for paint, junction in zip(cross_streets, junctions):
            center, width = paint['center_x_m'], paint['half_width_m']
            require(junction['street_center_x_m'] == center and junction['street_half_width_m'] == width and
                    abs(junction['street_entry_x_m']-(center-width)) < TOLERANCE_M and
                    abs(junction['street_exit_x_m']-(center+width)) < TOLERANCE_M,
                    'Paint and stationary physical-body proof use different street envelopes')
            require(type(junction['stopped_body_samples_inside']) is int and
                    junction['stopped_body_samples_inside'] == 0 and
                    '200 Hz' in junction['stopped_sample_contract'],
                    'A stationary actual native body occupies the painted street envelope')
            require(min(band['min_x_m'] for band in paint['main_road_zebra_object_bounds']) >
                    junction['mapped_stopline_x_m'], 'Zebra paint is not beyond its actual mapped stop line')
        queued = evidence['queued_vehicle_si_footprints']
        require(queued['ids'] == [0, 8, 9] and queued['stationary_speed_threshold_m_s'] == .1 and
                queued['stationary_frame_junction_checks'] > 0 and
                queued['minimum_stationary_npc_junction_reserve_m'] > 0,
                'Stationary queued vehicle envelopes are not proven outside the streets')
        checked.append({'seed': case['seed'], 'zero_stationary_ego_samples_in_each_street': True,
                        'queued_stationary_frame_checks': queued['stationary_frame_junction_checks'],
                        'minimum_queued_junction_reserve_m': queued['minimum_stationary_npc_junction_reserve_m'],
                        'recorded_raw_sha256': case['raw_sha256']})
    return {'proof_kind': 'Measured zebra-face containment implies non-overlap with stationary bodies excluded by the referenced physical report',
            'scope': 'Actual 200 Hz sampled ego bodies and calibrated straight queued NPC SI envelopes; not all arbitrary decorative meshes or unsampled-time verification',
            'physical_report_sha256': hashlib.sha256(raw).hexdigest(),
            'physical_fixture_sha256': report['fixture_sha256'],
            'physical_checker_sha256': report['checker_dependency_sha256']['scripts/check-japan-city-demo.py'],
            'raw_native_samples_rechecked': False, 'logical_nonoverlap_verified': True, 'cases': checked}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/japan-display-audit.json'))
    parser.add_argument('--physical-report', type=Path,
                        help='Optional accepted three-seed native report for the stationary-paint containment proof')
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    hashes = source_hashes()
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    proof = {'schema': 'rustdriving-japan-display-audit-v1', 'display_only': True,
             'scope': 'Authored display geometry audit; not native driving acceptance or physical mesh collision.',
             'blender_version': bpy.app.version_string, 'units': 'metres',
             'tolerance_m': TOLERANCE_M, 'worker': check_workers()}
    counts, checked = check_junction_sidewalks()
    proof['city_counts'], proof['checked_raised_sidewalk_faces'] = counts, checked
    proof['cross_streets'] = check_cross_streets()
    if args.physical_report:
        proof['stationary_paint_nonoverlap'] = check_stationary_paint_nonoverlap(args.physical_report, proof['cross_streets'])
    zone = japan.work_zone('Original work zone')
    bpy.context.view_layer.update()
    require(min(v.z for v in evaluated_points(zone)) >= -TOLERANCE_M,
            'Work-zone dressing penetrates flat datum')
    materials = {'off': japan.material('Inactive display lens', (.02, .02, .02))}
    _, lenses = japan.horizontal_signal('Japan head', (12, 4), 0, materials)
    require(set(lenses) == {'Red', 'Yellow', 'Green'}, 'Incomplete signal lens mapping')
    require(lenses['Green'].location.y > lenses['Yellow'].location.y > lenses['Red'].location.y,
            'Incorrect Japanese horizontal signal lens ordering')
    # This exercises lettering capability at an authored location. The driving
    # renderer never adds permanent STOP wording to signal-only mapped lines.
    text = japan.stop_text('Mapped stop-line font capability', (0, 50, .07), 0)
    require(text.data.body in ('止まれ', 'STOP'), 'Unexpected road lettering')
    proof['font'] = {'body': text.data.body,
                     'license': text.get('font_license', 'Blender built-in Bfont'),
                     'sha256': text.get('font_sha256')}
    require(hashes == source_hashes(), 'Display source changed during the audit')
    proof['source_sha256'], proof['passed'] = hashes, True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(proof, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'passed': True, 'worker_pose_cases': 10,
                      'checked_raised_sidewalk_faces': checked, 'source_sha256': hashes}))


if __name__ == '__main__':
    main()
