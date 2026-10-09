"""Blender worker: render recorded world states, never advance a simulator."""
import json
import math
from pathlib import Path
import sys

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_assets import STYLE, barrel, car, cube, environment, material, stop_sign, traffic_signal, yield_sign


def line(name, points, mat, width=0.04):
    curve = bpy.data.curves.new(name, 'CURVE')
    curve.dimensions = '3D'
    curve.bevel_depth = width
    curve.bevel_resolution = 2
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(mat)
    set_line(obj, points)
    return obj


def set_line(obj, points):
    obj.data.splines.clear()
    if len(points) < 2:
        return
    spline = obj.data.splines.new('POLY')
    spline.points.add(len(points)-1)
    for vertex, point in zip(spline.points, points):
        vertex.co = (*point, 1)


def road(points, half_width, asphalt, marking, center_marking=True):
    vertices, borders = [], [[], []]
    for i, point in enumerate(points):
        before, after = points[max(0, i-1)], points[min(len(points)-1, i+1)]
        dx, dy = after['x']-before['x'], after['y']-before['y']
        length = math.hypot(dx, dy)
        nx, ny = -dy/length, dx/length
        for side, sign in enumerate([-1, 1]):
            x, y = point['x']+sign*nx*half_width, point['y']+sign*ny*half_width
            vertices.append((x, y, 0.02))
            borders[side].append((x, y, 0.055))
    mesh = bpy.data.meshes.new('Recorded road corridor')
    mesh.from_pydata(vertices, [], [(2*i, 2*i+2, 2*i+3, 2*i+1) for i in range(len(points)-1)])
    obj = bpy.data.objects.new('Road', mesh)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(asphalt)
    for border in borders:
        line('Corridor edge', border, marking, 0.045)
    for i in (range(0, len(points)-1, 3) if center_marking else []):
        a, b = points[i:i+2]
        line('Decorative center marking', [(a['x'], a['y'], 0.06), (b['x'], b['y'], 0.06)], marking, 0.04)
    return obj


def native_cuboid_audit(models):
    """Read actual world mesh vertices, rather than requested fixture values."""
    bpy.context.view_layer.update()
    return [{'id': identifier,
             'center_m': list(obj.matrix_world.translation),
             'yaw_rad': math.atan2(obj.matrix_world[1][0], obj.matrix_world[0][0]),
             'world_corners_m': [list(obj.matrix_world @ vertex.co) for vertex in obj.data.vertices]}
            for identifier, obj in models.items()]


def main():
    request = json.loads(Path(sys.argv[sys.argv.index('--')+1]).read_text())
    run = json.loads(Path(request['run']).read_text())
    actor_models = {int(key): value for key, value in request.get('actor_models', {}).items()}
    urban = request.get('environment') == 'urban'
    if any(model in ('elder', 'child', 'parent_stroller') for model in actor_models.values()):
        from blender_family_assets import elder, child as family_child, parent_stroller, animate_family
    if urban or any(model in ('truck', 'dog') for model in actor_models.values()):
        from blender_city_assets import STYLE as CITY_STYLE, truck, dog, animate_dog, city_environment
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.render.engine = 'CYCLES'
    scene.cycles.device = 'CPU'
    scene.cycles.samples = request['samples']
    scene.cycles.use_denoising = False
    scene.cycles.max_bounces = 3
    scene.render.resolution_x, scene.render.resolution_y = 960, 540
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGB'
    scene.render.use_persistent_data = True
    scene.world.use_nodes = True
    scene.world.node_tree.nodes['Background'].inputs[0].default_value = (.65, .76, .95, 1)
    scene.world.node_tree.nodes['Background'].inputs[1].default_value = .45
    asphalt = material('Asphalt', (.065, .08, .11))
    marking = material('Ivory marking', (.70, .77, .79))
    grass = material('Ground', (.085, .14, .115))
    blue = material('Ego blue', (.02, .27, .88), .45, .24)
    orange = material('Actor amber', (.95, .30, .035), .35, .27)
    glass = material('Dark windows', (.015, .045, .075), .65, .14)
    tire = material('Rubber', (.01, .012, .015))
    headlight = material('Headlight', (.90, .96, 1))
    teal = material('Planned path', (.02, .85, .55))
    purple = material('Predicted motion', (.65, .22, .95))
    red = material('Map closure overlay', (.88, .03, .04))
    cube('Ground plane', (100, 0, -.12), (1000, 1000, .2), grass)
    navigation = run['scenario'].get('navigation')
    edges = navigation['network']['edges'] if navigation else [run['route']]
    opposing_display_lanes = []
    service_display_lanes = []
    if request.get('camera') == 'street' and actor_models and not navigation:
        # Infer a display strip from actual opposing vehicle motion. It is not
        # an additional operational route, lane rule or physical road boundary.
        route = run['route']
        a, b = route['points'][0], route['points'][-1]
        dx, dy = b['x']-a['x'], b['y']-a['y']
        length = math.hypot(dx, dy)
        ux, uy = dx/length, dy/length
        for identifier, model in actor_models.items():
            if model not in ('hatchback', 'sedan', 'van', 'pickup', 'truck'):
                continue
            positions = [actor['position'] for frame in run['frames'] for actor in frame['objects'] if actor['id'] == identifier]
            if len(positions) < 2:
                continue
            first, last = positions[0], positions[-1]
            lateral = -(first['x']-a['x'])*uy+(first['y']-a['y'])*ux
            travel = (last['x']-first['x'])*ux+(last['y']-first['y'])*uy
            if abs(travel) > 1 and abs(lateral) > route['half_width']:
                lane_group = opposing_display_lanes if travel < 0 else service_display_lanes
                if not any(abs(lateral-lane['offset_m']) < .1 for lane in lane_group):
                    lane_group.append({'actor_id': identifier, 'offset_m': lateral, 'display_only': True})
                else:
                    continue
                edges = list(edges)+[{'points': [{'x': p['x']-uy*lateral, 'y': p['y']+ux*lateral}
                                               for p in route['points']], 'half_width': route['half_width']}]
    intersection_specs = run['scenario'].get('yield_intersections', [])
    if intersection_specs:
        # These perpendicular streets are display geometry inferred from the
        # known conflict rectangles, not extra simulator roads or obstacles.
        edges = list(edges)
        for spec in intersection_specs:
            bounds = spec['conflict_bounds']
            center_x = (bounds['min']['x']+bounds['max']['x'])/2
            center_y = (bounds['min']['y']+bounds['max']['y'])/2
            half_width = (bounds['max']['x']-bounds['min']['x'])/2
            edges.append({'points':[{'x':center_x,'y':center_y+i*2} for i in range(-18,19)],
                          'half_width':half_width})
    if urban:
        edges = [dict(edge, half_width=1.8) for edge in edges]
    closures = {}
    roads = []
    for edge in edges:
        points = edge['points']
        if request.get('camera') == 'street':
            first, second, before, last = points[0], points[1], points[-2], points[-1]
            dx, dy = second['x']-first['x'], second['y']-first['y']
            length = math.hypot(dx, dy)
            ex, ey = last['x']-before['x'], last['y']-before['y']
            extent = math.hypot(ex, ey)
            points = [{'x': first['x']-20*dx/length, 'y': first['y']-20*dy/length}]+points+[
                {'x': last['x']+20*ex/extent, 'y': last['y']+20*ey/extent}]
        surface = road(points, edge['half_width'], asphalt, marking,
                       center_marking=not opposing_display_lanes)
        bpy.context.view_layer.objects.active = surface
        solid = surface.modifiers.new('Road thickness', 'SOLIDIFY')
        solid.thickness = .04
        solid.offset = -1
        bpy.ops.object.modifier_apply(modifier=solid.name)
        roads.append(surface)
        if 'id' in edge:
            closures[edge['id']] = line('Map closure', [(p['x'], p['y'], .09) for p in edge['points']], red, .16)
            closures[edge['id']].hide_render = True
    # Overlapping authored corridors become one surface, avoiding coplanar
    # flicker at the fork and merge. This changes only decorative geometry.
    for surface in roads[1:]:
        bpy.context.view_layer.objects.active = roads[0]
        union = roads[0].modifiers.new('Joined junction', 'BOOLEAN')
        union.operation, union.solver, union.object = 'UNION', 'EXACT', surface
        bpy.ops.object.modifier_apply(modifier=union.name)
        bpy.data.objects.remove(surface, do_unlink=True)
    exclusion_paths = []
    if request.get('camera') == 'street':
        # Keep cosmetic trees/buildings out of the actual recorded road-user
        # sweeps. This does not add obstacles or sensing to the simulator.
        for identifier in actor_models:
            positions = [actor['position'] for frame in run['frames'] for actor in frame['objects'] if actor['id'] == identifier]
            if len(positions) >= 2:
                exclusion_paths.append({'points': positions, 'half_width': 1.2})
    scenery = (city_environment if urban else environment)(edges, exclusion_paths=exclusion_paths)
    if urban:
        # Keep the display facades behind the recorded traffic in this fixed
        # street view. Foreground roofs otherwise hide ego and the stroller.
        # These buildings are cosmetic and never enter physical sensing.
        a, b = run['route']['points'][0], run['route']['points'][-1]
        dx, dy = b['x']-a['x'], b['y']-a['y']
        length = math.hypot(dx, dy)
        center = opposing_display_lanes[0]['offset_m']/2 if opposing_display_lanes else 0
        hidden = 0
        for obj in list(bpy.data.objects):
            if obj.get('scenery_kind') != 'building':
                continue
            lateral = (-(obj.location.x-a['x'])*dy+(obj.location.y-a['y'])*dx)/length
            if lateral > center:
                obj.hide_render = True
                for scenery_part in obj.children_recursive:
                    scenery_part.hide_render = True
                hidden += 1
        scenery['foreground_buildings_hidden_for_camera'] = hidden
        pavement = material('Urban public plaza paving', (.26, .30, .32))
        cycling = material('Urban cycle path', (.18, .25, .27))
        for low, high in [(-24, -12), (19, 25)]:
            cube('Display pedestrian plaza', (40, (low+high)/2, .025), (120, high-low, .05), pavement)
        for low, high in [(-6.15, -1.85), (1.85, 6.2)]:
            cube('Display crossing footpath', (40, (low+high)/2, .025), (120, high-low, .05), pavement)
        for lateral in [8, 11]:
            cube('Display recorded bicycle path', (40, lateral, .025), (120, 2.2, .05), cycling)
        # Original zebra paint at actual recorded crossing positions; signal
        # timing and actor yielding are checked separately in the driving log.
        for identifier in [1, 4, 6]:
            first = next((a for frame in run['frames'] for a in frame['objects'] if a['id'] == identifier), None)
            if first:
                for offset in [-1.2, -.6, 0, .6, 1.2]:
                    cube('Display crossing stripe '+str(identifier), (first['position']['x']+offset, 0, .067), (.3, 3.5, .014), marking)
        for lane in [0]+[spec['offset_m'] for spec in opposing_display_lanes+service_display_lanes]:
            direction = -1 if lane < 0 else 1
            for distance in range(5, 81, 18):
                arrow = line('Display lane direction arrow', [(distance-direction*.8, lane, .08), (distance+direction*.6, lane, .08)], marking, .065)
                line('Display lane direction arrowhead', [(distance, lane-.35, .08), (distance+direction*.6, lane, .08), (distance, lane+.35, .08)], marking, .065)
    ground_mode = request.get('ground_mode', False)
    # Corridor coloring sits above the recorded flat support. Its edges and
    # inferred opposing lane remain display-only; the full physical tile is
    # retained and independently audited without pretending it is all asphalt.
    native_models = {}
    ground_models = {}
    if request.get('native_scene'):
        physical = material('Native physical cuboid orange', (.98, .25, .025), roughness=.55)
        for box in request['native_scene']['static_cuboids']:
            obj = cube('Native physical cuboid '+box['id'], box['center_m'],
                       [2*half for half in box['half_extents_m']], physical)
            obj.rotation_euler.z = box['yaw_rad']
            obj['native_scene_id'] = box['id']
            obj['native_scene_physical_geometry'] = True
            native_models[box['id']] = obj
        if ground_mode:
            for box in request['native_scene']['ground_cuboids']:
                obj = cube('Native physical road '+box['id'], box['center_m'],
                           [2*half for half in box['half_extents_m']], grass)
                obj.rotation_euler.z = box['yaw_rad']
                obj['native_scene_id'] = box['id']
                obj['native_scene_physical_ground_geometry'] = True
                ground_models[box['id']] = obj
    signal_specs = run['scenario'].get('traffic_signals', [])
    signal_models = {}
    stop_specs = run['scenario'].get('stop_signs', [])
    if signal_specs or stop_specs or intersection_specs:
        lamp_materials = {'off':material('Inactive signal lens', (.014,.016,.018))}
        for color,rgb in [('Red',(.95,.015,.012)),('Yellow',(1,.53,.015)),('Green',(.018,.80,.10))]:
            mat = material('Active signal '+color, rgb)
            shader = mat.node_tree.nodes.get('Principled BSDF')
            emission = shader.inputs.get('Emission Color') or shader.inputs.get('Emission')
            if emission: emission.default_value = (*rgb,1)
            if shader.inputs.get('Emission Strength'): shader.inputs['Emission Strength'].default_value = 1.5
            lamp_materials[color] = mat
        def route_at(distance,lateral=0):
            points,lengths=run['route']['points'],run['route']['lengths']
            i=next((i for i in range(1,len(lengths)) if lengths[i]>=distance),len(lengths)-1)
            a,b=points[i-1],points[i]
            t=(distance-lengths[i-1])/(lengths[i]-lengths[i-1])
            yaw=math.atan2(b['y']-a['y'],b['x']-a['x'])
            return (a['x']+(b['x']-a['x'])*t-math.sin(yaw)*lateral,
                    a['y']+(b['y']-a['y'])*t+math.cos(yaw)*lateral),yaw
        for spec in signal_specs:
            mapped=spec['stop_line'];width=run['route']['half_width']
            position,yaw=route_at(mapped['route_s_m'],width+1)
            parent,lenses=traffic_signal('Mapped signal '+mapped['id'],position,yaw,lamp_materials)
            parent['stop_line_id']=mapped['id']
            a,_=route_at(mapped['route_s_m'],-width);b,_=route_at(mapped['route_s_m'],width)
            line('Mapped stop line '+mapped['id'],[(*a,.075),(*b,.075)],marking,.12)
            signal_models[mapped['id']]=lenses
        for mapped in stop_specs:
            width=run['route']['half_width']
            position,yaw=route_at(mapped['route_s_m'],width+1)
            parent=stop_sign('Mapped stop sign '+mapped['id'],position,yaw)
            parent['stop_line_id']=mapped['id']
            a,_=route_at(mapped['route_s_m'],-width);b,_=route_at(mapped['route_s_m'],width)
            line('Mapped stop sign line '+mapped['id'],[(*a,.075),(*b,.075)],marking,.12)
        for spec in intersection_specs:
            mapped = spec['stop_line']; width = run['route']['half_width']
            position,yaw = route_at(mapped['route_s_m'],width+1)
            parent = yield_sign('Mapped yield sign '+mapped['id'],position,yaw)
            parent['intersection_id'] = mapped['id']
            a,_ = route_at(mapped['route_s_m'],-width); b,_ = route_at(mapped['route_s_m'],width)
            line('Mapped yield line '+mapped['id'],[(*a,.075),(*b,.075)],marking,.10)
    bpy.ops.object.light_add(type='SUN', location=(0, 0, 20))
    sun = bpy.context.object
    sun.rotation_euler = (.5, -.4, -.35)
    sun.data.energy, sun.data.angle = 2.5, .12
    bpy.ops.object.camera_add()
    camera = bpy.context.object
    camera.data.lens = 38
    camera.data.clip_end = 1000
    scene.camera = camera
    ego = car('Recorded ego', run['vehicle']['radius'], blue, glass, tire, headlight, ego=True)
    # This local view uses one orthographic scale for all road users: physical
    # collision radii and camera depth cannot change their displayed size.
    if request.get('camera') == 'street':
        camera.data.type = 'ORTHO'
        camera.data.ortho_scale = 64 if urban else 52
    body_envelope = None
    if ground_mode and request.get('body_calibration'):
        calibration = request['body_calibration']
        body_envelope = cube('Research physical body envelope', (0, 0, 0),
                             (calibration['length_m'], calibration['width_m'], calibration['height_m']), teal)
        body_envelope['research_body_envelope'] = True
        wire = body_envelope.modifiers.new('Research envelope wire', 'WIREFRAME')
        wire.thickness = .018
        wire.use_replace = True
        # Audit original eight vertices; the render-only wire modifier does
        # not change the calibration corners or pretend to be a cosmetic car.
    dynamic_ids = {s['id'] for f in run['frames'] for s in f.get('traffic', [])}
    if intersection_specs:
        first_positions = {}
        for frame in run['frames']:
            for actor in frame['objects']:
                p = actor['position']
                first = first_positions.setdefault(actor['id'],p)
                if math.hypot(p['x']-first['x'],p['y']-first['y']) > 1e-6:
                    dynamic_ids.add(actor['id'])
    if actor_models:
        first_positions = {}
        for recorded in run['frames']:
            for actor in recorded['objects']:
                first = first_positions.setdefault(actor['id'], actor['position'])
                if math.hypot(actor['position']['x']-first['x'], actor['position']['y']-first['y']) > 1e-6:
                    dynamic_ids.add(actor['id'])
        from blender_vru_assets import pedestrian, cyclist, animate as animate_vru
    actors = {}
    vehicle_models = []
    actor_paints = [orange, material('Van ivory', (.72,.75,.68), .2, .3),
                    material('Pickup green', (.06,.36,.23), .35, .27)]
    for frame in run['frames']:
        for actor in frame['objects']:
            if actor['id'] in actors:
                continue
            selected = actor_models.get(actor['id'])
            if selected == 'dog':
                actors[actor['id']] = dog('Recorded dog '+str(actor['id']), actor['radius'])
                vehicle_models.append({'id': actor['id'], 'model': selected, 'display_only': True})
            elif selected in ('pedestrian', 'cyclist', 'elder', 'child', 'parent_stroller'):
                palette = [(0.04, .30, .56), (.76, .22, .06), (.32, .14, .52)]
                jacket = material('Recorded road-user jacket '+str(actor['id']), palette[len(vehicle_models) % len(palette)])
                maker = {'pedestrian': pedestrian, 'cyclist': cyclist}
                if selected in ('elder', 'child', 'parent_stroller'):
                    maker.update(elder=elder, child=family_child, parent_stroller=parent_stroller)
                actors[actor['id']] = maker[selected](
                    'Recorded '+selected+' '+str(actor['id']), actor['radius'], {'jacket': jacket})
                vehicle_models.append({'id': actor['id'], 'model': selected, 'display_only': True})
            elif actor['id'] in dynamic_ids or selected:
                number = sum(row['model'] in ('hatchback', 'sedan', 'van', 'pickup', 'truck') for row in vehicle_models)
                variant = selected or request.get('traffic_models', ['hatchback'])[number % len(request.get('traffic_models', ['hatchback']))]
                paint = actor_paints[number % len(actor_paints)] if actor_models or len(request.get('traffic_models', ['hatchback'])) > 1 else orange
                label = 'Recorded traffic actor '+str(actor['id'])
                actors[actor['id']] = truck(label, actor['radius'], paint, glass, tire, headlight) if variant == 'truck' else car(label, actor['radius'], paint, glass, tire, headlight, variant=variant)
                vehicle_models.append({'id': actor['id'], 'model': variant, 'color': list(paint.diffuse_color[:3])})
            else:
                actors[actor['id']] = barrel(actor['radius'], orange, marking)
    planned = line('Actual planned trajectory', [], teal, .075)
    predictions = line('Actual track forecasts', [], purple, .04)
    leash_material = material('Dog leash', (.055, .065, .085))
    dog_pairs = {int(key): value for key, value in request.get('dog_pairs', {}).items()}
    leashes = {owner: line('Recorded companion leash '+str(owner), [], leash_material, .012) for owner in dog_pairs}
    audit = []
    output = Path(request['frames_directory'])
    distances = [0.0]
    actor_distances = {}
    last_actor_positions = {}
    for before, after in zip(run['frames'], run['frames'][1:]):
        a, b = before['truth']['pose']['position'], after['truth']['pose']['position']
        distances.append(distances[-1]+math.hypot(b['x']-a['x'], b['y']-a['y']))
    for number, index in enumerate(request['indices']):
        frame = run['frames'][index]
        pose = frame['truth']['pose']
        x, y = pose['position']['x'], pose['position']['y']
        ego.location, ego.rotation_euler = (x, y, .02), (0, 0, pose['yaw'])
        if ground_mode:
            ego.location.z = 0
        if body_envelope is not None:
            calibration = request['body_calibration']
            ox, oy = calibration['center_offset_body_m']
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
            body_envelope.location = (x+c*ox-s*oy, y+s*ox+c*oy,
                                      calibration['bottom_m']+calibration['height_m']/2)
            body_envelope.rotation_euler.z = pose['yaw']
        camera.location = (x-11, y-16, 16)
        target = (x+4, y, 0)
        if request.get('camera') == 'street':
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
            target = (x+12*c, y+12*s, 0)
            if urban:
                lateral = opposing_display_lanes[0]['offset_m']/2 if opposing_display_lanes else 0
                target = (target[0]-lateral*s, target[1]+lateral*c, 0)
                camera.location = (target[0]-18*c-32*s, target[1]-18*s+32*c, 30)
            else:
                camera.location = (target[0]-18*c+32*s, target[1]-18*s-32*c, 26)
        elif request.get('camera') == 'traffic':
            bodies = [{'position': pose['position'], 'radius': run['vehicle']['radius']}]+[a for a in frame['objects'] if a['id'] in dynamic_ids]
            positions = [a['position'] for a in bodies]
            min_x,max_x = min(p['x'] for p in positions),max(p['x'] for p in positions)
            min_y,max_y = min(p['y'] for p in positions),max(p['y'] for p in positions)
            height = max(16, math.hypot(max_x-min_x,max_y-min_y)*.45+16)
            center_x,center_y = (min_x+max_x)/2,(min_y+max_y)/2
            target = (center_x,center_y,0)
            # Fit the complete display bounds, not just the center positions:
            # a long queue can otherwise clip ego at the start in perspective.
            corners = [Vector((a['position']['x']+sx*(max(a['radius'], actors.get(a.get('id'), ego).get('display_radius_m', a['radius']))+.5),
                               a['position']['y']+sy*(max(a['radius'], actors.get(a.get('id'), ego).get('display_radius_m', a['radius']))+.5),z))
                       for a in bodies for sx in [-1,1] for sy in [-1,1]
                       for z in [0,max(2*a['radius'], actors.get(a.get('id'), ego).get('display_height_m', 0))]]
            for attempt in range(32):
                camera.location = (center_x-.7*height,center_y-height,height)
                camera.rotation_euler = (Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
                bpy.context.view_layer.update()
                projected = [world_to_camera_view(scene,camera,p) for p in corners]
                if all(.05 <= p.x <= .95 and .05 <= p.y <= .95 and p.z > 0 for p in projected):
                    break
                height *= 1.1
            else:
                raise ValueError('Traffic camera could not fit recorded vehicle bounds')
        camera.rotation_euler = (Vector(target)-camera.location).to_track_quat('-Z', 'Y').to_euler()
        for wheel in ego.children:
            if wheel.get('rolling_wheel'):
                wheel.rotation_euler.y = distances[index]/ego['wheel_radius']
        for id, obj in actors.items():
            obj.hide_render = not any(a['id'] == id for a in frame['objects'])
            obj.hide_viewport = obj.hide_render
            for child in obj.children_recursive:
                child.hide_render = obj.hide_render
                child.hide_viewport = obj.hide_render
        for actor in frame['objects']:
            obj = actors[actor['id']]
            obj.location.x, obj.location.y = actor['position']['x'], actor['position']['y']
            if actor['id'] in dynamic_ids:
                p = actor['position']
                old = last_actor_positions.get(actor['id'], p)
                actor_distances[actor['id']] = actor_distances.get(actor['id'], 0)+math.hypot(p['x']-old['x'],p['y']-old['y'])
                last_actor_positions[actor['id']] = p
                for wheel in obj.children:
                    if wheel.get('rolling_wheel'):
                        wheel.rotation_euler.y = actor_distances[actor['id']]/obj['wheel_radius']
            if actor['id'] in dynamic_ids:
                previous = next((a for a in run['frames'][index-1]['objects'] if a['id'] == actor['id']), None) if index > 0 else None
                if previous:
                    dx, dy = actor['position']['x']-previous['position']['x'], actor['position']['y']-previous['position']['y']
                    if math.hypot(dx, dy) > 1e-6:
                        obj.rotation_euler.z = math.atan2(dy, dx)
                elif intersection_specs or actor_models:
                    # First display pose uses the next actual recorded movement;
                    # it never uses a scripted actor velocity or advances time.
                    for later in run['frames'][index+1:]:
                        following = next((a for a in later['objects'] if a['id'] == actor['id']),None)
                        if following:
                            dx,dy = following['position']['x']-actor['position']['x'],following['position']['y']-actor['position']['y']
                            if math.hypot(dx,dy) > 1e-6:
                                obj.rotation_euler.z = math.atan2(dy,dx)
                                break
        for actor in frame['objects']:
            obj = actors[actor['id']]
            if obj.get('vru_kind') or obj.get('dog_kind'):
                prior = next((a for a in run['frames'][max(0, index-1)]['objects'] if a['id'] == actor['id']), None)
                dt = frame['time']-run['frames'][max(0, index-1)]['time']
                speed = math.hypot(actor['position']['x']-prior['position']['x'], actor['position']['y']-prior['position']['y'])/dt if prior and dt > 0 else 0
                animator = animate_family if obj.get('avatar_kind') else animate_dog if obj.get('dog_kind') else animate_vru
                animator(obj, frame['time'], speed)
        bpy.context.view_layer.update()
        active = {actor['id'] for actor in frame['objects']}
        for owner, companion in dog_pairs.items():
            leashes[owner].hide_render = owner not in active or companion not in active
            if not leashes[owner].hide_render:
                walker, pet = actors[owner], actors[companion]
                hands = [child for child in walker.children_recursive if ' hand' in child.name]
                collar = pet.matrix_world @ Vector(pet['leash_anchor_local_m'])
                hand = min((child.matrix_world.translation for child in hands), key=lambda p: (p-collar).length) if hands else walker.matrix_world @ Vector((.1, -.2, .9))
                midpoint = (hand+collar)/2
                midpoint.z -= .14
                set_line(leashes[owner], [hand, midpoint, collar])
        set_line(planned, [(p['position']['x'], p['position']['y'], .11) for p in frame['trajectory']['points']])
        forecast = frame['predictions'][0]['positions'] if frame['predictions'] else []
        set_line(predictions, [(p['x'], p['y'], .12) for p in forecast])
        closed = (frame.get('navigation') or {}).get('closed_edges', [])
        for id, obj in closures.items():
            obj.hide_render = id not in closed
        rendered_pose = {'position': {'x': float(ego.location.x), 'y': float(ego.location.y)},
                         'yaw': float(ego.rotation_euler.z)}
        rendered_objects = [{'id': a['id'], 'position': {'x': float(actors[a['id']].location.x),
                                                       'y': float(actors[a['id']].location.y)}} for a in frame['objects']]
        state = {'frame_index': index, 'time': frame['time'], 'ego_pose': rendered_pose,
                 'objects': rendered_objects, 'closed_edges': closed,
                 'camera_position': list(camera.location)}
        state['display_scales'] = {'ego': list(ego.scale),
                                   'actors': {str(a['id']): list(actors[a['id']].scale) for a in frame['objects']}}
        if native_models or ground_mode:
            state['native_cuboids'] = native_cuboid_audit(native_models)
        if ground_mode:
            state['native_ground_cuboids'] = native_cuboid_audit(ground_models)
            if body_envelope is not None:
                state['body_envelope'] = native_cuboid_audit({'research-body-envelope': body_envelope})[0]
        audit.append(state)
        for spec in signal_specs:
            color=next(p['color'] for p in reversed(spec['phases']) if p['from']<=frame['time']+1e-9)
            for lamp,obj in signal_models[spec['stop_line']['id']].items():
                obj.data.materials[0]=lamp_materials[lamp if lamp==color else 'off']
        scene.render.filepath = str(output/f'{number:04d}.png')
        bpy.ops.render.render(write_still=True)
    (output/'audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    scene_info = {'style': CITY_STYLE if urban else STYLE, 'seed': 1729, 'scenery_counts': scenery,
                  'ego_model': 'hatchback', 'traffic_models': vehicle_models,
                  'camera': request.get('camera', 'ego'),
                  'actor_models': actor_models,
                  'road_user_meshes_display_only': bool(actor_models),
                  'mapped_signal_ids': list(signal_models),
                  'mapped_stop_sign_ids': [s['id'] for s in stop_specs],
                  'mapped_intersection_ids': [s['stop_line']['id'] for s in intersection_specs]}
    scene_info['opposing_display_lanes'] = opposing_display_lanes
    scene_info['service_display_lanes'] = service_display_lanes
    scene_info['left_traffic_display'] = bool(opposing_display_lanes) and all(lane['offset_m'] < 0 for lane in opposing_display_lanes)
    if scene_info['left_traffic_display']:
        scene_info['main_street_center_offset_m'] = max(lane['offset_m'] for lane in opposing_display_lanes)/2
    scene_info['environment'] = request.get('environment', 'suburban')
    if urban:
        scene_info['painted_lane_half_width_m'] = 1.8
        scene_info['crossing_stripe_actor_ids'] = [1, 4, 6]
    scene_info['dog_pairs'] = dog_pairs
    scene_info['dog_leashes_display_only'] = bool(dog_pairs)
    scene_info['display_road_extension_m'] = 20 if request.get('camera') == 'street' else 0
    scene_info['scenery_excludes_recorded_actor_paths'] = bool(exclusion_paths)
    scene_info['camera_projection'] = camera.data.type
    if camera.data.type == 'ORTHO':
        scene_info['camera_orthographic_scale_m'] = camera.data.ortho_scale
    scene_info['vehicle_display_dimensions_m'] = {
        'ego': dict(ego['display_dimensions_m']),
        'actors': {str(identifier): dict(obj['display_dimensions_m'])
                   for identifier, obj in actors.items() if obj.get('vehicle_model')}}
    scene_info['vehicle_display_scales'] = {'ego': list(ego.scale),
        'actors': {str(identifier): list(obj.scale) for identifier, obj in actors.items()}}
    if native_models or ground_mode:
        scene_info['native_scene_name'] = request['native_scene']['name']
        scene_info['native_cuboids'] = native_cuboid_audit(native_models)
    if ground_mode:
        scene_info['native_ground_cuboids'] = native_cuboid_audit(ground_models)
        scene_info['road_surface'] = 'actual SceneV2 support cuboids at flat datum 0 m with display-only asphalt corridor overlays'
        scene_info['scenery_physical'] = False
        if body_envelope is not None:
            scene_info['body_envelope'] = native_cuboid_audit({'research-body-envelope': body_envelope})[0]
    (output/'scene-info.json').write_text(json.dumps(scene_info, indent=2)+'\n')
    if request.get('scene_output'):
        bpy.ops.wm.save_as_mainfile(filepath=request['scene_output'])


if __name__ == '__main__':
    main()
