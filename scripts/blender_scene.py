"""Blender worker: render recorded world states, never advance a simulator."""
import json
import math
from pathlib import Path
import sys

import bpy
from bpy_extras.object_utils import world_to_camera_view
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from blender_assets import STYLE, barrel, car, cube, environment, material, traffic_signal


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


def road(points, half_width, asphalt, marking):
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
    for i in range(0, len(points)-1, 3):
        a, b = points[i:i+2]
        line('Decorative center marking', [(a['x'], a['y'], 0.06), (b['x'], b['y'], 0.06)], marking, 0.04)
    return obj


def main():
    request = json.loads(Path(sys.argv[sys.argv.index('--')+1]).read_text())
    run = json.loads(Path(request['run']).read_text())
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
    closures = {}
    roads = []
    for edge in edges:
        surface = road(edge['points'], edge['half_width'], asphalt, marking)
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
    scenery = environment(edges)
    signal_specs = run['scenario'].get('traffic_signals', [])
    signal_models = {}
    if signal_specs:
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
    dynamic_ids = {s['id'] for f in run['frames'] for s in f.get('traffic', [])}
    actors = {}
    vehicle_models = []
    actor_paints = [orange, material('Van ivory', (.72,.75,.68), .2, .3),
                    material('Pickup green', (.06,.36,.23), .35, .27)]
    for frame in run['frames']:
        for actor in frame['objects']:
            if actor['id'] in actors:
                continue
            if actor['id'] in dynamic_ids:
                number = len(vehicle_models)
                variant = request.get('traffic_models', ['hatchback'])[number % len(request.get('traffic_models', ['hatchback']))]
                paint = actor_paints[number % len(actor_paints)] if len(request.get('traffic_models', ['hatchback'])) > 1 else orange
                actors[actor['id']] = car('Recorded reactive actor', actor['radius'], paint, glass, tire, headlight, variant=variant)
                vehicle_models.append({'id': actor['id'], 'model': variant, 'color': list(paint.diffuse_color[:3])})
            else:
                actors[actor['id']] = barrel(actor['radius'], orange, marking)
    planned = line('Actual planned trajectory', [], teal, .075)
    predictions = line('Actual track forecasts', [], purple, .04)
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
        camera.location = (x-11, y-16, 16)
        target = (x+4, y, 0)
        if request.get('camera') == 'traffic':
            bodies = [{'position': pose['position'], 'radius': run['vehicle']['radius']}]+[a for a in frame['objects'] if a['id'] in dynamic_ids]
            positions = [a['position'] for a in bodies]
            min_x,max_x = min(p['x'] for p in positions),max(p['x'] for p in positions)
            min_y,max_y = min(p['y'] for p in positions),max(p['y'] for p in positions)
            height = max(16, math.hypot(max_x-min_x,max_y-min_y)*.45+16)
            center_x,center_y = (min_x+max_x)/2,(min_y+max_y)/2
            target = (center_x,center_y,0)
            # Fit the complete display bounds, not just the center positions:
            # a long queue can otherwise clip ego at the start in perspective.
            corners = [Vector((a['position']['x']+sx*(a['radius']+.5),
                               a['position']['y']+sy*(a['radius']+.5),z))
                       for a in bodies for sx in [-1,1] for sy in [-1,1]
                       for z in [0,2*a['radius']]]
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
            if actor['id'] in dynamic_ids and index > 0:
                previous = next((a for a in run['frames'][index-1]['objects'] if a['id'] == actor['id']), None)
                if previous:
                    dx, dy = actor['position']['x']-previous['position']['x'], actor['position']['y']-previous['position']['y']
                    if math.hypot(dx, dy) > 1e-6:
                        obj.rotation_euler.z = math.atan2(dy, dx)
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
        audit.append({'frame_index': index, 'time': frame['time'], 'ego_pose': rendered_pose,
                      'objects': rendered_objects, 'closed_edges': closed,
                      'camera_position': list(camera.location)})
        for spec in signal_specs:
            color=next(p['color'] for p in reversed(spec['phases']) if p['from']<=frame['time']+1e-9)
            for lamp,obj in signal_models[spec['stop_line']['id']].items():
                obj.data.materials[0]=lamp_materials[lamp if lamp==color else 'off']
        scene.render.filepath = str(output/f'{number:04d}.png')
        bpy.ops.render.render(write_still=True)
    (output/'audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    (output/'scene-info.json').write_text(json.dumps({'style': STYLE, 'seed': 1729, 'scenery_counts': scenery,
                                                   'ego_model': 'hatchback', 'traffic_models': vehicle_models,
                                                   'camera': request.get('camera', 'ego'),
                                                   'mapped_signal_ids': list(signal_models)}, indent=2)+'\n')
    if request.get('scene_output'):
        bpy.ops.wm.save_as_mainfile(filepath=request['scene_output'])


if __name__ == '__main__':
    main()
