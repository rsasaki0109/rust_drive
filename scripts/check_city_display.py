"""Independent RustDriving urban/truck/dog evaluated-mesh display audit.

blender --background --factory-startup --threads 2 --python-exit-code 1
--python scripts/check_city_display.py -- --output assets/city-display-audit.json
Optional --preview artifacts/city-models-preview.png renders the display truck.
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

sys.path.insert(0,str(Path(__file__).resolve().parent))
from blender_assets import cube, material
from blender_city_assets import animate_dog, city_environment, dog, truck

TOLERANCE = 2e-5


def require(value,message):
    if not value:
        raise RuntimeError(message)


def geometry(root):
    graph = bpy.context.evaluated_depsgraph_get()
    points, body, tires = [], [], []
    inverse = root.matrix_world.inverted()
    for obj in root.children_recursive:
        if obj.type not in ('MESH','CURVE','FONT'):
            continue
        evaluated = obj.evaluated_get(graph)
        mesh = evaluated.to_mesh()
        try:
            local = [inverse @ evaluated.matrix_world @ vertex.co for vertex in mesh.vertices]
            points.extend(local)
            if obj.get('vehicle_display_part') == 'body':
                body.extend(local)
            if obj.name.startswith('Truck tire'):
                transform = obj.parent.matrix_world.inverted() @ evaluated.matrix_world
                radial = [math.hypot((transform @ vertex.co).x,(transform @ vertex.co).z)
                          for vertex in mesh.vertices]
                require(max(radial)-min(radial) < TOLERANCE,'Truck tire is not circular')
                require(abs(radial[0]-root['wheel_radius']) < TOLERANCE,'Wrong wheel animation radius')
                tires.append(radial[0])
        finally:
            evaluated.to_mesh_clear()
    require(bool(points),'No evaluated model geometry')
    return points,body,tires


def bounds(points):
    low = [min(point[i] for point in points) for i in range(3)]
    high = [max(point[i] for point in points) for i in range(3)]
    return {'minimum_m':low,'maximum_m':high,'extent_m':[b-a for a,b in zip(low,high)]}


def pose(root):
    return (tuple(root.location),tuple(root.rotation_euler),tuple(root.scale))


def fit(root,points):
    require(max(math.hypot(p.x,p.y) for p in points) <= root['display_radius_m']+TOLERANCE,
            'Mesh exceeds horizontal camera bound')
    require(max(p.z for p in points) <= root['display_height_m']+TOLERANCE,
            'Mesh exceeds camera height bound')
    require(min(p.z for p in points) >= -TOLERANCE,'Geometry penetrates flat datum')


def distance_to_path(x,y,edge):
    distances = []
    for a,b in zip(edge['points'],edge['points'][1:]):
        dx,dy = b['x']-a['x'],b['y']-a['y']
        square = dx*dx+dy*dy
        t = max(0,min(1,((x-a['x'])*dx+(y-a['y'])*dy)/square)) if square else 0
        distances.append(math.hypot(x-a['x']-t*dx,y-a['y']-t*dy))
    return min(distances)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('assets/city-display-audit.json'))
    parser.add_argument('--preview',type=Path)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    materials = [material('Audit '+name,color) for name,color in
                 [('paint',(.12,.34,.58)),('glass',(.09,.2,.27)),
                  ('tire',(.015,.015,.016)),('headlight',(.85,.85,.78))]]
    truck_cases,dog_cases = [],[]
    reference = None
    for radius in (1.0,2.2847319317591728):
        root = truck('RustDriving truck audit',radius,*materials)
        root.location,root.rotation_euler.z = (17,-9,0),.72
        initial = pose(root)
        measurements = []
        wheels = [obj for obj in root.children if obj.get('rolling_wheel')]
        require(len(wheels)==6,'Truck needs six wheels')
        require(root['declared_circle_radius']==radius,'Physical radius metadata changed')
        for angle in (0,.19,.73,1.57,2.81):
            for wheel in wheels:
                wheel.rotation_euler.y = angle
            bpy.context.view_layer.update()
            points,body,tires = geometry(root)
            fit(root,points)
            body_bounds = bounds(body)
            actual = body_bounds['extent_m'][:2]+[body_bounds['maximum_m'][2]]
            require(all(abs(a-b)<TOLERANCE for a,b in zip(actual,(7.5,2.45,3.3))),
                    f'Wrong truck SI body dimensions: {actual}')
            require(len(tires)==6,'Six actual circular tire meshes required')
            require(initial==pose(root),'Wheel animation changed root pose')
            measurements.append({'wheel_angle_rad':angle,'body':body_bounds,
                                 'full_display':bounds(points),'actual_tire_radii_m':tires})
        if reference is not None:
            require(measurements==reference,'Physical radius changes truck display geometry')
        reference = measurements
        truck_cases.append({'physical_radius_m':radius,'root_pose_unchanged':True,
                            'wheel_count':6,'measurements':measurements})
    reference = None
    for radius in (.3,.7):
        root = dog('RustDriving dog audit',radius)
        root.location,root.rotation_euler.z = (17,-9,0),.72
        initial = pose(root)
        measurements = []
        for time,speed in ((0,0),(.19,1.2),(.37,2),(.71,1.5),(1.3,.8)):
            animate_dog(root,time,speed)
            bpy.context.view_layer.update()
            points,_,_ = geometry(root)
            fit(root,points)
            actual = bounds(points)
            require(.85 < actual['extent_m'][0] < 1.10 and .60 < actual['maximum_m'][2] < .70,
                    'Dog geometry is not the declared medium SI size')
            require(initial==pose(root),'Dog gait changed root pose')
            measurements.append({'time_s':time,'cosmetic_speed_mps':speed,'bounds':actual})
        if reference is not None:
            require(measurements==reference,'Physical radius changes dog display geometry')
        reference = measurements
        dog_cases.append({'physical_radius_m':radius,'root_pose_unchanged':True,'measurements':measurements})
    # Crossing and parallel recorded paths force actual exclusion-aware layout.
    roads = [{'points':[{'x':0,'y':0},{'x':90,'y':0}],'half_width':6.}]
    actor_paths = [{'points':[{'x':0,'y':14},{'x':90,'y':14}],'half_width':1.2},
                   {'points':[{'x':36,'y':-30},{'x':36,'y':30}],'half_width':1.2}]
    counts = city_environment(roads,exclusion_paths=actor_paths)
    bpy.context.view_layer.update()
    scenery = []
    for root in bpy.data.objects:
        if not root.get('placement_circle'):
            continue
        circle = root['placement_circle']
        points,_,_ = geometry(root)
        require(max(math.hypot(p.x,p.y) for p in points) <= circle['radius_m']+TOLERANCE,
                'City geometry exceeds its conservative placement circle')
        margins = [distance_to_path(circle['x'],circle['y'],edge)-edge['half_width']-circle['radius_m']
                   for edge in roads+actor_paths]
        require(min(margins) > .5-TOLERANCE,'City scenery crosses a road or recorded actor path')
        scenery.append({'kind':root['scenery_kind'],'placement_circle':dict(circle),
                        'minimum_path_margin_m':min(margins)})
    require(counts['buildings']>=4 and counts['lamps']>=2,'Urban audit layout is unexpectedly empty')
    repository = Path(__file__).resolve().parent.parent
    sources = [Path(__file__).resolve(),repository/'scripts/blender_city_assets.py',repository/'scripts/blender_assets.py']
    report = {'schema':'rustdriving-city-display-audit-v1','passed':True,'display_only':True,
              'scope':'Original SI cosmetics and independent evaluated geometry/placement audit. No mesh-based sensing, collision, semantic detection or driving acceptance claim.',
              'blender_version':bpy.app.version_string,'units':'metres','tolerance_m':TOLERANCE,
              'truck_pose_cases':10,'dog_pose_cases':10,'truck':truck_cases,'dog':dog_cases,
              'city':{'counts':counts,'roads':roads,'excluded_recorded_paths':actor_paths,'placements':scenery},
              'source_sha256':{str(path.relative_to(repository)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}}
    if args.preview:
        for obj in bpy.data.objects:
            obj.hide_render = True
        preview = truck('RustDriving display truck',1.,*materials)
        for obj in [preview]+list(preview.children_recursive):
            obj.hide_render=False
        floor = cube('Showroom floor',(0,0,-.055),(50,50,.1),material('Showroom grey',(.3,.34,.36)))
        scene = bpy.context.scene
        scene.render.engine='CYCLES'
        scene.cycles.device='CPU'
        scene.cycles.samples=8
        scene.cycles.use_denoising=False
        scene.cycles.max_bounces=3
        scene.world.use_nodes=True
        scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.64,.76,.92,1)
        scene.world.node_tree.nodes['Background'].inputs[1].default_value=.5
        bpy.ops.object.light_add(type='AREA',location=(3,-5,8))
        light=bpy.context.object
        light.data.energy,light.data.size=1700,6
        light.rotation_euler=(Vector((0,0,1.5))-light.location).to_track_quat('-Z','Y').to_euler()
        bpy.ops.object.camera_add(location=(10,-13,7))
        camera=bpy.context.object
        camera.rotation_euler=(Vector((0,0,1.4))-camera.location).to_track_quat('-Z','Y').to_euler()
        camera.data.type,camera.data.ortho_scale='ORTHO',11
        scene.camera=camera
        scene.render.resolution_x,scene.render.resolution_y=1000,640
        scene.render.resolution_percentage=100
        scene.render.filepath=str(args.preview.resolve())
        args.preview.parent.mkdir(parents=True,exist_ok=True)
        bpy.ops.render.render(write_still=True)
        report['showroom_preview']={'file':str(args.preview),'sha256':hashlib.sha256(args.preview.read_bytes()).hexdigest(),
                                    'display_only':True,'driving_record':False,'samples':8,'device':'CPU'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'passed':True,'truck_pose_cases':10,'dog_pose_cases':10,'city':counts,'source_sha256':report['source_sha256']}))


if __name__=='__main__':
    main()
