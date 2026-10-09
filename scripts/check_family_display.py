"""Independent RustDriving family mesh/contact/camera-bound display audit.

blender --background --factory-startup --threads 2 --python-exit-code 1
--python scripts/check_family_display.py -- --output assets/family-display-audit.json
Optional --preview assets/family-models.png renders a showroom, not a driving record.
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
from blender_assets import cube,material
import blender_family_assets as family

TOLERANCE = 2e-5
POSES = ((0,0),(.19,.6),(.37,.7),(.71,1.3),(1.3,.5))


def require(value,message):
    if not value:
        raise RuntimeError(message)


def pose(root):
    return (tuple(root.location),tuple(root.rotation_euler),tuple(root.scale))


def bounds(points):
    low = [min(point[i] for point in points) for i in range(3)]
    high = [max(point[i] for point in points) for i in range(3)]
    return {'minimum_m':low,'maximum_m':high,'extent_m':[b-a for a,b in zip(low,high)]}


def inspect(root):
    graph = bpy.context.evaluated_depsgraph_get()
    inverse = root.matrix_world.inverted()
    points,tires = [],[]
    for obj in root.children_recursive:
        if obj.type not in ('MESH','CURVE','FONT'):
            continue
        evaluated = obj.evaluated_get(graph)
        mesh = evaluated.to_mesh()
        try:
            transform = inverse @ evaluated.matrix_world
            points.extend(transform @ vertex.co for vertex in mesh.vertices)
            if 'stroller tire' in obj.name:
                transform = obj.parent.matrix_world.inverted() @ evaluated.matrix_world
                radial = [math.hypot((transform @ vertex.co).x,(transform @ vertex.co).z)
                          for vertex in mesh.vertices]
                require(max(radial)-min(radial)<TOLERANCE,'Stroller tire is stretched')
                require(abs(radial[0]-.15)<TOLERANCE,'Stroller tire is not actual SI .15 m radius')
                tires.append(radial[0])
        finally:
            evaluated.to_mesh_clear()
    require(bool(points),'Missing avatar geometry')
    actual = bounds(points)
    require(max(math.hypot(p.x,p.y) for p in points)<=root['display_radius_m']+TOLERANCE,
            'Avatar/accessories exceed camera radius bound')
    require(actual['maximum_m'][2]<=root['display_height_m']+TOLERANCE,'Avatar exceeds camera height bound')
    require(actual['minimum_m'][2]>=-TOLERANCE,'Avatar geometry penetrates flat datum')
    kind = root['avatar_kind']
    if kind=='child':
        require(1.10<actual['maximum_m'][2]<1.25,'Child has incorrect SI height')
    else:
        require(1.75<actual['maximum_m'][2]<2.0,'Adult has incorrect SI height')
    contacts = []
    if kind in ('elder','parent_stroller'):
        # Compare actual joint transforms with declared accessory geometry.
        rig = family._FAMILIES[root.as_pointer()]['rig']
        for side in ((-1,) if kind=='elder' else (-1,1)):
            hand = rig['limbs'][side]['hand']
            actual_hand = inverse @ hand.matrix_world.translation
            if kind=='elder':
                target = inverse @ rig['model'].matrix_world @ Vector((.27,-.28,.97))
            else:
                target = Vector((.38,side*.26,1.11))
            error = (actual_hand-target).length
            require(error<TOLERANCE,'Actual hand joint misses cane grip/stroller pushbar')
            contacts.append({'side':side,'actual_hand_local_m':list(actual_hand),
                             'target_local_m':list(target),'error_m':error})
    require(len(tires)==(4 if kind=='parent_stroller' else 0),'Incorrect stroller wheel count')
    return {'bounds':actual,'actual_stroller_tire_radii_m':tires,'accessory_hand_contacts':contacts}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('assets/family-display-audit.json'))
    parser.add_argument('--preview',type=Path)
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else [])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    results=[]
    for kind,builder in [('elder',family.elder),('child',family.child),('parent_stroller',family.parent_stroller)]:
        reference=None
        for radius in (.3,1.8):
            root=builder('RustDriving '+kind,radius)
            root.location,root.rotation_euler.z=(17,-9,0),.72
            original=pose(root)
            require(tuple(root.scale)==(1.,1.,1.),'World root must retain unit scale')
            require(root['declared_circle_radius']==radius,'Physical radius metadata changed')
            measurements=[]
            for time,speed in POSES:
                family.animate_family(root,time,speed)
                bpy.context.view_layer.update()
                actual=inspect(root)
                require(original==pose(root),'Display animation changes recorded root pose/scale')
                measurements.append({'time_s':time,'cosmetic_speed_mps':speed,**actual})
            if reference is not None:
                for actual,expected in zip(measurements,reference):
                    for key in ('minimum_m','maximum_m'):
                        require(all(abs(a-b)<TOLERANCE for a,b in zip(actual['bounds'][key],expected['bounds'][key])),
                                'Physical radius changes avatar size')
            reference=measurements
            results.append({'kind':kind,'physical_radius_m':radius,'root_pose_unchanged':True,
                            'camera_height_m':root['display_height_m'],'camera_radius_m':root['display_radius_m'],
                            'measurements':measurements})
    repository=Path(__file__).resolve().parent.parent
    sources=[Path(__file__).resolve(),repository/'scripts/blender_family_assets.py',
             repository/'scripts/blender_vru_assets.py',repository/'scripts/blender_assets.py']
    hashes={str(path.relative_to(repository)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    report={'schema':'rustdriving-family-display-audit-v1','passed':True,'display_only':True,
            'scope':'Original display avatars, cane/stroller hand contacts, SI shapes and round wheels. No physical mesh collision or semantic detection; baby belongs to one combined parent actor.',
            'blender_version':bpy.app.version_string,'units':'metres','tolerance_m':TOLERANCE,
            'evaluated_pose_cases':30,'source_sha256':hashes,'results':results}
    if args.preview:
        for obj in bpy.data.objects:
            obj.hide_render=True
        models=[family.elder('Showroom elder',.4),family.child('Showroom child',.3),
                family.parent_stroller('Showroom parent',1.8)]
        for root,position in zip(models,[(-1.9,-.25,0),(-.4,-.70,0),(1.,.3,0)]):
            root.location=position
            family.animate_family(root,.39,.6)
        cube('Family showroom floor',(0,0,-.055),(30,30,.1),material('Showroom concrete',(.32,.35,.37)))
        scene=bpy.context.scene
        scene.render.engine='CYCLES'
        scene.cycles.device='CPU'
        scene.cycles.samples=8
        scene.cycles.max_bounces=3
        scene.cycles.use_denoising=False
        scene.world.use_nodes=True
        scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.64,.76,.92,1)
        scene.world.node_tree.nodes['Background'].inputs[1].default_value=.5
        bpy.ops.object.light_add(type='AREA',location=(2,-4,6))
        light=bpy.context.object
        light.data.energy,light.data.size=1400,5
        light.rotation_euler=(Vector((0,0,1))-light.location).to_track_quat('-Z','Y').to_euler()
        bpy.ops.object.camera_add(location=(5,-9,3.6))
        camera=bpy.context.object
        camera.rotation_euler=(Vector((.15,0,1))-camera.location).to_track_quat('-Z','Y').to_euler()
        camera.data.type,camera.data.ortho_scale='ORTHO',6.4
        scene.camera=camera
        scene.render.resolution_x,scene.render.resolution_y=1100,720
        scene.render.resolution_percentage=100
        scene.render.filepath=str(args.preview.resolve())
        args.preview.parent.mkdir(parents=True,exist_ok=True)
        bpy.ops.render.render(write_still=True)
        metadata={'schema':'rustdriving-family-models-v1','display_only':True,'driving_record':False,
                  'scope':'Original showroom display models; no independently simulated baby.',
                  'renderer':{'blender':bpy.app.version_string,'device':'CPU','samples':8,'resolution_px':[1100,720]},
                  'source_sha256':hashes,'image':{'file':args.preview.name,'bytes':args.preview.stat().st_size,
                  'sha256':hashlib.sha256(args.preview.read_bytes()).hexdigest()}}
        args.preview.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
        report['showroom_preview']=metadata
    require(hashes=={str(path.relative_to(repository)):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
            'Display source changed during audit/render')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'passed':True,'evaluated_pose_cases':30,'source_sha256':hashes}))


if __name__=='__main__':
    main()
