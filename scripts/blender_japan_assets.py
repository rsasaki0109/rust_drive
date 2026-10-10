"""Original Apache-2.0 RustDriving Japanese streetscape display assets.

SI cosmetics only. Cross streets are aligned to recorded actors by the caller;
work-zone dressing never adds physical obstacles or operational observations.
Optional Japanese lettering uses an installed OFL-1.1 Noto CJK font, without
downloading or redistributing it; otherwise road lettering is Romanized.
"""
import hashlib
import math
from pathlib import Path

import bpy

from blender_assets import cube, cylinder, material, mesh
from blender_city_assets import city_environment
import blender_vru_assets as vru

STYLE = 'original-japanese-city-display-v1'
JUNCTION_CENTERS = (15.,36.)
JUNCTION_HALF_WIDTHS = (3.4,1.35)
JUNCTION_ONE_WAY = (False,True)
_WORKERS = {}
_CJK_FONT = None


def _root(name):
    root=bpy.data.objects.new(name,None)
    bpy.context.collection.objects.link(root)
    root['display_only']=True
    root['asset_style']=STYLE
    return root


def _text(name,body,position,size,mat,parent=None,rotation=(math.pi/2,0,0),font=None):
    data=bpy.data.curves.new(name,'FONT')
    data.body,data.size=body,size
    data.align_x,data.align_y='CENTER','CENTER'
    data.extrude=.001
    if font:
        data.font=font
    obj=bpy.data.objects.new(name,data)
    bpy.context.collection.objects.link(obj)
    obj.parent,obj.location,obj.rotation_euler=parent,position,rotation
    obj.data.materials.append(mat)
    obj['display_only']=True
    return obj


def japan_city_details():
    """Add original street-facing storefronts to existing city buildings."""
    trim=material('Japan storefront trim',(.09,.12,.14),.15,.45)
    glass=material('Japan shop glazing',(.10,.24,.29),.25,.28)
    ivory=material('Japan sign lettering',(.94,.93,.83))
    signs=[('KONBINI',(.045,.34,.20)),('BOOKS',(.18,.24,.47)),
           ('CAFE',(.48,.20,.11)),('KUSURI',(.46,.12,.17))]
    counts={'storefronts':0,'signs':0,'balconies':0,'air_conditioners':0}
    for root in list(bpy.data.objects):
        if root.get('scenery_kind')!='building':
            continue
        index=counts['storefronts']
        label,color=signs[index%len(signs)]
        sign=material('Original '+label+' sign',color)
        cube('Japan storefront glazing',(0,-4.095,1.3),(7.0,.025,2.1),glass,root)
        for x in (-3.5,-1.75,0,1.75,3.5):
            cube('Japan shop mullion',(x,-4.14,1.3),(.07,.06,2.22),trim,root)
        cube('Japan shop sign',(0,-4.145,2.98),(8.0,.20,.70),sign,root,.025)
        _text('Original storefront label',label,(0,-4.252,2.98),.43,ivory,root)
        cube('Japan shop awning',(0,-4.40,2.58),(8.10,.62,.10),sign,root,.015)
        for x in (-3.5,-2.5,-1.5,-.5,.5,1.5,2.5,3.5):
            cube('Japan awning stripe',(x,-4.40,2.638),(.18,.60,.005),ivory,root)
        height=max(obj.location.z*2 for obj in root.children if obj.name.startswith('City building mass'))
        for z in (4.5,7.7,10.9):
            if z+1.1>height:
                continue
            for side in (-1,1):
                cube('Japan apartment balcony',(side*3.7,-4.35,z), (2.5,.68,.10),trim,root)
                for xoffset in (-1.15,0,1.15):
                    cylinder('Japan balcony rail post',(side*3.7+xoffset,-4.65,z+.43),.018,.86,trim,root)
                cube('Japan balcony handrail',(side*3.7,-4.65,z+.87),(2.5,.035,.035),trim,root)
                cube('Japan wall air conditioner',(side*3.7,-4.18,z+1.05),(.62,.31,.42),ivory,root,.02)
                counts['air_conditioners']+=1
                counts['balconies']+=1
        counts['storefronts']+=1
        counts['signs']+=1
    return counts


def japan_environment(edges,exclusion_paths=None):
    """City rows/sidewalks respect both supplied corridors and cross streets.

    Dense auxiliary corridor sampling lets the original sidewalk helper omit
    raised slabs at actual junctions. The caller splits its own median/lines.
    """
    crossroads=[{'points':[{'x':center,'y':float(y)} for y in range(-30,35,2)],
                 'half_width':width} for center,width in zip(JUNCTION_CENTERS,JUNCTION_HALF_WIDTHS)]
    counts=city_environment(list(edges)+crossroads,exclusion_paths=exclusion_paths)
    counts.update(japan_city_details())
    counts['style']=STYLE
    counts['junction_centers_x_m']=list(JUNCTION_CENTERS)
    counts['cross_street_half_widths_m']=list(JUNCTION_HALF_WIDTHS)
    counts['cross_street_one_way_directions']=['+Y' if value else 'two-way' for value in JUNCTION_ONE_WAY]
    counts['junction_sidewalks_cut']=True
    bpy.context.scene['display_asset_style']=STYLE
    return counts


def cross_street(name,center_x,min_y=-30,max_y=34,half_width=3.6,one_way=False):
    """Decorative road/paint at a recorded cross-car corridor; no physical map."""
    if not all(math.isfinite(value) for value in (center_x,min_y,max_y,half_width)) or min_y>=max_y or half_width<=0:
        raise ValueError('Cross street needs finite positive dimensions')
    root=_root(name)
    root['cross_street_center_x_m']=center_x
    root['cross_street_bounds_m']={'min_y':min_y,'max_y':max_y,'half_width':half_width}
    root['one_way_direction']='+Y' if one_way else 'two-way'
    asphalt=material(name+' asphalt',(.065,.08,.11))
    white=material(name+' white paint',(.76,.78,.73))
    amber=material(name+' centerline',(.80,.49,.08))
    cube(name+' asphalt',(center_x,(min_y+max_y)/2,.035),(2*half_width,max_y-min_y,.03),asphalt,root)
    # Mark only outside each main carriageway junction, avoiding paint through
    # live crossing lanes. Main-direction median/edge gaps belong to the caller.
    for low,high in ((min_y,-10.4),(-5.6,-2.4),(2.4,max_y)):
        if high<=low:
            continue
        if not one_way:
            cube(name+' center paint',(center_x,(low+high)/2,.057),(.075,high-low,.012),amber,root)
        elif high-low>2.0:
            middle=(low+high)/2
            arrow=mesh(name+' one-way +Y arrow',
                       [(center_x-.075,middle-.60,.069),(center_x+.075,middle-.60,.069),
                        (center_x+.075,middle+.30,.069),(center_x+.34,middle+.30,.069),
                        (center_x,middle+.85,.069),(center_x-.34,middle+.30,.069),
                        (center_x-.075,middle+.30,.069)],[(0,1,2,3,4,5,6)],white,root)
            arrow['display_arrow_direction']='+Y'
        for side in (-1,1):
            cube(name+' road edge',(center_x+side*(half_width-.10),(low+high)/2,.057),(.065,high-low,.012),white,root)
    for main_y in (-8.,0.):
        bands=(0.,) if one_way else (-max(0.,half_width-.8),max(0.,half_width-.8))
        for offset in bands:
            # Paint stays inside the independently validated street envelope;
            # vehicles stopped outside the junction cannot occupy this band.
            for i in range(7):
                cube(name+' main-road zebra',(center_x+offset,main_y-1.35+i*.45,.069),
                     (1.5,.24,.014),white,root)
        for side in (-1,1):
            # Crosswalk across the perpendicular road, outside the live main lane.
            stripe_count=max(1,math.floor((2*half_width-.6)/.6+1e-9)+1)
            for i in range(stripe_count):
                cube(name+' cross-road zebra',(center_x-half_width+.30+i*.60,main_y+side*2.6,.069),
                     (.28,1.45,.014),white,root)
    return root


def stop_text(name,position,yaw,size=1.25):
    """Optional display road lettering; caller supplies an actual mapped line."""
    global _CJK_FONT
    font_path=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')
    body='STOP'
    if font_path.is_file():
        if _CJK_FONT is None:
            _CJK_FONT=bpy.data.fonts.load(str(font_path))
        body='止まれ'
    obj=_text(name,body,position,size,material(name+' white paint',(.78,.80,.75)),
              rotation=(0,0,yaw-math.pi/2),font=_CJK_FONT)
    obj['lettering_scope']='Display text only; not a recognized or operational sign'
    if _CJK_FONT:
        obj['font_license']='SIL Open Font License 1.1; installed Noto CJK; no font redistribution'
        obj['font_sha256']=hashlib.sha256(font_path.read_bytes()).hexdigest()
    else:
        obj['font_source']='Blender built-in Bfont; Roman fallback'
    return obj


def horizontal_signal(name,position,yaw,materials):
    """Original Japanese horizontal display head; caller sets recorded colors."""
    root=_root(name)
    root.location,root.rotation_euler=(*position,0),(0,0,yaw)
    dark=material(name+' signal housing',(.028,.035,.038),.2,.35)
    steel=material(name+' signal pole',(.29,.33,.34),.65,.4)
    cylinder(name+' signal pole',(0,0,1.85),.065,3.7,steel,root)
    cube(name+' horizontal housing',(0,0,3.82),(.30,1.10,.36),dark,root,.045)
    lenses={}
    # Viewed from the front (-X), green is on the left (+Y), then yellow/red.
    for color,y in (('Green',.33),('Yellow',0),('Red',-.33)):
        lenses[color]=cylinder(name+' '+color+' lens',(-.172,y,3.82),.12,.04,materials['off'],root,(0,math.pi/2,0))
        cube(name+' signal visor',(-.24,y,4.0),(.28,.27,.035),dark,root)
    return root,lenses


def _cone(name,x,y,parent,orange,white,rubber):
    cube(name+' base',(x,y,.035),(.38,.38,.07),rubber,parent,.015)
    count=24
    levels=[(.07,.16),(.34,.085),(.43,.06),(.68,.015)]
    vertices=[(x+r*math.cos(i*math.tau/count),y+r*math.sin(i*math.tau/count),z)
              for z,r in levels for i in range(count)]
    for k,mat in ((0,orange),(1,white),(2,orange)):
        mesh(name+' cone stripe',vertices,[(k*count+i,k*count+(i+1)%count,
             (k+1)*count+(i+1)%count,(k+1)*count+i) for i in range(count)],mat,parent)


def work_zone(name,start_s=62,end_s=78,lateral=0):
    """Cosmetic dressing only; the actual physical barrier remains separate."""
    root=_root(name)
    root['work_zone_scope']='Display cones/tape/excavation tint/equipment; physical barrier is separately recorded native geometry'
    root['work_zone_bounds_m']={'start_s':start_s,'end_s':end_s,'lateral':lateral}
    orange=material(name+' cone orange',(.94,.22,.025))
    white=material(name+' reflective white',(.90,.89,.78))
    dark=material(name+' rubber and soil',(.075,.06,.045),roughness=.9)
    yellow=material(name+' safety yellow',(.93,.65,.06))
    metal=material(name+' equipment alloy',(.32,.36,.38),.55,.5)
    for x in [start_s+i*3.2 for i in range(int((end_s-start_s)/3.2)+1)]:
        for side in (-1,1):
            y=lateral+side*1.70
            _cone(name+' safety cone',x,y,root,orange,white,dark)
            cylinder(name+' tape post',(x,y,.53),.025,1.06,metal,root)
    for side in (-1,1):
        for i in range(int((end_s-start_s)/.4)):
            cube(name+' striped safety tape',(start_s+.2+i*.4,lateral+side*1.70,.85),
                 (.40,.018,.10),yellow if i%2==0 else dark,root)
    cube(name+' excavation display tint',((start_s+end_s)/2,lateral,.056),
         (end_s-start_s-2,2.80,.014),dark,root)
    # Small original tools/piles sit in the closed zone. This is a surface
    # illustration, not a hole in the simulator's supporting ground.
    cube(name+' equipment crate',(end_s-1,lateral+.55,.30),(.70,.55,.48),orange,root,.03)
    for offset in (-.4,0,.4):
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=1,radius=1)
        obj=bpy.context.object
        obj.name,obj.parent,obj.location,obj.scale=name+' decorative aggregate',root,(end_s-2+offset,lateral-.5,.25),(.45,.38,.25)
        obj.data.materials.append(dark)
    sign=cube(name+' work warning board',(start_s-1,lateral+2.3,1.10),(1.0,.06,.65),yellow,root,.025)
    cylinder(name+' warning stand',(start_s-1,lateral+2.3,.52),.035,1.04,metal,root)
    _text(name+' WORKS label','WORKS',(start_s-1,lateral+2.263,1.10),.22,dark,root)
    return root


def worker(name,radius,materials=None):
    """Original fixed adult display worker with hardhat/vest/shovel."""
    palette=dict(materials or {})
    palette.setdefault('jacket',material(name+' safety orange',(.90,.28,.035),roughness=.8))
    root=vru.pedestrian(name,.4,palette)
    root['declared_circle_radius']=radius
    root['worker_kind']='original-road-worker'
    root['asset_style']=STYLE
    root['display_radius_m'],root['display_height_m']=.85,2.20
    root['display_dimensions_m']={'height':1.95,'accessory':'shovel and hardhat'}
    rig=vru._RIGS[root.as_pointer()]
    model=rig['model']
    yellow=material(name+' hardhat',(.95,.69,.06),roughness=.5)
    steel=material(name+' shovel metal',(.36,.39,.40),.6,.5)
    dark=material(name+' shovel grip',(.025,.028,.03))
    reflective=material(name+' vest reflective bands',(.91,.91,.70),.15,.4)
    head=next(obj for obj in rig['torso'].children if obj.type=='EMPTY' and 'head joint' in obj.name)
    vru._ellipsoid(name+' hardhat crown',(-.01,0,.10),(.128,.105,.075),yellow,head)
    vru._ellipsoid(name+' hardhat brim',(.012,0,.060),(.151,.122,.013),yellow,head)
    for side in (-1,1):
        vru._curve(name+' vertical reflective vest band',[(.13,side*.11,.11),(.135,side*.13,.32),(.10,side*.14,.42)],
                   .016,reflective,rig['torso'])
    vru._curve(name+' horizontal reflective vest band',[(.106,-.155,.14),(.135,0,.14),(.106,.155,.14)],
               .017,reflective,rig['torso'])
    vru._span(name+' shovel shaft',(.34,-.27,.13),(.34,-.27,.99),.014,dark,model)
    blade=cube(name+' shovel blade',(.34,-.27,.12),(.035,.18,.23),steel,model,.02)
    blade.rotation_euler.y=.12
    _WORKERS[root.as_pointer()]=rig
    animate_worker(root,0,0)
    return root


def animate_worker(root,time,observed_speed):
    """Pose a display worker; world root and physical radius stay unchanged."""
    vru.animate(root,time,observed_speed)
    rig=_WORKERS[root.as_pointer()]
    vru._arm(rig['limbs'][-1],(0,-.215,1.345),(.14,-.27,1.12),(.34,-.27,.99))
