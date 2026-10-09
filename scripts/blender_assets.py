"""Original procedural display assets; no sensing or collision geometry."""
import math
import random

import bmesh
import bpy


STYLE = 'suburban-test-road-v2'
VEHICLE_MODELS = ('hatchback', 'sedan', 'van', 'pickup')


def material(name, color, metallic=0, roughness=.45):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color, 1)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    if metallic == 0:
        specular = shader.inputs.get('Specular IOR Level') or shader.inputs.get('Specular')
        if specular:
            specular.default_value = 0
    return mat


def mesh(name, vertices, faces, mat, parent=None, bevel=0):
    data = bpy.data.meshes.new(name)
    data.from_pydata(vertices, [], faces)
    data.update()
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(mat)
    if parent:
        obj.parent = parent
    if bevel:
        mod = obj.modifiers.new('Rounded edges', 'BEVEL')
        mod.width, mod.segments = bevel, 2
        obj.modifiers.new('Normals', 'WEIGHTED_NORMAL')
    return obj


def cube(name, position, size, mat, parent=None, bevel=0):
    x, y, z = size
    vertices = [(a*x/2, b*y/2, c*z/2) for a, b, c in
                [(-1,-1,-1), (1,-1,-1), (1,1,-1), (-1,1,-1),
                 (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1)]]
    obj = mesh(name, vertices, [(0,3,2,1),(4,5,6,7),(0,1,5,4),
                              (1,2,6,5),(2,3,7,6),(3,0,4,7)], mat, parent, bevel)
    obj.location = position
    return obj


def cylinder(name, position, radius, depth, mat, parent=None, rotation=(0,0,0)):
    count = 24
    vertices = [(radius*math.cos(i*math.tau/count), radius*math.sin(i*math.tau/count), z)
                for z in [-depth/2, depth/2] for i in range(count)]
    faces = [tuple(reversed(range(count))), tuple(range(count, 2*count))]
    faces += [(i, (i+1)%count, (i+1)%count+count, i+count) for i in range(count)]
    obj = mesh(name, vertices, faces, mat, parent)
    obj.location, obj.rotation_euler = position, rotation
    for polygon in obj.data.polygons[2:]:
        polygon.use_smooth = True
    return obj


def variant_cabin(parent, r, variant, paint, glass, trim):
    rear, front, rear_roof, front_roof, height = {
        'sedan': (-.55,.34,-.35,.10,.84),
        'van': (-.78,.42,-.74,.22,1.22),
        'pickup': (-.12,.44,-.12,.18,1.04),
    }[variant]
    points=[(rear,-.41,.53),(front,-.41,.53),(front_roof,-.34,height),(rear_roof,-.34,height),
            (rear,.41,.53),(front,.41,.53),(front_roof,.34,height),(rear_roof,.34,height)]
    mesh(variant+' cabin',[(x*r,y*r,z*r) for x,y,z in points],
         [(0,1,2,3),(7,6,5,4),(0,4,5,1),(3,2,6,7),(1,5,6,2),(4,0,3,7)],paint,parent,.015*r)
    def panel(name, points):
        mesh(name,[(x*r,y*r,z*r) for x,y,z in points],[(0,1,2,3)],glass,parent)
    def slope(a,b,z):return a+(b-a)*(z-.53)/(height-.53)
    low_front,high_front=slope(front,front_roof,.57)+.005,slope(front,front_roof,height-.035)+.005
    low_rear,high_rear=slope(rear,rear_roof,.57)-.005,slope(rear,rear_roof,height-.035)-.005
    panel('Windshield',[(low_front,-.37,.57),(low_front,.37,.57),
                         (high_front,.30,height-.035),(high_front,-.30,height-.035)])
    panel('Rear glazing',[(low_rear,.37,.57),(low_rear,-.37,.57),
                          (high_rear,-.30,height-.035),(high_rear,.30,height-.035)])
    for side in [-1,1]:
        back = -.04 if variant=='van' else rear+.07
        roof_back = -.04 if variant=='van' else rear_roof+.025
        panel('Side glazing',[(front-.05,side*.412,.575),(back,side*.412,.575),
                               (roof_back,side*.355,height-.035),(front_roof-.025,side*.355,height-.035)])
        if variant=='sedan':
            cube('Sedan B pillar',(-.14*r,side*.38*r,.70*r),(.035*r,.035*r,.27*r),paint,parent)
        if variant=='van':
            cube('Cargo side panel',(-.40*r,side*.414*r,.85*r),(.54*r,.012*r,.44*r),paint,parent,.025*r)
            cube('Cargo door rail',(-.37*r,side*.427*r,.68*r),(.60*r,.014*r,.022*r),trim,parent)
    if variant=='van':
        cube('Rear door seam',(-.78*r,0,.82*r),(.018*r,.016*r,.50*r),trim,parent)
    if variant=='pickup':
        cube('Pickup bed floor',(-.49*r,0,.55*r),(.68*r,.77*r,.035*r),trim,parent)
        for side in [-1,1]:
            cube('Pickup bed rail',(-.49*r,side*.385*r,.66*r),(.70*r,.045*r,.21*r),paint,parent,.015*r)
        cube('Pickup tailgate',(-.825*r,0,.65*r),(.055*r,.77*r,.23*r),paint,parent,.015*r)


def car(name, radius, paint, glass, tire, headlight, ego=False, variant='hatchback'):
    """Original display vehicle, scaled to the recorded circular footprint."""
    if variant not in VEHICLE_MODELS:
        raise ValueError('Unknown display vehicle model: '+variant)
    parent = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(parent)
    parent['asset_style'] = STYLE
    parent['display_only'] = True
    parent['vehicle_model'] = variant
    parent['wheel_radius'] = .205*radius
    r = radius
    steel = material(name+' brushed alloy', (.42,.48,.55), .75, .22)
    trim = material(name+' black trim', (.018,.025,.035), .2, .3)
    red = material(name+' tail lights', (.72,.012,.024), .25, .24)
    # Chamfered cross-sections give the hood, shoulders and rear their own shape.
    sections = [(-.88,.33,.40),(-.74,.47,.52),(-.38,.48,.56),
                (.20,.47,.54),(.67,.43,.46),(.88,.33,.38)]
    vertices = []
    for x, w, top in sections:
        vertices += [(x*r,y*r,z*r) for y,z in
                     [(-w*.85,.22),(-w,.29),(-w,top-.045),(-w*.83,top),
                      (w*.83,top),(w,top-.045),(w,.29),(w*.85,.22)]]
    faces = [tuple(reversed(range(8))), tuple(range(40,48))]
    faces += [(8*i+j,8*(i+1)+j,8*(i+1)+(j+1)%8,8*i+(j+1)%8)
              for i in range(5) for j in range(8)]
    mesh('Sculpted body', vertices, faces, paint, parent, .025*r)
    if variant != 'hatchback':
        variant_cabin(parent,r,variant,paint,glass,trim)
    cabin = [(-.68,-.40,.51),(.30,-.40,.51),(.00,-.32,.96),(-.48,-.32,.96),
             (-.68,.40,.51),(.30,.40,.51),(.00,.32,.96),(-.48,.32,.96)]
    if variant == 'hatchback':
        mesh('Cabin and pillars', [(x*r,y*r,z*r) for x,y,z in cabin],
             [(0,1,2,3),(7,6,5,4),(0,4,5,1),(3,2,6,7),(1,5,6,2),(4,0,3,7)],
             paint, parent, .015*r)
    def panel(name, points, mat):
        return mesh(name, [(x*r,y*r,z*r) for x,y,z in points], [(0,1,2,3)], mat, parent)
    if variant == 'hatchback':
        panel('Windshield', [(.278,-.369,.548),(.278,.369,.548),(.015,.298,.932),(.015,-.298,.932)], glass)
        panel('Rear glass', [(-.666,.366,.548),(-.666,-.366,.548),(-.484,-.298,.932),(-.484,.298,.932)], glass)
    for side in [-1,1]:
        if variant == 'hatchback':
            panel('Front side window', [(.237,side*.401,.554),(-.23,side*.401,.554),
                                    (-.23,side*.326,.927),(-.02,side*.326,.927)], glass)
            panel('Rear side window', [(-.28,side*.401,.554),(-.623,side*.401,.554),
                                   (-.462,side*.326,.927),(-.28,side*.326,.927)], glass)
        cube('Side sill', (0,side*.473*r,.285*r), (1.25*r,.04*r,.065*r), trim, parent, .01*r)
        cube('Door handle', (-.21*r,side*.483*r,.49*r), (.115*r,.018*r,.025*r), steel, parent, .008*r)
        cube('Mirror stem', (.20*r,side*.47*r,.60*r), (.035*r,.13*r,.025*r), trim, parent)
        cube('Side mirror', (.18*r,side*.55*r,.62*r), (.12*r,.11*r,.075*r), paint, parent, .02*r)
        for x in [-.56,.56]:
            hub = bpy.data.objects.new('Rolling wheel', None)
            bpy.context.collection.objects.link(hub)
            hub.parent, hub.location = parent, (x*r,side*.493*r,.235*r)
            hub['rolling_wheel'] = True
            cylinder('Tire', (0,0,0), .205*r, .15*r, tire, hub, (math.pi/2,0,0))
            cylinder('Dark wheel face', (0,side*.078*r,0), .146*r, .012*r, trim, hub, (math.pi/2,0,0))
            cylinder('Alloy hub', (0,side*.09*r,0), .045*r, .025*r, steel, hub, (math.pi/2,0,0))
            for k in range(5):
                angle = k*math.tau/5
                spoke = cube('Alloy spoke', (.084*r*math.sin(angle),side*.092*r,.084*r*math.cos(angle)),
                             (.028*r,.02*r,.17*r), steel, hub, .005*r)
                spoke.rotation_euler.y = angle
        cube('Headlight', (.822*r,side*.26*r,.405*r), (.065*r,.19*r,.075*r), headlight, parent, .02*r)
        cube('Tail light', (-.82*r,side*.29*r,.415*r), (.055*r,.14*r,.09*r), red, parent, .015*r)
    cube('Front grille', (.878*r,0,.32*r), (.028*r,.39*r,.08*r), trim, parent, .01*r)
    for y in [-.12,0,.12]:
        cube('Grille blade', (.895*r,y*r,.32*r), (.012*r,.012*r,.055*r), steel, parent)
    cube('Front bumper', (.86*r,0,.255*r), (.04*r,.54*r,.035*r), trim, parent, .01*r)
    cube('Rear bumper', (-.85*r,0,.27*r), (.04*r,.57*r,.045*r), trim, parent, .01*r)
    cube('Rear plate', (-.881*r,0,.36*r), (.015*r,.17*r,.055*r), headlight, parent)
    if ego:
        # Cosmetic sensor housing; the recorded LiDAR sweep remains unchanged.
        cylinder('Display sensor base', (-.22*r,0,1.015*r), .15*r, .05*r, trim, parent)
        cylinder('Display sensor housing', (-.22*r,0,1.075*r), .115*r, .08*r, steel, parent)
        cylinder('Display sensor band', (-.22*r,0,1.087*r), .117*r, .026*r, glass, parent)
    return parent


def outside_roads(x, y, edges, margin):
    for edge in edges:
        for a,b in zip(edge['points'], edge['points'][1:]):
            dx,dy = b['x']-a['x'],b['y']-a['y']
            square = dx*dx+dy*dy
            t = min(1,max(0,((x-a['x'])*dx+(y-a['y'])*dy)/square)) if square else 0
            if math.hypot(x-a['x']-t*dx,y-a['y']-t*dy) < edge['half_width']+margin:
                return False
    return True


def sidewalk(edge, side, edges, paving, curb):
    points = edge['points']
    boundaries = []
    for i,p in enumerate(points):
        a,b = points[max(0,i-1)],points[min(len(points)-1,i+1)]
        dx,dy = b['x']-a['x'],b['y']-a['y'];length=math.hypot(dx,dy)
        if length < 1e-8:
            boundaries.append(None)
        else:
            boundaries.append([(p['x']-side*dy/length*d,p['y']+side*dx/length*d)
                               for d in [edge['half_width']+.08,edge['half_width']+.25,edge['half_width']+1.45]])
    vertices=[[],[]];faces=[[],[]];count=0
    for a,b in zip(boundaries,boundaries[1:]):
        other_edges=[other for other in edges if other is not edge]
        if a is None or b is None or not all(
                outside_roads(x,y,edges,.06) and outside_roads(x,y,other_edges,1.5)
                for x,y in a+b):
            continue
        for k,z in [(0,.17),(1,.13)]:
            corners=[a[k],b[k],b[k+1],a[k+1]]
            # Preserve upward normals on both sides of the road.
            if side < 0:corners.reverse()
            offset=len(vertices[k]);vertices[k] += [(x,y,z) for x,y in corners]
            faces[k].append(tuple(range(offset,offset+4)))
        count+=1
    objects=[]
    for k,mat in [(0,curb),(1,paving)]:
        if not faces[k]:continue
        obj=mesh('Curb' if k==0 else 'Sidewalk',vertices[k],faces[k],mat)
        # Adjacent sections share boundaries, including on curves. Welding
        # removes interior walls before thickening, avoiding overlapping slabs.
        edit=bmesh.new();edit.from_mesh(obj.data)
        bmesh.ops.remove_doubles(edit,verts=list(edit.verts),dist=1e-5)
        edit.to_mesh(obj.data);edit.free()
        bpy.context.view_layer.objects.active=obj
        solid=obj.modifiers.new('Pavement thickness','SOLIDIFY');solid.thickness=.15;solid.offset=-1
        bpy.ops.object.modifier_apply(modifier=solid.name)
        objects.append(obj)
    return count,objects


def environment(edges):
    """Deterministic scenery placed outside all authored road corridors."""
    rng = random.Random(1729)
    paving = material('Concrete pavement', (.38,.40,.37))
    curb = material('Pale curb', (.62,.64,.59))
    bark = material('Tree bark', (.15,.075,.035))
    greens = [material('Foliage '+str(i), c) for i,c in enumerate(
        [(.095,.22,.10),(.14,.29,.12),(.21,.34,.14)])]
    metal = material('Street furniture', (.075,.10,.13), .55)
    light = material('Lamp diffuser', (.87,.90,.78))
    walls = [material('Building facade '+str(i),c) for i,c in enumerate(
        [(.38,.23,.17),(.54,.52,.44),(.27,.34,.37)])]
    windows = material('Building windows', (.09,.19,.25), .25, .2)
    roof = material('Building roof', (.17,.20,.23))
    occupied = []
    counts = {'trees':0,'buildings':0,'lamps':0,'sidewalk_sections':0}
    def available(x,y,radius):
        return outside_roads(x,y,edges,radius+.5) and all(
            math.hypot(x-a,y-b)>radius+r+.8 for a,b,r in occupied)
    def tree(x,y):
        cylinder('Tree trunk', (x,y,1.6), .14, 3.2, bark)
        for dx,dy,z,scale in [(-.6,0,3.3,(1.25,1.25,1.5)),(.65,.2,3.8,(1.2,1.2,1.45)),(0,0,4.5,(1.15,1.15,1.5))]:
            bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2,radius=1,location=(x+dx,y+dy,z))
            obj=bpy.context.object;obj.name='Tree crown';obj.scale=scale
            obj.data.materials.append(rng.choice(greens))
        counts['trees']+=1;occupied.append((x,y,2.1))
    def building(x,y):
        height=rng.choice([4.2,5.4,6.6]);width=rng.choice([7,9])
        cube('Campus building',(x,y,height/2),(width,5,height),rng.choice(walls),bevel=.06)
        cube('Roof fascia',(x,y,height+.10),(width+.25,5.25,.25),roof)
        for xx in range(-2,3):
            for z in [1.6,3.3] if height<5 else [1.6,3.3,4.8]:
                cube('Facade window',(x+xx*1.25,y-2.51,z),(.8,.025,.9),windows)
                cube('Window sill',(x+xx*1.25,y-2.55,z-.48),(.93,.12,.09),curb)
        cube('Entrance',(x,y-2.53,.95),(1.1,.06,1.9),metal)
        counts['buildings']+=1;occupied.append((x,y,6))
    for edge in edges:
        points=edge['points'];s=0;next_tree=8;next_lamp=4;next_building=26
        for side in [-1,1]:
            count,_=sidewalk(edge,side,edges,paving,curb)
            counts['sidewalk_sections']+=count
        for a,b in zip(points,points[1:]):
            dx,dy=b['x']-a['x'],b['y']-a['y'];length=math.hypot(dx,dy)
            if length<1e-8:continue
            nx,ny=-dy/length,dx/length;mid=((a['x']+b['x'])/2,(a['y']+b['y'])/2)
            if s+length>=next_tree:
                x,y=mid[0]+nx*(edge['half_width']+4.5),mid[1]+ny*(edge['half_width']+4.5)
                if available(x,y,2.1):tree(x,y)
                next_tree=s+length+18
            if s+length>=next_lamp:
                x,y=mid[0]-nx*(edge['half_width']+1.8),mid[1]-ny*(edge['half_width']+1.8)
                if available(x,y,.5):
                    cylinder('Lamp post',(x,y,2.05),.055,4.1,metal)
                    cube('Lamp arm',(x,y+.36,4.08),(.075,.8,.065),metal)
                    cube('Lamp head',(x,y+.70,4.05),(.30,.48,.10),metal,bevel=.025)
                    cube('Lamp diffuser',(x,y+.70,3.99),(.23,.35,.015),light)
                    counts['lamps']+=1;occupied.append((x,y,.5))
                next_lamp=s+length+26
            if s+length>=next_building:
                x,y=(mid[0]+10*dx/length+nx*(edge['half_width']+9),
                     mid[1]+10*dy/length+ny*(edge['half_width']+9))
                if available(x,y,6):building(x,y)
                next_building=s+length+38
            s+=length
    # Geometry is cosmetic and never imported into the simulator.
    bpy.context.scene['display_asset_style']=STYLE
    bpy.context.scene['scenery_seed']=1729
    return counts


def barrel(radius, amber, white):
    obj=cylinder('Recorded obstacle barrel',(0,0,.80),radius,1.5,amber)
    for z in [-.38,.38]:
        cylinder('Reflective band',(0,0,z),radius*1.003,.13,white,obj)
    return obj
