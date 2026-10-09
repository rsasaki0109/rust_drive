"""Original Apache-2.0 urban display assets; never sensing/collision geometry.

All dimensions are metres. Vehicle local +X faces forward and +Z faces up.
Truck cosmetics do not replace the recorded actor's physical proxy circle.
"""
import math

import bpy
from mathutils import Vector

from blender_assets import cube, cylinder, material, mesh, outside_roads, sidewalk

STYLE = 'original-urban-display-v1'
_DOG_RIGS = {}


def _root(name):
    root = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(root)
    root['display_only'] = True
    root['asset_style'] = STYLE
    return root


def truck(name, radius, paint, glass, tire, headlight):
    """Fixed SI six-wheel delivery truck; radius is physical metadata only."""
    root = _root(name)
    root['vehicle_model'] = 'truck'
    root['declared_circle_radius'] = radius
    root['display_dimensions_m'] = {'length': 7.5, 'width': 2.45, 'height': 3.3}
    root['display_dimension_scope'] = 'Body including cargo box; excludes mirrors and wheels'
    root['display_radius_m'] = 4.15
    root['display_height_m'] = 3.4
    root['wheel_radius'] = .46
    trim = material(name+' trim', (.025,.03,.035), .15, .4)
    steel = material(name+' brushed steel', (.5,.56,.6), .7, .3)
    cargo = material(name+' cargo ivory', (.75,.78,.77), .08, .5)
    red = material(name+' rear lamps', (.75,.015,.025), .15, .28)
    amber = material(name+' side lamps', (.95,.35,.025), .1, .3)
    def part(label, position, size, mat, bevel=0):
        obj = cube(label, position, size, mat, root, bevel)
        obj['vehicle_display_part'] = 'body'
        return obj
    part('Truck chassis', (0,0,.59), (7.2,1.65,.18), trim, .025)
    # Recess the shell slightly inside its rails so surface decals do not
    # overlap the shell faces. Outer rails still define the exact 2.45 m width.
    part('Truck cargo box', (-1.10,0,2.075), (5.30,2.438,2.45), cargo, .04)
    # Raised trim remains inside the calibrated outer body dimensions.
    for side in (-1,1):
        part('Cargo lower rail', (-1.10,side*1.2075,.92), (5.24,.035,.14), steel)
        part('Cargo upper rail', (-1.10,side*1.2075,3.24), (5.24,.035,.08), steel)
        part('Cargo blue livery', (-1.10,side*1.224,1.87), (4.90,.002,.18), paint)
        for x in (-3.68,-2.55,-1.42,-.29,.84,1.48):
            part('Cargo panel seam', (x,side*1.224,2.075), (.012,.002,2.20), steel)
        for x in (-3.2,-1.5,.2):
            part('Cargo amber reflector', (x,side*1.224,.93), (.10,.002,.04), amber)
    part('Truck rear door', (-3.747,0,2.06), (.006,2.24,2.23), cargo)
    for z in (1.03,1.36,1.69,2.02,2.35,2.68,3.01):
        part('Roll-up door seam', (-3.751,0,z), (.002,2.22,.012), trim)
    # Rear seam face is set inside the exact -3.75 m body endpoint.
    for obj in root.children:
        if obj.name.startswith('Roll-up door seam'):
            obj.location.x = -3.749
    part('Rear door latch', (-3.74,0,1.10), (.02,.30,.045), steel)
    part('Rear bumper', (-3.58,0,.56), (.20,2.3,.14), steel, .018)
    cabin = [(1.55,-1.08,.88),(3.75,-1.08,.88),(3.58,-1.05,2.00),(3.14,-1.04,2.83),(1.55,-1.08,2.83),
             (1.55,1.08,.88),(3.75,1.08,.88),(3.58,1.05,2.00),(3.14,1.04,2.83),(1.55,1.08,2.83)]
    obj = mesh('Truck cab', cabin, [(0,4,3,2,1),(5,6,7,8,9),(0,1,6,5),(1,2,7,6),
                                    (2,3,8,7),(3,4,9,8),(4,0,5,9)], paint, root, .045)
    obj['vehicle_display_part'] = 'body'
    def panel(label, vertices, mat):
        obj = mesh(label, vertices, [(0,1,2,3)], mat, root)
        obj['vehicle_display_part'] = 'body'
    panel('Truck windshield', [(3.565,-.94,2.04),(3.565,.94,2.04),(3.17,.94,2.77),(3.17,-.94,2.77)], glass)
    for side in (-1,1):
        panel('Truck cab side window', [(1.75,side*1.082,1.97),(3.42,side*1.062,1.97),
                                        (3.11,side*1.052,2.70),(1.75,side*1.082,2.70)], glass)
        part('Cab door handle', (1.95,side*1.105,1.79), (.19,.035,.04), steel, .008)
        part('Cab access step', (2.10,side*1.145,.77), (1.08,.16,.12), steel, .012)
        for z in (.71,.80):
            part('Cab step tread', (2.10,side*1.18,z), (.95,.035,.014), trim)
        for z in (1.94,2.63):
            mirror = cube('Truck mirror arm', (3.06,side*1.28,z), (.045,.4,.035), trim, root)
            mirror['vehicle_display_part'] = 'mirror'
        mirror = cube('Truck side mirror', (3.05,side*1.49,2.285), (.16,.12,.68), trim, root, .025)
        mirror['vehicle_display_part'] = 'mirror'
        mirror = cube('Truck mirror glass', (3.044,side*1.555,2.285), (.13,.008,.58), glass, root, .015)
        mirror['vehicle_display_part'] = 'mirror'
        part('Truck front lamp', (3.732,side*.78,1.22), (.03,.38,.20), headlight, .012)
        part('Truck front indicator', (3.732,side*.98,1.22), (.03,.10,.18), amber, .01)
        part('Truck rear lamp', (-3.735,side*.96,.79), (.03,.22,.12), red, .012)
    part('Truck front grille', (3.735,0,1.29), (.03,1.06,.45), trim, .015)
    for z in (1.16,1.28,1.40):
        part('Truck grille slat', (3.748,0,z), (.004,.97,.02), steel)
    part('Truck cab bumper', (3.70,0,.98), (.10,2.18,.17), trim, .015)
    for x in (2.55,-1.48,-2.75):
        cylinder('Truck axle', (x,0,.46), .06, 2.08, trim, root, (math.pi/2,0,0))
        for side in (-1,1):
            hub = _root('Truck rolling wheel')
            hub.parent, hub.location = root, (x,side*1.085,.46)
            hub['rolling_wheel'] = True
            cylinder('Truck tire', (0,0,0), .46, .24, tire, hub, (math.pi/2,0,0))
            cylinder('Truck wheel rim', (0,side*.123,0), .30, .016, steel, hub, (math.pi/2,0,0))
            cylinder('Truck wheel hub', (0,side*.139,0), .12, .025, trim, hub, (math.pi/2,0,0))
            for i in range(8):
                angle = i*math.tau/8
                cylinder('Truck lug bolt', (.19*math.sin(angle),side*.148,.19*math.cos(angle)),
                         .018,.014,steel,hub,(math.pi/2,0,0))
    return root


def city_environment(edges, exclusion_paths=None):
    """Deterministic original streetscape outside roads and recorded actor paths.

    Placement circles conservatively include roof/facade details. Pavement is
    display support, not a physical obstruction; buildings/lamps avoid paths.
    """
    placement_edges = list(edges)+list(exclusion_paths or [])
    occupied = []
    counts = {'trees': 0, 'buildings': 0, 'lamps': 0, 'sidewalk_sections': 0,
              'display_only': True, 'style': STYLE}
    paving = material('City sidewalk', (.47,.46,.42))
    curb = material('City curbs', (.66,.67,.64))
    trims = material('City facade frame', (.12,.15,.17), .2, .5)
    roof = material('City roof', (.24,.27,.28))
    glazing = material('City glazing', (.10,.22,.29), .2, .3)
    warm_glazing = material('City warm glazing', (.42,.43,.36), .1, .4)
    facades = [material('City facade '+str(i), rgb) for i,rgb in enumerate(
        [(.49,.48,.44),(.32,.39,.43),(.56,.37,.29),(.66,.65,.58)])]
    def available(x,y,radius):
        return outside_roads(x,y,placement_edges,radius+.5) and all(
            math.hypot(x-a,y-b) > radius+r+.8 for a,b,r in occupied)
    def building(x,y,yaw,index):
        width, depth = 11., 8.
        floors = (3,5,7,4)[index % 4]
        height = 3.2*floors
        radius = 7.2
        root = _root('Original city building')
        root.location, root.rotation_euler.z = (x,y,0), yaw
        root['placement_circle'] = {'x': x, 'y': y, 'radius_m': radius}
        root['scenery_kind'] = 'building'
        cube('City building mass', (0,0,height/2), (width,depth,height), facades[index%len(facades)], root, .045)
        cube('City roof parapet', (0,0,height+.08), (width+.18,depth+.18,.16), roof, root)
        cube('City rooftop equipment', (1.5,1.,height+.45), (2.,1.5,.75), trims, root, .035)
        cube('City rooftop service hut', (-2.,1.3,height+.8), (2.2,2.,1.5), facades[index%len(facades)], root, .04)
        for floor in range(floors):
            z = 1.85+3.2*floor
            cube('City floor belt', (0,-4.07,z-1.05), (11.1,.14,.10), curb, root)
            for column in range(5):
                xx = -4.3+2.15*column
                cube('City window frame', (xx,-4.025,z), (1.50,.065,1.90), trims, root)
                cube('City window', (xx,-4.066,z), (1.35,.02,1.74),
                     warm_glazing if (floor+column+index)%5==0 else glazing, root)
                cube('City window mullion', (xx,-4.08,z), (.045,.022,1.75), curb, root)
            for side in (-1,1):
                for yy in (-2.7,0,2.7):
                    cube('City side window', (side*5.505,yy,z), (.018,1.3,1.75), glazing, root)
        cube('City entrance frame', (0,-4.1,1.25), (2.1,.20,2.5), trims, root)
        cube('City entrance glazing', (0,-4.215,1.25), (1.9,.02,2.3), glazing, root)
        cube('City entrance canopy', (0,-4.48,2.75), (3.2,.95,.14), roof, root)
        occupied.append((x,y,radius))
        counts['buildings'] += 1
    next_distance = 4.
    for edge in edges:
        for side in (-1,1):
            sections,_ = sidewalk(edge, side, edges, paving, curb)
            counts['sidewalk_sections'] += sections
        distance = 0.
        next_distance = 8.
        for a,b in zip(edge['points'],edge['points'][1:]):
            dx,dy = b['x']-a['x'], b['y']-a['y']
            length = math.hypot(dx,dy)
            if length < 1e-8:
                continue
            nx,ny = -dy/length, dx/length
            while next_distance <= distance+length:
                fraction = (next_distance-distance)/length
                px,py = a['x']+fraction*dx, a['y']+fraction*dy
                for side in (-1,1):
                    setback = edge['half_width']+10.0
                    for shift in (0,5,10):
                        x,y = px+side*nx*(setback+shift), py+side*ny*(setback+shift)
                        if available(x,y,7.2):
                            yaw = math.atan2(dy,dx)+(math.pi if side < 0 else 0)
                            building(x,y,yaw,counts['buildings'])
                            break
                    lx,ly = px+side*nx*(edge['half_width']+1.9), py+side*ny*(edge['half_width']+1.9)
                    if available(lx,ly,.7):
                        lamp = _root('Original city lamp')
                        lamp.location = (lx,ly,0)
                        lamp['placement_circle'] = {'x': lx, 'y': ly, 'radius_m': .7}
                        lamp['scenery_kind'] = 'lamp'
                        cylinder('City lamp pole',(0,0,2.6),.065,5.2,trims,lamp)
                        cube('City lamp head',(0,0,5.16),(.7,.30,.10),roof,lamp,.025)
                        occupied.append((lx,ly,.7))
                        counts['lamps'] += 1
                next_distance += 18.
            distance += length
    bpy.context.scene['display_asset_style'] = STYLE
    return counts


def _ellipsoid(name, position, radii, mat, parent):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=20, ring_count=12, radius=1)
    obj = bpy.context.object
    obj.name, obj.parent, obj.location, obj.scale = name, parent, position, radii
    obj.data.materials.append(mat)
    for face in obj.data.polygons:
        face.use_smooth = True
    return obj


def _dog_span(obj, start, end):
    delta = Vector(end)-Vector(start)
    obj.location = (Vector(start)+Vector(end))/2
    obj.rotation_euler = delta.to_track_quat('Z','Y').to_euler()
    obj.scale.z = delta.length


def dog(name, radius, materials=None):
    """Original medium-sized dog; measured circle is metadata, not mesh scale."""
    supplied = materials or {}
    coat = supplied.get('coat') or material(name+' warm coat', (.39,.20,.07), roughness=.8)
    cream = supplied.get('cream') or material(name+' cream markings', (.76,.67,.48), roughness=.9)
    dark = supplied.get('dark') or material(name+' nose and eyes', (.018,.012,.009), roughness=.4)
    collar = supplied.get('collar') or material(name+' teal collar', (.015,.30,.29))
    root = _root(name)
    root['dog_kind'] = 'original-medium-dog'
    root['declared_circle_radius'] = radius
    root['display_dimensions_m'] = {'length': .95, 'width': .28, 'height': .65}
    root['display_dimension_scope'] = 'Approximate cosmetic dog size, not physical collision dimensions'
    root['display_radius_m'], root['display_height_m'] = .65, .72
    root['leash_anchor_local_m'] = [.19,0,.43]
    _ellipsoid('Dog torso',(-.055,0,.34),(.275,.12,.16),coat,root)
    _ellipsoid('Dog chest',(.12,0,.31),(.105,.125,.165),cream,root)
    _ellipsoid('Dog neck',(.18,0,.44),(.095,.09,.13),coat,root)
    _ellipsoid('Dog collar',(.19,0,.43),(.105,.10,.032),collar,root)
    _ellipsoid('Dog head',(.27,0,.535),(.12,.087,.105),coat,root)
    _ellipsoid('Dog muzzle',(.36,0,.495),(.085,.064,.047),cream,root)
    _ellipsoid('Dog nose',(.438,0,.506),(.022,.046,.026),dark,root)
    _ellipsoid('Dog lower jaw',(.345,0,.466),(.075,.055,.018),coat,root)
    for side in (-1,1):
        _ellipsoid('Dog floppy ear',(.21,side*.086,.53),(.052,.025,.09),coat,root)
        _ellipsoid('Dog eye',(.315,side*.075,.565),(.014,.012,.014),dark,root)
        _ellipsoid('Dog brow',(.307,side*.075,.585),(.025,.014,.009),cream,root)
    tail = cylinder('Dog tail',(0,0,0),.032,1,coat,root)
    _dog_span(tail,(-.29,0,.395),(-.46,0,.49))
    tail_tip = _ellipsoid('Dog tail tip',(-.46,0,.49),(.036,.036,.039),cream,root)
    legs = []
    for front in (False,True):
        for side in (-1,1):
            x = .13 if front else -.24
            y = side*.085
            upper = cylinder('Dog upper leg',(0,0,0),.037 if front else .048,1,coat,root)
            lower = cylinder('Dog lower leg',(0,0,0),.028,1,coat,root)
            paw = _ellipsoid('Dog paw',(x,y,.043),(.052,.042,.043),cream,root)
            legs.append((front,side,x,y,upper,lower,paw))
    _DOG_RIGS[root.name] = (tail,tail_tip,legs)
    animate_dog(root,0,0)
    return root


def animate_dog(root, time, observed_speed):
    """Estimated cosmetic gait only; never changes the recorded root pose."""
    tail, tail_tip, legs = _DOG_RIGS[root.name]
    moving = min(max(abs(observed_speed),0),2)/2
    phase = time*(5.5+2*min(abs(observed_speed),2))
    for front,side,x,y,upper,lower,paw in legs:
        leg_phase = phase+(0 if (front == (side > 0)) else math.pi)
        swing = .055*moving*math.sin(leg_phase)
        lift = .035*moving*max(math.cos(leg_phase),0)
        hip = (x,y,.34 if front else .30)
        foot = (x+swing,y,.043+lift)
        knee = (x-.025-swing*.35,y,.18+lift*.4)
        _dog_span(upper,hip,knee)
        _dog_span(lower,knee,foot)
        paw.location = foot
    wag = .045*math.sin(time*4)*moving
    _dog_span(tail,(-.29,0,.395),(-.46,wag,.49))
    tail_tip.location = (-.46,wag,.49)
