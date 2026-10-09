"""Original Apache-2.0 pedestrian/cyclist display meshes and cosmetic articulation.

Local +X faces forward, +Y faces left, +Z points up. These meshes do not enter
RNE sensing/collision; the recorded actors retain their declared circle radii.
animate() never changes the root's recorded position, yaw, or scale.
"""
import math

import bpy
from mathutils import Vector

from blender_assets import cube, material, mesh

STYLE = 'vru-procedural-v1'
_RIGS = {}


def _empty(name, parent=None, position=(0, 0, 0)):
    obj = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(obj)
    obj.parent, obj.location = parent, position
    return obj


def _palette(name, supplied=None):
    supplied = supplied or {}
    colors = {'skin': (.55, .32, .20), 'jacket': (.04, .30, .56),
              'trousers': (.055, .07, .10), 'shoe': (.027, .03, .034),
              'hair': (.047, .021, .014), 'helmet': (.82, .88, .20),
              'frame': (.78, .075, .028), 'metal': (.50, .57, .62),
              'rubber': (.014, .017, .021), 'eye': (.75, .78, .73),
              'detail': (.028, .035, .039), 'sole': (.16, .18, .17)}
    return {key: supplied.get(key) or material(name+' '+key, rgb,
            metallic=.75 if key == 'metal' else .35 if key == 'frame' else 0,
            roughness=.26 if key in ('metal', 'frame') else .72)
            for key, rgb in colors.items()}


def _ellipsoid(name, position, radii, mat, parent):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=14, radius=1)
    obj = bpy.context.object
    obj.name, obj.parent, obj.location, obj.scale = name, parent, position, radii
    obj.data.materials.append(mat)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    return obj


def _curve(name, points, width, mat, parent, closed=False):
    data = bpy.data.curves.new(name, 'CURVE')
    data.dimensions, data.bevel_depth, data.bevel_resolution = '3D', width, 2
    spline = data.splines.new('POLY')
    spline.points.add(len(points)-1)
    for vertex, point in zip(spline.points, points):
        vertex.co = (*point, 1)
    spline.use_cyclic_u = closed
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.parent = parent
    obj.data.materials.append(mat)
    return obj


def _tube(name, radius, mat, parent, taper=1):
    # Rounded tapered cross sections, with a unit longitudinal axis for posing.
    sections = [(0, .75), (.08, 1), (.48, .98), (.92, taper), (1, taper*.72)]
    vertices = [(radius*r*math.cos(i*math.tau/16), radius*r*math.sin(i*math.tau/16), z)
                for z, r in sections for i in range(16)]
    faces = [tuple(reversed(range(16))), tuple(range(64, 80))]
    faces += [(16*j+i, 16*j+(i+1)%16, 16*(j+1)+(i+1)%16, 16*(j+1)+i)
              for j in range(4) for i in range(16)]
    obj = mesh(name, vertices, faces, mat, parent)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    return obj


def _pose_span(obj, start, end):
    start, end = Vector(start), Vector(end)
    delta = end-start
    obj.location = start
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = delta.to_track_quat('Z', 'Y')
    obj.scale.z = max(delta.length, 1e-6)


def _span(name, start, end, radius, mat, parent, taper=1):
    obj = _tube(name, radius, mat, parent, taper)
    _pose_span(obj, start, end)
    return obj


def _torus(name, center, radius, thickness, mat, parent, count=48):
    # Tire/rim lie in the bicycle's XZ plane; axle is the Y axis.
    vertices = []
    for i in range(count):
        a = i*math.tau/count
        for j in range(10):
            b = j*math.tau/10
            r = radius+thickness*math.cos(b)
            vertices.append((r*math.cos(a), thickness*math.sin(b), r*math.sin(a)))
    faces = [(10*i+(j+1)%10, 10*((i+1)%count)+(j+1)%10, 10*((i+1)%count)+j, 10*i+j)
             for i in range(count) for j in range(10)]
    obj = mesh(name, vertices, faces, mat, parent)
    obj.location = center
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    return obj


def _jacket(name, parent, colors):
    sections = [(0, .105, .14), (.10, .12, .17), (.28, .13, .20),
                (.39, .12, .22), (.45, .075, .15), (.49, .052, .065)]
    vertices = [(depth*math.cos(i*math.tau/24), width*math.sin(i*math.tau/24), z)
                for z, depth, width in sections for i in range(24)]
    faces = [tuple(reversed(range(24))), tuple(range(120, 144))]
    faces += [(24*j+i, 24*j+(i+1)%24, 24*(j+1)+(i+1)%24, 24*(j+1)+i)
              for j in range(5) for i in range(24)]
    obj = mesh(name+' tailored jacket', vertices, faces, colors['jacket'], parent)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    _curve(name+' zipper', [(sections[i][1]+.003, 0, sections[i][0]) for i in range(1, 6)],
           .003, colors['detail'], parent)
    for side in (-1, 1):
        _curve(name+' pocket seam', [(.112, side*.08, .095), (.122, side*.15, .165)],
               .003, colors['detail'], parent)
        _curve(name+' shoulder seam', [(0, side*.203, .39), (.09, side*.16, .415)],
               .002, colors['detail'], parent)
    _ellipsoid(name+' jacket collar', (0, 0, .47), (.063, .08, .028), colors['jacket'], parent)


def _head(name, parent, position, colors, helmet=False):
    root = _empty(name+' head joint', parent, position)
    _ellipsoid(name+' natural head', (0, 0, 0), (.103, .081, .133), colors['skin'], root)
    _ellipsoid(name+' chin', (.042, 0, -.095), (.061, .059, .047), colors['skin'], root)
    _ellipsoid(name+' nose bridge', (.098, 0, .015), (.025, .014, .041), colors['skin'], root)
    _ellipsoid(name+' nose tip', (.119, 0, -.004), (.020, .022, .016), colors['skin'], root)
    for side in (-1, 1):
        _ellipsoid(name+' nostril', (.126, side*.012, -.013), (.004, .005, .003), colors['detail'], root)
        _ellipsoid(name+' ear', (-.012, side*.079, -.008), (.025, .017, .037), colors['skin'], root)
        _ellipsoid(name+' eye', (.094, side*.033, .037), (.008, .018, .010), colors['eye'], root)
        _ellipsoid(name+' iris', (.102, side*.030, .037), (.003, .007, .007), colors['hair'], root)
        _curve(name+' eyebrow', [(.094, side*.015, .057), (.092, side*.035, .063),
                                 (.083, side*.049, .055)], .004, colors['hair'], root)
    _curve(name+' mouth', [(.090, -.027, -.060), (.096, 0, -.066), (.090, .027, -.060)],
           .0025, colors['detail'], root)
    # A varying scalp boundary exposes the forehead and covers the back/temples.
    vertices = []
    for j in range(10):
        for i in range(32):
            phi = i*math.tau/32
            theta = .03+(j/9)*(1.54-.32*math.cos(phi))
            vertices.append((.108*math.sin(theta)*math.cos(phi),
                             .086*math.sin(theta)*math.sin(phi), .138*math.cos(theta)))
    faces = [(32*(j+1)+i, 32*(j+1)+(i+1)%32, 32*j+(i+1)%32, 32*j+i)
             for j in range(9) for i in range(32)]
    scalp = mesh(name+' sculpted hair cap', vertices, faces, colors['hair'], root)
    for polygon in scalp.data.polygons:
        polygon.use_smooth = True
    if helmet:
        _ellipsoid(name+' cycle helmet shell', (-.008, 0, .083), (.127, .105, .084), colors['helmet'], root)
        for side in (-1, 1):
            for k in range(3):
                y = side*(.027+k*.024)
                _curve(name+' helmet ventilation inset', [(-.082, y, .124), (-.025, y, .160),
                    (.044, y, .151), (.089, y, .116)], .009, colors['detail'], root)
            _curve(name+' helmet chin strap', [(.027, side*.094, .064), (.018, side*.077, -.055),
                (.051, side*.028, -.123)], .006, colors['detail'], root)
        _ellipsoid(name+' helmet visor', (.099, 0, .071), (.064, .087, .012), colors['helmet'], root)
    return root


def _shoe(name, parent, colors):
    root = _empty(name, parent)
    _ellipsoid(name+' rounded upper', (.045, 0, 0), (.135, .055, .052), colors['shoe'], root)
    _ellipsoid(name+' outsole', (.047, 0, -.035), (.140, .058, .018), colors['sole'], root)
    for x in (.003, .03, .055):
        _curve(name+' lace', [(x-.008, -.032, .039), (x+.008, .032, .039)], .0025, colors['sole'], root)
    return root


def _limbs(name, parent, colors):
    limbs = {}
    for side in (-1, 1):
        limbs[side] = {
            'thigh': _tube(name+' trouser thigh', .081, colors['trousers'], parent, .85),
            'calf': _tube(name+' trouser calf', .062, colors['trousers'], parent, .70),
            'knee': _ellipsoid(name+' knee joint', (0, 0, 0), (.075, .066, .076), colors['trousers'], parent),
            'foot': _shoe(name+' shoe', parent, colors),
            'upper_arm': _tube(name+' jacket sleeve', .061, colors['jacket'], parent, .84),
            'forearm': _tube(name+' sleeve forearm', .046, colors['jacket'], parent, .76),
            'elbow': _ellipsoid(name+' sleeve elbow', (0, 0, 0), (.053, .052, .057), colors['jacket'], parent),
            'hand': _ellipsoid(name+' hand', (0, 0, 0), (.029, .024, .051), colors['skin'], parent),
        }
    return limbs


def _root(name, radius, kind):
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('display actor radius must be positive and finite')
    root = _empty(name)
    root['asset_style'], root['display_only'], root['vru_kind'] = STYLE, True, kind
    root['declared_circle_radius'] = radius
    root['mesh_scope'] = 'Original cosmetic mesh; RNE sensing/collision retain declared actor circles'
    model = _empty(name+' display mesh frame', root)
    # The display size follows the recorded footprint only as a visual cue.
    scale = max(.75, min(1.30, radius/(.38 if kind == 'pedestrian' else .95)))
    model.scale = (scale, scale, scale)
    root['display_height_m'] = (2.0 if kind == 'pedestrian' else 1.90)*scale
    root['display_radius_m'] = (.62 if kind == 'pedestrian' else 1.12)*scale
    return root, model, scale


def pedestrian(name, radius, materials=None):
    """Return a human display root; optional palette values are Blender materials."""
    root, model, scale = _root(name, radius, 'pedestrian')
    colors = _palette(name, materials)
    torso = _empty(name+' articulated torso', model, (0, 0, .96))
    _jacket(name, torso, colors)
    _ellipsoid(name+' trouser pelvis', (0, 0, .945), (.115, .145, .12), colors['trousers'], model)
    _span(name+' neck', (0, 0, 1.42), (0, 0, 1.53), .036, colors['skin'], model)
    _head(name, torso, (0, 0, .665), colors)
    rig = {'kind': 'pedestrian', 'model': model, 'scale': scale, 'torso': torso,
           'limbs': _limbs(name, model, colors)}
    _RIGS[root.as_pointer()] = rig
    animate(root, 0., 0.)
    return root


def _wheel(name, model, x, colors):
    wheel = _empty(name+' wheel axle', model, (x, 0, .35))
    wheel['cosmetic_rolling_wheel'] = True
    _torus(name+' inflated tire', (0, 0, 0), .323, .027, colors['rubber'], wheel)
    _torus(name+' alloy rim', (0, 0, 0), .299, .011, colors['metal'], wheel)
    for side in (-1, 1):
        _torus(name+' tire sidewall seam', (0, side*.021, 0), .322, .002, colors['detail'], wheel)
    _span(name+' hub', (0, -.043, 0), (0, .043, 0), .027, colors['metal'], wheel)
    for k in range(32):
        angle = k*math.tau/32
        side = -1 if k % 2 else 1
        crossing = angle+side*math.tau*2/32
        _span(name+' crossed spoke', (.025*math.cos(crossing), side*.022, .025*math.sin(crossing)),
              (.296*math.cos(angle), side*.006, .296*math.sin(angle)), .0016, colors['metal'], wheel)
    wheel['spoke_count'] = 32
    return wheel


def cyclist(name, radius, materials=None):
    """Return a helmeted rider and a detailed original diamond-frame bicycle."""
    root, model, scale = _root(name, radius, 'cyclist')
    colors = _palette(name, materials)
    rear, front, bracket = (-.55, 0, .35), (.57, 0, .35), (-.09, 0, .35)
    seat, head_top, head_bottom = (-.22, 0, .85), (.38, 0, .88), (.445, 0, .68)
    for label, a, b, width in [('seat tube', bracket, seat, .022), ('top tube', seat, head_top, .021),
                             ('down tube', bracket, head_bottom, .026), ('head tube', head_bottom, head_top, .025)]:
        _span(name+' '+label, a, b, width, colors['frame'], model)
    for side in (-1, 1):
        axle = (rear[0], side*.045, rear[2])
        _span(name+' chainstay', (-.09, side*.032, .35), axle, .014, colors['frame'], model)
        _span(name+' seatstay', (-.22, side*.018, .85), axle, .012, colors['frame'], model)
        _curve(name+' curved fork', [(head_bottom[0], side*.029, head_bottom[2]),
            (.48, side*.04, .49), (front[0], side*.045, front[2])], .016, colors['frame'], model)
    _span(name+' seat post', seat, (-.245, 0, .97), .016, colors['metal'], model)
    _ellipsoid(name+' saddle rear', (-.26, 0, .975), (.12, .083, .023), colors['rubber'], model)
    _ellipsoid(name+' saddle nose', (-.15, 0, .968), (.087, .036, .019), colors['rubber'], model)
    _curve(name+' stem', [head_top, (.42, 0, 1.025), (.55, 0, 1.07)], .016, colors['metal'], model)
    _curve(name+' swept handlebar', [(.48, -.28, 1.07), (.53, -.18, 1.07), (.55, 0, 1.07),
        (.53, .18, 1.07), (.48, .28, 1.07)], .013, colors['metal'], model)
    for side in (-1, 1):
        _span(name+' handlebar grip', (.48, side*.21, 1.07), (.48, side*.29, 1.07), .022, colors['rubber'], model)
        _curve(name+' brake lever', [(.51, side*.19, 1.06), (.56, side*.245, 1.035),
            (.54, side*.27, 1.03)], .005, colors['metal'], model)
        _curve(name+' brake cable', [(.53, side*.18, 1.05), (.60, side*.10, .91),
            (.47, side*.02, .71)], .002, colors['detail'], model)
    wheels = [_wheel(name+' rear', model, rear[0], colors), _wheel(name+' front', model, front[0], colors)]
    crank = _empty(name+' crank spindle', model, bracket)
    _span(name+' bottom bracket axle', (0, -.13, 0), (0, .13, 0), .023, colors['metal'], crank)
    _torus(name+' outer chainring', (0, -.10, 0), .095, .008, colors['metal'], crank, 40)
    _torus(name+' inner chainring', (0, -.115, 0), .067, .006, colors['metal'], crank, 32)
    for k in range(32):
        angle = k*math.tau/32
        tooth = cube(name+' chainring tooth', (.102*math.sin(angle), -.10, .102*math.cos(angle)),
                     (.009, .016, .013), colors['metal'], crank, .001)
        tooth.rotation_euler.y = angle
    for side in (-1, 1):
        _span(name+' crank arm', (0, side*.115, 0), (0, side*.115, side*.165), .012, colors['metal'], crank)
    _torus(name+' rear sprocket', (rear[0], -.10, rear[2]), .05, .007, colors['metal'], model, 32)
    chain = [(-.09+.10*math.cos(a), -.107, .35+.10*math.sin(a))
             for a in [-math.pi/2+i*math.pi/16 for i in range(17)]]
    chain += [(-.55+.052*math.cos(a), -.107, .35+.052*math.sin(a))
              for a in [math.pi/2+i*math.pi/16 for i in range(17)]]
    _curve(name+' visible drive chain', chain, .004, colors['detail'], model, True)
    for k in range(0, len(chain), 2):
        _ellipsoid(name+' chain link glint', chain[k], (.006, .005, .004), colors['metal'], model)
    # Rider body leans forward, with articulated knees following actual pedals.
    torso = _empty(name+' forward leaning rider', model, (-.20, 0, 1.02))
    torso.rotation_euler.y = .59
    _jacket(name, torso, colors)
    _ellipsoid(name+' rider pelvis', (-.22, 0, 1.015), (.105, .12, .075), colors['trousers'], model)
    _span(name+' rider neck', (.075, 0, 1.415), (.15, 0, 1.49), .034, colors['skin'], model)
    _head(name, model, (.205, 0, 1.535), colors, helmet=True)
    limbs = _limbs(name, model, colors)
    pedals = {side: cube(name+' pedal platform', (0, 0, 0), (.095, .075, .020),
                         colors['rubber'], model, .008) for side in (-1, 1)}
    root['wheel_radius'] = .35*scale
    _RIGS[root.as_pointer()] = {'kind': 'cyclist', 'model': model, 'scale': scale,
        'torso': torso, 'limbs': limbs, 'wheels': wheels, 'crank': crank, 'pedals': pedals}
    animate(root, 0., 0.)
    return root


def _leg(limb, hip, knee, ankle):
    _pose_span(limb['thigh'], hip, knee)
    _pose_span(limb['calf'], knee, ankle)
    limb['knee'].location = knee
    limb['foot'].location = ankle


def _arm(limb, shoulder, elbow, wrist):
    _pose_span(limb['upper_arm'], shoulder, elbow)
    _pose_span(limb['forearm'], elbow, wrist)
    limb['elbow'].location = elbow
    limb['hand'].location = wrist


def _cycle_knee(hip, ankle):
    delta = Vector(ankle)-Vector(hip)
    distance = max(delta.length, 1e-6)
    a = (.45**2-.44**2+distance**2)/(2*distance)
    height = math.sqrt(max(.45**2-a*a, 0.))
    normal = Vector((-delta.z, 0, delta.x)).normalized()
    return Vector(hip)+delta*(a/distance)+normal*height


def animate(root, time, observed_speed):
    """Pose display joints from recorded time/speed; never move the actor root.

    Pedal cadence and gait are cosmetic estimates, not measured limb trajectories.
    Stored snapshots need no simulator step or accumulated animation state.
    """
    if not math.isfinite(time) or not math.isfinite(observed_speed):
        raise ValueError('display animation requires finite observed time/speed')
    rig = _RIGS.get(root.as_pointer())
    if rig is None:
        raise ValueError('VRU root was not built by this display module')
    speed = min(abs(observed_speed), 12.)
    if rig['kind'] == 'pedestrian':
        amount = min(speed/1.3, 1.)
        phase = time*math.tau*(.8+.38*speed) if speed > .02 else 0.
        bob = .018*amount*abs(math.sin(phase))
        rig['torso'].location.z = .96+bob
        for side, limb in rig['limbs'].items():
            angle = phase+(math.pi if side < 0 else 0.)
            stride = -.29*amount*math.cos(angle)
            lift = .115*amount*max(0., math.sin(angle))
            hip = (0, side*.105, .93+bob)
            knee = (stride*.50+.035*amount, side*.115, .50+bob*.5+lift*.5)
            ankle = (stride, side*.12, .085+lift)
            _leg(limb, hip, knee, ankle)
            limb['foot'].rotation_euler.y = -.12*amount*math.sin(angle)
            swing = -.20*amount*math.cos(angle)
            _arm(limb, (0, side*.215, 1.345+bob), (swing*.5, side*.245, 1.11+bob),
                 (swing+.018, side*.25, .92+bob))
    else:
        cadence = min(1.6, speed/(.35*rig['scale']*math.tau*2.8))
        phase = time*math.tau*cadence if speed > .02 else .55
        rig['crank'].rotation_euler.y = phase
        for wheel in rig['wheels']:
            wheel.rotation_euler.y = time*speed/(.35*rig['scale'])
        for side, limb in rig['limbs'].items():
            angle = phase+(math.pi if side < 0 else 0.)
            pedal = (-.09+.165*math.sin(angle), side*.145, .35+.165*math.cos(angle))
            rig['pedals'][side].location = pedal
            ankle = (pedal[0]-.028, side*.145, pedal[2]+.057)
            hip = (-.22, side*.083, 1.02)
            _leg(limb, hip, _cycle_knee(hip, ankle), ankle)
            limb['foot'].rotation_euler.y = .08
            _arm(limb, (.075, side*.185, 1.345), (.255, side*.235, 1.17),
                 (.48, side*.255, 1.09))
