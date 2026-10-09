"""Original Apache-2.0 RustDriving family display avatars and accessories.

Human joints reuse the original frozen VRU rig. All poses are estimated display
animation; physical actors remain their recorded circles/capsules. A baby and
stroller are cosmetics belonging to one combined parent actor, not extra truth.
"""
import math

import bpy
from mathutils import Vector

from blender_assets import cube, cylinder, material, mesh
import blender_vru_assets as vru

_FAMILIES = {}


def _avatar(name,radius,kind,materials=None):
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('Physical metadata radius must be positive and finite')
    root = vru.pedestrian(name,.4,materials)
    root['avatar_kind'] = kind
    root['declared_circle_radius'] = radius
    root['asset_style'] = 'original-family-display-v1'
    root['display_only'] = True
    root['mesh_scope'] = 'Original display avatar/accessories; no semantic detection or mesh collision'
    _FAMILIES[root.as_pointer()] = {'kind':kind,'rig':vru._RIGS[root.as_pointer()]}
    return root


def elder(name,radius,materials=None):
    palette = dict(materials or {})
    palette.setdefault('hair',material(name+' silver hair',(.56,.57,.54),roughness=.9))
    palette.setdefault('jacket',material(name+' wool jacket',(.22,.27,.21),roughness=.9))
    root = _avatar(name,radius,'elder',palette)
    rig = _FAMILIES[root.as_pointer()]['rig']
    model = rig['model']
    dark = material(name+' glasses and cane grip',(.035,.027,.025),roughness=.6)
    wood = material(name+' walking cane',(.20,.085,.035),roughness=.55)
    head = next(obj for obj in rig['torso'].children if obj.type=='EMPTY' and 'head joint' in obj.name)
    for side in (-1,1):
        circle = [(.115,side*.033+.023*math.cos(a),.037+.018*math.sin(a))
                  for a in [i*math.tau/24 for i in range(24)]]
        vru._curve(name+' spectacle rim',circle,.003,dark,head,True)
        vru._curve(name+' spectacle temple',[(.113,side*.055,.038),(.012,side*.085,.035)],.003,dark,head)
        vru._curve(name+' cheek crease',[(.104,side*.046,-.016),(.097,side*.043,-.040)],.0016,wood,head)
        vru._curve(name+' forehead crease',[(.094,side*.013,.083),(.082,side*.040,.087)],.0015,wood,head)
    vru._curve(name+' spectacle bridge',[(.115,-.010,.043),(.120,0,.048),(.115,.010,.043)],.0028,dark,head)
    cane = vru._empty(name+' cane assembly',model)
    vru._span(name+' cane shaft',(.22,-.28,.025),(.22,-.28,.92),.013,wood,cane)
    vru._curve(name+' curved cane handle',[(.27+.05*math.cos(a),-.28,.92+.05*math.sin(a))
               for a in [i*math.pi/20 for i in range(21)]],.014,dark,cane)
    cylinder(name+' rubber cane tip',(.22,-.28,.020),.021,.04,dark,cane)
    root['display_height_m'],root['display_radius_m'] = 2.12,.80
    root['display_dimensions_m'] = {'height':1.87,'accessory':'walking cane'}
    _FAMILIES[root.as_pointer()]['cane'] = cane
    animate_family(root,0,0)
    return root


def child(name,radius,materials=None):
    palette = dict(materials or {})
    palette.setdefault('jacket',material(name+' orange jacket',(.80,.25,.045),roughness=.75))
    palette.setdefault('trousers',material(name+' indigo trousers',(.18,.12,.30),roughness=.8))
    palette.setdefault('hair',material(name+' chestnut hair',(.15,.060,.025),roughness=.9))
    root = _avatar(name,radius,'child',palette)
    rig = _FAMILIES[root.as_pointer()]['rig']
    rig['model'].scale = (.65,.65,.65)
    head = next(obj for obj in rig['torso'].children if obj.type=='EMPTY' and 'head joint' in obj.name)
    head.scale = (1.18,1.13,1.08)
    badge = material(name+' jacket badge',(.95,.78,.13),roughness=.7)
    vru._ellipsoid(name+' jacket badge',(.125,-.08,.26),(.008,.023,.026),badge,rig['torso'])
    root['display_height_m'],root['display_radius_m'] = 1.30,.50
    root['display_dimensions_m'] = {'height':1.17,'internal_fixed_scale':.65}
    animate_family(root,0,0)
    return root


def parent_stroller(name,radius,materials=None):
    palette = dict(materials or {})
    palette.setdefault('jacket',material(name+' plum jacket',(.40,.14,.28),roughness=.8))
    root = _avatar(name,radius,'parent_stroller',palette)
    family = _FAMILIES[root.as_pointer()]
    frame = material(name+' stroller alloy',(.33,.40,.43),.7,.3)
    fabric = material(name+' stroller blue fabric',(.075,.25,.32),roughness=.9)
    dark = material(name+' stroller trim',(.025,.032,.035),roughness=.8)
    blanket = material(name+' baby blanket',(.69,.54,.34),roughness=.95)
    skin = material(name+' baby skin',(.64,.40,.27),roughness=.85)
    for side in (-1,1):
        y = side*.28
        for label,start,end in [('longitudinal frame',(.48,y,.19),(1.29,y,.19)),
                                ('front support',(1.25,y,.19),(.76,y,.78)),
                                ('rear support',(.48,y,.19),(1.06,y,.69)),
                                ('pushbar support',(.66,y,.55),(.38,y,1.11))]:
            vru._span(name+' stroller '+label,start,end,.016,frame,root)
    vru._span(name+' stroller pushbar',(.38,-.28,1.11),(.38,.28,1.11),.024,dark,root)
    root['pushbar_contacts_local_m'] = [[.38,-.26,1.11],[.38,.26,1.11]]
    cube(name+' stroller basket',(.87,0,.29),(.56,.43,.20),dark,root,.025)
    # A sloped seat and fabric side panels make a recognizable stroller bucket.
    mesh(name+' stroller seat',[(.65,-.235,.80),(.65,.235,.80),(.76,.235,.53),(.76,-.235,.53),
                               (1.14,-.235,.57),(1.14,.235,.57)],
         [(0,1,2,3),(3,2,5,4)],fabric,root)
    for side in (-1,1):
        mesh(name+' stroller side fabric',[(.65,side*.235,.80),(.76,side*.235,.53),
             (1.14,side*.235,.57),(1.10,side*.235,.69)],[(0,1,2,3)],fabric,root)
    # Open front of the canopy leaves the original baby face visible.
    canopy = []
    for x in (.62,.87):
        for i in range(13):
            angle = math.pi*i/12
            canopy.append((x,.265*math.cos(angle),.79+.255*math.sin(angle)))
    mesh(name+' stroller curved canopy',canopy,[(i,i+1,i+14,i+13) for i in range(12)],fabric,root)
    for x in (.62,.87):
        vru._curve(name+' canopy seam',[(x,.267*math.cos(a),.79+.257*math.sin(a))
                   for a in [math.pi*i/24 for i in range(25)]],.005,frame,root)
    vru._ellipsoid(name+' bundled baby',(1.01,0,.67),(.15,.16,.065),blanket,root)
    vru._ellipsoid(name+' baby head',(.89,0,.775),(.079,.066,.083),skin,root)
    for side in (-1,1):
        vru._ellipsoid(name+' baby eye',(.954,side*.027,.793),(.006,.009,.007),dark,root)
    vru._ellipsoid(name+' baby nose',(.969,0,.774),(.012,.010,.008),skin,root)
    wheels = []
    for x in (.48,1.25):
        for side in (-1,1):
            hub = vru._empty(name+' stroller rolling wheel',root,(x,side*.315,.15))
            hub['family_rolling_wheel'] = True
            hub['display_wheel_radius_m'] = .15
            cylinder(name+' stroller tire',(0,0,0),.15,.055,dark,hub,(math.pi/2,0,0))
            cylinder(name+' stroller wheel hub',(0,side*.032,0),.057,.018,frame,hub,(math.pi/2,0,0))
            wheels.append(hub)
    family['wheels'] = wheels
    root['display_height_m'],root['display_radius_m'] = 2.12,1.65
    root['display_dimensions_m'] = {'adult_height':1.87,'group_length':1.80,'stroller_width':.69}
    root['group_scope'] = 'Single recorded parent/stroller actor; baby is cosmetic, not an independently simulated or detected actor'
    animate_family(root,0,0)
    return root


def animate_family(root,time,observed_speed):
    """Repeatable cosmetic articulation; root position/yaw/scale stay unchanged."""
    vru.animate(root,time,observed_speed)
    family = _FAMILIES[root.as_pointer()]
    rig = family['rig']
    if family['kind']=='elder':
        rig['torso'].rotation_euler.y = .08
        vru._arm(rig['limbs'][-1],(0,-.215,1.345),(.12,-.27,1.105),(.27,-.28,.97))
    elif family['kind']=='parent_stroller':
        scale = rig['model'].scale.x
        for side,limb in rig['limbs'].items():
            wrist = (.38/scale,side*.26/scale,1.11/scale)
            vru._arm(limb,(0,side*.215,1.345),(.17,side*.27,1.19),wrist)
        for wheel in family['wheels']:
            wheel.rotation_euler.y = time*abs(observed_speed)/.15
