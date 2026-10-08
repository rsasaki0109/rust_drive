#!/usr/bin/env python3
"""Render real RustDrive telemetry; never synthesize trajectories or success metrics."""
import argparse
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT = 1200, 720
BG = '#0a1220'
PANEL = '#101d2e'
MUTED = '#8698b3'
TEXT = '#edf4ff'
TEAL = '#46e3c2'
BLUE = '#5699ff'
ORANGE = '#ffb15c'
PURPLE = '#be96ff'


def font(size, bold=False):
    filename = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    for path in [Path('/usr/share/fonts/truetype/dejavu') / filename, Path(filename)]:
        try:
            return ImageFont.truetype(str(path), size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)


def xy(p):
    return p['x'], p['y']


def render(run, frame, index):
    im = Image.new('RGB', (WIDTH, HEIGHT), BG)
    d = ImageDraw.Draw(im)
    d.text((28, 16), 'RustDrive', fill=TEXT, font=font(34, True))
    d.text((232, 31), 'A RUST-NATIVE AUTONOMOUS DRIVING STACK', fill=MUTED, font=font(13, True))
    d.rounded_rectangle((963, 23, 1172, 57), 16, fill='#19352f')
    badge = 'RNE CPU  /  3x' if run.get('backend', '').startswith('rne-') else '2D SIMULATION  /  3x'
    d.text((980, 31), badge, fill=TEAL, font=font(13, True))
    d.rounded_rectangle((24, 82, 862, 510), 16, fill=PANEL)
    # Render to a separate layer so off-screen geometry cannot overwrite the HUD.
    canvas = Image.new('RGB', (838, 428), PANEL)
    c = ImageDraw.Draw(canvas)
    ex, ey = xy(frame['truth']['pose']['position'])
    yaw = frame['truth']['pose']['yaw']
    scale = 11.3
    def world(p):
        x, y = xy(p)
        return (170 + (x-ex)*scale, 220-(y-ey)*scale)
    for x in range(-100, 1200, 113):
        offset = int(ex*scale) % 113
        c.line([(x-offset, 0), (x-offset, 428)], fill='#172739')
    for y in range(0, 428, 56):
        c.line([(0, y), (838, y)], fill='#172739')
    selected_route = run['route']
    for change in run.get('route_history', []):
        if change['time'] <= frame['time']:
            selected_route = change['plan']['route']
    route = selected_route['points']
    navigation = run.get('navigation')
    if navigation:
        nav = run['scenario']['navigation']
        closed_edges = (frame.get('navigation') or nav)['closed_edges']
        for edge in nav['network']['edges']:
            c.line([world(p) for p in edge['points']], fill='#182638',
                   width=int(2*edge['half_width']*scale), joint='curve')
    center = [world(p) for p in route]
    boundaries=[]
    for side in [-1,1]:
        boundary=[]
        for i, point in enumerate(route):
            a=route[max(0,i-1)];b=route[min(len(route)-1,i+1)]
            dx=b['x']-a['x'];dy=b['y']-a['y'];length=math.hypot(dx,dy)
            boundary.append(world({'x':point['x']-dy/length*side*selected_route['half_width'],
                                   'y':point['y']+dx/length*side*selected_route['half_width']}))
        boundaries.append(boundary)
    c.polygon(boundaries[0]+list(reversed(boundaries[1])),fill='#1b2b3f')
    for i in range(0, len(center)-2, 6):
        c.line(center[i:i+3], fill='#52647d', width=2)
    # Road boundary polylines, derived from the actual route.
    for side in [-1, 1]:
        edge=[]
        for i, point in enumerate(route):
            a=route[max(0, i-1)]; b=route[min(len(route)-1, i+1)]
            dx=b['x']-a['x']; dy=b['y']-a['y']; length=math.hypot(dx, dy)
            edge.append(world({'x':point['x']-dy/length*side*selected_route['half_width'],
                               'y':point['y']+dx/length*side*selected_route['half_width']}))
        c.line(edge, fill='#68809d', width=2, joint='curve')
    if navigation:
        for edge in nav['network']['edges']:
            if edge['id'] in closed_edges:
                geometry = [world(p) for p in edge['points']]
                for j in range(0, len(geometry)-1, 4):
                    c.line(geometry[j:j+2], fill=ORANGE, width=3)
                cx,cy=geometry[len(geometry)//2]
                c.text((cx-28,cy+8),'CLOSED',fill=ORANGE,font=font(11,True))
    for f in run['frames'][max(0,index-100):index:3]:
        p=world(f['truth']['pose']['position']);c.ellipse((p[0]-1,p[1]-1,p[0]+1,p[1]+1), fill=BLUE)
    trajectory = [world(p['position']) for p in frame['trajectory']['points']]
    if len(trajectory)>1:
        c.line(trajectory, fill='#1c6c67', width=9, joint='curve')
        c.line(trajectory, fill=TEAL, width=3, joint='curve')
    for prediction in frame['predictions']:
        for p in prediction['positions'][::4]:
            px,py=world(p);c.ellipse((px-2,py-2,px+2,py+2), fill=PURPLE)
    pose=frame['estimate']['pose']; cosine=math.cos(pose['yaw']); sine=math.sin(pose['yaw'])
    for p in frame['lidar'][::2]:
        hit={'x':pose['position']['x']+p['x']*cosine-p['y']*sine,
             'y':pose['position']['y']+p['x']*sine+p['y']*cosine}
        px,py=world(hit)
        if 0<=px<838 and 0<=py<428:
            c.line((170,220,px,py), fill='#234447',width=1)
            c.ellipse((px-2,py-2,px+2,py+2),fill=TEAL)
    for obj in frame['objects']:
        px,py=world(obj['position']);r=obj['radius']*scale
        c.ellipse((px-r,py-r,px+r,py+r), fill='#99612f', outline=ORANGE, width=2)
    for track in frame['tracks']:
        px,py=world(track['position']);r=track['radius']*scale
        c.ellipse((px-r,py-r,px+r,py+r), outline=PURPLE,width=2)
        c.text((px+r+3,py-r-6),f"T{track['id']}",fill=PURPLE,font=font(11,True))
    px,py=world(frame['truth']['pose']['position']);r=run['vehicle']['radius']*scale
    c.ellipse((px-r,py-r,px+r,py+r), outline='#345880',width=2)
    corners=[]
    for x,y in [(1.15,.65),(1.15,-.65),(-1.15,-.65),(-1.15,.65)]:
        corners.append((px+(x*math.cos(yaw)-y*math.sin(yaw))*scale,
                        py-(x*math.sin(yaw)+y*math.cos(yaw))*scale))
    c.polygon(corners,fill=BLUE,outline=TEXT)
    c.line((px,py,px+20*math.cos(yaw),py-20*math.sin(yaw)), fill=TEXT,width=2)
    c.text((18,16),'LIVE WORLD  /  SENSOR-DRIVEN CLOSED LOOP',fill=MUTED,font=font(12,True))
    c.text((18,401),'10 m',fill=MUTED,font=font(11))
    c.line((66,409,66+113,409),fill=MUTED,width=2)
    if navigation:
        # The inset displays supplied map topology, closures and selected route.
        # Its moving ego marker comes from recorded truth solely for visualization.
        c.rounded_rectangle((526,266,820,410),10,fill='#0d1928',outline='#34506b')
        phase=(frame.get('navigation') or {}).get('phase','Following')
        c.text((540,275),f"TO {nav['goal'].upper()} / {phase.upper()}",fill=TEAL,font=font(12,True))
        positions=[node['position'] for node in nav['network']['nodes']]
        positions += [p for edge in nav['network']['edges'] for p in edge['points']]
        xmin=min(p['x'] for p in positions);xmax=max(p['x'] for p in positions)
        ymin=min(p['y'] for p in positions);ymax=max(p['y'] for p in positions)
        factor=min(256/max(1,xmax-xmin),74/max(1,ymax-ymin))
        def mini(p):
            return 546+(p['x']-xmin)*factor,381-(p['y']-ymin)*factor
        for edge in nav['network']['edges']:
            color=ORANGE if edge['id'] in closed_edges else '#40536d'
            c.line([mini(p) for p in edge['points']],fill=color,width=2)
        c.line([mini(p) for p in route],fill=TEAL,width=3)
        for edge in nav['network']['edges']:
            if edge['id'] in (frame.get('navigation') or {}).get('pending_edges', []):
                pending=[mini(p) for p in edge['points']]
                for j in range(0,len(pending)-1,4):
                    c.line(pending[j:j+2],fill=PURPLE,width=2)
            if edge['id'] in closed_edges:
                c.line([mini(p) for p in edge['points']],fill=ORANGE,width=2)
        for node in nav['network']['nodes']:
            mx,my=mini(node['position'])
            color=ORANGE if node['id']==nav['goal'] else MUTED
            c.ellipse((mx-3,my-3,mx+3,my+3),fill=color)
        mx,my=mini(frame['truth']['pose']['position'])
        c.ellipse((mx-4,my-4,mx+4,my+4),fill=BLUE,outline=TEXT)
        closure=', '.join(closed_edges) or 'none'
        c.text((540,392),f'Known closures: {closure}',fill=MUTED,font=font(10))
    im.paste(canvas,(24,82))
    d=ImageDraw.Draw(im)
    d.rounded_rectangle((882,82,1176,510),16,fill=PANEL)
    mode=frame['trajectory']['mode'].upper()
    color=ORANGE if mode in ['YIELD','EMERGENCY'] else TEAL
    d.text((904,100),'DRIVING STATE',fill=MUTED,font=font(12,True))
    d.text((904,122),mode,fill=color,font=font(27,True))
    metrics=[('SPEED',f"{frame['truth']['speed']*3.6:04.1f}",'km/h'),
             ('SIMULATION TIME',f"{frame['time']:04.1f}",'s'),
             ('OBSTACLE CLEARANCE',f"{frame['clearance']:.2f}" if frame['clearance']<100 else '--','m'),
             ('LOCALIZATION ERROR',f"{math.dist(xy(frame['truth']['pose']['position']),xy(frame['estimate']['pose']['position'])):.3f}",'m')]
    for i,(label,value,unit) in enumerate(metrics):
        y=174+i*72
        d.text((904,y),label,fill=MUTED,font=font(11,True))
        d.text((904,y+18),value,fill=TEXT,font=font(29,True))
        d.text((1076,y+32),unit,fill=MUTED,font=font(13))
    d.rounded_rectangle((24,528,1176,622),14,fill=PANEL)
    d.text((44,540),'MISSION PROGRESS',fill=MUTED,font=font(11,True))
    length=selected_route['lengths'][-1];progress=min(1,frame['progress']/length)
    d.text((994,540),f"{frame['progress']:.0f} / {length:.0f} m",fill=TEXT,font=font(13,True))
    d.rounded_rectangle((44,574,1156,581),3,fill='#2b4058')
    d.rounded_rectangle((44,574,max(47,44+1112*progress),581),3,fill=TEAL)
    for obj in run['scenario']['objects']:
        x=44+1112*obj['s']/length
        d.ellipse((x-4,572,x+4,583),fill=ORANGE)
    d.text((44,595),'BLUE  ego     TEAL  LiDAR / plan     PURPLE  tracks / prediction     AMBER  simulator truth',fill=MUTED,font=font(12))
    stages=['LOCALIZE','PERCEIVE','MAP','PREDICT','PLAN','CONTROL']
    for i,stage in enumerate(stages):
        x=24+i*196
        d.rounded_rectangle((x,640,x+180,677),8,fill='#15293a')
        d.ellipse((x+12,654,x+18,660),fill=TEAL)
        d.text((x+29,651),stage,fill=TEXT,font=font(12,True))
        if i<5:d.text((x+184,649),'›',fill=MUTED,font=font(18))
    d.text((26,695),f"Seed {run['summary']['seed']}  •  20 Hz vehicle/control  •  10 Hz LiDAR  •  sensor observations drive every decision",fill=MUTED,font=font(11))
    return im


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,default=Path('assets/demo.gif'))
    args=parser.parse_args()
    run=json.loads(args.run.read_text())
    if run.get('schema_version')!=1 or not run['summary']['passed']:
        raise SystemExit('Refusing to render a success demo from an unsupported or failing run')
    frames=run['frames'];indices=list(range(0,len(frames),3))
    if indices[-1]!=len(frames)-1:indices.append(len(frames)-1)
    images=[render(run,frames[i],i) for i in indices]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    images[0].save(args.output,save_all=True,append_images=images[1:],duration=[100]*(len(images)-1)+[1400],loop=0,optimize=False)
    preview_index=next((j for j,i in enumerate(indices) if frames[i]['time']>=7.5),len(images)//3)
    images[preview_index].save(args.output.with_suffix('.png'))
    backend=run.get('backend', 'reference-bicycle')
    provenance={'schema_version':1,'backend':backend,'input_trace':str(args.run),
                'scenario':run['scenario'],'summary':run['summary'],
                'gif_frames':len(images),'playback_speed':3,
                'renderer_command':f'python3 scripts/render_demo.py {args.run} --output {args.output}'}
    if run.get('navigation'):
        provenance['navigation']=run['navigation']
    if run.get('route_history'):
        provenance['route_history']=run['route_history']
    # Match the full normalized configuration, never infer a recipe from the name.
    scenario_dir=Path(__file__).resolve().parent.parent/'scenarios'
    for scenario_path in sorted(scenario_dir.glob('*.json')):
        scenario=json.loads(scenario_path.read_text())
        scenario.setdefault('curve_amplitude',0)
        scenario.setdefault('lidar_dropout',None)
        scenario.setdefault('gnss_dropout',None)
        for obj in scenario['objects']:
            for key in ['speed','lateral_speed','active_from','moving_from']:
                obj.setdefault(key,0)
        if run['scenario']!=scenario:
            continue
        seed=run['summary']['seed']
        source_path=f'scenarios/{scenario_path.name}'
        if backend.startswith('rne-'):
            plant='dynamic' if backend.startswith('rne-dynamic') else 'kinematic'
            provenance['rne_expected_revision']=(scenario_dir.parent/'integrations/rne/rne-revision.txt').read_text().strip()
            provenance['command']=f'cargo +1.95.0 run --release --locked --manifest-path integrations/rne/Cargo.toml -- --scenario {source_path} --plant {plant} --seed {seed} --output {args.run.parent}'
        else:
            provenance['command']=f'cargo run --release --locked --bin rustdrive -- run --scenario {source_path} --seed {seed} --output {args.run.parent}'
        break
    args.output.with_suffix('.json').write_text(json.dumps(provenance,indent=2)+'\n')
    with Image.open(args.output) as image:
        assert image.n_frames==len(images) and image.size==(WIDTH,HEIGHT)
    print(f'{args.output}: {len(images)} frames, {WIDTH}x{HEIGHT}, {args.output.stat().st_size:,} bytes')


if __name__=='__main__':main()
