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
    d.text((980, 31), '2D SIMULATION  /  3x', fill=TEAL, font=font(13, True))
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
    route = run['route']['points']
    center = [world(p) for p in route]
    boundaries=[]
    for side in [-1,1]:
        boundary=[]
        for i, point in enumerate(route):
            a=route[max(0,i-1)];b=route[min(len(route)-1,i+1)]
            dx=b['x']-a['x'];dy=b['y']-a['y'];length=math.hypot(dx,dy)
            boundary.append(world({'x':point['x']-dy/length*side*run['route']['half_width'],
                                   'y':point['y']+dx/length*side*run['route']['half_width']}))
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
            edge.append(world({'x':point['x']-dy/length*side*run['route']['half_width'],
                               'y':point['y']+dx/length*side*run['route']['half_width']}))
        c.line(edge, fill='#68809d', width=2, joint='curve')
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
    length=run['route']['lengths'][-1];progress=min(1,frame['progress']/length)
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
    args.output.with_suffix('.json').write_text(json.dumps({'schema_version':1,'command':f"cargo run --release --locked --bin rustdrive -- run --scenario scenarios/mission.json --seed {run['summary']['seed']} --output artifacts/demo",'summary':run['summary'],'gif_frames':len(images),'playback_speed':3},indent=2)+'\n')
    with Image.open(args.output) as image:
        assert image.n_frames==len(images) and image.size==(WIDTH,HEIGHT)
    print(f'{args.output}: {len(images)} frames, {WIDTH}x{HEIGHT}, {args.output.stat().st_size:,} bytes')


if __name__=='__main__':main()
