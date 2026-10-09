"""Render verified RNE telemetry with Blender Cycles CPU and package a 3D GIF."""
import argparse
import hashlib
import json
import math
import shlex
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFilter
from render_demo import font

ROOT = Path(__file__).resolve().parent.parent


def encode_gif(images, output, durations):
    """Use a shared palette and mild spatial noise reduction for README size."""
    prepared = []
    for image in images:
        image = image.copy()
        image.paste(image.crop((0, 54, 960, 594)).filter(ImageFilter.MedianFilter(3)), (0, 54))
        prepared.append(image)
    samples = prepared[::max(1, len(prepared)//16)]
    palette_sample = Image.new('RGB', (480, 160*len(samples)))
    saturated = []
    for i, sample in enumerate(samples):
        thumb = sample.resize((240, 160))
        palette_sample.paste(thumb, (0, 160*i))
        saturated.extend(rgb for rgb in thumb.getdata() if max(rgb)-min(rgb) > .45*max(rgb))
    # Reserve color capacity for small cars, path overlays and HUD text instead
    # of letting the large ground/asphalt regions dominate the shared palette.
    if saturated:
        swatches = Image.new('RGB', (240, 160*len(samples)))
        swatches.putdata([saturated[i % len(saturated)] for i in range(swatches.width*swatches.height)])
        palette_sample.paste(swatches, (240, 0))
    palette = palette_sample.quantize(colors=192)
    encoded = [image.quantize(palette=palette, dither=Image.Dither.NONE) for image in prepared]
    encoded[0].save(output, save_all=True, append_images=encoded[1:], duration=durations, loop=0, optimize=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, default=Path('assets/rne-3d-demo.gif'))
    parser.add_argument('--scene-output', type=Path, help='Save an editable Blender scene at the last rendered frame')
    parser.add_argument('--traffic-models', nargs='+', choices=['hatchback','sedan','van','pickup'], default=['hatchback'], help='Display models assigned in stable actor appearance order')
    parser.add_argument('--camera', choices=['ego','traffic'], default='ego', help='Follow ego or frame ego and active reactive vehicles')
    parser.add_argument('--preview-time', type=float, help='Render one PNG instead of the complete GIF')
    parser.add_argument('--samples', type=int, default=16)
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.samples <= 128 or not 1 <= args.threads <= 64:
        parser.error('samples must be 1–128 and threads 1–64')
    if args.preview_time is not None and not math.isfinite(args.preview_time):
        parser.error('preview time must be finite')
    if args.scene_output:
        if args.scene_output.suffix != '.blend':
            parser.error('scene output must use the .blend extension')
        args.scene_output.parent.mkdir(parents=True, exist_ok=True)
    run = json.loads(args.run.read_text())
    if run.get('schema_version') != 1 or not run.get('backend', '').startswith('rne-') or not run['summary']['passed']:
        raise SystemExit('A successful schema-1 actual RNE recording is required')
    frames = run['frames']
    indices = [0]
    for i in range(1, len(frames)):
        if frames[i]['time']-frames[indices[-1]]['time'] >= .3-1e-9:
            indices.append(i)
    if indices[-1] != len(frames)-1:
        indices.append(len(frames)-1)
    if args.preview_time is not None:
        indices = [min(range(len(frames)), key=lambda i: abs(frames[i]['time']-args.preview_time))]
    cache = ROOT/'artifacts/3d'
    cache.mkdir(parents=True, exist_ok=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='capture-', dir=cache) as temporary:
        directory = Path(temporary)
        request = {'run': str(args.run.resolve()), 'indices': indices,
                   'frames_directory': temporary, 'samples': args.samples,
                   'scene_output': str(args.scene_output.resolve()) if args.scene_output else None,
                   'traffic_models': args.traffic_models, 'camera': args.camera}
        request_file = directory/'request.json'
        request_file.write_text(json.dumps(request))
        command = ['blender', '--background', '--factory-startup', '--threads', str(args.threads),
                   '--python-exit-code', '2', '--python', str(ROOT/'scripts/blender_scene.py'), '--', str(request_file)]
        with (directory/'blender.log').open('w') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise SystemExit((directory/'blender.log').read_text()[-5000:])
        scene_info = json.loads((directory/'scene-info.json').read_text())
        audit = json.loads((directory/'audit.json').read_text())
        images = []
        for number, index in enumerate(indices):
            frame, record = frames[index], audit[number]
            expected_pose = frame['truth']['pose']
            pose_error = math.hypot(record['ego_pose']['position']['x']-expected_pose['position']['x'], record['ego_pose']['position']['y']-expected_pose['position']['y'])
            expected_objects = {a['id']: a for a in frame['objects']}
            objects_match = len(record['objects']) == len(expected_objects) and all(
                a['id'] in expected_objects and math.hypot(a['position']['x']-expected_objects[a['id']]['position']['x'], a['position']['y']-expected_objects[a['id']]['position']['y']) <= 1e-4 for a in record['objects'])
            if record['time'] != frame['time'] or pose_error > 1e-4 or abs(record['ego_pose']['yaw']-expected_pose['yaw']) > 1e-5 or not objects_match:
                raise SystemExit('Rendered scene state differs from the recorded simulation')
            with Image.open(directory/f'{number:04d}.png') as rendered:
                image = Image.new('RGB', (960, 640), '#0a1220')
                image.paste(rendered, (0, 54))
            draw = ImageDraw.Draw(image)
            draw.text((22, 10), 'RustDrive', font=font(28, True), fill='#edf4ff')
            draw.text((204, 20), 'RNE NATIVE DYNAMICS  /  BLENDER 3D REPLAY', font=font(12, True), fill='#46e3c2')
            phase = (frame.get('navigation') or {}).get('phase', frame['trajectory']['mode'])
            draw.text((22, 606), f"{phase.upper()}   |   {frame['truth']['speed']*3.6:.1f} km/h   |   t = {frame['time']:.1f} s", font=font(15, True), fill='#edf4ff')
            draw.text((610, 608), 'BLUE ego   TRAFFIC actors   TEAL plan   /   3x', font=font(12), fill='#8698b3')
            images.append(image)
        if args.preview_time is not None:
            images[0].save(args.output.with_suffix('.png'))
            print(args.output.with_suffix('.png'))
            if args.scene_output:
                print(f'Editable scene: {args.scene_output}')
            return
        durations = [100]*(len(images)-1)+[1400]
        encode_gif(images, args.output, durations)
        images[min(len(images)-1, 50)].save(args.output.with_suffix('.png'))
        with Image.open(args.output) as gif:
            count, duration = gif.n_frames, 0
            for index in range(count):
                gif.seek(index)
                duration += gif.info['duration']
            assert gif.size == (960, 640) and duration == sum(durations)
        renderer_hash = hashlib.sha256()
        for source in ['blender_scene.py', 'blender_assets.py', 'render_demo_3d.py']:
            renderer_hash.update(source.encode())
            renderer_hash.update((ROOT/'scripts'/source).read_bytes())
        provenance = {'schema_version': 1, 'backend': run['backend'],
                      'renderer': 'Blender Cycles CPU', 'blender_version': subprocess.check_output(['blender', '--version'], text=True).splitlines()[0],
                      'input_trace': str(args.run), 'input_sha256': hashlib.sha256(args.run.read_bytes()).hexdigest(),
                      'rne_expected_revision': (ROOT/'integrations/rne/rne-revision.txt').read_text().strip(),
                      'scenario': run['scenario'], 'summary': run['summary'], 'gif_frames': count,
                      'sampled_frames': len(images), 'playback_speed': 3, 'scene_states_verified': len(audit),
                      'samples': args.samples, 'physics_domain': 'planar', 'scene': scene_info,
                      'renderer_source_sha256': renderer_hash.hexdigest(),
                      'gif_palette_colors': 192, 'gif_dither': False, 'spatial_filter': '3x3 median, viewport only',
                      'renderer_command': shlex.join(['python3','scripts/render_demo_3d.py',str(args.run),'--output',str(args.output),
                                                     '--samples',str(args.samples),'--threads',str(args.threads),'--camera',args.camera,
                                                     '--traffic-models',*args.traffic_models]+(['--scene-output',str(args.scene_output)] if args.scene_output else []))}
        args.output.with_suffix('.json').write_text(json.dumps(provenance, indent=2)+'\n')
        print(f'{args.output}: {count} frames, 960x640, {args.output.stat().st_size:,} bytes')


if __name__ == '__main__':
    main()
