"""Encode normal-speed gameplay PNG frames using Pillow's shared GIF palette."""
import argparse
import math
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

ENCODER = 'Pillow 12, shared 128-colour palette from 160x90 frame samples, no dithering'


def encode_gif(frames, output, width, height, frame_ms):
    paths = sorted(Path(frames).glob('frame-*.png'))
    if not paths or frame_ms <= 0 or frame_ms % 10:
        raise ValueError('GIF recording needs frames and a positive centisecond duration')
    columns = math.ceil(math.sqrt(len(paths)))
    atlas = Image.new('RGB', (160 * columns, 90 * math.ceil(len(paths) / columns)))
    for index, path in enumerate(paths):
        with Image.open(path) as source:
            if source.size != (width, height):
                raise ValueError('Unexpected capture dimensions: ' + str(path))
            sample = source.convert('RGB').resize((160, 90), Image.Resampling.LANCZOS)
            atlas.paste(sample, ((index % columns) * 160, (index // columns) * 90))
    palette = atlas.quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    images = []
    for path in paths:
        with Image.open(path) as source:
            images.append(source.convert('RGB').quantize(palette=palette, dither=Image.Dither.NONE))
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    images[0].save(output, save_all=True, append_images=images[1:], format='GIF',
                   duration=frame_ms, loop=0, disposal=1, optimize=False)
    with Image.open(output) as gif:
        assert gif.size == (width, height) and gif.n_frames == len(paths) and gif.info.get('loop') == 0
        duration = 0
        for index in range(gif.n_frames):
            gif.seek(index)
            duration += gif.info['duration']
        assert duration == len(paths) * frame_ms
    return duration / 1000


def check_encoder():
    # Distinct moving coloured blocks verify stored timing, order and palette
    # without needing Chromium or the driving simulation.
    with tempfile.TemporaryDirectory(prefix='rustdriving-gif-check-') as temporary:
        root = Path(temporary)
        for index in range(4):
            frame = Image.new('RGB', (96, 54), '#202b35')
            ImageDraw.Draw(frame).rectangle((8 + index * 18, 18, 18 + index * 18, 30), fill='#ffe040')
            frame.save(root / f'frame-{index:04d}.png')
        output = root / 'check.gif'
        assert encode_gif(root, output, 96, 54, 80) == .32
        with Image.open(output) as gif:
            for index in range(4):
                gif.seek(index)
                image = gif.convert('RGB')
                bright = image.getpixel((12 + index * 18, 24))
                assert bright[0] > 200 and bright[1] > 150 and bright[2] < 120
                if index:
                    dark = image.getpixel((12 + (index - 1) * 18, 24))
                    assert max(dark) < 100, 'previous block must not leave a trail'
    print('GIF encoder: 4 ordered frames, 80 ms each, dimensions, palette, loop and cleared motion passed', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', required=True)
    parser.parse_args()
    check_encoder()
