#!/usr/bin/env python3
"""Record the playable city with keyboard inputs; never write vehicle state."""

import argparse
import hashlib
import json
import math
import shutil
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

from PIL import Image
from playwright.sync_api import sync_playwright


ROOT = Path(__file__).resolve().parents[1]
STEP_MS = 40
FRAME_MS = 80
WIDTH, HEIGHT = 960, 540
ROUTE = [(0, -22), (0, 0), (50, 0), (50, -50), (0, -50), (0, 0)]
SOURCE_FILES = [
    'web/index.html', 'web/src/main.js', 'web/src/city.js', 'web/src/style.css',
    'web/package-lock.json', 'web/vite.config.js', 'scripts/build-playground.sh',
    'integrations/playground/src/lib.rs', 'integrations/playground/Cargo.toml',
    'integrations/playground/Cargo.lock', 'rust-toolchain.toml',
    'scripts/capture-city-chase.py',
]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class KeyboardDriver:
    """A repeatable input fixture for filming, separate from the research driver."""

    def __init__(self, page):
        self.page = page
        self.held = set()
        self.segment = 0
        self.previous_yaw = math.pi / 2
        self.pulsed = set()
        self.pulse_until = -1

    def set_keys(self, wanted):
        for key in sorted(self.held - wanted):
            self.page.keyboard.up(key)
        for key in sorted(wanted - self.held):
            self.page.keyboard.down(key)
        self.held = wanted

    def step(self, state, time):
        a, b = ROUTE[self.segment:self.segment + 2]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        progress = ((state['x'] - a[0]) * dx + (state['y'] - a[1]) * dy) / length
        if progress > length - 3 and self.segment < len(ROUTE) - 2:
            self.segment += 1
            a, b = ROUTE[self.segment:self.segment + 2]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            progress = ((state['x'] - a[0]) * dx + (state['y'] - a[1]) * dy) / length
        ahead, aim = max(0, progress) + 10, self.segment
        while ahead > length and aim < len(ROUTE) - 2:
            ahead -= length
            aim += 1
            a, b = ROUTE[aim:aim + 2]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
        target = (a[0] + dx * min(ahead, length) / length,
                  a[1] + dy * min(ahead, length) / length)
        error = wrap(math.atan2(target[1] - state['y'], target[0] - state['x']) - state['yaw'])
        yaw_rate = wrap(state['yaw'] - self.previous_yaw) / (STEP_MS / 1000)
        self.previous_yaw = state['yaw']
        steering = 2.4 * error - .85 * yaw_rate
        target_speed = 11 if abs(error) > .25 else 14
        wanted = ({'KeyW'} if state['speed'] < target_speed else
                  {'KeyS'} if state['speed'] > target_speed + 1.2 else set())
        if steering > .14:
            wanted.add('KeyA')
        elif steering < -.14:
            wanted.add('KeyD')
        if abs(error) > .32 and state['speed'] > 10 and self.segment not in self.pulsed:
            self.pulse_until = time + .16
            self.pulsed.add(self.segment)
        if time < self.pulse_until:
            wanted.add('Space')
        self.set_keys(wanted)


def capture(args, frames):
    trace, errors = [], []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),
            headless=True,
            args=['--no-sandbox', '--disable-dev-shm-usage', '--use-angle=swiftshader',
                  '--enable-unsafe-swiftshader'],
        )
        page = browser.new_page(viewport={'width': WIDTH, 'height': HEIGHT}, device_scale_factor=1)
        page.on('pageerror', lambda error: errors.append(str(error)))
        epoch = datetime(2026, 1, 1, tzinfo=timezone.utc)
        page.clock.install(time=epoch)
        page.clock.pause_at(epoch + timedelta(seconds=1))
        # Render at a controlled 25 Hz. Each callback still runs the unmodified
        # game's 120 Hz Rust physics accumulator; screenshots sample every other render.
        page.add_init_script('''
            window.__captureQueue = [];
            window.requestAnimationFrame = callback => window.__captureQueue.push(callback);
            window.__captureStep = () => {
                const callbacks = window.__captureQueue.splice(0);
                for (const callback of callbacks) callback(performance.now());
            };
        ''')
        page.goto(args.url, wait_until='networkidle')

        def tick():
            page.clock.run_for(STEP_MS)
            page.evaluate('window.__captureStep()')

        def snapshot():
            return page.evaluate('window.rustDriving.snapshot()')

        tick()
        assert snapshot()['ready'] and snapshot()['physics'] == 'rust-wasm'
        page.locator('#start').click(force=True)
        page.locator('#chase').click(force=True)
        page.locator('#world').focus()
        page.mouse.move(1, HEIGHT - 1)
        for _ in range(6000 // STEP_MS):
            tick()
        driver = KeyboardDriver(page)
        frame_count = round(args.seconds * 1000 / FRAME_MS)
        steps = frame_count * 2
        for step in range(steps):
            driver.step(snapshot(), step * STEP_MS / 1000)
            if step == 45:
                page.locator('#camera').click(force=True)
                page.locator('#world').focus()
            if step == 245:
                for _ in range(2):
                    page.locator('#camera').click(force=True)
                page.locator('#world').focus()
            tick()
            state = snapshot()
            trace.append({'step': step, 'keys': sorted(driver.held), 'state': state})
            if step % 2:
                index = step // 2
                page.screenshot(path=str(frames / f'frame-{index:04d}.png'))
                if index % 25 == 0:
                    print(f'Recorded {index + 1}/{frame_count} frames', flush=True)
        driver.set_keys(set())
        browser_version = browser.version
        browser.close()
    assert not errors, errors
    return trace, browser_version


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4174/rust_driving/')
    parser.add_argument('--seconds', type=float, default=14.4)
    parser.add_argument('--output', type=Path, default=ROOT / 'assets/city-chase-demo.gif')
    parser.add_argument('--evidence', type=Path, default=ROOT / 'artifacts/city-chase-capture')
    parser.add_argument('--keep-frames', type=Path, help='Optional fresh directory for the original PNG frames')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 8 <= args.seconds <= 16:
        parser.error('--seconds must be between 8 and 16')
    if not shutil.which('ffmpeg'):
        parser.error('ffmpeg is required')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.evidence.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rustdriving-city-chase-') as temporary:
        frames = Path(temporary)
        trace, browser_version = capture(args, frames)
        if args.keep_frames is not None:
            shutil.copytree(frames, args.keep_frames)
        states = [entry['state'] for entry in trace]
        assert all(state['chase'] and state['trafficCars'] == 22 for state in states)
        assert all(not state['paused'] for state in states)
        assert max(abs(state['rearSlip']) for state in states) > math.radians(15)
        assert max(state['smokeParticles'] for state in states) > 0
        moving_fraction = sum(abs(state['speed']) > 2 for state in states) / len(states)
        assert moving_fraction > .9, moving_fraction
        assert states[-1]['distance'] - states[0]['distance'] > 100
        subprocess.run([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
            '-framerate', str(1000 / FRAME_MS), '-i', str(frames / 'frame-%04d.png'),
            '-filter_complex',
            '[0:v]split[a][b];[a]palettegen=max_colors=128:stats_mode=full[p];'
            '[b][p]paletteuse=dither=none:diff_mode=rectangle',
            '-loop', '0', str(args.output),
        ], check=True)
        peak = max(range(1, len(states), 2), key=lambda i: states[i]['smokeParticles'])
        shutil.copyfile(frames / f'frame-{peak // 2:04d}.png', args.output.with_suffix('.png'))
        for index in [0, peak // 2, len(states) // 4, len(states) // 2 - 1]:
            shutil.copyfile(frames / f'frame-{index:04d}.png', args.evidence / f'frame-{index:04d}.png')
    trace_path = args.evidence / 'trace.json'
    trace_path.write_text(json.dumps(trace, indent=2) + '\n')
    with Image.open(args.output) as gif:
        assert gif.size == (WIDTH, HEIGHT) and gif.n_frames == len(states) // 2
        duration = 0
        for frame in range(gif.n_frames):
            gif.seek(frame)
            duration += gif.info['duration']
        assert duration == gif.n_frames * FRAME_MS
        assert gif.info.get('loop') == 0
        frame_count = gif.n_frames
    metadata = {
        'schema_version': 1,
        'backend': 'rust-wasm-city-playground',
        'renderer': 'Three.js / Chromium WebGL 2 / SwiftShader',
        'browser_version': browser_version,
        'encoder': 'FFmpeg global 128-color palette, no dithering',
        'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'capture_url': args.url,
        'source_sha256': {name: sha256(ROOT / name) for name in SOURCE_FILES},
        'wasm_sha256': sha256(ROOT / 'web/dist/drive.wasm'),
        'capture': {'width': WIDTH, 'height': HEIGHT, 'frames': frame_count,
                    'frame_duration_ms': FRAME_MS, 'playback_speed': 1,
                    'duration_seconds': duration / 1000, 'warmup_seconds': 6,
                    'render_step_ms': STEP_MS, 'physics_hz': 120,
                    'camera_switch_seconds': [1.8, 9.8]},
        'input': 'Scripted keyboard events and UI camera/chase buttons; read-only snapshots. No pose writes.',
        'route_fixture_m': ROUTE,
        'scope': 'Planar arcade gameplay recording; not autonomous-driving acceptance evidence.',
        'observed': {'buildings': states[0]['buildings'], 'intersections': states[0]['intersections'],
                     'ordinary_traffic_cars': 20, 'police_cars': 2,
                     'distance_m': states[-1]['distance'] - states[0]['distance'],
                     'moving_fraction': moving_fraction,
                     'rear_slip_peak_degrees': max(abs(state['rearSlip']) for state in states) * 180 / math.pi,
                     'smoke_particles_peak': max(state['smokeParticles'] for state in states),
                     'skid_segments': states[-1]['skidSegments'],
                     'uncaught_errors': []},
        'trace': str(trace_path.relative_to(ROOT)) if trace_path.is_relative_to(ROOT) else str(trace_path),
        'trace_sha256': sha256(trace_path),
        'gif_sha256': sha256(args.output),
        'poster_sha256': sha256(args.output.with_suffix('.png')),
    }
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
