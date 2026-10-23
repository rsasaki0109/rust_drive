"""Drive the actual ridge UI, verify flight/landing, and optionally record a GIF."""
import argparse
import hashlib
import json
import math
import shutil
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[2]
STEP_MS, FRAME_MS = 40, 80
WIDTH, HEIGHT = 960, 540
SOURCES = [
    'web/src/urban-city.js', 'web/src/urban-roads.js', 'web/src/guardrails.js',
    'integrations/playground/src/lib.rs', 'integrations/playground/src/elevation.rs',
    'integrations/playground/src/structures.rs', 'web/src/structures.js',
    'integrations/playground/Cargo.toml', 'integrations/playground/Cargo.lock',
    'web/src/junctions.js', 'web/src/vehicle-pose.js', 'web/src/main.js', 'web/src/ridge.js', 'web/src/city.js', 'web/src/scenery.js',
    'web/src/regions.js', 'web/src/style.css', 'web/index.html',
    'web/package-lock.json', 'web/vite.config.js', 'scripts/build-playground.sh',
    'rust-toolchain.toml', 'web/tests/ridge_smoke.py',
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4173/rust_driving/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record', type=Path, help='Intentionally refresh GIF, poster and provenance')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.record and not shutil.which('ffmpeg'):
        parser.error('Recording requires ffmpeg')
    trace, errors = [], []
    with tempfile.TemporaryDirectory(prefix='rustdriving-ridge-') as temp:
        frames = Path(temp)
        with sync_playwright() as p:
            browser = p.chromium.launch(
                executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),
                headless=True, args=['--no-sandbox', '--disable-dev-shm-usage',
                                     '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
            page = browser.new_page(viewport={'width': WIDTH, 'height': HEIGHT}, device_scale_factor=1)
            page.on('pageerror', lambda error: errors.append(str(error)))
            epoch = datetime(2026, 1, 1, tzinfo=timezone.utc)
            page.clock.install(time=epoch)
            page.clock.pause_at(epoch + timedelta(seconds=1))
            page.add_init_script("""
                window.__ridgeFrames = [];
                window.requestAnimationFrame = callback => window.__ridgeFrames.push(callback);
                window.__ridgeTick = () => {
                    const callbacks = window.__ridgeFrames.splice(0);
                    for (const callback of callbacks) callback(performance.now());
                };
            """)
            page.goto(args.url, wait_until='networkidle')

            def tick():
                page.clock.run_for(STEP_MS)
                page.evaluate('window.__ridgeTick()')

            def snap():
                return page.evaluate('window.rustDriving.snapshot()')

            tick()
            assert snap()['ready'] and snap()['physics'] == 'rust-wasm'
            page.locator('#district-start').select_option('ridge', force=True)
            tick()
            assert snap()['district'] == 'ridge' and snap()['grounded'] and snap()['height'] == 0
            page.screenshot(path=str(args.output / 'ridge-start.png'))
            page.locator('#start').click(force=True)
            page.locator('#world').focus()
            page.locator('#camera').click(force=True)
            page.locator('#world').focus()
            for _ in range(25):
                tick()
            held, switched, overhead, peak_frame, frame_count = set(), False, True, None, 0
            landed_at = None
            for step in range(1000):
                state = snap()
                # Moving traffic can deflect the player; use actual A/D events to keep the avenue.
                desired_yaw = max(-.18, min(.18, (150 - state['y']) * .06))
                error = math.atan2(math.sin(desired_yaw - state['yaw']), math.cos(desired_yaw - state['yaw']))
                command = error - .2 * state['yawRate'] - .08 * math.atan2(state['lateral'], max(1, abs(state['speed'])))
                wanted = {'KeyA'} if command > .03 else {'KeyD'} if command < -.03 else set()
                wanted |= {'KeyW'} if not state['landings'] else {'KeyS'} if state['speed'] > 1 else set()
                for key in sorted(held - wanted):
                    page.keyboard.up(key)
                for key in sorted(wanted - held):
                    page.keyboard.down(key)
                held = wanted
                if state['x'] > 320 and not switched:
                    while snap()['cameraMode'] != 0:
                        page.locator('#camera').click(force=True)
                    page.locator('#world').focus()
                    switched = True
                tick()
                state = snap()
                trace.append({'step': step, 'held': sorted(held), 'state': state})
                assert abs(state['carHeight'] - state['height'] - .02) < 1e-6
                assert state['x'] <= 560.01
                assert all(math.isfinite(state[key]) for key in ['x', 'y', 'height', 'verticalSpeed', 'pitch', 'speed'])
                if abs(state['pitch']) > .4 and not (args.output / 'ridge-climb.png').exists():
                    page.screenshot(path=str(args.output / 'ridge-climb.png'))
                if not state['grounded'] and state['airTime'] > 1 and peak_frame is None:
                    assert state['height'] > 35 and state['cameraHeight'] > state['height'] + 1
                    page.screenshot(path=str(args.output / 'ridge-flight.png'))
                    peak_frame = step // 2
                if state['landings'] and landed_at is None:
                    landed_at = step
                    page.screenshot(path=str(args.output / 'ridge-landing.png'))
                if args.record and step % 2:
                    page.screenshot(path=str(frames / f'frame-{frame_count:04d}.png'))
                    frame_count += 1
                    if frame_count % 50 == 0:
                        print(f'Recorded {frame_count} ridge frames', flush=True)
                if landed_at is not None and step - landed_at >= 50 and step % 2:
                    break
            for key in sorted(held):
                page.keyboard.up(key)
            states = [entry['state'] for entry in trace]
            (args.output / 'trace.json').write_text(json.dumps(trace, indent=2) + '\n')
            page.screenshot(path=str(args.output / 'ridge-end.png'))
            diagnostic_keys = ['x', 'y', 'height', 'speed', 'yaw', 'contacts', 'brokenGuardrails', 'landings', 'grounded']
            print(json.dumps({'ridge_drive': [
                dict({key: entry['state'][key] for key in diagnostic_keys}, nearbyTraffic=entry['state']['nearbyTraffic'])
                for entry in trace[::100] + trace[-1:]
            ]}), flush=True)
            assert peak_frame is not None and landed_at is not None, 'The UI run must climb, fly and land'
            assert max(s['height'] for s in states) > 48.5
            assert max(s['airTime'] for s in states) > 3
            assert states[-1]['grounded'] and states[-1]['landings'] == 1 and states[-1]['height'] == 0
            assert max(s['landingSpeed'] for s in states) > 20
            assert overhead and switched
            page.keyboard.press('Escape')
            tick()
            paused = snap()
            for _ in range(5):
                tick()
            assert snap()['paused'] and snap()['x'] == paused['x'] and snap()['height'] == paused['height']
            page.locator('#recover').click(force=True)
            tick()
            reset = snap()
            assert reset['x'] == 190 and reset['y'] == 150 and reset['height'] == 0 and reset['grounded']
            assert reset['landings'] == 0 and reset['bestAirTime'] == 0 and reset['keysHeld'] == 0
            assert not errors, errors
            version = browser.version
            browser.close()
        trace_path = args.output / 'trace.json'
        trace_path.write_text(json.dumps(trace, indent=2) + '\n')
        report = {
            'schema_version': 1, 'passed': True, 'backend': 'rust-wasm-ridge-flight',
            'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
            'browser_version': version, 'renderer': 'Three.js / Chromium WebGL 2 / SwiftShader',
            'input': 'Keyboard W/A/S/D and public start-area/camera/pause/recover controls; read-only diagnostic snapshots. No pose writes.',
            'source_sha256': {name: sha(ROOT / name) for name in SOURCES},
            'wasm_sha256': sha(ROOT / 'web/dist/drive.wasm'),
            'observed': {
                'peak_height_m': max(s['height'] for s in states),
                'peak_uphill_pitch_degrees': max(s['pitch'] for s in states if s['grounded']) * 180 / math.pi,
                'flight_seconds': max(s['airTime'] for s in states),
                'landing_speed_mps': max(s['landingSpeed'] for s in states),
                'landing_x_m': states[landed_at]['x'], 'landings': states[-1]['landings'],
                'pause_freezes_height': True, 'recover_clears_flight': True, 'uncaught_errors': errors,
            },
            'trace_sha256': sha(trace_path),
            'scope': 'Authored arcade terrain and ballistic flight; not autonomous-driving or real-vehicle evidence.',
        }
        if args.record:
            from PIL import Image
            args.record.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run([
                'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(1000 / FRAME_MS),
                '-i', str(frames / 'frame-%04d.png'), '-filter_complex',
                '[0:v]split[a][b];[a]palettegen=max_colors=128:stats_mode=full[p];[b][p]paletteuse=dither=none:diff_mode=rectangle',
                '-loop', '0', str(args.record),
            ], check=True)
            with Image.open(args.record) as gif:
                assert gif.size == (WIDTH, HEIGHT) and gif.n_frames == frame_count and gif.info.get('loop') == 0
                duration = 0
                for i in range(gif.n_frames):
                    gif.seek(i)
                    duration += gif.info['duration']
                assert duration == frame_count * FRAME_MS
            shutil.copyfile(args.output / 'ridge-climb.png', args.record.with_suffix('.png'))
            report['capture'] = {'width': WIDTH, 'height': HEIGHT, 'frames': frame_count,
                                 'frame_duration_ms': FRAME_MS, 'render_step_ms': STEP_MS,
                                 'physics_hz': 120, 'duration_seconds': duration / 1000, 'playback_speed': 1,
                                 'opening_view': 'Oblique panorama of the actual climb'}
            report['gif_sha256'] = sha(args.record)
            report['poster_sha256'] = sha(args.record.with_suffix('.png'))
            args.record.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
        (args.output / 'ridge-smoke.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
