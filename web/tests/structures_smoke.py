"""Drive real public UI through stacked roads; optionally record a normal-speed GIF."""
import argparse
import json
import math
import shutil
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path

from playwright.sync_api import sync_playwright
from ridge_smoke import FRAME_MS, HEIGHT, ROOT, SOURCES, STEP_MS, WIDTH, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4173/rust_driving/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.record and not shutil.which('ffmpeg'):
        parser.error('Recording requires FFmpeg')
    errors, trace, scenes = [], [], {}
    frame_count = 0
    with tempfile.TemporaryDirectory(prefix='rustdriving-stacked-') as temp:
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
                window.__structureFrames = [];
                window.requestAnimationFrame = callback => window.__structureFrames.push(callback);
                window.__structureTick = () => {
                    const callbacks = window.__structureFrames.splice(0);
                    for (const callback of callbacks) callback(performance.now());
                };
            """)
            page.goto(args.url, wait_until='networkidle')

            def tick():
                page.clock.run_for(STEP_MS)
                page.evaluate('window.__structureTick()')

            def snap():
                return page.evaluate('window.rustDriving.snapshot()')

            tick()
            assert snap()['ready'] and snap()['districtCount'] == 8
            for area in ['skyway', 'underpass', 'tunnel']:
                assert snap()['paused']
                page.locator('#district-start').select_option(area, force=True)
                tick()
                page.locator('#start').click(force=True)
                page.locator('#world').focus()
                if area == 'skyway':
                    page.locator('#camera').click(force=True)
                    page.locator('#world').focus()
                page.keyboard.down('KeyW')
                states, captured = [], False
                for step in range(900):
                    tick()
                    s = snap()
                    states.append(s)
                    trace.append({'area': area, 'step': step, 'key': 'KeyW', 'state': s})
                    assert abs(s['carHeight'] - s['height'] - .02) < 1e-6
                    assert all(math.isfinite(s[key]) for key in ['x', 'y', 'height', 'speed', 'cameraHeight'])
                    if s['ceiling'] < 1000:
                        assert s['height'] + 1.55 < s['ceiling'] + 1e-6
                        assert s['cameraHeight'] < s['ceiling'] - .1
                    crossing = (area == 'skyway' and 345 < s['x'] < 365
                                or area == 'underpass' and 16 < s['y'] < 24
                                or area == 'tunnel' and 345 < s['x'] < 365)
                    if crossing and not captured:
                        page.screenshot(path=str(args.output / (area + '.png')))
                        captured = True
                    record = (area == 'skyway' and 205 < s['x'] < 420
                              or area == 'underpass' and -10 < s['y'] < 38
                              or area == 'tunnel' and 225 < s['x'] < 470)
                    if args.record and record and step % 2:
                        page.screenshot(path=str(frames / f'frame-{frame_count:04d}.png'))
                        frame_count += 1
                        if frame_count % 50 == 0:
                            print(f'Recorded {frame_count} stacked-road frames', flush=True)
                    done = (area == 'skyway' and s['x'] > 510 and s['grounded']
                            or area == 'underpass' and s['y'] > 42
                            or area == 'tunnel' and s['x'] > 510 and s['grounded'])
                    if done:
                        break
                else:
                    raise AssertionError(area + ' never reached its exit')
                page.keyboard.up('KeyW')
                assert captured, area + ' never crossed the observation point'
                if area == 'skyway':
                    assert max(s['height'] for s in states) >= 11.99
                    assert abs(states[-1]['height']) < 1e-9
                elif area == 'underpass':
                    assert max(abs(s['height']) for s in states) < 1e-9
                    assert any(abs(s['ceiling'] - 10.9) < 1e-9 for s in states)
                else:
                    assert min(s['height'] for s in states) <= -7.99
                    assert abs(states[-1]['height']) < 1e-9
                    assert any(abs(s['ceiling'] + 1.2) < 1e-9 for s in states)
                    page.keyboard.press('KeyC')
                    tick()
                    page.locator('#recover').click(force=True)
                    tick()
                    assert snap()['height'] == 0 and snap()['keysHeld'] == 0
                scenes[area] = {'steps': len(states), 'minimum_height_m': min(s['height'] for s in states),
                                'maximum_height_m': max(s['height'] for s in states),
                                'minimum_ceiling_m': min(s['ceiling'] for s in states),
                                'exit_height_m': states[-1]['height'], 'exit_x_m': states[-1]['x'],
                                'exit_y_m': states[-1]['y'], 'uncaught_errors': list(errors)}
                page.keyboard.press('Escape')
                tick()
                paused = snap()
                for _ in range(5):
                    tick()
                assert snap()['paused'] and snap()['x'] == paused['x'] and snap()['height'] == paused['height']
            # Recover while the actual car/camera are below grade with W still held.
            page.locator('#district-start').select_option('tunnel', force=True)
            tick()
            page.locator('#start').click(force=True)
            page.locator('#world').focus()
            page.keyboard.down('KeyW')
            for _ in range(650):
                tick()
                if snap()['x'] > 300:
                    break
            else:
                raise AssertionError('The underground recovery approach did not reach cover')
            assert snap()['height'] < -7.9
            page.keyboard.press('KeyR')
            tick()
            reset = snap()
            assert reset['x'] == 190 and reset['y'] == -105 and reset['height'] == 0
            assert reset['grounded'] and reset['keysHeld'] == 0 and reset['cameraHeight'] >= .7
            page.keyboard.up('KeyW')
            version = browser.version
            assert not errors, errors
            browser.close()
        trace_path = args.output / 'trace.json'
        trace_path.write_text(json.dumps(trace, indent=2) + '\n')
        report = {
            'schema_version': 1, 'passed': True, 'backend': 'rust-wasm-stacked-roads',
            'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
            'browser_version': version, 'renderer': 'Three.js / Chromium WebGL 2 / SwiftShader',
            'input': 'Keyboard W and public start-area/camera/pause/recover controls; read-only snapshots, no pose writes.',
            'source_sha256': {name: sha(ROOT / name) for name in SOURCES + ['web/tests/structures_smoke.py']},
            'wasm_sha256': sha(ROOT / 'web/dist/drive.wasm'), 'scenes': scenes,
            'recover_from_underground_clears_controls_and_camera': True,
            'trace_sha256': sha(trace_path),
            'scope': 'Authored arcade layered roads; not autonomous-driving or real-vehicle evidence.',
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
            shutil.copyfile(args.output / 'skyway.png', args.record.with_suffix('.png'))
            report['capture'] = {'width': WIDTH, 'height': HEIGHT, 'frames': frame_count,
                                 'frame_duration_ms': FRAME_MS, 'render_step_ms': STEP_MS,
                                 'physics_hz': 120, 'duration_seconds': duration / 1000, 'playback_speed': 1,
                                 'segments': ['skyway climb/crossing', 'lower road', 'underground passage']}
            report['gif_sha256'] = sha(args.record)
            report['poster_sha256'] = sha(args.record.with_suffix('.png'))
            args.record.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
        (args.output / 'structures-smoke.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
