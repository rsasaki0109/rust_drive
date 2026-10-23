"""Record real city driving, sloped streets, guardrail fracture and the station loop."""
import argparse
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path
from playwright.sync_api import sync_playwright
from browser_clock import advance, control_clock
from ridge_smoke import ROOT, SOURCES, STEP_MS, FRAME_MS, WIDTH, HEIGHT, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4173/rust_driving/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.record and not shutil.which('ffmpeg'):
        parser.error('Recording requires FFmpeg')
    errors, trace = [], []
    frames_written = 0
    with tempfile.TemporaryDirectory(prefix='rustdriving-urban-') as temp:
        frames = Path(temp)
        with sync_playwright() as p:
            browser = p.chromium.launch(
                executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),
                headless=True, args=['--no-sandbox', '--disable-dev-shm-usage',
                                     '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
            page = browser.new_page(viewport={'width': WIDTH, 'height': HEIGHT}, device_scale_factor=1)
            page.on('pageerror', lambda error: errors.append(str(error)))
            control_clock(page)
            page.goto(args.url, wait_until='networkidle')
            advance(page, STEP_MS)
            snap = lambda: page.evaluate('window.rustDriving.snapshot({traffic: true})')
            assert snap()['ready'] and snap()['buildings'] == 107 and snap()['intersections'] == 54
            assert snap()['trafficCars'] == 40 and snap()['urbanBuildings'] >= 40
            assert set(snap()['trafficTypes']) == {'sedan', 'compact', 'taxi', 'van'}
            held = set()

            def keys(wanted):
                for key in sorted(held - wanted):
                    page.keyboard.up(key)
                for key in sorted(wanted - held):
                    page.keyboard.down(key)
                held.clear()
                held.update(wanted)

            def camera(mode):
                while snap()['cameraMode'] != mode:
                    page.locator('#camera').click(force=True)
                page.locator('#world').focus()

            def start(area):
                assert snap()['paused']
                page.locator('#district-start').select_option(area, force=True)
                advance(page, STEP_MS)
                page.locator('#start').click(force=True)
                if not snap()['day']:
                    page.locator('#day').click(force=True)
                if not snap()['chase']:
                    page.locator('#chase').click(force=True)
                camera(0)
                for _ in range(12):
                    advance(page, STEP_MS)
                assert snap()['chase'] and snap()['trafficCars'] == 42

            def steer(wanted_yaw, state):
                error = math.atan2(math.sin(wanted_yaw - state['yaw']), math.cos(wanted_yaw - state['yaw']))
                command = error - .2 * state['yawRate'] - .08 * math.atan2(state['lateral'], max(1, abs(state['speed'])))
                return {'KeyA'} if command > .03 else {'KeyD'} if command < -.03 else set()

            def capture(area, step, state):
                nonlocal frames_written
                assert all(math.isfinite(state[k]) for k in ['x', 'y', 'height', 'speed', 'cameraHeight'])
                assert abs(state['carHeight'] - state['height'] - .02) < 1e-6
                if state['ceiling'] < 1000:
                    assert state['height'] + 1.55 <= state['ceiling'] + 1e-6
                    assert state['cameraHeight'] < state['ceiling'] - .1
                trace.append({'area': area, 'step': step, 'held': sorted(held), 'state': state})
                if args.record and step % 2:
                    page.screenshot(path=str(frames / f'frame-{frames_written:04d}.png'))
                    frames_written += 1
                    if frames_written % 50 == 0:
                        print(f'Recorded {frames_written} urban frames', flush=True)

            start('urban')
            page.screenshot(path=str(args.output / 'urban-start.png'))
            broke_at, landed_at, impact_before, impact_after = None, None, None, None
            city_shot = impact_shot = traffic_shot = False
            shared_traffic_frames = 0
            for step in range(1200):
                before = snap()
                if 255 < before['x'] < 345:
                    camera(1)
                elif before['x'] >= 345:
                    camera(0)
                desired_yaw = max(-.18, min(.18, (150 - before['y']) * .06))
                wanted = steer(desired_yaw, before)
                wanted |= {'KeyW'} if not before['landings'] else {'KeyS'} if before['speed'] > 1 else set()
                keys(wanted)
                advance(page, STEP_MS)
                state = snap()
                capture('hillside-city', step, state)
                if step % 100 == 0:
                    print(json.dumps({'urban_step': step, 'state': {
                        k: state[k] for k in ['x', 'y', 'height', 'speed', 'yaw', 'contacts', 'brokenGuardrails', 'landings', 'grounded', 'nearbyTraffic']
                    }}), flush=True)
                ordinary_in_view = sum(not c['police'] for c in state['trafficInView'])
                police_in_view = sum(c['police'] for c in state['trafficInView'])
                if state['cameraMode'] == 1 and ordinary_in_view >= 2 and police_in_view >= 1:
                    shared_traffic_frames += 1
                    if not traffic_shot:
                        page.screenshot(path=str(args.output / 'urban-traffic.png'))
                        traffic_shot = True
                if state['x'] > 280 and state['cameraMode'] == 1 and not city_shot:
                    page.screenshot(path=str(args.output / 'urban-climb.png'))
                    city_shot = True
                if state['brokenGuardrails'] and broke_at is None:
                    assert state['grounded'], 'Guardrail must fracture before the car leaves the road'
                    broke_at = state['time']
                    impact_before, impact_after = before['speed'], state['speed']
                    assert 0 < impact_after < impact_before
                if broke_at is not None and state['time'] - broke_at > .25 and not impact_shot:
                    page.screenshot(path=str(args.output / 'guardrail-impact.png'))
                    impact_shot = True
                if state['landings'] and landed_at is None:
                    landed_at = step
                    page.screenshot(path=str(args.output / 'urban-landing.png'))
                if landed_at is not None and step - landed_at >= 40 and step % 2:
                    break
            else:
                (args.output / 'trace.json').write_text(json.dumps(trace, indent=2) + '\n')
                page.screenshot(path=str(args.output / 'urban-end.png'))
                raise AssertionError('The actual urban drive never broke the rail and landed')
            keys(set())
            hill = [e['state'] for e in trace]
            assert city_shot and impact_shot and broke_at is not None and landed_at is not None
            assert traffic_shot and shared_traffic_frames >= 10, 'The city view must frame ordinary traffic and a real pursuing police car'
            visible_tracks = {}
            for state in hill:
                for vehicle in state['trafficInView']:
                    visible_tracks.setdefault((vehicle['id'], vehicle['police']), []).append((vehicle['x'], vehicle['y']))
            moving_in_view = {False: 0, True: 0}
            for (_, police), positions in visible_tracks.items():
                if max(math.hypot(x - positions[0][0], y - positions[0][1]) for x, y in positions) > 4:
                    moving_in_view[police] += 1
            assert moving_in_view[False] >= 2 and moving_in_view[True] >= 1, 'The framed vehicles must actually move'
            assert max(s['height'] for s in hill) > 48.5 and max(s['airTime'] for s in hill) > 3
            assert hill[-1]['landings'] == 1 and hill[-1]['grounded'] and hill[-1]['height'] == 0
            assert max(s['trainX'] for s in hill) - min(s['trainX'] for s in hill) > 30
            assert max(s['elevatedTraffic'] for s in hill) > 0
            page.keyboard.press('Escape')
            advance(page, STEP_MS)
            paused = snap()
            advance(page, 200)
            assert snap()['x'] == paused['x'] and snap()['height'] == paused['height']
            page.locator('#recover').click(force=True)
            advance(page, STEP_MS)
            assert snap()['brokenGuardrails'] == 0 and snap()['keysHeld'] == 0 and snap()['height'] == 0
            # A visible cut joins a second actual UI-selected start in the same connected world.
            start('station')
            station_start = snap()
            station = []
            for step in range(250):
                state = snap()
                angle = math.atan2((state['y'] - 150) / 65, (state['x'] - 490) / 55) + .3
                target = (490 + 55 * math.cos(angle), 150 + 65 * math.sin(angle))
                wanted = steer(math.atan2(target[1] - state['y'], target[0] - state['x']), state)
                if state['speed'] < 13:
                    wanted.add('KeyW')
                elif state['speed'] > 16:
                    wanted.add('KeyS')
                keys(wanted)
                advance(page, STEP_MS)
                state = snap()
                station.append(state)
                capture('station-loop', step, state)
                if step % 100 == 0:
                    print(json.dumps({'station_step': step, 'state': {
                        k: state[k] for k in ['x', 'y', 'height', 'speed', 'yaw', 'contacts', 'ceiling']
                    }}), flush=True)
                assert abs(math.hypot((state['x'] - 490) / 55, (state['y'] - 150) / 65) - 1) < .18, json.dumps({'station_step': step, 'state': state})
                if step == 100:
                    page.screenshot(path=str(args.output / 'station-loop.png'))
            keys(set())
            assert station[-1]['distance'] - station_start['distance'] > 40
            assert abs(math.atan2(math.sin(station[-1]['yaw']), math.cos(station[-1]['yaw']))) > .4
            assert any(abs(s['ceiling'] - 8.9) < 1e-6 for s in station), 'The actual curve must pass beneath the railway'
            assert all(s['grounded'] and abs(s['height']) < 1e-9 for s in station)
            assert not errors, errors
            version = browser.version
            browser.close()
        trace_path = args.output / 'trace.json'
        trace_path.write_text(json.dumps(trace, indent=2) + '\n')
        report = {
            'schema_version': 1, 'passed': True, 'backend': 'rust-wasm-urban-city',
            'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
            'source_sha256': {name: sha(ROOT / name) for name in SOURCES + ['web/tests/browser_clock.py', 'web/tests/urban_smoke.py']},
            'wasm_sha256': sha(ROOT / 'web/dist/drive.wasm'),
            'browser_version': version, 'renderer': 'Three.js / Chromium WebGL 2 / SwiftShader',
            'input': 'Public area/day/chase/camera controls and actual W/A/S/D keyboard events. Read-only snapshots; no pose writes.',
            'city': {'buildings': 107, 'intersections': 54, 'ordinary_traffic': 40, 'police': 2,
                     'vehicle_types': ['sedan', 'compact', 'taxi', 'van'],
                     'ordinary_distribution': {'city': 20, 'hillside': 12, 'station': 8}},
            'observed': {
                'peak_height_m': max(s['height'] for s in hill),
                'uphill_pitch_degrees': max(s['pitch'] for s in hill if s['grounded']) * 180 / math.pi,
                'flight_seconds': max(s['airTime'] for s in hill),
                'guardrail_fractures': max(s['brokenGuardrails'] for s in hill),
                'guardrail_fractures_before_flight': True,
                'impact_speed_before_mps': impact_before, 'impact_speed_after_mps': impact_after,
                'landing_x_m': hill[landed_at]['x'], 'pause_freezes_motion': True,
                'recovery_restores_guardrails': True, 'elevated_traffic_moves': True,
                'station_curve_distance_m': station[-1]['distance'] - station_start['distance'],
                'station_drives_under_railway': True, 'uncaught_errors': errors,
                'shared_traffic_frame_count': shared_traffic_frames,
                'moving_ordinary_vehicles_in_view': moving_in_view[False],
                'moving_police_in_view': moving_in_view[True],
                'framing_diagnostic': 'Projected body centres within 90 m, with building-AABB and sampled terrain occlusion rejection; inspect captured PNGs separately.',
            },
            'trace_sha256': sha(trace_path),
            'scope': 'Authored Tokyo-inspired arcade city, not a geographic Tokyo reconstruction or autonomous-driving evidence.',
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
                assert gif.size == (WIDTH, HEIGHT) and gif.n_frames == frames_written and gif.info.get('loop') == 0
                duration = 0
                for i in range(gif.n_frames):
                    gif.seek(i)
                    duration += gif.info['duration']
                assert duration == frames_written * FRAME_MS
            outputs = {'poster': ('urban-climb.png', '.png'), 'start': ('urban-start.png', '.start.png'),
                       'impact': ('guardrail-impact.png', '.impact.png'), 'station': ('station-loop.png', '.station.png'),
                       'traffic': ('urban-traffic.png', '.traffic.png')}
            report['output_sha256'] = {}
            for key, (name, suffix) in outputs.items():
                destination = args.record.with_suffix(suffix)
                shutil.copyfile(args.output / name, destination)
                report['output_sha256'][key] = sha(destination)
            report['gif_sha256'] = sha(args.record)
            report['capture'] = {'width': WIDTH, 'height': HEIGHT, 'frames': frames_written,
                                 'duration_seconds': duration / 1000, 'frame_duration_ms': FRAME_MS,
                                 'render_step_ms': STEP_MS, 'physics_hz': 120, 'playback_speed': 1,
                                 'segments': ['city climb/pursuit/guardrail fracture/landing', 'station curve beneath railway'],
                                 'cuts': 'One cut between actual UI-selected starts; driving and camera controls run in the production game.'}
            args.record.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
        (args.output / 'urban-smoke.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
