"""Record five different playable scenarios through public UI and keyboard inputs."""
import argparse
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

from playwright.sync_api import sync_playwright
from browser_clock import advance, control_clock, wait_state
from ridge_smoke import ROOT, SOURCES, STEP_MS, FRAME_MS, WIDTH, HEIGHT, sha


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def forest_route():
    points = [(-146, 75)]
    for centre in [-125, -75]:
        points.extend((centre - 16 + 32 * i / 48,
                       75 + 5 * math.sin(i / 48 * math.tau)) for i in range(49))
        points.append((centre + 25, 75))
    return points


SCENARIOS = [
    dict(id='city-crossing', area='downtown', day=True, chase=False, camera=1,
         seconds=14.4, speed=11, lookahead=9,
         route=[(0, 90), (0, 150), (-50, 150), (-50, 100)]),
    dict(id='harbor-pursuit', area='harbor', day=False, chase=True, camera=0,
         seconds=16, speed=10, lookahead=9,
         route=[(100, -120), (100, -100), (50, -100), (50, -150), (100, -150)]),
    dict(id='forest-trail', area='forest', day=True, chase=False, camera=1,
         seconds=14.4, speed=6.5, lookahead=3.5, route=forest_route()),
    dict(id='plaza-drift', area='plaza', day=False, chase=False, camera=1,
         seconds=10.4, driver='handbrake/countersteer/circle'),
    dict(id='station-loop', area='station', day=True, chase=True, camera=0,
         seconds=12, driver='elliptic boulevard'),
]


class PathDriver:
    """Repeatable keyboard fixture. It reads telemetry and never writes game state."""

    def __init__(self, route, speed, lookahead):
        self.route, self.speed, self.lookahead = route, speed, lookahead
        self.segment = 0

    def keys(self, state):
        a, b = self.route[self.segment:self.segment + 2]
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        progress = ((state['x'] - a[0]) * dx + (state['y'] - a[1]) * dy) / length
        while self.segment < len(self.route) - 2 and (
                progress > length - 1.5 or math.hypot(state['x'] - b[0], state['y'] - b[1]) < 1.8):
            self.segment += 1
            a, b = self.route[self.segment:self.segment + 2]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            progress = ((state['x'] - a[0]) * dx + (state['y'] - a[1]) * dy) / length
        ahead, aim = max(0, progress) + self.lookahead, self.segment
        while ahead > length and aim < len(self.route) - 2:
            ahead -= length
            aim += 1
            a, b = self.route[aim:aim + 2]
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
        target = (a[0] + dx * min(ahead, length) / length,
                  a[1] + dy * min(ahead, length) / length)
        error = wrap(math.atan2(target[1] - state['y'], target[0] - state['x']) - state['yaw'])
        command = 2.4 * error - .85 * state['yawRate']
        speed = min(self.speed, 6.5 if abs(error) > .25 else self.speed)
        wanted = ({'KeyW'} if state['speed'] < speed else
                  {'KeyS'} if state['speed'] > speed + 1 else set())
        if command > .14:
            wanted.add('KeyA')
        elif command < -.14:
            wanted.add('KeyD')
        return wanted


def scenario_keys(config, path_driver, state, seconds):
    if path_driver:
        return path_driver.keys(state)
    if config['id'] == 'plaza-drift':
        wanted = {'KeyW'}
        if 3.2 <= seconds < 4.4:
            wanted |= {'KeyA', 'Space'}
        elif 4.4 <= seconds < 4.9:
            wanted.add('KeyD')
        elif seconds >= 4.9:
            wanted.add('KeyA')
            if state['speed'] > 11:
                wanted = (wanted - {'KeyW'}) | {'KeyS'}
        return wanted
    angle = math.atan2((state['y'] - 150) / 65, (state['x'] - 490) / 55) + .3
    target = (490 + 55 * math.cos(angle), 150 + 65 * math.sin(angle))
    error = wrap(math.atan2(target[1] - state['y'], target[0] - state['x']) - state['yaw'])
    command = error - .2 * state['yawRate'] - .08 * math.atan2(state['lateral'], max(1, abs(state['speed'])))
    wanted = {'KeyA'} if command > .03 else {'KeyD'} if command < -.03 else set()
    if state['speed'] < 13:
        wanted.add('KeyW')
    elif state['speed'] > 16:
        wanted.add('KeyS')
    return wanted


def visible_movers(states, police):
    tracks = {}
    for state in states:
        for car in state['trafficInView']:
            if car['police'] == police:
                tracks.setdefault(car['id'], []).append((car['x'], car['y']))
    return sum(max(math.hypot(x - points[0][0], y - points[0][1]) for x, y in points) > 4
               for points in tracks.values())


def path_distance(state, route):
    minimum = math.inf
    for a, b in zip(route, route[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0, min(1, ((state['x'] - a[0]) * dx + (state['y'] - a[1]) * dy) / (dx * dx + dy * dy)))
        minimum = min(minimum, math.hypot(state['x'] - a[0] - t * dx, state['y'] - a[1] - t * dy))
    return minimum


def validate(config, states, errors):
    assert not errors, errors
    assert states and all(not s['paused'] and s['grounded'] and abs(s['height']) < 1e-8 for s in states)
    assert all(s['trafficCars'] == (42 if config['chase'] else 40) and s['chase'] == config['chase']
               and s['day'] == config['day'] for s in states)
    distance = states[-1]['distance'] - states[0]['distance']
    moving_fraction = sum(abs(s['speed']) > 2 for s in states) / len(states)
    yaw_span = sum(abs(wrap(b['yaw'] - a['yaw'])) for a, b in zip(states, states[1:]))
    x_span = max(s['x'] for s in states) - min(s['x'] for s in states)
    y_span = max(s['y'] for s in states) - min(s['y'] for s in states)
    ordinary, police = visible_movers(states, False), visible_movers(states, True)
    observed = dict(distance_m=distance, moving_fraction=moving_fraction,
                    accumulated_yaw_change_radians=yaw_span, x_span_m=x_span, y_span_m=y_span,
                    moving_ordinary_vehicles_in_view=ordinary, moving_police_in_view=police,
                    contacts=states[-1]['contacts'], uncaught_errors=errors)
    assert moving_fraction > .7 and distance > 60, observed
    name = config['id']
    if name == 'city-crossing':
        assert distance > 80 and min(s['x'] for s in states) < -35 and max(s['y'] for s in states) > 140, observed
        assert yaw_span > 1 and ordinary >= 1, observed
    elif name == 'harbor-pursuit':
        assert distance > 85 and min(s['y'] for s in states) < -140 and max(s['y'] for s in states) > -110, observed
        assert yaw_span > 1 and police >= 1, observed
    elif name == 'forest-trail':
        deviation = max(path_distance(s, config['route']) for s in states)
        observed['maximum_path_deviation_m'] = deviation
        assert states[-1]['x'] > -85 and y_span > 6 and yaw_span > .8 and deviation < 4, observed
        assert any(s['yawRate'] > .25 for s in states) and any(s['yawRate'] < -.25 for s in states), observed
    elif name == 'plaza-drift':
        slip = max(abs(s['rearSlip']) for s in states) * 180 / math.pi
        smoke = max(s['smokeParticles'] for s in states)
        skids = max(s['skidSegments'] for s in states)
        observed.update(rear_slip_peak_degrees=slip, smoke_particles_peak=smoke, skid_segments=skids)
        assert slip > 15 and smoke >= 4 and skids >= 10 and x_span > 12 and y_span > 20, observed
        assert sum(s['district'] == 'plaza' for s in states) / len(states) > .85, observed
    elif name == 'station-loop':
        error = max(abs(math.hypot((s['x'] - 490) / 55, (s['y'] - 150) / 65) - 1) for s in states)
        observed.update(maximum_normalized_radius_error=error,
                        passes_beneath_railway=any(abs(s['ceiling'] - 8.9) < 1e-6 for s in states))
        assert error < .18 and observed['passes_beneath_railway'] and yaw_span > .4, observed
    return observed


def encode(frames, output, expected_frames):
    from PIL import Image
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(1000 / FRAME_MS),
        '-i', str(frames / 'frame-%04d.png'), '-filter_complex',
        '[0:v]split[a][b];[a]palettegen=max_colors=128:stats_mode=full[p];[b][p]paletteuse=dither=none:diff_mode=rectangle',
        '-loop', '0', str(output),
    ], check=True)
    with Image.open(output) as gif:
        assert gif.size == (WIDTH, HEIGHT) and gif.n_frames == expected_frames and gif.info.get('loop') == 0
        duration = 0
        for index in range(gif.n_frames):
            gif.seek(index)
            duration += gif.info['duration']
        assert duration == expected_frames * FRAME_MS
    return duration / 1000


def record(browser, config, args):
    name = config['id']
    evidence = args.output / name
    evidence.mkdir(parents=True, exist_ok=True)
    errors, trace, held = [], [], set()
    page = browser.new_page(viewport={'width': WIDTH, 'height': HEIGHT}, device_scale_factor=1)
    page.on('pageerror', lambda error: errors.append(str(error)))
    control_clock(page)
    page.goto(args.url, wait_until='networkidle')
    wait_state(page, 'window.rustDriving?.snapshot().ready', 40000)
    snap = lambda: page.evaluate('window.rustDriving.snapshot({traffic: true})')

    def camera(mode):
        for _ in range(3):
            if snap()['cameraMode'] == mode:
                break
            page.locator('#camera').click(force=True)
        assert snap()['cameraMode'] == mode
        page.locator('#world').focus()

    def keys(wanted):
        for key in sorted(held - wanted):
            page.keyboard.up(key)
        for key in sorted(wanted - held):
            page.keyboard.down(key)
        held.clear()
        held.update(wanted)

    try:
        page.locator('#district-start').select_option(config['area'], force=True)
        advance(page, STEP_MS)
        assert snap()['paused'] and snap()['physics'] == 'rust-wasm'
        page.locator('#start').click(force=True)
        if snap()['day'] != config['day']:
            page.locator('#day').click(force=True)
        if snap()['chase'] != config['chase']:
            page.locator('#chase').click(force=True)
        camera(config['camera'])
        page.mouse.move(1, HEIGHT - 1)
        advance(page, 800)
        path_driver = PathDriver(config['route'], config['speed'], config['lookahead']) if 'route' in config else None
        with tempfile.TemporaryDirectory(prefix='rustdriving-gallery-' + name + '-') as temp:
            frames = Path(temp)
            frame_count, poster_rank, poster_index = 0, -math.inf, 0
            for step in range(round(config['seconds'] * 1000 / STEP_MS)):
                before = snap()
                if name == 'harbor-pursuit':
                    camera(1 if 4 <= step * STEP_MS / 1000 < 12 else 0)
                wanted = scenario_keys(config, path_driver, before, step * STEP_MS / 1000)
                keys(wanted)
                advance(page, STEP_MS)
                state = snap()
                assert all(math.isfinite(state[k]) for k in ['x', 'y', 'height', 'speed', 'yaw', 'cameraHeight'])
                assert abs(state['carHeight'] - state['height'] - .02) < 1e-6
                if state['ceiling'] < 1000:
                    assert state['height'] + 1.55 < state['ceiling'] + 1e-6
                    assert state['cameraHeight'] < state['ceiling'] - .1
                trace.append(dict(step=step, keys=sorted(held), state=state))
                if step % 2:
                    page.screenshot(path=str(frames / f'frame-{frame_count:04d}.png'))
                    if name == 'plaza-drift':
                        rank = state['smokeParticles']
                    elif name == 'forest-trail':
                        rank = abs(state['y'] - 75) if state['x'] > -110 else -1
                    elif name == 'station-loop':
                        rank = 100 if state['ceiling'] < 1000 else -1
                    else:
                        rank = len(state['trafficInView']) + (5 if any(c['police'] for c in state['trafficInView']) else 0)
                    if rank > poster_rank:
                        poster_rank, poster_index = rank, frame_count
                    frame_count += 1
                if step % 100 == 0:
                    print(json.dumps(dict(scenario=name, step=step, state={k: state[k] for k in [
                        'x', 'y', 'speed', 'yaw', 'contacts', 'smokeParticles', 'nearbyTraffic']})), flush=True)
            keys(set())
            assert snap()['keysHeld'] == 0
            trace_path = evidence / 'trace.json'
            trace_path.write_text(json.dumps(trace, indent=2) + '\n')
            states = [entry['state'] for entry in trace]
            observed = validate(config, states, errors)
            output = args.assets / (name + '-demo.gif')
            duration = encode(frames, output, frame_count)
            shutil.copyfile(frames / f'frame-{poster_index:04d}.png', output.with_suffix('.png'))
            shutil.copyfile(frames / 'frame-0000.png', evidence / 'start.png')
            shutil.copyfile(frames / f'frame-{frame_count - 1:04d}.png', evidence / 'end.png')
            report = dict(
                schema_version=1, passed=True, scenario=config, backend='rust-wasm-city-playground',
                source_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                source_sha256={name: sha(ROOT / name) for name in dict.fromkeys(SOURCES + [
                    'web/tests/browser_clock.py', 'web/tests/gallery_smoke.py'])},
                wasm_sha256=sha(ROOT / 'web/dist/drive.wasm'),
                browser_version=browser.version, renderer='Three.js / Chromium WebGL 2 / SwiftShader',
                input='Public area/day/chase/camera controls and real W/A/S/D/Space keyboard events; read-only snapshots. No pose or NPC writes.',
                capture=dict(width=WIDTH, height=HEIGHT, frames=frame_count, frame_duration_ms=FRAME_MS,
                             render_step_ms=STEP_MS, physics_hz=120, duration_seconds=duration,
                             warmup_seconds=.8, playback_speed=1, cuts=0,
                             camera_changes_seconds=[4, 12] if name == 'harbor-pursuit' else []),
                observed=observed, trace_sha256=sha(trace_path), gif_sha256=sha(output),
                poster_sha256=sha(output.with_suffix('.png')),
                framing_scope='Projected body-centre diagnostic with terrain/building occlusion checks; review captured PNGs separately.',
                scope='Authored arcade gameplay, separate from autonomous-driving validation.')
            output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
            (evidence / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(dict(scenario=name, passed=True, observed=observed, capture=report['capture'])), flush=True)
            return report
    except BaseException:
        (evidence / 'trace.json').write_text(json.dumps(trace, indent=2) + '\n')
        page.screenshot(path=str(evidence / 'failure.png'))
        print(json.dumps(dict(scenario=name, failed=True, final_state=snap(), uncaught_errors=errors)), flush=True)
        raise
    finally:
        keys(set())
        page.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4173/rust_driving/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--assets', type=Path, default=ROOT / 'assets')
    parser.add_argument('--scenario', action='append', choices=[s['id'] for s in SCENARIOS])
    args = parser.parse_args()
    if not shutil.which('ffmpeg'):
        parser.error('Recording requires FFmpeg')
    args.output.mkdir(parents=True, exist_ok=True)
    args.assets.mkdir(parents=True, exist_ok=True)
    reports, failed = [], []
    with sync_playwright() as p:
        browser = p.chromium.launch(
            executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),
            headless=True, args=['--no-sandbox', '--disable-dev-shm-usage',
                                 '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
        for config in SCENARIOS:
            if args.scenario and config['id'] not in args.scenario:
                continue
            try:
                reports.append(record(browser, config, args))
            except Exception as error:
                failed.append(dict(scenario=config['id'], error=str(error)))
        browser.close()
    summary = dict(passed=not failed, recorded=[r['scenario']['id'] for r in reports], failures=failed)
    (args.output / 'gallery-smoke.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    assert not failed, failed


if __name__ == '__main__':
    main()
