"""Record actual people, taxi dispatch and completed trips in the browser city."""
import argparse
import hashlib
import json
import math
import os
import shutil
import tempfile
from pathlib import Path
from urllib.parse import parse_qsl, urlencode, urlsplit, urlunsplit

from PIL import Image
from playwright.sync_api import sync_playwright
from browser_clock import advance, control_clock, wait_state
from gif_capture import ENCODER, encode_gif

ROOT = Path(__file__).resolve().parents[2]
WIDTH, HEIGHT, FRAME_MS, FRAME_COUNT = 1280, 800, 100, 280
SOURCES = [
    'web/src/fleet.js', 'web/src/fleet-routing.js', 'web/src/fleet-view.js',
    'web/src/fleet.css', 'web/src/city.js', 'web/src/junctions.js',
    'web/src/traffic-turns.js', 'web/src/urban-roads.js', 'web/src/main.js',
    'web/tests/fleet_smoke.py', 'web/tests/fleet.test.mjs',
    'web/tests/browser_clock.py', 'web/tests/gif_capture.py',
    'integrations/playground/src/lib.rs',
]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fleet_url(url):
    parts = urlsplit(url)
    query = dict(parse_qsl(parts.query))
    query['mode'] = 'fleet'
    return urlunsplit((parts.scheme, parts.netloc, parts.path, urlencode(query), parts.fragment))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4173/rust_driving/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    errors, samples = [], []
    with tempfile.TemporaryDirectory(prefix='rustdriving-taxi-') as temp:
        frames = Path(temp)
        with sync_playwright() as p:
            browser = p.chromium.launch(
                executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),
                headless=True, args=['--no-sandbox', '--disable-dev-shm-usage',
                                     '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
            page = browser.new_page(viewport={'width': WIDTH, 'height': HEIGHT}, device_scale_factor=1)
            page.on('pageerror', lambda error: errors.append(str(error)))
            control_clock(page)
            page.goto(fleet_url(args.url), wait_until='networkidle')
            wait_state(page, 'window.rustDrivingFleet?.snapshot().ready', 10000)
            snap = lambda: page.evaluate('window.rustDrivingFleet.snapshot()')
            first = snap()
            assert first['paused'] and first['time'] == 0
            assert len(first['taxis']) == 8 and first['ambientCars'] >= 30
            assert len(first['bodyColors']) == 5
            page.get_by_role('button', name='START', exact=False).click()
            advance(page, 1000)
            assert not snap()['paused'] and .9 < snap()['time'] < 1.1
            page.locator('#fleet-play').click()
            stopped = snap()
            advance(page, 800)
            assert snap()['time'] == stopped['time'], 'pause must stop demand and vehicles'
            page.locator('#fleet-play').click()

            # A real pre-roll reaches the first dropoff; no pose or state is set.
            # Include the first alighting agent and the next dispatches in the GIF.
            advance(page, 39000)
            while not any(e['type'] == 'alight' for e in snap()['events']) and snap()['time'] < 90:
                advance(page, 500)
            start = snap()
            assert any(e['type'] == 'alight' for e in start['events']), json.dumps(start['metrics'])
            (args.output / 'capture-start.json').write_text(json.dumps(start, indent=2) + '\n')
            frame_hashes, max_people, max_taxis = {}, 0, 0
            for index in range(FRAME_COUNT):
                advance(page, FRAME_MS)
                state = snap()
                samples.append(state)
                assigned = [person['taxi'] for person in state['people'] if person['taxi']]
                assert len(assigned) == len(set(assigned)), 'each taxi must have at most one person'
                assert all(math.isfinite(taxi[key]) for taxi in state['taxis'] for key in ['x', 'y', 'speed'])
                max_people = max(max_people, state['visiblePeople'])
                max_taxis = max(max_taxis, state['visibleTaxis'])
                if args.record:
                    path = frames / f'frame-{index:04d}.png'
                    page.screenshot(path=str(path))
                    frame_hashes[path.name] = sha(path)
                elif index in (0, FRAME_COUNT // 2, FRAME_COUNT - 1):
                    page.screenshot(path=str(args.output / f'fleet-{index:04d}.png'))
                if index % 70 == 0:
                    print('Fleet capture:', index, round(state['time'], 2), state['metrics'], flush=True)
            end = snap()
            clip_events = [e for e in end['events'] if e['time'] > start['time']]
            assert any(e['type'] == 'complete' for e in clip_events), 'the recording must include a completed trip'
            assert any(e['type'] == 'board' for e in clip_events), 'the recording must include a new boarding'
            assert len({e['purpose'] for e in end['events'] if e['type'] == 'request'}) == 5
            assert max_people >= 5 and max_taxis >= 6, (max_people, max_taxis)
            assert any(p['walked'] > 5 for p in end['people']), 'visible walking must come from simulated motion'
            assert abs((end['time'] - start['time']) - FRAME_COUNT * FRAME_MS / 1000) < .03

            # Longer acceptance than the visible clip, including real abandonments.
            while snap()['time'] < 120:
                advance(page, 1000)
            final = snap()
            assert final['metrics']['served'] >= 3, json.dumps(final['metrics'])
            assert final['metrics']['cancelled'] >= 1, json.dumps(final['metrics'])
            assert final['metrics']['requested'] == len([e for e in final['events'] if e['type'] == 'request'])
            assert final['metrics']['served'] == len([e for e in final['events'] if e['type'] == 'complete'])
            for event in final['events']:
                if event['type'] in ('board', 'alight'):
                    assert event['speed'] < .25
            page.locator('#fleet-follow').click()
            advance(page, 200)
            assert snap()['cameraMode'] == 'follow'
            page.locator('#fleet-overview').click()
            advance(page, 200)
            assert snap()['cameraMode'] == 'city'

            # A small screen keeps the controls and scene within the viewport.
            page.set_viewport_size({'width': 390, 'height': 844})
            advance(page, 200)
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
            assert page.locator('#fleet-play').is_visible()
            page.screenshot(path=str(args.output / 'fleet-mobile.png'))
            assert not errors, errors
            browser.close()

        trace_path = args.output / 'fleet-trace.json'
        trace_path.write_text(json.dumps({'start': start, 'samples': samples, 'end': end, 'acceptance': final}, separators=(',', ':')) + '\n')
        receipt = {
            'scope': 'Authored arcade simulation of individual travel demand and a one-rider taxi fleet; not calibrated population data or autonomous-driving validation.',
            'sourceCommit': os.environ.get('RUSTDRIVING_SOURCE_SHA', 'uncommitted'),
            'seed': first['seed'], 'width': WIDTH, 'height': HEIGHT,
            'warmupSimulationSeconds': start['time'],
            'recordedSimulationSeconds': end['time'] - start['time'],
            'frameMilliseconds': FRAME_MS, 'frames': FRAME_COUNT, 'encoder': ENCODER,
            'normalPlaybackSpeed': True, 'stateWrites': 'none; UI start/pause and deterministic frame clock only',
            'sourceSha256': {path: sha(ROOT / path) for path in SOURCES},
            'traceSha256': sha(trace_path), 'frameSha256': frame_hashes,
            'startMetrics': start['metrics'], 'endMetrics': end['metrics'],
            'acceptanceMetrics': final['metrics'], 'visiblePeopleMax': max_people, 'visibleTaxisMax': max_taxis,
            'clipEvents': clip_events,
        }
        if args.record:
            receipt['gifSeconds'] = encode_gif(frames, args.record, WIDTH, HEIGHT, FRAME_MS)
            receipt['gifSha256'] = sha(args.record)
            poster = args.record.with_suffix('.png')
            with Image.open(frames / f'frame-{FRAME_COUNT // 2:04d}.png') as image:
                image.save(poster)
            receipt['posterSha256'] = sha(poster)
            args.record.with_suffix('.json').write_text(json.dumps(receipt, indent=2) + '\n')
        (args.output / 'fleet-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print(json.dumps({key: receipt[key] for key in ['warmupSimulationSeconds', 'recordedSimulationSeconds', 'endMetrics', 'acceptanceMetrics', 'visiblePeopleMax', 'visibleTaxisMax']}, indent=2))


if __name__ == '__main__':
    main()
