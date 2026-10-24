"""Exercise and record building exits, city time, nightlife and physical train transfers."""
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
WIDTH, HEIGHT, FRAME_MS = 1280, 800, 100
SOURCES = [
    'web/src/fleet.js', 'web/src/city-life.js', 'web/src/fleet-places.js',
    'web/src/fleet-routing.js', 'web/src/fleet-view.js', 'web/src/fleet.css',
    'web/src/city.js', 'web/src/junctions.js', 'web/src/traffic-turns.js', 'web/src/urban-roads.js',
    'web/src/main.js', 'web/tests/fleet_smoke.py', 'web/tests/fleet.test.mjs',
    'web/tests/city-life.test.mjs', 'web/tests/fleet-places.test.mjs',
    'web/tests/browser_clock.py', 'web/tests/gif_capture.py', 'integrations/playground/src/lib.rs',
]
SEGMENTS = {
    'morning': [(0, 14, 'office'), (64, 18, 'city')],
    'night': [(0, 12, 'nightlife'), (96, 20, 'station')],
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fleet_url(url, period):
    parts = urlsplit(url)
    query = dict(parse_qsl(parts.query))
    query.update(mode='fleet', period=period)
    return urlunsplit((parts.scheme, parts.netloc, parts.path, urlencode(query), parts.fragment))


def run_case(browser, args, period):
    output = args.output / period
    output.mkdir(parents=True, exist_ok=True)
    errors, samples, cuts, frame_hashes = [], [], [], {}
    page = browser.new_page(viewport={'width': WIDTH, 'height': HEIGHT}, device_scale_factor=1)
    page.on('pageerror', lambda error: (errors.append(str(error)), print('PAGE ERROR:', error, flush=True)))
    control_clock(page)
    snapshot = lambda: page.evaluate('window.rustDrivingFleet.snapshot()')
    captured = 0

    def tick(milliseconds):
        before = snapshot()
        advance(page, milliseconds)
        state = snapshot()
        assert not errors, errors
        assert not state['paused'], 'rendering/visibility unexpectedly paused the city'
        assert abs((state['time'] - before['time']) - milliseconds / 1000 * before['playbackRate']) < .055, {
            'failure': 'stalled or skipped simulation frames', 'before': before['time'], 'after': state['time']}
        return state

    def until(target):
        # Bound every pre-roll: a frame error cannot turn this into an endless CI wait.
        for _ in range(math.ceil(max(0, target - snapshot()['time']) * 2) + 12):
            remaining = target - snapshot()['time']
            if remaining < .02:
                return
            tick(min(1000, max(5, round(remaining * 1000 / snapshot()['playbackRate']))))
        raise AssertionError('Did not reach bounded pre-roll target: ' + str(target))

    try:
        page.goto(fleet_url(args.url, period), wait_until='networkidle')
        wait_state(page, 'window.rustDrivingFleet?.snapshot().ready', 10000)
        advance(page, 200)
        first = snapshot()
        assert first['paused'] and first['time'] == 0
        assert first['period'] == period and first['ambientCars'] >= 30
        assert len(first['taxis']) == (12 if period == 'night' else 8)
        assert len(first['purposeColors']) == 5
        page.locator('#fleet-play').click()

        if args.proof_only:
            until(4)
            page.screenshot(path=str(output / ('office.png' if period == 'morning' else 'nightlife.png')))
            until(8)
            state = snapshot()
            facility = 'office' if period == 'morning' else 'nightlife'
            assert any(e['type'] == 'building-exit' and e.get('building') == facility for e in state['events'])
            if period == 'night':
                page.locator('#fleet-station').click()
                page.locator('#fleet-rate').click()
                assert snapshot()['playbackRate'] == 4
                until(35)
                state = snapshot()
                assert any(e['type'] == 'train-arrive' for e in state['railEvents'])
                assert state['metrics']['railAlighted'] >= 1
                page.screenshot(path=str(output / 'train.png'))
            (output / 'proof.json').write_text(json.dumps(state, indent=2) + '\n')
            print('CITY PROOF', period, state['time'], state['metrics'], flush=True)
            return {'period': period, 'metrics': state['metrics'], 'lighting': state['lighting']}

        with tempfile.TemporaryDirectory(prefix='rustdriving-city-life-') as temp:
            frames = Path(temp)
            max_people, max_taxis = 0, 0
            for target, seconds, camera in SEGMENTS[period]:
                if target > snapshot()['time'] + 1:
                    if snapshot()['playbackRate'] != 4: page.locator('#fleet-rate').click()
                until(target)
                if snapshot()['playbackRate'] != 1: page.locator('#fleet-rate').click()
                assert snapshot()['playbackRate'] == 1
                page.locator('#fleet-' + ('overview' if camera == 'city' else camera)).click()
                start = snapshot()['time']
                first_frame = captured
                for _ in range(seconds * 1000 // FRAME_MS):
                    state = tick(FRAME_MS)
                    assigned = [p['taxi'] for p in state['people'] if p['taxi']]
                    assert len(assigned) == len(set(assigned)), 'exclusive taxi assignments'
                    assert all(math.isfinite(t[key]) for t in state['taxis'] for key in ('x', 'y', 'speed'))
                    samples.append(state)
                    max_people = max(max_people, state['visiblePeople'])
                    max_taxis = max(max_taxis, state['visibleTaxis'])
                    if args.record:
                        path = frames / f'frame-{captured:04d}.png'
                        page.screenshot(path=str(path))
                        frame_hashes[path.name] = sha(path)
                    elif captured in (30, 100, 200, 319):
                        page.screenshot(path=str(output / f'city-{captured:04d}.png'))
                    captured += 1
                end = snapshot()['time']
                assert abs((end - start) - seconds) < .03
                cuts.append({'firstFrame': first_frame, 'frames': captured - first_frame,
                             'camera': camera, 'simulationStart': start, 'simulationEnd': end})
                print('CITY SEGMENT', period, camera, start, end, snapshot()['metrics'], flush=True)
            if snapshot()['playbackRate'] != 4: page.locator('#fleet-rate').click()
            until(120)
            if snapshot()['playbackRate'] != 1: page.locator('#fleet-rate').click()
            final = snapshot()
            assert final['metrics']['served'] >= 3, final['metrics']
            assert final['metrics']['cancelled'] >= 1, final['metrics']
            assert final['metrics']['requested'] == sum(e['type'] == 'request' for e in final['events'])
            assert final['metrics']['served'] == sum(e['type'] == 'complete' for e in final['events'])
            facility = 'office' if period == 'morning' else 'nightlife'
            exits = [e for e in final['events'] if e['type'] == 'building-exit' and e.get('building') == facility]
            assert len(exits) >= 3
            for event in exits:
                departure = next(e for e in final['events'] if e['type'] == 'building-depart' and e['person'] == event['person'])
                assert event['time'] - departure['time'] > .5 and event['doorOpen'] >= .92
                requested = next((e for e in final['events'] if e['type'] == 'request' and e['person'] == event['person']), None)
                assert requested is None or requested['time'] > event['time']
            for event in final['events']:
                if event['type'] in ('board', 'alight'):
                    assert event['speed'] < .25
            assert final['metrics']['railAlighted'] >= 4
            rail_requests = [e for e in final['events'] if e['type'] == 'request' and e.get('source') == 'train']
            assert rail_requests, 'train passengers must actually reach the taxi curb'
            for event in rail_requests:
                alighted = next(e for e in final['events'] if e['type'] == 'train-alight' and e['person'] == event['person'])
                assert event['time'] - alighted['time'] > 40 and event['height'] == 0
            if period == 'night':
                assert final['metrics']['onDuty'] == 12 and not final['clock']['railOpen']
                assert any(e['type'] == 'train-arrive' and e['lastService'] for e in final['railEvents'])
                assert all(e['time'] < 80 for e in final['railEvents'] if e['type'] == 'train-spawn')
            else:
                assert final['metrics']['onDuty'] == 8
                assert max_taxis >= 6
            assert max_people >= 3

            page.locator('#fleet-play').click()
            stopped = snapshot()
            advance(page, 400)
            assert snapshot()['time'] == stopped['time']
            page.locator('#fleet-play').click()
            page.locator('#fleet-follow').click()
            tick(200)
            assert snapshot()['cameraMode'] == 'follow'
            page.locator('#fleet-overview').click()
            tick(200)
            assert snapshot()['cameraMode'] == 'city'
            page.set_viewport_size({'width': 390, 'height': 844})
            tick(200)
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
            assert page.locator('#fleet-play').is_visible() and page.locator('#fleet-period').is_visible()
            page.screenshot(path=str(output / 'mobile.png'))
            assert not errors, errors
            if period == 'morning':
                with page.expect_navigation(wait_until='networkidle'):
                    page.locator('#fleet-period').select_option('dusk')
                wait_state(page, "window.rustDrivingFleet?.snapshot().period === 'dusk'", 10000)
                advance(page, 200)
                dusk = snapshot()
                assert dusk['paused'] and dusk['time'] == 0
                assert len(dusk['taxis']) == 10
                assert .82 < dusk['lighting']['ambient'] < first['lighting']['ambient']
                page.screenshot(path=str(output / 'dusk.png'))

            trace_path = output / 'trace.json'
            trace_path.write_text(json.dumps({'cuts': cuts, 'samples': samples, 'acceptance': final}, separators=(',', ':')) + '\n')
            receipt = {
                'scope': 'Authored optional city simulation: building doors, individual taxi demand and North Line transfers; not a calibrated population or autonomous-driving validation.',
                'sourceCommit': os.environ.get('RUSTDRIVING_SOURCE_SHA', 'uncommitted'),
                'period': period, 'seed': first['seed'], 'width': WIDTH, 'height': HEIGHT,
                'cityClockRate': 15, 'physicalMovementPlaybackRate': 1,
                'editing': 'Two normal-speed recorded intervals separated by an unrecorded simulation pre-roll (UI 4x speed); recordings use UI 1x. No agent poses or counters are written by the recorder.',
                'segments': cuts, 'recordedSimulationSeconds': sum(c['simulationEnd'] - c['simulationStart'] for c in cuts),
                'frameMilliseconds': FRAME_MS, 'frames': captured, 'encoder': ENCODER,
                'sourceSha256': {path: sha(ROOT / path) for path in SOURCES}, 'traceSha256': sha(trace_path),
                'frameSha256': frame_hashes, 'acceptanceMetrics': final['metrics'],
                'visiblePeopleFrustumMax': max_people, 'visibleTaxisFrustumMax': max_taxis,
                'buildingExits': exits, 'railEvents': final['railEvents'], 'railTaxiRequests': rail_requests,
            }
            if args.record:
                target = args.record if period == 'morning' else args.record.with_name('nightlife-train-demo.gif')
                receipt['gifSeconds'] = encode_gif(frames, target, WIDTH, HEIGHT, FRAME_MS)
                receipt['gifSha256'] = sha(target)
                with Image.open(frames / 'frame-0060.png') as image:
                    image.save(target.with_suffix('.png'))
                receipt['posterSha256'] = sha(target.with_suffix('.png'))
                target.with_suffix('.json').write_text(json.dumps(receipt, indent=2) + '\n')
            (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
            return {'period': period, 'metrics': final['metrics'], 'lighting': final['lighting']}
    except Exception as error:
        print('CITY FAILURE', period, repr(error), errors, flush=True)
        page.screenshot(path=str(output / 'failure.png'))
        (output / 'failure.json').write_text(json.dumps({'error': repr(error), 'pageErrors': errors}, indent=2) + '\n')
        raise
    finally:
        page.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:4173/rust_driving/')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--record', type=Path)
    parser.add_argument('--proof-only', action='store_true', help='Short door/platform render acceptance before recording')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=shutil.which('chromium') or shutil.which('chromium-browser'),
                                   headless=True, args=['--no-sandbox', '--disable-dev-shm-usage',
                                                        '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
        results = [run_case(browser, args, period) for period in ('morning', 'night')]
        assert results[0]['lighting']['ambient'] > results[1]['lighting']['ambient']
        browser.close()
    (args.output / 'summary.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
