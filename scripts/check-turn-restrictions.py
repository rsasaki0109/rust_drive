#!/usr/bin/env python3
"""Execute legal-turn routes in real reference/native plants and replay all sensors.

Authored maps/OSM syntax are test fixtures, not independently surveyed road rules.
Physical checks reconstruct the existing 20 Hz circular/linear-segment domain;
no native contact response, lane negotiation or between-substep guarantee.
"""
import argparse
import copy
import gzip
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

ROOT = Path(__file__).resolve().parent.parent

CASES = ('turn-no-main', 'turn-only-detour', 'turn-arrival-memory', 'osm-turn')
EXPECTED = ('approach', 'detour', 'east-exit')
EXPECTED_OSM = ('osm-way-10-0-forward', 'osm-way-30-0-forward', 'osm-way-40-0-forward')


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_fingerprint():
    files = [ROOT / name for name in ('Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml')]
    files += list((ROOT / 'crates').glob('*/Cargo.toml')) + list((ROOT / 'crates').glob('**/*.rs'))
    files += [ROOT / 'integrations/rne' / name for name in ('Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml', 'rne-revision.txt')]
    files += list((ROOT / 'integrations/rne/src').glob('*.rs'))
    digest = hashlib.sha256()
    for file in sorted(set(files)):
        digest.update(str(file.relative_to(ROOT)).encode() + b'\0' + file.read_bytes() + b'\0')
    return digest.hexdigest()


def invoke(args):
    result = subprocess.run([str(a) for a in args], cwd=ROOT, capture_output=True, text=True)
    return result.returncode, result.stderr[-2000:]


def xy(p):
    return p['x'], p['y']


def point_segment(point, a, b):
    delta = b[0] - a[0], b[1] - a[1]
    norm = delta[0] ** 2 + delta[1] ** 2
    t = min(1, max(0, sum((point[i] - a[i]) * delta[i] for i in range(2)) / norm)) if norm else 0
    return math.dist(point, tuple(a[i] + t * delta[i] for i in range(2)))


def legal_path(network, ids, start, goal, closed):
    edges = {e['id']: e for e in network['edges']}
    previous = None
    at = start
    points = []
    for identity in ids:
        edge = edges[identity]
        require(edge['from'] == at and identity not in closed, 'disconnected or closed route edge')
        for rule in network['turn_restrictions']:
            if rule['from_edge'] != previous:
                continue
            require(rule['via_node'] == at, 'restriction via node differs from actual junction')
            allowed = identity != rule['to_edge'] if rule['kind'] == 'no' else identity == rule['to_edge']
            require(allowed, 'selected route violates turn restriction')
        points.extend(edge['points'][1:] if points else edge['points'])
        at = edge['to']
        previous = identity
    require(at == goal, 'route does not reach requested goal')
    return points


def verify_run(output, expected_edges):
    run = json.loads((output / 'run.json').read_text())
    summary = run['summary']
    nav = run['scenario']['navigation']
    ids = run['navigation']['edge_ids']
    require(tuple(ids) == expected_edges, 'wrong legal detour')
    points = legal_path(nav['network'], ids, nav['start'], nav['goal'], nav['closed_edges'])
    require(points == run['route']['points'], 'driving geometry differs from legal route')
    require(summary['passed'] and summary['reached_goal'] and summary['collisions'] == 0
            and summary['road_violations'] == 0 and summary['final_speed'] <= .2,
            'existing physical goal acceptance failed')
    floor = run['scenario'].get('min_clearance_m', 0.0)
    radius = run['vehicle']['radius']
    clearance = math.inf
    reserve = math.inf
    frames = run['frames']
    for i, frame in enumerate(frames):
        p = xy(frame['truth']['pose']['position'])
        require(all(math.isfinite(value) for value in p), 'non-finite physical ego position')
        offset = min(point_segment(p, xy(a), xy(b)) for a, b in zip(points, points[1:]))
        reserve = min(reserve, run['route']['half_width'] - radius - offset)
        for actor in frame['objects']:
            require(all(math.isfinite(value) for value in xy(actor['position'])), 'non-finite physical actor position')
            clearance = min(clearance, math.dist(p, xy(actor['position'])) - radius - actor['radius'])
            if i:
                before = frames[i - 1]
                old = next(a for a in before['objects'] if a['id'] == actor['id'])
                a = tuple(x - y for x, y in zip(xy(before['truth']['pose']['position']), xy(old['position'])))
                b = tuple(x - y for x, y in zip(p, xy(actor['position'])))
                clearance = min(clearance, point_segment((0, 0), a, b) - radius - actor['radius'])
    require(reserve >= -1e-7, 'actual recorded circle left selected-route corridor')
    require(clearance >= floor, 'independent recorded segment clearance below unchanged fixture floor')
    goal = next(n['position'] for n in nav['network']['nodes'] if n['id'] == nav['goal'])
    goal_distance = math.dist(xy(frames[-1]['truth']['pose']['position']), xy(goal))
    require(goal_distance <= 2, 'actual vehicle did not stop at mapped goal')
    ticks = 0
    with (output / 'sensors.jsonl').open() as stream:
        config = json.loads(next(stream))['header']['config']
        require(config['navigation']['network'] == nav['network'], 'restricted map missing from replay header')
        require(config['route']['points'] == points, 'replay route differs from restricted-map search')
        for line in stream:
            record = json.loads(line)
            if record['kind'] != 'tick':
                continue
            tick = record['tick']
            status = tick['expected']['navigation']
            require(status['active_edges'] == ids and status['phase'] == 'Following', 'illegal active navigation path')
            require('truth' not in tick['input'] and 'objects' not in tick['input'], 'operational truth input')
            ticks += 1
    replay = json.loads((output / 'replay/replay.json').read_text())
    require(replay['verified'] and replay['ticks'] == ticks == summary['steps'], 'incomplete sensor replay')
    return {'passed': True, 'summary': summary, 'edge_ids': ids, 'replay_ticks': ticks,
            'recorded_segment_min_clearance_m': None if math.isinf(clearance) else clearance,
            'clearance_floor_m': floor, 'recorded_corridor_min_reserve_m': reserve,
            'goal_distance_m': goal_distance, 'map_search_in_replay': True}


def mutation_checks(output, cli):
    results = []
    for label in ('removed_rule', 'invalid_via', 'expected_forbidden_path'):
        target = output / (label + '.jsonl')
        with (output / 'sensors.jsonl').open() as src, target.open('w') as dst:
            head = json.loads(next(src))
            rules = head['header']['config']['navigation']['network']['turn_restrictions']
            if label == 'removed_rule':
                rules.clear()
            elif label == 'invalid_via':
                rules[0]['via_node'] = 'missing-node'
            dst.write(json.dumps(head) + '\n')
            changed = False
            for line in src:
                if label == 'expected_forbidden_path' and not changed:
                    item = json.loads(line)
                    if item['kind'] == 'tick':
                        item['tick']['expected']['navigation']['active_edges'] = ['approach', 'main', 'east-exit']
                        line = json.dumps(item) + '\n'
                        changed = True
                dst.write(line)
        replay_dir = output / ('rejected-' + label)
        code, error = invoke([cli, 'replay', '--log', target, '--output', replay_dir])
        require(code == 2 and not (replay_dir / 'replay.json').exists(), 'corrupt restricted-map replay accepted')
        results.append({'mutation': label, 'exit_code': code, 'diagnostics': error, 'input_sha256': sha(target)})
    return results


def archive_case(output):
    archive = output.with_suffix('.tar.gz')
    files = {str(p.relative_to(output)): sha(p) for p in output.rglob('*') if p.is_file()}
    require(not any(p.is_symlink() for p in output.rglob('*')), 'evidence symlink')
    with tarfile.open(archive, 'w:gz', compresslevel=6) as bundle:
        bundle.add(output, arcname=output.name)
    verified = {}
    with tarfile.open(archive, 'r:gz') as bundle:
        for member in bundle:
            if member.isfile():
                h = hashlib.sha256()
                with bundle.extractfile(member) as stream:
                    while data := stream.read(1024 * 1024):
                        h.update(data)
                verified[str(Path(member.name).relative_to(output.name))] = h.hexdigest()
    require(files == verified, 'archive member hash mismatch')
    with gzip.open(archive, 'rb') as stream:
        while stream.read(1024 * 1024):
            pass
    require(files == {str(p.relative_to(output)): sha(p) for p in output.rglob('*') if p.is_file()}, 'raw evidence changed during archive')
    result = {'path': str(archive), 'sha256': sha(archive), 'bytes': archive.stat().st_size,
              'files_sha256': files, 'gzip_crc_verified': True}
    shutil.rmtree(output)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['reference', 'native', 'both'], default='both')
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 7, 42])
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/turn-restrictions')
    parser.add_argument('--report', type=Path)
    parser.add_argument('--compact', action='store_true')
    args = parser.parse_args()
    suffix = '.exe' if sys.platform == 'win32' else ''
    cli = ROOT / 'target/release' / ('rustdriving' + suffix)
    native = ROOT / 'integrations/rne/target/release' / ('rustdriving-rne' + suffix)
    require(cli.is_file() and (args.backend == 'reference' or native.is_file()), 'build selected locked release binaries first')
    fixtures = [ROOT / 'scenarios' / (name + '.json') for name in CASES[:3]] + [ROOT / 'maps/osm/turn-restriction-fixture.json']
    source = source_fingerprint()
    checker = sha(Path(__file__))
    frozen = {str(p.relative_to(ROOT)): sha(p) for p in fixtures}
    report = {'schema_version': 1, 'passed': False, 'complete': False,
              'scope': 'authored unconditional node-via turn rules; planar driving, recorded 20 Hz linear/circular physical checks',
              'source_fingerprint_sha256': source, 'checker_sha256': checker,
              'source_fingerprint_scope': 'Rust sources, manifests, locks, toolchains and native pin; selected fixtures hashed separately',
              'fixture_sha256': frozen, 'rne_revision': (ROOT / 'integrations/rne/rne-revision.txt').read_text().strip(),
              'binary_sha256': {'reference': sha(cli)}, 'cases': [], 'negative_cases': []}
    if args.backend != 'reference':
        pin = subprocess.check_output(['git', '-C', str(ROOT.parent / 'RobotNativeEngine'), 'rev-parse', 'HEAD'], text=True).strip()
        require(pin == report['rne_revision'], 'native checkout differs from pin')
        report['binary_sha256']['native'] = sha(native)
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.report or args.output / 'report.json'
    report_path.parent.mkdir(parents=True, exist_ok=True)
    save = lambda: report_path.write_text(json.dumps(report, indent=2) + '\n')
    save()
    imported = args.output / 'imported-scenario.json'
    code, error = invoke([cli, 'import-osm', '--input', fixtures[-1], '--output', args.output / 'imported-map.json',
                          '--origin-lat', 0, '--origin-lon', 0, '--default-half-width', 5.5, '--scenario-output', imported,
                          '--start', 'osm-node-1', '--goal', 'osm-node-4', '--duration', 70, '--cruise-speed', 6])
    require(code == 0, 'authored OSM import failed: ' + error)
    scenarios = {name: ROOT / 'scenarios' / (name + '.json') for name in CASES[:3]} | {'osm-turn': imported}
    blocked = copy.deepcopy(json.loads(scenarios['turn-only-detour'].read_text()))
    blocked['navigation']['closed_edges'] = ['detour']
    blocked_path = args.output / 'unreachable.json'
    blocked_path.write_text(json.dumps(blocked, indent=2) + '\n')
    for backend in (['reference', 'native'] if args.backend == 'both' else [args.backend]):
        out = args.output / ('unreachable-' + backend)
        command = [cli, 'run'] if backend == 'reference' else [native, '--plant', 'dynamic']
        code, error = invoke(command + ['--scenario', blocked_path, '--seed', 7, '--output', out])
        require(code == 2 and not (out / 'run.json').exists(), 'unreachable request fell back through prohibited turn')
        report['negative_cases'].append({'backend': backend, 'kind': 'closed_only_allowed_turn', 'exit_code': code, 'diagnostics': error})
        for name, scenario in scenarios.items():
            for seed in args.seeds:
                output = args.output / f'{backend}-{name}-seed-{seed}'
                require(not output.exists(), 'use a new evidence directory; existing recordings are preserved')
                command = [cli, 'run'] if backend == 'reference' else [native, '--plant', 'dynamic']
                code, error = invoke(command + ['--scenario', scenario, '--seed', seed, '--output', output])
                require(code == 0, 'physical episode failed; retained raw capture: ' + error)
                code, error = invoke([cli, 'replay', '--log', output / 'sensors.jsonl', '--output', output / 'replay'])
                require(code == 0, 'full replay failed: ' + error)
                row = {'backend': backend, 'case': name, 'seed': seed} | verify_run(output, EXPECTED_OSM if name == 'osm-turn' else EXPECTED)
                row['mutations'] = mutation_checks(output, cli) if seed == args.seeds[0] else []
                if args.compact:
                    row['archive'] = archive_case(output)
                else:
                    row['raw_sha256'] = {str(p.relative_to(output)): sha(p) for p in output.rglob('*') if p.is_file()}
                report['cases'].append(row)
                save()
                print(f'{backend} {name} seed {seed}: legal route, physical goal and {row["replay_ticks"]} replay ticks verified', flush=True)
    require(source == source_fingerprint() and checker == sha(Path(__file__))
            and frozen == {str(p.relative_to(ROOT)): sha(p) for p in fixtures}, 'sources or fixtures changed during evaluation')
    report.update(passed=True, complete=True, cases_verified=len(report['cases']),
                  replay_ticks_verified=sum(row['replay_ticks'] for row in report['cases']))
    save()
    print(f'{report_path}: passed=True, {report["cases_verified"]} actual closed-loop episodes')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, StopIteration, subprocess.SubprocessError) as error:
        print('turn restriction check: ' + str(error), file=sys.stderr)
        sys.exit(2)
