#!/usr/bin/env python3
"""Independently audit the three full-rate reference outputs of the resume demo."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import check_hazards


def xy(point):
    return point['x'], point['y']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def point_segment(p, a, b):
    d, q = (b[0]-a[0], b[1]-a[1]), (p[0]-a[0], p[1]-a[1])
    squared = d[0]**2 + d[1]**2
    t = min(1., max(0., (q[0]*d[0]+q[1]*d[1])/squared)) if squared else 0.
    return math.hypot(q[0]-t*d[0], q[1]-t*d[1])


def segment_distance(a, b, c, d):
    # Minimize the independent two-parameter squared-distance quadratic and
    # check its four boundaries, including degenerate and collinear segments.
    r, s, w = (b[0]-a[0], b[1]-a[1]), (d[0]-c[0], d[1]-c[1]), (a[0]-c[0], a[1]-c[1])
    def dot(u, v):
        return u[0]*v[0]+u[1]*v[1]
    aa, bb, cc, dd, ee = dot(r, r), dot(r, s), dot(s, s), dot(r, w), dot(s, w)
    denominator = aa*cc-bb*bb
    distances = [point_segment(a, c, d), point_segment(b, c, d), point_segment(c, a, b), point_segment(d, a, b)]
    if denominator > 0.:
        t, u = (bb*ee-cc*dd)/denominator, (aa*ee-bb*dd)/denominator
        if 0. <= t <= 1. and 0. <= u <= 1.:
            distances.append(math.hypot(w[0]+t*r[0]-u*s[0], w[1]+t*r[1]-u*s[1]))
    return min(distances)


def no_truth(value):
    if isinstance(value, dict):
        assert not set(value) & {'truth', 'objects', 'world_motion', 'active_from', 'moving_from'}
        for child in value.values():
            no_truth(child)
    elif isinstance(value, list):
        for child in value:
            no_truth(child)


def physical_checks(frames):
    minimum, road_margin, previous = 1000., 1000., None
    for f in frames:
        p = xy(f['truth']['pose']['position'])
        # Each region is a convex subset of the unchanged physical lane union.
        # Both endpoints fitting the same eroded region covers the full step.
        def margins(q):
            return [min(q[0]-1.25, 78.75-q[0], q[1]+.5, 4.-q[1]),
                    .5-point_segment(q, (0., 0.), (80., 0.)),
                    .5-point_segment(q, (0., 3.5), (160., 3.5))]
        old_margins = margins(xy(previous['truth']['pose']['position'])) if previous else margins(p)
        road_margin = min(road_margin, max(min(a, b) for a, b in zip(old_margins, margins(p))))
        old = {o['id']: o for o in previous['objects']} if previous else {}
        for actor in f['objects']:
            q = xy(actor['position'])
            relative = p[0]-q[0], p[1]-q[1]
            distance = math.hypot(*relative)
            if actor['id'] in old:
                pp, qq = xy(previous['truth']['pose']['position']), xy(old[actor['id']]['position'])
                distance = point_segment((0., 0.), (pp[0]-qq[0], pp[1]-qq[1]), relative)
            minimum = min(minimum, distance-1.25-actor['radius'])
        previous = f
    assert minimum >= 1.-1e-8 and road_margin >= -1e-8
    return minimum, road_margin


def retry_checks(frames, ticks, first):
    index, f = first
    state = f['lane_change']['return_state']['resume_state']
    r = state['request']
    assert r['start'] == ticks[index]['expected']['estimate']['pose']
    assert r['source_start'] == {'x': 0., 'y': 0.} and r['source_end'] == {'x': 80., 'y': 0.}
    assert r['target_start'] == {'x': 0., 'y': 3.5} and r['target_end'] == {'x': 80., 'y': 3.5}
    assert r['source_width_m'] == r['target_width_m'] == 3.5
    assert state['source_blocking_tracks'] == state['target_blocking_tracks'] == []
    assert state['clear_since'] > state['stopped_at'] and f['time']-state['clear_since'] >= 1.-1e-9
    scans, motion_stamps = [], []
    for tick in ticks[:index+1]:
        observed, expected = tick['input'], tick['expected']
        if observed['time']+1e-9 < state['clear_since']:
            continue
        assert not expected['health'] and not observed['lidar_failed']
        motion = observed['odometry']
        assert observed['time']-motion['stamp'] <= .05+1e-9
        assert abs(motion['speed']) <= .05 and abs(motion['yaw_rate']) <= .01
        assert abs(expected['estimate']['speed']) <= .1
        scan = observed['lidar']
        if scan is not None and scan['stamp']+1e-9 >= state['clear_since']:
            scans.append(scan['stamp'])
            motion_stamps.append(motion['stamp'])
    assert len(set(scans)) == len(scans) >= 3 and len(set(motion_stamps)) == len(scans)
    assert min(scans) > state['stopped_at'] and max(b-a for a, b in zip(scans, scans[1:])) <= .2+1e-9
    assert scans[-1]-scans[0] >= 1.-1e-9
    length = r['distance_m']
    assert 10. <= length <= 30. and r['start']['position']['x']+length >= 55.
    # Solve the six endpoint constraints directly in the power basis. Initial
    # second derivative is zero for this stationary-pose connector.
    start = r['start']
    y0, a1 = start['position']['y']-3.5, math.tan(start['yaw'])*length
    a3, a4, a5 = -10*y0-6*a1, 15*y0+8*a1, -6*y0-3*a1
    def lateral(u):
        return 3.5+y0+a1*u+a3*u**3+a4*u**4+a5*u**5
    maximum = 0.
    for i in range(10_001):
        u = i/10_000
        dy = (a1+3*a3*u*u+4*a4*u**3+5*a5*u**4)/length
        ddy = (6*a3*u+12*a4*u*u+20*a5*u**3)/(length*length)
        maximum = max(maximum, abs(ddy)/(1+dy*dy)**1.5)
        assert -.5-1e-9 <= lateral(u) <= 4.+1e-9
    assert maximum <= state['curvature_bound_m_inv']+1e-12 <= math.tan(.55)/2.7/1.005+1e-12
    count = math.ceil(length/.25)
    curve = [(start['position']['x']+length*i/count, lateral(i/count)) for i in range(count+1)]
    connector = sum(math.dist(a, b) for a, b in zip(curve, curve[1:]))
    assert abs(connector-state['connector_length_m']) < 1e-10
    path = [xy(p['position']) for p in ticks[index]['expected']['trajectory']['points']]
    assert math.dist(path[0], xy(start['position'])) < 1e-12
    predictions = ticks[index]['expected']['predictions']
    observed = min((segment_distance(a, b, xy(c), xy(d))-1.25-p['radius']
                    for a, b in zip(path, path[1:]) for p in predictions
                    for c, d in zip(p['positions'], p['positions'][1:])), default=1000.)
    assert observed >= 1.3-1e-9 and abs(observed-state['observed_clearance_m']) < 1e-8
    course = curve+[(80., 3.5), (160., 3.5)]
    margin = min(.5-min(point_segment(xy(f['truth']['pose']['position']), a, b)
                       for a, b in zip(course, course[1:])) for f in frames[index:])
    assert margin >= -1e-8
    for later in frames[index:]:
        retry = later['lane_change']['return_state']['resume_state']
        assert retry['request'] == r and retry['started_at'] == f['time']
    return {'resume_started_at': f['time'], 'post_stop_clear_scans': len(scans),
            'selected_connector_m': length, 'dense_max_curvature_m_inv': maximum,
            'independent_preview_clearance_m': observed, 'tracking_margin_m': margin}


def audit(root):
    rows = []
    for case in ['release', 'blocked', 'reblocked']:
        folder = root/case
        authored = json.loads((folder/'scenario.json').read_text())
        plan = json.loads((folder/'plan.json').read_text())
        run = json.loads((folder/'run/run.json').read_text())
        replay = json.loads((folder/'replay/replay.json').read_text())
        frames, summary = run['frames'], run['summary']
        assert summary['passed'] and not summary['failures']
        assert summary['collisions'] == summary['road_violations'] == 0
        assert summary['reached_goal'] == (case == 'release')
        assert authored['duration'] == 120. and authored['min_clearance_m'] == 1. and authored['cruise_speed'] == 2.
        assert authored['half_width'] == 1.75 and run['vehicle'] == {'wheelbase': 2.7, 'radius': 1.25, 'max_steer': .55}
        assert authored['lane_change'] == plan['lane_change_gate'] and run['route'] == plan['plan']['route']
        assert run['route']['points'][-1] == {'x': 160., 'y': 3.5}
        assert replay['verified'] and replay['ticks'] == len(frames) == summary['steps']
        for f in frames:
            expected = [] if f['time']+1e-9 < 20. else [(52.+(0. if case == 'blocked' else 4.)*max(0., f['time']-40.), 3.5)]
            if case == 'reblocked' and f['time']+1e-9 >= 54.:
                expected.append((66., 3.5))
            assert len(f['objects']) == len(expected)
            for actor, position in zip(f['objects'], expected):
                assert math.dist(xy(actor['position']), position) < 1e-8 and actor['radius'] == 1.1
        minimum, road_margin = physical_checks(frames)
        assert abs(minimum-summary['min_clearance']) < 1e-8
        records = [json.loads(line) for line in (folder/'run/sensors.jsonl').read_text().splitlines()]
        no_truth(records[0]['header']['config'])
        ticks = [r['tick'] for r in records if r['kind'] == 'tick']
        assert len(ticks) == len(frames)
        held = [f for f in frames if f['lane_change']['phase'] == 'Aborted']
        assert held and any(f['lane_change']['phase'] == 'Aborting' for f in frames)
        assert max(f['truth']['pose']['position']['y'] for f in frames[:frames.index(held[0])]) > .1
        position = xy(held[0]['truth']['pose']['position'])
        assert all(abs(f['truth']['pose']['position']['y'])+1.25 <= 1.75 and abs(f['truth']['speed']) <= .05 for f in held)
        drift = max(math.dist(xy(f['truth']['pose']['position']), position) for f in held)
        hold = held[-1]['time']-held[0]['time']
        assert hold >= 1. and drift < .05
        resumed = [(i, f) for i, f in enumerate(frames) if f['lane_change']['phase'] == 'Resuming']
        row = {'case': case, 'seed': summary['seed'], 'full_truth_ticks': len(frames), 'replay_ticks': replay['ticks'],
               'minimum_independent_swept_clearance_m': minimum, 'physical_lane_margin_m': road_margin,
               'physical_source_hold_s': hold, 'hold_drift_m': drift}
        if case == 'blocked':
            assert not resumed and frames[-1]['lane_change']['phase'] == 'Aborted'
            assert frames[-1]['lane_change']['return_state']['resume_state']['decision'] == 'TrafficBlocked'
        else:
            assert resumed and resumed[0][1]['time'] >= 41.
            assert sum(f['lane_change']['phase'] == 'Resuming' and frames[i-1]['lane_change']['phase'] != 'Resuming'
                       for i, f in enumerate(frames) if i) == 1
            row.update(retry_checks(frames, ticks, resumed[0]))
            if case == 'release':
                final = frames[-1]['truth']['pose']['position']
                assert frames[-1]['lane_change']['phase'] == 'Completed' and final['x'] >= 157. and abs(final['y']-3.5) <= .25
            else:
                assert frames[-1]['truth']['speed'] <= .05 and frames[-1]['truth']['pose']['position']['x'] < 64.
                assert any(t['expected']['tracks'] and t['expected']['command']['acceleration'] < 0. for t in ticks if t['input']['time'] >= 54.)
        profiles = check_hazards.check_speed_profiles(folder/'run/sensors.jsonl')
        commands = check_hazards.control_metrics(folder/'run/sensors.jsonl')
        assert commands['max_normal_commanded_steering_rate_rad_s'] <= .7+1e-8
        row.update(summary=summary, profiles=profiles, commands=commands, passed=True,
                   scenario_sha256=digest(folder/'scenario.json'), sensors_sha256=digest(folder/'run/sensors.jsonl'), run_sha256=digest(folder/'run/run.json'))
        rows.append(row)
    return {'scope': 'Three authored reference worlds; independent full-tick audit. No general automotive validity.',
            'audit_source_sha256': digest(Path(__file__)), 'profile_oracle_sha256': digest(Path(check_hazards.__file__)), 'results': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = audit(args.input.resolve())
    if args.output:
        with args.output.open('x') as out:
            out.write(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps([{k: row[k] for k in ['case', 'minimum_independent_swept_clearance_m', 'full_truth_ticks', 'passed']} for row in report['results']], indent=2))


if __name__ == '__main__':
    main()
