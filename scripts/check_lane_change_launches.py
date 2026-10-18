#!/usr/bin/env python3
"""Audit full-rate physical launch holds, braking room and post-stop observations."""
import argparse
import json
import math
from pathlib import Path
import check_hazards
from check_lane_change_resumes import digest, no_truth, physical_checks, point_segment, segment_distance, xy

CASES = {'rear': (30., 6.), 'rear-slow': (35., 4.), 'launch-blocked': (54., 0.), 'launch-recovery': (30., 6.)}


def course(request):
    length = request['distance_m']
    start = request['start']
    y0, a1 = start['position']['y']-3.5, math.tan(start['yaw'])*length
    # Independent power-basis solution with stationary initial curvature.
    a3, a4, a5 = -10*y0-6*a1, 15*y0+8*a1, -6*y0-3*a1
    count = math.ceil(length/.25)
    points = [(start['position']['x']+length*i/count,
               3.5+y0+a1*i/count+a3*(i/count)**3+a4*(i/count)**4+a5*(i/count)**5) for i in range(count+1)]
    points[0] = xy(start['position'])
    return points+[(80., 3.5), (160., 3.5)]


def progress(point, points):
    closest, selected, length = math.inf, 0., 0.
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        squared = dx*dx+dy*dy
        t = min(1., max(0., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/squared))
        error = math.hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)
        segment = math.sqrt(squared)
        if error < closest:
            closest, selected = error, length+t*segment
        length += segment
    return selected


def source_boundary(points):
    length = 0.
    for a, b in zip(points, points[1:]):
        segment = math.dist(a, b)
        assert abs(a[1]) <= .25+1e-8 and 1.5 <= a[0] <= 78.5
        if abs(b[1]) > .25:
            boundary = math.copysign(.25, b[1])
            return length+(boundary-a[1])/(b[1]-a[1])*segment
        length += segment
    raise AssertionError('no source-boundary crossing')


def audit_case(folder, case):
    run = json.loads((folder/'run/run.json').read_text())
    authored = json.loads((folder/'scenario.json').read_text())
    plan = json.loads((folder/'plan.json').read_text())
    replay = json.loads((folder/'replay/replay.json').read_text())
    frames, summary = run['frames'], run['summary']
    assert summary['passed'] and not summary['failures'] and summary['collisions'] == summary['road_violations'] == 0
    assert summary['reached_goal'] == (case != 'launch-blocked')
    assert authored['duration'] == 120. and authored['min_clearance_m'] == 1. and authored['cruise_speed'] == 2.
    assert authored['half_width'] == 1.75 and run['vehicle'] == {'wheelbase': 2.7, 'radius': 1.25, 'max_steer': .55}
    assert authored['lane_change'] == plan['lane_change_gate'] and run['route'] == plan['plan']['route']
    assert run['route']['points'][-1] == {'x': 160., 'y': 3.5}
    assert replay['verified'] and replay['ticks'] == len(frames) == summary['steps']
    x, speed = CASES[case]
    for f in frames:
        expected = [] if f['time']+1e-9 < 20. else [(52.+4.*max(0., f['time']-40.), 3.5)]
        if f['time']+1e-9 >= 49.:
            expected.append((x+speed*(f['time']-49.), 3.5))
        assert len(f['objects']) == len(expected)
        for actor, position in zip(f['objects'], expected):
            assert math.dist(xy(actor['position']), position) < 1e-8 and actor['radius'] == 1.1
    minimum, road_margin = physical_checks(frames)
    assert abs(minimum-summary['min_clearance']) < 1e-8
    records = [json.loads(line) for line in (folder/'run/sensors.jsonl').read_text().splitlines()]
    no_truth(records[0]['header']['config'])
    ticks = [r['tick'] for r in records if r['kind'] == 'tick']
    assert len(ticks) == len(frames)
    def state(f):
        return f['lane_change']['return_state']['resume_state']
    first = next(i for i, f in enumerate(frames) if state(f).get('launch_hold'))
    initial = state(frames[first])['launch_hold']
    request = state(frames[first])['request']
    points = course(request)
    start_s = progress(xy(ticks[first]['expected']['estimate']['pose']['position']), points)
    wheel = ticks[first]['input']['odometry']
    velocity = max(abs(ticks[first]['expected']['estimate']['speed']), abs(wheel['speed']))+.1
    deceleration = authored['motion_limits']['max_deceleration_m_s2']*.5
    required = .3*velocity+velocity*velocity/(2.*deceleration)+.1
    boundary = source_boundary(points)
    assert abs(initial['source_limit_s_m']-boundary) < 1e-9
    assert abs(initial['reaction_speed_m_s']-velocity) < 1e-12 and initial['deceleration_m_s2'] == deceleration
    assert abs(initial['planning_endpoint_s_m']-start_s-required-1.) < 1e-9
    assert initial['planning_endpoint_s_m'] <= boundary and initial['target_blocking_tracks']
    assert 49. <= initial['initiated_at'] < 50. and state(frames[first])['started_at'] < initial['initiated_at']
    held = []
    released = None
    for i, f in enumerate(frames[first:], first):
        retry = state(f)
        assert retry['request'] == request
        h = retry['launch_hold']
        assert h['initiated_at'] == initial['initiated_at'] and h['planning_endpoint_s_m'] == initial['planning_endpoint_s_m']
        if h['phase'] == 'Released':
            if released is None:
                released = i
            assert h['released_at'] == frames[released]['time']
        else:
            assert released is None and f['command']['acceleration'] <= -deceleration+1e-9
            assert abs(f['truth']['pose']['position']['y'])+1.25 <= 1.75
            assert progress(xy(f['truth']['pose']['position']), points) < boundary
            held.append(f)
    assert held
    assert all(b['truth']['speed'] <= a['truth']['speed']+1e-9 for a, b in zip(held, held[1:]))
    first_stop = next(i for i, f in enumerate(held) if f['truth']['speed'] <= .05)
    stopped = held[first_stop:]
    drift = max(math.dist(xy(f['truth']['pose']['position']), xy(stopped[0]['truth']['pose']['position'])) for f in stopped)
    duration = stopped[-1]['time']-stopped[0]['time']
    assert drift < .05 and duration >= 1.
    row = {'case': case, 'seed': summary['seed'], 'minimum_independent_swept_clearance_m': minimum,
           'physical_lane_margin_m': road_margin, 'hold_drift_m': drift, 'physical_launch_hold_s': duration,
           'initiated_at': initial['initiated_at'], 'independent_source_limit_s_m': boundary,
           'independent_stop_endpoint_s_m': start_s+required+1., 'full_truth_ticks': len(frames), 'replay_ticks': replay['ticks']}
    if case == 'launch-blocked':
        assert released is None and state(frames[-1])['launch_hold']['target_blocking_tracks']
    else:
        assert released is not None
        event = state(frames[released])['launch_hold']
        assert event['clear_since'] > event['after_fault_at'] >= event['stopped_at']
        stop_index = next(i for i, t in enumerate(ticks) if abs(t['input']['time']-event['stopped_at']) < 1e-9)
        measured = [t['input']['odometry'] for t in ticks[stop_index-2:stop_index+1]]
        assert len({m['stamp'] for m in measured}) == 3 and all(abs(m['speed']) <= .05 and abs(m['yaw_rate']) <= .01 for m in measured)
        scans = []
        for tick in ticks[:released+1]:
            o = tick['input']
            if o['time']+1e-9 < event['clear_since']:
                continue
            assert not tick['expected']['health'] and not o['lidar_failed']
            assert abs(o['odometry']['speed']) <= .05 and o['time']-o['odometry']['stamp'] <= .05+1e-9
            if o['lidar'] is not None:
                scans.append(o['lidar']['stamp'])
        assert len(set(scans)) == len(scans) >= 3 and scans[-1]-scans[0] >= 1.-1e-9
        assert min(scans) > event['after_fault_at'] and max(b-a for a, b in zip(scans, scans[1:])) <= .2+1e-9
        expected = ticks[released]['expected']
        crossing_end = state(frames[released])['connector_length_m']+1.25+.5
        last = next(i for i, p in enumerate(expected['trajectory']['points']) if progress(xy(p['position']), points) >= crossing_end)
        path = [xy(p['position']) for p in expected['trajectory']['points'][:last+1]]
        clearance = min((segment_distance(a, b, xy(c), xy(d))-1.25-p['radius']
                         for a, b in zip(path, path[1:]) for p in expected['predictions'] for c, d in zip(p['positions'], p['positions'][1:])), default=1000.)
        assert clearance >= 1.3-1e-9 and abs(clearance-event['crossing_preview_clearance_m']) < 1e-8
        if case == 'launch-recovery':
            assert event['after_fault_at'] >= 51. and event['clear_since'] > 51.
            assert all(t['input']['lidar_failed'] for t in ticks if 50. <= t['input']['time'] < 51.-1e-9)
        final = frames[-1]['truth']['pose']['position']
        assert frames[-1]['lane_change']['phase'] == 'Completed' and final['x'] >= 157. and abs(final['y']-3.5) < .25
        row.update(released_at=event['released_at'], stop_confirmed_at=event['stopped_at'], clear_since=event['clear_since'],
                   post_stop_clear_scans=len(scans), independent_crossing_preview_clearance_m=clearance)
    profiles = check_hazards.check_speed_profiles(folder/'run/sensors.jsonl')
    commands = check_hazards.control_metrics(folder/'run/sensors.jsonl')
    assert commands['max_normal_commanded_steering_rate_rad_s'] <= .7+1e-8
    row.update(summary=summary, profiles=profiles, commands=commands, passed=True,
               scenario_sha256=digest(folder/'scenario.json'), sensors_sha256=digest(folder/'run/sensors.jsonl'), run_sha256=digest(folder/'run/run.json'))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rows = [audit_case(args.input.resolve()/case, case) for case in CASES]
    report = {'scope': 'Four authored reference worlds; independent full-rate physical and acquisition audit. No general automotive validity.',
              'audit_source_sha256': digest(Path(__file__)), 'physics_oracle_sha256': digest(Path(__file__).with_name('check_lane_change_resumes.py')),
              'profile_oracle_sha256': digest(Path(check_hazards.__file__)), 'results': rows}
    if args.output:
        with args.output.open('x') as out:
            out.write(json.dumps(report, indent=2, sort_keys=True)+'\n')
    print(json.dumps([{k: r[k] for k in ['case', 'minimum_independent_swept_clearance_m', 'physical_launch_hold_s', 'passed']} for r in rows], indent=2))


if __name__ == '__main__':
    main()
