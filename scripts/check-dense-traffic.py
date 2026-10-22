#!/usr/bin/env python3
"""Independently audit the 24-vehicle, three-queue native demonstration."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from statistics import mean

from check_hazards import check_traffic

ROOT = Path(__file__).resolve().parent.parent
SCENARIO = ROOT / 'scenarios/traffic-dense-queue.json'


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def subset(expected, actual):
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(k in actual and subset(v, actual[k]) for k, v in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(expected) == len(actual) and all(subset(a, b) for a, b in zip(expected, actual))
    return type(expected) is type(actual) and expected == actual or isinstance(expected, (int, float)) and not isinstance(expected, bool) and type(actual) in (int, float) and expected == actual


def swept_distance(a, b, c, d):
    x, y = a['x']-b['x'], a['y']-b['y']
    dx, dy = c['x']-d['x']-x, c['y']-d['y']-y
    fraction = max(0., min(1., -(x*dx+y*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(x+fraction*dx, y+fraction*dy)


def motion_diversity(frames, specs, lanes):
    """Measure physical motion, retaining each car's initial longitudinal offset.

    Compare the same queue rank across lanes so a braking wave within three
    otherwise identical queues cannot disguise synchronized lane motion.
    Speeds come from position differences, independent of traffic telemetry.
    """
    speeds, displacements = [], []
    for before, after in zip(frames, frames[1:]):
        if not 5. <= after['time'] <= 20.:
            continue
        dt = after['time']-before['time']
        require(dt > 0., 'nonpositive motion observation interval')
        old = {a['id']: a['position']['x'] for a in before['objects']}
        new = {a['id']: a['position']['x'] for a in after['objects']}
        for row in zip(*lanes.values()):
            velocity = [(new[i]-old[i])/dt for i in row]
            travel = [new[i]-specs[i]['s'] for i in row]
            require(all(math.isfinite(v) for v in velocity+travel), 'nonfinite motion diversity evidence')
            speeds.append(max(velocity)-min(velocity))
            displacements.append(max(travel)-min(travel))
    require(bool(speeds), 'missing motion diversity observation window')
    speed_spread, travel_spread = mean(speeds), mean(displacements)
    require(speed_spread >= .3 and travel_spread >= 2.,
            'traffic motion is synchronized across lanes despite initial offsets or reported speeds')
    return dict(window_seconds=[5., 20.], matched_queue_ranks=8,
                position_derived_mean_cross_lane_speed_range_m_s=speed_spread,
                mean_cross_lane_travel_range_m=travel_spread,
                required_speed_range_m_s=.3, required_travel_range_m=2.)


def verify(run, replay):
    expected = json.loads(SCENARIO.read_text())
    require(subset(expected, run['scenario']), 'recorded scenario differs from the supplied fixture')
    require(run['backend'] == 'rne-dynamic-rapier-lidar', 'actual native dynamic recording required')
    summary, frames = run['summary'], run['frames']
    require(summary['passed'] and not summary['failures'] and summary['collisions'] == 0
            and summary['road_violations'] == 0 and summary['min_clearance'] >= 1., 'physical acceptance failed')
    require(summary['simulated_seconds'] == expected['duration']
            and summary['steps'] == len(frames) == round(expected['duration']/.05)+1,
            'complete declared timeline required')
    require(replay['verified'] is True and replay['ticks'] == len(frames), 'full sensor-only replay required')
    specs = run['scenario']['objects']
    ids = {i for i, spec in enumerate(specs) if spec.get('following')}
    lanes = {lane: [i for i in sorted(ids) if specs[i]['lateral'] == lane] for lane in [-7., 0., 7.]}
    require(len(ids) == 24 and all(len(group) == 8 for group in lanes.values()), 'all three eight-vehicle queues required')
    minimum_ego = math.inf
    minimum_pair = math.inf
    previous = None
    for index, frame in enumerate(frames):
        require(abs(frame['time']-index*.05) <= 1e-8, 'control tick omitted or reordered')
        objects, states = frame['objects'], frame.get('traffic', [])
        require(all(type(a['id']) is int for a in objects+states), 'actor identities must be integer IDs')
        require(len(objects) == len(specs) and {a['id'] for a in objects} == set(range(len(specs))), 'physical actor omitted or duplicated')
        require(len(states) == 24 and {a['id'] for a in states} == ids, 'reactive vehicle omitted or duplicated')
        by_id = {a['id']: a for a in objects}
        ego = frame['truth']
        require(all(math.isfinite(v) for v in [ego['pose']['position']['x'], ego['pose']['position']['y'],
                                              ego['pose']['yaw'], ego['speed']])
                and 0 <= ego['speed'] <= 12., 'ego exceeded finite native speed/pose bounds')
        require(abs(ego['pose']['position']['y'])+2.7+.4 <= run['route']['half_width'],
                'expanded display body left its authored road corridor')
        for actor in objects:
            spec = specs[actor['id']]
            require(all(math.isfinite(v) for v in [actor['position']['x'], actor['position']['y'], actor['radius']])
                    and actor['radius'] == spec['radius'], 'nonfinite or changed physical proxy')
            if index == 0 or actor['id'] not in ids:
                require(abs(actor['position']['x']-spec['s']) <= 1e-8
                        and abs(actor['position']['y']-spec['lateral']) <= 1e-8,
                        'initial or stationary actor differs from supplied geometry')
        for actor in states:
            spec = specs[actor['id']]
            require(0 <= actor['speed_m_s'] <= spec['speed']+.1, 'actor exceeded the demonstration speed bound')
            require(abs(by_id[actor['id']]['position']['y']-spec['lateral']) <= 1e-8
                    and by_id[actor['id']]['radius'] == spec['radius'], 'recorded lane or physical proxy changed')
        for group in lanes.values():
            require(all(by_id[a]['position']['x'] < by_id[b]['position']['x'] for a, b in zip(group, group[1:])), 'queue order changed')
        if previous:
            before = {a['id']: a for a in previous['objects']}
            for actor in objects:
                # 2.7 m covers the evaluated ego display mesh. Keep a further
                # .4 m inter-sample reserve; this is not contact simulation.
                gap = swept_distance(previous['truth']['pose']['position'], before[actor['id']]['position'],
                                     frame['truth']['pose']['position'], actor['position'])-2.7-actor['radius']-.4
                minimum_ego = min(minimum_ego, gap)
            for a, actor in enumerate(objects):
                for other in objects[a+1:]:
                    if actor['id'] not in ids and other['id'] not in ids:
                        continue
                    gap = swept_distance(before[actor['id']]['position'], before[other['id']]['position'],
                                         actor['position'], other['position'])-actor['radius']-other['radius']-.2
                    minimum_pair = min(minimum_pair, gap)
        previous = frame
    require(minimum_ego >= 1. and minimum_pair >= 1., 'expanded display/actor clearance floor failed')
    travel = min(a['route_s_m']-specs[a['id']]['s'] for a in frames[-1]['traffic'])
    require(travel >= 15., 'each of the 24 vehicles must actually travel at least fifteen metres')
    start = frames[-1]['time']
    for frame in reversed(frames):
        if frame['truth']['speed'] >= .1 or any(a['speed_m_s'] >= .05 for a in frame['traffic']):
            break
        start = frame['time']
    hold = frames[-1]['time']-start
    require(hold >= 5., 'all vehicles and ego must remain stopped together for five seconds')
    return dict(reactive_vehicles=24, queue_lanes=3, vehicles_per_lane=8,
                motion_diversity=motion_diversity(frames, specs, lanes),
                minimum_vehicle_travel_m=travel, complete_queue_hold_seconds=hold,
                minimum_expanded_ego_clearance_with_reserve_m=minimum_ego,
                minimum_actor_pair_clearance_with_reserve_m=minimum_pair,
                ego_display_radius_m=2.7, ego_inter_sample_reserve_m=.4, actor_inter_sample_reserve_m=.2)


def check_log(run, path):
    fields = ['time', 'estimate', 'tracks', 'predictions', 'trajectory', 'command', 'emergency', 'localization']
    with path.open() as stream:
        header = json.loads(next(stream))['header']
        require(header['config']['route'] == run['route'], 'displayed route differs from sensor-only configuration')
        index = 0
        for line in stream:
            record = json.loads(line)
            if record['kind'] != 'tick':
                continue
            require(index < len(run['frames']), 'extra sensor-only tick')
            expected = record['tick']['expected']
            frame = run['frames'][index]
            require(all(frame[k] == expected[k] for k in fields), 'rendered driver output differs from sensor-only trace')
            index += 1
    require(index == len(run['frames']), 'sensor-only trace omitted a displayed frame')
    return dict(bound_ticks=index, fields=fields, route_identical=True)


def mutations(run, replay, log):
    tests = [
        ('omit_vehicle', lambda x: x['frames'][0]['traffic'].pop()),
        ('duplicate_physical_actor', lambda x: x['frames'][0]['objects'].__setitem__(0, copy.deepcopy(x['frames'][0]['objects'][1]))),
        ('change_lane', lambda x: x['frames'][0]['objects'][0]['position'].__setitem__('y', 99.)),
        ('omit_tick', lambda x: x['frames'].pop(1)),
        ('move_proxy', lambda x: x['frames'][1]['objects'][0]['position'].update(x=x['frames'][1]['truth']['pose']['position']['x'], y=x['frames'][1]['truth']['pose']['position']['y'])),
        ('conceal_physical_failure', lambda x: x['summary'].__setitem__('collisions', 1)),
        ('missing_final_stop', lambda x: x['frames'][-1]['traffic'][0].__setitem__('speed_m_s', .2)),
        ('nonfinite_actor', lambda x: x['frames'][0]['objects'][0]['position'].__setitem__('x', math.nan)),
    ]
    rejected = []
    for name, mutate in tests:
        changed = copy.deepcopy(run)
        mutate(changed)
        try:
            verify(changed, replay)
        except (ValueError, KeyError):
            rejected.append(name)
        else:
            raise ValueError('corruption was accepted: ' + name)
    changed = copy.deepcopy(run)
    changed['frames'][0]['command'] = {}
    try:
        check_log(changed, log)
    except ValueError:
        rejected.append('rendered_command_differs_from_sensor_log')
    else:
        raise ValueError('a forged rendered command was accepted')
    changed = copy.deepcopy(run)
    specs = changed['scenario']['objects']
    lanes = {lane: [i for i, spec in enumerate(specs) if spec.get('following') and spec['lateral'] == lane]
             for lane in [-7., 0., 7.]}
    for frame in changed['frames']:
        objects = {a['id']: a for a in frame['objects']}
        for row in zip(*lanes.values()):
            reference = row[1]
            travel = objects[reference]['position']['x']-specs[reference]['s']
            for actor in row:
                objects[actor]['position']['x'] = specs[actor]['s']+travel
    try:
        # Keep the original, varied telemetry and all initial offsets. The
        # physical-position check must still reject copied lane trajectories.
        motion_diversity(changed['frames'], specs, lanes)
    except ValueError:
        rejected.append('synchronized_motion_despite_varied_telemetry')
    else:
        raise ValueError('copied lane motion was accepted')
    return rejected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, nargs='+', required=True, help='Complete native episode directories, including replay/replay.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = []
    for folder in args.input:
        run = json.loads((folder / 'run.json').read_text())
        replay = json.loads((folder / 'replay/replay.json').read_text())
        require(json.loads((folder / 'summary.json').read_text()) == run['summary'], 'external summary differs from recording')
        proof = verify(run, replay)
        proof['sensor_trace_binding'] = check_log(run, folder / 'sensors.jsonl')
        proof['traffic_kinematics_and_proximity'] = check_traffic(run, folder / 'sensors.jsonl', 'traffic-queue')
        rejected = mutations(run, replay, folder / 'sensors.jsonl')
        cases.append(dict(seed=run['summary']['seed'], summary=run['summary'], replay=replay,
                          independent=proof, corruptions_rejected=rejected,
                          evidence_sha256={name: digest(folder / name) for name in
                              ['run.json', 'sensors.jsonl', 'summary.json', 'replay/replay.json']}))
        print(f"seed {run['summary']['seed']}: all 24 vehicles, {replay['ticks']} replay ticks, three queues and {len(rejected)} corruptions verified", flush=True)
    require(len({c['seed'] for c in cases}) == len(cases), 'duplicate seed evidence')
    report = dict(schema_version=2, passed=True, scenario_sha256=digest(SCENARIO),
                  checker_sha256=digest(Path(__file__)), traffic_oracle_sha256=digest(ROOT / 'scripts/check_hazards.py'),
                  cases=cases, scope='Authored straight three-queue CPU simulation with physical circular proxies; no lane changes, arbitrary traffic negotiation, contact response or real-vehicle validation.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
