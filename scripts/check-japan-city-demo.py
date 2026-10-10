#!/usr/bin/env python3
"""Audit a Japanese city recording with actual cross traffic and construction.

Actor IDs describe simulator evidence only. Operational perception is unlabeled
LiDAR; moving obstacles remain upright capsules and conservative planar circles.
Pedestrian/cyclist behavior is prescribed motion, not semantic recognition or
human decision modeling. The research ego body has no contact-force response.
The opt-in f64 capsule query uses actual synchronized physical ECS colliders,
including capsules missed by native GJK. It expands no shapes and keeps ground
and other collider hit distances native. The independent oracle is unchanged.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('vru_ground_oracle', ROOT/'scripts/check-ground-scenes.py')
g = importlib.util.module_from_spec(spec)
spec.loader.exec_module(g)
SCENARIO = ROOT/'scenarios/native-japan-city-demo.json'
SCENE = ROOT/'scenes/japan-city-construction.json'
SCENE_SHA = 'e1179f645bf1a21359bc0ce094dd58d473d51c85578e167e663aa2cd26e7a8b0'
SCENARIO_SHA = '2bc1c6e0d425c23cbb827f702cea03669a331906df11c0cdd29d76d641fbdff5'
ACTOR_COUNT = 23
PAIR_COUNT = ACTOR_COUNT * (ACTOR_COUNT - 1) // 2
# Authored cross-street extents are an explicit fixture contract, not inferred
# from whether the recorded episode happened to pass an ego-center coordinate.
JUNCTIONS = [(19, 13.8, 15., 3.4), (20, 36., 36., 1.35)]


def supplied_fields(actual, expected):
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(k in actual and supplied_fields(actual[k], v) for k, v in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(supplied_fields(a, b) for a, b in zip(actual, expected))
    return actual == expected


def verify_signals(run, evidence, log):
    """City stop-after-green case; the old permanent-red fixture oracle is unchanged."""
    require = g.require
    header, ticks = g.rays.load_log(log)
    signals = run['scenario']['traffic_signals']
    require(header['config']['stop_lines'] == [s['stop_line'] for s in signals]
            and not any(k in header['config'] for k in ['traffic_signals', 'phases', 'signal_dropout_windows']),
            'signal schedules entered the operational configuration')
    phase = lambda signal, now: next(p['color'] for p in reversed(signal['phases']) if p['from'] <= now + 1e-9)
    latest, accepted = None, 0
    crossed, crossings = set(), []
    red_margin, observed_margin = math.inf, math.inf
    npc_margin, npc_decisions = math.inf, 0
    queue_hold_start, queue_hold_duration, queue_hold_interval = None, 0., None
    previous_front = None
    frames = run['frames']
    require(len(ticks) == len(frames) == 921, 'signal proof omitted an actual control tick')
    for index, (frame, tick) in enumerate(zip(frames, ticks)):
        now, sample = frame['time'], tick['input'].get('traffic_signal')
        require(abs(now-index*.05) < 1e-8 and abs(now-tick['input']['time']) < 1e-8,
                'signal control/acquisition clock changed')
        dropout = any(w['from']-1e-9 <= now < w['until']-1e-9 for w in run['scenario'].get('signal_dropout_windows', []))
        require(bool(sample) == (index % 4 == 0 and not dropout), 'signal observation cadence/dropout changed')
        # NPC metadata describes a decision at the previous control tick. Its
        # observed snapshot must therefore be checked before receiving this tick.
        for traffic in frame['traffic']:
            identity = traffic['id']
            policy = run['scenario']['objects'][identity].get('following', {}).get('signal_control')
            if policy is None:
                continue
            telemetry = traffic.get('signal_control')
            actor = next(actor for actor in frame['objects'] if actor['id'] == identity)
            front = actor['position']['x'] + policy['front_extent_m']
            for signal in signals:
                line = signal['stop_line']['route_s_m']
                initial_front = run['scenario']['objects'][identity]['s'] + policy['front_extent_m']
                if initial_front < line and phase(signal, now) != 'Green':
                    npc_margin = min(npc_margin, line - front)
                    require(line-front >= policy['stop_margin_m']-1e-8, 'controlled NPC physical front crossed the true red stop margin')
            if telemetry is None:
                continue
            npc_decisions += 1
            decision_time = now - .05
            require(index > 0 and abs(telemetry['decision_stamp']-decision_time) < 1e-8
                    and abs(telemetry['physical_front_stamp']-now) < 1e-8,
                    'NPC signal decision/integrated physical front timestamps changed')
            fresh = latest is not None and decision_time-latest['stamp'] <= .5+1e-9
            color = next((state['color'] for state in latest['states'] if state['id'] == telemetry['id']), 'Unknown') if fresh else 'Unknown'
            stamp_matches = (telemetry['observation_stamp'] is None and latest is None) or (
                telemetry['observation_stamp'] is not None and latest is not None
                and abs(telemetry['observation_stamp']-latest['stamp']) < 1e-8)
            require(stamp_matches
                    and telemetry['color'] == color and telemetry['fresh'] == fresh
                    and telemetry['permissive'] == (fresh and color == 'Green')
                    and telemetry['front_extent_m'] == policy['front_extent_m']
                    and telemetry['stop_margin_m'] == policy['stop_margin_m']
                    and abs(telemetry['physical_front_s_m']-front) < 1e-8
                    and not telemetry['crossed_nonpermissive'], 'NPC policy used unavailable signals or inconsistent physical evidence')
            line = next(signal['stop_line']['route_s_m'] for signal in signals if signal['stop_line']['id'] == telemetry['id'])
            require(telemetry['stop_line_s_m'] == line
                    and abs(telemetry['remaining_stop_distance_m']-(line-policy['stop_margin_m']-front)) < 1e-8,
                    'NPC physical front/stopline evidence is inconsistent')
        if sample:
            require(sample['stamp'] == now
                    and sample['states'] == [{'id': s['stop_line']['id'], 'color': phase(s, now)} for s in signals],
                    'observed infrastructure state differs from the true sampled phase')
            latest, accepted = sample, accepted + 1
        fresh = latest is not None and now-latest['stamp'] <= .5+1e-9
        status = tick['expected']['traffic_controls']
        require(not status['fault'] and status['last_accepted_stamp'] == (latest['stamp'] if latest else None)
                and frame['traffic_controls'] == status, 'ego accepted signal diagnostics differ from actual observations')
        front = frame['truth']['pose']['position']['x'] + run['vehicle']['radius']
        for signal in signals:
            identity, line = signal['stop_line']['id'], signal['stop_line']['route_s_m']
            real = phase(signal, now)
            color = next(state['color'] for state in latest['states'] if state['id'] == identity) if fresh else 'Unknown'
            require(next(state['color'] for state in status['signals'] if state['id'] == identity) == color,
                    'ego released green without fresh accepted infrastructure observation')
            if identity not in crossed and real != 'Green':
                red_margin = min(red_margin, line-front)
            if identity not in crossed and color != 'Green':
                observed_margin = min(observed_margin, line-front)
            if (previous_front is None or previous_front < line) and front >= line:
                require(identity not in crossed and real == color == 'Green', 'ego physical front crossed without true and observed green')
                crossed.add(identity)
                crossings.append({'id': identity, 'time_s': now, 'observed_color': color})
        previous_front = front
        lead = next(actor for actor in frame['objects'] if actor['id'] == 0)
        lead_status = next(actor for actor in frame['traffic'] if actor['id'] == 0)
        lead_signal = lead_status.get('signal_control')
        holding = (signals[0]['stop_line']['id'] in crossed and phase(signals[1], now) != 'Green'
                   and frame['truth']['speed'] <= .1 and lead_status['speed_m_s'] <= .1
                   and lead_signal is not None and not lead_signal['permissive']
                   and 0.5-1e-8 <= signals[1]['stop_line']['route_s_m']-lead_signal['physical_front_s_m'] <= .55
                   and front < lead['position']['x']-lead['radius'])
        if holding:
            if queue_hold_start is None:
                queue_hold_start = now
            duration = now-queue_hold_start
            if duration > queue_hold_duration:
                queue_hold_duration, queue_hold_interval = duration, [queue_hold_start, now]
        else:
            queue_hold_start = None
    require(len(crossed) == len(signals) == 2 and accepted > 0 and red_margin >= 1 and observed_margin >= 1
            and npc_decisions > 0 and npc_margin >= .5-1e-8,
            'city signal crossing/red physical margin acceptance failed')
    require(abs(run['summary']['signal_min_stopline_margin_m']-red_margin) < 1e-8,
            'independent ego nonpermissive margin differs from native summary')
    continuous_margin, visible_front_margin = math.inf, math.inf
    for sample in evidence['body_guard']['motion_samples']:
        now = sample['time']
        pose = sample['pose']
        x = pose['position']['x']
        extent = 2.14*abs(math.cos(pose['yaw'])) + 1.1*abs(math.sin(pose['yaw']))
        for signal in signals:
            if phase(signal, now) != 'Green':
                line = signal['stop_line']['route_s_m']
                continuous_margin = min(continuous_margin, line-x-run['vehicle']['radius']-12*.005/2)
                visible_front_margin = min(visible_front_margin, line-x-extent-12*.005/2)
    require(continuous_margin >= 1 and visible_front_margin >= 1,
            '200 Hz circumscribed/full SI front violated continuous red-light 1 m clearance')
    npc_continuous_margin = math.inf
    for first, second in zip(frames, frames[1:]):
        dt = second['time']-first['time']
        for identity in [0, 8, 9]:
            spec = run['scenario']['objects'][identity]
            policy = spec['following']['signal_control']
            positions = [next(actor['position']['x'] for actor in frame['objects'] if actor['id'] == identity)
                         for frame in [first, second]]
            for signal in signals:
                line = signal['stop_line']['route_s_m']
                if spec['s']+policy['front_extent_m'] < line and phase(signal, first['time']) != 'Green':
                    # Endpoint Lipschitz envelope covers the full interval,
                    # including a green transition at its endpoint.
                    upper = (sum(positions)+spec['speed']*dt)/2 + policy['front_extent_m']
                    npc_continuous_margin = min(npc_continuous_margin, line-upper)
    require(npc_continuous_margin >= 0, 'one controlled NPC may cross red between actual samples')
    require(queue_hold_duration >= 8.-1e-8,
            'ego did not continuously stop for at least 8 seconds behind the actually stopped red-light lead')
    return {'signals': 2, 'accepted_snapshots': accepted, 'control_ticks': len(ticks),
            'stopline_crossings': crossings, 'min_true_nonpermissive_margin_m': red_margin,
            'min_observed_nonpermissive_margin_m': observed_margin,
            'continuous_ego_circle_red_margin_bound_m': continuous_margin,
            'continuous_ego_full_si_front_red_margin_bound_m': visible_front_margin,
            'npc_decisions_verified': npc_decisions, 'min_npc_true_red_front_margin_m': npc_margin,
            'continuous_npc_red_front_margin_bound_m': npc_continuous_margin,
            'npc_front_extents_m': {str(i): run['scenario']['objects'][i]['following']['signal_control']['front_extent_m'] for i in [0, 8, 9]},
            'ego_stopped_behind_red_queue_interval_s': queue_hold_interval,
            'continuous_ego_stopped_behind_red_queue_duration_s': queue_hold_duration,
            'scope': 'two forward-route signals; opposing/service lanes have no mapped traffic signal',
            'snapshot_reconstruction_verified': True, 'schedule_labels_absent_from_pipeline': True, 'passed': True}


def verify_painted_lane(evidence):
    """Full visible SI mirror envelope; separate from the unchanged physical guard."""
    samples = evidence['body_guard']['motion_samples']
    minimum = math.inf
    half_length, half_width, lane_half_width = 2.14, 1.1, 1.8
    for first, second in zip(samples, samples[1:]):
        dt = second['time']-first['time']
        angle = abs(math.remainder(second['pose']['yaw']-first['pose']['yaw'], 2*math.pi))
        extents = [abs(sample['pose']['position']['y']) + half_length*abs(math.sin(sample['pose']['yaw']))
                   + half_width*abs(math.cos(sample['pose']['yaw'])) for sample in [first, second]]
        # Same 200 Hz translation/angular envelope as the physical body oracle.
        reserve = lane_half_width-max(extents)-12*dt/2-math.hypot(half_length, half_width)*angle/2
        minimum = min(minimum, reserve)
    g.require(len(samples) == 9201 and minimum >= 0, 'actual full visible vehicle envelope left the painted lane')
    return {'full_length_m': 4.28, 'mirror_width_m': 2.2, 'painted_lane_half_width_m': 1.8,
            'physical_corridor_half_width_m': 3.0, 'continuous_minimum_painted_lane_reserve_m': minimum,
            'motion_samples': len(samples), 'display_audit_only': True}


def verify_actors(run, evidence, log):
    acquisitions = evidence['acquisitions']
    require = g.require
    require(evidence['operating_mode'] == 'lidar3d_ground_body', 'VRU demo changed native body/ground mode')
    require(evidence.get('precise_capsule_rays') is True
            and evidence.get('capsule_query_refinement') == {
                'kind': 'native_rapier_non_capsule_hits_f64_physical_capsule_recovery',
                'geometry': 'synchronized physical ECS capsule and rigid local offset',
                'native_pin_unchanged': True, 'shape_expansion_m': 0.0,
                'recovers_native_false_negatives': True,
                'maximum_capsules_per_acquisition': 1024,
                'other_shapes': 'native hit distance and surface unchanged',
                'ordering': 'physical capsule and native non-capsule distance then entity index; one return per capsule'},
            'city capture lacks the bounded synchronized physical-capsule query contract')
    require(len(acquisitions) == 461, 'city demo changed acquisition count/deadline')
    _, ticks = g.rays.load_log(log)
    beams = g.directions(evidence['lidar3d'])
    actor_counts = {str(i): {'measured_returns': 0, 'preserved_returns': 0} for i in range(ACTOR_COUNT)}
    for index, acquisition in enumerate(acquisitions):
        time = acquisition['time']
        actors = {actor['id']: actor for actor in acquisition['objects']}
        require(set(actors) == set(range(ACTOR_COUNT)), 'actual stable car/pedestrian/cyclist IDs missing')
        fixture = run['scenario']
        for identity, actor_spec in enumerate(fixture['objects']):
            if actor_spec.get('following') is not None:
                continue  # Stateful following is independently speed-bounded below.
            motion_time = min(time, actor_spec.get('moving_until', time))
            elapsed = max(motion_time - max(actor_spec['active_from'], actor_spec['moving_from']), 0.)
            expected = (max(0., min(fixture['road_length'], actor_spec['s'] + actor_spec['speed'] * elapsed)),
                        actor_spec['lateral'] + actor_spec['lateral_speed'] * elapsed)
            require(math.dist(g.scenes.xy(actors[identity]['position']), expected) < 1e-7,
                    'actual actor motion differs from the supplied route-frame schedule')
        require([actors[i]['radius'] for i in range(ACTOR_COUNT)] == [spec['radius'] for spec in fixture['objects']], 'physical actor radius changed')
        tick = ticks[index * 2]
        removed, _ = g.check_plane(tick['input']['lidar3d'], tick['expected']['ground'])
        pose = acquisition['pose']
        origin = (*g.scenes.xy(pose['position']), evidence['lidar3d']['mount_height_m'])
        c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
        for point_index, measured in enumerate(acquisition['cloud_3d']['returns']):
            ordinal = measured['ray_index']
            value = acquisition['cloud_3d']['ranges_m'][ordinal]
            dx, dy, dz = beams[ordinal]
            direction = (c * dx - s * dy, s * dx + c * dy, dz)
            candidates = [(distance, actor['id']) for actor in actors.values()
                          if (distance := g.rays.capsule_ray(origin, direction, actor)) is not None]
            nearest, identity = min(candidates, default=(math.inf, None))
            ground = min((distance for box in evidence['scene']['ground_cuboids']+evidence['scene']['static_cuboids']
                          if (distance := g.rays.box_ray(origin, direction, box)) is not None), default=math.inf)
            # This stricter subset excludes ambiguous GJK boundary beams already
            # accounted for by the imported independent physical ray oracle.
            if identity is not None and nearest < ground and abs(value - nearest) <= .06:
                actor_counts[str(identity)]['measured_returns'] += 1
                actor_counts[str(identity)]['preserved_returns'] += not removed[point_index]
    require(all(actor_counts[str(i)]['measured_returns'] > 0
                and actor_counts[str(i)]['preserved_returns'] > 0 for i in [0, 1, 4, 6, 17, 18]),
            'a critical crossing or owner/dog actor lacks actual measured and preserved LiDAR evidence')
    # Visible display actors need not be visible to the ego laser. Score each
    # actual ID honestly; the two queued cars can be entirely hidden by the lead.
    unobserved = [i for i in range(ACTOR_COUNT) if actor_counts[str(i)]['measured_returns'] == 0]
    for identity in range(ACTOR_COUNT):
        row = actor_counts[str(identity)]
        row['observed'] = row['measured_returns'] > 0
        row['removed_by_measured_ground_fit'] = row['measured_returns'] - row['preserved_returns']
        row['acquisitions_geometrically_in_sensor_range'] = 0
        row['sampled_in_range_candidate_beams'] = 0
        row['blocked_candidate_beams'] = 0
    for acquisition in acquisitions:
        pose = acquisition['pose']
        origin = (*g.scenes.xy(pose['position']), evidence['lidar3d']['mount_height_m'])
        c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
        actors = {actor['id']: actor for actor in acquisition['objects']}
        for identity, actor in actors.items():
            if math.dist(origin[:2], g.scenes.xy(actor['position'])) - actor['radius'] <= 45:
                actor_counts[str(identity)]['acquisitions_geometrically_in_sensor_range'] += 1
        for identity in unobserved:
            actor = actors[identity]
            row = actor_counts[str(identity)]
            for dx, dy, dz in beams:
                direction = (c * dx - s * dy, s * dx + c * dy, dz)
                distance = g.rays.capsule_ray(origin, direction, actor)
                if distance is None or not .2 <= distance <= 45:
                    continue
                row['sampled_in_range_candidate_beams'] += 1
                nearer = [value for other in actors.values() if other['id'] != identity
                          and (value := g.rays.capsule_ray(origin, direction, other)) is not None]
                nearer += [value for box in evidence['scene']['ground_cuboids']+evidence['scene']['static_cuboids']
                           if (value := g.rays.box_ray(origin, direction, box)) is not None]
                row['blocked_candidate_beams'] += min(nearer, default=math.inf) < distance - .06
    for row in actor_counts.values():
        row['acquisitions_outside_sensor_range'] = len(acquisitions) - row['acquisitions_geometrically_in_sensor_range']
        row['observability_scope'] = 'measured native returns' if row['observed'] else (
            'fully occluded in every sampled in-range candidate beam'
            if row['sampled_in_range_candidate_beams'] > 0
            and row['blocked_candidate_beams'] == row['sampled_in_range_candidate_beams']
            else 'outside sensor range or unobserved discrete-beam coverage; no detection claimed')
    samples = evidence['motion_samples']
    cursor = 0
    minimum = {str(i): math.inf for i in range(ACTOR_COUNT)}
    speeds = {i: math.hypot(spec['speed'], spec['lateral_speed']) for i, spec in enumerate(run['scenario']['objects'])}
    def motion_bound(identity, interval_start):
        spec = run['scenario']['objects'][identity]
        # The independently reconstructed prescribed position above proves the
        # fixed destination after moving_until. Use that exact zero-motion
        # contract only for intervals starting after the stop; never infer a
        # smaller bound merely from similar measured endpoint positions.
        if spec.get('following') is None and spec.get('moving_until') is not None and interval_start >= spec['moving_until']:
            return 0.
        return speeds[identity]
    for first, second in zip(acquisitions, acquisitions[1:]):
        dt = second['time'] - first['time']
        previous = {actor['id']: actor for actor in first['objects']}
        for actor in second['objects']:
            identity = actor['id']
            require(math.dist(g.scenes.xy(actor['position']), g.scenes.xy(previous[identity]['position'])) <= motion_bound(identity, first['time']) * dt + 1e-7,
                    'actual actor exceeded the bound used by the continuous clearance proof')
    for sample in samples:
        while cursor + 1 < len(acquisitions) and acquisitions[cursor + 1]['time'] <= sample['time'] + 1e-9:
            cursor += 1
        acquisition = acquisitions[cursor]
        elapsed = max(0., sample['time'] - acquisition['time'])
        for actor in acquisition['objects']:
            identity = actor['id']
            # Previous actual acquisition plus a declared actor-speed bound;
            # the extra half-substep reserve covers motion between 200 Hz samples.
            speed_bound = motion_bound(identity, acquisition['time'])
            margin = speed_bound * elapsed + (12. + speed_bound) * .005 / 2
            reserve = math.dist(sample['position'], g.scenes.xy(actor['position'])) - run['vehicle']['radius'] - actor['radius'] - margin
            minimum[str(identity)] = min(minimum[str(identity)], reserve)
    require(all(math.isfinite(value) and value >= 1. for value in minimum.values()),
            'independent continuous circumscribed-circle clearance fell below the unchanged 1 m floor')
    pair_bounds = {}
    for first, second in zip(acquisitions, acquisitions[1:]):
        dt = second['time'] - first['time']
        a = {actor['id']: actor for actor in first['objects']}
        b = {actor['id']: actor for actor in second['objects']}
        for i in range(ACTOR_COUNT):
            for j in range(i + 1, ACTOR_COUNT):
                d0 = math.dist(g.scenes.xy(a[i]['position']), g.scenes.xy(a[j]['position']))
                d1 = math.dist(g.scenes.xy(b[i]['position']), g.scenes.xy(b[j]['position']))
                speed_bound = motion_bound(i, first['time']) + motion_bound(j, first['time'])
                # At any time t, distance >= max(d0-v*t,d1-v*(dt-t)).
                # Its minimum is the intersection when inside the interval;
                # this bound covers the entire interval, not just sampled times.
                crossing = (d0 - d1 + speed_bound * dt) / (2 * speed_bound) if speed_bound else 0.
                crossing = min(dt, max(0., crossing))
                distance_bound = max(d0 - speed_bound * crossing, d1 - speed_bound * (dt - crossing))
                reserve = distance_bound - a[i]['radius'] - a[j]['radius']
                key = f'{i}-{j}'
                pair_bounds[key] = min(pair_bounds.get(key, math.inf), reserve)
    require(len(pair_bounds) == PAIR_COUNT and all(math.isfinite(value) and value >= 1. for value in pair_bounds.values()),
            'one continuous actor-pair clearance fell below the unchanged 1 m floor')
    crossings = []
    for identity, crossing_time, maximum_ego_x in [(1, 2. + 5./1.2, 14.), (4, 14. + 5./1.2, 34.), (6, 16. + 5./1.1, 40.)]:
        start = next(actor for actor in acquisitions[0]['objects'] if actor['id'] == identity)
        end = next(actor for actor in acquisitions[-1]['objects'] if actor['id'] == identity)
        require(start['position']['y'] * end['position']['y'] < 0
                and abs(start['position']['y']) > 3. and abs(end['position']['y']) > 3.,
                'one actual pedestrian failed to cross the ego corridor')
        crossing = min(acquisitions, key=lambda a: abs(a['time'] - crossing_time))
        pedestrian = next(actor for actor in crossing['objects'] if actor['id'] == identity)
        require(abs(pedestrian['position']['y']) < .1 and crossing['pose']['position']['x'] < maximum_ego_x,
                'ego failed to remain behind a time-separated crossing with physical reserve')
        crossings.append({'id': identity, 'time_s': crossing['time'], 'lateral_m': pedestrian['position']['y'],
                          'ego_x_m': crossing['pose']['position']['x']})
    frames = run['frames']
    def frame(time):
        return min(frames, key=lambda f: abs(f['time'] - time))
    require(any(f['truth']['speed'] <= .2 for f in frames if 3 <= f['time'] <= 7),
            'actual ego never stopped for the early crossing')
    require(max(f['truth']['speed'] for f in frames if 8 <= f['time'] <= 16) >= 1
            and max(f['truth']['speed'] for f in frames if 24 <= f['time'] <= 34) >= 1
            and frame(34)['truth']['pose']['position']['x'] - frame(24)['truth']['pose']['position']['x'] >= 8,
            'actual ego did not resume and make meaningful progress')
    lead = lambda time: next(actor for actor in frame(time)['objects'] if actor['id'] == 0)
    require(math.dist(g.scenes.xy(lead(20)['position']), g.scenes.xy(lead(23)['position'])) < .05
            and lead(32)['position']['x'] - lead(24)['position']['x'] >= 8,
            'actual signal-controlled lead did not stop before red and resume after green')
    dog_separations = []
    for acquisition in acquisitions:
        actors = {actor['id']: actor for actor in acquisition['objects']}
        separation = math.dist(g.scenes.xy(actors[17]['position']), g.scenes.xy(actors[18]['position']))
        require(abs(separation - 1.8) < 1e-8, 'actual owner/dog pair separation changed')
        require(actors[17]['position']['x'] == actors[18]['position']['x'], 'owner/dog path timing differs')
        dog_separations.append(separation)
    require(run['scenario']['objects'][0]['lateral'] == 0 and run['scenario']['objects'][0]['speed'] > 0
            and all(run['scenario']['objects'][i]['lateral'] == -8 and run['scenario']['objects'][i]['speed'] < 0 for i in [3, 7, 10]),
            'actual main-road traffic changed from the supplied left-side routes')
    for identity in [3, 7, 10]:
        actual = [next(actor for actor in acquisition['objects'] if actor['id'] == identity) for acquisition in acquisitions]
        require(all(abs(actor['position']['y']+8) < 1e-9 for actor in actual)
                and actual[-1]['position']['x'] < actual[0]['position']['x']
                and all(second['position']['x'] <= first['position']['x']+1e-9 for first, second in zip(actual, actual[1:])),
                'actual recorded opposing route centroids/directions do not establish left-side traffic')
    return {'stable_display_ids': {'0': 'lead_car', '1': 'walking_pedestrian', '2': 'cyclist',
                                  '3': 'oncoming_car', '4': 'walking_pedestrian', '5': 'oncoming_cyclist',
                                  '6': 'walking_pedestrian', '7': 'oncoming_car', '8': 'car', '9': 'car',
                                  '10': 'oncoming_truck', '11': 'truck', '12': 'cyclist', '13': 'oncoming_cyclist',
                                  '14': 'elder', '15': 'child', '16': 'parent_stroller_group',
                                  '17': 'dog_walker', '18': 'dog', '19': 'crossing_sedan', '20': 'crossing_hatchback', '21': 'construction_worker', '22': 'construction_truck'},
            'motion_scope': 'prescribed crossings and bicycle/opposing motion; bounded stateful lead following; no human decision model',
            'per_actor_lidar_evidence': actor_counts,
            'minimum_continuous_circle_clearance_bound_m': minimum,
            'minimum_continuous_actor_pair_clearance_bound_m': pair_bounds,
            'actor_motion_bound_scope': 'configured speed until verified prescribed moving_until; exact zero afterward; stateful followers retain configured speed bound',
            'pedestrian_crossings': crossings,
            'stop_resume_verified': True,
            'ego_progress_at_7s_m': frame(7)['truth']['pose']['position']['x'],
            'ego_progress_at_24s_m': frame(24)['truth']['pose']['position']['x'],
            'ego_progress_at_34s_m': frame(34)['truth']['pose']['position']['x'],
            'left_hand_main_road': {'road_center_y_m': -4, 'ego_forward_lane_y_m': 0, 'opposing_lane_y_m': -8,
                                   'actual_opposing_centroids_and_negative_x_motion_verified': True},
            'dog_pair': {'owner_id': 17, 'dog_id': 18, 'minimum_center_distance_m': min(dog_separations),
                         'maximum_center_distance_m': max(dog_separations),
                         'proxy_scope': 'upright native capsule: display dog shape/height is different; no quadruped dynamics'},
            'body_actor_contact_physics': False, 'semantic_recognition': False}


def verify_junctions_and_construction(run, evidence, physical, signals):
    """Use actual recorded positions; display roads cannot establish traversal."""
    require = g.require
    samples = evidence['body_guard']['motion_samples']
    # Queued vehicle display meshes have a calibrated SI footprint larger than
    # their sensing capsules. The policy's front extent conservatively encloses
    # the symmetric sedan/hatchback/van mesh length; straight +X route motion is
    # independently verified here. No mesh is substituted for a physical proxy.
    stationary_npc_checks = 0
    npc_minimum_junction_reserve = math.inf
    for frame in run['frames']:
        for identity in [0, 8, 9]:
            actor = next(a for a in frame['objects'] if a['id'] == identity)
            telemetry = next(a for a in frame['traffic'] if a['id'] == identity)
            require(abs(actor['position']['y']) < 1e-8,
                    'queued NPC left the straight route used by the SI longitudinal envelope')
            if telemetry['speed_m_s'] > .1:
                continue
            extent = run['scenario']['objects'][identity]['following']['signal_control']['front_extent_m']
            for _, _, center, half_width in JUNCTIONS:
                rear, front = actor['position']['x']-extent, actor['position']['x']+extent
                reserve = max(center-half_width-front, rear-center-half_width)
                require(reserve > 0, 'stationary queued NPC SI footprint overlaps an authored main-route junction')
                npc_minimum_junction_reserve = min(npc_minimum_junction_reserve, reserve)
                stationary_npc_checks += 1
    require(stationary_npc_checks > 0, 'no actual queued NPC stationary footprint was checked')
    hold_start, hold_end = signals['ego_stopped_behind_red_queue_interval_s']
    queue_poses = [s['pose'] for s in samples if hold_start <= s['time'] <= hold_end]
    require(queue_poses, 'missing native body poses during independently verified red queue hold')
    def si_extent(pose):
        return max(run['vehicle']['radius'], 2.14*abs(math.cos(pose['yaw']))+1.1*abs(math.sin(pose['yaw'])))
    queue_rear = min(p['position']['x']-si_extent(p) for p in queue_poses)-12*.005/2
    queue_front = max(p['position']['x']+si_extent(p) for p in queue_poses)+12*.005/2
    require(queue_rear > JUNCTIONS[0][2]+JUNCTIONS[0][3]
            and queue_front < JUNCTIONS[1][2]-JUNCTIONS[1][3],
            'actual red queue occupies an authored cross street instead of the space between streets')
    first_red_front = max(s['pose']['position']['x']+si_extent(s['pose'])
                          for s in samples if s['time'] < 8)+12*.005/2
    require(first_red_front < JUNCTIONS[0][2]-JUNCTIONS[0][3],
            'first red queue intrudes into the first cross street')
    crossings = []
    for junction_index, (identity, route_x, center, half_width) in enumerate(JUNCTIONS):
        matches = [(a, b) for a, b in zip(samples, samples[1:])
                   if a['pose']['position']['x'] < route_x <= b['pose']['position']['x']]
        require(len(matches) == 1, 'ego did not actually traverse both authored intersection coordinates')
        a, b = matches[0]
        elapsed = ((route_x-a['pose']['position']['x'])
                   /(b['pose']['position']['x']-a['pose']['position']['x']))
        ego_crossing_time = a['time']+elapsed*(b['time']-a['time'])
        spec = run['scenario']['objects'][identity]
        traffic_crossing_time = spec['moving_from']-spec['lateral']/spec['lateral_speed']
        require(traffic_crossing_time < ego_crossing_time,
                'cross traffic did not clear before the recorded ego intersection traversal')
        entry, exit = center-half_width, center+half_width
        stopline = run['scenario']['traffic_signals'][junction_index]['stop_line']['route_s_m']
        require(stopline+.06 < entry, 'mapped stopline or its 12 cm display marking enters a cross street')
        cross_x = [next(a['position']['x'] for a in q['objects'] if a['id'] == identity)
                   for q in evidence['acquisitions']]
        require(spec['lateral_speed'] > 0 and all(abs(x-route_x) < 1e-8 for x in cross_x),
                'actual crossing car left its authored +Y street')
        lane_min, lane_max = (entry, center) if junction_index == 0 else (entry, exit)
        # Independent evaluated mesh audit: sedan mirrors reach 1.116736 m;
        # hatchback mirrors reach 1.092191 m. Round both upward, and retain a
        # larger physical capsule if the authored proxy ever requires it.
        crossing_car_half_width = max(1.13 if identity == 19 else 1.1, spec['radius'])
        lane_reserve = min(min(x-crossing_car_half_width-lane_min,
                               lane_max-x-crossing_car_half_width) for x in cross_x)
        require(lane_reserve > 0, 'crossing car conservative SI/capsule width leaves its authored lane')
        occupied = []
        stopped_inside = []
        for previous, sample in zip(samples, samples[1:]):
            pose = sample['pose']
            x, yaw = pose['position']['x'], pose['yaw']
            # Mirror-aware SI envelope and the larger research body circle
            # are both tested. Neither changes the physical clearance guard.
            extent = max(run['vehicle']['radius'], 2.14*abs(math.cos(yaw))+1.1*abs(math.sin(yaw)))
            if x+extent >= entry and x-extent <= exit:
                occupied.append(sample['time'])
                measured_speed = math.hypot(x-previous['pose']['position']['x'],
                                            pose['position']['y']-previous['pose']['position']['y'])/(sample['time']-previous['time'])
                if measured_speed <= .1:
                    stopped_inside.append(sample['time'])
        require(occupied and not stopped_inside, 'ego stopped inside an authored cross-street envelope')
        last = samples[-1]['pose']
        extent = max(run['vehicle']['radius'], 2.14*abs(math.cos(last['yaw']))+1.1*abs(math.sin(last['yaw'])))
        rear = last['position']['x']-extent
        require(rear > exit, 'final full ego body has not exited the authored cross street')
        crossings.append({'cross_traffic_id': identity, 'ego_route_x_m': route_x,
                          'street_center_x_m': center, 'street_half_width_m': half_width,
                          'street_entry_x_m': entry, 'street_exit_x_m': exit,
                          'mapped_stopline_x_m': stopline,
                          'stopline_marking_to_street_reserve_m': entry-stopline-.06,
                          'street_direction': 'two_way_left_traffic' if junction_index == 0 else '+Y_one_way',
                          'crossing_car_lane_x_bounds_m': [lane_min, lane_max],
                          'crossing_car_half_width_m': crossing_car_half_width,
                          'crossing_car_minimum_lane_edge_reserve_m': lane_reserve,
                          'actual_ego_body_occupied_interval_s': [min(occupied), max(occupied)],
                          'stopped_body_samples_inside': len(stopped_inside),
                          'stopped_sample_contract': 'successive actual 200 Hz native body position displacement / elapsed time <= 0.1 m/s',
                          'final_conservative_full_body_rear_x_m': rear,
                          'final_full_body_exit_reserve_m': rear-exit,
                          'ego_center_crossing_time_s': ego_crossing_time,
                          'cross_traffic_center_crossing_time_s': traffic_crossing_time,
                          'ego_crossing_pose_interval_s': [a['time'], b['time']]})
    boxes = evidence['scene']['static_cuboids']
    require(len(boxes) == 1 and boxes[0]['id'] == 'construction-roadblock',
            'actual physical construction cuboid missing')
    minimum = {str(i): math.inf for i in range(ACTOR_COUNT)}
    for first, second in zip(evidence['acquisitions'], evidence['acquisitions'][1:]):
        dt = second['time']-first['time']
        a = {actor['id']: actor for actor in first['objects']}
        b = {actor['id']: actor for actor in second['objects']}
        for identity, spec in enumerate(run['scenario']['objects']):
            speed = math.hypot(spec['speed'], spec['lateral_speed'])
            if (spec.get('following') is None and spec.get('moving_until') is not None
                    and first['time'] >= spec['moving_until']):
                speed = 0.
            for box in boxes:
                d0 = g.scenes.segment_rectangle_distance(g.scenes.xy(a[identity]['position']),
                                                         g.scenes.xy(a[identity]['position']), box)
                d1 = g.scenes.segment_rectangle_distance(g.scenes.xy(b[identity]['position']),
                                                         g.scenes.xy(b[identity]['position']), box)
                t = min(dt, max(0., (d0-d1+speed*dt)/(2*speed))) if speed else 0.
                reserve = max(d0-speed*t, d1-speed*(dt-t))-a[identity]['radius']
                minimum[str(identity)] = min(minimum[str(identity)], reserve)
    require(all(math.isfinite(value) and value >= 1 for value in minimum.values()),
            'one physical actor violates continuous construction footprint clearance>=1m')
    require(physical['min_clearance_m'] >= 1,
            'actual ego body violates unchanged construction clearance>=1m')
    return {'intersections': crossings,
            'traversal_scope': 'straight actual conservative full-body exit from both explicit street envelopes; no stopped body inside either junction; prescribed cross traffic, no general junction right-of-way policy',
            'queued_vehicle_si_footprints': {'ids': [0, 8, 9], 'stationary_speed_threshold_m_s': .1,
                'stationary_frame_junction_checks': stationary_npc_checks,
                'minimum_stationary_npc_junction_reserve_m': npc_minimum_junction_reserve,
                'longitudinal_half_extents_m': {str(i): run['scenario']['objects'][i]['following']['signal_control']['front_extent_m'] for i in [0, 8, 9]},
                'ego_red_queue_native_pose_interval_s': [hold_start, hold_end],
                'ego_red_queue_continuous_rear_bound_x_m': queue_rear,
                'ego_red_queue_continuous_front_bound_x_m': queue_front,
                'first_red_ego_continuous_front_bound_x_m': first_red_front,
                'scope': 'calibrated symmetric SI display envelopes of the straight forward-route queued vehicles; physical proxies and 1 m collision floors unchanged'},
            'construction': {'native_static_cuboids': boxes,
                             'ego_body_continuous_clearance_m': physical['min_clearance_m'],
                             'minimum_continuous_actor_footprint_clearance_bound_m': minimum,
                             'actual_static_returns': physical['actual_static_returns'],
                             'removed_actual_static_returns': physical['removed_actual_static_returns'],
                             'observed_by_lidar': physical['actual_static_returns'] > 0,
                             'height_eligible_static_returns': physical['height_eligible_static_returns'],
                             'construction_avoidance_implemented': False,
                             'scope': 'native physical cuboid and independent clearance; lead-stop ending, no construction detour/contact-force claim'},
            'fixture_changes_from_old_city': {'first19_actor_policy_unchanged_except': {'4': {'old_s_m': 38, 'new_s_m': 40}, '8': {'old_s_m': 60, 'new_s_m': 80}, '9': {'old_s_m': 76, 'new_s_m': 96}},
                                             'reason': 'separate pedestrian4 from the one-way crossing-car stream; place queued cars beyond construction rather than inside its collider'}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/japan-city-demo-v4')
    parser.add_argument('--verify-existing', type=Path, help='root containing seed-1/seed-7/seed-42 captures')
    parser.add_argument('--seeds', type=int, nargs='+', default=[1, 7, 42])
    parser.add_argument('--report', type=Path, default=ROOT/'assets/japan-city-demo-results.json')
    parser.add_argument('--compact', action='store_true', help='archive/hash/CRC verify all; preserve seed-7 raw for rendering')
    parser.add_argument('--failed-trial', type=Path, nargs='+', help='retain separately failed development trials without scoring them as positive cases')
    args = parser.parse_args()
    require, sha = g.require, g.rays.sha
    require(sha(SCENE) == SCENE_SHA and sha(SCENARIO) == SCENARIO_SHA, 'frozen Japanese fixture changed')
    fixture = json.loads(SCENARIO.read_text())
    require(fixture['duration'] == 46 and fixture['expected'] == 'stop'
            and fixture['min_clearance_m'] == 1 and len(fixture['objects']) == ACTOR_COUNT, 'VRU deadline/physical acceptance changed')
    dependencies = [Path(__file__), ROOT/'scripts/check-ground-scenes.py', ROOT/'scripts/check-lidar-3d.py',
                    ROOT/'scripts/check-native-scenes.py', ROOT/'scripts/check_hazards.py']
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in dependencies}
    fingerprint = g.hazards.source_fingerprint()
    pin = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    revision = subprocess.run(['git', '-C', str(ROOT.parent/'RobotNativeEngine'), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    require(revision == pin, 'native checkout differs from the pinned revision')
    report = {'schema_version': 1, 'passed': False, 'complete': False,
              'source_fingerprint_sha256': fingerprint, 'checker_dependency_sha256': hashes,
              'fixture_sha256': {'scenario': sha(SCENARIO), 'scene': sha(SCENE)}, 'rne_revision': pin,
              'scope': 'authored 46-second Japanese city:23 circle/capsule actors, actual crossing traffic, signals and one native construction cuboid; no construction avoidance or general junction rules',
              'semantic_recognition': False, 'human_decision_model': False,
              'cases': []}
    if args.failed_trial:
        report['failed_development_trials'] = [json.loads(path.read_text()) for path in args.failed_trial]
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/'report.json'
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    cli, native = ROOT/'target/release/rustdriving', ROOT/'integrations/rne/target/release/rustdriving-rne'
    for seed in args.seeds:
        raw = (args.verify_existing or args.output)/f'seed-{seed}'
        if args.verify_existing is None:
            code, error = g.hazards.invoke([native, '--plant', 'dynamic', '--scenario', SCENARIO, '--scene', SCENE,
                '--lidar-3d', '--ground-segmentation', '--vehicle-body', '--precise-capsule-rays', '--seed', seed, '--output', raw])
            require(code == 0, f'native VRU episode failed: {error}')
        require(sum(p.stat().st_size for p in raw.rglob('*') if p.is_file()) <= 512 * 1024 * 1024,
                'city raw case exceeds the explicit 512 MiB evidence budget')
        code, error = g.hazards.invoke([cli, 'replay', '--log', raw/'sensors.jsonl', '--output', raw/'replay'])
        require(code == 0, f'whole raw sensor-only replay failed: {error}')
        run = json.load((raw/'run.json').open())
        evidence = json.load((raw/'scene.json').open())
        replay = json.load((raw/'replay/replay.json').open())
        summary = run['summary']
        require(supplied_fields(run['scenario'], fixture), 'serialized actual scenario changed')
        require(summary['passed'] and summary['seed'] == seed and summary['simulated_seconds'] == 46
                and summary['steps'] == 921 and summary['collisions'] == summary['road_violations'] == 0
                and summary['min_clearance'] >= 1 and summary['final_speed'] <= .2
                and summary['progress'] >= 40, 'unchanged VRU physical acceptance failed')
        require(replay['verified'] and replay['ticks'] == 921, 'sensor-only replay omitted a control tick')
        print(f'seed {seed}: native acceptance and all 921 sensor-only replay ticks verified', flush=True)
        physical = g.check_ground(run, json.loads(SCENE.read_text()), evidence, raw/'sensors.jsonl')
        print(f'seed {seed}: unchanged independent full-grid native ray/body/ground oracle passed', flush=True)
        actors = verify_actors(run, evidence, raw/'sensors.jsonl')
        signals = verify_signals(run, evidence, raw/'sensors.jsonl')
        lane = verify_painted_lane(evidence)
        junctions = verify_junctions_and_construction(run, evidence, physical, signals)
        row = {'seed': seed, 'summary': summary, 'replay': replay, 'physical_xyz_ground_body': physical,
               'actors': actors, 'signals': signals, 'painted_lane': lane,
               'junctions_and_construction': junctions,
               'raw_sha256': {name: sha(raw/name) for name in ['run.json', 'scene.json', 'sensors.jsonl']}}
        if args.compact:
            row['archive'] = g.rays.compact_case(raw, row, seed == 7)
        report['cases'].append(row)
        report_path.write_text(json.dumps(report, indent=2)+'\n')
        print(f'seed {seed}: all 253 continuous actor pairs, ego clearance, signals, lane and actor evidence verified', flush=True)
    require(fingerprint == g.hazards.source_fingerprint() and hashes == {str(p.relative_to(ROOT)): sha(p) for p in dependencies}
            and sha(SCENE) == SCENE_SHA and sha(SCENARIO) == SCENARIO_SHA,
            'source/checker changed during the independent proof')
    report.update(passed=True, complete=True, cases_verified=len(report['cases']),
                  replay_ticks_verified=sum(row['replay']['ticks'] for row in report['cases']))
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{args.report}: {len(report["cases"])} actual native Japanese city episodes; passed=True')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'Japanese city check: {error}', file=sys.stderr)
        sys.exit(2)
