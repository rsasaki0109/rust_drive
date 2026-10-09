#!/usr/bin/env python3
"""Run real hazard scenarios, verify sensor recomputation, and retain acceptance evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
CASES = {
    'reference': ['occluded-crossing', 'cut-in', 'multiple-blocked', 'opposing-crossings', 'route-direct', 'route-detour', 'route-south', 'route-handover', 'route-handover-fast', 'route-no-path', 'route-reopen', 'gnss-spike', 'gnss-burst', 'gnss-persistent-bias', 'gnss-burst-traffic', 'gnss-burst-traffic-hold', 'traffic-lead-stop', 'traffic-follower-brake', 'traffic-queue', 'traffic-follower-deadline', 'traffic-fleet-queue', 'signal-red-green', 'signal-red-stop', 'signal-stale-stop', 'signal-stale-recovery', 'signal-two-stops', 'signal-approach-change'],
    'rne-dynamic': ['occluded-crossing', 'cut-in', 'low-friction', 'low-friction-stop', 'multiple-blocked', 'opposing-crossings', 'route-direct', 'route-detour', 'route-south', 'route-handover', 'route-handover-fast', 'route-no-path', 'route-reopen', 'gnss-spike', 'gnss-burst', 'gnss-persistent-bias', 'gnss-burst-traffic', 'gnss-burst-traffic-hold', 'traffic-lead-stop', 'traffic-follower-brake', 'traffic-queue', 'traffic-follower-deadline', 'traffic-fleet-queue', 'signal-red-green', 'signal-red-stop', 'signal-stale-stop', 'signal-stale-recovery', 'signal-two-stops', 'signal-approach-change'],
}
# Fixed regression floors, chosen against the preceding measured fixture results.
# They are simulation test constraints, not a universal safe-distance specification.
CLEARANCE_FLOORS_M = {
    **dict.fromkeys(['signal-red-green','signal-red-stop','signal-stale-stop','signal-stale-recovery','signal-two-stops','signal-approach-change'],1.0),
    'traffic-lead-stop': 1.0, 'traffic-follower-brake': 1.0, 'traffic-queue': 1.0, 'traffic-follower-deadline': 1.0, 'traffic-fleet-queue': 1.0,
    'gnss-burst-traffic': 0.5, 'gnss-burst-traffic-hold': 0.5, 'gnss-spike': 0.5, 'gnss-burst': 0.5, 'gnss-persistent-bias': 4.0,
    'occluded-crossing': 1.0, 'cut-in': 0.7, 'multiple-blocked': 3.0,
    'opposing-crossings': 0.7, 'low-friction': 0.4, 'low-friction-stop': 4.0,
    'route-direct': 0.5, 'route-detour': 0.5, 'route-south': 0.5,
    'route-handover': 0.5, 'route-handover-fast': 0.5, 'route-no-path': 4.0, 'route-reopen': 0.5,
}
EXPECTED_EDGES = {
    'route-direct': ['approach', 'main', 'east-exit'],
    'route-detour': ['approach', 'detour', 'east-exit'],
    'route-south': ['approach', 'south-branch'],
    'route-handover': ['approach', 'detour', 'east-exit'],
    'route-handover-fast': ['approach', 'detour', 'east-exit'],
    'route-reopen': ['approach', 'detour', 'east-exit'],
}


def clearance_regression(summary, case):
    floor = CLEARANCE_FLOORS_M[case]
    value = summary['min_clearance']
    return {'floor_m': floor, 'measured_m': value,
            'passed': math.isfinite(value) and value >= floor}


def check_signals(run, log):
    """Reconstruct observations, true front crossings and continuous close-line holds."""
    with log.open() as stream:
        header = json.loads(next(stream))['header']
        ticks = [r['tick'] for line in stream if (r := json.loads(line))['kind'] == 'tick']
    specs = run['scenario']['traffic_signals']
    config = header['config']
    if config['stop_lines'] != [s['stop_line'] for s in specs] or any(k in config for k in ['traffic_signals','phases','signal_dropout_windows']):
        raise ValueError('signal schedule/fault truth entered the operational configuration')
    if any(abs(p['y']) > 1e-9 for p in run['route']['points']):
        raise ValueError('these independent signal fixture checks require a straight route')
    frames = run['frames']
    if len(frames) != len(ticks) or len(ticks) != run['summary']['steps']:
        raise ValueError('signal evidence lacks a physical/control tick')
    latest = None
    accepted = 0
    crossed = set()
    crossings = []
    previous_front = None
    truth_margin = unknown_margin = math.inf
    hold_start = {s['stop_line']['id']: None for s in specs}
    hold_longest = dict.fromkeys(hold_start, 0.0)
    radius = run['vehicle']['radius']
    def phase(spec, now):
        return next(p['color'] for p in reversed(spec['phases']) if p['from'] <= now+1e-9)
    for index, (frame, tick) in enumerate(zip(frames, ticks)):
        inp, out = tick['input'], tick['expected']
        now = frame['time']
        if abs(now-index*.05) > 1e-8 or abs(inp['time']-now) > 1e-8:
            raise ValueError('signal evidence changed the control/acquisition clock')
        dropout = any(w['from']-1e-9 <= now < w['until']-1e-9 for w in run['scenario'].get('signal_dropout_windows', []))
        sample = inp.get('traffic_signal')
        if bool(sample) != (index%4 == 0 and not dropout):
            raise ValueError('signal acquisition does not match the declared sampling/dropout')
        if sample:
            expected = [{'id':s['stop_line']['id'], 'color':phase(s,now)} for s in specs]
            if sample['stamp'] != now or sample['states'] != expected:
                raise ValueError('signal observation differs from acquired infrastructure state')
            latest = sample
            accepted += 1
        fresh = latest is not None and now-latest['stamp'] <= .5+1e-9
        status = out['traffic_controls']
        if status['fault'] or status['last_accepted_stamp'] != (latest['stamp'] if latest else None):
            raise ValueError('valid signal evidence changed the accepted-age diagnostics')
        if frame['traffic_controls'] != status:
            raise ValueError('physical telemetry has inconsistent signal diagnostics')
        front = frame['truth']['pose']['position']['x']+radius
        for spec in specs:
            id, line = spec['stop_line']['id'], spec['stop_line']['route_s_m']
            real = phase(spec,now)
            color = next(s['color'] for s in latest['states'] if s['id']==id) if fresh else 'Unknown'
            actual = next(s for s in status['signals'] if s['id']==id)
            if actual['color'] != color:
                raise ValueError('green was released without a fresh accepted observation')
            if real != 'Green' and (previous_front is None or previous_front < line):
                truth_margin = min(truth_margin,line-front)
            if id not in crossed and color != 'Green':
                unknown_margin = min(unknown_margin,line-front)
            crossing = (previous_front is None or previous_front < line) and front >= line
            if crossing:
                if id in crossed or real != 'Green' or color != 'Green':
                    raise ValueError('physical front crossed without true AND freshly observed green')
                crossed.add(id)
                crossings.append({'id':id,'time':now,'observed_color':color})
            holding = id not in crossed and color != 'Green' and 0 <= line-front <= 3.5 and frame['truth']['speed'] < .1
            if holding:
                if hold_start[id] is None: hold_start[id] = now
                hold_longest[id] = max(hold_longest[id],now-hold_start[id])
            else: hold_start[id] = None
        previous_front = front
    if (not accepted or any(d < 2.0-1e-8 for d in hold_longest.values())
            or unknown_margin < 1.0 or truth_margin < 1.0
            or run['summary'].get('signal_violations',0) != 0):
        raise ValueError('signal stopping violated hold, clearance or crossing acceptance')
    moving_change = False
    for spec in specs:
        yellow = next((p['from'] for p in spec['phases'] if p['color']=='Yellow' and p['from']>0),None)
        if spec['phases'][0]['color']=='Green' and yellow is not None:
            if not any(f['time']<yellow and f['truth']['speed']>2.0 for f in frames):
                raise ValueError('approach-change fixture never drove before the signal changed')
            moving_change = True
    reported = run['summary'].get('signal_min_stopline_margin_m')
    if (reported is None) != (not math.isfinite(truth_margin)) or (reported is not None and abs(reported-truth_margin)>1e-8):
        raise ValueError('independent physical stop-line margin differs from summary')
    if run['scenario']['expected']=='goal' and len(crossed) != len(specs):
        raise ValueError('goal scenario did not cross every signal after its physical hold')
    if run['scenario']['expected']=='stop' and crossed:
        raise ValueError('permanent red/stale fixture crossed its stop line')
    return {'signals':len(specs),'accepted_snapshots':accepted,'control_ticks':len(ticks),
            'stopline_crossings':crossings,'continuous_close_line_holds_s':hold_longest,
            'min_true_nonpermissive_margin_m':truth_margin if math.isfinite(truth_margin) else None,
            'min_observed_nonpermissive_margin_m':unknown_margin,
            'snapshot_reconstruction_verified':True,'moving_phase_change_exercised':moving_change,
            'schedule_labels_absent_from_pipeline':True,'passed':True}


def check_navigation(run, case):
    """Check selected topology/geometry against fixture expectations independently."""
    nav = dict(run['scenario']['navigation'])
    if run['scenario'].get('navigation_updates'):
        nav['closed_edges'] = run['scenario']['navigation_updates'][-1]['closed_edges']
    plan = run['navigation']
    edges = {edge['id']: edge for edge in nav['network']['edges']}
    nodes = {node['id']: node['position'] for node in nav['network']['nodes']}
    at = nav['start']
    points = []
    distance = 0.0
    half_width = math.inf
    node_ids = [at]
    for id in plan['edge_ids']:
        edge = edges[id]
        if id in nav['closed_edges'] or edge['from'] != at:
            raise ValueError('selected path violates closure or connectivity')
        geometry = edge['points']
        if (math.dist(xy(geometry[0]), xy(nodes[edge['from']])) > 1e-6
                or math.dist(xy(geometry[-1]), xy(nodes[edge['to']])) > 1e-6):
            raise ValueError('edge geometry disagrees with its map endpoints')
        half_width = min(half_width, edge['half_width'])
        distance += sum(math.dist(xy(a), xy(b)) for a, b in zip(geometry, geometry[1:]))
        points.extend(geometry[1:] if points else geometry)
        at = edge['to']
        node_ids.append(at)
    if (at != nav['goal'] or plan['edge_ids'] != EXPECTED_EDGES[case]
            or plan['node_ids'] != node_ids or points != run['route']['points']
            or points != plan['route']['points'] or abs(distance - plan['distance_m']) > 1e-7
            or abs(distance - run['route']['lengths'][-1]) > 1e-7
            or half_width != run['route']['half_width'] or half_width != plan['route']['half_width']):
        raise ValueError('selected route differs from the expected mapped destination')
    end_distance = math.dist(xy(run['frames'][-1]['truth']['pose']['position']), xy(nodes[nav['goal']]))
    if end_distance > 2.0:
        raise ValueError('vehicle did not stop within 2 m of the mapped destination')
    return {'edge_ids': plan['edge_ids'], 'distance_m': distance,
            'goal_distance_m': end_distance, 'passed': True}


def xy(point):
    return point['x'], point['y']


def check_gnss_fault(run, log, case):
    """Verify actual biased observations, accepted-age braking and physical recovery."""
    with log.open() as stream:
        header = json.loads(next(stream))['header']
        ticks = [r['tick'] for line in stream if (r := json.loads(line))['kind'] == 'tick']
    if 'gnss_bias_windows' in header['config']:
        raise ValueError('simulator fault labels entered pipeline configuration')
    windows = run['scenario']['gnss_bias_windows']
    frames = {round(f['time']*20): f for f in run['frames']}
    accepted_stamp = observed_stamp = None
    accepted = rejected = biased = 0
    stale_times = []
    recovery = None
    previous = None
    for tick in ticks:
        inp, out = tick['input'], tick['expected']
        diagnostic = out['localization']
        fix = inp.get('gnss')
        if fix:
            observed_stamp = fix['stamp']
            offset = next((w['offset'] for w in windows if w['from']-1e-9 <= fix['stamp'] < w['until']-1e-9), {'x': 0.0, 'y': 0.0})
            truth = frames[round(fix['stamp']*20)]['truth']['pose']['position']
            if any(abs(fix['position'][axis]-truth[axis]-offset[axis]) > 0.14+1e-8 for axis in ['x', 'y']):
                raise ValueError('recorded GNSS does not contain the scheduled bias and bounded sensor noise')
            is_biased = math.hypot(*xy(offset)) > 0
            if is_biased:
                biased += 1
                if diagnostic['last_decision'] != 'RejectedInnovation' or diagnostic['last_nis'] <= 36:
                    raise ValueError('biased fix was not rejected by the innovation gate')
                # Rejected GNSS cannot shift the state beyond wheel/gyro prediction.
                pose = previous['estimate']['pose']
                dt = out['time']-previous['time']
                speed = max(0.0, inp['odometry']['speed'])
                predicted = (pose['position']['x']+math.cos(pose['yaw'])*speed*dt,
                             pose['position']['y']+math.sin(pose['yaw'])*speed*dt)
                if math.dist(xy(out['estimate']['pose']['position']), predicted) > 1e-8:
                    raise ValueError('rejected GNSS mutated the predicted position')
            if diagnostic['last_decision'] == 'Accepted':
                accepted += 1
                accepted_stamp = fix['stamp']
                if case.startswith('gnss-burst') and recovery is None and fix['stamp'] >= windows[-1]['until']:
                    recovery = out['time']
            elif diagnostic['last_decision'] == 'RejectedInnovation':
                rejected += 1
            else:
                raise ValueError('new finite GNSS has an unexpected correction decision')
        if (diagnostic['last_observed_stamp'] != observed_stamp
                or diagnostic['last_accepted_stamp'] != accepted_stamp
                or diagnostic['accepted_fixes'] != accepted or diagnostic['rejected_fixes'] != rejected):
            raise ValueError('GNSS receipt/acceptance diagnostics disagree with observations')
        stale = accepted_stamp is None or out['time']-accepted_stamp > 0.75+1e-9
        if ('StaleGnss' in out['health']) != stale:
            raise ValueError('GNSS health refreshed from an unaccepted fix')
        if stale:
            stale_times.append(out['time'])
            if not out['emergency'] or out['command'] != {'acceleration': -6.0, 'steering': 0.0}:
                raise ValueError('stale GNSS did not emit the defined emergency command')
        if previous and previous['emergency'] and not out['emergency']:
            if abs(out['command']['steering']) > 0.7*(out['time']-previous['time'])+1e-8:
                raise ValueError('recovery steering did not start from the emitted zero command')
        previous = out
    if biased != rejected or not biased or run['summary']['localization_max_error'] > 0.5:
        raise ValueError('GNSS rejection or bounded localization regression failed')
    if case == 'gnss-spike' and (biased != 1 or stale_times):
        raise ValueError('single rejected fix caused unintended GNSS-stale braking')
    if case.startswith('gnss-burst'):
        if not stale_times or recovery is None or recovery > windows[-1]['until']+0.4:
            raise ValueError('burst did not brake and accept a good fix after the window')
        held = [f for f in run['frames'] if 7.0 <= f['time'] < 8.0 and f['truth']['speed'] < 0.1]
        if not held or not run['summary']['reached_goal']:
            raise ValueError('burst did not physically stop and resume to the goal')
    if case == 'gnss-persistent-bias' and (not stale_times or 'StaleGnss' not in ticks[-1]['expected']['health']
            or run['summary']['final_speed'] > 0.1 or run['summary']['reached_goal']):
        raise ValueError('persistent fault did not remain stopped with stale accepted GNSS')
    return {'biased_fixes': biased, 'rejected_fixes': rejected, 'accepted_fixes': accepted,
            'first_stale_s': stale_times[0] if stale_times else None,
            'first_recovered_fix_s': recovery, 'max_localization_error_m': run['summary']['localization_max_error'],
            'max_error_limit_m': 0.5, 'fault_labels_absent_from_pipeline': True, 'passed': True}


def check_goal_hold(run, log):
    with log.open() as stream:
        header = json.loads(next(stream))['header']
    if 'goal_hold_seconds' in header['config']:
        raise ValueError('physical goal residence acceptance entered pipeline configuration')
    hold = run['scenario']['goal_hold_seconds']
    frames = run['frames']
    end = frames[-1]['time']
    held = [f for f in frames if f['time'] >= end-hold-1e-9]
    if not held or held[0]['time'] > end-hold+1e-9:
        raise ValueError('recording does not cover the required terminal residence')
    # Recompute polyline projection from truth, independently of reported progress.
    route = run['route']['points']
    segments = list(zip(route, route[1:]))
    def progress(position):
        best = (math.inf, 0.0)
        accumulated = 0.0
        for a, b in segments:
            dx, dy = b['x']-a['x'], b['y']-a['y']
            length = math.hypot(dx, dy)
            u = max(0.0, min(1.0, ((position['x']-a['x'])*dx+(position['y']-a['y'])*dy)/(length*length)))
            d = math.dist(xy(position), (a['x']+u*dx, a['y']+u*dy))
            if d < best[0]:
                best = d, accumulated+u*length
            accumulated += length
        return best[1], accumulated
    positions = [f['truth']['pose']['position'] for f in held]
    excursion = max(math.dist(xy(p), xy(positions[0])) for p in positions)
    clearance = min(math.dist(xy(f['truth']['pose']['position']), xy(o['position']))
                    -run['vehicle']['radius']-o['radius'] for f in held for o in f['objects'])
    if (any(f['truth']['speed'] >= 0.2 or progress(p)[0] < progress(p)[1]-2.0-1e-8
            for f, p in zip(held, positions)) or excursion > 0.5 or clearance < 0.5):
        raise ValueError('truth did not remain stopped and clear at the goal for the complete hold')
    lead = next(o for o in frames[-1]['objects'] if o['id'] == 1)
    if math.dist(xy(lead['position']), xy(route[-1])) > 1e-8:
        raise ValueError('evaluation ended before the scripted lead reached the endpoint')
    return {'required_seconds': hold, 'observed_seconds': end-held[0]['time'],
            'maximum_excursion_m': excursion, 'minimum_sampled_clearance_m': clearance,
            'lead_reached_endpoint': True, 'acceptance_absent_from_pipeline': True, 'passed': True}


def check_traffic(run, log, case):
    """Independent fixture truth, measured ranges, actor kinematics and stop/resume checks."""
    with log.open() as stream:
        header = json.loads(next(stream))['header']
        ticks = [record['tick'] for line in stream if (record := json.loads(line))['kind'] == 'tick']
    if any(key in header['config'] for key in ['objects', 'following', 'traffic', 'stop_windows']):
        raise ValueError('traffic truth/configuration entered the ego pipeline')
    frames = run['frames']
    specs = run['scenario']['objects']
    length = sum(math.dist(xy(a), xy(b)) for a, b in zip(run['route']['points'], run['route']['points'][1:]))
    if any(abs(p['y']) > 1e-9 for p in run['route']['points']):
        raise ValueError('these independent traffic fixture checks require their straight route')
    reactive = {i: s for i, s in enumerate(specs) if s.get('following')}
    previous = None
    minimum_pair = math.inf
    max_acceleration = 0.0
    sensor_samples = 0
    for frame in frames:
        objects = {o['id']: o for o in frame['objects']}
        states = {a['id']: a for a in frame.get('traffic', [])}
        for id, actor in states.items():
            if id not in reactive or id not in objects:
                raise ValueError('actor state lacks its physical body')
            p, config = objects[id]['position'], reactive[id]['following']
            if (abs(p['x']-actor['route_s_m']) > 1e-8 or abs(p['y']-reactive[id]['lateral']) > 1e-8
                    or not math.isfinite(actor['speed_m_s']) or actor['speed_m_s'] < 0
                    or abs(p['y'])+objects[id]['radius'] > run['route']['half_width']
                    or p['x']+objects[id]['radius'] > length+1e-8):
                raise ValueError('actor truth violates its route/finite-state bounds')
            a = actor['acceleration_m_s2']
            if not -config['max_deceleration_m_s2']-1e-8 <= a <= config['max_acceleration_m_s2']+1e-8:
                raise ValueError('actor command exceeds calibrated acceleration bounds')
            if previous:
                old = next((s for s in previous.get('traffic', []) if s['id'] == id), None)
                if old:
                    dt = frame['time']-previous['time']
                    if abs(dt-0.05) > 1e-8:
                        raise ValueError('reactive actor evidence is missing a control tick')
                    expected_speed = max(0.0, old['speed_m_s']+a*dt)
                    if (abs(actor['speed_m_s']-expected_speed) > 1e-8
                            or abs(actor['route_s_m']-old['route_s_m']-(old['speed_m_s']+expected_speed)*0.5*dt) > 1e-8):
                        raise ValueError('actor was clamped/teleported instead of physically integrated')
                    max_acceleration = max(max_acceleration, abs((actor['speed_m_s']-old['speed_m_s'])/dt))
                    # Reconstruct the pre-step ideal route-aligned proximity sensor.
                    candidates = [max(0.0, length-reactive[id]['radius']-old['route_s_m'])]
                    scene = [o for o in previous['objects'] if o['id'] != id]
                    scene += [{'position': previous['truth']['pose']['position'], 'radius': run['vehicle']['radius']}]
                    for o in scene:
                        dx = o['position']['x']-old['route_s_m']
                        side = o['position']['y']-reactive[id]['lateral']
                        radius = reactive[id]['radius']+o['radius']
                        if dx >= 0 and abs(side) < radius:
                            candidates.append(dx-math.sqrt(radius*radius-side*side))
                    visible = [g for g in candidates if g <= config['sensor_range_m']]
                    measured = min(visible) if visible else None
                    observation = actor['observation']
                    if abs(observation['stamp']-previous['time']) > 1e-8:
                        raise ValueError('actor proximity observation is not from the pre-step scene')
                    actual = observation['gap_m']
                    if (actual is None) != (measured is None) or actual is not None and abs(actual-measured) > 1e-8:
                        raise ValueError('actor gap differs from independently reconstructed sensing')
                    old_gap = old['observation']['gap_m']
                    closing = (max(-12.0, min(12.0, (old_gap-measured)/dt)) if old_gap is not None
                               else old['speed_m_s']) if measured is not None else None
                    if (closing is None) != (observation['closing_speed_m_s'] is None) or closing is not None and abs(closing-observation['closing_speed_m_s']) > 1e-8:
                        raise ValueError('actor closing speed is not a measured range difference')
                    sensor_samples += 1
        for i, a in enumerate(frame['objects']):
            for b in frame['objects'][i+1:]:
                if a['id'] not in reactive and b['id'] not in reactive:
                    continue
                separation = math.dist(xy(a['position']), xy(b['position']))-a['radius']-b['radius']
                if previous:
                    old_objects = {o['id']: o for o in previous['objects']}
                    if a['id'] in old_objects and b['id'] in old_objects:
                        pa, pb = old_objects[a['id']]['position'], old_objects[b['id']]['position']
                        x, y = pa['x']-pb['x'], pa['y']-pb['y']
                        dx, dy = a['position']['x']-b['position']['x']-x, a['position']['y']-b['position']['y']-y
                        u = max(0.0, min(1.0, -(x*dx+y*dy)/(dx*dx+dy*dy))) if dx*dx+dy*dy else 0.0
                        separation = min(separation, math.hypot(x+u*dx, y+u*dy)-a['radius']-b['radius'])
                minimum_pair = min(minimum_pair, separation)
        previous = frame
    if sensor_samples == 0 or run['summary'].get('traffic_collisions', 0) or run['summary'].get('traffic_road_violations', 0):
        raise ValueError('reactive traffic was untested or violated physical acceptance')
    if math.isfinite(minimum_pair):
        if minimum_pair < 1.0 or abs(minimum_pair-run['summary']['traffic_min_clearance']) > 1e-8:
            raise ValueError('independent actor-pair sweep differs from physical acceptance')
    def hold_duration(predicate):
        start = None
        longest = 0.0
        for f in frames:
            if predicate(f):
                if start is None: start = f['time']
                longest = max(longest, f['time']-start)
            else: start = None
        return longest
    if case == 'traffic-lead-stop':
        ego_hold = hold_duration(lambda f: 17 <= f['time'] < 24 and f['truth']['speed'] < 0.1)
        if ego_hold < 2.0 or not any(f['time'] > 26 and f['time'] < 42 and f['truth']['speed'] > 2 and f['truth']['pose']['position']['x'] > 100 for f in frames):
            raise ValueError('ego did not wait for and resume behind the stopped lead')
        if not all(f['traffic'][0]['speed_m_s'] < 0.1 for f in frames if 11 <= f['time'] < 23.9):
            raise ValueError('lead did not physically stop in its requested window')
    elif case in ['traffic-queue', 'traffic-fleet-queue']:
        if any(a['speed_m_s'] > 0.05 for a in frames[-1]['traffic']) or not math.isfinite(minimum_pair):
            raise ValueError('queue did not stop with measured actor-pair separation')
        if any(a['route_s_m'] < reactive[a['id']]['s']+15 for a in frames[-1]['traffic']):
            raise ValueError('queue fixture never exercised traffic motion')
        if case == 'traffic-fleet-queue':
            if len(reactive) != 3 or any(len(f.get('traffic', [])) != 3
                    or {a['id'] for a in f['traffic']} != set(reactive) for f in frames):
                raise ValueError('fleet evidence must retain all three reactive vehicles at every tick')
            order = sorted(reactive, key=lambda id: reactive[id]['s'])
            for f in frames:
                positions = {a['id']: a['route_s_m'] for a in f['traffic']}
                if any(positions[a] >= positions[b] for a, b in zip(order, order[1:])):
                    raise ValueError('fleet vehicle order changed on a single-lane queue')
            # Require a continuous terminal hold, rather than a lucky final frame.
            fleet_hold = 0.0
            for f in reversed(frames):
                if f['truth']['speed'] >= 0.1 or any(a['speed_m_s'] >= 0.05 for a in f['traffic']):
                    break
                fleet_hold = frames[-1]['time']-f['time']
            if fleet_hold < 5.0-1e-8:
                raise ValueError('ego and fleet did not remain stopped together for five seconds')
    elif case.startswith('traffic-follower'):
        fault_end = run['scenario']['gnss_bias_windows'][0]['until']
        ego_hold = hold_duration(lambda f: 11 <= f['time'] < fault_end and f['truth']['speed'] < 0.1)
        actor_hold = hold_duration(lambda f: 12 <= f['time'] < fault_end and f.get('traffic') and f['traffic'][0]['speed_m_s'] < 0.1)
        minimum_fault_speed = min(f['traffic'][0]['speed_m_s'] for f in frames if 12 <= f['time'] < fault_end)
        if ego_hold < 1.0 or (case == 'traffic-follower-brake' and actor_hold < 0.25) or minimum_fault_speed >= 2.0:
            raise ValueError('ego/follower did not physically stop during the GNSS fault')
        if case != 'traffic-follower-short-range' and not any(16 < f['time'] < 30 and f['traffic'][0]['speed_m_s'] > 2.0 and f['truth']['speed'] > 2.0 for f in frames):
            raise ValueError('reactive follower and ego never resumed')
        if not any('StaleGnss' in t['expected']['health'] for t in ticks):
            raise ValueError('ego braking fixture did not exercise accepted-GNSS expiry')
        for t in ticks:
            fix = t['input'].get('gnss')
            if fix and 10 <= fix['stamp'] < fault_end and t['expected']['localization']['last_decision'] != 'RejectedInnovation':
                raise ValueError('follower fixture did not reject the actual GNSS burst')
        residence = 0.0
        for f in reversed(frames):
            if f['truth']['speed'] >= 0.2 or f['truth']['pose']['position']['x'] < length-2.0:
                break
            residence = frames[-1]['time']-f['time']
        if case in ['traffic-follower-brake', 'traffic-follower-deadline'] and (residence < 8.0-1e-8 or not run['summary']['reached_goal']):
            raise ValueError('ego did not remain at the goal for the complete physical hold')
    return {**({'reactive_vehicle_count': 3, 'terminal_queue_hold_seconds': fleet_hold}
               if case == 'traffic-fleet-queue' else {}),
            'sensor_samples': sensor_samples, 'max_measured_actor_acceleration_m_s2': max_acceleration,
            'minimum_actor_pair_clearance_m': minimum_pair if math.isfinite(minimum_pair) else None,
            'actor_kinematics_verified': True, 'sensor_reconstruction_verified': True,
            'traffic_labels_absent_from_pipeline': True, 'passed': True}


def check_live_navigation(run, log, case):
    """Check stop-before-switch, update replay state and stationary world objects."""
    ticks = []
    with log.open() as stream:
        for line in stream:
            record = json.loads(line)
            if record['kind'] == 'tick':
                ticks.append(record['tick'])
    updates = [t['input']['navigation_update'] for t in ticks if t['input'].get('navigation_update')]
    if updates != run['scenario']['navigation_updates']:
        raise ValueError('recorded map snapshots differ from the scheduled inputs')
    observed_switches = 0
    revision = 0
    closures = run['scenario']['navigation']['closed_edges']
    for i, tick in enumerate(ticks):
        out = tick['expected']
        nav = out['navigation']
        snapshot = tick['input'].get('navigation_update')
        if snapshot:
            revision, closures = snapshot['revision'], snapshot['closed_edges']
        if nav['revision'] != revision or nav['closed_edges'] != closures:
            raise ValueError('navigation state did not apply the delivered closure snapshot')
        if ((nav['phase'] == 'Following' and set(nav['active_edges']) & set(closures))
                or set(nav['pending_edges']) & set(closures)):
            raise ValueError('following or pending path includes a closed edge')
        if nav['switches'] > observed_switches:
            if nav['switches'] != observed_switches + 1 or i < 2:
                raise ValueError('invalid handover count')
            for prior in ticks[i-2:i+1]:
                state = prior['expected']
                if state['health'] or abs(state['estimate']['speed']) > 0.05:
                    raise ValueError('handover lacks three healthy stopped estimates')
            observed_switches += 1
    history = run['route_history']
    if len(history) != observed_switches + 1:
        raise ValueError('route history differs from the replayed switch count')
    for change in history[1:]:
        if change['true_speed'] > 0.1 or abs(change['estimated_speed']) > 0.05:
            raise ValueError('moving vehicle changed routes')
        if change['time'] <= updates[0]['stamp']:
            raise ValueError('route changed before the closure was delivered')
    # Both fixtures contain a stationary object; route changes must not move it.
    positions = [f['objects'][0]['position'] for f in run['frames']]
    if any(p != positions[0] for p in positions):
        raise ValueError('world object teleported during navigation')
    if run['summary'].get('closure_violations', 0):
        raise ValueError('truth entered a closed edge')
    if case in ['route-handover', 'route-handover-fast', 'route-reopen']:
        if observed_switches != 1 or ticks[-1]['expected']['navigation']['phase'] != 'Following':
            raise ValueError('detour handover did not complete')
    else:
        if (observed_switches or ticks[-1]['expected']['navigation']['phase'] != 'Blocked'
                or run['summary']['progress'] + run['vehicle']['radius'] >= 40.0
                or run['summary']['final_speed'] >= 0.2):
            raise ValueError('unreachable route did not hold before the closed fork')
    return {'switches': observed_switches, 'history': [{k: h[k] for k in ['time', 'estimated_speed', 'true_speed']} | {'edge_ids': h['plan']['edge_ids']} for h in history],
            'stationary_world_verified': True, 'passed': True}


def invoke(args):
    result = subprocess.run([str(a) for a in args], cwd=ROOT, capture_output=True, text=True)
    return result.returncode, result.stderr[-2000:]


def source_fingerprint():
    files = [ROOT/'Cargo.toml', ROOT/'Cargo.lock', ROOT/'rust-toolchain.toml']
    files += list((ROOT/'crates').glob('*/Cargo.toml')) + list((ROOT/'crates').glob('**/*.rs'))
    files += list((ROOT/'scenarios').glob('*.json'))
    files += [ROOT/'integrations/rne'/p for p in ['Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml', 'rne-revision.txt']]
    files += list((ROOT/'integrations/rne/src').glob('*.rs'))
    digest = hashlib.sha256()
    for file in sorted(set(files)):
        digest.update(str(file.relative_to(ROOT)).encode()+b'\0'+file.read_bytes()+b'\0')
    return digest.hexdigest()


def check_speed_profiles(log):
    """Check distance, speed and time kinematics independently of planner code."""
    active = holds = 0
    max_acceleration = max_deceleration = 0.0
    with log.open() as stream:
        header = json.loads(next(stream))
        limits = header['header']['config'].get('motion_limits') or {}
        forward = limits.get('max_acceleration_m_s2', 2.0)
        braking = limits.get('max_deceleration_m_s2', 2.5)
        for line in stream:
            record = json.loads(line)
            if record.get('kind') != 'tick':
                continue
            expected = record['tick']['expected']
            path = expected['trajectory']['points']
            if expected['trajectory']['mode'] == 'Emergency':
                continue
            if not path or path[0]['time'] != 0.0:
                raise ValueError('trajectory must start at relative time zero')
            if abs(path[0]['speed'] - max(0.0, expected['estimate']['speed'])) > 1e-8:
                raise ValueError('profile initial speed differs from the estimated state')
            active += 1
            for point in path:
                values = [point['time'], point['speed'], *point['position'].values()]
                if not all(math.isfinite(value) for value in values) or point['speed'] < 0:
                    raise ValueError('non-finite or negative profile state')
            for a, b in zip(path, path[1:]):
                duration = b['time'] - a['time']
                if duration <= 0:
                    raise ValueError('profile time must increase')
                distance = math.hypot(b['position']['x'] - a['position']['x'], b['position']['y'] - a['position']['y'])
                integrated_distance = 0.5 * (a['speed'] + b['speed']) * duration
                if abs(distance - integrated_distance) > 1e-7 * max(1.0, distance):
                    raise ValueError('speed integral disagrees with segment distance')
                acceleration = (b['speed'] - a['speed']) / duration
                if acceleration > forward + 1e-7 or acceleration < -braking - 1e-7:
                    raise ValueError('profile exceeds calibrated longitudinal authority')
                max_acceleration = max(max_acceleration, acceleration)
                max_deceleration = max(max_deceleration, -acceleration)
                if distance < 1e-10:
                    if a['speed'] != 0 or b['speed'] != 0:
                        raise ValueError('stationary segment has nonzero speed')
                    holds += 1
    if not active:
        raise ValueError('no active speed profiles checked')
    return {'verified': True, 'active_trajectories': active, 'stationary_holds': holds,
            'max_acceleration_m_s2': max_acceleration,
            'max_deceleration_m_s2': max_deceleration}


def check_motion_predictions(log):
    """Check forecast physics and observation support from the sensor log alone."""
    history = {}
    forecasts = braking_forecasts = stale_fallbacks = reacceleration_fallbacks = 0
    max_deceleration = 0.0
    with log.open() as stream:
        next(stream)
        for line in stream:
            record = json.loads(line)
            if record.get('kind') != 'tick':
                continue
            tick = record['tick']
            now = tick['input']['time']
            tracks = {t['id']: t for t in tick['expected']['tracks']}
            history = {id: samples for id, samples in history.items() if id in tracks}
            for forecast in tick['expected']['predictions']:
                track = tracks[forecast['id']]
                samples = history.setdefault(track['id'], [])
                if samples and track['last_seen'] < samples[-1]['last_seen']:
                    samples.clear()
                if not samples or track['last_seen'] > samples[-1]['last_seen']:
                    samples.append(track)
                samples[:] = [s for s in samples if track['last_seen']-s['last_seen'] <= 0.8+1e-9][-32:]
                v = math.hypot(*xy(track['velocity']))
                dt = forecast['dt']
                positions = list(map(xy, forecast['positions']))
                origin = xy(track['position'])
                baseline = [(origin[0]+track['velocity']['x']*i*dt*(v >= 0.7),
                             origin[1]+track['velocity']['y']*i*dt*(v >= 0.7)) for i in range(len(positions))]
                changed = any(math.dist(a, b) > 1e-8 for a, b in zip(positions, baseline))
                forecasts += 1
                stale = now-track['last_seen'] > 0.15+1e-9
                reaccelerating = len(samples) >= 2 and v >= math.hypot(*xy(samples[-2]['velocity']))
                if changed and (stale or reaccelerating or v < 0.7):
                    raise ValueError('braking forecast lacks fresh decreasing-speed support')
                stale_fallbacks += int(stale and not changed)
                reacceleration_fallbacks += int(reaccelerating and not changed)
                if not changed:
                    continue
                braking_forecasts += 1
                old = min(samples, key=lambda s: abs(s['last_seen']-(track['last_seen']-0.6)))
                if abs(track['last_seen']-old['last_seen']-0.6) > 0.025 or math.hypot(*xy(old['velocity']))-v < 0.18-1e-8:
                    raise ValueError('braking hypothesis appeared without sustained measured deceleration')
                if len(positions) != 41 or abs(dt-0.2) > 1e-9 or math.dist(origin, positions[0]) > 1e-8:
                    raise ValueError('forecast horizon, spacing or initial state changed')
                direction = (track['velocity']['x']/v, track['velocity']['y']/v)
                distance = [(p[0]-origin[0])*direction[0]+(p[1]-origin[1])*direction[1] for p in positions]
                for p, d in zip(positions, distance):
                    if math.dist(p, (origin[0]+direction[0]*d, origin[1]+direction[1]*d)) > 1e-8:
                        raise ValueError('braking forecast invented lateral motion')
                means = [(b-a)/dt for a, b in zip(distance, distance[1:])]
                if min(means) < -1e-8 or max(means) > v+1e-8:
                    raise ValueError('braking forecast reverses or accelerates')
                for a, b in zip(means, means[1:]):
                    deceleration = (a-b)/dt
                    if not -1e-8 <= deceleration <= 2.0+1e-8:
                        raise ValueError('forecast exceeds bounded deceleration')
                    max_deceleration = max(max_deceleration, deceleration)
                if max(means[5:])-min(means[5:]) > 1e-8:
                    raise ValueError('braking assumption persisted beyond one second')
    return {'verified': True, 'forecasts': forecasts, 'braking_forecasts': braking_forecasts,
            'stale_fallbacks': stale_fallbacks, 'reacceleration_fallbacks': reacceleration_fallbacks,
            'max_forecast_deceleration_m_s2': max_deceleration}


def control_metrics(log):
    """Measure actual emitted commands, including emergency transitions."""
    count = emergency = pairs = 0
    squared_acceleration_changes = max_normal_steering_rate = 0.0
    previous = None
    with log.open() as stream:
        for line in stream:
            record = json.loads(line)
            if record.get('kind') != 'tick':
                continue
            current = record['tick']['expected']
            count += 1
            emergency += int(current['emergency'])
            if previous:
                duration = current['time'] - previous['time']
                if duration <= 0:
                    raise ValueError('command clock must increase')
                acceleration_change = (current['command']['acceleration'] - previous['command']['acceleration']) / duration
                squared_acceleration_changes += acceleration_change**2
                pairs += 1
                if not current['emergency'] and not previous['emergency']:
                    rate = abs(current['command']['steering'] - previous['command']['steering']) / duration
                    max_normal_steering_rate = max(max_normal_steering_rate, rate)
            previous = current
    if not count or not pairs:
        raise ValueError('insufficient command samples')
    return {'ticks': count, 'emergency_ticks': emergency, 'emergency_fraction': emergency / count,
            'commanded_acceleration_change_rms_m_s3': math.sqrt(squared_acceleration_changes / pairs),
            'max_normal_commanded_steering_rate_rad_s': max_normal_steering_rate}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['all', *CASES], default='all')
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/hazards')
    args = parser.parse_args()
    if any(seed < 0 or seed > 2**64-1 for seed in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('seeds must be distinct unsigned 64-bit integers')
    args.output.mkdir(parents=True, exist_ok=True)
    report_file = args.output/'report.json'
    report_file.unlink(missing_ok=True)
    backends = list(CASES) if args.backend == 'all' else [args.backend]
    cli = ROOT/'target/release/rustdrive'
    rne = ROOT/'integrations/rne/target/release/rustdrive-rne'
    if not cli.is_file() or ('rne-dynamic' in backends and not rne.is_file()):
        raise SystemExit('Build release binaries first: bash scripts/check-hazards.sh')
    revision = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    if 'rne-dynamic' in backends:
        head = subprocess.run(['git', '-C', str(ROOT.parent/'RobotNativeEngine'), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
        if head != revision:
            raise SystemExit('RNE revision differs from the committed pin; existing checkout is preserved')
    report = {'schema_version': 1, 'source_fingerprint_sha256': source_fingerprint(),
              'rne_revision': revision if 'rne-dynamic' in backends else None,
              'seeds': args.seeds, 'runs': [], 'passed': True}
    for backend in backends:
        for case in CASES[backend]:
            for seed in args.seeds:
                output = args.output/backend/case/f'seed-{seed}'
                output.mkdir(parents=True, exist_ok=True)
                # Remove previous success evidence before invoking a fresh run.
                summary_file = output/'summary.json'
                summary_file.unlink(missing_ok=True)
                replay_file = output/'replay/replay.json'
                replay_file.unlink(missing_ok=True)
                command = [cli, 'run'] if backend == 'reference' else [rne, '--plant', 'dynamic']
                command += ['--scenario', ROOT/'scenarios'/f'{case}.json', '--seed', seed, '--output', output]
                code, error = invoke(command)
                row = {'backend': backend, 'scenario': case, 'seed': seed, 'exit_code': code,
                       'output': str(output), 'passed': False}
                if summary_file.exists():
                    summary = json.loads(summary_file.read_text())
                    row['summary'] = summary
                    code_replay, replay_error = invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                    row['replay_exit_code'] = code_replay
                    if replay_file.exists():
                        row['replay'] = json.loads(replay_file.read_text())
                    ok = (code == 0 and summary['passed'] and summary['collisions'] == 0
                          and summary['road_violations'] == 0 and code_replay == 0
                          and summary.get('traffic_collisions', 0) == 0 and summary.get('traffic_road_violations', 0) == 0
                          and summary.get('closure_violations', 0) == 0 and summary.get('signal_violations', 0) == 0
                          and row.get('replay', {}).get('verified') is True
                          and row['replay']['ticks'] == summary['steps'])
                    row['speed_profiles'] = check_speed_profiles(output/'sensors.jsonl')
                    row['motion_predictions'] = check_motion_predictions(output/'sensors.jsonl')
                    if case == 'traffic-follower-deadline' and not row['motion_predictions']['braking_forecasts']:
                        raise ValueError('deadline regression did not exercise observed-braking forecasts')
                    row['control_metrics'] = control_metrics(output/'sensors.jsonl')
                    row['clearance_regression'] = clearance_regression(summary, case)
                    ok &= row['clearance_regression']['passed']
                    ok &= row['control_metrics']['max_normal_commanded_steering_rate_rad_s'] <= 0.7 + 1e-8
                    if backend == 'rne-dynamic' and case == 'low-friction':
                        row['tracking_regression_passed'] = summary['emergency_steps'] <= 20
                        ok &= row['tracking_regression_passed']
                    run = json.loads((output/'run.json').read_text())
                    if case.startswith('signal-'):
                        row['traffic_controls'] = check_signals(run, output/'sensors.jsonl')
                    if case.startswith('gnss-'):
                        row['gnss_fault'] = check_gnss_fault(run, output/'sensors.jsonl', case)
                    if case.startswith('traffic-'):
                        row['traffic'] = check_traffic(run, output/'sensors.jsonl', case)
                    if case == 'gnss-burst-traffic-hold':
                        row['goal_hold'] = check_goal_hold(run, output/'sensors.jsonl')
                    if case in EXPECTED_EDGES:
                        row['navigation'] = check_navigation(run, case)
                    if case in ['route-handover', 'route-handover-fast', 'route-no-path', 'route-reopen']:
                        row['live_navigation'] = check_live_navigation(run, output/'sensors.jsonl', case)
                    if run['scenario'].get('dynamics'):
                        frames = run['frames']
                        acceleration = max(abs((b['truth']['speed']-a['truth']['speed'])/(b['time']-a['time'])) for a,b in zip(frames, frames[1:]))
                        row['max_measured_abs_longitudinal_acceleration_m_s2'] = acceleration
                        ok &= acceleration <= run['scenario']['dynamics']['friction_coefficient']*9.81+1e-8
                    row['passed'] = bool(ok)
                    if replay_error:
                        row['replay_stderr'] = replay_error
                if error:
                    row['stderr'] = error
                report['runs'].append(row)
                report['passed'] &= row['passed']
                print(f"{backend:11s} {case:20s} seed {seed:3d}: {'PASS' if row['passed'] else 'FAIL'}", flush=True)
    if 'rne-dynamic' in backends:
        report['known_failures'] = []
        for seed in [1, 42]:
            output = args.output/'known-failure/traffic-follower-short-range'/f'seed-{seed}'
            output.mkdir(parents=True, exist_ok=True)
            (output/'summary.json').unlink(missing_ok=True)
            (output/'replay/replay.json').unlink(missing_ok=True)
            code, error = invoke([rne, '--plant', 'dynamic', '--scenario', ROOT/'scenarios/traffic-follower-short-range.json', '--seed', seed, '--output', output])
            summary = json.loads((output/'summary.json').read_text())
            replay_code, replay_error = invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
            replay = json.loads((output/'replay/replay.json').read_text())
            run = json.loads((output/'run.json').read_text())
            traffic = check_traffic(run, output/'sensors.jsonl', 'traffic-follower-short-range')
            predictions = check_motion_predictions(output/'sensors.jsonl')
            rejected = (code == 1 and not summary['passed'] and not summary['reached_goal']
                        and summary['collisions'] == 0 and summary['road_violations'] == 0
                        and summary.get('traffic_collisions', 0) == 0 and summary.get('traffic_road_violations', 0) == 0
                        and summary['min_clearance'] < 1.0 and replay_code == 0 and replay['verified']
                        and replay['ticks'] == summary['steps'] and any(f.startswith('minimum swept clearance') for f in summary['failures']))
            report['known_failures'].append({'backend': 'rne-dynamic', 'scenario': 'traffic-follower-short-range',
                'seed': seed, 'exit_code': code, 'summary': summary, 'replay': replay,
                'traffic': traffic, 'motion_predictions': predictions, 'acceptance_rejection_verified': bool(rejected)})
            report['passed'] &= bool(rejected)
            print(f'rne-dynamic traffic-follower-short-range seed {seed}: short-range clearance failure; rejection verified={rejected}', flush=True)
    report_file.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_file}: {len(report["runs"])} runs; passed={report["passed"]}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'hazard check: {error}', file=sys.stderr)
        sys.exit(2)
