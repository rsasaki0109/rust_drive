#!/usr/bin/env python3
"""Find contained constant-motion witnesses that must block uncommitted lane entry.

This checks sufficient evidence of a conflict by dense time sampling. It does
not certify clear space, replay sensors, or reconstruct the entire gate policy.
"""
import argparse
import gzip
import json
import math
from pathlib import Path


def first_witness(config, output, track, forecast):
    lane = config['lane_change']
    start, end = lane['target_start'], lane['target_end']
    length = math.hypot(end['x']-start['x'], end['y']-start['y'])
    axis = ((end['x']-start['x'])/length, (end['y']-start['y'])/length)

    def project(p):
        x, y = p['x']-start['x'], p['y']-start['y']
        return x*axis[0]+y*axis[1], -x*axis[1]+y*axis[0]

    age = output['time']-track['last_seen']
    if age < 0:
        raise ValueError('future acquired witness')
    x0, y0 = project(track['position'])
    v = track['velocity']
    vx, vy = v['x']*axis[0]+v['y']*axis[1], -v['x']*axis[1]+v['y']*axis[0]
    ego_x = project(output['estimate']['pose']['position'])[0]
    ego_speed = max(config['cruise_speed'], output['estimate']['speed'])+.5
    body = config['vehicle']['radius']+track['radius']
    behind = body+6.+2.*max(0., vx)
    ahead = body+6.+2.*ego_speed
    lateral = lane['target_width_m']/2.+track['radius']+.5
    for i in range(1201):
        time = i/200.
        x, y = x0+vx*(age+time), y0+vy*(age+time)
        # Strict interior samples avoid claiming a conflict from boundary roundoff.
        if (x > ego_x-behind+1e-7 and x < ego_x+ego_speed*time+ahead-1e-7
                and abs(y) < lateral-1e-7):
            world = (track['position']['x']+v['x']*(age+time),
                     track['position']['y']+v['y']*(age+time))
            points, extra, dt = forecast['positions'], forecast['additional_radius_m'], forecast['dt']
            if len(extra) != len(points) or len(points) < 2 or dt <= 0:
                raise ValueError('malformed witness occupancy')
            step = time/dt
            knot = min(math.floor(step), len(points)-2)
            fraction = min(1., step-knot)
            center = tuple((1-fraction)*points[knot][a]+fraction*points[knot+1][a] for a in ['x', 'y'])
            radius = (1-fraction)*extra[knot]+fraction*extra[knot+1]
            if math.dist(center, world)+track['radius'] > forecast['radius']+radius+1e-8:
                raise ValueError('constant-motion witness is not contained in the recorded envelope')
            return {'future_seconds': time, 'longitudinal_m': x, 'lateral_m': y,
                    'continued_speed_m_s': vx, 'rear_headway_distance_m': behind}
    return None


def audit(log):
    opener = gzip.open(log, 'rt') if str(log).endswith('.gz') else log.open()
    ticks = occupied = witnesses = 0
    failures = []
    with opener as stream:
        config = json.loads(next(stream))['header']['config']
        if not config.get('lane_change'):
            raise ValueError('lane-change configuration required')
        for line in stream:
            row = json.loads(line)
            if row['kind'] != 'tick':
                continue
            output = row['tick']['expected']
            ticks += 1
            state = output['lane_change']
            if state['phase'] not in ['Waiting', 'Ready']:
                continue
            tracks = {t['id']: t for t in output['tracks']}
            for forecast in output['predictions']:
                if not forecast.get('additional_radius_m'):
                    continue
                occupied += 1
                witness = first_witness(config, output, tracks[forecast['id']], forecast)
                if witness is None:
                    continue
                witnesses += 1
                if (forecast['id'] not in state['blocking_tracks'] or state['phase'] != 'Waiting'
                        or state['clear_since'] is not None):
                    failures.append({'time': output['time'], 'track_id': forecast['id'],
                                     'phase': state['phase'], 'contained_witness': witness})
    return {'verified': not failures, 'ticks': ticks, 'uncertain_forecasts_checked': occupied,
            'contained_conflict_witnesses': witnesses, 'missing_or_unrevoked_conflicts': len(failures),
            'failures': failures, 'future_sample_interval_seconds': .005,
            'clear_space_certified': False, 'sensor_replay_verified_by_this_audit': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.log)
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result['verified'] else 1)
