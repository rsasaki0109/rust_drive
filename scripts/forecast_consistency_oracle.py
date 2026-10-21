"""Audit measured forecast containment and braking from sensor-only recordings.

Forecasts are transformed back into continuous wheel/gyro coordinates. The
radial boundary of each new measured disc is tested against an older forecast;
physical actor truth never enters this audit. Independent physical coverage is
still scored by check_forecast_envelope, including cold acquisitions.
"""
import collections
import math

from tracking_frame import continuous_observations
from planar_sweep_oracle import acquired as acquired_sweep, blank_recovery

DOWNSTREAM = {'InconsistentForecast', 'InvalidNavigation', 'InvalidStopSignRecovery',
              'InvalidTrafficSignalRecovery', 'InvalidIntersectionRecovery', 'InvalidTrafficSignal'}


def blank(clear=False):
    value = dict(state='Observing', fault=None, last_scan_acquisition_s=None,
                evaluated_acquisition_s=None, evaluated_at_control_s=None,
                fault_control_time_s=None, recovery_after_control_s=None,
                checked_measurements=0, unmonitored_measurements=0, missed_measurements=0,
                complete_scene_check=False, recovery_samples=0,
                recovery_first_acquisition_s=None, held_identities=[], misses=[], retained_references=0)
    if clear:value['clear_sweep']=blank_recovery()
    return value


def compare(actual, expected, path='forecast_consistency'):
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or actual.keys() != expected.keys():
            raise ValueError(path+': diagnostic schema differs')
        for k, v in expected.items():compare(actual[k], v, path+'.'+k)
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) != len(expected):
            raise ValueError(path+': witness count differs')
        for i, v in enumerate(expected):compare(actual[i], v, path+f'[{i}]')
    elif isinstance(expected, float):
        if type(actual) not in (int, float) or not math.isfinite(actual) or abs(actual-expected) > 2e-8:
            raise ValueError(path+': geometry or clock differs')
    elif type(actual) is not type(expected) or actual != expected:
        raise ValueError(path+': state or ownership differs')


def witness(reference, measured):
    origin, acquisition, p = reference
    elapsed = measured['stamp']-origin
    last = len(p['positions'])-1
    if elapsed < 0. or elapsed > last*p['dt']:raise ValueError('forecast horizon cannot be extended')
    f, i = math.modf(elapsed/p['dt']);i = min(int(i), last);j = min(i+1, last)
    center = {k: (1.-f)*p['positions'][i][k]+f*p['positions'][j][k] for k in ['x', 'y']}
    extra = p.get('additional_radius_m') or [0.]*len(p['positions'])
    radius = p['radius']+(1.-f)*extra[i]+f*extra[j]
    dx, dy = measured['center']['x']-center['x'], measured['center']['y']-center['y']
    distance = math.hypot(dx, dy)
    direction = (dx/distance, dy/distance) if distance else (1., 0.)
    boundary = [measured['center'][k]+measured['radius']*direction[n] for n, k in enumerate(['x', 'y'])]
    shortfall = math.dist(boundary, [center[k] for k in ['x', 'y']])-radius
    return dict(id=measured['id'], reference_origin_s=origin, reference_acquisition_s=acquisition,
                measurement_acquisition_s=measured['stamp'], predicted_center=center,
                predicted_radius_m=radius, measured_center=measured['center'],
                measured_radius_m=measured['radius'], shortfall_m=shortfall)


def validate_geometry(stamp, tracks, predictions, measurements):
    if max(len(tracks), len(predictions), len(measurements)) > 128:raise ValueError('unsupported identity capacity')
    if len({t['id'] for t in tracks}) != len(tracks) or len({m['id'] for m in measurements}) != len(measurements):
        raise ValueError('duplicate measurement/track identity')
    active = {t['id']: t for t in tracks};forecasts = {p['id']: p for p in predictions}
    if len(forecasts) != len(predictions) or active.keys() != forecasts.keys():raise ValueError('forecast ownership differs')
    def finite(values):return all(type(v) in (int, float) and math.isfinite(v) for v in values)
    for t in tracks:
        if (not finite([*t['position'].values(), *t['velocity'].values(), t['radius'], t['last_seen']])
                or t['radius'] < 0. or not 0. <= t['last_seen'] <= stamp):raise ValueError('invalid tracked geometry')
    for p in predictions:
        extra = p.get('additional_radius_m') or []
        if (not 1 <= len(p['positions']) <= 64 or not finite([p['radius'], p['dt'], *extra])
                or p['radius'] < 0. or p['dt'] <= 0. or min(extra, default=0.) < 0.
                or (extra and len(extra) != len(p['positions']))
                or any(not finite(q.values()) for q in p['positions'])):raise ValueError('invalid forecast geometry')
    for m in measurements:
        if (not finite([*m['center'].values(), m['radius'], m['stamp']]) or m['radius'] < 0.
                or m['stamp'] != stamp or m['id'] not in active
                or active[m['id']]['last_seen'] != stamp):raise ValueError('invalid acquired measurement ownership')
    return active, forecasts


def reconstruct(config, ticks):
    if (config.get('forecast_consistency_guard') is not True or not config.get('forecast_envelope')
            or config.get('enclose_track_measurements') is not True):raise ValueError('guard prerequisites missing')
    if config.get('multi_height_lidar') or config.get('lidar3d'):
        raise ValueError('this independent acquisition audit supports planar LiDAR recordings')
    policy=config.get('clear_sweep_recovery')
    diagnostic = blank(policy is not None);history = {};held = set();accepted = None;last_vote = None;epoch = 0
    reset_required = False;previous = None;previous_blocked = False
    counters = collections.Counter();recoveries = [];first_miss = None;maximum_shortfall = 0.
    first_hold = None;clear_recoveries=[]
    for tick in ticks:
        inp, out = tick['input'], tick['expected'];now = inp['time']
        if not math.isfinite(now) or now < 0. or (previous is not None and now <= previous) or out['time'] != now:
            raise ValueError('invalid control clock')
        previous = now
        if out.get('localization_epoch', 0) != epoch:
            if out['localization_epoch'] != epoch+1:raise ValueError('localization epoch skipped')
            epoch += 1;diagnostic = blank(policy is not None);history = {};held = set();accepted = last_vote = None
            reset_required = previous_blocked = False;first_hold = None
        def reset_votes():
            nonlocal last_vote
            diagnostic['recovery_samples'] = 0;diagnostic['recovery_first_acquisition_s'] = None;last_vote = None
            reset_clear()
        def reset_clear():
            if policy is not None:
                diagnostic['clear_sweep'].update(confirmation_samples=0,first_acquisition_s=None,last_acquisition_s=None)
        if out.get('motion_discontinuity') and not reset_required:
            history = {};reset_required = True
            diagnostic.update(fault='MotionDiscontinuity', fault_control_time_s=now, recovery_after_control_s=now)
            reset_votes()
        scan = inp.get('lidar')
        if scan is not None and not inp.get('lidar_failed', False) and not out.get('motion_discontinuity') and 'InvalidLidar' not in out['health']:
            stamp = scan['stamp']
            if not math.isfinite(stamp) or not 0. <= stamp <= now:raise ValueError('invalid accepted acquisition')
            accepted = max(accepted if accepted is not None else stamp, stamp)
        healthy = not set(out['health']).difference(DOWNSTREAM)
        new = accepted is not None and (diagnostic['last_scan_acquisition_s'] is None or accepted > diagnostic['last_scan_acquisition_s'])
        if not reset_required and not healthy:
            if diagnostic['fault']:
                diagnostic['recovery_after_control_s'] = now;reset_votes()
            if new:diagnostic['last_scan_acquisition_s'] = accepted
        elif not reset_required and new:
            diagnostic['last_scan_acquisition_s'] = accepted
            tracks, predictions = continuous_observations(out)
            raw = out.get('track_measurements')
            if not isinstance(raw, list):raise ValueError('raw measurement evidence missing')
            active, forecasts = validate_geometry(accepted, tracks, predictions, raw)
            clear_proof=None
            if policy is not None and scan is not None and scan['stamp']==accepted:
                proof=acquired_sweep(scan,policy);motion=inp.get('odometry')
                if (proof is not None and accepted==now and motion is not None
                        and motion['stamp']==now and abs(motion['speed'])<=.2):
                    if raw:raise ValueError('empty acquired rays contradict measured proxies')
                    clear_proof=proof;diagnostic['clear_sweep']['last_evidence']=proof
                    counters['clear_qualifying_acquisitions']+=1
            comparable = set();misses = []
            for measured in raw:
                eligible = [r for r in history.get(measured['id'], [])
                            if r[1] < accepted and r[0] <= accepted <= r[0]+r[2]['dt']*(len(r[2]['positions'])-1)]
                if not eligible:continue
                reference = eligible[-1];comparable.add(measured['id'])
                w = witness(reference, measured)
                if w['shortfall_m'] > 1e-8:misses.append(w)
            complete = active.keys() <= comparable and held <= comparable
            diagnostic.update(evaluated_acquisition_s=accepted, evaluated_at_control_s=now,
                              checked_measurements=len(comparable), unmonitored_measurements=len(raw)-len(comparable),
                              missed_measurements=len(misses), complete_scene_check=complete, misses=misses)
            counters['acquisitions'] += 1;counters['checked_measurements'] += len(comparable)
            counters['unmonitored_measurements'] += len(raw)-len(comparable);counters['missed_measurements'] += len(misses)
            if misses:
                held.update(w['id'] for w in misses)
                if len(held) > 128:raise ValueError('unsupported held identity capacity')
                diagnostic.update(fault='MeasuredMiss', fault_control_time_s=now, recovery_after_control_s=now)
                reset_votes();counters['missed_acquisitions'] += 1
                maximum_shortfall = max(maximum_shortfall, max(w['shortfall_m'] for w in misses))
                if first_miss is None:first_miss = dict(control_time_s=now, **misses[0])
            elif diagnostic['fault']:
                if (clear_proof is not None and diagnostic['fault']=='MeasuredMiss' and held
                        and accepted>diagnostic['recovery_after_control_s']):
                    diagnostic.update(recovery_samples=0,recovery_first_acquisition_s=None);last_vote=None
                    clear=diagnostic['clear_sweep']
                    if clear['last_acquisition_s'] is None or accepted-clear['last_acquisition_s']>.35+1e-9:
                        reset_clear();clear['first_acquisition_s']=accepted
                    clear['confirmation_samples']=min(3,clear['confirmation_samples']+1);clear['last_acquisition_s']=accepted
                    if clear['confirmation_samples']>=3 and accepted-clear['first_acquisition_s']>=.2-1e-9:
                        clear['release_count']+=1
                        clear['last_release']=dict(identities=sorted(held),control_time_s=now,
                            first_acquisition_s=clear['first_acquisition_s'],last_acquisition_s=accepted,
                            acquisition_count=clear['confirmation_samples'])
                        clear_recoveries.append(dict(clear['last_release']))
                        recoveries.append(dict(hold_start_control_s=first_hold,release_control_s=now,
                            first_acquisition_s=clear['first_acquisition_s'],last_acquisition_s=accepted,basis='CompleteEmptySweep'))
                        diagnostic.update(fault=None,fault_control_time_s=None,recovery_after_control_s=None)
                        held.clear();reset_votes()
                elif complete and accepted > diagnostic['recovery_after_control_s']:
                    reset_clear()
                    if last_vote is None or accepted-last_vote > .35+1e-9:
                        reset_votes();diagnostic['recovery_first_acquisition_s'] = accepted
                    diagnostic['recovery_samples'] = min(3, diagnostic['recovery_samples']+1);last_vote = accepted
                    if diagnostic['recovery_samples'] >= 3 and accepted-diagnostic['recovery_first_acquisition_s'] >= .2-1e-9:
                        recoveries.append(dict(hold_start_control_s=first_hold, release_control_s=now,
                                               first_acquisition_s=diagnostic['recovery_first_acquisition_s'], last_acquisition_s=accepted))
                        diagnostic.update(fault=None, fault_control_time_s=None, recovery_after_control_s=None)
                        held.clear();reset_votes()
                else:reset_votes()
            history = {k: v for k, v in history.items() if k in active}
            for m in raw:
                history.setdefault(m['id'], collections.deque(maxlen=16)).append((now, accepted, forecasts[m['id']]))
        elif not reset_required:counters['cached_control_ticks'] += 1
        blocked = diagnostic['fault'] is not None
        confirming=diagnostic['recovery_samples'] or (policy is not None and diagnostic['clear_sweep']['confirmation_samples'])
        diagnostic['state'] = 'ResetRequired' if reset_required else ('Confirming' if confirming else 'Holding') if blocked else 'Observing'
        diagnostic['held_identities'] = sorted(held);diagnostic['retained_references'] = sum(map(len, history.values()))
        compare(out.get('forecast_consistency'), diagnostic)
        if ('InconsistentForecast' in out['health']) != blocked:raise ValueError('forecast hold health differs')
        if blocked:
            if not previous_blocked:first_hold = now;counters['hold_entries'] += 1
            if (out.get('emergency') is not True or out['command']['acceleration'] > -5.99
                    or abs(out['command']['steering']) > 1e-12):raise ValueError('forecast fault lost same-cycle emergency braking')
            counters['braking_control_ticks'] += 1
        else:first_hold = None
        previous_blocked = blocked;counters['verified_control_ticks'] += 1
    result = dict(passed=True, **{k: counters[k] for k in ['verified_control_ticks', 'acquisitions', 'checked_measurements',
                'unmonitored_measurements', 'missed_measurements', 'missed_acquisitions', 'cached_control_ticks',
                'hold_entries', 'braking_control_ticks']}, recoveries=recoveries, first_miss=first_miss,
                maximum_shortfall_m=maximum_shortfall, final_state=diagnostic['state'],
                final_held_identities=diagnostic['held_identities'], sensor_only=True,
                reference_horizons_respected=True, physical_uncertainty_calibrated=False)
    if policy is not None:result.update(clear_sweep_recoveries=clear_recoveries,clear_qualifying_acquisitions=counters['clear_qualifying_acquisitions'])
    return result
