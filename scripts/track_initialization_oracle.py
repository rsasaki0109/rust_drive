"""Batch-fit oracle for optional acquired-track initialization and enclosure.

Uses complete raw associated measurements, not filtered positions or truth.
Raw-sensor replay separately checks clustering and association. This oracle
checks filtering, support ownership and unchanged cached acquisitions.
"""
import copy
import math
from tracking_frame import continuous_observations


def xy(p):
    values = p['x'], p['y']
    if not all(math.isfinite(v) for v in values): raise ValueError('nonfinite vector')
    return values


def fitted(history):
    """Independent dense centered sums and direct residuals, without recurrence."""
    first = history[0]['stamp']; n = len(history)
    times = [m['stamp']-first for m in history]
    if n < 3 or times[-1] < .2-1e-9: return None
    mean_t = math.fsum(times)/n
    mean = [math.fsum(m['center'][k] for m in history)/n for k in ['x','y']]
    denominator = math.fsum((t-mean_t)**2 for t in times)
    if denominator <= 1e-12: return None
    velocity = tuple(math.fsum((t-mean_t)*(m['center'][k]-mean[j]) for t,m in zip(times,history))/denominator
                     for j,k in enumerate(['x','y']))
    residuals = [math.fsum((m['center'][k]-mean[j]-velocity[j]*(t-mean_t))**2 for j,k in enumerate(['x','y']))
                 for t,m in zip(times,history)]
    rms = math.sqrt(math.fsum(residuals)/n)
    if rms > .12 or math.hypot(*velocity) > 15.: return None
    return tuple(mean[j]+velocity[j]*(times[-1]-mean_t) for j in range(2)), velocity, rms


def reconstruct(config, ticks):
    confirm=config.get('confirm_track_initialization') is True
    enclose=config.get('enclose_track_measurements') is True
    recent_fit=config.get('fit_recent_track_motion') is True
    if recent_fit and (confirm or not enclose): raise ValueError('unsupported recent-motion tracker policies')
    if not confirm and not enclose: raise ValueError('tracked measurement policy missing')
    tracks = {}; pending = {}; recent = {}; moving_support = {}; cached = []; last_scan = None; next_id = 0
    recent_updates = recent_unsupported = recent_maximum_samples = 0
    epoch = 0; invalid_motion = False; confirmations = []; births = updates = coasts = 0
    maximum_position_error = maximum_velocity_error = 0.
    for index,tick in enumerate(ticks):
        inp,out = tick['input'],tick['expected']; now=inp['time']
        if out['time'] != now: raise ValueError('filter/control clock differs')
        if inp.get('multi_height_lidar') or inp.get('lidar3d'): raise ValueError('oracle requires XY scan inputs')
        lost = out.get('motion_discontinuity') is not None
        new_epoch = out.get('localization_epoch',0)
        if new_epoch != epoch or (lost and not invalid_motion):
            tracks={};pending={};recent={};moving_support={};cached=[];last_scan=None;next_id=0;epoch=new_epoch
        invalid_motion=lost
        measurements=out.get('track_measurements')
        if not isinstance(measurements,list): raise ValueError('raw track measurements missing')
        scan=inp.get('lidar'); health=out['health']
        accepted=(scan is not None and not lost and not inp.get('lidar_failed',False)
                  and not {'InvalidLidar','AcquisitionFailed'}.intersection(health)
                  and math.isfinite(scan['stamp']) and 0.<=scan['stamp']<=now
                  and now-scan['stamp']<=.35+1e-9
                  and (last_scan is None or scan['stamp']>last_scan))
        if accepted:
            stamp=scan['stamp'];last_scan=stamp
            tracks={i:t for i,t in tracks.items() if stamp-t['last_seen']<.6}
            pending={i:h for i,h in pending.items() if i in tracks}
            recent={i:h for i,h in recent.items() if i in tracks}
            moving_support={i:t for i,t in moving_support.items() if i in tracks}
            seen=set()
            for m in measurements:
                identity=m['id'];center=xy(m['center']);radius=m['radius']
                if (identity in seen or m['stamp']!=stamp or not math.isfinite(radius) or radius<0.):
                    raise ValueError('invalid or duplicated raw acquisition')
                seen.add(identity)
                if identity not in tracks:
                    if identity!=next_id+1: raise ValueError('identity allocation changed')
                    next_id=identity;births+=1
                    tracks[identity]=dict(position=center,velocity=(0.,0.),radius=radius,last_seen=stamp)
                    if confirm: pending[identity]=[copy.deepcopy(m)]
                    if recent_fit: recent[identity]=[copy.deepcopy(m)]
                    continue
                old=tracks[identity];dt=stamp-old['last_seen']
                if not 0.<dt<.6: raise ValueError('invalid per-track interval')
                predicted=tuple(p+v*dt for p,v in zip(old['position'],old['velocity']))
                if math.dist(predicted,center)>3.+1e-8: raise ValueError('association gate changed')
                residual=tuple(p-q for p,q in zip(center,predicted))
                position=tuple(p+.75*r for p,r in zip(predicted,residual));velocity=old['velocity']
                if identity in pending:
                    history=pending[identity]
                    radii=[h['radius'] for h in history]+[radius]
                    if stamp-history[0]['stamp']>.6+1e-9 or max(radii)-min(radii)>.12+1e-9:
                        history[:]=[copy.deepcopy(m)]
                    else: history.append(copy.deepcopy(m))
                    result=fitted(history)
                    if result is not None:
                        position,velocity,rms=result
                        confirmations.append(dict(tick=index,id=identity,stamp=stamp,samples=len(history),
                                                  span_s=stamp-history[0]['stamp'],rms_m=rms,speed_m_s=math.hypot(*velocity)))
                        del pending[identity]
                else:
                    velocity=tuple(v+.12*r/max(.01,min(dt,1.)) for v,r in zip(velocity,residual))
                if recent_fit:
                    history=recent[identity]
                    if history and stamp-history[-1]['stamp']>.35+1e-9:
                        history=[];moving_support.pop(identity,None)
                    retained=history
                    retained=[h for h in retained if stamp-h['stamp']<=.6+1e-9]+[copy.deepcopy(m)]
                    retained=retained[-32:]
                    starts=[i for i in range(max(0,len(retained)-2)) if stamp-retained[i]['stamp']>=.2-1e-9]
                    recent[identity]=retained[max(starts):] if starts else retained
                    history=recent[identity]
                    recent_maximum_samples=max(recent_maximum_samples,len(history))
                    radii=[h['radius'] for h in history]
                    result=fitted(history) if max(radii)-min(radii)<=.12+1e-9 else None
                    if result is not None and math.hypot(*result[1])>=.7:moving_support[identity]=stamp
                    if identity not in moving_support or stamp-moving_support[identity]>.6+1e-9:result=None
                    if result is not None:
                        position,velocity,_=result;recent_updates+=1
                    else:recent_unsupported+=1
                speed=math.hypot(*velocity)
                if speed>15.: velocity=tuple(v*15./speed for v in velocity)
                smoothed=.8*old['radius']+.2*radius
                extent=max(smoothed,math.dist(position,center)+radius) if enclose else smoothed
                tracks[identity]=dict(position=position,velocity=velocity,radius=extent,last_seen=stamp)
                updates+=1
            cached=copy.deepcopy(measurements)
        elif measurements!=cached: raise ValueError('cached raw acquisitions changed or renewed')
        else: coasts+=1
        reported,_=continuous_observations(out)
        if len(reported)!=len(tracks) or len({t['id'] for t in reported})!=len(reported):
            raise ValueError('track set changed')
        for t in reported:
            if t['id'] not in tracks: raise ValueError('unobserved track')
            old=tracks[t['id']]
            pe=math.dist(xy(t['position']),old['position']);ve=math.dist(xy(t['velocity']),old['velocity'])
            maximum_position_error=max(maximum_position_error,pe);maximum_velocity_error=max(maximum_velocity_error,ve)
            if (pe>1e-7 or ve>1e-7 or abs(t['radius']-old['radius'])>1e-10 or t['last_seen']!=old['last_seen']):
                raise ValueError(f'filter reconstruction differs at tick {index}, id {t["id"]}: position={pe}, velocity={ve}')
    result=dict(passed=True,ticks=len(ticks),births=births,updates=updates,cached_ticks=coasts,
                confirmations=confirmations,maximum_position_difference_m=maximum_position_error,
                maximum_velocity_difference_m_s=maximum_velocity_error,
                independent_batch_fit=True,clustering_association_checked_by_sensor_replay=True)
    if recent_fit:result['recent_motion_fit']=dict(fit_updates=recent_updates,unsupported_updates=recent_unsupported,
                                                 maximum_samples=recent_maximum_samples,independent_dense_batch=True)
    return result
