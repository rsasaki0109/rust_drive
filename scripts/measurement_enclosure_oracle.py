"""Certify acquired discs using boundary witnesses in the continuous frame.

The corrected center is fixed. Containment and an active boundary witness prove
the minimum sufficient radius for the smoothed disc and newest measured disc.
This does not assert that the measured proxy encloses an unseen physical body.
"""
import math
from tracking_frame import continuous_observations


def xy(p): return p['x'],p['y']


def certify(center,radius,measurement,smoothed):
    q=xy(measurement['center']);r=measurement['radius']
    if not all(math.isfinite(v) for v in [*center,radius,*q,r,smoothed]) or min(radius,r,smoothed)<0.:
        raise ValueError('invalid measured geometry')
    dx,dy=q[0]-center[0],q[1]-center[1];length=math.hypot(dx,dy)
    direction=(dx/length,dy/length) if length else (1.,0.)
    farthest=(q[0]+r*direction[0],q[1]+r*direction[1])
    # Exact radial extremum of the raw disc, plus the smoothed disc's boundary.
    witnesses=[math.dist(center,farthest),smoothed]
    deficit=max(distance-radius for distance in witnesses)
    slack=radius-max(witnesses)
    if deficit>1e-8: raise ValueError('newest measured disc or smoothed extent was lost')
    if slack>1e-8: raise ValueError('radius exceeds the minimum measured enclosure')
    return deficit,slack


def reconstruct(config,ticks):
    if config.get('enclose_track_measurements') is not True: raise ValueError('measurement enclosure policy missing')
    previous={};seen=set();epoch=0;measurements=updates=cached=0
    maximum_deficit=maximum_slack=maximum_extension=0.
    for tick in ticks:
        out=tick['expected'];current,_=continuous_observations(out)
        if out.get('localization_epoch',0)!=epoch:
            previous={};seen=set();epoch=out.get('localization_epoch',0)
        tracks={t['id']:t for t in current}
        raw=out.get('track_measurements')
        if not isinstance(raw,list):raise ValueError('raw measurement evidence missing')
        if len({m['id'] for m in raw})!=len(raw):raise ValueError('duplicate raw measurement ownership')
        acquired={m['id']:m for m in raw}
        for identity,t in tracks.items():
            old=previous.get(identity)
            if old is None or t['last_seen']!=old['last_seen']:
                if identity not in acquired or acquired[identity]['stamp']!=t['last_seen']:
                    raise ValueError('new track observation lacks raw measurement evidence')
        for m in raw:
            if m['id'] not in tracks:raise ValueError('raw measurement has no retained track')
            key=(m['id'],m['stamp']);t=tracks[m['id']]
            if t['last_seen']!=m['stamp']:raise ValueError('measurement/track acquisition ownership differs')
            if key in seen:continue
            seen.add(key);old=previous.get(m['id'])
            smoothed=.8*old['radius']+.2*m['radius'] if old else m['radius']
            deficit,slack=certify(xy(t['position']),t['radius'],m,smoothed)
            maximum_deficit=max(maximum_deficit,deficit);maximum_slack=max(maximum_slack,slack)
            maximum_extension=max(maximum_extension,t['radius']-smoothed)
            measurements+=1;updates+=old is not None
        for identity,t in tracks.items():
            old=previous.get(identity)
            if old and t['last_seen']==old['last_seen']:
                if abs(t['radius']-old['radius'])>1e-10:raise ValueError('cached radius changed without an acquisition')
                cached+=1
        previous=tracks
    return dict(passed=True,acquired_measurements=measurements,associated_updates=updates,cached_tracks=cached,
                maximum_containment_deficit_m=maximum_deficit,maximum_nonminimal_slack_m=maximum_slack,
                maximum_extension_above_smoothed_radius_m=maximum_extension,exact_boundary_witnesses=True,
                physical_unseen_shape_coverage_claimed=False)
