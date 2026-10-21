"""Independent raw-acquisition reference for cold velocity support and reachable discs."""
import copy
import math

from observed_detour_oracle import equal
from tracking_frame import continuous_observations


def fitted(history):
    """Batch normal equations with no tracker speed clipping or recurrence."""
    n=len(history)
    if n<3 or history[-1]['stamp']-history[0]['stamp']<.2-1e-9:return None
    times=[m['stamp']-history[0]['stamp'] for m in history];mean_t=math.fsum(times)/n
    centers=[[m['center'][k]-history[0]['center'][k] for m in history] for k in ['x','y']]
    means=[math.fsum(axis)/n for axis in centers];variance=math.fsum((t-mean_t)**2 for t in times)
    if variance<=1e-12:return None
    velocity=[math.fsum((t-mean_t)*(p-mean) for t,p in zip(times,axis))/variance for axis,mean in zip(centers,means)]
    rms=math.sqrt(math.fsum((p-mean-v*(t-mean_t))**2 for axis,mean,v in zip(centers,means,velocity)
        for t,p in zip(times,axis))/n)
    return velocity if all(math.isfinite(v) for v in velocity) and rms<=.12 else None


def blank(identity):
    return dict(id=identity,confirmed=False,first_acquisition_s=None,last_acquisition_s=None,
        acquisition_count=0,last_observed_acquisition_s=None,confirmation_acquisition_s=None,
        velocity_residual_m_s=None)


class ColdMotionOracle:
    def __init__(self,config):
        self.maximum=config.get('forecast_cold_max_speed_m_s');self.bounds=config.get('forecast_envelope')
        if (type(self.maximum) not in [int,float] or not math.isfinite(self.maximum) or not .7<=self.maximum<=15.
                or self.bounds is None or config.get('fit_recent_track_motion') is not True
                or config.get('enclose_track_measurements') is not True or config.get('forecast_consistency_guard') is not True):
            raise ValueError('cold motion prerequisites missing')
        self.states={};self.accepted=None;self.fault=None;self.consumed=None;self.epoch=0
        self.ticks=self.expanded=self.confirmations=0;self.maximum_radius=0.

    def observe(self,tick,tracks):
        source=tick['input'];out=tick['expected'];now=source['time']
        if out.get('localization_epoch',0)!=self.epoch:
            self.states={};self.accepted=self.fault=self.consumed=None;self.epoch=out['localization_epoch']
        failed=any(h in out['health'] for h in ['InvalidLidar','AcquisitionFailed'])
        scan=source.get('lidar')
        if source.get('lidar_failed'):self.fault=now
        if scan is not None:
            stamp=scan['stamp'];valid=(type(stamp) in [int,float] and math.isfinite(stamp) and 0.<=stamp<=now+1e-9
                and len(scan['points'])<=20000 and all(math.isfinite(v) for p in scan['points'] for v in p.values())
                and all(math.hypot(p['x'],p['y'])<=200. for p in scan['points']))
            if not valid or source.get('lidar_failed'):self.fault=now
            elif (not failed and out.get('motion_discontinuity') is None and now-stamp<=.35+1e-9
                  and (self.accepted is None or stamp>self.accepted) and (self.fault is None or stamp>self.fault)):
                self.accepted=stamp;self.fault=None
        scene=self.accepted;fresh=(scene is not None and now-scene<=.35+1e-9 and (self.consumed is None or scene>self.consumed))
        ids={t['id'] for t in tracks}
        if len(ids)!=len(tracks) or len(ids)>128:raise ValueError('unbounded or duplicate cold identities')
        self.states={i:e for i,e in self.states.items() if i in ids}
        if failed:
            self.states={};self.consumed=max(now,self.consumed or 0.)
        elif fresh:self.consumed=scene
        measurements=out.get('track_measurements')
        if not isinstance(measurements,list):raise ValueError('raw cold measurements missing')
        for track in tracks:
            ident=track['id'];state=self.states.setdefault(ident,dict(samples=[],report=blank(ident)));report=state['report']
            last=report['last_observed_acquisition_s']
            if last is not None and now-last>.35+1e-9:
                state=dict(samples=[],report=blank(ident));self.states[ident]=state;report=state['report']
            sample=next((m for m in measurements if m['id']==ident),None)
            if not fresh or failed or sample is None:continue
            if sample['stamp']!=scene or sample['stamp']!=track['last_seen']:raise ValueError('cold acquisition ownership changed')
            last=report['last_observed_acquisition_s']
            if last is not None and scene-last>.35+1e-9:
                state=dict(samples=[],report=blank(ident));self.states[ident]=state;report=state['report']
            report['last_observed_acquisition_s']=scene
            if report['confirmed']:continue
            history=state['samples'];history.append(copy.deepcopy(sample))
            while len(history)>3 and scene-history[1]['stamp']>=.2-1e-9:history.pop(0)
            while len(history)>32 or (history and scene-history[0]['stamp']>.6+1e-9):history.pop(0)
            report.update(first_acquisition_s=history[0]['stamp'] if history else None,last_acquisition_s=history[-1]['stamp'] if history else None,
                acquisition_count=len(history),velocity_residual_m_s=None)
            radii=[m['radius'] for m in history]
            fit=fitted(history) if radii and max(radii)-min(radii)<=.12+1e-9 else None
            if fit is not None:
                velocity=fit
                if math.hypot(*velocity)>self.maximum+1e-9:raise ValueError('measured velocity exceeds cold prior')
                residual=math.dist(velocity,[track['velocity'][k] for k in ['x','y']]);report['velocity_residual_m_s']=residual
                if residual<=self.bounds['velocity_error_m_s']+1e-9:
                    report.update(confirmed=True,confirmation_acquisition_s=scene);state['samples']=[];self.confirmations+=1
        expected=dict(maximum_speed_m_s=self.maximum,tracks=[self.states[i]['report'] for i in sorted(self.states)])
        if not equal(out.get('cold_motion'),expected):raise ValueError('cold support clocks, identities or confirmation differ')
        self.ticks+=1

    def floor(self,track,forecast,now):
        if self.states[track['id']]['report']['confirmed']:return None
        self.expanded+=1;age=now-track['last_seen'];origin=[track['position'][k] for k in ['x','y']]
        radii=[math.dist(origin,[point[k] for k in ['x','y']])+self.bounds['position_error_m']
               +self.maximum*(age+i*forecast['dt']) for i,point in enumerate(forecast['positions'])]
        self.maximum_radius=max(self.maximum_radius,max(radii,default=0.));return radii


def reconstruct(config,ticks):
    observer=ColdMotionOracle(config)
    for tick in ticks:
        tracks,predictions=continuous_observations(tick['expected'],config.get('forecast_pose_sigma'))
        observer.observe(tick,tracks);by_id={t['id']:t for t in tracks}
        for p in predictions:observer.floor(by_id[p['id']],p,tick['input']['time'])
    return dict(passed=True,verified_ticks=observer.ticks,expanded_forecasts=observer.expanded,
        confirmations=observer.confirmations,maximum_reachable_extra_radius_m=observer.maximum_radius,
        maximum_speed_m_s=observer.maximum,first_acquisitions_scored=True,
        nominal_centers_preserved=True,actor_truth_used=False,maximum_speed_calibrated=False)
